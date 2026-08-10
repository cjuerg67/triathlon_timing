#include "transport_manager.hpp"

#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "app_config.hpp"
#include "esp_modem_api.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

namespace {
static const char *TAG = "transport_manager";
static bool s_netif_initialized = false;
static bool s_event_loop_initialized = false;
static bool s_eth_handlers_registered = false;
static bool s_wifi_handlers_registered = false;
static bool s_ppp_handlers_registered = false;

bool isModemRegistered(int reg_state) {
    return reg_state == 1 || reg_state == 5;
}

int parseRegistrationStatus(const char *response, const char *prefix) {
    if (response == nullptr || prefix == nullptr) {
        return -1;
    }

    const char *line = std::strstr(response, prefix);
    if (line == nullptr) {
        return -1;
    }

    const char *colon = std::strchr(line, ':');
    if (colon == nullptr) {
        return -1;
    }

    const char *status_ptr = std::strrchr(colon, ',');
    if (status_ptr != nullptr) {
        ++status_ptr;
    } else {
        status_ptr = colon + 1;
    }

    while (*status_ptr == ' ' || *status_ptr == '\t') {
        ++status_ptr;
    }

    return std::atoi(status_ptr);
}

bool queryRegisteredByAt(esp_modem_dce_t *dce) {
    if (dce == nullptr) {
        return false;
    }

    const char *cmds[] = { "AT+CREG?", "AT+CGREG?", "AT+CEREG?" };
    const char *prefixes[] = { "+CREG", "+CGREG", "+CEREG" };

    for (int i = 0; i < 3; ++i) {
        char out[ESP_MODEM_C_API_STR_BUF_SIZE] = {0};
        const esp_err_t ret = esp_modem_at(dce, cmds[i], out, 2500);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "%s failed: %s", cmds[i], esp_err_to_name(ret));
            continue;
        }

        const int stat = parseRegistrationStatus(out, prefixes[i]);
        ESP_LOGI(TAG, "%s status=%d raw=%s", prefixes[i], stat, out);
        if (isModemRegistered(stat)) {
            return true;
        }
    }

    return false;
}

void runAtBestEffort(esp_modem_dce_t *dce, const char *cmd, int timeout_ms) {
    if (dce == nullptr || cmd == nullptr) {
        return;
    }
    char out[ESP_MODEM_C_API_STR_BUF_SIZE] = {0};
    const esp_err_t ret = esp_modem_at(dce, cmd, out, timeout_ms);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "AT %s -> %s", cmd, out[0] != '\0' ? out : "OK");
    } else {
        ESP_LOGW(TAG, "AT %s failed: %s", cmd, esp_err_to_name(ret));
    }
}

void logSignalQuality(esp_modem_dce_t *dce) {
    int rssi = 0;
    int ber = 0;
    const esp_err_t ret = esp_modem_get_signal_quality(dce, &rssi, &ber);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SIM7000 signal rssi=%d ber=%d", rssi, ber);
    } else {
        ESP_LOGW(TAG, "SIM7000 signal read failed: %s", esp_err_to_name(ret));
    }
}
}  // namespace

bool TransportManager::connectAny() {
    if (refreshActiveTransport()) {
        return true;
    }

    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(app_config::kWifiConnectTimeoutMs);
    while ((xTaskGetTickCount() - start) < timeout) {
        vTaskDelay(pdMS_TO_TICKS(250));
        if (refreshActiveTransport()) {
            return true;
        }
    }

    active_transport_ = TransportType::kNone;
    ESP_LOGW(TAG, "No network transport is currently available; continuing without MQTT publishing");
    return false;
}

bool TransportManager::refreshActiveTransport() {
    (void)ensureWifiConnection();
    (void)ensureEthernetConnection();

    const TransportType previous = active_transport_;
    if (wifi_connected_) {
        if (gprs_gate_reason_ != TransportType::kWifi) {
            ESP_LOGI(TAG, "GPRS not attempted: Wi-Fi is available");
            gprs_gate_reason_ = TransportType::kWifi;
        }
        active_transport_ = TransportType::kWifi;
    } else if (ethernet_link_up_ && ethernet_got_ip_) {
        if (gprs_gate_reason_ != TransportType::kEthernet) {
            ESP_LOGI(TAG, "GPRS not attempted: Ethernet is available");
            gprs_gate_reason_ = TransportType::kEthernet;
        }
        active_transport_ = TransportType::kEthernet;
    } else {
        if (gprs_gate_reason_ != TransportType::kNone) {
            ESP_LOGI(TAG, "No Wi-Fi/LAN available; attempting GPRS now");
            gprs_gate_reason_ = TransportType::kNone;
        }

        if (ensureGprsConnection()) {
            active_transport_ = TransportType::kGprs;
        } else {
            active_transport_ = TransportType::kNone;
        }
    }

    if (active_transport_ != previous) {
        ESP_LOGW(TAG,
                 "Active transport changed %d -> %d",
                 static_cast<int>(previous),
                 static_cast<int>(active_transport_));
        setDefaultNetifForActiveTransport();
    } else if (active_transport_ != TransportType::kNone) {
        // Keep the default route aligned with the selected transport.
        setDefaultNetifForActiveTransport();
    } else if (app_config::kEnableVerboseTransportLogs) {
        ESP_LOGI(TAG,
                 "Transport refresh stable=%d wifi=%d eth_link=%d eth_ip=%d gprs=%d",
                 static_cast<int>(active_transport_),
                 wifi_connected_,
                 ethernet_link_up_,
                 ethernet_got_ip_,
                 gprs_ready_);
    }

    return active_transport_ != TransportType::kNone;
}

TransportType TransportManager::activeTransport() const {
    return active_transport_;
}

bool TransportManager::ensureWifiConnection() {
    if (!wifi_enabled_) {
        ESP_LOGI(TAG, "Wi-Fi transport disabled; skipping initialization");
        return false;
    }

    if (wifi_initialized_) {
        return true;
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(ret));
        return false;
    }

    if (!s_netif_initialized) {
        ret = esp_netif_init();
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(ret));
            return false;
        }
        s_netif_initialized = true;
    }

    if (!s_event_loop_initialized) {
        ret = esp_event_loop_create_default();
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(ret));
            return false;
        }
        s_event_loop_initialized = true;
    }

    if (wifi_netif_ == nullptr) {
        wifi_netif_ = esp_netif_create_default_wifi_sta();
    }
    if (wifi_netif_ == nullptr) {
        ESP_LOGE(TAG, "esp_netif_create_default_wifi_sta failed");
        return false;
    }

    if (!s_wifi_handlers_registered) {
        ret = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &TransportManager::wifiEventHandler, this);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Register Wi-Fi event handler failed: %s", esp_err_to_name(ret));
            return false;
        }

        ret = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &TransportManager::wifiGotIpEventHandler, this);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Register Wi-Fi IP event handler failed: %s", esp_err_to_name(ret));
            return false;
        }
        s_wifi_handlers_registered = true;
    }

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&init_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %s", esp_err_to_name(ret));
        return false;
    }

    wifi_config_t wifi_config = {};
    std::strncpy(reinterpret_cast<char *>(wifi_config.sta.ssid), app_config::kWifiSsid, sizeof(wifi_config.sta.ssid) - 1);
    std::strncpy(reinterpret_cast<char *>(wifi_config.sta.password), app_config::kWifiPassword, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_wifi_connect();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_INIT) {
        ESP_LOGW(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(ret));
        return false;
    }

    wifi_initialized_ = true;
    ESP_LOGI(TAG, "Wi-Fi transport initialization started for SSID %s", app_config::kWifiSsid);
    return true;
}

bool TransportManager::ensureEthernetConnection() {
    if (!ethernet_enabled_) {
        ESP_LOGI(TAG, "Ethernet transport disabled; skipping initialization");
        return false;
    }

    if (ethernet_initialized_) {
        return true;
    }

    esp_err_t ret = ESP_OK;
    if (!s_netif_initialized) {
        ret = esp_netif_init();
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(ret));
            return false;
        }
        s_netif_initialized = true;
    }

    if (!s_event_loop_initialized) {
        ret = esp_event_loop_create_default();
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(ret));
            return false;
        }
        s_event_loop_initialized = true;
    }

    if (!s_eth_handlers_registered) {
        ret = esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &TransportManager::ethEventHandler, this);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Register ETH event handler failed: %s", esp_err_to_name(ret));
            return false;
        }

        ret = esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &TransportManager::gotIpEventHandler, this);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Register IP event handler failed: %s", esp_err_to_name(ret));
            return false;
        }
        s_eth_handlers_registered = true;
    }

    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
    ethernet_netif_ = esp_netif_new(&netif_config);
    if (ethernet_netif_ == nullptr) {
        ESP_LOGE(TAG, "esp_netif_new failed for Ethernet");
        return false;
    }

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
    if (mac == nullptr) {
        ESP_LOGE(TAG, "esp_eth_mac_new_esp32 failed");
        esp_netif_destroy(ethernet_netif_);
        ethernet_netif_ = nullptr;
        return false;
    }

    esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_config);
    if (phy == nullptr) {
        ESP_LOGE(TAG, "esp_eth_phy_new_generic failed");
        mac->del(mac);
        esp_netif_destroy(ethernet_netif_);
        ethernet_netif_ = nullptr;
        return false;
    }

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    ret = esp_eth_driver_install(&eth_config, &ethernet_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_driver_install failed: %s", esp_err_to_name(ret));
        phy->del(phy);
        mac->del(mac);
        esp_netif_destroy(ethernet_netif_);
        ethernet_netif_ = nullptr;
        return false;
    }

    ethernet_glue_ = esp_eth_new_netif_glue(ethernet_handle_);
    if (ethernet_glue_ == nullptr) {
        ESP_LOGE(TAG, "esp_eth_new_netif_glue failed");
        esp_eth_driver_uninstall(ethernet_handle_);
        ethernet_handle_ = nullptr;
        phy->del(phy);
        mac->del(mac);
        esp_netif_destroy(ethernet_netif_);
        ethernet_netif_ = nullptr;
        return false;
    }

    ret = esp_netif_attach(ethernet_netif_, ethernet_glue_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_attach failed: %s", esp_err_to_name(ret));
        esp_eth_del_netif_glue(ethernet_glue_);
        esp_eth_driver_uninstall(ethernet_handle_);
        ethernet_glue_ = nullptr;
        ethernet_handle_ = nullptr;
        phy->del(phy);
        mac->del(mac);
        esp_netif_destroy(ethernet_netif_);
        ethernet_netif_ = nullptr;
        return false;
    }

    ret = esp_eth_start(ethernet_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_start failed: %s", esp_err_to_name(ret));
        esp_eth_del_netif_glue(ethernet_glue_);
        esp_eth_driver_uninstall(ethernet_handle_);
        ethernet_glue_ = nullptr;
        ethernet_handle_ = nullptr;
        phy->del(phy);
        mac->del(mac);
        esp_netif_destroy(ethernet_netif_);
        ethernet_netif_ = nullptr;
        return false;
    }

    ethernet_initialized_ = true;
    ESP_LOGI(TAG, "Ethernet transport initialization started; waiting for link/IP");
    return true;
}

bool TransportManager::ensureGprsConnection() {
    if (!app_config::kEnableGprsTransport) {
        ESP_LOGI(TAG, "GPRS transport disabled; skipping initialization");
        return false;
    }

    if (gprs_ready_) {
        return true;
    }

    if (gprs_initialized_ && !gprs_ready_) {
        ESP_LOGW(TAG, "Retrying SIM7000 PPP bring-up after previous failure");
    }

    auto fail_gprs = [this](const char *message, esp_err_t err = ESP_FAIL) {
        if (message != nullptr) {
            if (err == ESP_FAIL) {
                ESP_LOGE(TAG, "%s", message);
            } else {
                ESP_LOGE(TAG, "%s: %s", message, esp_err_to_name(err));
            }
        }
        if (gprs_dce_ != nullptr) {
            esp_modem_destroy(gprs_dce_);
            gprs_dce_ = nullptr;
        }
        gprs_ready_ = false;
        gprs_initialized_ = false;
        return false;
    };

    esp_err_t ret = ESP_OK;
    if (!s_netif_initialized) {
        ret = esp_netif_init();
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(ret));
            gprs_initialized_ = true;
            gprs_ready_ = false;
            return false;
        }
        s_netif_initialized = true;
    }

    if (!s_event_loop_initialized) {
        ret = esp_event_loop_create_default();
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(ret));
            gprs_initialized_ = true;
            gprs_ready_ = false;
            return false;
        }
        s_event_loop_initialized = true;
    }

    if (gprs_netif_ == nullptr) {
        esp_netif_config_t ppp_netif_cfg = ESP_NETIF_DEFAULT_PPP();
        gprs_netif_ = esp_netif_new(&ppp_netif_cfg);
    }
    if (gprs_netif_ == nullptr) {
        ESP_LOGE(TAG, "esp_netif_new failed for PPP");
        gprs_initialized_ = true;
        gprs_ready_ = false;
        return false;
    }

    esp_netif_ppp_config_t ppp_config = { true, true };
    esp_netif_ppp_set_params(gprs_netif_, &ppp_config);

    if (!s_ppp_handlers_registered) {
        ret = esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_GOT_IP, &TransportManager::gotIpEventHandler, this);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Register PPP got IP event handler failed: %s", esp_err_to_name(ret));
            gprs_initialized_ = true;
            gprs_ready_ = false;
            return false;
        }

        ret = esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_LOST_IP, &TransportManager::gotIpEventHandler, this);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Register PPP lost IP event handler failed: %s", esp_err_to_name(ret));
            gprs_initialized_ = true;
            gprs_ready_ = false;
            return false;
        }
        s_ppp_handlers_registered = true;
    }

    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    dte_config.uart_config.port_num = static_cast<uart_port_t>(app_config::kSim7000UartPort);
    dte_config.uart_config.tx_io_num = app_config::kSim7000UartTxPin;
    dte_config.uart_config.rx_io_num = app_config::kSim7000UartRxPin;
    dte_config.uart_config.rts_io_num = UART_PIN_NO_CHANGE;
    dte_config.uart_config.cts_io_num = UART_PIN_NO_CHANGE;
    dte_config.uart_config.flow_control = ESP_MODEM_FLOW_CONTROL_NONE;
    dte_config.uart_config.rx_buffer_size = 2048;
    dte_config.uart_config.tx_buffer_size = 2048;
    dte_config.uart_config.event_queue_size = 16;
    dte_config.task_stack_size = 4096;
    dte_config.task_priority = 5;

    ESP_LOGI(TAG,
             "SIM7000 UART config port=%d tx=%d rx=%d",
             static_cast<int>(dte_config.uart_config.port_num),
             static_cast<int>(dte_config.uart_config.tx_io_num),
             static_cast<int>(dte_config.uart_config.rx_io_num));

    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(app_config::kSim7000Apn);
    esp_err_t modem_ret = ESP_FAIL;
    const int baud_candidates[] = {
        app_config::kSim7000UartBaudRate,
        115200,
        9600,
        57600,
        38400,
        19200,
    };

    bool modem_synced = false;
    for (int baud_index = 0; baud_index < static_cast<int>(sizeof(baud_candidates) / sizeof(baud_candidates[0])); ++baud_index) {
        dte_config.uart_config.baud_rate = baud_candidates[baud_index];

        if (gprs_dce_ != nullptr) {
            esp_modem_destroy(gprs_dce_);
            gprs_dce_ = nullptr;
        }

        gprs_dce_ = esp_modem_new_dev(ESP_MODEM_DCE_SIM7000, &dte_config, &dce_config, gprs_netif_);
        if (gprs_dce_ == nullptr) {
            ESP_LOGW(TAG, "esp_modem_new_dev failed for SIM7000 at %d baud", baud_candidates[baud_index]);
            continue;
        }

        ESP_LOGI(TAG, "Trying SIM7000 sync at %d baud", baud_candidates[baud_index]);
        for (int attempt = 1; attempt <= 5; ++attempt) {
            modem_ret = esp_modem_sync(gprs_dce_);
            if (modem_ret == ESP_OK) {
                modem_synced = true;
                ESP_LOGI(TAG, "SIM7000 sync successful at %d baud", baud_candidates[baud_index]);
                break;
            }
            ESP_LOGW(TAG,
                     "esp_modem_sync attempt %d/5 failed at %d baud: %s",
                     attempt,
                     baud_candidates[baud_index],
                     esp_err_to_name(modem_ret));
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        if (modem_synced) {
            break;
        }
    }

    if (!modem_synced) {
        return fail_gprs("Modem sync failed at all baud candidates; cannot start PPP", modem_ret);
    }

    (void)esp_modem_set_mode(gprs_dce_, ESP_MODEM_MODE_COMMAND);
    (void)esp_modem_set_echo(gprs_dce_, false);
    (void)esp_modem_set_apn(gprs_dce_, app_config::kSim7000Apn);
    ESP_LOGI(TAG, "Applying SIM7000 radio and attach settings");
    (void)esp_modem_set_radio_state(gprs_dce_, 1);
    (void)esp_modem_set_network_attachment_state(gprs_dce_, 1);
    logSignalQuality(gprs_dce_);

    esp_modem_sim_pin_state_t pin_state = ESP_MODEM_SIM_PIN_STATE_UNKNOWN;
    modem_ret = esp_modem_read_pin_state(gprs_dce_, &pin_state);
    if (modem_ret != ESP_OK) {
        ESP_LOGW(TAG, "CPIN read failed: %s", esp_err_to_name(modem_ret));
    } else if (pin_state != ESP_MODEM_SIM_PIN_STATE_READY) {
        ESP_LOGE(TAG, "SIM not ready (CPIN state=%d)", static_cast<int>(pin_state));
        return fail_gprs(nullptr);
    }

    int reg_state = 0;
    bool registered = false;
    ESP_LOGI(TAG, "Waiting for SIM7000 network registration");
    for (int attempt = 1; attempt <= 90; ++attempt) {
        modem_ret = esp_modem_get_network_registration_state(gprs_dce_, &reg_state);
        if (modem_ret == ESP_OK && isModemRegistered(reg_state)) {
            registered = true;
            break;
        }

        if (attempt % 5 == 0 && queryRegisteredByAt(gprs_dce_)) {
            ESP_LOGI(TAG, "AT fallback confirms SIM7000 is registered");
            registered = true;
            break;
        }

        // If the modem is not searching, nudge it back to full radio and attached state.
        if (modem_ret == ESP_OK && reg_state == 0 && (attempt == 10 || attempt == 30 || attempt == 60)) {
            ESP_LOGW(TAG, "Registration state=0; forcing radio full mode and attached state");
            (void)esp_modem_set_radio_state(gprs_dce_, 1);
            (void)esp_modem_set_network_attachment_state(gprs_dce_, 1);
            logSignalQuality(gprs_dce_);
        }

        if (modem_ret == ESP_OK && reg_state == 0 && attempt == 20) {
            ESP_LOGW(TAG, "Registration state=0 persists; toggling attach state");
            (void)esp_modem_set_network_attachment_state(gprs_dce_, 0);
            vTaskDelay(pdMS_TO_TICKS(1200));
            (void)esp_modem_set_network_attachment_state(gprs_dce_, 1);
            runAtBestEffort(gprs_dce_, "AT+CGATT?", 2000);
        }

        if (modem_ret == ESP_OK && reg_state == 0 && attempt == 40) {
            ESP_LOGW(TAG, "Registration state=0 persists; forcing automatic operator search");
            runAtBestEffort(gprs_dce_, "AT+COPS=0", 3000);
            runAtBestEffort(gprs_dce_, "AT+COPS?", 3000);
        }

        if (modem_ret == ESP_OK && reg_state == 0 && attempt == 70) {
            ESP_LOGW(TAG, "Registration state=0 persists; cycling modem radio");
            (void)esp_modem_set_radio_state(gprs_dce_, 0);
            vTaskDelay(pdMS_TO_TICKS(2000));
            (void)esp_modem_set_radio_state(gprs_dce_, 1);
            runAtBestEffort(gprs_dce_, "AT+CREG?", 2000);
            runAtBestEffort(gprs_dce_, "AT+CEREG?", 2000);
            logSignalQuality(gprs_dce_);
        }

        if (modem_ret == ESP_OK && reg_state == 0 && attempt == 80) {
            ESP_LOGW(TAG, "Registration state=0 persists; resetting SIM7000");
            const esp_err_t reset_ret = esp_modem_reset(gprs_dce_);
            ESP_LOGW(TAG, "esp_modem_reset result: %s", esp_err_to_name(reset_ret));
            vTaskDelay(pdMS_TO_TICKS(5000));
            (void)esp_modem_sync(gprs_dce_);
            (void)esp_modem_set_mode(gprs_dce_, ESP_MODEM_MODE_COMMAND);
            (void)esp_modem_set_echo(gprs_dce_, false);
            (void)esp_modem_set_apn(gprs_dce_, app_config::kSim7000Apn);
            (void)esp_modem_set_radio_state(gprs_dce_, 1);
            (void)esp_modem_set_network_attachment_state(gprs_dce_, 1);
            runAtBestEffort(gprs_dce_, "AT+CPIN?", 2000);
        }

        if (attempt % 5 == 0) {
            ESP_LOGW(TAG,
                     "Network registration pending (attempt %d/90, state=%d, ret=%s)",
                     attempt,
                     reg_state,
                     esp_err_to_name(modem_ret));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    if (!registered) {
        return fail_gprs("SIM7000 not registered to network; aborting PPP start");
    }

    int attach_state = 0;
    modem_ret = esp_modem_get_network_attachment_state(gprs_dce_, &attach_state);
    if (modem_ret != ESP_OK || attach_state != 1) {
        modem_ret = esp_modem_set_network_attachment_state(gprs_dce_, 1);
        if (modem_ret != ESP_OK) {
            return fail_gprs("Failed to request GPRS attach", modem_ret);
        }
        for (int attempt = 1; attempt <= 15; ++attempt) {
            modem_ret = esp_modem_get_network_attachment_state(gprs_dce_, &attach_state);
            if (modem_ret == ESP_OK && attach_state == 1) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        if (attach_state != 1) {
            return fail_gprs("GPRS attachment did not complete");
        }
    }

    ret = ESP_FAIL;
    for (int attempt = 1; attempt <= 3; ++attempt) {
        ret = esp_modem_set_mode(gprs_dce_, ESP_MODEM_MODE_DATA);
        if (ret == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "esp_modem_set_mode(DATA) attempt %d/3 failed: %s", attempt, esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (ret != ESP_OK) {
        return fail_gprs("esp_modem_set_mode(DATA) failed", ret);
    }

    for (int attempt = 0; attempt < 120 && !gprs_ready_; ++attempt) {
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    if (!gprs_ready_) {
        return fail_gprs("PPP did not obtain IP in time");
    }

    gprs_initialized_ = true;
    gprs_ready_ = true;
    return true;
}

void TransportManager::setDefaultNetifForActiveTransport() {
    if (active_transport_ == TransportType::kGprs && gprs_netif_ != nullptr && gprs_ready_) {
        esp_netif_set_default_netif(gprs_netif_);
        return;
    }

    if (active_transport_ == TransportType::kEthernet && ethernet_netif_ != nullptr && ethernet_link_up_ && ethernet_got_ip_) {
        esp_netif_set_default_netif(ethernet_netif_);
        return;
    }

    if (active_transport_ == TransportType::kWifi && wifi_netif_ != nullptr && wifi_connected_) {
        esp_netif_set_default_netif(wifi_netif_);
    }
}

void TransportManager::wifiEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    (void)event_base;
    (void)event_data;

    auto *self = static_cast<TransportManager *>(arg);
    if (self == nullptr) {
        return;
    }

    switch (event_id) {
    case WIFI_EVENT_STA_START:
        ESP_LOGI(TAG, "Wi-Fi station started");
        esp_wifi_connect();
        break;
    case WIFI_EVENT_STA_CONNECTED:
        self->wifi_connected_ = true;
        ESP_LOGI(TAG, "Wi-Fi station connected");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        self->wifi_connected_ = false;
        ESP_LOGW(TAG, "Wi-Fi station disconnected; reconnecting");
        esp_wifi_connect();
        break;
    default:
        break;
    }
}

void TransportManager::wifiGotIpEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    (void)event_base;
    (void)event_id;

    auto *self = static_cast<TransportManager *>(arg);
    if (self == nullptr) {
        return;
    }

    if (event_data == nullptr) {
        return;
    }

    self->wifi_connected_ = true;
    if (self->wifi_netif_ != nullptr) {
        esp_netif_set_default_netif(self->wifi_netif_);
    }
    ESP_LOGI(TAG, "Wi-Fi station received an IP address");
}

void TransportManager::ethEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    (void)event_base;
    (void)event_data;

    auto *self = static_cast<TransportManager *>(arg);
    if (self == nullptr) {
        return;
    }

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        self->ethernet_link_up_ = true;
        ESP_LOGI(TAG, "Ethernet link connected");
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        self->ethernet_link_up_ = false;
        self->ethernet_got_ip_ = false;
        ESP_LOGW(TAG, "Ethernet link disconnected");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet driver started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet driver stopped");
        break;
    default:
        break;
    }
}

void TransportManager::gotIpEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    (void)event_base;

    auto *self = static_cast<TransportManager *>(arg);
    if (self == nullptr) {
        return;
    }

    if (event_data == nullptr) {
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_PPP_GOT_IP) {
        self->gprs_ready_ = true;
        self->setDefaultNetifForActiveTransport();
        ESP_LOGI(TAG, "SIM7000 PPP got IP");
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_PPP_LOST_IP) {
        self->gprs_ready_ = false;
        ESP_LOGW(TAG, "SIM7000 PPP lost IP");
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_ETH_GOT_IP) {
        self->ethernet_got_ip_ = true;
        self->setDefaultNetifForActiveTransport();
        ESP_LOGI(TAG, "Ethernet interface received an IP address");
    }
}
