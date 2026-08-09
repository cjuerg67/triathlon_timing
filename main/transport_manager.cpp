#include "transport_manager.hpp"

#include <cstdio>
#include <cstring>

#include "app_config.hpp"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
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

bool readUartLine(uart_port_t port, char *line, size_t line_size, uint32_t timeout_ms) {
    if (line == nullptr || line_size < 2) {
        return false;
    }

    line[0] = '\0';
    size_t used = 0;
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (xTaskGetTickCount() < deadline) {
        uint8_t ch = 0;
        const int read = uart_read_bytes(port, &ch, 1, pdMS_TO_TICKS(20));
        if (read <= 0) {
            continue;
        }

        if (ch == '\r') {
            continue;
        }

        if (ch == '\n') {
            if (used == 0) {
                continue;
            }
            line[used] = '\0';
            return true;
        }

        if (used < (line_size - 1)) {
            line[used++] = static_cast<char>(ch);
        }
    }

    if (used > 0) {
        line[used] = '\0';
        return true;
    }
    return false;
}

bool sim7000Command(uart_port_t port,
                    const char *cmd,
                    uint32_t timeout_ms,
                    char *first_data_line,
                    size_t first_data_line_size) {
    if (cmd == nullptr) {
        return false;
    }

    if (first_data_line != nullptr && first_data_line_size > 0) {
        first_data_line[0] = '\0';
    }

    uart_flush(port);
    (void)uart_write_bytes(port, cmd, std::strlen(cmd));
    (void)uart_write_bytes(port, "\r\n", 2);

    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (xTaskGetTickCount() < deadline) {
        char line[160] = {0};
        if (!readUartLine(port, line, sizeof(line), 200)) {
            continue;
        }

        if (std::strcmp(line, "OK") == 0) {
            return true;
        }
        if (std::strcmp(line, "ERROR") == 0 || std::strstr(line, "+CME ERROR") != nullptr) {
            ESP_LOGW(TAG, "SIM7000 cmd failed: %s -> %s", cmd, line);
            return false;
        }

        if (first_data_line != nullptr && first_data_line_size > 0 && first_data_line[0] == '\0') {
            std::snprintf(first_data_line, first_data_line_size, "%s", line);
        }
    }

    ESP_LOGW(TAG, "SIM7000 cmd timeout: %s", cmd);
    return false;
}
}  // namespace

bool TransportManager::connectAny() {
    // Try immediately using priority wifi > ethernet > gprs.
    if (refreshActiveTransport()) {
        return true;
    }

    // Give Wi-Fi/LAN a short chance to come up; keep reevaluating full priority.
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
    // Keep interfaces initialized and choose best currently available transport.
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
        ESP_LOGW(TAG, "Active transport changed %d -> %d",
                 static_cast<int>(previous), static_cast<int>(active_transport_));
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

    if (gprs_initialized_) {
        if (!gprs_ready_) {
            // Allow retries on later refresh cycles.
            ESP_LOGW(TAG, "GPRS not ready yet; scheduling a re-attach attempt");
            gprs_initialized_ = false;
            return false;
        }

        // Lightweight health probe for active sessions.
        const uart_port_t health_port = static_cast<uart_port_t>(app_config::kSim7000UartPort);
        char health_line[96] = {0};
        if (!sim7000Command(health_port, "AT+CGATT?", 2500, health_line, sizeof(health_line)) ||
            std::strstr(health_line, ": 1") == nullptr) {
            ESP_LOGW(TAG, "SIM7000 detached from packet domain");
            gprs_ready_ = false;
            gprs_initialized_ = false;
            return false;
        }
        return true;
    }

    ESP_LOGI(TAG, "SIM7000 GPRS attach attempt started");

    const uart_port_t port = static_cast<uart_port_t>(app_config::kSim7000UartPort);
    if (!uart_is_driver_installed(port)) {
        const esp_err_t install_ret = uart_driver_install(port, 4096, 512, 0, nullptr, 0);
        if (install_ret != ESP_OK) {
            ESP_LOGE(TAG, "SIM7000 UART driver install failed: %s", esp_err_to_name(install_ret));
            gprs_initialized_ = true;
            gprs_ready_ = false;
            return false;
        }
    }

    const uart_config_t uart_cfg = {
        .baud_rate = app_config::kSim7000UartBaudRate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {.allow_pd = 0, .backup_before_sleep = 0},
    };

    if (uart_param_config(port, &uart_cfg) != ESP_OK ||
        uart_set_pin(port,
                     app_config::kSim7000UartTxPin,
                     app_config::kSim7000UartRxPin,
                     UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "SIM7000 UART config failed");
        gprs_initialized_ = true;
        gprs_ready_ = false;
        return false;
    }

    char line[160] = {0};
    if (!sim7000Command(port, "AT", 2000, nullptr, 0) ||
        !sim7000Command(port, "ATE0", 2000, nullptr, 0)) {
        ESP_LOGW(TAG, "SIM7000 not responding to AT");
        gprs_initialized_ = true;
        gprs_ready_ = false;
        return false;
    }

    if (!sim7000Command(port, "AT+CPIN?", 4000, line, sizeof(line)) ||
        std::strstr(line, "READY") == nullptr) {
        ESP_LOGW(TAG, "SIM7000 SIM not ready: %s", line);
        gprs_initialized_ = true;
        gprs_ready_ = false;
        return false;
    }

    (void)sim7000Command(port, "AT+CSQ", 2000, line, sizeof(line));

    char pdp_cmd[196] = {0};
    std::snprintf(pdp_cmd, sizeof(pdp_cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", app_config::kSim7000Apn);
    if (!sim7000Command(port, "AT+CGATT=1", 20000, nullptr, 0) ||
        !sim7000Command(port, pdp_cmd, 5000, nullptr, 0) ||
        !sim7000Command(port, "AT+CGACT=1,1", 30000, nullptr, 0) ||
        !sim7000Command(port, "AT+CGPADDR=1", 5000, line, sizeof(line))) {
        ESP_LOGW(TAG, "SIM7000 PDP activation failed");
        gprs_initialized_ = true;
        gprs_ready_ = false;
        return false;
    }

    if (std::strstr(line, "0.0.0.0") != nullptr) {
        ESP_LOGW(TAG, "SIM7000 PDP has no valid IP: %s", line);
        gprs_initialized_ = true;
        gprs_ready_ = false;
        return false;
    }

    ESP_LOGI(TAG, "SIM7000 GPRS attached: %s", line);
    gprs_initialized_ = true;
    gprs_ready_ = true;
    return true;
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
    (void)event_id;

    auto *self = static_cast<TransportManager *>(arg);
    if (self == nullptr) {
        return;
    }

    if (event_data == nullptr) {
        return;
    }

    self->ethernet_got_ip_ = true;
    if (self->ethernet_netif_ != nullptr) {
        esp_netif_set_default_netif(self->ethernet_netif_);
    }
    ESP_LOGI(TAG, "Ethernet interface received an IP address");
}
