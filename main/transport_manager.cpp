#include "transport_manager.hpp"

#include <cstring>

#include "app_config.hpp"
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
}  // namespace

bool TransportManager::connectAny() {
    if (ensureWifiConnection()) {
        for (int attempt = 0; attempt < 60; ++attempt) {
            if (wifi_connected_) {
                active_transport_ = TransportType::kWifi;
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(250));
        }

        ESP_LOGW(TAG, "Wi-Fi connection did not become ready in time");
    }

    if (ensureEthernetConnection()) {
        for (int attempt = 0; attempt < 40; ++attempt) {
            if (ethernet_link_up_ && ethernet_got_ip_) {
                active_transport_ = TransportType::kEthernet;
                return true;
            }

            if (attempt == 0) {
                ESP_LOGI(TAG, "Waiting for Ethernet link and IP address...");
            }
            vTaskDelay(pdMS_TO_TICKS(250));
        }

        ESP_LOGW(TAG, "Ethernet link/IP did not become ready in time after initialization");
    }

    if (ensureGprsConnection()) {
        active_transport_ = TransportType::kGprs;
        return true;
    }

    active_transport_ = TransportType::kNone;
    ESP_LOGW(TAG, "No network transport is currently available; continuing without MQTT publishing");
    return false;
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

    esp_netif_set_default_netif(ethernet_netif_);

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
    if (!gprs_enabled_) {
        ESP_LOGI(TAG, "GPRS transport disabled; skipping initialization");
        return false;
    }

    ESP_LOGI(TAG, "GPRS transport is not configured in the current build; Wi-Fi will be used when available");
    return false;
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
    ESP_LOGI(TAG, "Ethernet interface received an IP address");
}
