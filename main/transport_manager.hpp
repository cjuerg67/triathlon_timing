#pragma once

#include "esp_eth.h"
#include "esp_netif.h"
#include "esp_wifi.h"

enum class TransportType {
    kNone,
    kWifi,
    kEthernet,
    kGprs,
};

class TransportManager {
public:
    bool connectAny();
    bool refreshActiveTransport();
    TransportType activeTransport() const;

private:
    bool ensureWifiConnection();
    bool ensureEthernetConnection();
    bool ensureGprsConnection();
    static void wifiEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static void wifiGotIpEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static void ethEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static void gotIpEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

    TransportType active_transport_ = TransportType::kNone;
    bool wifi_enabled_ = false;
    bool ethernet_enabled_ = true;
    bool wifi_initialized_ = false;
    bool wifi_connected_ = false;
    bool ethernet_initialized_ = false;
    bool ethernet_link_up_ = false;
    bool ethernet_got_ip_ = false;
    TransportType gprs_gate_reason_ = TransportType::kNone;
    bool gprs_initialized_ = false;
    bool gprs_ready_ = false;
    esp_netif_t *wifi_netif_ = nullptr;
    esp_eth_handle_t ethernet_handle_ = nullptr;
    esp_netif_t *ethernet_netif_ = nullptr;
    esp_eth_netif_glue_handle_t ethernet_glue_ = nullptr;
};
