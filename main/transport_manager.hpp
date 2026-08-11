#pragma once

#include "esp_eth.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_modem_api.h"

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
    void ensureProvisioningServiceName();
    void setDefaultNetifForActiveTransport();
    static void wifiEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static void wifiGotIpEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static void wifiProvEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static void ethEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static void gotIpEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

    TransportType active_transport_ = TransportType::kNone;
    bool wifi_enabled_ = true;
    bool ethernet_enabled_ = true;
    bool wifi_initialized_ = false;
    bool wifi_connected_ = false;
    bool wifi_provisioning_started_ = false;
    bool wifi_provisioned_ = false;
    bool wifi_prov_mgr_initialized_ = false;
    char wifi_prov_service_name_[32] = {0};
    bool ethernet_initialized_ = false;
    bool ethernet_link_up_ = false;
    bool ethernet_got_ip_ = false;
    TransportType gprs_gate_reason_ = TransportType::kNone;
    bool gprs_initialized_ = false;
    bool gprs_ready_ = false;
    bool gprs_attempted_ = false;
    bool gprs_boot_soft_reset_done_ = false;
    esp_netif_t *gprs_netif_ = nullptr;
    esp_modem_dce_t *gprs_dce_ = nullptr;
    esp_netif_t *wifi_netif_ = nullptr;
    esp_eth_handle_t ethernet_handle_ = nullptr;
    esp_netif_t *ethernet_netif_ = nullptr;
    esp_eth_netif_glue_handle_t ethernet_glue_ = nullptr;
};
