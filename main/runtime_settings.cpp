#include "runtime_settings.hpp"

#include <cstdio>
#include <cstring>

#include "app_config.hpp"
#include "freertos/FreeRTOS.h"
#include "lwip/ip4_addr.h"
#include "nvs.h"
#include "esp_log.h"

namespace {
static const char *TAG = "runtime_settings";
static const char *kNamespace = "runtime_cfg";
static const char *kLanKey = "mqtt_lan_uri";
static const char *kGprsKey = "mqtt_gprs_uri";

static char s_lan_broker_uri[96] = {0};
static char s_gprs_broker_uri[96] = {0};
static bool s_initialized = false;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

bool extractIpFromUri(const char *uri, char *ip_out, size_t ip_out_size) {
    if (uri == nullptr || ip_out == nullptr || ip_out_size == 0) {
        return false;
    }

    const char *start = uri;
    if (std::strncmp(uri, "mqtt://", 7) == 0) {
        start = uri + 7;
    }

    const char *end = start;
    while (*end != '\0' && *end != ':' && *end != '/') {
        ++end;
    }

    const size_t len = static_cast<size_t>(end - start);
    if (len == 0 || len >= ip_out_size) {
        return false;
    }

    std::memcpy(ip_out, start, len);
    ip_out[len] = '\0';
    return true;
}

bool isValidIpv4(const char *ip) {
    if (ip == nullptr || ip[0] == '\0') {
        return false;
    }

    ip4_addr_t addr = {};
    return ip4addr_aton(ip, &addr) != 0;
}

void buildUriFromIp(const char *ip, char *uri_out, size_t uri_out_size) {
    if (uri_out == nullptr || uri_out_size == 0) {
        return;
    }
    std::snprintf(uri_out, uri_out_size, "mqtt://%s", (ip != nullptr) ? ip : "");
}

void saveUrisToNvs() {
    nvs_handle_t nvs = 0;
    esp_err_t ret = nvs_open(kNamespace, NVS_READWRITE, &nvs);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed while saving runtime settings: %s", esp_err_to_name(ret));
        return;
    }

    ret = nvs_set_str(nvs, kLanKey, s_lan_broker_uri);
    if (ret == ESP_OK) {
        ret = nvs_set_str(nvs, kGprsKey, s_gprs_broker_uri);
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs);
    }

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed persisting runtime settings: %s", esp_err_to_name(ret));
    }
    nvs_close(nvs);
}

void loadDefaults() {
    std::snprintf(s_lan_broker_uri, sizeof(s_lan_broker_uri), "%s", app_config::kMqttBrokerLan);
    std::snprintf(s_gprs_broker_uri, sizeof(s_gprs_broker_uri), "%s", app_config::kMqttBrokerGprs);
}

}  // namespace

namespace runtime_settings {

esp_err_t init() {
    taskENTER_CRITICAL(&s_lock);
    if (s_initialized) {
        taskEXIT_CRITICAL(&s_lock);
        return ESP_OK;
    }
    taskEXIT_CRITICAL(&s_lock);

    loadDefaults();

    nvs_handle_t nvs = 0;
    esp_err_t ret = nvs_open(kNamespace, NVS_READWRITE, &nvs);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed, using defaults: %s", esp_err_to_name(ret));
    } else {
        size_t len = sizeof(s_lan_broker_uri);
        if (nvs_get_str(nvs, kLanKey, s_lan_broker_uri, &len) != ESP_OK) {
            std::snprintf(s_lan_broker_uri, sizeof(s_lan_broker_uri), "%s", app_config::kMqttBrokerLan);
        }

        len = sizeof(s_gprs_broker_uri);
        if (nvs_get_str(nvs, kGprsKey, s_gprs_broker_uri, &len) != ESP_OK) {
            std::snprintf(s_gprs_broker_uri, sizeof(s_gprs_broker_uri), "%s", app_config::kMqttBrokerGprs);
        }
        nvs_close(nvs);
    }

    taskENTER_CRITICAL(&s_lock);
    s_initialized = true;
    taskEXIT_CRITICAL(&s_lock);

    ESP_LOGI(TAG, "Runtime brokers lan=%s gprs=%s", s_lan_broker_uri, s_gprs_broker_uri);
    return ESP_OK;
}

void getLanBrokerUri(char *out, size_t out_size) {
    if (out == nullptr || out_size == 0) {
        return;
    }
    taskENTER_CRITICAL(&s_lock);
    std::snprintf(out, out_size, "%s", s_lan_broker_uri);
    taskEXIT_CRITICAL(&s_lock);
}

void getGprsBrokerUri(char *out, size_t out_size) {
    if (out == nullptr || out_size == 0) {
        return;
    }
    taskENTER_CRITICAL(&s_lock);
    std::snprintf(out, out_size, "%s", s_gprs_broker_uri);
    taskEXIT_CRITICAL(&s_lock);
}

void getBrokerIps(char *lan_ip_out, size_t lan_ip_out_size,
                  char *gprs_ip_out, size_t gprs_ip_out_size) {
    char lan_uri[96] = {0};
    char gprs_uri[96] = {0};
    getLanBrokerUri(lan_uri, sizeof(lan_uri));
    getGprsBrokerUri(gprs_uri, sizeof(gprs_uri));

    if (lan_ip_out != nullptr && lan_ip_out_size > 0) {
        if (!extractIpFromUri(lan_uri, lan_ip_out, lan_ip_out_size)) {
            lan_ip_out[0] = '\0';
        }
    }

    if (gprs_ip_out != nullptr && gprs_ip_out_size > 0) {
        if (!extractIpFromUri(gprs_uri, gprs_ip_out, gprs_ip_out_size)) {
            gprs_ip_out[0] = '\0';
        }
    }
}

esp_err_t setBrokerIps(const char *lan_ip, const char *gprs_ip) {
    if (!isValidIpv4(lan_ip) || !isValidIpv4(gprs_ip)) {
        return ESP_ERR_INVALID_ARG;
    }

    taskENTER_CRITICAL(&s_lock);
    buildUriFromIp(lan_ip, s_lan_broker_uri, sizeof(s_lan_broker_uri));
    buildUriFromIp(gprs_ip, s_gprs_broker_uri, sizeof(s_gprs_broker_uri));
    taskEXIT_CRITICAL(&s_lock);

    saveUrisToNvs();
    ESP_LOGI(TAG, "Updated runtime brokers lan=%s gprs=%s", s_lan_broker_uri, s_gprs_broker_uri);
    return ESP_OK;
}

}  // namespace runtime_settings
