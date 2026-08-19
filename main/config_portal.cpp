#include "config_portal.hpp"

#include <cstdio>
#include <cstring>
#include <ctime>

#include "app_config.hpp"
#include "esp_event.h"
#include "esp_https_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/ip4_addr.h"
#include "runtime_settings.hpp"
#include "transport_manager.hpp"

namespace {
static const char *TAG = "config_portal";
static ConfigPortal *s_instance = nullptr;
static httpd_handle_t s_server = nullptr;
static esp_netif_t *s_ap_netif = nullptr;
static bool s_running = false;
static TransportManager *s_transport_manager = nullptr;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

extern const unsigned char servercert_pem_start[] asm("_binary_servercert_pem_start");
extern const unsigned char servercert_pem_end[] asm("_binary_servercert_pem_end");
extern const unsigned char prvtkey_pem_start[] asm("_binary_prvtkey_pem_start");
extern const unsigned char prvtkey_pem_end[] asm("_binary_prvtkey_pem_end");

static const char kHtmlPage[] =
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Triathlon Config</title>"
    "<style>body{font-family:Verdana,sans-serif;background:#f2efe8;margin:0;padding:24px;}"
    "main{max-width:900px;margin:auto;background:#fff;padding:20px;border-radius:12px;box-shadow:0 10px 28px rgba(0,0,0,.08);}"
    "h1{margin-top:0;color:#223;}h2{margin:18px 0 8px;color:#234;font-size:1.05rem;}label{display:block;margin:14px 0 6px;}input{width:100%;padding:10px;border:1px solid #bbb;border-radius:8px;}"
    "button{margin-top:16px;padding:10px 14px;border:0;border-radius:8px;background:#1f6f8b;color:#fff;font-weight:700;}"
    "small{color:#666;display:block;margin-top:8px;}#msg{margin-top:12px;min-height:20px;color:#164;}#layout{display:flex;gap:24px;align-items:flex-start;}#formPanel{flex:1 1 0;}#infoPanel{flex:0 0 320px;}#infoBox{display:grid;gap:8px;}#infoBox>div{padding:10px 12px;border:1px solid #d6d6d6;border-radius:8px;background:#f8f9fa;color:#223;}@media (max-width:700px){#layout{display:block;}#infoPanel{max-width:none;margin-top:20px;}}</style></head><body><main>"
    "<h1>MQTT Runtime Config</h1>"
    "<div id='layout'><div id='formPanel'><form id='cfgForm'><label>MQTT IP for Wi-Fi/LAN</label><input id='lan_ip' name='lan_ip' required pattern='[0-9.]{7,15}'>"
    "<label>MQTT IP for GPRS</label><input id='gprs_ip' name='gprs_ip' required pattern='[0-9.]{7,15}'>"
    "<button type='submit'>Save</button><div id='msg'></div></form></div>"
    "<aside id='infoPanel'><section><h2>Infos</h2><div id='infoBox'><div id='wifi_rssi'>Wi‑Fi RSSI: loading...</div><div id='gprs_rssi'>GPRS RSSI: loading...</div><div id='cpu_busy'>CPU busy: loading...</div><div id='device_time'>Time: loading...</div></div></section></aside></div>"
    "<script>const wifiRssi=document.getElementById('wifi_rssi');const gprsRssi=document.getElementById('gprs_rssi');const cpuBusy=document.getElementById('cpu_busy');const deviceTime=document.getElementById('device_time');async function load(){const r=await fetch('/api/config');if(!r.ok)return;const j=await r.json();"
    "lan_ip.value=j.lan_ip||'';gprs_ip.value=j.gprs_ip||'';const wifiRssiValue=Number(j.wifi_rssi_dbm);wifiRssi.textContent=(Number.isFinite(wifiRssiValue)&&wifiRssiValue>-127)?('Wi‑Fi RSSI: '+wifiRssiValue+' dBm'):'Wi‑Fi RSSI: not connected';"
    "const gprsRssiValue=Number(j.gprs_rssi_dbm);gprsRssi.textContent=(Number.isFinite(gprsRssiValue)&&gprsRssiValue>-127)?('GPRS RSSI: '+gprsRssiValue+' dBm'):'GPRS RSSI: not connected';"
    "const cpuBusyValue=Number(j.cpu_busy_percent);cpuBusy.textContent=(Number.isFinite(cpuBusyValue)&&cpuBusyValue>=0)?('CPU busy: '+cpuBusyValue+'%'):'CPU busy: unavailable';"
    "deviceTime.textContent=(j.time_valid&&j.time)?('Time: '+j.time):'Time: not synchronized';}"
    "cfgForm.addEventListener('submit',async(e)=>{e.preventDefault();const b=new URLSearchParams(new FormData(cfgForm));"
    "const r=await fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b});"
    "msg.textContent=await r.text();});load();setInterval(load,5000);</script></main></body></html>";

static int currentWifiRssiDbm() {
    int rssi = -127;
    const esp_err_t ret = esp_wifi_sta_get_rssi(&rssi);
    if (ret != ESP_OK) {
        return -127;
    }
    return rssi;
}

static bool currentTimeString(char *out, size_t out_size) {
    if (out == nullptr || out_size == 0) {
        return false;
    }

    const std::time_t now = std::time(nullptr);
    const std::tm *tm_now = std::localtime(&now);
    if (tm_now == nullptr || (tm_now->tm_year + 1900) < 2024) {
        out[0] = '\0';
        return false;
    }

    const size_t written = std::strftime(out, out_size, "%Y-%m-%d %H:%M:%S", tm_now);
    return written != 0;
}

#if defined(configGENERATE_RUN_TIME_STATS) && (configGENERATE_RUN_TIME_STATS > 0)
static int currentCpuBusyPercent() {
    static constexpr UBaseType_t kMaxTasks = 16;
    TaskStatus_t tasks[kMaxTasks] = {};
    const UBaseType_t task_count = uxTaskGetSystemState(tasks, kMaxTasks, nullptr);
    if (task_count == 0) {
        return -1;
    }

    uint32_t total_runtime = 0;
    uint32_t idle_runtime = 0;
    for (UBaseType_t i = 0; i < task_count; ++i) {
        total_runtime += tasks[i].ulRunTimeCounter;
        if (std::strcmp(tasks[i].pcTaskName, "IDLE") == 0) {
            idle_runtime = tasks[i].ulRunTimeCounter;
        }
    }

    if (total_runtime == 0) {
        return -1;
    }

    const uint32_t busy_runtime = total_runtime > idle_runtime ? total_runtime - idle_runtime : 0;
    int busy_percent = static_cast<int>((busy_runtime * 100u) / total_runtime);
    if (busy_percent < 0) {
        busy_percent = 0;
    } else if (busy_percent > 100) {
        busy_percent = 100;
    }
    return busy_percent;
}
#else
static int currentCpuBusyPercent() {
    return -1;
}
#endif

bool isValidIpv4(const char *ip) {
    if (ip == nullptr || ip[0] == '\0') {
        return false;
    }
    ip4_addr_t addr = {};
    return ip4addr_aton(ip, &addr) != 0;
}

bool formValue(const char *body, const char *key, char *out, size_t out_size) {
    if (body == nullptr || key == nullptr || out == nullptr || out_size == 0) {
        return false;
    }

    const size_t key_len = std::strlen(key);
    const char *p = body;
    while (p != nullptr && *p != '\0') {
        if (std::strncmp(p, key, key_len) == 0 && p[key_len] == '=') {
            p += key_len + 1;
            size_t i = 0;
            while (p[i] != '\0' && p[i] != '&' && i < out_size - 1) {
                out[i] = p[i];
                ++i;
            }
            out[i] = '\0';
            return true;
        }
        p = std::strchr(p, '&');
        if (p != nullptr) {
            ++p;
        }
    }

    return false;
}

void unauthorized(httpd_req_t *req) {
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=triathlon-config");
    httpd_resp_send(req, "Authentication required", HTTPD_RESP_USE_STRLEN);
}

bool checkAuth(httpd_req_t *req) {
    const size_t auth_len = httpd_req_get_hdr_value_len(req, "Authorization");
    if (auth_len == 0 || auth_len > 127) {
        unauthorized(req);
        return false;
    }

    char auth[128] = {0};
    if (httpd_req_get_hdr_value_str(req, "Authorization", auth, sizeof(auth)) != ESP_OK) {
        unauthorized(req);
        return false;
    }

    if (std::strcmp(auth, app_config::kConfigPortalBasicAuthValue) != 0) {
        unauthorized(req);
        return false;
    }

    return true;
}

}  // namespace

void ConfigPortal::setTransportManager(TransportManager *transport_manager) {
    s_transport_manager = transport_manager;
}

bool ConfigPortal::isRunning() const {
    taskENTER_CRITICAL(&s_lock);
    const bool running = s_running;
    taskEXIT_CRITICAL(&s_lock);
    return running;
}

esp_err_t ConfigPortal::startSoftAp() {
    wifi_mode_t current_mode = WIFI_MODE_NULL;
    esp_err_t ret = esp_wifi_get_mode(&current_mode);
    if (ret == ESP_OK && current_mode != WIFI_MODE_NULL && current_mode != WIFI_MODE_STA && current_mode != WIFI_MODE_APSTA) {
        ESP_LOGW(TAG, "Wi‑Fi is in unsupported mode=%d; config portal requires AP or APSTA mode", current_mode);
        return ESP_ERR_INVALID_STATE;
    }

    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    if (s_ap_netif == nullptr) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        if (s_ap_netif == nullptr) {
            return ESP_FAIL;
        }
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    if (current_mode != WIFI_MODE_APSTA) {
        ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    wifi_config_t ap_cfg = {};
    std::snprintf(reinterpret_cast<char *>(ap_cfg.ap.ssid), sizeof(ap_cfg.ap.ssid), "%s", app_config::kConfigPortalApSsid);
    std::snprintf(reinterpret_cast<char *>(ap_cfg.ap.password), sizeof(ap_cfg.ap.password), "%s", app_config::kConfigPortalApPassword);
    ap_cfg.ap.ssid_len = std::strlen(app_config::kConfigPortalApSsid);
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.channel = 1;
    ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ret = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_CONN) {
        return ret;
    }

    ESP_LOGI(TAG, "Config SoftAP started: SSID=%s", app_config::kConfigPortalApSsid);
    return ESP_OK;
}

esp_err_t ConfigPortal::startHttpsServer() {
    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();
    conf.servercert = servercert_pem_start;
    conf.servercert_len = servercert_pem_end - servercert_pem_start;
    conf.prvtkey_pem = prvtkey_pem_start;
    conf.prvtkey_len = prvtkey_pem_end - prvtkey_pem_start;
    conf.port_secure = app_config::kConfigPortalHttpsPort;

    esp_err_t ret = httpd_ssl_start(&s_server, &conf);
    if (ret != ESP_OK) {
        return ret;
    }

    const httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = &ConfigPortal::rootHandler,
        .user_ctx = this,
    };

    const httpd_uri_t cfg_get = {
        .uri = "/api/config",
        .method = HTTP_GET,
        .handler = &ConfigPortal::configGetHandler,
        .user_ctx = this,
    };

    const httpd_uri_t cfg_post = {
        .uri = "/api/config",
        .method = HTTP_POST,
        .handler = &ConfigPortal::configPostHandler,
        .user_ctx = this,
    };

    httpd_register_uri_handler(s_server, &root);
    httpd_register_uri_handler(s_server, &cfg_get);
    httpd_register_uri_handler(s_server, &cfg_post);
    return ESP_OK;
}

esp_err_t ConfigPortal::start(bool start_soft_ap) {
    if (isRunning()) {
        touchActivity();
        return ESP_OK;
    }

    s_instance = this;

    esp_err_t ret = ESP_OK;
    if (start_soft_ap) {
        ret = startSoftAp();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "startSoftAp failed: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG,
                 "Wi‑Fi SoftAP active: SSID=%s password=%s",
                 app_config::kConfigPortalApSsid,
                 app_config::kConfigPortalApPassword);
    } else {
        ESP_LOGI(TAG,
                 "Config portal is using the active network connection; not starting SoftAP");
    }

    ret = startHttpsServer();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "startHttpsServer failed: %s", esp_err_to_name(ret));
        return ret;
    }

    taskENTER_CRITICAL(&s_lock);
    s_running = true;
    taskEXIT_CRITICAL(&s_lock);

    if (start_soft_ap) {
        ESP_LOGI(TAG,
                 "HTTPS config portal running on https://192.168.4.1:%d (user=%s)",
                 app_config::kConfigPortalHttpsPort,
                 app_config::kConfigPortalUsername);
    } else {
        ESP_LOGI(TAG,
                 "HTTPS config portal running on the active network (user=%s)",
                 app_config::kConfigPortalUsername);
    }
    return ESP_OK;
}

void ConfigPortal::touchActivity() {
    // Activity tracking is intentionally disabled: the portal stays available until the device is rebooted.
}

void ConfigPortal::stopInternal() {
    if (s_server != nullptr) {
        httpd_ssl_stop(s_server);
        s_server = nullptr;
    }

    esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        (void)esp_wifi_stop();
    }

    taskENTER_CRITICAL(&s_lock);
    s_running = false;
    taskEXIT_CRITICAL(&s_lock);

    ESP_LOGI(TAG, "Config portal stopped");
}

esp_err_t ConfigPortal::rootHandler(httpd_req_t *req) {
    if (!checkAuth(req)) {
        return ESP_OK;
    }

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, kHtmlPage, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t ConfigPortal::configGetHandler(httpd_req_t *req) {
    if (!checkAuth(req)) {
        return ESP_OK;
    }

    char lan_ip[32] = {0};
    char gprs_ip[32] = {0};
    runtime_settings::getBrokerIps(lan_ip, sizeof(lan_ip), gprs_ip, sizeof(gprs_ip));

    const int wifi_rssi_dbm = currentWifiRssiDbm();
    const int gprs_rssi_dbm = (s_transport_manager != nullptr) ? s_transport_manager->getCurrentGprsRssiDbm() : -127;
    const int cpu_busy_percent = currentCpuBusyPercent();
    char time_buf[32] = {0};
    const bool time_valid = currentTimeString(time_buf, sizeof(time_buf));

    char resp[360] = {0};
    std::snprintf(resp, sizeof(resp),
                  "{\"lan_ip\":\"%s\",\"gprs_ip\":\"%s\",\"wifi_rssi_dbm\":%d,\"gprs_rssi_dbm\":%d,\"cpu_busy_percent\":%d,\"time_valid\":%s,\"time\":\"%s\"}",
                  lan_ip,
                  gprs_ip,
                  wifi_rssi_dbm,
                  gprs_rssi_dbm,
                  cpu_busy_percent,
                  time_valid ? "true" : "false",
                  time_valid ? time_buf : "");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t ConfigPortal::configPostHandler(httpd_req_t *req) {
    if (!checkAuth(req)) {
        return ESP_OK;
    }

    if (req->content_len <= 0 || req->content_len > 256) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, "Invalid request size", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    char body[257] = {0};
    const int read = httpd_req_recv(req, body, req->content_len);
    if (read <= 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, "Body read failed", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    body[read] = '\0';

    char lan_ip[32] = {0};
    char gprs_ip[32] = {0};
    if (!formValue(body, "lan_ip", lan_ip, sizeof(lan_ip)) ||
        !formValue(body, "gprs_ip", gprs_ip, sizeof(gprs_ip)) ||
        !isValidIpv4(lan_ip) ||
        !isValidIpv4(gprs_ip)) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, "Invalid IPv4 values", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    const esp_err_t ret = runtime_settings::setBrokerIps(lan_ip, gprs_ip);
    if (ret != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_send(req, "Failed to store settings", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "Saved. MQTT will switch to new broker on next monitor cycle.", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
