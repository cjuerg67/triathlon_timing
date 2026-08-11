#pragma once

#include "esp_http_server.h"
#include "esp_err.h"

class ConfigPortal {
public:
    esp_err_t start(bool start_soft_ap = false);
    bool isRunning() const;

private:
    esp_err_t startSoftAp();
    esp_err_t startHttpsServer();
    void stopInternal();
    void touchActivity();

    static void inactivityTask(void *arg);
    static esp_err_t rootHandler(httpd_req_t *req);
    static esp_err_t configGetHandler(httpd_req_t *req);
    static esp_err_t configPostHandler(httpd_req_t *req);
};
