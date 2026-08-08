#pragma once

#include <cstring>

#include "esp_err.h"
#include "mqtt_client.h"
#include "time_service.hpp"

class MqttPublisher {
public:
    esp_err_t start(const char *broker_uri, const char *client_id);
    bool isConnected() const;
    bool publishTag(const char *rfid_id, const char *hhmmss);
    bool publishGpsStatus(const char *hhmmss, TimeSource source);

private:
    static void mqttEventHandler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

    esp_mqtt_client_handle_t client_ = nullptr;
    bool connected_ = false;
    char broker_uri_[96] = {0};
};
