#pragma once

#include <cstring>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "mqtt_client.h"
#include "time_service.hpp"

class MqttPublisher {
public:
    esp_err_t start(const char *broker_uri, const char *client_id);
    bool isConnected() const;
    bool publishTag(const char *rfid_id, const char *hhmmss);
    bool publishStarterNumber(const char *number, const char *hhmmss);
    bool publishGpsStatus(const char *hhmmss, TimeSource source);
    void getLastTag(char *out, size_t out_size) const;

private:
    static void mqttEventHandler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

    esp_mqtt_client_handle_t client_ = nullptr;
    bool connected_ = false;
    char broker_uri_[96] = {0};
    char last_tag_[64] = {0};
    char last_tag_time_[16] = {0};
    SemaphoreHandle_t publish_ack_sem_ = nullptr;
    int pending_ack_msg_id_ = -1;
    bool pending_ack_wait_ = false;
    bool pending_ack_confirmed_ = false;
    mutable portMUX_TYPE state_lock_ = portMUX_INITIALIZER_UNLOCKED;
};
