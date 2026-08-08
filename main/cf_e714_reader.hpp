#pragma once

#include "esp_err.h"
#include "mqtt_publisher.hpp"
#include "reader_interface.hpp"
#include "time_service.hpp"

class CfE714Reader : public IRfidReader {
public:
    esp_err_t start(MqttPublisher *mqtt_publisher) override;
    void setTimeService(TimeService *ts);

private:
    static void readerTask(void *pvParameters);

    static MqttPublisher *s_mqtt_publisher;
    static TimeService *s_time_service;
    static char s_cached_time[16];
    static TimeSource s_cached_time_src;
};
