#pragma once

#include "esp_err.h"

class MqttPublisher;

class IRfidReader {
public:
    virtual ~IRfidReader() = default;
    virtual esp_err_t start(MqttPublisher *mqtt_publisher) = 0;
};
