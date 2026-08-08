#pragma once

#include "esp_err.h"
#include "mqtt_publisher.hpp"
#include "time_service.hpp"

class KeyboardReader {
public:
    esp_err_t start(MqttPublisher *mqtt_publisher, TimeService *time_service);
};
