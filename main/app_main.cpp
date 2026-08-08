#include <cstdio>
#include <cstring>

#include "app_config.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_publisher.hpp"
#include "rfid_event.hpp"
#include "time_service.hpp"
#include "transport_manager.hpp"
#include "keyboard_reader.hpp"
#include "yrm100_reader.hpp"

namespace {
static const char *TAG = "triathlon_main";

const char *brokerForTransport(TransportType transport) {
    return (transport == TransportType::kGprs) ? app_config::kMqttBrokerGprs : app_config::kMqttBrokerLan;
}
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "app_main started");
    ESP_LOGI(TAG, "RFID reader and MQTT publish path active");

    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 0);

    TransportManager transport_manager;
    MqttPublisher mqtt_publisher;
    TimeService time_service;
    Yrm100Reader reader;
    KeyboardReader keyboard_reader;

    const bool connected_transport = transport_manager.connectAny();
    const char *broker_uri = brokerForTransport(transport_manager.activeTransport());
    ESP_LOGI(TAG, "transport connected=%d broker=%s", connected_transport, broker_uri != nullptr ? broker_uri : "null");

    if (connected_transport) {
        time_service.initNtp();
        const esp_err_t mqtt_start_ret = mqtt_publisher.start(broker_uri, app_config::kMqttClientId);
        if (mqtt_start_ret != ESP_OK) {
            ESP_LOGW(TAG, "MQTT init deferred: %s", esp_err_to_name(mqtt_start_ret));
        }
    }

    const esp_err_t reader_start_ret = reader.start(&mqtt_publisher);
    reader.setTimeService(&time_service);
    ESP_LOGI(TAG, "reader start ret=%d", reader_start_ret);

    const esp_err_t kb_start_ret = keyboard_reader.start(&mqtt_publisher, &time_service);
    ESP_LOGI(TAG, "keyboard start ret=%d", kb_start_ret);

    int loop_count = 0;
    while (true) {
        gpio_set_level(GPIO_NUM_2, loop_count % 2);
        loop_count++;

        if (connected_transport && !mqtt_publisher.isConnected() && (loop_count % 50) == 0) {
            const esp_err_t mqtt_retry_ret = mqtt_publisher.start(broker_uri, app_config::kMqttClientId);
            if (mqtt_retry_ret != ESP_OK) {
                ESP_LOGW(TAG, "MQTT retry failed: %s", esp_err_to_name(mqtt_retry_ret));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
