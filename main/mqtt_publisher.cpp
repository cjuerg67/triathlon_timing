#include "mqtt_publisher.hpp"

#include <cstdio>

#include "app_config.hpp"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "rfid_deduplicator.hpp"

namespace {
static const char *TAG = "mqtt_publisher";
static RfidDeduplicator s_publish_deduplicator(app_config::kRfidDuplicateDebounceMs);
static SemaphoreHandle_t s_publish_dedup_mutex = nullptr;
}

esp_err_t MqttPublisher::start(const char *broker_uri, const char *client_id) {
    if (broker_uri == nullptr || client_id == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (client_ != nullptr && connected_ && std::strncmp(broker_uri_, broker_uri, sizeof(broker_uri_)) == 0) {
        return ESP_OK;
    }

    if (client_ != nullptr) {
        esp_mqtt_client_stop(client_);
        esp_mqtt_client_destroy(client_);
        client_ = nullptr;
        connected_ = false;
    }

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = broker_uri;
    mqtt_cfg.session.last_will.topic = nullptr;
    mqtt_cfg.credentials.client_id = client_id;

    connected_ = false;

    client_ = esp_mqtt_client_init(&mqtt_cfg);
    if (client_ == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }

    std::snprintf(broker_uri_, sizeof(broker_uri_), "%s", broker_uri);

    esp_err_t event_ret = esp_mqtt_client_register_event(client_, MQTT_EVENT_ANY, &MqttPublisher::mqttEventHandler, this);
    if (event_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register MQTT event handler: %s", esp_err_to_name(event_ret));
        esp_mqtt_client_destroy(client_);
        client_ = nullptr;
        return event_ret;
    }

    ESP_LOGI(TAG, "Starting MQTT client for %s", broker_uri);
    const esp_err_t start_ret = esp_mqtt_client_start(client_);
    ESP_LOGI(TAG, "MQTT start ret=%d", start_ret);
    return start_ret;
}

bool MqttPublisher::isConnected() const {
    return connected_;
}

void MqttPublisher::getLastTag(char *out, size_t out_size) const {
    if (out == nullptr || out_size == 0) {
        return;
    }

    taskENTER_CRITICAL(&state_lock_);
    std::snprintf(out, out_size, "%s", last_tag_);
    taskEXIT_CRITICAL(&state_lock_);
}

bool MqttPublisher::publishTag(const char *rfid_id, const char *hhmmss) {
    if (client_ == nullptr || rfid_id == nullptr || hhmmss == nullptr) {
        return false;
    }

    taskENTER_CRITICAL(&state_lock_);
    std::snprintf(last_tag_, sizeof(last_tag_), "%s", rfid_id);
    std::snprintf(last_tag_time_, sizeof(last_tag_time_), "%s", hhmmss);
    taskEXIT_CRITICAL(&state_lock_);

    if (s_publish_dedup_mutex == nullptr) {
        s_publish_dedup_mutex = xSemaphoreCreateMutex();
        if (s_publish_dedup_mutex == nullptr) {
            ESP_LOGW(TAG, "Failed to create publish dedup mutex");
            return false;
        }
    }

    const uint32_t now_ms = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
    RfidEvent outbound_event = {};
    std::snprintf(outbound_event.reader_id, sizeof(outbound_event.reader_id), "%s", "mqtt_out");
    std::snprintf(outbound_event.rfid_id, sizeof(outbound_event.rfid_id), "%s", rfid_id);
    outbound_event.tick_ms = now_ms;

    if (xSemaphoreTake(s_publish_dedup_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        const bool drop = s_publish_deduplicator.shouldDrop(outbound_event, now_ms);
        xSemaphoreGive(s_publish_dedup_mutex);
        if (drop) {
            ESP_LOGD(TAG, "Suppressed duplicate publish for EPC %s", rfid_id);
            return true;
        }
    }

    if (!connected_) {
        ESP_LOGI(TAG, "MQTT not connected; skipping publish for EPC %s", rfid_id);
        return false;
    }

    char payload[160] = {0};
    std::snprintf(payload, sizeof(payload), "{\"rfid_id\":\"%s\",\"time\":\"%s\"}", rfid_id, hhmmss);

    const int msg_id_event = esp_mqtt_client_publish(client_, app_config::kMqttTopicTag, payload, 0, 0, 0);
    const int msg_id_last = esp_mqtt_client_publish(client_, app_config::kMqttTopicTagLast, payload, 0, 0, 1);
    ESP_LOGI(TAG, "publishTag epc=%s msg_id_event=%d msg_id_last=%d msg_Topic=%s", rfid_id, msg_id_event, msg_id_last, app_config::kMqttTopicTag);
    return msg_id_event >= 0 && msg_id_last >= 0;
}

bool MqttPublisher::publishGpsStatus(const char *hhmmss, TimeSource source) {
    if (client_ == nullptr || !connected_ || hhmmss == nullptr) {
        return false;
    }

    const char *source_str = "fallback";
    if (source == TimeSource::kGps) {
        source_str = "gps";
    } else if (source == TimeSource::kGsm) {
        source_str = "gsm";
    }

    char payload[128] = {0};
    std::snprintf(payload, sizeof(payload), "{\"time\":\"%s\",\"source\":\"%s\"}", hhmmss, source_str);

    return esp_mqtt_client_publish(client_, app_config::kMqttTopicGps, payload, 0, 0, 0) >= 0;
}

bool MqttPublisher::publishStarterNumber(const char *number, const char *hhmmss) {
    if (client_ == nullptr || !connected_ || number == nullptr || hhmmss == nullptr) {
        return false;
    }
    char payload[128] = {0};
    std::snprintf(payload, sizeof(payload), "{\"starter_number\":\"%s\",\"time\":\"%s\"}", number, hhmmss);
    const int msg_id = esp_mqtt_client_publish(client_, app_config::kMqttTopicStarter, payload, 0, 0, 0);
    ESP_LOGI(TAG, "publishStarterNumber number=%s msg_id=%d", number, msg_id);
    return msg_id >= 0;
}

void MqttPublisher::mqttEventHandler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    (void)base;
    (void)event_data;

    auto *self = static_cast<MqttPublisher *>(handler_args);
    if (self == nullptr) {
        return;
    }

    switch (event_id) {
    case MQTT_EVENT_BEFORE_CONNECT:
        self->connected_ = false;
        ESP_LOGI(TAG, "MQTT connecting to %s", self->broker_uri_);
        break;
    case MQTT_EVENT_CONNECTED:
        self->connected_ = true;
        ESP_LOGI(TAG, "MQTT connected");
        break;
    case MQTT_EVENT_DISCONNECTED:
        self->connected_ = false;
        ESP_LOGW(TAG, "MQTT disconnected");
        break;
    case MQTT_EVENT_ERROR:
        self->connected_ = false;
        ESP_LOGE(TAG, "MQTT connection error for %s", self->broker_uri_);
        break;
    default:
        ESP_LOGD(TAG, "MQTT event %d", static_cast<int>(event_id));
        break;
    }
}
