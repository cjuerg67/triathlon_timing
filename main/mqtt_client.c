#include "mqtt_client.h"

#include <stddef.h>

struct mqtt_event_registration {
    esp_event_handler_t handler;
    void *handler_args;
    int event;
};

static struct mqtt_event_registration s_registration = {0};

esp_mqtt_client_handle_t esp_mqtt_client_init(const esp_mqtt_client_config_t *config) {
    (void)config;
    return (esp_mqtt_client_handle_t)1;
}

esp_err_t esp_mqtt_client_start(esp_mqtt_client_handle_t client) {
    (void)client;
    if (s_registration.handler != NULL) {
        s_registration.handler(s_registration.handler_args, "MQTT_EVENTS", MQTT_EVENT_CONNECTED, NULL);
    }
    return ESP_OK;
}

esp_err_t esp_mqtt_client_stop(esp_mqtt_client_handle_t client) {
    (void)client;
    if (s_registration.handler != NULL) {
        s_registration.handler(s_registration.handler_args, "MQTT_EVENTS", MQTT_EVENT_DISCONNECTED, NULL);
    }
    return ESP_OK;
}

esp_err_t esp_mqtt_client_destroy(esp_mqtt_client_handle_t client) {
    (void)client;
    s_registration.handler = NULL;
    s_registration.handler_args = NULL;
    s_registration.event = 0;
    return ESP_OK;
}

esp_err_t esp_mqtt_client_register_event(esp_mqtt_client_handle_t client, int event, esp_event_handler_t handler, void *handler_args) {
    (void)client;
    s_registration.handler = handler;
    s_registration.handler_args = handler_args;
    s_registration.event = event;
    return ESP_OK;
}

int esp_mqtt_client_publish(esp_mqtt_client_handle_t client, const char *topic, const char *data, int len, int qos, int retain) {
    (void)client;
    (void)topic;
    (void)data;
    (void)len;
    (void)qos;
    (void)retain;
    return 1;
}
