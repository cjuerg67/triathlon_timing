#pragma once

#include "esp_err.h"
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void *esp_mqtt_client_handle_t;

enum {
    MQTT_EVENT_ANY = -1,
    MQTT_EVENT_CONNECTED = 1,
    MQTT_EVENT_DISCONNECTED = 2,
};

typedef struct {
    struct {
        struct {
            const char *uri;
        } address;
    } broker;
    struct {
        struct {
            const char *topic;
        } last_will;
    } session;
    struct {
        const char *client_id;
    } credentials;
} esp_mqtt_client_config_t;

esp_mqtt_client_handle_t esp_mqtt_client_init(const esp_mqtt_client_config_t *config);
esp_err_t esp_mqtt_client_start(esp_mqtt_client_handle_t client);
esp_err_t esp_mqtt_client_stop(esp_mqtt_client_handle_t client);
esp_err_t esp_mqtt_client_destroy(esp_mqtt_client_handle_t client);
esp_err_t esp_mqtt_client_register_event(esp_mqtt_client_handle_t client, int event, esp_event_handler_t handler, void *handler_args);
int esp_mqtt_client_publish(esp_mqtt_client_handle_t client, const char *topic, const char *data, int len, int qos, int retain);

#ifdef __cplusplus
}
#endif
