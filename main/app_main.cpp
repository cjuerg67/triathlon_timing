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
#include "status_lcd.hpp"
#include "cf_e714_reader.hpp"
#include "keyboard_reader.hpp"
#include "config_portal.hpp"
#include "runtime_settings.hpp"
#include "yrm100_reader.hpp"

namespace {
static const char *TAG = "triathlon_main";

const char *transportName(TransportType transport) {
    switch (transport) {
    case TransportType::kWifi:
        return "wifi";
    case TransportType::kEthernet:
        return "lan";
    case TransportType::kGprs:
        return "gprs";
    case TransportType::kNone:
    default:
        return "none";
    }
}

void brokerForTransport(TransportType transport, char *out, size_t out_size) {
    if (out == nullptr || out_size == 0) {
        return;
    }

    if (transport == TransportType::kGprs) {
        runtime_settings::getGprsBrokerUri(out, out_size);
    } else {
        runtime_settings::getLanBrokerUri(out, out_size);
    }
}
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "app_main started");
    ESP_LOGI(TAG, "RFID reader and MQTT publish path active");

    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 0);

    static TransportManager transport_manager;
    static MqttPublisher mqtt_publisher;
    static TimeService time_service;
    static StatusLcd status_lcd;
    static Yrm100Reader yrm100_reader;
    static CfE714Reader cf_e714_reader;
    static KeyboardReader keyboard_reader;
    static ConfigPortal config_portal;
    config_portal.setTransportManager(&transport_manager);

    if (runtime_settings::init() != ESP_OK) {
        ESP_LOGW(TAG, "Runtime settings init failed; using compile-time MQTT defaults");
    }

    if (app_config::kEnableConfigPortal) {
        ESP_LOGI(TAG, "Config portal will wait until Wi‑Fi or LAN connectivity is established before starting");
    }

    if (app_config::kEnableStatusLcd) {
        const esp_err_t lcd_init_ret = status_lcd.init();
        if (lcd_init_ret != ESP_OK) {
            ESP_LOGW(TAG, "Status LCD init failed: %s", esp_err_to_name(lcd_init_ret));
        }
    }

    bool connected_transport = transport_manager.connectAny();
    TransportType active_transport = transport_manager.activeTransport();
    if (app_config::kEnableConfigPortal) {
        if (connected_transport && active_transport == TransportType::kWifi) {
            const esp_err_t portal_ret = config_portal.start(false);
            if (portal_ret != ESP_OK) {
                ESP_LOGW(TAG, "Config portal start failed on active Wi‑Fi: %s", esp_err_to_name(portal_ret));
            }
        } else if (connected_transport && active_transport == TransportType::kEthernet) {
            const esp_err_t portal_ret = config_portal.start(false);
            if (portal_ret != ESP_OK) {
                ESP_LOGW(TAG, "Config portal start failed on active LAN: %s", esp_err_to_name(portal_ret));
            }
        } else {
            ESP_LOGI(TAG,
                     "Config portal waiting: no established Wi‑Fi or LAN transport yet (active=%d connected=%d)",
                     static_cast<int>(active_transport),
                     connected_transport);
        }
    }

    char broker_uri[96] = {0};
    brokerForTransport(active_transport, broker_uri, sizeof(broker_uri));
    ESP_LOGI(TAG, "transport connected=%d broker=%s", connected_transport, broker_uri);

    if (connected_transport) {
        time_service.initNtp();
        const esp_err_t mqtt_start_ret = mqtt_publisher.start(broker_uri, app_config::kMqttClientId);
        if (mqtt_start_ret != ESP_OK) {
            ESP_LOGW(TAG, "MQTT init deferred: %s", esp_err_to_name(mqtt_start_ret));
        }
    }

    const esp_err_t yrm100_start_ret = yrm100_reader.start(&mqtt_publisher);
    yrm100_reader.setTimeService(&time_service);
    ESP_LOGI(TAG, "reader start ret=%d", yrm100_start_ret);

    if (app_config::kEnableCfE714Reader) {
        const esp_err_t cf_start_ret = cf_e714_reader.start(&mqtt_publisher);
        cf_e714_reader.setTimeService(&time_service);
        ESP_LOGI(TAG, "cf_e714 reader start ret=%d", cf_start_ret);
    } else {
        ESP_LOGI(TAG, "cf_e714 reader disabled in config");
    }

    const esp_err_t kb_start_ret = keyboard_reader.start(&mqtt_publisher, &time_service);
    ESP_LOGI(TAG, "keyboard start ret=%d", kb_start_ret);

    int loop_count = 0;
    TickType_t last_transport_check = 0;
    TickType_t last_status_log = 0;
    TickType_t last_lcd_update = 0;
    TickType_t next_mqtt_retry = 0;
    uint32_t mqtt_retry_interval_ms = app_config::kMqttReconnectIntervalMs;
    while (true) {
        gpio_set_level(GPIO_NUM_2, loop_count % 2);
        loop_count++;

        const TickType_t now_ticks = xTaskGetTickCount();
        if ((now_ticks - last_transport_check) >= pdMS_TO_TICKS(app_config::kTransportRefreshIntervalMs)) {
            const TransportType previous_transport = active_transport;
            connected_transport = transport_manager.refreshActiveTransport();
            active_transport = transport_manager.activeTransport();
            char desired_broker_uri[96] = {0};
            brokerForTransport(active_transport, desired_broker_uri, sizeof(desired_broker_uri));

            if (app_config::kEnableConfigPortal && !config_portal.isRunning() && connected_transport) {
                const bool use_soft_ap = (active_transport != TransportType::kWifi && active_transport != TransportType::kEthernet);
                const esp_err_t portal_ret = config_portal.start(use_soft_ap);
                if (portal_ret != ESP_OK) {
                    ESP_LOGW(TAG, "Config portal start failed after transport came up: %s", esp_err_to_name(portal_ret));
                } else {
                    ESP_LOGI(TAG, "Config portal started after transport became ready on %s (soft_ap=%d)",
                             transportName(active_transport),
                             use_soft_ap);
                }
            }

            if (connected_transport &&
                (active_transport != previous_transport || std::strncmp(broker_uri, desired_broker_uri, sizeof(broker_uri)) != 0)) {
                std::snprintf(broker_uri, sizeof(broker_uri), "%s", desired_broker_uri);
                ESP_LOGW(TAG, "Transport switch detected. Restarting MQTT on %s",
                         broker_uri);
                const esp_err_t switch_ret = mqtt_publisher.start(broker_uri, app_config::kMqttClientId);
                if (switch_ret != ESP_OK) {
                    ESP_LOGW(TAG, "MQTT restart after transport switch failed: %s", esp_err_to_name(switch_ret));
                    mqtt_retry_interval_ms = app_config::kMqttReconnectIntervalMs;
                    next_mqtt_retry = now_ticks + pdMS_TO_TICKS(mqtt_retry_interval_ms);
                } else {
                    mqtt_retry_interval_ms = app_config::kMqttReconnectIntervalMs;
                    next_mqtt_retry = now_ticks + pdMS_TO_TICKS(mqtt_retry_interval_ms);
                }
            } else if (app_config::kEnableVerboseTransportLogs) {
                ESP_LOGI(TAG,
                         "transport monitor connected=%d active=%d mqtt_connected=%d",
                         connected_transport,
                         static_cast<int>(active_transport),
                         mqtt_publisher.isConnected());
            }
            last_transport_check = now_ticks;
        }

        if (app_config::kEnableRuntimeStatusLine &&
            (now_ticks - last_status_log) >= pdMS_TO_TICKS(app_config::kRuntimeStatusIntervalMs)) {
            ESP_LOGI(TAG,
                     "status transport=%s transport_up=%d mqtt_up=%d broker=%s retry_ms=%lu",
                     transportName(active_transport),
                     connected_transport,
                     mqtt_publisher.isConnected(),
                     broker_uri,
                     static_cast<unsigned long>(mqtt_retry_interval_ms));
            last_status_log = now_ticks;
        }

        if (mqtt_publisher.isConnected()) {
            mqtt_retry_interval_ms = app_config::kMqttReconnectIntervalMs;
            next_mqtt_retry = now_ticks + pdMS_TO_TICKS(mqtt_retry_interval_ms);
        }

        if (connected_transport && !mqtt_publisher.isConnected() && now_ticks >= next_mqtt_retry) {
            brokerForTransport(active_transport, broker_uri, sizeof(broker_uri));
            if (app_config::kEnableVerboseTransportLogs) {
                ESP_LOGI(TAG,
                         "MQTT retry on transport=%d broker=%s interval_ms=%lu",
                         static_cast<int>(active_transport),
                         broker_uri,
                         static_cast<unsigned long>(mqtt_retry_interval_ms));
            }
            const esp_err_t mqtt_retry_ret = mqtt_publisher.start(broker_uri, app_config::kMqttClientId);
            if (mqtt_retry_ret != ESP_OK) {
                ESP_LOGW(TAG, "MQTT retry failed: %s", esp_err_to_name(mqtt_retry_ret));
                const uint32_t doubled = mqtt_retry_interval_ms * 2;
                mqtt_retry_interval_ms = (doubled > app_config::kMqttReconnectMaxIntervalMs)
                                         ? app_config::kMqttReconnectMaxIntervalMs
                                         : doubled;
            } else {
                mqtt_retry_interval_ms = app_config::kMqttReconnectIntervalMs;
            }
            next_mqtt_retry = now_ticks + pdMS_TO_TICKS(mqtt_retry_interval_ms);
        }

        if (app_config::kEnableStatusLcd &&
            (now_ticks - last_lcd_update) >= pdMS_TO_TICKS(app_config::kStatusLcdUpdateIntervalMs)) {
            char last_tag[64] = {0};
            mqtt_publisher.getLastTag(last_tag, sizeof(last_tag));
            status_lcd.update(time_service,
                              active_transport,
                              connected_transport,
                              mqtt_publisher.isConnected(),
                              last_tag);
            last_lcd_update = now_ticks;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
