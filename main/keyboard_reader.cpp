#include "keyboard_reader.hpp"

#include <cstdio>
#include <cstring>

#include "app_config.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "usb/hid_host.h"

namespace {
static const char *TAG = "keyboard_reader";

static MqttPublisher *s_mqtt_publisher = nullptr;
static TimeService *s_time_service = nullptr;
static char s_buf[32] = {0};
static size_t s_buf_len = 0;
static uint8_t s_prev_keys[6] = {0};

// Publish is offloaded here so the HID callback returns instantly and the USB
// interrupt-IN URB is resubmitted before the keyboard sends its key-release packet.
static QueueHandle_t s_publish_queue = nullptr;

static void publishTask(void *) {
    char number[32];
    while (true) {
        if (xQueueReceive(s_publish_queue, number, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "starter number entered: %s", number);
            if (s_mqtt_publisher != nullptr && s_mqtt_publisher->isConnected()) {
                char time_buf[16] = "00:00:00";
                if (s_time_service != nullptr) {
                    s_time_service->getCurrentTimeHHMMSS(time_buf, sizeof(time_buf));
                }
                s_mqtt_publisher->publishStarterNumber(number, time_buf);
            } else {
                ESP_LOGW(TAG, "MQTT not connected; dropped starter number %s", number);
            }
        }
    }
}

static char keycodeToChar(uint8_t keycode) {
    if (keycode >= 0x1E && keycode <= 0x26) return static_cast<char>('1' + (keycode - 0x1E));
    if (keycode == 0x27) return '0';
    if (keycode >= 0x59 && keycode <= 0x61) return static_cast<char>('1' + (keycode - 0x59)); // KP1-KP9
    if (keycode == 0x62) return '0'; // KP0
    if (keycode == 0x28 || keycode == 0x58) return '\n'; // Enter / KP Enter
    if (keycode == 0x2A) return '\b'; // Backspace
    return '\0';
}

static void processReport(const uint8_t *data, size_t len) {
    if (data == nullptr || len < 3) {
        return;
    }

    // Boot protocol: data[0]=modifier, data[1]=reserved, data[2..7]=keycodes
    const uint8_t *keys = &data[2];
    const size_t key_count = (len - 2 < 6) ? (len - 2) : 6;

    for (size_t i = 0; i < key_count; ++i) {
        const uint8_t key = keys[i];
        if (key == 0x00 || key == 0x01) {
            continue;
        }

        bool is_new = true;
        for (size_t j = 0; j < 6; ++j) {
            if (s_prev_keys[j] == key) { is_new = false; break; }
        }
        if (!is_new) continue;

        const char ch = keycodeToChar(key);
        if (ch == '\0') continue;

        if (ch == '\b') {
            if (s_buf_len > 0) s_buf[--s_buf_len] = '\0';
            ESP_LOGI(TAG, "backspace -> buf='%.*s'", static_cast<int>(s_buf_len), s_buf);
        } else if (ch == '\n') {
            if (s_buf_len > 0) {
                s_buf[s_buf_len] = '\0';
                if (s_publish_queue != nullptr) {
                    xQueueSend(s_publish_queue, s_buf, 0);
                }
                s_buf_len = 0;
                std::memset(s_buf, 0, sizeof(s_buf));
            }
        } else if (s_buf_len < sizeof(s_buf) - 1) {
            s_buf[s_buf_len++] = ch;
            ESP_LOGI(TAG, "key '%c' -> buf='%.*s'", ch, static_cast<int>(s_buf_len), s_buf);
        }
    }

    std::memcpy(s_prev_keys, keys, 6);
    if (key_count < 6) std::memset(s_prev_keys + key_count, 0, 6 - key_count);
}

static void interfaceCallback(hid_host_device_handle_t device,
                               const hid_host_interface_event_t event, void *arg) {
    switch (event) {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT: {
        uint8_t data[8] = {0};
        size_t data_len = 0;
        if (hid_host_device_get_raw_input_report_data(device, data, sizeof(data), &data_len) == ESP_OK) {
            processReport(data, data_len);
        }
        break;
    }
    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Keyboard disconnected");
        std::memset(s_prev_keys, 0, sizeof(s_prev_keys));
        hid_host_device_close(device);
        break;
    case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
        ESP_LOGW(TAG, "Keyboard transfer error");
        break;
    default:
        break;
    }
}

static void deviceCallback(hid_host_device_handle_t device,
                            const hid_host_driver_event_t event, void *arg) {
    if (event != HID_HOST_DRIVER_EVENT_CONNECTED) return;

    hid_host_dev_params_t params = {};
    if (hid_host_device_get_params(device, &params) != ESP_OK) return;

    if (params.proto != HID_PROTOCOL_KEYBOARD) {
        ESP_LOGD(TAG, "Non-keyboard HID device ignored (proto=%d)", static_cast<int>(params.proto));
        return;
    }

    ESP_LOGI(TAG, "Keyboard connected");
    const hid_host_device_config_t dev_cfg = {
        .callback = interfaceCallback,
        .callback_arg = nullptr,
    };
    if (hid_host_device_open(device, &dev_cfg) != ESP_OK) return;

    // Skip set_protocol/set_idle — most keypads default to boot protocol and
    // control transfers to these requests cause timeout-induced reconnect loops.
    if (hid_host_device_start(device) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to start keyboard input; closing device");
        hid_host_device_close(device);
    }
}
} // namespace

esp_err_t KeyboardReader::start(MqttPublisher *mqtt_publisher, TimeService *time_service) {
    s_mqtt_publisher = mqtt_publisher;
    s_time_service = time_service;

    s_publish_queue = xQueueCreate(4, sizeof(s_buf));

    const hid_host_driver_config_t config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 0,
        .callback = deviceCallback,
        .callback_arg = nullptr,
    };
    const esp_err_t ret = hid_host_install(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "hid_host_install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    xTaskCreate(publishTask, "kb_publish", 4096, nullptr, 4, nullptr);
    ESP_LOGI(TAG, "HID keyboard driver started");
    return ret;
}
