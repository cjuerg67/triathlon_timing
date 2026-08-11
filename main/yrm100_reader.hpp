#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "mqtt_publisher.hpp"
#include "reader_interface.hpp"
#include "time_service.hpp"
#include "usb/usb_host.h"
#include "usb/vcp.hpp"

class Yrm100Reader : public IRfidReader {
public:
    esp_err_t start(MqttPublisher *mqtt_publisher) override;
    void setTimeService(TimeService *ts);

private:
    struct ReaderRxState {
        uint8_t data[2048];
        size_t len;
        size_t consumed;
        size_t valid_frames;
        uint8_t last_type;
        uint8_t last_cmd;
    };

    static void usbLibDaemonTask(void *pvParameters);
    static void rfidTask(void *pvParameters);
    static void timeUpdateTask(void *pvParameters);
    static void publishTask(void *pvParameters);
    static void usbEventCallback(const usb_host_client_event_msg_t *event_msg, void *arg);
    static bool readerDataCallback(const uint8_t *data, size_t data_len, void *user_arg);
    static bool waitForNewFrame(ReaderRxState *state, size_t previous_valid_frames, uint32_t timeout_ms);
    static esp_err_t sendReaderFrame(CdcAcmDevice *dev, const char *label, const std::vector<uint8_t> &frame, uint32_t timeout_ms);

    static void processReaderStream(ReaderRxState *state);
    static std::vector<uint8_t> buildYrm100Frame(uint8_t msg_type, uint8_t cmd_code, const std::vector<uint8_t> &data);
    static std::vector<uint8_t> buildInventoryFrame();
    static std::vector<uint8_t> buildStopMultiFrame();

    static void emitTagFromInventoryFrame(const uint8_t *frame, size_t frame_len);

    static MqttPublisher *s_mqtt_publisher;
    static TimeService *s_time_service;
    static char s_cached_time[16];
    static TimeSource s_cached_time_src;
    static usb_host_client_handle_t s_client_handle;
};
