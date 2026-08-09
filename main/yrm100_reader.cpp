#include "yrm100_reader.hpp"

#include <array>
#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_config.hpp"
#include "rfid_event.hpp"
#include "rfid_deduplicator.hpp"
#include "usb/vcp_ch34x.hpp"

using namespace esp_usb;

namespace {
static const char *TAG = "yrm100_reader";
static constexpr bool kVerboseLogs = false;
static constexpr uint32_t kInventoryPollIntervalMs = 120;
static constexpr uint32_t kInventoryTxTimeoutMs = 300;
static constexpr uint32_t kInventoryErrorBackoffMs = 500;
static constexpr size_t kConfirmSlots = 24;
static constexpr uint16_t kReadMultiLoopCount = 0xFFFF;

#define YRM100_STATUS_LOG(...)                               \
    do {                                                     \
        if (app_config::kEnableYrm100StatusLogs) {          \
            ESP_LOGI(TAG, __VA_ARGS__);                     \
        }                                                    \
    } while (0)

void logBytes(const char *label, const std::vector<uint8_t> &bytes) {
    char buffer[256] = {0};
    size_t offset = 0;
    for (size_t i = 0; i < bytes.size() && offset < sizeof(buffer) - 4; ++i) {
        offset += std::snprintf(buffer + offset, sizeof(buffer) - offset, "%02X ", bytes[i]);
    }
    ESP_LOGD(TAG, "%s: %s", label, buffer);
}

struct ProbeCommand {
    const char *name;
    std::vector<uint8_t> frame;
    uint32_t settle_ms;
    uint32_t wait_ms;
    bool expect_response;
};

struct ConfirmEntry {
    bool used;
    char epc[96];
    uint32_t first_seen_ms;
    uint32_t last_seen_ms;
    uint8_t count;
};

ConfirmEntry s_confirm_entries[kConfirmSlots] = {};

bool extractEpcWindow(const uint8_t *tag_data,
                      size_t data_len,
                      size_t &epc_start,
                      size_t &epc_bytes,
                      uint16_t &pc) {
    if (tag_data == nullptr || data_len < 6) {
        return false;
    }

    // The reader's notification frames observed on the wire are 17 bytes:
    // RSSI(1) + PC(2) + EPC(12) + CRC(2). The real tag ID is the 12-byte EPC.
    if (data_len != 17) {
        return false;
    }

    epc_start = 3;
    epc_bytes = 12;
    pc = (static_cast<uint16_t>(tag_data[1]) << 8) | tag_data[2];
    return true;
}

bool isConfirmedTag(const char *epc, uint32_t now_ms) {
    if (epc == nullptr || epc[0] == '\0') {
        return false;
    }

    size_t free_slot = kConfirmSlots;
    for (size_t i = 0; i < kConfirmSlots; ++i) {
        if (!s_confirm_entries[i].used) {
            if (free_slot == kConfirmSlots) {
                free_slot = i;
            }
            continue;
        }

        if (std::strncmp(s_confirm_entries[i].epc, epc, sizeof(s_confirm_entries[i].epc)) != 0) {
            continue;
        }

        const uint32_t age_ms = now_ms - s_confirm_entries[i].first_seen_ms;
        if (age_ms <= app_config::kYrm100ConfirmWindowMs) {
            if (s_confirm_entries[i].count < 255) {
                s_confirm_entries[i].count++;
            }
        } else {
            s_confirm_entries[i].first_seen_ms = now_ms;
            s_confirm_entries[i].count = 1;
        }

        s_confirm_entries[i].last_seen_ms = now_ms;
        return s_confirm_entries[i].count >= app_config::kYrm100RequiredSightings;
    }

    if (free_slot == kConfirmSlots) {
        free_slot = 0;
    }

    s_confirm_entries[free_slot].used = true;
    std::snprintf(s_confirm_entries[free_slot].epc, sizeof(s_confirm_entries[free_slot].epc), "%s", epc);
    s_confirm_entries[free_slot].first_seen_ms = now_ms;
    s_confirm_entries[free_slot].last_seen_ms = now_ms;
    s_confirm_entries[free_slot].count = 1;
    return app_config::kYrm100RequiredSightings <= 1;
}
}

MqttPublisher *Yrm100Reader::s_mqtt_publisher = nullptr;
TimeService *Yrm100Reader::s_time_service = nullptr;
usb_host_client_handle_t Yrm100Reader::s_client_handle = nullptr;
char Yrm100Reader::s_cached_time[16] = "00:00:00";
TimeSource Yrm100Reader::s_cached_time_src = TimeSource::kFallback;
static RfidDeduplicator s_deduplicator(app_config::kRfidDuplicateDebounceMs);

std::vector<uint8_t> Yrm100Reader::buildYrm100Frame(uint8_t msg_type, uint8_t cmd_code, const std::vector<uint8_t> &data) {
    std::vector<uint8_t> frame;
    frame.push_back(0xBB);
    frame.push_back(msg_type);
    frame.push_back(cmd_code);

    if (data.empty()) {
        frame.push_back(0x00);
        frame.push_back(0x00);
        frame.push_back(cmd_code);
        frame.push_back(0x7E);
        return frame;
    }

    frame.push_back(static_cast<uint8_t>((data.size() >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(data.size() & 0xFF));
    frame.insert(frame.end(), data.begin(), data.end());

    uint8_t checksum = 0;
    for (size_t i = 1; i < frame.size(); ++i) {
        checksum += frame[i];
    }
    checksum &= 0xFF;
    frame.push_back(checksum);
    frame.push_back(0x7E);
    return frame;
}

std::vector<uint8_t> Yrm100Reader::buildInventoryFrame() {
    return {0xBB, 0x00, 0x22, 0x00, 0x00, 0x22, 0x7E};
}

std::vector<uint8_t> Yrm100Reader::buildStopMultiFrame() {
    return buildYrm100Frame(0x00, 0x28, {});
}

static std::vector<uint8_t> buildReadMultiFrame(uint16_t loop_count) {
    const uint8_t hi = static_cast<uint8_t>((loop_count >> 8) & 0xFF);
    const uint8_t lo = static_cast<uint8_t>(loop_count & 0xFF);
    // BB 00 27 00 03 22 [loop_hi] [loop_lo] [checksum] 7E
    const uint8_t checksum = static_cast<uint8_t>((0x00 + 0x27 + 0x00 + 0x03 + 0x22 + hi + lo) & 0xFF);
    return {0xBB, 0x00, 0x27, 0x00, 0x03, 0x22, hi, lo, checksum, 0x7E};
}

void Yrm100Reader::emitTagFromInventoryFrame(const uint8_t *frame, size_t frame_len) {
    if (frame == nullptr || frame_len < 7 || s_mqtt_publisher == nullptr) {
        return;
    }

    // Accept both response(0x01) and notification(0x02) inventory frames.
    if ((frame[1] != 0x01 && frame[1] != 0x02) || frame[2] != 0x22) {
        return;
    }

    ESP_LOGD(TAG, "inventory frame len=%zu", frame_len);

    const uint16_t data_len = (static_cast<uint16_t>(frame[3]) << 8) | frame[4];
    if (data_len == 0 || (5U + data_len) > frame_len) {
        return;
    }

    const uint8_t *tag_data = frame + 5;
    size_t epc_start = 0;
    size_t epc_bytes = 0;
    uint16_t pc = 0;
    (void)extractEpcWindow(tag_data, data_len, epc_start, epc_bytes, pc);

    if (epc_bytes != 12) {
        return;
    }

    char epc_hex[96] = {0};
    size_t epc_offset = 0;
    for (size_t k = 0; k < epc_bytes && epc_offset < sizeof(epc_hex) - 3; ++k) {
        epc_offset += std::snprintf(epc_hex + epc_offset, sizeof(epc_hex) - epc_offset, "%02X", tag_data[epc_start + k]);
    }

    if (epc_hex[0] == '\0') {
        return;
    }

    // Basic EPC sanity: exact 12-byte EPC => 24 hex chars.
    if (epc_offset != 24) {
        return;
    }

    // Reject degenerate all-00 or all-FF payloads.
    bool all_zero = true;
    bool all_ff = true;
    for (size_t k = 0; k < epc_bytes; ++k) {
        const uint8_t b = tag_data[epc_start + k];
        all_zero = all_zero && (b == 0x00);
        all_ff = all_ff && (b == 0xFF);
    }
    if (all_zero || all_ff) {
        return;
    }

    const char *normalized_epc = epc_hex;

    const uint32_t now_ms = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (!isConfirmedTag(normalized_epc, now_ms)) {
        return;
    }

    RfidEvent event = {};
    std::snprintf(event.rfid_id, sizeof(event.rfid_id), "%s", normalized_epc);
    std::snprintf(event.reader_id, sizeof(event.reader_id), "yrm100");
    event.tick_ms = now_ms;
    if (s_deduplicator.shouldDrop(event, now_ms)) {
        return;
    }

    char time_buf[16] = {0};
    // use pre-resolved cache; calling time service here blocks the USB callback
    std::snprintf(time_buf, sizeof(time_buf), "%s", s_cached_time);
    if (s_cached_time_src == TimeSource::kFallback) {
        std::snprintf(time_buf, sizeof(time_buf), "%lu",
                     static_cast<unsigned long>(xTaskGetTickCount() * portTICK_PERIOD_MS));
    }

    if (s_mqtt_publisher != nullptr) {
        ESP_LOGI(TAG,
                 "Tag detected EPC=%s time=%s mqtt_connected=%d",
                 normalized_epc,
                 time_buf,
                 s_mqtt_publisher->isConnected());
        if (s_mqtt_publisher->isConnected()) {
            const bool published = s_mqtt_publisher->publishTag(normalized_epc, time_buf);
            if (!published) {
                ESP_LOGW(TAG, "MQTT publish failed for EPC %s", normalized_epc);
            }
        } else {
            YRM100_STATUS_LOG("MQTT not connected yet; logged reader event for EPC %s", normalized_epc);
        }
    }
}

void Yrm100Reader::processReaderStream(ReaderRxState *state) {
    if (state == nullptr || state->len == 0) {
        return;
    }

    size_t idx = state->consumed;
    while (idx + 7 < state->len) {
        if (state->data[idx] != 0xBB) {
            ++idx;
            continue;
        }

        const uint16_t data_len = (static_cast<uint16_t>(state->data[idx + 3]) << 8) | state->data[idx + 4];
        const size_t frame_len = static_cast<size_t>(data_len) + 7;
        if (idx + frame_len > state->len) {
            break;
        }

        const uint8_t *frame = &state->data[idx];
        if (frame[frame_len - 1] != 0x7E) {
            ++idx;
            continue;
        }

        uint8_t checksum = 0;
        for (size_t j = 1; j < frame_len - 2; ++j) {
            checksum = static_cast<uint8_t>(checksum + frame[j]);
        }

        const uint8_t rx_checksum = frame[frame_len - 2];
        if (checksum != rx_checksum) {
            ++idx;
            continue;
        }

        const uint8_t msg_type = frame[1];
        const uint8_t cmd = frame[2];
        state->valid_frames++;
        state->last_type = msg_type;
        state->last_cmd = cmd;

        YRM100_STATUS_LOG("valid frame type=0x%02X cmd=0x%02X data_len=%u", msg_type, cmd, data_len);
        if ((msg_type == 0x01 || msg_type == 0x02) && cmd == 0x22 && data_len > 0) {
            std::vector<uint8_t> payload(frame + 5, frame + 5 + data_len);
            logBytes("payload", payload);
            emitTagFromInventoryFrame(frame, frame_len);
        }

        idx += frame_len;
    }

    state->consumed = idx;
    if (state->consumed > 0 && state->consumed == state->len) {
        state->len = 0;
        state->consumed = 0;
        return;
    }

    if (state->consumed > 1024 || state->len == sizeof(state->data)) {
        const size_t remaining = state->len - state->consumed;
        if (remaining > 0) {
            std::memmove(state->data, state->data + state->consumed, remaining);
        }
        state->len = remaining;
        state->consumed = 0;
    }
}

bool Yrm100Reader::readerDataCallback(const uint8_t *data, size_t data_len, void *user_arg) {
    auto *state = static_cast<ReaderRxState *>(user_arg);
    if (state == nullptr || data == nullptr || data_len == 0) {
        return true;
    }

    if (state->len + data_len > sizeof(state->data)) {
        data_len = sizeof(state->data) - state->len;
    }

    if (data_len > 0) {
        std::vector<uint8_t> rx_bytes(data, data + data_len);
        logBytes("rx chunk", rx_bytes);
        ESP_LOGD(TAG, "rx chunk len=%zu total=%zu consumed=%zu", data_len, state->len + data_len, state->consumed);
        std::memcpy(state->data + state->len, data, data_len);
        state->len += data_len;
        processReaderStream(state);
    }

    return true;
}

bool Yrm100Reader::waitForNewFrame(ReaderRxState *state, size_t previous_valid_frames, uint32_t timeout_ms) {
    const int steps = static_cast<int>(timeout_ms / 50);
    for (int i = 0; i < steps; ++i) {
        if (state->valid_frames > previous_valid_frames) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return state->valid_frames > previous_valid_frames;
}

esp_err_t Yrm100Reader::sendReaderFrame(CdcAcmDevice *dev, const char *label, const std::vector<uint8_t> &frame, uint32_t timeout_ms) {
    ESP_LOGD(TAG, "tx %s frame len=%zu", label, frame.size());
    logBytes(label, frame);
    std::vector<uint8_t> tx_bytes = frame;
    const esp_err_t err = dev->tx_blocking(tx_bytes.data(), tx_bytes.size(), timeout_ms);
    ESP_LOGD(TAG, "tx %s ret=%d", label, err);
    return err;
}

void Yrm100Reader::usbEventCallback(const usb_host_client_event_msg_t *event_msg, void *arg) {
    (void)arg;
    if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        ESP_LOGD(TAG, "USB device connected");
    } else if (event_msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        ESP_LOGW(TAG, "USB device disconnected");
    }
}

void Yrm100Reader::usbLibDaemonTask(void *pvParameters) {
    (void)pvParameters;
    while (true) {
        if (s_client_handle != nullptr) {
            usb_host_client_handle_events(s_client_handle, pdMS_TO_TICKS(50));
        }

        uint32_t event_flags = 0;
        usb_host_lib_handle_events(pdMS_TO_TICKS(50), &event_flags);
    }
}

void Yrm100Reader::rfidTask(void *pvParameters) {
    (void)pvParameters;

    static bool driver_registered = false;
    if (!driver_registered) {
        VCP::register_driver<CH34x>();
        driver_registered = true;
    }

    ESP_LOGD(TAG, "Waiting for USB reader...");

    ReaderRxState rx_state = {};
    rx_state.len = 0;
    rx_state.consumed = 0;
    rx_state.valid_frames = 0;
    rx_state.last_type = 0;
    rx_state.last_cmd = 0;

    cdc_acm_host_device_config_t dev_cfg = {
        .connection_timeout_ms = 3000,
        .out_buffer_size = 64,
        .in_buffer_size = 64,
        .event_cb = nullptr,
        .data_cb = readerDataCallback,
        .user_arg = &rx_state,
    };

    CdcAcmDevice *dev = VCP::open(NANJING_QINHENG_MICROE_VID, CH340_PID_1, &dev_cfg, 0);
    if (dev == nullptr) {
        ESP_LOGW(TAG, "No YRM100/CH34x reader detected");
        // Deregister so the VCP client no longer intercepts USB events (e.g. keyboard).
        if (s_client_handle != nullptr) {
            usb_host_client_deregister(s_client_handle);
            s_client_handle = nullptr;
        }
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGD(TAG, "Reader device opened; waiting briefly for the USB stack to settle");
    vTaskDelay(pdMS_TO_TICKS(1000));

    bool got_response = false;
    bool dtr_state = true;
    bool rts_state = true;
    static constexpr std::array<int, 1> baud_rates = {115200};
    const std::vector<ProbeCommand> setup_sequence = {
        {"Get Module Info (HW)", buildYrm100Frame(0x00, 0x03, {0x00}), 150, 700, true},
        {"Set Region (CHN2)", buildYrm100Frame(0x00, 0x07, {0x01}), 150, 700, true},
        {"Get Power", buildYrm100Frame(0x00, 0xB7, {}), 150, 700, true},
    };

    for (int baud_rate : baud_rates) {
        rx_state.len = 0;
        rx_state.consumed = 0;
        rx_state.valid_frames = 0;
        rx_state.last_type = 0;
        rx_state.last_cmd = 0;
        std::memset(rx_state.data, 0, sizeof(rx_state.data));

        cdc_acm_line_coding_t line_coding = {
            .dwDTERate = static_cast<uint32_t>(baud_rate),
            .bCharFormat = 0,
            .bParityType = 0,
            .bDataBits = 8,
        };
        esp_err_t err = dev->line_coding_set(&line_coding);
        if (err != ESP_OK && err != ESP_ERR_NOT_SUPPORTED) {
            ESP_LOGW(TAG, "line_coding_set failed for %d baud: %s", baud_rate, esp_err_to_name(err));
        }
        err = dev->set_control_line_state(dtr_state, rts_state);
        if (err != ESP_OK && err != ESP_ERR_NOT_SUPPORTED) {
            ESP_LOGW(TAG, "set_control_line_state failed for %d baud with DTR=%d RTS=%d: %s",
                     baud_rate, dtr_state, rts_state, esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(250));

        ESP_LOGD(TAG, "Running YRM100 setup sequence at %d baud with DTR=%d RTS=%d", baud_rate, dtr_state, rts_state);
        for (const ProbeCommand &cmd : setup_sequence) {
            const size_t before_frames = rx_state.valid_frames;
            err = sendReaderFrame(dev, cmd.name, cmd.frame, 2000);
            if (err != ESP_OK) {
                continue;
            }

            vTaskDelay(pdMS_TO_TICKS(cmd.settle_ms));
            const bool got_frame = waitForNewFrame(&rx_state, before_frames, cmd.wait_ms);
            if (got_frame) {
                got_response = true;
            } else if (cmd.expect_response) {
                ESP_LOGW(TAG, "No response frame after %s (baud=%d)", cmd.name, baud_rate);
            }
        }

        if (got_response) {
            break;
        }
    }

    processReaderStream(&rx_state);
    if (!got_response && rx_state.valid_frames == 0) {
        ESP_LOGW(TAG, "No response bytes received from reader across the tested baud rates");
    }

    // Align with vendor demo behavior: start multi-read once and parse async
    // inventory notifications, instead of spamming single-inventory commands.
    const esp_err_t read_multi_err =
        sendReaderFrame(dev, "ReadMulti", buildReadMultiFrame(kReadMultiLoopCount), 2000);
    if (read_multi_err != ESP_OK) {
        ESP_LOGW(TAG, "ReadMulti command failed: %s; fallback to single inventory polling",
                 esp_err_to_name(read_multi_err));

        YRM100_STATUS_LOG("Waiting for reader frames on USB (single inventory mode)");
        while (true) {
            const esp_err_t inventory_err =
                sendReaderFrame(dev, "Inventory", buildInventoryFrame(), kInventoryTxTimeoutMs);
            if (inventory_err != ESP_OK) {
                ESP_LOGW(TAG, "Inventory command failed: %s", esp_err_to_name(inventory_err));
                vTaskDelay(pdMS_TO_TICKS(kInventoryErrorBackoffMs));
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(kInventoryPollIntervalMs));
        }
    }

    YRM100_STATUS_LOG("Waiting for reader frames on USB (multi inventory mode)");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void Yrm100Reader::timeUpdateTask(void *pvParameters) {
    (void)pvParameters;
    while (true) {
        if (s_time_service != nullptr) {
            s_cached_time_src = s_time_service->getCurrentTimeHHMMSS(s_cached_time, sizeof(s_cached_time));
            const char *src_name = (s_cached_time_src == TimeSource::kGps) ? "GPS"
                                 : (s_cached_time_src == TimeSource::kNtp) ? "NTP"
                                 : (s_cached_time_src == TimeSource::kGsm) ? "GSM"
                                 : "fallback";
            YRM100_STATUS_LOG("time=%s [%s]", s_cached_time, src_name);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void Yrm100Reader::setTimeService(TimeService *ts) {
    s_time_service = ts;
}

esp_err_t Yrm100Reader::start(MqttPublisher *mqtt_publisher) {
    s_mqtt_publisher = mqtt_publisher;

    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .root_port_unpowered = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
        .enum_filter_cb = nullptr,
        .fifo_settings_custom = {
            .nptx_fifo_lines = 0,
            .ptx_fifo_lines = 0,
            .rx_fifo_lines = 0,
        },
        .peripheral_map = 0,
    };

    esp_err_t ret = usb_host_install(&host_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (!app_config::kEnableRfidReader) {
        YRM100_STATUS_LOG("RFID reader and VCP client disabled");
        xTaskCreatePinnedToCore(usbLibDaemonTask, "usb_daemon", 4096, nullptr, 10, nullptr, 0);
        xTaskCreatePinnedToCore(timeUpdateTask, "time_update", 4096, nullptr, 2, nullptr, 0);
        return ESP_OK;
    }

    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 16,
        .flags = {
            .notify_dev_removed = 0,
            .reserved31 = 0,
        },
        .async = {
            .client_event_callback = usbEventCallback,
            .callback_arg = nullptr,
        },
    };

    ret = usb_host_client_register(&client_config, &s_client_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_client_register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    xTaskCreatePinnedToCore(usbLibDaemonTask, "usb_daemon", 4096, nullptr, 10, nullptr, 0);
    xTaskCreatePinnedToCore(rfidTask, "yrm100_task", 8192, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(timeUpdateTask, "time_update", 4096, nullptr, 2, nullptr, 0);
    return ESP_OK;
}
