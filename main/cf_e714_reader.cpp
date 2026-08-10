#include "cf_e714_reader.hpp"

#include <cstdio>
#include <cstring>
#include <ctime>

#include "app_config.hpp"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rfid_deduplicator.hpp"
#include "rfid_event.hpp"

namespace {
static const char *TAG = "cf_e714_reader";

static constexpr uint8_t kComAddr = 0xFF;  // broadcast address
static constexpr uint8_t kCmdInventory = 0x01;
static constexpr uint8_t kStatusOk = 0x01;
static constexpr int kRxBufSize = 1024;
static constexpr uint32_t kResponseTimeoutMs = 2000;

static RfidDeduplicator s_dedup(app_config::kRfidDuplicateDebounceMs);

// CRC-16/CCITT (poly 0x8408, init 0xFFFF) — used by CHAFON UHFReader288 protocol
static uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            crc = (crc & 1u) ? ((crc >> 1) ^ 0x8408u) : (crc >> 1);
        }
    }
    return crc;
}

// Build Inventory_G2 command frame into buf, returns total frame length.
// Frame: [Len][ComAdr][0x01][QValue=4][Session=0][MaskMem=0][MaskAdr=0,0]
//        [MaskLen=0][MaskFlag=0][AdrTID=0][LenTID=0][TIDFlag=0][Target=0]
//        [InAnt=0][Scantime][FastFlag=0][CRC_L][CRC_H]
static size_t buildInventoryCmd(uint8_t *buf, size_t buf_size, uint8_t scantime_100ms) {
    static constexpr size_t kFrameLen = 19;
    if (buf_size < kFrameLen) return 0;

    // 16 data bytes (ComAdr through FastFlag)
    uint8_t data[16] = {
        kComAddr,       // ComAdr
        kCmdInventory,  // Cmd
        0x04,           // QValue = 4
        0x00,           // Session = 0
        0x00,           // MaskMem
        0x00, 0x00,     // MaskAdr
        0x00,           // MaskLen = 0 (no mask data)
        0x00,           // MaskFlag
        0x00,           // AdrTID
        0x00,           // LenTID
        0x00,           // TIDFlag = 0 (EPC only)
        0x00,           // Target
        0x00,           // InAnt = 0
        scantime_100ms, // Scantime
        0x00,           // FastFlag = 0
    };
    const uint16_t crc = crc16(data, sizeof(data));

    buf[0] = 18;  // Len = 16 data bytes + 2 CRC bytes
    std::memcpy(&buf[1], data, 16);
    buf[17] = static_cast<uint8_t>(crc & 0xFF);
    buf[18] = static_cast<uint8_t>(crc >> 8);
    return kFrameLen;
}

// Read exactly 'count' bytes from UART with a total deadline.
static bool uartReadExact(uart_port_t port, uint8_t *dst, size_t count, TickType_t deadline) {
    size_t received = 0;
    while (received < count) {
        const TickType_t now = xTaskGetTickCount();
        if (now >= deadline) return false;
        const TickType_t remaining = deadline - now;
        const int n = uart_read_bytes(port, dst + received, count - received,
                                      remaining < pdMS_TO_TICKS(20) ? remaining : pdMS_TO_TICKS(20));
        if (n > 0) received += static_cast<size_t>(n);
    }
    return true;
}

static bool getSystemTimeHhMmSs(char *out, size_t out_size) {
    if (out == nullptr || out_size < 9) {
        return false;
    }

    const std::time_t now = std::time(nullptr);
    std::tm tm_now = {};
    if (now <= 0 || localtime_r(&now, &tm_now) == nullptr || (tm_now.tm_year + 1900) < 2024) {
        return false;
    }

    std::snprintf(out, out_size, "%02d:%02d:%02d", tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);
    return true;
}

// Parse the EPC list from a validated inventory response and publish each tag.
// 'payload' starts at Status byte; 'payload_len' is the count of bytes from Status to end of CRC.
static void processInventoryResponse(const uint8_t *payload, size_t payload_len,
                                     MqttPublisher *publisher,
                                     const char *cached_time, TimeSource cached_src) {
    if (payload_len < 4) return; // need at least: Status(1) + CardNum(1) + CRC(2)
    if (payload[0] != kStatusOk) return;

    const size_t epc_list_end = payload_len - 3; // exclude Status(1) + CardNum(1) + CRC(2) = last 3 bytes before CRC
    // layout after Status: [EPCLen][EPC...][RSSI][Ant] per tag, then [CardNum][CRC_L][CRC_H]
    // epc_list runs from index 1 to payload_len-4 (inclusive)
    size_t pos = 1; // skip Status byte
    while (pos < epc_list_end) {
        const uint8_t epc_len = payload[pos];
        if (epc_len == 0 || pos + epc_len + 2 > epc_list_end) break; // +RSSI+Ant

        const uint8_t *epc = &payload[pos + 1];
        const uint8_t rssi = payload[pos + 1 + epc_len];
        pos += 1 + epc_len + 2; // EPCLen + EPC + RSSI + Ant

        // Format EPC as hex string
        char epc_hex[65] = {0};
        for (uint8_t k = 0; k < epc_len && k < 32; ++k) {
            std::snprintf(epc_hex + k * 2, sizeof(epc_hex) - k * 2, "%02X", epc[k]);
        }
        if (epc_hex[0] == '\0') continue;

        // Deduplication
        const uint32_t now_ms = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
        RfidEvent evt = {};
        std::snprintf(evt.rfid_id, sizeof(evt.rfid_id), "%s", epc_hex);
        std::snprintf(evt.reader_id, sizeof(evt.reader_id), "cf_e714");
        evt.tick_ms = now_ms;
        if (s_dedup.shouldDrop(evt, now_ms)) continue;

        // Timestamp
        char time_buf[16];
        if (cached_src == TimeSource::kFallback) {
            if (!getSystemTimeHhMmSs(time_buf, sizeof(time_buf))) {
                std::snprintf(time_buf, sizeof(time_buf), "%lu", static_cast<unsigned long>(now_ms));
            }
        } else {
            std::snprintf(time_buf, sizeof(time_buf), "%s", cached_time);
        }

        ESP_LOGI(TAG, "epc=%s rssi=%d time=%s", epc_hex, rssi, time_buf);
        if (publisher != nullptr && publisher->isConnected()) {
            publisher->publishTag(epc_hex, time_buf);
        }
    }
}
} // namespace

MqttPublisher *CfE714Reader::s_mqtt_publisher = nullptr;
TimeService *CfE714Reader::s_time_service = nullptr;
char CfE714Reader::s_cached_time[16] = "00:00:00";
TimeSource CfE714Reader::s_cached_time_src = TimeSource::kFallback;

void CfE714Reader::setTimeService(TimeService *ts) {
    s_time_service = ts;
}

void CfE714Reader::readerTask(void *pvParameters) {
    (void)pvParameters;

    const uart_port_t port = static_cast<uart_port_t>(app_config::kCfE714UartPort);
    const uart_config_t uart_cfg = {
        .baud_rate = app_config::kCfE714BaudRate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {.allow_pd = 0, .backup_before_sleep = 0},
    };

    if (uart_driver_install(port, kRxBufSize, 256, 0, nullptr, 0) != ESP_OK ||
        uart_param_config(port, &uart_cfg) != ESP_OK ||
        uart_set_pin(port,
                     app_config::kCfE714TxPin,
                     app_config::kCfE714RxPin,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "CF-E714 UART init failed");
        vTaskDelete(nullptr);
        return;
    }
    ESP_LOGI(TAG, "CF-E714 reader started on UART%d baud=%d",
             app_config::kCfE714UartPort, app_config::kCfE714BaudRate);

    uint8_t cmd[19];
    uint8_t resp[256];

    while (true) {
        // Update cached time once per loop
        if (s_time_service != nullptr) {
            s_cached_time_src = s_time_service->getCurrentTimeHHMMSS(s_cached_time, sizeof(s_cached_time));
        }

        // Send inventory command (Scantime=1 → 100 ms scan)
        const size_t cmd_len = buildInventoryCmd(cmd, sizeof(cmd), 1);
        uart_flush(port);
        uart_write_bytes(port, cmd, cmd_len);

        // Read response: first byte is Len
        const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(kResponseTimeoutMs);
        if (!uartReadExact(port, resp, 1, deadline)) continue;

        const uint8_t len = resp[0];
        if (len < 6 || len > static_cast<uint8_t>(sizeof(resp) - 1)) continue;

        if (!uartReadExact(port, resp + 1, len, deadline)) continue;

        // Verify CRC: covers bytes [1..len-2] (ComAdr through data, excluding Len and CRC)
        const uint16_t rx_crc = static_cast<uint16_t>(resp[len - 1]) |
                                 (static_cast<uint16_t>(resp[len]) << 8);
        const uint16_t calc_crc = crc16(&resp[1], len - 2);
        if (rx_crc != calc_crc) {
            ESP_LOGW(TAG, "CRC mismatch: expected 0x%04X got 0x%04X", calc_crc, rx_crc);
            continue;
        }

        // resp[2] = Cmd, resp[3] = Status; EPC payload starts at resp[3]
        if (resp[2] != kCmdInventory) continue;

        // payload: Status(1) + EPC_list + CardNum(1) + CRC(2) → length = len - 2 (ComAdr + Cmd)
        processInventoryResponse(&resp[3], static_cast<size_t>(len) - 2,
                                 s_mqtt_publisher, s_cached_time, s_cached_time_src);

        vTaskDelay(pdMS_TO_TICKS(50)); // brief yield before next poll
    }
}

esp_err_t CfE714Reader::start(MqttPublisher *mqtt_publisher) {
    s_mqtt_publisher = mqtt_publisher;
    xTaskCreatePinnedToCore(readerTask, "cf_e714_task", 4096, nullptr, 5, nullptr, 1);
    return ESP_OK;
}
