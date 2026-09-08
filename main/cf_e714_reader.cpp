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

// Send a configuration command and read the response.
// Returns true if the command succeeded (Status = 0x00).
static bool sendConfigCommand(uart_port_t port, const uint8_t *cmd, size_t cmd_len,
                              const char *cmd_name) {
    // Flush and give reader time to settle
    uart_flush(port);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Log the command bytes being sent
    ESP_LOGI(TAG, "%s: sending %zu bytes:", cmd_name, cmd_len);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, cmd, cmd_len, ESP_LOG_INFO);
    
    const int written = uart_write_bytes(port, cmd, cmd_len);
    if (written < 0) {
        ESP_LOGW(TAG, "%s: uart_write_bytes failed", cmd_name);
        return false;
    }
    
    // Wait for transmission to complete
    uart_wait_tx_done(port, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(50));

    // Check if ANY bytes are available
    size_t available = 0;
    uart_get_buffered_data_len(port, &available);
    ESP_LOGI(TAG, "%s: %zu bytes available in RX buffer after TX", cmd_name, available);

    uint8_t resp[16];
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
    if (!uartReadExact(port, resp, 1, deadline)) {
        // Check again if data arrived late
        uart_get_buffered_data_len(port, &available);
        ESP_LOGW(TAG, "%s: no response (timeout reading Len), %zu bytes in buffer", cmd_name, available);
        
        // Dump any garbage that might be in buffer
        if (available > 0) {
            uint8_t garbage[32];
            const size_t to_read = available < sizeof(garbage) ? available : sizeof(garbage);
            const int n = uart_read_bytes(port, garbage, to_read, pdMS_TO_TICKS(100));
            if (n > 0) {
                ESP_LOGW(TAG, "%s: received unexpected data:", cmd_name);
                ESP_LOG_BUFFER_HEX_LEVEL(TAG, garbage, n, ESP_LOG_WARN);
            }
        }
        return false;
    }

    const uint8_t len = resp[0];
    ESP_LOGI(TAG, "%s: received Len byte = 0x%02X", cmd_name, len);
    if (len < 5 || len > 15) {
        ESP_LOGW(TAG, "%s: invalid response length %d", cmd_name, len);
        return false;
    }

    if (!uartReadExact(port, resp + 1, len, deadline)) {
        ESP_LOGW(TAG, "%s: timeout reading response body", cmd_name);
        return false;
    }

    // Verify CRC
    const uint16_t rx_crc = static_cast<uint16_t>(resp[len - 1]) |
                             (static_cast<uint16_t>(resp[len]) << 8);
    const uint16_t calc_crc = crc16(&resp[1], len - 2);
    if (rx_crc != calc_crc) {
        ESP_LOGW(TAG, "%s: CRC mismatch (rx=0x%04X calc=0x%04X)", cmd_name, rx_crc, calc_crc);
        return false;
    }

    // Check status byte (resp[3])
    if (resp[3] != 0x00) {
        ESP_LOGW(TAG, "%s: command failed with status 0x%02X", cmd_name, resp[3]);
        return false;
    }

    ESP_LOGI(TAG, "%s: success", cmd_name);
    return true;
}

// Configure CF-E714 RF settings using the CHAFON UHFReader288 protocol.
// Commands: 0x22 (region/frequency), 0x2f (RF power), 0x3f (antenna multiplexing).
static bool configureReaderDefaults(uart_port_t port) {
    const int country_code = app_config::kCfE714CountryCode;
    const int output_power_dbm = app_config::kCfE714OutputPowerDbm;
    const int antenna_port = app_config::kCfE714AntennaPort;

    if (country_code == 0 && output_power_dbm == 0 && antenna_port == 0) {
        ESP_LOGI(TAG, "CF-E714 RF settings left at factory defaults (all zero)");
        return true;
    }

    ESP_LOGI(TAG, "Configuring CF-E714: region=%d power=%d dBm antenna=0x%02X",
             country_code, output_power_dbm, antenna_port);

    bool all_ok = true;

    // Command 0x22: Modify working frequency (region)
    // Frame: [Len][ComAdr][0x22][MaxFre][MinFre][CRC_L][CRC_H]
    // For EU band (code 4): MaxFre=0x8E (bit7-6=01, bit5-0=14), MinFre=0x40 (bit7-6=01, bit5-0=0)
    if (country_code != 0) {
        const uint8_t max_fre = static_cast<uint8_t>((country_code << 6) | 0x0E); // EU: band 4, max point 14
        const uint8_t min_fre = static_cast<uint8_t>((country_code << 6) | 0x00); // EU: band 4, min point 0
        uint8_t data[4] = {kComAddr, 0x22, max_fre, min_fre};
        const uint16_t crc = crc16(data, 4);
        uint8_t cmd[7] = {0x06, data[0], data[1], data[2], data[3],
                          static_cast<uint8_t>(crc & 0xFF), static_cast<uint8_t>(crc >> 8)};
        if (!sendConfigCommand(port, cmd, sizeof(cmd), "SetRegion")) {
            all_ok = false;
        }
    }

    // Command 0x2f: Modify RF power
    // Frame: [Len][ComAdr][0x2f][Pwr][CRC_L][CRC_H]
    // Pwr: 0-30 (30 = ~1W)
    if (output_power_dbm > 0) {
        const uint8_t pwr = static_cast<uint8_t>(output_power_dbm > 30 ? 30 : output_power_dbm);
        uint8_t data[3] = {kComAddr, 0x2f, pwr};
        const uint16_t crc = crc16(data, 3);
        uint8_t cmd[6] = {0x05, data[0], data[1], data[2],
                          static_cast<uint8_t>(crc & 0xFF), static_cast<uint8_t>(crc >> 8)};
        if (!sendConfigCommand(port, cmd, sizeof(cmd), "SetRfPower")) {
            all_ok = false;
        }
    }

    // Command 0x3f: Setup antenna multiplexing (Format 1)
    // Frame: [Len][ComAdr][0x3f][Ant][CRC_L][CRC_H]
    // Ant: bit0=ant1, bit1=ant2, bit2=ant3, bit3=ant4, etc.
    if (antenna_port != 0) {
        uint8_t data[3] = {kComAddr, 0x3f, static_cast<uint8_t>(antenna_port)};
        const uint16_t crc = crc16(data, 3);
        uint8_t cmd[6] = {0x05, data[0], data[1], data[2],
                          static_cast<uint8_t>(crc & 0xFF), static_cast<uint8_t>(crc >> 8)};
        if (!sendConfigCommand(port, cmd, sizeof(cmd), "SetAntenna")) {
            all_ok = false;
        }
    }

    return all_ok;
}

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
        .flags = {},
    };

    // Official ESP-IDF pattern: install -> config -> pins
    // Reference: examples/peripherals/uart/uart_async_rxtxtasks
    if (uart_driver_install(port, kRxBufSize, 256, 0, nullptr, 0) != ESP_OK) {
        ESP_LOGE(TAG, "CF-E714 UART driver install failed");
        vTaskDelete(nullptr);
        return;
    }

    if (uart_param_config(port, &uart_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "CF-E714 UART param config failed");
        vTaskDelete(nullptr);
        return;
    }

    if (uart_set_pin(port,
                     app_config::kCfE714TxPin,
                     app_config::kCfE714RxPin,
                     UART_PIN_NO_CHANGE, 
                     UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "CF-E714 UART pin config failed");
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG, "CF-E714 reader started on UART%d baud=%d TX=%d RX=%d",
             app_config::kCfE714UartPort, app_config::kCfE714BaudRate,
             app_config::kCfE714TxPin, app_config::kCfE714RxPin);

    // Optional: UART loopback self-test (requires TX+RX shorted together temporarily)
    // Uncomment to verify UART hardware is working:
    /*
    ESP_LOGI(TAG, "UART loopback test (TX and RX must be connected)...");
    uart_flush(port);
    const char *test_msg = "TEST";
    uart_write_bytes(port, test_msg, 4);
    vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t loopback_buf[8];
    int loopback_read = uart_read_bytes(port, loopback_buf, 4, pdMS_TO_TICKS(500));
    if (loopback_read == 4 && memcmp(loopback_buf, test_msg, 4) == 0) {
        ESP_LOGI(TAG, "UART loopback test PASSED - UART hardware is working");
    } else {
        ESP_LOGE(TAG, "UART loopback test FAILED - read %d bytes", loopback_read);
    }
    uart_flush(port);
    */

    // Wait for reader hardware to stabilize after power-on/UART init
    ESP_LOGI(TAG, "Waiting 2s for CF-E714 reader to boot...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Flush any garbage data from buffer
    uart_flush(port);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Try a simple command first to verify communication
    ESP_LOGI(TAG, "Testing CF-E714 communication with GetReaderInfo command (0x21)...");
    {
        uint8_t data[2] = {kComAddr, 0x21};  // 0x21 = Get Reader Info
        const uint16_t crc = crc16(data, 2);
        uint8_t test_cmd[5] = {0x04, data[0], data[1],
                               static_cast<uint8_t>(crc & 0xFF), 
                               static_cast<uint8_t>(crc >> 8)};
        
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, test_cmd, sizeof(test_cmd), ESP_LOG_INFO);
        uart_write_bytes(port, test_cmd, sizeof(test_cmd));
        uart_wait_tx_done(port, pdMS_TO_TICKS(100));
        vTaskDelay(pdMS_TO_TICKS(200));
        
        size_t available = 0;
        uart_get_buffered_data_len(port, &available);
        ESP_LOGI(TAG, "GetReaderInfo: %zu bytes available in RX buffer", available);
        
        if (available > 0) {
            uint8_t test_resp[64];
            const int n = uart_read_bytes(port, test_resp, available < 64 ? available : 64, pdMS_TO_TICKS(200));
            if (n > 0) {
                ESP_LOGI(TAG, "GetReaderInfo response:");
                ESP_LOG_BUFFER_HEX_LEVEL(TAG, test_resp, n, ESP_LOG_INFO);
            }
        } else {
            ESP_LOGW(TAG, "No response to GetReaderInfo - reader may not be connected/powered");
        }
        
        uart_flush(port);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "Attempting CF-E714 RF configuration...");
    (void)configureReaderDefaults(port);

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
