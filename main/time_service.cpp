#include "time_service.hpp"

#include <cstdio>
#include <cstring>
#include <ctime>
#include <cctype>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
static const char *TAG = "time_service";
static constexpr int kGsmRxPin = 22;
static constexpr int kGsmTxPin = 23;
static constexpr int kUartRxBufferSize = 2048;
static constexpr int kUartTxBufferSize = 512;
}

TimeService::TimeService() {
    const uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {
            .allow_pd = 0,
            .backup_before_sleep = 0,
        },
    };

    if (uart_driver_install(static_cast<uart_port_t>(uart_port_), kUartRxBufferSize, kUartTxBufferSize, 0, nullptr, 0) != ESP_OK) {
        ESP_LOGW(TAG, "SIM7000 UART driver install failed");
        return;
    }

    if (uart_param_config(static_cast<uart_port_t>(uart_port_), &uart_cfg) != ESP_OK) {
        ESP_LOGW(TAG, "SIM7000 UART config failed");
        return;
    }

    if (uart_set_pin(static_cast<uart_port_t>(uart_port_), kGsmTxPin, kGsmRxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGW(TAG, "SIM7000 UART pin config failed");
        return;
    }

    uart_initialized_ = true;
}

TimeSource TimeService::getCurrentTimeHHMMSS(char *out, size_t out_size) const {
    if (out == nullptr || out_size < 9) {
        return TimeSource::kFallback;
    }

    int hour = 0;
    int minute = 0;
    int second = 0;

    if (tryGetGpsTime(hour, minute, second)) {
        std::snprintf(out, out_size, "%02d:%02d:%02d", hour, minute, second);
        return TimeSource::kGps;
    }

    if (tryGetGsmNetworkTime(hour, minute, second)) {
        std::snprintf(out, out_size, "%02d:%02d:%02d", hour, minute, second);
        return TimeSource::kGsm;
    }

    const std::time_t now = std::time(nullptr);
    const std::tm *tm_now = std::localtime(&now);
    if (tm_now != nullptr && (tm_now->tm_year + 1900) >= 2024) {
        std::snprintf(out, out_size, "%02d:%02d:%02d", tm_now->tm_hour, tm_now->tm_min, tm_now->tm_sec);
    } else {
        std::snprintf(out, out_size, "00:00:00");
    }
    return TimeSource::kFallback;
}

bool TimeService::queryAtLine(const char *command, char *line_out, size_t line_out_size, uint32_t timeout_ms) const {
    if (!uart_initialized_ || command == nullptr || line_out == nullptr || line_out_size < 2) {
        return false;
    }

    uart_port_t port = static_cast<uart_port_t>(uart_port_);
    uart_flush(port);
    (void)uart_write_bytes(port, command, std::strlen(command));
    (void)uart_write_bytes(port, "\r\n", 2);

    line_out[0] = '\0';

    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    char rx[384] = {0};
    size_t used = 0;

    while (xTaskGetTickCount() < deadline && used < (sizeof(rx) - 1)) {
        uint8_t ch = 0;
        int read = uart_read_bytes(port, &ch, 1, pdMS_TO_TICKS(50));
        if (read <= 0) {
            continue;
        }

        if (ch == '\r') {
            continue;
        }

        if (ch == '\n') {
            if (used == 0) {
                continue;
            }

            rx[used] = '\0';
            if (std::strcmp(rx, "OK") == 0 || std::strcmp(rx, "ERROR") == 0) {
                used = 0;
                continue;
            }

            std::snprintf(line_out, line_out_size, "%s", rx);
            return true;
        }

        rx[used++] = static_cast<char>(ch);
    }

    return false;
}

bool TimeService::parseGsmClock(const char *line, int &hour, int &minute, int &second) const {
    if (line == nullptr) {
        return false;
    }

    int yy = 0;
    int mon = 0;
    int day = 0;
    int tz = 0;
    int matched = std::sscanf(line, "+CCLK: \"%2d/%2d/%2d,%2d:%2d:%2d%2d\"", &yy, &mon, &day, &hour, &minute, &second, &tz);
    return matched >= 6;
}

bool TimeService::parseGnsUtc(const char *line, int &hour, int &minute, int &second) const {
    if (line == nullptr) {
        return false;
    }

    const char *utc = std::strstr(line, ",");
    if (utc == nullptr) {
        return false;
    }
    ++utc;

    // Expected UTC field format from +CGNSINF: YYYYMMDDhhmmss.sss
    if (std::strlen(utc) < 14) {
        return false;
    }

    if (!std::isdigit(static_cast<unsigned char>(utc[8])) ||
        !std::isdigit(static_cast<unsigned char>(utc[9])) ||
        !std::isdigit(static_cast<unsigned char>(utc[10])) ||
        !std::isdigit(static_cast<unsigned char>(utc[11])) ||
        !std::isdigit(static_cast<unsigned char>(utc[12])) ||
        !std::isdigit(static_cast<unsigned char>(utc[13]))) {
        return false;
    }

    hour = (utc[8] - '0') * 10 + (utc[9] - '0');
    minute = (utc[10] - '0') * 10 + (utc[11] - '0');
    second = (utc[12] - '0') * 10 + (utc[13] - '0');
    return true;
}

bool TimeService::tryGetGpsTime(int &hour, int &minute, int &second) const {
    if (!uart_initialized_) {
        return false;
    }

    char line[256] = {0};
    if (!gps_power_enabled_) {
        // One-time GPS power request; if this fails, we'll still try reading anyway.
        (void)queryAtLine("AT+CGNSPWR=1", line, sizeof(line), 1200);
        gps_power_enabled_ = true;
    }

    if (!queryAtLine("AT+CGNSINF", line, sizeof(line), 2000)) {
        return false;
    }

    if (std::strstr(line, "+CGNSINF:") == nullptr) {
        return false;
    }

    return parseGnsUtc(line, hour, minute, second);
}

bool TimeService::tryGetGsmNetworkTime(int &hour, int &minute, int &second) const {
    if (!uart_initialized_) {
        return false;
    }

    char line[128] = {0};
    if (!queryAtLine("AT+CCLK?", line, sizeof(line), 1500)) {
        return false;
    }

    if (std::strstr(line, "+CCLK:") == nullptr) {
        return false;
    }

    return parseGsmClock(line, hour, minute, second);
}
