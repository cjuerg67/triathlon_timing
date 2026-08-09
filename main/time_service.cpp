#include "time_service.hpp"
#include "app_config.hpp"

#include <cstdio>
#include <cstring>
#include <ctime>
#include <cctype>
#include <cstdint>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sys/time.h"

namespace {
static const char *TAG = "time_service";
static constexpr int kUartRxBufferSize = 2048;
static constexpr int kUartTxBufferSize = 512;

static int64_t daysFromCivil(int year, unsigned month, unsigned day) {
    year -= month <= 2;
    const int era = (year >= 0 ? year : year - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(year - era * 400);
    const unsigned doy = (153U * (month + (month > 2 ? -3U : 9U)) + 2U) / 5U + day - 1U;
    const unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
    return static_cast<int64_t>(era) * 146097 + static_cast<int64_t>(doe) - 719468;
}
}

TimeService::TimeService() {
    const uart_config_t uart_cfg = {
        .baud_rate = app_config::kSim7000UartBaudRate,
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

    if (uart_set_pin(static_cast<uart_port_t>(uart_port_),
                     app_config::kSim7000UartTxPin,
                     app_config::kSim7000UartRxPin,
                     UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGW(TAG, "SIM7000 UART pin config failed");
        return;
    }

    uart_initialized_ = true;

    setenv("TZ", app_config::kTimezone, 1);
    tzset();
    ESP_LOGI(TAG, "Timezone set to %s", app_config::kTimezone);
}

bool TimeService::applyUtcClock(int year, int month, int day, int hour, int minute, int second) const {
    if (year < 1970 || month < 1 || month > 12 || day < 1 || day > 31 ||
        hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 60) {
        return false;
    }

    const int64_t days = daysFromCivil(year, static_cast<unsigned>(month), static_cast<unsigned>(day));
    const int64_t epoch = days * 86400LL + static_cast<int64_t>(hour) * 3600LL + static_cast<int64_t>(minute) * 60LL + second;
    if (epoch < 0) {
        return false;
    }

    const timeval tv = {
        .tv_sec = static_cast<time_t>(epoch),
        .tv_usec = 0,
    };
    if (settimeofday(&tv, nullptr) != 0) {
        ESP_LOGW(TAG, "settimeofday failed");
        return false;
    }

    ESP_LOGI(TAG, "System clock set to %04d-%02d-%02d %02d:%02d:%02d UTC", year, month, day, hour, minute, second);
    return true;
}

void TimeService::initNtp() {
    if (ntp_initialized_) {
        return;
    }
    ESP_LOGI(TAG, "Starting NTP client, server: %s", app_config::kNtpServer);
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, app_config::kNtpServer);
    esp_sntp_init();
    ntp_initialized_ = true;
    ESP_LOGI(TAG, "NTP client started");
}

TimeSource TimeService::getCurrentTimeHHMMSS(char *out, size_t out_size) const {
    if (out == nullptr || out_size < 9) {
        return TimeSource::kFallback;
    }

    int hour = 0;
    int minute = 0;
    int second = 0;

    if (tryGetNtpTime(hour, minute, second)) {
        std::snprintf(out, out_size, "%02d:%02d:%02d", hour, minute, second);
        return TimeSource::kNtp;
    }

    if (tryGetGsmNetworkTime(hour, minute, second)) {
        const std::time_t now = std::time(nullptr);
        const std::tm *tm_now = std::localtime(&now);
        if (tm_now != nullptr) {
            std::snprintf(out, out_size, "%02d:%02d:%02d", tm_now->tm_hour, tm_now->tm_min, tm_now->tm_sec);
        } else {
            std::snprintf(out, out_size, "%02d:%02d:%02d", hour, minute, second);
        }
        return TimeSource::kGsm;
    }

    if (tryGetGpsTime(hour, minute, second)) {
        const std::time_t now = std::time(nullptr);
        const std::tm *tm_now = std::localtime(&now);
        if (tm_now != nullptr) {
            std::snprintf(out, out_size, "%02d:%02d:%02d", tm_now->tm_hour, tm_now->tm_min, tm_now->tm_sec);
        } else {
            std::snprintf(out, out_size, "%02d:%02d:%02d", hour, minute, second);
        }
        return TimeSource::kGps;
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

bool TimeService::parseGsmClock(const char *line,
                                int &year,
                                int &month,
                                int &day,
                                int &hour,
                                int &minute,
                                int &second,
                                int &utc_offset_quarters) const {
    if (line == nullptr) {
        return false;
    }

    int yy = 0;
    int tz = 0;
    int matched = std::sscanf(line, "+CCLK: \"%2d/%2d/%2d,%2d:%2d:%2d%2d\"", &yy, &month, &day, &hour, &minute, &second, &tz);
    if (matched < 6) {
        return false;
    }

    year = 2000 + yy;
    utc_offset_quarters = tz;
    return true;
}

bool TimeService::parseGnsUtc(const char *line, int &year, int &month, int &day, int &hour, int &minute, int &second) const {
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

    if (!std::isdigit(static_cast<unsigned char>(utc[0])) ||
        !std::isdigit(static_cast<unsigned char>(utc[1])) ||
        !std::isdigit(static_cast<unsigned char>(utc[2])) ||
        !std::isdigit(static_cast<unsigned char>(utc[3])) ||
        !std::isdigit(static_cast<unsigned char>(utc[4])) ||
        !std::isdigit(static_cast<unsigned char>(utc[5])) ||
        !std::isdigit(static_cast<unsigned char>(utc[6])) ||
        !std::isdigit(static_cast<unsigned char>(utc[7])) ||
        !std::isdigit(static_cast<unsigned char>(utc[8])) ||
        !std::isdigit(static_cast<unsigned char>(utc[9])) ||
        !std::isdigit(static_cast<unsigned char>(utc[10])) ||
        !std::isdigit(static_cast<unsigned char>(utc[11])) ||
        !std::isdigit(static_cast<unsigned char>(utc[12])) ||
        !std::isdigit(static_cast<unsigned char>(utc[13]))) {
        return false;
    }

    year = (utc[0] - '0') * 1000 + (utc[1] - '0') * 100 + (utc[2] - '0') * 10 + (utc[3] - '0');
    month = (utc[4] - '0') * 10 + (utc[5] - '0');
    day = (utc[6] - '0') * 10 + (utc[7] - '0');
    hour = (utc[8] - '0') * 10 + (utc[9] - '0');
    minute = (utc[10] - '0') * 10 + (utc[11] - '0');
    second = (utc[12] - '0') * 10 + (utc[13] - '0');
    return true;
}

bool TimeService::tryGetNtpTime(int &hour, int &minute, int &second) const {
    if (!ntp_initialized_) {
        return false;
    }
    if (!ntp_synced_) {
        const sntp_sync_status_t status = esp_sntp_get_sync_status();
        if (status != SNTP_SYNC_STATUS_COMPLETED) {
            ESP_LOGD(TAG, "NTP sync pending (status=%d)", static_cast<int>(status));
            return false;
        }
        ntp_synced_ = true;
        ESP_LOGI(TAG, "NTP sync completed");
    }
    const std::time_t now = std::time(nullptr);
    const std::tm *tm_now = std::localtime(&now);
    if (tm_now == nullptr || (tm_now->tm_year + 1900) < 2024) {
        ESP_LOGW(TAG, "NTP synced but system time looks invalid");
        return false;
    }
    ESP_LOGD(TAG, "NTP time: %02d:%02d:%02d local", tm_now->tm_hour, tm_now->tm_min, tm_now->tm_sec);
    hour = tm_now->tm_hour;
    minute = tm_now->tm_min;
    second = tm_now->tm_sec;
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

    int year = 0;
    int month = 0;
    int day = 0;
    if (!parseGnsUtc(line, year, month, day, hour, minute, second)) {
        return false;
    }

    if (!applyUtcClock(year, month, day, hour, minute, second)) {
        return false;
    }

    return true;
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

    int year = 0;
    int month = 0;
    int day = 0;
    int utc_offset_quarters = 0;
    if (!parseGsmClock(line, year, month, day, hour, minute, second, utc_offset_quarters)) {
        return false;
    }

    const int offset_seconds = utc_offset_quarters * 15 * 60;
    int64_t epoch = daysFromCivil(year, static_cast<unsigned>(month), static_cast<unsigned>(day)) * 86400LL
                  + static_cast<int64_t>(hour) * 3600LL
                  + static_cast<int64_t>(minute) * 60LL
                  + second;
    epoch -= offset_seconds;
    if (epoch < 0) {
        return false;
    }

    const int utc_year = 1970; // placeholder for logging below
    (void)utc_year;

    const int utc_hour = static_cast<int>((epoch % 86400LL) / 3600LL);
    const int utc_minute = static_cast<int>((epoch % 3600LL) / 60LL);
    const int utc_second = static_cast<int>(epoch % 60LL);

    // Reconstruct UTC date from epoch for settimeofday.
    // Convert using localtime on a UTC TZ-less representation by setting the clock directly.
    const timeval tv = {
        .tv_sec = static_cast<time_t>(epoch),
        .tv_usec = 0,
    };
    if (settimeofday(&tv, nullptr) != 0) {
        ESP_LOGW(TAG, "settimeofday failed for GSM time");
        return false;
    }

    ESP_LOGI(TAG, "System clock set from GSM time %04d-%02d-%02d %02d:%02d:%02d UTC", year, month, day, utc_hour, utc_minute, utc_second);
    return true;
}
