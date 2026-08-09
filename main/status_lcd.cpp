#include "status_lcd.hpp"

#include <cstdio>
#include <cstring>

#include "app_config.hpp"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
static const char *TAG = "status_lcd";

constexpr uint8_t kRsMask = 0x01;
constexpr uint8_t kEnableMask = 0x04;
constexpr uint8_t kNibbleShift = 4;
constexpr size_t kLineChars = 16;

constexpr uint8_t kCmdClear = 0x01;
constexpr uint8_t kCmdEntryModeSet = 0x06;
constexpr uint8_t kCmdDisplayOn = 0x0C;
constexpr uint8_t kCmdFunctionSet = 0x28;

uint8_t lineAddress(uint8_t row) {
    return row == 0 ? 0x80 : 0xC0;
}
}

esp_err_t StatusLcd::init() {
    if (initialized_) {
        return ESP_OK;
    }

    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = static_cast<i2c_port_num_t>(app_config::kStatusLcdI2cPort),
        .sda_io_num = static_cast<gpio_num_t>(app_config::kStatusLcdSdaPin),
        .scl_io_num = static_cast<gpio_num_t>(app_config::kStatusLcdSclPin),
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
            .allow_pd = 0,
        },
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &bus_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(ret));
        return ret;
    }

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = app_config::kStatusLcdAddress,
        .scl_speed_hz = app_config::kStatusLcdI2cFrequencyHz,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };

    ret = i2c_master_bus_add_device(bus_, &dev_cfg, &dev_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    // HD44780 init sequence for 4-bit mode via expander.
    for (int i = 0; i < 3; ++i) {
        ret = write4Bit(0x03, false);
        if (ret != ESP_OK) {
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ret = write4Bit(0x02, false);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(1));

    ret = writeCommand(kCmdFunctionSet);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = writeCommand(kCmdDisplayOn);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = writeCommand(kCmdClear);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
    ret = writeCommand(kCmdEntryModeSet);
    if (ret != ESP_OK) {
        return ret;
    }

    initialized_ = true;
    renderLine(0, "Triathlon Timer");
    renderLine(1, "LCD ready");
    return ESP_OK;
}

void StatusLcd::update(TimeService &time_service,
                       TransportType transport,
                       bool transport_up,
                       bool mqtt_up,
                       const char *latest_tag) {
    if (!initialized_) {
        return;
    }

    char time_buf[16] = {0};
    const TimeSource source = time_service.getCurrentTimeHHMMSS(time_buf, sizeof(time_buf));

    char line1[17] = {0};
    std::snprintf(line1,
                  sizeof(line1),
                  "%8.8s%c %c%c%c",
                  time_buf,
                  timeSourceCode(source),
                  transportCode(transport),
                  transport_up ? '+' : '-',
                  mqtt_up ? 'M' : 'm');

    char tag_view[13] = {0};
    const char *tag = (latest_tag != nullptr) ? latest_tag : "";
    const size_t tag_len = std::strlen(tag);
    if (tag_len == 0) {
        std::snprintf(tag_view, sizeof(tag_view), "----");
        scroll_offset_ = 0;
    } else if (tag_len <= sizeof(tag_view) - 1) {
        std::snprintf(tag_view, sizeof(tag_view), "%s", tag);
        scroll_offset_ = 0;
    } else {
        const size_t visible = sizeof(tag_view) - 1;
        const size_t max_offset = tag_len - visible;
        if (scroll_offset_ > max_offset) {
            scroll_offset_ = 0;
        }
        std::memcpy(tag_view, tag + scroll_offset_, visible);
        tag_view[visible] = '\0';
        scroll_offset_++;
    }

    char line2[17] = {0};
    std::snprintf(line2, sizeof(line2), "TAG:%s", tag_view);

    renderLine(0, line1);
    renderLine(1, line2);
}

esp_err_t StatusLcd::writeCommand(uint8_t value) {
    esp_err_t ret = write4Bit((value >> 4) & 0x0F, false);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = write4Bit(value & 0x0F, false);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ret;
}

esp_err_t StatusLcd::writeData(uint8_t value) {
    esp_err_t ret = write4Bit((value >> 4) & 0x0F, true);
    if (ret != ESP_OK) {
        return ret;
    }
    return write4Bit(value & 0x0F, true);
}

esp_err_t StatusLcd::write4Bit(uint8_t value, bool rs) {
    uint8_t bus = static_cast<uint8_t>((value << kNibbleShift) | backlight_mask_);
    if (rs) {
        bus |= kRsMask;
    }

    esp_err_t ret = writeExpander(bus);
    if (ret != ESP_OK) {
        return ret;
    }
    return pulseEnable(bus);
}

esp_err_t StatusLcd::pulseEnable(uint8_t value) {
    esp_err_t ret = writeExpander(static_cast<uint8_t>(value | kEnableMask));
    if (ret != ESP_OK) {
        return ret;
    }
    ets_delay_us(1);
    ret = writeExpander(static_cast<uint8_t>(value & ~kEnableMask));
    ets_delay_us(50);
    return ret;
}

esp_err_t StatusLcd::writeExpander(uint8_t value) {
    if (dev_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit(dev_, &value, 1, 100);
}

void StatusLcd::renderLine(uint8_t row, const char *text) {
    (void)writeCommand(lineAddress(row));

    char padded[17] = {0};
    std::snprintf(padded, sizeof(padded), "%-16.16s", text != nullptr ? text : "");
    for (size_t i = 0; i < kLineChars; ++i) {
        (void)writeData(static_cast<uint8_t>(padded[i]));
    }
}

char StatusLcd::transportCode(TransportType transport) {
    switch (transport) {
    case TransportType::kWifi:
        return 'W';
    case TransportType::kEthernet:
        return 'E';
    case TransportType::kGprs:
        return 'G';
    case TransportType::kNone:
    default:
        return 'N';
    }
}

char StatusLcd::timeSourceCode(TimeSource source) {
    switch (source) {
    case TimeSource::kGps:
        return 'G';
    case TimeSource::kNtp:
        return 'N';
    case TimeSource::kGsm:
        return 'S';
    case TimeSource::kFallback:
    default:
        return 'F';
    }
}
