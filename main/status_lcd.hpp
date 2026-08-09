#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "time_service.hpp"
#include "transport_manager.hpp"

class StatusLcd {
public:
    esp_err_t init();
    void update(TimeService &time_service,
                TransportType transport,
                bool transport_up,
                bool mqtt_up,
                const char *latest_tag);

private:
    esp_err_t writeCommand(uint8_t value);
    esp_err_t writeData(uint8_t value);
    esp_err_t write4Bit(uint8_t value, bool rs);
    esp_err_t pulseEnable(uint8_t value);
    esp_err_t writeExpander(uint8_t value);
    void renderLine(uint8_t row, const char *text);
    static char transportCode(TransportType transport);
    static char timeSourceCode(TimeSource source);

    bool initialized_ = false;
    uint8_t backlight_mask_ = 0x08;
    size_t scroll_offset_ = 0;
    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t dev_ = nullptr;
};
