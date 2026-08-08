#pragma once

#include <cstdint>

struct RfidEvent {
    char rfid_id[96];
    char reader_id[24];
    uint32_t tick_ms;
};
