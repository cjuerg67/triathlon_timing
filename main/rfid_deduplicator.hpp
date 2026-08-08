#pragma once

#include <cstdint>

#include "rfid_event.hpp"

class RfidDeduplicator {
public:
    explicit RfidDeduplicator(uint32_t debounce_ms);
    bool shouldDrop(const RfidEvent &event, uint32_t now_ms);

private:
    static constexpr size_t kMaxEntries = 32;

    struct Entry {
        bool used;
        char reader_id[24];
        char rfid_id[96];
        uint32_t last_seen_ms;
    };

    uint32_t debounce_ms_;
    Entry entries_[kMaxEntries];
};
