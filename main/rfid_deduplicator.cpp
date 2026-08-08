#include "rfid_deduplicator.hpp"

#include <cstdio>
#include <cstring>

RfidDeduplicator::RfidDeduplicator(uint32_t debounce_ms) : debounce_ms_(debounce_ms), entries_{} {}

bool RfidDeduplicator::shouldDrop(const RfidEvent &event, uint32_t now_ms) {
    size_t free_slot = kMaxEntries;

    for (size_t i = 0; i < kMaxEntries; ++i) {
        if (!entries_[i].used) {
            if (free_slot == kMaxEntries) {
                free_slot = i;
            }
            continue;
        }

        if (std::strncmp(entries_[i].reader_id, event.reader_id, sizeof(entries_[i].reader_id)) == 0 &&
            std::strncmp(entries_[i].rfid_id, event.rfid_id, sizeof(entries_[i].rfid_id)) == 0) {
            if ((now_ms - entries_[i].last_seen_ms) < debounce_ms_) {
                return true;
            }

            entries_[i].last_seen_ms = now_ms;
            return false;
        }
    }

    if (free_slot == kMaxEntries) {
        free_slot = 0;
    }

    entries_[free_slot].used = true;
    std::snprintf(entries_[free_slot].reader_id, sizeof(entries_[free_slot].reader_id), "%s", event.reader_id);
    std::snprintf(entries_[free_slot].rfid_id, sizeof(entries_[free_slot].rfid_id), "%s", event.rfid_id);
    entries_[free_slot].last_seen_ms = now_ms;
    return false;
}
