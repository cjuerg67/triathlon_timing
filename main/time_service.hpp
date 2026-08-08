#pragma once

#include <cstddef>
#include <cstdint>

enum class TimeSource {
    kGps,
    kGsm,
    kFallback,
};

class TimeService {
public:
    TimeService();
    TimeSource getCurrentTimeHHMMSS(char *out, size_t out_size) const;

private:
    bool queryAtLine(const char *command, char *line_out, size_t line_out_size, uint32_t timeout_ms) const;
    bool parseGsmClock(const char *line, int &hour, int &minute, int &second) const;
    bool parseGnsUtc(const char *line, int &hour, int &minute, int &second) const;
    bool tryGetGpsTime(int &hour, int &minute, int &second) const;
    bool tryGetGsmNetworkTime(int &hour, int &minute, int &second) const;

    int uart_port_ = 2;
    bool uart_initialized_ = false;
    mutable bool gps_power_enabled_ = false;
};
