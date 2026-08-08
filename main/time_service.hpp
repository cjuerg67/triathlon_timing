#pragma once

#include <cstddef>
#include <cstdint>

#include "app_config.hpp"

enum class TimeSource {
    kGps,
    kNtp,
    kGsm,
    kFallback,
};

class TimeService {
public:
    TimeService();
    void initNtp();
    TimeSource getCurrentTimeHHMMSS(char *out, size_t out_size) const;

private:
    bool queryAtLine(const char *command, char *line_out, size_t line_out_size, uint32_t timeout_ms) const;
    bool parseGsmClock(const char *line, int &hour, int &minute, int &second) const;
    bool parseGnsUtc(const char *line, int &hour, int &minute, int &second) const;
    bool tryGetGpsTime(int &hour, int &minute, int &second) const;
    bool tryGetNtpTime(int &hour, int &minute, int &second) const;
    bool tryGetGsmNetworkTime(int &hour, int &minute, int &second) const;

    int uart_port_ = app_config::kSim7000UartPort;
    bool uart_initialized_ = false;
    bool ntp_initialized_ = false;
    mutable bool ntp_synced_ = false;
    mutable bool gps_power_enabled_ = false;
};
