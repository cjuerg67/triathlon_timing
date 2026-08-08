#pragma once

#include <cstdint>

namespace app_config {

static constexpr const char *kMqttBrokerLan = "mqtt://192.168.2.142";
static constexpr const char *kMqttBrokerGprs = "mqtt://10.65.240.213";
static constexpr int kMqttPort = 1883;
static constexpr const char *kMqttTopicTag = "triathlon/tags";
static constexpr const char *kMqttTopicTagLast = "triathlon/tags/last";
static constexpr const char *kMqttTopicGps = "triathlon/gps";
static constexpr const char *kMqttClientId = "triathlon-timer";
static constexpr uint32_t kGpsPublishIntervalMs = 30000;
static constexpr uint32_t kRfidDuplicateDebounceMs = 2000;
static constexpr uint32_t kMqttReconnectIntervalMs = 5000;
static constexpr const char *kNtpServer = "pool.ntp.org";
static constexpr const char *kTimezone = "CET-1CEST,M3.5.0,M10.5.0/3";
static constexpr const char *kWifiSsid = "***REMOVED***";
static constexpr const char *kWifiPassword = "***REMOVED***";
static constexpr uint32_t kWifiConnectTimeoutMs = 10000;
static constexpr int kC6UartPort = 0;
static constexpr int kC6UartTxPin = 5;
static constexpr int kC6UartRxPin = 4;

}  // namespace app_config
