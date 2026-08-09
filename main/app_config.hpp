#pragma once

#include <cstdint>

namespace app_config {

    // Wi-Fi
    static constexpr const char *kWifiSsid = "***REMOVED***";
    static constexpr const char *kWifiPassword = "***REMOVED***";
    static constexpr uint32_t kWifiConnectTimeoutMs = 10000;
    static constexpr int kC6UartPort = 0;
    static constexpr int kC6UartTxPin = 5;
    static constexpr int kC6UartRxPin = 4;

    // MQTT
    static constexpr const char *kMqttBrokerLan = "mqtt://192.168.2.142";
    static constexpr const char *kMqttBrokerGprs = "mqtt://10.65.240.213";
    static constexpr int kMqttPort = 1883;
    static constexpr const char *kMqttTopicTag = "triathlon/tags";
    static constexpr const char *kMqttTopicTagLast = "triathlon/tags/last";
    static constexpr const char *kMqttTopicGps = "triathlon/gps";
    static constexpr const char *kMqttTopicStarter = "triathlon/starter";
    static constexpr const char *kMqttClientId = "triathlon-timer";
    static constexpr uint32_t kMqttReconnectIntervalMs = 5000;
    static constexpr uint32_t kMqttReconnectMaxIntervalMs = 30000;

    // GPS
    static constexpr int kGpsUartPort = 1;
    static constexpr int kGpsUartTxPin = 17;
    static constexpr int kGpsUartRxPin = 16;
    static constexpr uint32_t kGpsPublishIntervalMs = 30000;

    // RFID reader
    static constexpr uint32_t kRfidDuplicateDebounceMs = 2000;
    static constexpr uint32_t kYrm100MinEpcBytes = 8;
    static constexpr uint32_t kYrm100ConfirmWindowMs = 700;
    static constexpr uint32_t kYrm100RequiredSightings = 2;
    static constexpr bool kEnableRfidReader = true;
    static constexpr bool kEnableCfE714Reader = false;
    static constexpr bool kEnableGprsTransport = true;
    static constexpr bool kEnableVerboseTransportLogs = false;
    static constexpr bool kEnableYrm100StatusLogs = false;
    static constexpr uint32_t kTransportRefreshIntervalMs = 3000;
    static constexpr const char *kNtpServer = "pool.ntp.org";
    static constexpr const char *kTimezone = "CET-1CEST,M3.5.0,M10.5.0/3";

    // Single HD44780 16x2 with PCF8574 backpack (typical address 0x27).
    static constexpr int kStatusLcdI2cPort = 0;
    static constexpr int kStatusLcdSdaPin = 7;
    static constexpr int kStatusLcdSclPin = 8;
    static constexpr uint8_t kStatusLcdAddress = 0x27;
    static constexpr uint32_t kStatusLcdI2cFrequencyHz = 100000;
    static constexpr bool kEnableRuntimeStatusLine = true;
    static constexpr bool kEnableStatusLcd = true;
    static constexpr uint32_t kRuntimeStatusIntervalMs = 5000;
    static constexpr uint32_t kStatusLcdUpdateIntervalMs = 500;

    // SIM7000 GPRS modem — adjust pins to match your board wiring
    static constexpr int kSim7000UartPort = 2;
    static constexpr int kSim7000UartTxPin = 23;
    static constexpr int kSim7000UartRxPin = 22;
    static constexpr int kSim7000UartBaudRate = 115200;
    static constexpr const char *kSim7000Apn = "iot.1nce.net";
    static constexpr const char *kSim7000ApnUser = "";
    static constexpr const char *kSim7000ApnPassword = "";

    // CF-E714 TTL RFID reader — adjust pins to match your board wiring
    static constexpr int kCfE714UartPort = 1;
    static constexpr int kCfE714TxPin = 10;
    static constexpr int kCfE714RxPin = 11;
    static constexpr int kCfE714BaudRate = 57600;

}  // namespace app_config
