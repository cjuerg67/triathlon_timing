# Triathlon Timing Firmware (ESP-IDF)

This project is the new implementation baseline for triathlon timing.

## Goals

- Read UHF RFID tags from YRM100 (USB CH34x reader path).
- Publish tag events over MQTT.
- Minimal tag payload: {"rfid_id":"...","time":"HH:MM:SS"}.
- Time source fallback design: GPS (SIM7000) -> GSM network time -> fallback clock.
- Transport fallback design: Wi-Fi -> Ethernet PHY -> SIM7000 GPRS.
- Prepared for a second RFID reader via pluggable reader interface.

## Current Status

- Project scaffold created.
- YRM100 USB-host reader module integrated and emits tag events.
- MQTT publisher and transport/time manager interfaces are wired.
- Wi-Fi transport is now wired through the ESP-IDF STA path; Ethernet and GPRS remain hardware-specific fallbacks for future board integration.

## Build (ESP-IDF)

1. Open an ESP-IDF terminal.
2. cd triathlon_timing
3. idf.py set-target esp32p4
4. idf.py build
