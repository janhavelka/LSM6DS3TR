/**
 * @file I2cScanner.h
 * @brief Simple I2C bus scanner utility for examples.
 *
 * NOT part of the library API. This is a diagnostic tool for examples.
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "examples/common/Log.h"

namespace i2c_scanner {

/**
 * @brief Attempt to recover a stuck I2C bus by toggling SCL.
 * @param wire Wire instance to restore after bit-bang recovery.
 * @param sda SDA pin number
 * @param scl SCL pin number
 * @param freqHz I2C clock frequency to restore.
 * @param timeoutMs I2C timeout to restore.
 */
inline void recoverBus(TwoWire& wire, int sda, int scl, uint32_t freqHz, uint16_t timeoutMs) {
  wire.end();

  pinMode(scl, OUTPUT);
  pinMode(sda, INPUT_PULLUP);

  for (int i = 0; i < 9; i++) {
    digitalWrite(scl, LOW);
    delayMicroseconds(5);
    digitalWrite(scl, HIGH);
    delayMicroseconds(5);
    if (digitalRead(sda)) {
      break;
    }
  }

  pinMode(sda, OUTPUT);
  digitalWrite(sda, LOW);
  delayMicroseconds(5);
  digitalWrite(scl, HIGH);
  delayMicroseconds(5);
  digitalWrite(sda, HIGH);
  delayMicroseconds(5);

  wire.begin(sda, scl);
  wire.setClock(freqHz);
#if defined(ARDUINO_ARCH_ESP32)
  wire.setTimeOut(timeoutMs);
#else
  (void)timeoutMs;
#endif
}

struct ScanOptions {
  uint16_t scanTimeoutMs = 50;
  uint16_t restoreTimeoutMs = 50;
  uint32_t restoreClockHz = 400000;
  int sda = -1;
  int scl = -1;
  bool recoverOnTimeout = false;
};

inline void scan(TwoWire& wire, const ScanOptions& options);

inline void restoreBusSettings(TwoWire& wire, const ScanOptions& options) {
  wire.setClock(options.restoreClockHz);
#if defined(ARDUINO_ARCH_ESP32)
  wire.setTimeOut(options.restoreTimeoutMs);
#endif
}

/**
 * @brief Scan I2C bus and print found devices.
 * @param wire Reference to Wire object (must be initialized).
 * @param timeoutMs Timeout per address probe in milliseconds (default 50ms).
 */
inline void scan(TwoWire& wire, uint16_t timeoutMs = 50) {
  ScanOptions options;
  options.scanTimeoutMs = timeoutMs;
  options.restoreTimeoutMs = timeoutMs;
  scan(wire, options);
}

/**
 * @brief Scan I2C bus and restore caller-provided Wire settings afterward.
 * @param wire Reference to Wire object (must be initialized).
 * @param options Scan and restore settings.
 */
inline void scan(TwoWire& wire, const ScanOptions& options) {
  LOGI("Scanning I2C bus (timeout=%dms)...", options.scanTimeoutMs);

#if defined(ARDUINO_ARCH_ESP32)
  wire.setTimeOut(options.scanTimeoutMs);
#endif

  LOGI("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");

  uint8_t count = 0;
  bool timedOut = false;
  for (uint8_t row = 0; row < 8; row++) {
    LOG_SERIAL.printf("%02X: ", row * 16);

    for (uint8_t col = 0; col < 16; col++) {
      uint8_t addr = row * 16 + col;
      if (addr < 0x08 || addr > 0x77) {
        LOG_SERIAL.print("   ");
        continue;
      }

      wire.beginTransmission(addr);
      uint8_t error = wire.endTransmission(true);

      if (error == 0) {
        LOG_SERIAL.printf("%02X ", addr);
        count++;
      } else if (error == 5) {
        LOG_SERIAL.print("TO ");
        timedOut = true;
      } else {
        LOG_SERIAL.print("-- ");
      }

      yield();
      if (timedOut && options.recoverOnTimeout) {
        break;
      }
    }
    LOG_SERIAL.println();
    yield();

    if (timedOut && options.recoverOnTimeout) {
      break;
    }
  }

  if (timedOut && options.recoverOnTimeout && options.sda >= 0 && options.scl >= 0) {
    LOGI("I2C timeout detected during scan; recovering bus and aborting scan.");
    recoverBus(wire, options.sda, options.scl, options.restoreClockHz, options.restoreTimeoutMs);
  } else {
    restoreBusSettings(wire, options);
  }

  LOGI("Scan complete. Found %d device(s).", count);

  if (count > 0) {
    LOGI("Common addresses: 0x3C/0x3D=OLED, 0x6A/0x6B=LSM6DS3TR, 0x76/0x77=BME280");
  }
}

}  // namespace i2c_scanner
