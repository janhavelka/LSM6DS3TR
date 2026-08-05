/**
 * @file I2cTransport.h
 * @brief Wire-based I2C transport adapter for LSM6DS3TR examples.
 *
 * This file provides Wire-compatible I2C callbacks that can be
 * used with the LSM6DS3TR driver. The library does not depend on Wire
 * directly; this adapter bridges them.
 *
 * NOT part of the library API. Example-only.
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "LSM6DS3TR/Status.h"

namespace transport {

inline LSM6DS3TR::Status mapWireResult(uint8_t result, const char* context) {
  switch (result) {
    case 0:
      return LSM6DS3TR::Status::Ok();
    case 1:
      return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::INVALID_PARAM, context, result);
    case 2:
      return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_NACK_ADDR, context, result);
    case 3:
      return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_NACK_DATA, context, result);
    case 4:
      return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_BUS, context, result);
    case 5:
      return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_TIMEOUT, context, result);
    default:
      return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_ERROR, context, result);
  }
}

inline void applyTimeout(TwoWire& wire, uint32_t timeoutMs) {
  if (timeoutMs > 0U) {
    const uint16_t boundedTimeout =
        timeoutMs > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(timeoutMs);
    wire.setTimeOut(boundedTimeout);
  }
}

/**
 * @brief Perform one address-only ACK probe on the example-owned bus.
 * @note An ACK proves only that some device is present. Use the driver's
 *       WHO_AM_I probe to establish LSM6DS3TR-C identity.
 * @note This owner-level transaction is intentionally outside the driver's
 *       passive transport counters. The CLI records scan failures separately.
 */
inline LSM6DS3TR::Status wireProbe(TwoWire& wire, uint8_t address,
                                   uint32_t timeoutMs) {
  applyTimeout(wire, timeoutMs);
  wire.beginTransmission(address);
  return mapWireResult(wire.endTransmission(true), "I2C address probe failed");
}

/**
 * @brief Change the application-owned Wire clock without touching the sensor.
 * @note Success preserves driver configuration provenance because no sensor
 *       register is accessed; the caller must serialize this owner mutation.
 */
inline LSM6DS3TR::Status setWireFrequency(TwoWire& wire, uint32_t frequencyHz) {
  if (!wire.setClock(frequencyHz)) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_ERROR,
                                    "Wire clock change failed",
                                    static_cast<int32_t>(frequencyHz));
  }
  return LSM6DS3TR::Status::Ok();
}

/**
 * @brief Wire-based I2C write implementation.
 */
inline LSM6DS3TR::Status wireWrite(uint8_t addr, const uint8_t* data, size_t len,
                                   uint32_t timeoutMs, void* user) {
  TwoWire* wire = static_cast<TwoWire*>(user);
  if (wire == nullptr) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::INVALID_CONFIG, "Wire instance is null");
  }
  if (!data || len == 0) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::INVALID_PARAM, "Invalid I2C write params");
  }
  if (len > 128) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::INVALID_PARAM, "Write exceeds I2C buffer",
                                    static_cast<int32_t>(len));
  }

  applyTimeout(*wire, timeoutMs);
  wire->beginTransmission(addr);
  size_t written = wire->write(data, len);
  if (written != len) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_ERROR, "I2C write incomplete",
                                    static_cast<int32_t>(written));
  }

  uint8_t result = wire->endTransmission(true);
  return mapWireResult(result, "I2C write failed");
}

/**
 * @brief Wire-based I2C write-read implementation.
 */
inline LSM6DS3TR::Status wireWriteRead(uint8_t addr, const uint8_t* tx, size_t txLen,
                                       uint8_t* rx, size_t rxLen, uint32_t timeoutMs,
                                       void* user) {
  TwoWire* wire = static_cast<TwoWire*>(user);
  if (wire == nullptr) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::INVALID_CONFIG, "Wire instance is null");
  }
  if ((txLen > 0 && tx == nullptr) || (rxLen > 0 && rx == nullptr)) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::INVALID_PARAM, "Invalid I2C read params");
  }
  if (txLen == 0 || rxLen == 0) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::INVALID_PARAM, "I2C read length invalid");
  }
  if (txLen > 128 || rxLen > 128) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::INVALID_PARAM, "I2C read exceeds buffer");
  }

  applyTimeout(*wire, timeoutMs);
  wire->beginTransmission(addr);
  size_t written = wire->write(tx, txLen);
  if (written != txLen) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_ERROR, "I2C write incomplete",
                                    static_cast<int32_t>(written));
  }

  uint8_t result = wire->endTransmission(false);  // Repeated start
  if (result != 0) {
    return mapWireResult(result, "I2C write phase failed");
  }

  size_t read = wire->requestFrom(addr, static_cast<uint8_t>(rxLen));
  if (read != rxLen) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_ERROR, "I2C read length mismatch",
                                    static_cast<int32_t>(read));
  }

  for (size_t i = 0; i < rxLen; ++i) {
    if (wire->available()) {
      rx[i] = static_cast<uint8_t>(wire->read());
    } else {
      return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_ERROR, "I2C data not available");
    }
  }

  return LSM6DS3TR::Status::Ok();
}

/**
 * @brief Initialize Wire with application-selected pins, frequency, and timeout.
 * @return OK only when bus initialization and clock selection both succeed.
 */
inline LSM6DS3TR::Status initWire(int sda, int scl, uint32_t freq,
                                  uint16_t timeoutMs) {
  // Supply the owner-selected clock to begin() so initialization is one
  // atomic peripheral operation. ESP32-S2 must not depend on a second
  // immediate i2cSetClock() reconfiguration succeeding.
  if (!Wire.begin(sda, scl, freq)) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_ERROR,
                                    "Wire initialization failed");
  }
  const uint32_t actualFrequency = Wire.getClock();
  if (actualFrequency != freq) {
    (void)Wire.end();
    return LSM6DS3TR::Status::Error(
        LSM6DS3TR::Err::I2C_ERROR, "Wire frequency verification failed",
        static_cast<int32_t>(actualFrequency));
  }
  Wire.setTimeOut(timeoutMs);
  return LSM6DS3TR::Status::Ok();
}

}  // namespace transport
