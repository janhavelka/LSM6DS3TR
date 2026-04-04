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

/**
 * @brief Wire-based I2C write implementation.
 */
inline LSM6DS3TR::Status wireWrite(uint8_t addr, const uint8_t* data, size_t len,
                                   uint32_t timeoutMs, void* user) {
  (void)timeoutMs;

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
  (void)timeoutMs;

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
 * @brief Initialize Wire with default pins and frequency.
 */
inline bool initWire(int sda, int scl, uint32_t freq = 400000, uint16_t timeoutMs = 50) {
#if defined(ARDUINO_ARCH_ESP32)
  // Toggle SCL to release any stuck slave
  pinMode(scl, OUTPUT);
  pinMode(sda, INPUT_PULLUP);
  for (int i = 0; i < 9; i++) {
    digitalWrite(scl, LOW);
    delayMicroseconds(5);
    digitalWrite(scl, HIGH);
    delayMicroseconds(5);
  }
  // Generate STOP condition
  pinMode(sda, OUTPUT);
  digitalWrite(sda, LOW);
  delayMicroseconds(5);
  digitalWrite(scl, HIGH);
  delayMicroseconds(5);
  digitalWrite(sda, HIGH);
  delayMicroseconds(5);
#endif

  Wire.begin(sda, scl);
  Wire.setClock(freq);
  Wire.setTimeOut(timeoutMs);
  return true;
}

}  // namespace transport
