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
#include <esp32-hal-i2c.h>

#include "LSM6DS3TR/Status.h"

namespace transport {

inline LSM6DS3TR::Status mapWireProbeResult(uint8_t result,
                                            const char* context) {
  switch (result) {
    case 0:
      return LSM6DS3TR::Status::Ok();
    case 1:
      return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::INVALID_PARAM, context, result);
    case 2:
      return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_NACK_ADDR, context, result);
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

inline LSM6DS3TR::Status mapEspI2cResult(esp_err_t result,
                                        const char* context) {
  switch (result) {
    case ESP_OK:
      return LSM6DS3TR::Status::Ok();
    case ESP_ERR_INVALID_ARG:
      return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::INVALID_PARAM,
                                      context, result);
    case ESP_ERR_TIMEOUT:
      return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_TIMEOUT,
                                      context, result);
    case ESP_ERR_INVALID_STATE:
    case ESP_ERR_INVALID_RESPONSE:
    case ESP_ERR_NOT_FOUND:
    case ESP_FAIL:
      // The pinned HAL can use these values for a NACK or an internal/resource
      // failure, and managed transfers do not identify the ACK phase. Preserve
      // native detail without inventing a busy/address/data classification.
      return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_ERROR,
                                      context, result);
    default:
      // Native resource and implementation errors are not proof of an
      // electrical bus fault. Keep the class generic and retain raw detail.
      return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_ERROR,
                                      context, result);
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
  return mapWireProbeResult(wire.endTransmission(true),
                            "I2C address probe failed");
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

  // Use the same native-result-preserving HAL boundary as write-read. Wire's
  // endTransmission() compresses several esp_err_t values into "other error".
  return mapEspI2cResult(
      i2cWrite(wire->getBusNum(), addr, data, len, timeoutMs),
      "I2C write failed");
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

  // Wire's repeated-start path discards the native esp_err_t. The ESP32 HAL
  // provides the same combined transaction, owns its bus lock, and preserves
  // the error needed by the application's recovery policy.
  size_t read = 0;
  const esp_err_t result = i2cWriteReadNonStop(
      wire->getBusNum(), addr, tx, txLen, rx, rxLen, timeoutMs, &read);
  if (result != ESP_OK) {
    return mapEspI2cResult(result, "I2C write-read failed");
  }
  if (read != rxLen) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_BUS, "I2C read length mismatch",
                                    static_cast<int32_t>(read));
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
