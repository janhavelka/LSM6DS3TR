/// @file Wire.h
/// @brief ESP-IDF I2C master facade with the small TwoWire surface used by examples.
///
/// The implementation uses the ESP-IDF v6 `driver/i2c_master.h` APIs. It is
/// example-local glue and is not part of the framework-neutral driver core.
#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"

class TwoWire {
public:
  bool begin(int sda, int scl);
  void end();
  void setClock(uint32_t freqHz);
  void setTimeOut(uint16_t timeoutMs);

  void beginTransmission(uint8_t address);
  size_t write(const uint8_t* data, size_t len);
  size_t write(uint8_t value);
  uint8_t endTransmission(bool stop = true);

  size_t requestFrom(uint8_t address, uint8_t quantity);
  int available() const;
  int read();

private:
  static constexpr size_t BUFFER_CAPACITY = 128U;
  static constexpr size_t MAX_ADDRESS = 128U;

  i2c_master_dev_handle_t deviceFor(uint8_t address);
  uint8_t mapError(esp_err_t err) const;
  int timeoutMs() const;

  i2c_master_bus_handle_t _bus = nullptr;
  i2c_master_dev_handle_t _devices[MAX_ADDRESS] = {};
  uint32_t _freqHz = 400000U;
  uint16_t _timeoutMs = 50U;
  int _sda = -1;
  int _scl = -1;

  uint8_t _txAddress = 0;
  uint8_t _txBuffer[BUFFER_CAPACITY] = {};
  size_t _txLen = 0;
  bool _hasPendingRepeatedStart = false;

  uint8_t _rxBuffer[BUFFER_CAPACITY] = {};
  size_t _rxLen = 0;
  size_t _rxIndex = 0;
};

extern TwoWire Wire;
