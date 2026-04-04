/// @file Config.h
/// @brief Configuration structure for LSM6DS3TR driver
#pragma once

#include <cstddef>
#include <cstdint>
#include "LSM6DS3TR/Status.h"

namespace LSM6DS3TR {

/// I2C write callback signature
/// @param addr     I2C device address (7-bit)
/// @param data     Pointer to data to write
/// @param len      Number of bytes to write
/// @param timeoutMs Maximum time to wait for completion
/// @param user     User context pointer passed through from Config
/// @return Status indicating success or failure
using I2cWriteFn = Status (*)(uint8_t addr, const uint8_t* data, size_t len,
                              uint32_t timeoutMs, void* user);

/// I2C write-then-read callback signature
/// @param addr     I2C device address (7-bit)
/// @param txData   Pointer to data to write
/// @param txLen    Number of bytes to write
/// @param rxData   Pointer to buffer for read data
/// @param rxLen    Number of bytes to read
/// @param timeoutMs Maximum time to wait for completion
/// @param user     User context pointer passed through from Config
/// @return Status indicating success or failure
using I2cWriteReadFn = Status (*)(uint8_t addr, const uint8_t* txData, size_t txLen,
                                  uint8_t* rxData, size_t rxLen, uint32_t timeoutMs,
                                  void* user);

/// Millisecond timestamp callback.
/// @param user User context pointer passed through from Config
/// @return Current monotonic milliseconds
using NowMsFn = uint32_t (*)(void* user);

/// Output Data Rate (shared between accel and gyro, except 1.6 Hz accel-LP-only)
enum class Odr : uint8_t {
  POWER_DOWN = 0,  ///< Sensor disabled
  HZ_1_6    = 11,  ///< 1.6 Hz (accelerometer low-power only)
  HZ_12_5   = 1,   ///< 12.5 Hz
  HZ_26     = 2,   ///< 26 Hz
  HZ_52     = 3,   ///< 52 Hz
  HZ_104    = 4,   ///< 104 Hz
  HZ_208    = 5,   ///< 208 Hz
  HZ_416    = 6,   ///< 416 Hz
  HZ_833    = 7,   ///< 833 Hz
  HZ_1660   = 8,   ///< 1.66 kHz
  HZ_3330   = 9,   ///< 3.33 kHz
  HZ_6660   = 10   ///< 6.66 kHz
};

/// Accelerometer full-scale selection
/// NOTE: Register encoding is non-sequential: 00=±2g, 01=±16g, 10=±4g, 11=±8g
enum class AccelFs : uint8_t {
  G_2  = 0,  ///< ±2 g  (sensitivity 0.061 mg/LSB)
  G_16 = 1,  ///< ±16 g (sensitivity 0.488 mg/LSB)
  G_4  = 2,  ///< ±4 g  (sensitivity 0.122 mg/LSB)
  G_8  = 3   ///< ±8 g  (sensitivity 0.244 mg/LSB)
};

/// Gyroscope full-scale selection
enum class GyroFs : uint8_t {
  DPS_125  = 0xFF, ///< ±125 dps  (sensitivity 4.375 mdps/LSB) — uses FS_125 bit
  DPS_250  = 0,    ///< ±250 dps  (sensitivity 8.75 mdps/LSB)
  DPS_500  = 1,    ///< ±500 dps  (sensitivity 17.50 mdps/LSB)
  DPS_1000 = 2,    ///< ±1000 dps (sensitivity 35 mdps/LSB)
  DPS_2000 = 3     ///< ±2000 dps (sensitivity 70 mdps/LSB)
};

/// Configuration for LSM6DS3TR driver
struct Config {
  // === I2C Transport (required) ===
  I2cWriteFn i2cWrite = nullptr;         ///< I2C write function pointer
  I2cWriteReadFn i2cWriteRead = nullptr; ///< I2C write-read function pointer
  void* i2cUser = nullptr;               ///< User context for callbacks

  // === Timing Hooks (optional) ===
  NowMsFn nowMs = nullptr;               ///< Monotonic millisecond source
  void* timeUser = nullptr;              ///< User context for timing hook

  // === Device Settings ===
  uint8_t i2cAddress = 0x6A;             ///< 0x6A (SA0=GND) or 0x6B (SA0=VDD)
  uint32_t i2cTimeoutMs = 50;            ///< I2C transaction timeout in ms

  // === Sensor Configuration ===
  Odr odrXl = Odr::HZ_104;              ///< Accelerometer output data rate
  Odr odrG = Odr::HZ_104;               ///< Gyroscope output data rate
  AccelFs fsXl = AccelFs::G_2;           ///< Accelerometer full-scale
  GyroFs fsG = GyroFs::DPS_250;          ///< Gyroscope full-scale
  bool bdu = true;                       ///< Block Data Update (recommended: true)

  // === Health Tracking ===
  uint8_t offlineThreshold = 5;          ///< Consecutive failures before OFFLINE state
};

} // namespace LSM6DS3TR
