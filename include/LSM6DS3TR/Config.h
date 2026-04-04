/// @file Config.h
/// @brief Configuration structure for LSM6DS3TR driver
#pragma once

#include <cstddef>
#include <cstdint>

#include "LSM6DS3TR/Status.h"

namespace LSM6DS3TR {

/// I2C write callback signature
using I2cWriteFn = Status (*)(uint8_t addr, const uint8_t* data, size_t len,
                              uint32_t timeoutMs, void* user);

/// I2C write-then-read callback signature
using I2cWriteReadFn = Status (*)(uint8_t addr, const uint8_t* txData, size_t txLen,
                                  uint8_t* rxData, size_t rxLen, uint32_t timeoutMs,
                                  void* user);

/// Millisecond timestamp callback
using NowMsFn = uint32_t (*)(void* user);

/// Output data rate register encoding
enum class Odr : uint8_t {
  POWER_DOWN = 0,  ///< Sensor disabled
  HZ_12_5    = 1,  ///< 12.5 Hz
  HZ_26      = 2,  ///< 26 Hz
  HZ_52      = 3,  ///< 52 Hz
  HZ_104     = 4,  ///< 104 Hz
  HZ_208     = 5,  ///< 208 Hz
  HZ_416     = 6,  ///< 416 Hz
  HZ_833     = 7,  ///< 833 Hz
  HZ_1660    = 8,  ///< 1.66 kHz
  HZ_3330    = 9,  ///< 3.33 kHz
  HZ_6660    = 10, ///< 6.66 kHz
  HZ_1_6     = 11  ///< 1.6 Hz (accelerometer low-power only)
};

/// Accelerometer full-scale selection
/// Register encoding is 00=2g, 01=16g, 10=4g, 11=8g
enum class AccelFs : uint8_t {
  G_2  = 0, ///< +/-2 g
  G_16 = 1, ///< +/-16 g
  G_4  = 2, ///< +/-4 g
  G_8  = 3  ///< +/-8 g
};

/// Gyroscope full-scale selection
enum class GyroFs : uint8_t {
  DPS_125  = 0xFF, ///< +/-125 dps, uses FS_125 bit
  DPS_250  = 0,    ///< +/-250 dps
  DPS_500  = 1,    ///< +/-500 dps
  DPS_1000 = 2,    ///< +/-1000 dps
  DPS_2000 = 3     ///< +/-2000 dps
};

/// Accelerometer power mode
enum class AccelPowerMode : uint8_t {
  HIGH_PERFORMANCE = 0, ///< High-performance mode
  LOW_POWER_NORMAL = 1  ///< Low-power / normal mode family
};

/// Gyroscope power mode
enum class GyroPowerMode : uint8_t {
  HIGH_PERFORMANCE = 0, ///< High-performance mode
  LOW_POWER_NORMAL = 1  ///< Low-power / normal mode family
};

/// Gyroscope high-pass filter cutoff
enum class GyroHpfMode : uint8_t {
  HZ_0_0081 = 0,
  HZ_0_0324 = 1,
  HZ_2_07   = 2,
  HZ_16_32  = 3
};

/// FIFO operating mode
enum class FifoMode : uint8_t {
  BYPASS               = 0,
  FIFO                 = 1,
  CONTINUOUS_TO_FIFO   = 3,
  BYPASS_TO_CONTINUOUS = 4,
  CONTINUOUS           = 6
};

/// FIFO decimation factor
#ifdef DISABLED
#undef DISABLED
#endif
enum class FifoDecimation : uint8_t {
  DISABLED = 0,
  NONE     = 1,
  DIV_2    = 2,
  DIV_3    = 3,
  DIV_4    = 4,
  DIV_8    = 5,
  DIV_16   = 6,
  DIV_32   = 7
};

/// Accelerometer user-offset register weight
enum class AccelOffsetWeight : uint8_t {
  MG_1  = 0, ///< About 1 mg/LSB
  MG_16 = 1  ///< About 15.6 mg/LSB
};

/// Configuration for LSM6DS3TR driver
struct Config {
  // I2C transport
  I2cWriteFn i2cWrite = nullptr;
  I2cWriteReadFn i2cWriteRead = nullptr;
  void* i2cUser = nullptr;

  // Timing hooks
  NowMsFn nowMs = nullptr;
  void* timeUser = nullptr;

  // Device settings
  uint8_t i2cAddress = 0x6A;
  uint32_t i2cTimeoutMs = 50;

  // Sensor configuration
  Odr odrXl = Odr::HZ_104;
  Odr odrG = Odr::HZ_104;
  AccelFs fsXl = AccelFs::G_2;
  GyroFs fsG = GyroFs::DPS_250;
  AccelPowerMode accelPowerMode = AccelPowerMode::HIGH_PERFORMANCE;
  GyroPowerMode gyroPowerMode = GyroPowerMode::HIGH_PERFORMANCE;
  bool bdu = true;

  // Health tracking
  uint8_t offlineThreshold = 5;
};

}  // namespace LSM6DS3TR
