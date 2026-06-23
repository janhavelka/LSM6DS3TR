/// @file Config.h
/// @brief Configuration structure for LSM6DS3TR driver
#pragma once

#include <cstddef>
#include <cstdint>

#include "LSM6DS3TR/Status.h"

namespace LSM6DS3TR {

/// @brief I2C write callback signature.
/// @param addr I2C device address (7-bit).
/// @param data Pointer to bytes to write.
/// @param len Number of bytes to write.
/// @param timeoutMs Transaction timeout in milliseconds.
/// @param user User context pointer.
/// @return Status indicating success or failure. Callback must complete
/// synchronously and must not return IN_PROGRESS; the driver normalizes that to
/// I2C_BUSY for compatibility with shared-bus adapters.
using I2cWriteFn = Status (*)(uint8_t addr, const uint8_t* data, size_t len,
                              uint32_t timeoutMs, void* user);

/// @brief I2C write-then-read callback signature.
/// @param addr I2C device address (7-bit).
/// @param txData Pointer to bytes to write before the read.
/// @param txLen Number of bytes to write.
/// @param rxData Output buffer for read bytes.
/// @param rxLen Number of bytes to read.
/// @param timeoutMs Transaction timeout in milliseconds.
/// @param user User context pointer.
/// @return Status indicating success or failure. Callback must complete
/// synchronously and must not return IN_PROGRESS; the driver normalizes that to
/// I2C_BUSY for compatibility with shared-bus adapters.
using I2cWriteReadFn = Status (*)(uint8_t addr, const uint8_t* txData, size_t txLen,
                                  uint8_t* rxData, size_t rxLen, uint32_t timeoutMs,
                                  void* user);

/// @brief Millisecond timestamp callback.
/// @param user User context pointer.
/// @return Current monotonic milliseconds.
using NowMsFn = uint32_t (*)(void* user);

/// @brief Quality limits for automatic software bias calibration.
struct CalibrationLimits {
  float accelMaxPeakToPeakG = 0.08f;       ///< Maximum accel peak-to-peak per axis
  float accelMaxHorizontalMeanG = 0.20f;   ///< Maximum absolute X/Y mean while Z-up
  float accelMinZMeanG = 0.80f;            ///< Minimum accepted Z-axis mean
  float accelMaxZMeanG = 1.20f;            ///< Maximum accepted Z-axis mean
  float gyroMaxPeakToPeakDps = 3.0f;       ///< Maximum gyro peak-to-peak per axis
};

/// @brief Output data rate register encoding.
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

/// @brief Accelerometer full-scale selection.
/// Register encoding is 00=2g, 01=16g, 10=4g, 11=8g
enum class AccelFs : uint8_t {
  G_2  = 0, ///< +/-2 g
  G_16 = 1, ///< +/-16 g
  G_4  = 2, ///< +/-4 g
  G_8  = 3  ///< +/-8 g
};

/// @brief Gyroscope full-scale selection.
enum class GyroFs : uint8_t {
  DPS_125  = 0xFF, ///< +/-125 dps, uses FS_125 bit
  DPS_250  = 0,    ///< +/-250 dps
  DPS_500  = 1,    ///< +/-500 dps
  DPS_1000 = 2,    ///< +/-1000 dps
  DPS_2000 = 3     ///< +/-2000 dps
};

/// @brief Accelerometer power mode.
enum class AccelPowerMode : uint8_t {
  HIGH_PERFORMANCE = 0, ///< High-performance mode
  LOW_POWER_NORMAL = 1  ///< Low-power / normal mode family
};

/// @brief Gyroscope power mode.
enum class GyroPowerMode : uint8_t {
  HIGH_PERFORMANCE = 0, ///< High-performance mode
  LOW_POWER_NORMAL = 1  ///< Low-power / normal mode family
};

/// @brief Gyroscope high-pass filter cutoff.
enum class GyroHpfMode : uint8_t {
  HZ_0_0081 = 0, ///< 0.0081 Hz
  HZ_0_0324 = 1, ///< 0.0324 Hz
  HZ_2_07   = 2, ///< 2.07 Hz
  HZ_16_32  = 3  ///< 16.32 Hz
};

/// @brief FIFO operating mode.
enum class FifoMode : uint8_t {
  BYPASS               = 0, ///< FIFO disabled
  FIFO                 = 1, ///< Stop when FIFO is full
  CONTINUOUS_TO_FIFO   = 3, ///< Continuous until trigger, then FIFO
  BYPASS_TO_CONTINUOUS = 4, ///< Bypass until trigger, then continuous
  CONTINUOUS           = 6  ///< Continuous FIFO
};

/// @brief FIFO decimation factor.
#ifdef DISABLED
#undef DISABLED
#endif
enum class FifoDecimation : uint8_t {
  DISABLED = 0, ///< Do not store this source
  NONE     = 1, ///< Store every sample
  DIV_2    = 2, ///< Store every 2nd sample
  DIV_3    = 3, ///< Store every 3rd sample
  DIV_4    = 4, ///< Store every 4th sample
  DIV_8    = 5, ///< Store every 8th sample
  DIV_16   = 6, ///< Store every 16th sample
  DIV_32   = 7  ///< Store every 32nd sample
};

/// @brief Accelerometer user-offset register weight.
enum class AccelOffsetWeight : uint8_t {
  MG_1  = 0, ///< About 1 mg/LSB
  MG_16 = 1  ///< About 15.6 mg/LSB
};

/// @brief Configuration for LSM6DS3TR driver.
struct Config {
  // I2C transport
  I2cWriteFn i2cWrite = nullptr;         ///< I2C write function pointer
  I2cWriteReadFn i2cWriteRead = nullptr; ///< I2C write-read function pointer
  void* i2cUser = nullptr;               ///< User context for I2C callbacks

  // Timing hooks
  NowMsFn nowMs = nullptr;  ///< Optional monotonic source; direct reads use 0 when absent
  void* timeUser = nullptr; ///< User context for timing hook

  // Device settings
  uint8_t i2cAddress = 0x6A;  ///< 0x6A (SA0=GND) or 0x6B (SA0=VDD)
  uint32_t i2cTimeoutMs = 50; ///< I2C transaction timeout in milliseconds

  // Sensor configuration
  Odr odrXl = Odr::HZ_104; ///< Accelerometer output data rate
  Odr odrG = Odr::HZ_104;  ///< Gyroscope output data rate
  AccelFs fsXl = AccelFs::G_2;     ///< Accelerometer full-scale range
  GyroFs fsG = GyroFs::DPS_250;    ///< Gyroscope full-scale range
  AccelPowerMode accelPowerMode = AccelPowerMode::HIGH_PERFORMANCE; ///< Accelerometer power mode
  GyroPowerMode gyroPowerMode = GyroPowerMode::HIGH_PERFORMANCE;    ///< Gyroscope power mode
  bool bdu = true; ///< Enable block data update for coherent multi-byte reads

  // Health tracking
  uint8_t offlineThreshold = 5; ///< Consecutive tracked failures before OFFLINE

  // Calibration quality
  CalibrationLimits calibrationLimits; ///< Stillness/orientation limits for bias capture
};

}  // namespace LSM6DS3TR
