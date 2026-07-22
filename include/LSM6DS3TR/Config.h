/// @file Config.h
/// @brief Transport binding and replayable LSM6DS3TR-C device profile.
#pragma once

#include <cstddef>
#include <cstdint>

#include "LSM6DS3TR/Status.h"

namespace LSM6DS3TR {

/// @brief Execute one complete, synchronous, timeout-bounded I2C write.
/// @param address Seven-bit device address.
/// @param data Bytes to transmit, including the register address.
/// @param length Number of bytes at @p data.
/// @param timeoutMs Per-attempt timeout selected by DriverConfig.
/// @param user Opaque application context from DriverConfig::i2cUser.
/// @return Status::Ok() on confirmed success, otherwise a mapped transport error.
/// @note The callback performs exactly one physical attempt. The application
/// owns the bus, locking, retries, recovery, and scheduling.
using I2cWriteFn = Status (*)(uint8_t address, const uint8_t* data, size_t length,
                              uint32_t timeoutMs, void* user);

/// @brief Execute one complete write-then-read I2C transaction.
/// @param address Seven-bit device address.
/// @param txData Bytes to transmit, including the register address.
/// @param txLength Number of bytes at @p txData.
/// @param rxData Caller-owned receive buffer.
/// @param rxLength Number of bytes to receive.
/// @param timeoutMs Per-attempt timeout selected by DriverConfig.
/// @param user Opaque application context from DriverConfig::i2cUser.
/// @return Status::Ok() on confirmed success, otherwise a mapped transport error.
/// @note The callback performs exactly one physical attempt and must not retry.
using I2cWriteReadFn = Status (*)(uint8_t address, const uint8_t* txData, size_t txLength,
                                  uint8_t* rxData, size_t rxLength,
                                  uint32_t timeoutMs, void* user);

/// @brief Supported seven-bit addresses selected by the SA0 pin.
enum class SensorAddress : uint8_t {
  SA0_GND = 0x6A,  ///< SA0 tied low.
  SA0_VDD = 0x6B  ///< SA0 tied high.
};

/// @brief Zero-I/O transport binding. No sensor settings are applied by bind().
struct DriverConfig {
  I2cWriteFn i2cWrite = nullptr;          ///< Required single-attempt write callback.
  I2cWriteReadFn i2cWriteRead = nullptr;  ///< Required single-attempt combined callback.
  void* i2cUser = nullptr;                ///< Opaque context forwarded unchanged.
  SensorAddress address = SensorAddress::SA0_GND;  ///< Bound device address.
  uint32_t i2cTimeoutMs = 50;  ///< Nonzero timeout passed to every callback.
};

/// @brief Output data-rate encoding shared by accelerometer and gyroscope.
/// @note HZ_1_6 is valid only for the accelerometer in low-power/normal mode.
enum class Odr : uint8_t {
  POWER_DOWN = 0,  ///< Sensor disabled.
  HZ_12_5 = 1,    ///< 12.5 Hz.
  HZ_26 = 2,      ///< 26 Hz.
  HZ_52 = 3,      ///< 52 Hz.
  HZ_104 = 4,     ///< 104 Hz.
  HZ_208 = 5,     ///< 208 Hz.
  HZ_416 = 6,     ///< 416 Hz; high-performance mode only.
  HZ_833 = 7,     ///< 833 Hz; high-performance mode only.
  HZ_1660 = 8,    ///< 1.66 kHz; high-performance mode only.
  HZ_3330 = 9,    ///< 3.33 kHz; high-performance mode only.
  HZ_6660 = 10,   ///< 6.66 kHz; high-performance mode only.
  HZ_1_6 = 11     ///< 1.6 Hz accelerometer low-power/normal mode only.
};

/// @brief Accelerometer full-scale range.
enum class AccelFs : uint8_t {
  G_2 = 0,   ///< +/-2 g.
  G_16 = 1,  ///< +/-16 g.
  G_4 = 2,   ///< +/-4 g.
  G_8 = 3    ///< +/-8 g.
};

/// @brief Gyroscope full-scale range.
enum class GyroFs : uint8_t {
  DPS_125 = 0xFF,  ///< +/-125 degrees per second.
  DPS_250 = 0,     ///< +/-250 degrees per second.
  DPS_500 = 1,     ///< +/-500 degrees per second.
  DPS_1000 = 2,    ///< +/-1000 degrees per second.
  DPS_2000 = 3     ///< +/-2000 degrees per second.
};

/// @brief Accelerometer power/performance mode.
enum class AccelPowerMode : uint8_t {
  HIGH_PERFORMANCE = 0,  ///< High-performance operating mode.
  LOW_POWER_NORMAL = 1   ///< Low-power/normal mode, limited to 208 Hz.
};

/// @brief Gyroscope power/performance mode.
enum class GyroPowerMode : uint8_t {
  HIGH_PERFORMANCE = 0,  ///< High-performance operating mode.
  LOW_POWER_NORMAL = 1   ///< Low-power/normal mode, limited to 208 Hz.
};

/// @brief Register-level gyroscope high-pass cutoff encoding.
/// @note Production DeviceProfile values currently require high-pass disabled.
enum class GyroHpfMode : uint8_t {
  HZ_0_016 = 0,  ///< 0.016 Hz cutoff encoding.
  HZ_0_065 = 1,  ///< 0.065 Hz cutoff encoding.
  HZ_0_260 = 2,  ///< 0.260 Hz cutoff encoding.
  HZ_1_040 = 3   ///< 1.040 Hz cutoff encoding.
};

/// @brief Accelerometer user-offset scale.
enum class AccelOffsetWeight : uint8_t {
  MG_1 = 0,  ///< 1 mg per LSB.
  MG_16 = 1  ///< 16 mg per LSB.
};

/// @brief Signed sensor-native accelerometer user offsets.
/// @note Each value must be in -127..127; -128 is a reserved encoding.
struct AccelUserOffset {
  int8_t x = 0;  ///< X-axis register value.
  int8_t y = 0;  ///< Y-axis register value.
  int8_t z = 0;  ///< Z-axis register value.
};

/// @brief Accelerometer filter choices represented by DeviceProfile.
struct AccelFilterConfig {
  bool lpf2Enabled = false;  ///< Enable the supported LPF2 output path.
  bool highPassSlopeEnabled = false;  ///< Reserved for future typed support.
  bool lowPassOn6d = false;  ///< Reserved for future interrupt-profile support.
};

/// @brief Gyroscope filter choices represented by DeviceProfile.
struct GyroFilterConfig {
  bool lpf1Enabled = false;  ///< Enable LPF1 in high-performance mode.
  bool highPassEnabled = false;  ///< Unsupported in production profiles.
  GyroHpfMode highPassMode = GyroHpfMode::HZ_0_016;  ///< Register encoding when enabled.
};

/// @brief FIFO policy represented by the production profile.
///
/// Version 2 production jobs support bypass only. Data removal remains available
/// as the explicitly destructive, bounded FIFO_PURGE operation.
struct FifoProfile {
  bool enabled = false;  ///< Must remain false in version 2 production profiles.
};

/// @brief Interrupt policy represented by the production profile.
///
/// Interrupt routing is intentionally disabled until a complete electrical,
/// route, latch, threshold, and duration profile is provided by a product.
struct InterruptProfile {
  bool enabled = false;  ///< Must remain false in version 2 production profiles.
};

/// @brief Complete replayable state owned by the production driver.
struct DeviceProfile {
  Odr accelOdr = Odr::HZ_104;  ///< Accelerometer output data rate.
  AccelFs accelFullScale = AccelFs::G_2;  ///< Accelerometer measurement range.
  AccelPowerMode accelPowerMode = AccelPowerMode::HIGH_PERFORMANCE;  ///< Accelerometer mode.
  Odr gyroOdr = Odr::HZ_104;  ///< Gyroscope output data rate.
  GyroFs gyroFullScale = GyroFs::DPS_250;  ///< Gyroscope measurement range.
  GyroPowerMode gyroPowerMode = GyroPowerMode::HIGH_PERFORMANCE;  ///< Gyroscope mode.
  AccelFilterConfig accelFilter = {};  ///< Accelerometer filter policy.
  GyroFilterConfig gyroFilter = {};  ///< Gyroscope filter policy.
  bool gyroSleepEnabled = false;  ///< Put a configured gyroscope into sleep mode.
  bool blockDataUpdate = true;  ///< Must remain enabled for managed burst reads.
  AccelOffsetWeight accelOffsetWeight = AccelOffsetWeight::MG_1;  ///< Offset scale.
  AccelUserOffset accelUserOffset = {};  ///< Signed hardware offset values.
  FifoProfile fifo = {};  ///< FIFO policy; version 2 requires bypass.
  InterruptProfile interrupts = {};  ///< Interrupt policy; version 2 requires disabled.
};

/// @brief Quality limits for bounded bias calibration.
struct CalibrationLimits {
  float accelMaxPeakToPeakG = 0.08f;  ///< Per-axis acceleration stability limit.
  float gyroMaxPeakToPeakDps = 3.0f;  ///< Per-axis angular-rate stability limit.
};

}  // namespace LSM6DS3TR
