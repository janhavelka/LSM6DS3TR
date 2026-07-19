/// @file Config.h
/// @brief Transport binding and replayable LSM6DS3TR-C device profile.
#pragma once

#include <cstddef>
#include <cstdint>

#include "LSM6DS3TR/Status.h"

namespace LSM6DS3TR {

/// A callback is one complete, synchronous, timeout-bounded bus transaction.
/// The application owns the bus, locking, retries, recovery, and scheduling.
using I2cWriteFn = Status (*)(uint8_t address, const uint8_t* data, size_t length,
                              uint32_t timeoutMs, void* user);
using I2cWriteReadFn = Status (*)(uint8_t address, const uint8_t* txData, size_t txLength,
                                  uint8_t* rxData, size_t rxLength,
                                  uint32_t timeoutMs, void* user);

enum class SensorAddress : uint8_t {
  SA0_GND = 0x6A,
  SA0_VDD = 0x6B
};

/// @brief Zero-I/O transport binding. No sensor settings are applied by bind().
struct DriverConfig {
  I2cWriteFn i2cWrite = nullptr;
  I2cWriteReadFn i2cWriteRead = nullptr;
  void* i2cUser = nullptr;
  SensorAddress address = SensorAddress::SA0_GND;
  uint32_t i2cTimeoutMs = 50;
};

enum class Odr : uint8_t {
  POWER_DOWN = 0,
  HZ_12_5 = 1,
  HZ_26 = 2,
  HZ_52 = 3,
  HZ_104 = 4,
  HZ_208 = 5,
  HZ_416 = 6,
  HZ_833 = 7,
  HZ_1660 = 8,
  HZ_3330 = 9,
  HZ_6660 = 10,
  HZ_1_6 = 11
};

enum class AccelFs : uint8_t {
  G_2 = 0,
  G_16 = 1,
  G_4 = 2,
  G_8 = 3
};

enum class GyroFs : uint8_t {
  DPS_125 = 0xFF,
  DPS_250 = 0,
  DPS_500 = 1,
  DPS_1000 = 2,
  DPS_2000 = 3
};

enum class AccelPowerMode : uint8_t {
  HIGH_PERFORMANCE = 0,
  LOW_POWER_NORMAL = 1
};

enum class GyroPowerMode : uint8_t {
  HIGH_PERFORMANCE = 0,
  LOW_POWER_NORMAL = 1
};

enum class GyroHpfMode : uint8_t {
  HZ_0_0081 = 0,
  HZ_0_0324 = 1,
  HZ_2_07 = 2,
  HZ_16_32 = 3
};

enum class AccelOffsetWeight : uint8_t {
  MG_1 = 0,
  MG_16 = 1
};

struct AccelUserOffset {
  int8_t x = 0;
  int8_t y = 0;
  int8_t z = 0;
};

struct AccelFilterConfig {
  bool lpf2Enabled = false;
  bool highPassSlopeEnabled = false;
  bool lowPassOn6d = false;
};

struct GyroFilterConfig {
  bool lpf1Enabled = false;
  bool highPassEnabled = false;
  GyroHpfMode highPassMode = GyroHpfMode::HZ_0_0081;
};

/// @brief FIFO policy represented by the production profile.
///
/// Version 2 production jobs support bypass only. Data removal remains available
/// as the explicitly destructive, bounded FIFO_PURGE operation.
struct FifoProfile {
  bool enabled = false;
};

/// @brief Interrupt policy represented by the production profile.
///
/// Interrupt routing is intentionally disabled until a complete electrical,
/// route, latch, threshold, and duration profile is provided by a product.
struct InterruptProfile {
  bool enabled = false;
};

/// @brief Complete replayable state owned by the production driver.
struct DeviceProfile {
  Odr accelOdr = Odr::HZ_104;
  AccelFs accelFullScale = AccelFs::G_2;
  AccelPowerMode accelPowerMode = AccelPowerMode::HIGH_PERFORMANCE;
  Odr gyroOdr = Odr::HZ_104;
  GyroFs gyroFullScale = GyroFs::DPS_250;
  GyroPowerMode gyroPowerMode = GyroPowerMode::HIGH_PERFORMANCE;
  AccelFilterConfig accelFilter = {};
  GyroFilterConfig gyroFilter = {};
  bool gyroSleepEnabled = false;
  bool blockDataUpdate = true;
  AccelOffsetWeight accelOffsetWeight = AccelOffsetWeight::MG_1;
  AccelUserOffset accelUserOffset = {};
  FifoProfile fifo = {};
  InterruptProfile interrupts = {};
};

/// @brief Quality limits for bounded bias calibration.
struct CalibrationLimits {
  float accelMaxPeakToPeakG = 0.08f;
  float gyroMaxPeakToPeakDps = 3.0f;
};

}  // namespace LSM6DS3TR
