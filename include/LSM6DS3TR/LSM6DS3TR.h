/// @file LSM6DS3TR.h
/// @brief Main driver class for LSM6DS3TR-C IMU
#pragma once

#include <cstddef>
#include <cstdint>

#include "LSM6DS3TR/CommandTable.h"
#include "LSM6DS3TR/Config.h"
#include "LSM6DS3TR/Status.h"
#include "LSM6DS3TR/Version.h"

namespace LSM6DS3TR {

/// Driver state for health monitoring
enum class DriverState : uint8_t {
  UNINIT,
  READY,
  DEGRADED,
  OFFLINE
};

/// 3-axis raw sample
struct RawAxes {
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
};

/// 3-axis physical sample
struct Axes {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

/// Combined IMU measurement
struct Measurement {
  Axes accel;
  Axes gyro;
  float temperatureC = 0.0f;
};

/// Combined raw IMU sample
struct RawMeasurement {
  RawAxes accel;
  RawAxes gyro;
  int16_t temperature = 0;
};

/// Accelerometer user-offset register values
struct AccelUserOffset {
  int8_t x = 0;
  int8_t y = 0;
  int8_t z = 0;
};

/// High-level accelerometer filter state managed by the driver
struct AccelFilterConfig {
  bool lpf2Enabled = false;
  bool highPassSlopeEnabled = false;
  bool lowPassOn6d = false;
};

/// High-level gyroscope filter state managed by the driver
struct GyroFilterConfig {
  bool lpf1Enabled = false;
  bool highPassEnabled = false;
  GyroHpfMode highPassMode = GyroHpfMode::HZ_0_0081;
};

/// FIFO configuration managed by the driver
struct FifoConfig {
  uint16_t threshold = 0;
  Odr odr = Odr::POWER_DOWN;
  FifoMode mode = FifoMode::BYPASS;
  FifoDecimation accelDecimation = FifoDecimation::DISABLED;
  FifoDecimation gyroDecimation = FifoDecimation::DISABLED;
  bool stopOnThreshold = false;
  bool onlyHighData = false;
  bool storeTemperature = false;
  bool storeTimestampStep = false;
};

/// Parsed FIFO status
struct FifoStatus {
  uint16_t unreadWords = 0;
  uint16_t pattern = 0;
  bool watermark = false;
  bool overrun = false;
  bool fullSmart = false;
  bool empty = true;
};

/// LSM6DS3TR-C driver class
class LSM6DS3TR {
public:
  // Lifecycle
  Status begin(const Config& config);
  void tick(uint32_t nowMs);
  void end();

  // Diagnostics
  Status probe();
  Status recover();

  // Driver state
  DriverState state() const { return _driverState; }
  bool isOnline() const {
    return _driverState == DriverState::READY ||
           _driverState == DriverState::DEGRADED;
  }

  // Health tracking
  uint32_t lastOkMs() const { return _lastOkMs; }
  uint32_t lastErrorMs() const { return _lastErrorMs; }
  Status lastError() const { return _lastError; }
  uint8_t consecutiveFailures() const { return _consecutiveFailures; }
  uint32_t totalFailures() const { return _totalFailures; }
  uint32_t totalSuccess() const { return _totalSuccess; }

  // Measurement API
  Status requestMeasurement();
  bool measurementReady() const { return _measurementReady; }
  Status getMeasurement(Measurement& out);
  Status getRawMeasurement(RawMeasurement& out) const;

  // Direct read API
  Status readAccelRaw(RawAxes& out);
  Status readGyroRaw(RawAxes& out);
  Status readTemperatureRaw(int16_t& out);
  Status readAllRaw(RawMeasurement& out);
  Axes convertAccel(const RawAxes& raw) const;
  Axes convertGyro(const RawAxes& raw) const;
  float convertTemperature(int16_t raw) const;

  // Core configuration
  Status setAccelOdr(Odr odr);
  Status setGyroOdr(Odr odr);
  Status setAccelFs(AccelFs fs);
  Status setGyroFs(GyroFs fs);
  Status getAccelOdr(Odr& out) const;
  Status getGyroOdr(Odr& out) const;
  Status getAccelFs(AccelFs& out) const;
  Status getGyroFs(GyroFs& out) const;
  Status softReset();
  Status boot();
  Status readWhoAmI(uint8_t& id);
  Status readStatusReg(uint8_t& status);
  Status isAccelDataReady(bool& ready);
  Status isGyroDataReady(bool& ready);
  Status isTempDataReady(bool& ready);

  // Sensitivity helpers
  float accelSensitivity() const;
  float gyroSensitivity() const;

  // Power and filter control
  Status setAccelPowerMode(AccelPowerMode mode);
  Status getAccelPowerMode(AccelPowerMode& out) const;
  Status setGyroPowerMode(GyroPowerMode mode);
  Status getGyroPowerMode(GyroPowerMode& out) const;
  Status setGyroSleepEnabled(bool enabled);
  Status getGyroSleepEnabled(bool& enabled) const;
  Status setAccelFilterConfig(const AccelFilterConfig& config);
  Status getAccelFilterConfig(AccelFilterConfig& out) const;
  Status setGyroFilterConfig(const GyroFilterConfig& config);
  Status getGyroFilterConfig(GyroFilterConfig& out) const;

  // Timestamp and embedded functions
  Status setTimestampEnabled(bool enabled);
  Status getTimestampEnabled(bool& enabled) const;
  Status setTimestampHighResolution(bool enabled);
  Status getTimestampHighResolution(bool& enabled) const;
  Status readTimestamp(uint32_t& out);
  Status resetTimestamp();
  Status setPedometerEnabled(bool enabled);
  Status getPedometerEnabled(bool& enabled) const;
  Status setSignificantMotionEnabled(bool enabled);
  Status getSignificantMotionEnabled(bool& enabled) const;
  Status setTiltEnabled(bool enabled);
  Status getTiltEnabled(bool& enabled) const;
  Status setWristTiltEnabled(bool enabled);
  Status getWristTiltEnabled(bool& enabled) const;
  Status readStepCounter(uint16_t& out);
  Status readStepTimestamp(uint16_t& out);
  Status resetStepCounter();

  // Offsets and FIFO
  Status setAccelOffsetWeight(AccelOffsetWeight weight);
  Status getAccelOffsetWeight(AccelOffsetWeight& out) const;
  Status setAccelUserOffset(const AccelUserOffset& offset);
  Status getAccelUserOffset(AccelUserOffset& out) const;
  Status configureFifo(const FifoConfig& config);
  Status getFifoConfig(FifoConfig& out) const;
  Status readFifoStatus(FifoStatus& out);
  Status readFifoWord(uint16_t& out);

  // Register and source access
  Status readRegisterValue(uint8_t reg, uint8_t& value);
  Status writeRegisterValue(uint8_t reg, uint8_t value);
  Status readRegisterBlock(uint8_t startReg, uint8_t* buf, size_t len);
  Status refreshCachedConfig();
  Status readWakeUpSource(uint8_t& value);
  Status readTapSource(uint8_t& value);
  Status read6dSource(uint8_t& value);
  Status readFunctionSource1(uint8_t& value);
  Status readFunctionSource2(uint8_t& value);
  Status readWristTiltStatus(uint8_t& value);

private:
  // Transport wrappers
  Status _i2cWriteReadRaw(const uint8_t* txBuf, size_t txLen,
                          uint8_t* rxBuf, size_t rxLen);
  Status _i2cWriteRaw(const uint8_t* buf, size_t len);
  Status _i2cWriteReadTracked(const uint8_t* txBuf, size_t txLen,
                              uint8_t* rxBuf, size_t rxLen);
  Status _i2cWriteTracked(const uint8_t* buf, size_t len);

  // Register access
  Status readRegs(uint8_t startReg, uint8_t* buf, size_t len);
  Status writeRegs(uint8_t startReg, const uint8_t* buf, size_t len);
  Status readRegister(uint8_t reg, uint8_t& value);
  Status writeRegister(uint8_t reg, uint8_t value);
  Status _readRegisterRaw(uint8_t reg, uint8_t& value);
  Status _updateRegister(uint8_t reg, uint8_t mask, uint8_t value);

  // Health management
  Status _updateHealth(const Status& st);

  // Internal helpers
  Status _applyConfig();
  Status _readRawAll();
  uint32_t _nowMs() const;
  static uint8_t _buildCtrl1Xl(Odr odr, AccelFs fs);
  static uint8_t _buildCtrl2G(Odr odr, GyroFs fs);
  uint8_t _buildCtrl4C() const;
  uint8_t _buildCtrl6C() const;
  uint8_t _buildCtrl7G() const;
  uint8_t _buildCtrl8Xl() const;
  uint8_t _buildCtrl10C() const;
  uint8_t _buildWakeUpDur() const;
  uint8_t _buildFifoCtrl2() const;
  uint8_t _buildFifoCtrl3() const;
  uint8_t _buildFifoCtrl4() const;
  uint8_t _buildFifoCtrl5() const;

  // State
  Config _config;
  bool _initialized = false;
  DriverState _driverState = DriverState::UNINIT;

  // Health counters
  uint32_t _lastOkMs = 0;
  uint32_t _lastErrorMs = 0;
  Status _lastError = Status::Ok();
  uint8_t _consecutiveFailures = 0;
  uint32_t _totalFailures = 0;
  uint32_t _totalSuccess = 0;

  // Measurement state
  bool _measurementRequested = false;
  bool _measurementReady = false;
  bool _hasSample = false;
  RawMeasurement _rawMeasurement;

  // Managed runtime configuration
  AccelPowerMode _accelPowerMode = AccelPowerMode::HIGH_PERFORMANCE;
  GyroPowerMode _gyroPowerMode = GyroPowerMode::HIGH_PERFORMANCE;
  bool _gyroSleepEnabled = false;
  AccelFilterConfig _accelFilterConfig;
  GyroFilterConfig _gyroFilterConfig;
  bool _timestampEnabled = false;
  bool _timestampHighResolution = false;
  bool _pedometerEnabled = false;
  bool _significantMotionEnabled = false;
  bool _tiltEnabled = false;
  bool _wristTiltEnabled = false;
  AccelOffsetWeight _accelOffsetWeight = AccelOffsetWeight::MG_1;
  AccelUserOffset _accelUserOffset;
  FifoConfig _fifoConfig;
};

}  // namespace LSM6DS3TR
