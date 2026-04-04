/// @file LSM6DS3TR.h
/// @brief Main driver class for LSM6DS3TR-C IMU
#pragma once

#include <cstddef>
#include <cstdint>
#include "LSM6DS3TR/Status.h"
#include "LSM6DS3TR/Config.h"
#include "LSM6DS3TR/CommandTable.h"
#include "LSM6DS3TR/Version.h"

namespace LSM6DS3TR {

/// Driver state for health monitoring
enum class DriverState : uint8_t {
  UNINIT,    ///< begin() not called or end() called
  READY,     ///< Operational, consecutiveFailures == 0
  DEGRADED,  ///< 1 <= consecutiveFailures < offlineThreshold
  OFFLINE    ///< consecutiveFailures >= offlineThreshold
};

/// 3-axis raw sample (16-bit signed per axis)
struct RawAxes {
  int16_t x = 0;  ///< X-axis raw ADC value
  int16_t y = 0;  ///< Y-axis raw ADC value
  int16_t z = 0;  ///< Z-axis raw ADC value
};

/// 3-axis float sample in physical units
struct Axes {
  float x = 0.0f;  ///< X-axis value
  float y = 0.0f;  ///< Y-axis value
  float z = 0.0f;  ///< Z-axis value
};

/// Combined IMU measurement result (float, physical units)
struct Measurement {
  Axes accel;          ///< Acceleration in g
  Axes gyro;           ///< Angular rate in dps (degrees per second)
  float temperatureC = 0.0f; ///< Temperature in Celsius
};

/// Combined raw IMU sample (16-bit signed per axis)
struct RawMeasurement {
  RawAxes accel;       ///< Raw accelerometer ADC values
  RawAxes gyro;        ///< Raw gyroscope ADC values
  int16_t temperature = 0; ///< Raw temperature ADC value
};

/// LSM6DS3TR-C driver class
class LSM6DS3TR {
public:
  // =========================================================================
  // Lifecycle
  // =========================================================================

  /// Initialize the driver with configuration
  /// @param config Configuration including transport callbacks
  /// @return Status::Ok() on success, error otherwise
  Status begin(const Config& config);

  /// Process pending operations (call regularly from loop)
  /// @param nowMs Current timestamp in milliseconds
  void tick(uint32_t nowMs);

  /// Shutdown the driver and release resources
  void end();

  // =========================================================================
  // Diagnostics
  // =========================================================================

  /// Check if device is present on the bus (no health tracking)
  /// @return Status::Ok() if device responds with correct WHO_AM_I
  Status probe();

  /// Attempt to recover from DEGRADED/OFFLINE state
  /// @return Status::Ok() if device now responsive, error otherwise
  Status recover();

  // =========================================================================
  // Driver State
  // =========================================================================

  /// Get current driver state
  DriverState state() const { return _driverState; }

  /// Check if driver is ready for operations
  bool isOnline() const {
    return _driverState == DriverState::READY ||
           _driverState == DriverState::DEGRADED;
  }

  // =========================================================================
  // Health Tracking
  // =========================================================================

  /// Timestamp of last successful I2C operation
  uint32_t lastOkMs() const { return _lastOkMs; }

  /// Timestamp of last failed I2C operation
  uint32_t lastErrorMs() const { return _lastErrorMs; }

  /// Most recent error status
  Status lastError() const { return _lastError; }

  /// Consecutive failures since last success
  uint8_t consecutiveFailures() const { return _consecutiveFailures; }

  /// Total failure count (lifetime)
  uint32_t totalFailures() const { return _totalFailures; }

  /// Total success count (lifetime)
  uint32_t totalSuccess() const { return _totalSuccess; }

  // =========================================================================
  // Measurement API
  // =========================================================================

  /// Request a measurement (non-blocking)
  /// Marks intent to read on next data-ready (via tick())
  /// Returns IN_PROGRESS if request accepted, BUSY if already pending
  /// @return Status with IN_PROGRESS on success
  Status requestMeasurement();

  /// Check if measurement is ready to read
  bool measurementReady() const { return _measurementReady; }

  /// Get measurement result (float, physical units)
  /// Returns MEASUREMENT_NOT_READY if not available
  /// Clears ready flag after successful read
  /// @param out Measurement result
  /// @return Status::Ok() on success
  Status getMeasurement(Measurement& out);

  /// Get raw ADC values from last measurement
  /// Returns MEASUREMENT_NOT_READY until at least one sample captured
  /// @param out Raw measurement result
  /// @return Status::Ok() on success
  Status getRawMeasurement(RawMeasurement& out) const;

  // =========================================================================
  // Direct Read API (blocking)
  // =========================================================================

  /// Read accelerometer data (blocking, tracked)
  /// @param out Raw 3-axis accelerometer values
  /// @return Status::Ok() on success
  Status readAccelRaw(RawAxes& out);

  /// Read gyroscope data (blocking, tracked)
  /// @param out Raw 3-axis gyroscope values
  /// @return Status::Ok() on success
  Status readGyroRaw(RawAxes& out);

  /// Read temperature (blocking, tracked)
  /// @param out Raw 16-bit temperature value
  /// @return Status::Ok() on success
  Status readTemperatureRaw(int16_t& out);

  /// Read all sensor data in a single burst (blocking, tracked)
  /// @param out Raw measurement (temp + gyro + accel)
  /// @return Status::Ok() on success
  Status readAllRaw(RawMeasurement& out);

  /// Convert raw accelerometer to g
  Axes convertAccel(const RawAxes& raw) const;

  /// Convert raw gyroscope to dps
  Axes convertGyro(const RawAxes& raw) const;

  /// Convert raw temperature to Celsius
  float convertTemperature(int16_t raw) const;

  // =========================================================================
  // Configuration
  // =========================================================================

  /// Set accelerometer ODR
  Status setAccelOdr(Odr odr);

  /// Set gyroscope ODR
  Status setGyroOdr(Odr odr);

  /// Set accelerometer full-scale
  Status setAccelFs(AccelFs fs);

  /// Set gyroscope full-scale
  Status setGyroFs(GyroFs fs);

  /// Get accelerometer ODR (cached)
  Status getAccelOdr(Odr& out) const;

  /// Get gyroscope ODR (cached)
  Status getGyroOdr(Odr& out) const;

  /// Get accelerometer full-scale (cached)
  Status getAccelFs(AccelFs& out) const;

  /// Get gyroscope full-scale (cached)
  Status getGyroFs(GyroFs& out) const;

  /// Software reset — resets all registers to defaults, re-applies config
  Status softReset();

  /// Read WHO_AM_I register
  Status readWhoAmI(uint8_t& id);

  /// Read status register
  Status readStatusReg(uint8_t& status);

  /// Check if accelerometer data is ready
  Status isAccelDataReady(bool& ready);

  /// Check if gyroscope data is ready
  Status isGyroDataReady(bool& ready);

  /// Check if temperature data is ready
  Status isTempDataReady(bool& ready);

  // =========================================================================
  // Sensitivity Helpers
  // =========================================================================

  /// Get accelerometer sensitivity in mg/LSB for current FS
  float accelSensitivity() const;

  /// Get gyroscope sensitivity in mdps/LSB for current FS
  float gyroSensitivity() const;

private:
  // =========================================================================
  // Transport Wrappers
  // =========================================================================

  /// Raw I2C write-read (no health tracking)
  Status _i2cWriteReadRaw(const uint8_t* txBuf, size_t txLen,
                          uint8_t* rxBuf, size_t rxLen);
  /// Raw I2C write (no health tracking)
  Status _i2cWriteRaw(const uint8_t* buf, size_t len);
  /// Tracked I2C write-read (updates health)
  Status _i2cWriteReadTracked(const uint8_t* txBuf, size_t txLen,
                              uint8_t* rxBuf, size_t rxLen);
  /// Tracked I2C write (updates health)
  Status _i2cWriteTracked(const uint8_t* buf, size_t len);

  // =========================================================================
  // Register Access
  // =========================================================================

  /// Read registers (uses tracked path)
  Status readRegs(uint8_t startReg, uint8_t* buf, size_t len);
  /// Write registers (uses tracked path)
  Status writeRegs(uint8_t startReg, const uint8_t* buf, size_t len);
  /// Read single register (uses tracked path)
  Status readRegister(uint8_t reg, uint8_t& value);
  /// Write single register (uses tracked path)
  Status writeRegister(uint8_t reg, uint8_t value);
  /// Read single register (raw path, no health tracking)
  Status _readRegisterRaw(uint8_t reg, uint8_t& value);

  // =========================================================================
  // Health Management
  // =========================================================================

  /// Update health counters and state based on operation result
  /// Called ONLY from tracked transport wrappers
  Status _updateHealth(const Status& st);

  // =========================================================================
  // Internal
  // =========================================================================

  /// Apply initial sensor configuration after begin()
  Status _applyConfig();
  /// Read all raw sensor data (temp + gyro + accel) into _rawMeasurement
  Status _readRawAll();
  /// Get current timestamp via config callback or return 0
  uint32_t _nowMs() const;

  /// Build CTRL1_XL register value from ODR and FS
  static uint8_t _buildCtrl1Xl(Odr odr, AccelFs fs);
  /// Build CTRL2_G register value from ODR and FS
  static uint8_t _buildCtrl2G(Odr odr, GyroFs fs);

  // =========================================================================
  // State
  // =========================================================================

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
};

} // namespace LSM6DS3TR
