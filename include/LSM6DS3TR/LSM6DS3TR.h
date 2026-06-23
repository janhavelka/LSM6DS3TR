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

/// @brief Driver state for health monitoring.
enum class DriverState : uint8_t {
  UNINIT,    ///< begin() not called or end() called
  READY,     ///< Operational, consecutiveFailures == 0
  DEGRADED,  ///< 1 <= consecutiveFailures < offlineThreshold
  OFFLINE    ///< consecutiveFailures >= offlineThreshold
};

/// @brief Three-axis raw sample.
struct RawAxes {
  int16_t x = 0;  ///< Raw X-axis code
  int16_t y = 0;  ///< Raw Y-axis code
  int16_t z = 0;  ///< Raw Z-axis code
};

/// @brief Three-axis physical sample.
struct Axes {
  float x = 0.0f;  ///< X-axis value
  float y = 0.0f;  ///< Y-axis value
  float z = 0.0f;  ///< Z-axis value
};

/// @brief Converted accelerometer, gyroscope, and temperature sample.
struct Measurement {
  Axes accel;                 ///< Acceleration in g
  Axes gyro;                  ///< Angular rate in dps
  float temperatureC = 0.0f;  ///< Die temperature in degrees Celsius
};

/// @brief Raw accelerometer, gyroscope, and temperature sample.
struct RawMeasurement {
  RawAxes accel;          ///< Raw accelerometer axes
  RawAxes gyro;           ///< Raw gyroscope axes
  int16_t temperature = 0; ///< Raw temperature code
};

/// @brief Accelerometer user-offset register values.
struct AccelUserOffset {
  int8_t x = 0;  ///< X-axis offset register value
  int8_t y = 0;  ///< Y-axis offset register value
  int8_t z = 0;  ///< Z-axis offset register value
};

/// @brief High-level accelerometer filter state managed by the driver.
struct AccelFilterConfig {
  bool lpf2Enabled = false;           ///< Enable LPF2 path
  bool highPassSlopeEnabled = false;  ///< Enable high-pass/slope filter path
  bool lowPassOn6d = false;           ///< Use low-pass filtered data for 6D
};

/// @brief High-level gyroscope filter state managed by the driver.
struct GyroFilterConfig {
  bool lpf1Enabled = false;  ///< Enable LPF1 path
  bool highPassEnabled = false;  ///< Enable gyroscope high-pass filter
  GyroHpfMode highPassMode = GyroHpfMode::HZ_0_0081;  ///< High-pass cutoff
};

/// @brief FIFO configuration managed by the driver.
struct FifoConfig {
  uint16_t threshold = 0;  ///< FIFO threshold in words
  Odr odr = Odr::POWER_DOWN;  ///< FIFO output data rate
  FifoMode mode = FifoMode::BYPASS;  ///< FIFO operating mode
  FifoDecimation accelDecimation = FifoDecimation::DISABLED;  ///< Accel batching decimation
  FifoDecimation gyroDecimation = FifoDecimation::DISABLED;   ///< Gyro batching decimation
  bool stopOnThreshold = false;  ///< Stop collecting at threshold
  bool onlyHighData = false;     ///< Store only high data bytes
  bool storeTemperature = false; ///< Store temperature samples
  bool storeTimestampStep = false; ///< Store timestamp-step samples
};

/// @brief Parsed FIFO status.
struct FifoStatus {
  uint16_t unreadWords = 0;  ///< Number of unread FIFO words
  uint16_t pattern = 0;      ///< FIFO pattern index
  bool watermark = false;    ///< FIFO threshold reached
  bool overrun = false;      ///< FIFO overrun flag
  bool fullSmart = false;    ///< Smart-full flag
  bool empty = true;         ///< FIFO empty flag
};

/// @brief Decoded STATUS_REG data-ready flags.
struct StatusReg {
  uint8_t raw = 0;  ///< Raw STATUS_REG value
  bool accelDataReady = false;  ///< XLDA flag
  bool gyroDataReady = false;   ///< GDA flag
  bool tempDataReady = false;   ///< TDA flag
};

/// @brief Sensor-hub output window, SENSORHUB1_REG through SENSORHUB12_REG.
struct SensorHubData {
  uint8_t bytes[12] = {};  ///< Bytes read from SENSORHUB1_REG onward
  uint8_t count = 0;       ///< Number of valid bytes in bytes[]
};

/// @brief Snapshot of cached configuration and runtime state without I2C.
struct SettingsSnapshot {
  bool initialized = false;                 ///< True after begin() succeeds
  DriverState state = DriverState::UNINIT;  ///< Current driver health state
  uint8_t i2cAddress = 0x6A;                ///< Active 7-bit I2C address
  uint32_t i2cTimeoutMs = 0;                ///< Active I2C timeout
  uint8_t offlineThreshold = 0;             ///< Failure threshold for OFFLINE
  bool hasNowMsHook = false;                ///< True when Config::nowMs is set
  Odr odrXl = Odr::POWER_DOWN;              ///< Cached accelerometer ODR
  Odr odrG = Odr::POWER_DOWN;               ///< Cached gyroscope ODR
  AccelFs fsXl = AccelFs::G_2;              ///< Cached accelerometer full-scale
  GyroFs fsG = GyroFs::DPS_250;             ///< Cached gyroscope full-scale
  bool bdu = true;                          ///< Cached block-data-update setting
  AccelPowerMode accelPowerMode = AccelPowerMode::HIGH_PERFORMANCE; ///< Cached accel power mode
  GyroPowerMode gyroPowerMode = GyroPowerMode::HIGH_PERFORMANCE;    ///< Cached gyro power mode
  bool gyroSleepEnabled = false;            ///< Cached gyroscope sleep state
  AccelFilterConfig accelFilter = {};       ///< Cached accelerometer filter configuration
  GyroFilterConfig gyroFilter = {};         ///< Cached gyroscope filter configuration
  bool timestampEnabled = false;            ///< Cached timestamp enable state
  bool timestampHighResolution = false;     ///< Cached timestamp resolution state
  bool pedometerEnabled = false;            ///< Cached pedometer enable state
  bool significantMotionEnabled = false;    ///< Cached significant-motion enable state
  bool tiltEnabled = false;                 ///< Cached tilt enable state
  bool wristTiltEnabled = false;            ///< Cached wrist-tilt enable state
  AccelOffsetWeight accelOffsetWeight = AccelOffsetWeight::MG_1; ///< Cached offset weight
  AccelUserOffset accelUserOffset = {};     ///< Cached hardware accel offsets
  FifoConfig fifo = {};                     ///< Cached FIFO configuration
  Axes accelBias = {};                      ///< Software accel bias
  Axes gyroBias = {};                       ///< Software gyro bias
  bool measurementPending = false;          ///< True while a request is waiting for tick()
  bool measurementReady = false;            ///< True when getMeasurement() can consume a sample
  bool hasSample = false;                   ///< True after at least one sample has been cached
  uint32_t sampleTimestampMs = 0;           ///< Timestamp of the last cached sample
  RawMeasurement rawMeasurement = {};       ///< Last cached raw sample
  bool cachedConfigDirty = false;           ///< True when cached config may differ from chip registers
};

/// @brief Accelerometer and gyroscope self-test result.
struct SelfTestResult {
  Axes accelBaseline;      ///< Average accelerometer baseline in g
  Axes accelStimulus;      ///< Average accelerometer self-test response in g
  Axes accelDelta;         ///< Absolute accelerometer delta in g
  Axes gyroBaseline;       ///< Average gyroscope baseline in dps
  Axes gyroStimulus;       ///< Average gyroscope self-test response in dps
  Axes gyroDelta;          ///< Absolute gyroscope delta in dps
  bool accelPass = false;  ///< True when all accel axes are in datasheet range
  bool gyroPass = false;   ///< True when all gyro axes are in datasheet range
};

/// @brief Managed synchronous LSM6DS3TR-C IMU driver.
class LSM6DS3TR {
public:
  // Lifecycle
  /// @brief Initialize the driver, verify WHO_AM_I, and apply configuration.
  /// @param config Transport, timing, and sensor configuration.
  /// @return OK on success; otherwise a setup error status.
  Status begin(const Config& config);

  /// @brief Complete a requested asynchronous measurement when data is ready.
  /// @param nowMs Current monotonic time in milliseconds.
  void tick(uint32_t nowMs);

  /// @brief Advance the active chunked job by at most @p maxInstructions I2C transfers.
  /// @param nowMs Current monotonic time in milliseconds. Staged sample jobs use
  /// this value for ready deadlines and sample timestamps.
  /// @param maxInstructions Maximum register read, register write, or burst read instructions.
  /// Passing 0 makes no progress and does not arm deadlines.
  /// @return OK when the job completes, IN_PROGRESS while work remains, or a terminal error.
  Status poll(uint32_t nowMs, uint8_t maxInstructions = 1);

  /// @brief Check whether a chunked job is active.
  bool pollBusy() const;

  /// @brief Most recent poll progress or terminal status.
  Status lastPollStatus() const { return _lastPollStatus; }

  /// @brief Clear runtime state and transition to UNINIT.
  void end();

  /// @brief Check if begin() completed successfully and end() has not been called.
  bool isInitialized() const { return _initialized; }

  /// @brief Get the active configuration snapshot.
  const Config& getConfig() const { return _config; }

  // Diagnostics
  /// @brief Probe WHO_AM_I without health tracking.
  /// @return OK if the expected device ID is read.
  Status probe();

  /// @brief Attempt manual recovery by re-reading WHO_AM_I.
  /// @return OK on recovery; otherwise a tracked error status.
  Status recover();

  /// @brief Start a chunked software reset job.
  /// @return IN_PROGRESS when scheduled; call poll() to advance.
  Status startSoftReset();

  /// @brief Start a chunked memory boot job.
  /// @return IN_PROGRESS when scheduled; call poll() to advance.
  Status startBoot();

  /// @brief Start a chunked cached-configuration refresh job.
  /// @return IN_PROGRESS when scheduled; call poll() to advance.
  Status startRefreshCachedConfig();

  /// @brief Run the bounded blocking accelerometer and gyroscope self-test.
  ///
  /// The driver configures the datasheet self-test modes, averages @p samples
  /// per phase, checks datasheet response thresholds, and attempts to restore
  /// affected registers before returning.
  /// @param out Result structure populated with baseline, stimulus, and deltas.
  /// @param samples Average count per phase (1-100).
  /// @return OK on pass; SELF_TEST_FAIL on threshold failure; otherwise a
  ///         register, timeout, or validation status.
  Status runSelfTest(SelfTestResult& out, uint16_t samples = 5);

  // Driver state
  /// @brief Get current driver state.
  /// @return Driver state.
  DriverState state() const { return _driverState; }

  /// @brief Alias for state() used by shared diagnostics.
  DriverState driverState() const { return state(); }

  /// @brief Check whether normal I2C operations are allowed.
  /// @return true in READY or DEGRADED state.
  bool isOnline() const {
    return _driverState == DriverState::READY ||
           _driverState == DriverState::DEGRADED;
  }

  // Health tracking
  /// @brief Timestamp of last successful tracked I2C operation.
  uint32_t lastOkMs() const { return _lastOkMs; }

  /// @brief Timestamp of last failed tracked I2C operation.
  uint32_t lastErrorMs() const { return _lastErrorMs; }

  /// @brief Most recent tracked I2C error.
  Status lastError() const { return _lastError; }

  /// @brief Consecutive tracked I2C failures since the last success.
  uint8_t consecutiveFailures() const { return _consecutiveFailures; }

  /// @brief Lifetime tracked I2C failure count.
  uint32_t totalFailures() const { return _totalFailures; }

  /// @brief Lifetime tracked I2C success count.
  uint32_t totalSuccess() const { return _totalSuccess; }

  /// @brief True when cached configuration mirrors may differ from device registers.
  bool cachedConfigDirty() const { return _cachedConfigDirty; }

  // Measurement API
  /// @brief Request a combined sample to be completed by tick().
  /// @return IN_PROGRESS when the request is accepted.
  Status requestMeasurement();

  /// @brief Request a combined sample, optionally skipping the STATUS_REG readiness read.
  ///
  /// Ready-checked jobs arm their timeout on the first positive-budget poll()
  /// that executes the status-read step, not at request time.
  /// @param checkReady true to require a visible status-read instruction before the raw burst.
  /// @return IN_PROGRESS when the request is accepted.
  Status requestMeasurement(bool checkReady);

  /// @brief Check if the requested measurement is ready.
  /// @return true when getMeasurement() can return a fresh requested sample.
  bool measurementReady() const { return _measurementReady; }

  /// @brief True after at least one sample has been cached.
  bool hasSample() const { return _hasSample; }

  /// @brief Timestamp of the last cached sample, or 0 if none exists.
  ///
  /// Poll-completed samples use the nowMs value passed to the raw-burst poll.
  /// Direct blocking reads use Config::nowMs when present and 0 when absent.
  uint32_t sampleTimestampMs() const { return _sampleTimestampMs; }

  /// @brief Age of the cached sample in milliseconds.
  /// @param nowMs Current monotonic timestamp in milliseconds.
  /// @return `nowMs - sampleTimestampMs()` when a sample exists, otherwise 0.
  uint32_t sampleAgeMs(uint32_t nowMs) const {
    return _hasSample ? (nowMs - _sampleTimestampMs) : 0;
  }

  /// @brief Get the most recent converted measurement.
  /// @param out Converted sample in g, dps, and degrees Celsius.
  /// @return OK on success; MEASUREMENT_NOT_READY if no sample is cached.
  Status getMeasurement(Measurement& out);

  /// @brief Get the most recent raw measurement.
  /// @param out Raw sample.
  /// @return OK on success; MEASUREMENT_NOT_READY if no sample is cached.
  Status getRawMeasurement(RawMeasurement& out) const;

  /// @brief Get cached configuration and runtime state without I2C.
  /// @param out Snapshot to populate.
  /// @return Status::Ok() always.
  Status getSettings(SettingsSnapshot& out) const;

  /// @brief Return a by-value settings snapshot.
  SettingsSnapshot settings() const {
    SettingsSnapshot out;
    (void)getSettings(out);
    return out;
  }

  // Direct read API
  /// @brief Read raw accelerometer axes.
  /// @param out Raw accelerometer codes.
  /// @return Status from the burst read.
  Status readAccelRaw(RawAxes& out);

  /// @brief Read raw gyroscope axes.
  /// @param out Raw gyroscope codes.
  /// @return Status from the burst read.
  Status readGyroRaw(RawAxes& out);

  /// @brief Read raw temperature code.
  /// @param out Raw temperature code.
  /// @return Status from the register read.
  Status readTemperatureRaw(int16_t& out);

  /// @brief Read raw temperature, gyroscope, and accelerometer data.
  /// @param out Raw combined sample.
  /// @return Status from the burst read.
  Status readAllRaw(RawMeasurement& out);

  /// @brief Convert raw accelerometer axes to g.
  /// @param raw Raw accelerometer codes.
  /// @return Converted axes in g.
  Axes convertAccel(const RawAxes& raw) const;

  /// @brief Convert raw gyroscope axes to dps.
  /// @param raw Raw gyroscope codes.
  /// @return Converted axes in dps.
  Axes convertGyro(const RawAxes& raw) const;

  /// @brief Convert raw temperature code to degrees Celsius.
  /// @param raw Raw temperature code.
  /// @return Temperature in degrees Celsius.
  float convertTemperature(int16_t raw) const;

  /// @name Software Bias Calibration
  /// @brief Software-level bias compensation for accel and gyro.
  ///
  /// The accelerometer hardware offset registers (0x73-0x75) provide coarse
  /// offset correction at 8-bit resolution.  Software bias extends this with
  /// float-precision offsets subtracted during conversion.
  ///
  /// The gyroscope has NO hardware offset registers on the LSM6DS3TR-C;
  /// software bias is the only way to remove zero-rate offset.
  ///
  /// Bias values are stored in physical units (g for accel, dps for gyro).
  /// They are automatically subtracted in getMeasurement() and can be
  /// manually applied via correctAccel() / correctGyro().
  /// The low-level convertAccel() / convertGyro() are NOT affected.
  /// @{

  /// Set accelerometer software bias (g).  Subtracted from converted values.
  void setAccelBias(const Axes& bias);

  /// Get current accelerometer software bias (g).
  Axes accelBias() const;

  /// Set gyroscope software bias (dps).  Subtracted from converted values.
  void setGyroBias(const Axes& bias);

  /// Get current gyroscope software bias (dps).
  Axes gyroBias() const;

  /// Subtract current accel bias from a converted Axes value in-place.
  void correctAccel(Axes& inout) const;

  /// Subtract current gyro bias from a converted Axes value in-place.
  void correctGyro(Axes& inout) const;

  /// Capture accelerometer bias by averaging @p samples readings at rest.
  ///
  /// The sensor must be stationary with Z-axis pointing up (+1 g on Z).
  /// Blocks for approximately (samples / accelODR) seconds.
  /// Each sample waits for the XLDA data-ready flag with a bounded deadline
  /// and finite polling cap, so a stalled time source cannot spin forever.
  ///
  /// On success the bias is auto-applied (equivalent to setAccelBias(out))
  /// and returned via @p out so the caller can persist it.
  /// On quality failure the previous bias is left unchanged.
  ///
  /// @param samples  Number of readings to average (1-10000).
  /// @param out      Computed bias in g.
  /// @return OK on success; INVALID_PARAM if samples is 0 or > 10000;
  ///         CALIBRATION_UNSTABLE / CALIBRATION_ORIENTATION on quality failure;
  ///         TIMEOUT if data-ready never arrives; NOT_INITIALIZED / I2C errors
  ///         propagated from reads.
  Status captureAccelBias(uint16_t samples, Axes& out);

  /// @brief Start a chunked accelerometer bias capture diagnostic job.
  /// @param samples Number of readings to average (1-10000).
  /// @return IN_PROGRESS when scheduled; call poll() to advance.
  Status startAccelBiasCapture(uint16_t samples);

  /// Capture gyroscope zero-rate bias by averaging @p samples readings at rest.
  ///
  /// The sensor must be stationary (no rotation).
  /// Blocks for approximately (samples / gyroODR) seconds.
  /// Each sample waits for the GDA data-ready flag with a bounded deadline
  /// and finite polling cap, so a stalled time source cannot spin forever.
  ///
  /// On success the bias is auto-applied (equivalent to setGyroBias(out))
  /// and returned via @p out so the caller can persist it.
  /// On quality failure the previous bias is left unchanged.
  ///
  /// @param samples  Number of readings to average (1-10000).
  /// @param out      Computed bias in dps.
  /// @return OK on success; INVALID_PARAM if samples is 0 or > 10000;
  ///         CALIBRATION_UNSTABLE on quality failure;
  ///         TIMEOUT if data-ready never arrives; NOT_INITIALIZED / I2C errors
  ///         propagated from reads.
  Status captureGyroBias(uint16_t samples, Axes& out);

  /// @brief Start a chunked gyroscope bias capture diagnostic job.
  /// @param samples Number of readings to average (1-10000).
  /// @return IN_PROGRESS when scheduled; call poll() to advance.
  Status startGyroBiasCapture(uint16_t samples);

  /// @}

  // Core configuration
  /// @brief Set accelerometer output data rate.
  /// @param odr Output data rate.
  /// @return Status from validation and register update.
  Status setAccelOdr(Odr odr);

  /// @brief Set gyroscope output data rate.
  /// @param odr Output data rate.
  /// @return Status from validation and register update.
  Status setGyroOdr(Odr odr);

  /// @brief Set accelerometer full-scale range.
  /// @param fs Full-scale range.
  /// @return Status from validation and register update.
  Status setAccelFs(AccelFs fs);

  /// @brief Set gyroscope full-scale range.
  /// @param fs Full-scale range.
  /// @return Status from validation and register update.
  Status setGyroFs(GyroFs fs);

  /// @brief Get cached accelerometer output data rate.
  /// @param out Output data rate.
  /// @return OK on success.
  Status getAccelOdr(Odr& out) const;

  /// @brief Get cached gyroscope output data rate.
  /// @param out Output data rate.
  /// @return OK on success.
  Status getGyroOdr(Odr& out) const;

  /// @brief Get cached accelerometer full-scale range.
  /// @param out Full-scale range.
  /// @return OK on success.
  Status getAccelFs(AccelFs& out) const;

  /// @brief Get cached gyroscope full-scale range.
  /// @param out Full-scale range.
  /// @return OK on success.
  Status getGyroFs(GyroFs& out) const;

  /// @brief Issue software reset and poll SW_RESET with a bounded deadline.
  /// @return OK if reset completes before the deadline.
  Status softReset();

  /// @brief Issue memory boot command.
  /// @return Status from CTRL3_C update.
  Status boot();

  /// @brief Read WHO_AM_I.
  /// @param id Raw WHO_AM_I value.
  /// @return Status from register read.
  Status readWhoAmI(uint8_t& id);

  /// @brief Read raw STATUS_REG.
  /// @param status Raw STATUS_REG value.
  /// @return Status from register read.
  Status readStatusReg(uint8_t& status);

  /// @brief Read and decode STATUS_REG data-ready flags.
  /// @param out Decoded status flags.
  /// @return Status from register read.
  Status readStatus(StatusReg& out);

  /// @brief Read accelerometer data-ready flag.
  /// @param ready Set true when XLDA is set.
  /// @return Status from STATUS_REG read.
  Status isAccelDataReady(bool& ready);

  /// @brief Read gyroscope data-ready flag.
  /// @param ready Set true when GDA is set.
  /// @return Status from STATUS_REG read.
  Status isGyroDataReady(bool& ready);

  /// @brief Read temperature data-ready flag.
  /// @param ready Set true when TDA is set.
  /// @return Status from STATUS_REG read.
  Status isTempDataReady(bool& ready);

  // Sensitivity helpers
  /// @brief Get active accelerometer sensitivity.
  /// @return Sensitivity in g/LSB.
  float accelSensitivity() const;

  /// @brief Get active gyroscope sensitivity.
  /// @return Sensitivity in dps/LSB.
  float gyroSensitivity() const;

  // Power and filter control
  /// @brief Set accelerometer power mode.
  /// @param mode Power mode.
  /// @return Status from validation and register update.
  Status setAccelPowerMode(AccelPowerMode mode);

  /// @brief Get cached accelerometer power mode.
  /// @param out Power mode.
  /// @return OK on success.
  Status getAccelPowerMode(AccelPowerMode& out) const;

  /// @brief Set gyroscope power mode.
  /// @param mode Power mode.
  /// @return Status from validation and register update.
  Status setGyroPowerMode(GyroPowerMode mode);

  /// @brief Get cached gyroscope power mode.
  /// @param out Power mode.
  /// @return OK on success.
  Status getGyroPowerMode(GyroPowerMode& out) const;

  /// @brief Enable or disable gyroscope sleep mode.
  /// @param enabled true enables sleep mode.
  /// @return Status from register update.
  Status setGyroSleepEnabled(bool enabled);

  /// @brief Get cached gyroscope sleep enable state.
  /// @param enabled Output enable state.
  /// @return OK on success.
  Status getGyroSleepEnabled(bool& enabled) const;

  /// @brief Apply accelerometer filter configuration.
  /// @param config Filter configuration.
  /// @return Status from register updates.
  Status setAccelFilterConfig(const AccelFilterConfig& config);

  /// @brief Get cached accelerometer filter configuration.
  /// @param out Filter configuration.
  /// @return OK on success.
  Status getAccelFilterConfig(AccelFilterConfig& out) const;

  /// @brief Apply gyroscope filter configuration.
  /// @param config Filter configuration.
  /// @return Status from register updates.
  Status setGyroFilterConfig(const GyroFilterConfig& config);

  /// @brief Get cached gyroscope filter configuration.
  /// @param out Filter configuration.
  /// @return OK on success.
  Status getGyroFilterConfig(GyroFilterConfig& out) const;

  // Timestamp and embedded functions
  /// @brief Enable or disable the embedded timestamp counter.
  /// @param enabled true enables timestamping.
  /// @return Status from register update.
  Status setTimestampEnabled(bool enabled);

  /// @brief Get cached timestamp enable state.
  /// @param enabled Output enable state.
  /// @return OK on success.
  Status getTimestampEnabled(bool& enabled) const;

  /// @brief Enable or disable high-resolution timestamp mode.
  /// @param enabled true selects high-resolution timestamp mode.
  /// @return Status from register update.
  Status setTimestampHighResolution(bool enabled);

  /// @brief Get cached high-resolution timestamp state.
  /// @param enabled Output enable state.
  /// @return OK on success.
  Status getTimestampHighResolution(bool& enabled) const;

  /// @brief Read the 24-bit timestamp counter.
  /// @param out Timestamp counter value.
  /// @return Status from register burst read.
  Status readTimestamp(uint32_t& out);

  /// @brief Reset the timestamp counter.
  /// @return Status from register update.
  Status resetTimestamp();

  /// @brief Enable or disable pedometer function.
  ///
  /// Enabling requires accelerometer ODR >= 26 Hz. The driver sets the
  /// required embedded-function gate together with the pedometer bit in
  /// CTRL10_C.
  /// @param enabled true enables pedometer logic.
  /// @return OK on success; INVALID_PARAM when the current accel ODR is too low;
  ///         otherwise a register-update status.
  Status setPedometerEnabled(bool enabled);

  /// @brief Get cached pedometer enable state.
  /// @param enabled Output enable state.
  /// @return OK on success.
  Status getPedometerEnabled(bool& enabled) const;

  /// @brief Enable or disable significant-motion detection.
  ///
  /// Enabling requires accelerometer ODR >= 26 Hz. Threshold tuning is available
  /// through raw register APIs for applications that need non-default
  /// embedded-bank settings.
  /// @param enabled true enables significant-motion logic.
  /// @return OK on success; INVALID_PARAM when the current accel ODR is too low;
  ///         otherwise a register-update status.
  Status setSignificantMotionEnabled(bool enabled);

  /// @brief Get cached significant-motion enable state.
  /// @param enabled Output enable state.
  /// @return OK on success.
  Status getSignificantMotionEnabled(bool& enabled) const;

  /// @brief Enable or disable tilt detection.
  ///
  /// Enabling requires accelerometer ODR >= 26 Hz.
  /// @param enabled true enables tilt logic.
  /// @return OK on success; INVALID_PARAM when the current accel ODR is too low;
  ///         otherwise a register-update status.
  Status setTiltEnabled(bool enabled);

  /// @brief Get cached tilt enable state.
  /// @param enabled Output enable state.
  /// @return OK on success.
  Status getTiltEnabled(bool& enabled) const;

  /// @brief Enable or disable wrist-tilt detection.
  ///
  /// Enabling requires accelerometer ODR >= 26 Hz. Axis mask, threshold, and
  /// latency tuning remain available through raw register access.
  /// @param enabled true enables wrist-tilt logic.
  /// @return OK on success; INVALID_PARAM when the current accel ODR is too low;
  ///         otherwise a register-update status.
  Status setWristTiltEnabled(bool enabled);

  /// @brief Get cached wrist-tilt enable state.
  /// @param enabled Output enable state.
  /// @return OK on success.
  Status getWristTiltEnabled(bool& enabled) const;

  /// @brief Read the durable 16-bit pedometer step counter.
  ///
  /// The counter increments after the hardware pedometer debounce accepts a
  /// walking sequence. The transient STEP_DETECTED source bit may already be
  /// clear by the time software reads FUNC_SRC1 unless interrupt latching or
  /// routing is configured.
  /// @param out Step counter value.
  /// @return Status from register read.
  Status readStepCounter(uint16_t& out);

  /// @brief Read the timestamp captured when the last step was detected.
  /// @param out Step timestamp value.
  /// @return Status from register read.
  Status readStepTimestamp(uint16_t& out);

  /// @brief Reset the hardware step counter and clear PEDO_RST_STEP again.
  /// @return Status from register update.
  Status resetStepCounter();

  // Offsets and FIFO
  /// @brief Set accelerometer user-offset register weight.
  /// @param weight Offset LSB weight.
  /// @return Status from register update.
  Status setAccelOffsetWeight(AccelOffsetWeight weight);

  /// @brief Get cached accelerometer user-offset register weight.
  /// @param out Offset LSB weight.
  /// @return OK on success.
  Status getAccelOffsetWeight(AccelOffsetWeight& out) const;

  /// @brief Write accelerometer user-offset registers.
  /// @param offset Offset register values.
  /// @return Status from register writes.
  Status setAccelUserOffset(const AccelUserOffset& offset);

  /// @brief Get cached accelerometer user-offset register values.
  /// @param out Offset register values.
  /// @return OK on success.
  Status getAccelUserOffset(AccelUserOffset& out) const;

  /// @brief Configure FIFO mode, threshold, decimation, and stored data.
  /// @param config FIFO configuration.
  /// @return Status from validation and register writes.
  Status configureFifo(const FifoConfig& config);

  /// @brief Get cached FIFO configuration.
  /// @param out FIFO configuration.
  /// @return OK on success.
  Status getFifoConfig(FifoConfig& out) const;

  /// @brief Read and decode FIFO status registers.
  /// @param out FIFO status.
  /// @return Status from register burst read.
  Status readFifoStatus(FifoStatus& out);

  /// @brief Read one FIFO data word.
  /// @param out FIFO word.
  /// @return OK on success; FIFO_EMPTY when no unread words are available.
  Status readFifoWord(uint16_t& out);

  /// @brief Start a chunked FIFO drain that reads at most @p maxWords words.
  /// @return IN_PROGRESS when scheduled; call poll() to advance.
  Status startFifoDrain(uint16_t maxWords);

  /// @brief Number of FIFO words consumed by the active or last drain job.
  uint16_t fifoDrainWordsRead() const { return _fifoDrainWordsRead; }

  // Register and source access. Public raw access is bounded to the main
  // user register window through Z_OFS_USR and rejects zero-length or wrapping
  // blocks before touching the bus.
  /// @brief Read one public user register.
  /// @param reg Register address in the bounded user window.
  /// @param value Output register value.
  /// @return Status from validation and register read.
  Status readRegisterValue(uint8_t reg, uint8_t& value);

  /// @brief Write one public user register.
  /// @param reg Register address in the bounded user window.
  /// @param value Register value.
  /// @return Status from validation and register write.
  Status writeRegisterValue(uint8_t reg, uint8_t value);

  /// @brief Read a bounded block from the public user register window.
  /// @param startReg First register address.
  /// @param buf Output buffer.
  /// @param len Number of bytes to read.
  /// @return Status from validation and burst read.
  Status readRegisterBlock(uint8_t startReg, uint8_t* buf, size_t len);

  /// @brief Refresh cached runtime configuration from device registers.
  /// @return Status from register reads.
  Status refreshCachedConfig();

  /// @brief Read WAKE_UP_SRC.
  /// @param value Raw register value.
  /// @return Status from register read.
  Status readWakeUpSource(uint8_t& value);

  /// @brief Read TAP_SRC.
  /// @param value Raw register value.
  /// @return Status from register read.
  Status readTapSource(uint8_t& value);

  /// @brief Read D6D_SRC.
  /// @param value Raw register value.
  /// @return Status from register read.
  Status read6dSource(uint8_t& value);

  /// @brief Read FUNC_SRC1.
  ///
  /// Use CommandTable masks such as `MASK_STEP_DETECTED`,
  /// `MASK_STEP_COUNT_DELTA_IA`, `MASK_SIGN_MOTION_IA`, and `MASK_TILT_IA` to
  /// decode the raw value. Some bits are pulsed unless latched/routed.
  /// @param value Raw register value.
  /// @return Status from register read.
  Status readFunctionSource1(uint8_t& value);

  /// @brief Read FUNC_SRC2.
  ///
  /// Use CommandTable masks such as `MASK_WRIST_TILT_IA` and
  /// `MASK_SLAVE0_NACK` through `MASK_SLAVE3_NACK` to decode the raw value.
  /// @param value Raw register value.
  /// @return Status from register read.
  Status readFunctionSource2(uint8_t& value);

  /// @brief Read WRIST_TILT_IA.
  ///
  /// Use CommandTable masks such as `MASK_WRIST_TILT_XPOS` through
  /// `MASK_WRIST_TILT_ZNEG` to decode the triggered axis/sign bits.
  /// @param value Raw register value.
  /// @return Status from register read.
  Status readWristTiltStatus(uint8_t& value);

  /// @brief Read SENSORHUB1_REG through SENSORHUB12_REG.
  /// @param out Sensor-hub bytes and valid count.
  /// @param count Number of bytes to read, 1..12.
  /// @return Status from validation and burst read.
  Status readSensorHub(SensorHubData& out, uint8_t count = 12);

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
  Status _recordFailure(const Status& st);
  void _reassertOfflineLatch();
  Status _ensureNormalI2cAllowed() const;

  // Internal helpers
  Status _applyConfig();
  Status _readRawAllWithTimestamp(uint32_t sampleTimestampMs);
  Status _readRawAll();
  Status _settleSelfTest(uint32_t settleMs);
  Status _waitForSelfTestReady(bool accel);
  Status _readSelfTestAverage(bool accel, uint16_t samples, RawAxes& out);
  Status _startPollJob(uint8_t job, const Status& busyStatus);
  Status _finishPollJob(const Status& st);
  Status _pollSampleStep();
  Status _pollApplyConfigStep(uint8_t step, bool& done);
  Status _pollResetOrBootStep(bool bootJob);
  Status _pollRefreshStep(uint8_t step, bool& done);
  Status _commitStagedCachedConfig();
  Status _pollFifoDrainStep();
  Status _pollCalibrationStep(bool accelJob);
  Status _validateMeasurementRequest(bool checkReady) const;
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
  bool _allowOfflineI2c = false;
  bool _cachedConfigDirty = false;

  // Measurement state
  bool _measurementRequested = false;
  bool _measurementReady = false;
  bool _hasSample = false;
  uint32_t _sampleTimestampMs = 0;
  RawMeasurement _rawMeasurement;

  // Poll job state
  enum class PollJob : uint8_t {
    NONE,
    SAMPLE,
    SOFT_RESET,
    BOOT,
    REFRESH_CONFIG,
    FIFO_DRAIN,
    ACCEL_CALIBRATION,
    GYRO_CALIBRATION
  };
  PollJob _pollJob = PollJob::NONE;
  uint8_t _pollStep = 0;
  uint16_t _pollCount = 0;
  uint32_t _pollNowMs = 0;
  uint32_t _pollDeadlineMs = 0;
  bool _pollDeadlineArmed = false;
  bool _pollInstructionUsed = false;
  bool _pollStartedOffline = false;
  Status _lastPollStatus = Status::Ok();
  uint8_t _pollWakeUpDur = 0;
  uint16_t _fifoDrainMaxWords = 0;
  uint16_t _fifoDrainWordsRead = 0;
  uint16_t _fifoDrainWordsAvailable = 0;
  uint8_t _refreshCtrl1 = 0;
  uint8_t _refreshCtrl2 = 0;
  uint8_t _refreshCtrl3 = 0;
  uint8_t _refreshCtrl4 = 0;
  uint8_t _refreshCtrl6 = 0;
  uint8_t _refreshCtrl7 = 0;
  uint8_t _refreshCtrl8 = 0;
  uint8_t _refreshCtrl10 = 0;
  uint8_t _refreshWakeUpDur = 0;
  uint8_t _refreshOffsetData[3] = {};
  uint8_t _refreshFifoCtrl[5] = {};
  uint16_t _calibrationSamplesTarget = 0;
  uint16_t _calibrationSamplesDone = 0;
  uint16_t _calibrationPollsForSample = 0;
  double _calibrationSumX = 0.0;
  double _calibrationSumY = 0.0;
  double _calibrationSumZ = 0.0;
  float _calibrationMinX = 0.0f;
  float _calibrationMaxX = 0.0f;
  float _calibrationMinY = 0.0f;
  float _calibrationMaxY = 0.0f;
  float _calibrationMinZ = 0.0f;
  float _calibrationMaxZ = 0.0f;

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

  // Software bias calibration
  Axes _accelBias;
  Axes _gyroBias;
};

}  // namespace LSM6DS3TR
