/**
 * @file LSM6DS3TR.cpp
 * @brief LSM6DS3TR-C driver implementation.
 */

#include "LSM6DS3TR/LSM6DS3TR.h"

#include <Arduino.h>

#include <cstring>
#include <limits>

namespace LSM6DS3TR {
namespace {

static constexpr size_t MAX_WRITE_LEN = 16;
static constexpr size_t MAX_PUBLIC_READ_LEN = 128;
static constexpr uint32_t RESET_TIMEOUT_MS = 25;
static constexpr uint32_t BOOT_TIMEOUT_MS = 25;
static constexpr uint16_t RESET_MAX_POLLS = 255;

class ScopedOfflineI2cAllowance {
public:
  explicit ScopedOfflineI2cAllowance(bool& flag, bool allow) : _flag(flag), _old(flag) {
    _flag = allow;
  }

  ~ScopedOfflineI2cAllowance() {
    _flag = _old;
  }

  ScopedOfflineI2cAllowance(const ScopedOfflineI2cAllowance&) = delete;
  ScopedOfflineI2cAllowance& operator=(const ScopedOfflineI2cAllowance&) = delete;

private:
  bool& _flag;
  bool _old;
};

static bool deadlineReached(uint32_t nowMs, uint32_t deadlineMs) {
  return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}

static bool isValidOdrValue(Odr odr) {
  switch (odr) {
    case Odr::POWER_DOWN:
    case Odr::HZ_12_5:
    case Odr::HZ_26:
    case Odr::HZ_52:
    case Odr::HZ_104:
    case Odr::HZ_208:
    case Odr::HZ_416:
    case Odr::HZ_833:
    case Odr::HZ_1660:
    case Odr::HZ_3330:
    case Odr::HZ_6660:
    case Odr::HZ_1_6:
      return true;
    default:
      return false;
  }
}

static bool isActiveOdr(Odr odr) {
  return odr != Odr::POWER_DOWN;
}

/// Approximate ODR sample interval in milliseconds (for calibration timeouts).
static uint32_t odrIntervalMs(Odr odr) {
  switch (odr) {
    case Odr::HZ_1_6:  return 625;
    case Odr::HZ_12_5: return 80;
    case Odr::HZ_26:   return 39;
    case Odr::HZ_52:   return 20;
    case Odr::HZ_104:  return 10;
    case Odr::HZ_208:  return 5;
    case Odr::HZ_416:  return 3;
    case Odr::HZ_833:  return 2;
    case Odr::HZ_1660: return 1;
    case Odr::HZ_3330: return 1;
    case Odr::HZ_6660: return 1;
    default:           return 0;
  }
}

static bool isAccelOdrSupportedInLowPowerNormal(Odr odr) {
  switch (odr) {
    case Odr::POWER_DOWN:
    case Odr::HZ_1_6:
    case Odr::HZ_12_5:
    case Odr::HZ_26:
    case Odr::HZ_52:
    case Odr::HZ_104:
    case Odr::HZ_208:
      return true;
    default:
      return false;
  }
}

static bool isValidAccelOdr(Odr odr, AccelPowerMode powerMode) {
  if (!isValidOdrValue(odr)) {
    return false;
  }

  if (powerMode == AccelPowerMode::HIGH_PERFORMANCE) {
    return odr != Odr::HZ_1_6;
  }

  return isAccelOdrSupportedInLowPowerNormal(odr);
}

static bool isGyroOdrSupportedInLowPowerNormal(Odr odr) {
  switch (odr) {
    case Odr::POWER_DOWN:
    case Odr::HZ_12_5:
    case Odr::HZ_26:
    case Odr::HZ_52:
    case Odr::HZ_104:
    case Odr::HZ_208:
      return true;
    default:
      return false;
  }
}

static bool isValidGyroOdr(Odr odr, GyroPowerMode powerMode) {
  if (!isValidOdrValue(odr) || odr == Odr::HZ_1_6) {
    return false;
  }

  if (powerMode == GyroPowerMode::HIGH_PERFORMANCE) {
    return true;
  }

  return isGyroOdrSupportedInLowPowerNormal(odr);
}

static bool isValidAccelFs(AccelFs fs) {
  switch (fs) {
    case AccelFs::G_2:
    case AccelFs::G_4:
    case AccelFs::G_8:
    case AccelFs::G_16:
      return true;
    default:
      return false;
  }
}

static bool isValidGyroFs(GyroFs fs) {
  switch (fs) {
    case GyroFs::DPS_125:
    case GyroFs::DPS_250:
    case GyroFs::DPS_500:
    case GyroFs::DPS_1000:
    case GyroFs::DPS_2000:
      return true;
    default:
      return false;
  }
}

static bool isValidAccelPowerMode(AccelPowerMode mode) {
  switch (mode) {
    case AccelPowerMode::HIGH_PERFORMANCE:
    case AccelPowerMode::LOW_POWER_NORMAL:
      return true;
    default:
      return false;
  }
}

static bool isValidGyroPowerMode(GyroPowerMode mode) {
  switch (mode) {
    case GyroPowerMode::HIGH_PERFORMANCE:
    case GyroPowerMode::LOW_POWER_NORMAL:
      return true;
    default:
      return false;
  }
}

static bool isValidGyroHpfMode(GyroHpfMode mode) {
  switch (mode) {
    case GyroHpfMode::HZ_0_0081:
    case GyroHpfMode::HZ_0_0324:
    case GyroHpfMode::HZ_2_07:
    case GyroHpfMode::HZ_16_32:
      return true;
    default:
      return false;
  }
}

static bool isValidAccelOffsetWeight(AccelOffsetWeight weight) {
  switch (weight) {
    case AccelOffsetWeight::MG_1:
    case AccelOffsetWeight::MG_16:
      return true;
    default:
      return false;
  }
}

static bool isValidFifoMode(FifoMode mode) {
  switch (mode) {
    case FifoMode::BYPASS:
    case FifoMode::FIFO:
    case FifoMode::CONTINUOUS_TO_FIFO:
    case FifoMode::BYPASS_TO_CONTINUOUS:
    case FifoMode::CONTINUOUS:
      return true;
    default:
      return false;
  }
}

static bool isValidFifoDecimation(FifoDecimation decimation) {
  return static_cast<uint8_t>(decimation) <= static_cast<uint8_t>(FifoDecimation::DIV_32);
}

static bool isValidFifoOdr(Odr odr) {
  return odr == Odr::POWER_DOWN || isValidGyroOdr(odr, GyroPowerMode::HIGH_PERFORMANCE);
}

static bool isValidPublicRegisterAddress(uint8_t reg) {
  return reg <= cmd::REG_Z_OFS_USR;
}

static bool isValidPublicRegisterBlock(uint8_t startReg, size_t len) {
  if (len == 0 || len > MAX_PUBLIC_READ_LEN) {
    return false;
  }
  const uint16_t endReg = static_cast<uint16_t>(startReg) + static_cast<uint16_t>(len - 1u);
  return endReg <= cmd::REG_Z_OFS_USR;
}

static bool requiresAccelOdrAtLeast26Hz(Odr odr) {
  switch (odr) {
    case Odr::HZ_26:
    case Odr::HZ_52:
    case Odr::HZ_104:
    case Odr::HZ_208:
    case Odr::HZ_416:
    case Odr::HZ_833:
    case Odr::HZ_1660:
    case Odr::HZ_3330:
    case Odr::HZ_6660:
      return true;
    default:
      return false;
  }
}

static float accelSensitivityMgLsb(AccelFs fs) {
  switch (fs) {
    case AccelFs::G_2:
      return 0.061f;
    case AccelFs::G_4:
      return 0.122f;
    case AccelFs::G_8:
      return 0.244f;
    case AccelFs::G_16:
      return 0.488f;
    default:
      return 0.061f;
  }
}

static float gyroSensitivityMdpsLsb(GyroFs fs) {
  switch (fs) {
    case GyroFs::DPS_125:
      return 4.375f;
    case GyroFs::DPS_250:
      return 8.75f;
    case GyroFs::DPS_500:
      return 17.50f;
    case GyroFs::DPS_1000:
      return 35.0f;
    case GyroFs::DPS_2000:
      return 70.0f;
    default:
      return 8.75f;
  }
}

static uint8_t loByte(uint16_t value) {
  return static_cast<uint8_t>(value & 0xFFu);
}

}  // namespace

Status LSM6DS3TR::begin(const Config& config) {
  _initialized = false;
  _driverState = DriverState::UNINIT;

  _lastOkMs = 0;
  _lastErrorMs = 0;
  _lastError = Status::Ok();
  _consecutiveFailures = 0;
  _totalFailures = 0;
  _totalSuccess = 0;

  _measurementRequested = false;
  _measurementReady = false;
  _hasSample = false;
  _rawMeasurement = RawMeasurement{};

  _config = Config{};
  _accelPowerMode = AccelPowerMode::HIGH_PERFORMANCE;
  _gyroPowerMode = GyroPowerMode::HIGH_PERFORMANCE;
  _gyroSleepEnabled = false;
  _accelFilterConfig = AccelFilterConfig{};
  _gyroFilterConfig = GyroFilterConfig{};
  _timestampEnabled = false;
  _timestampHighResolution = false;
  _pedometerEnabled = false;
  _significantMotionEnabled = false;
  _tiltEnabled = false;
  _wristTiltEnabled = false;
  _accelOffsetWeight = AccelOffsetWeight::MG_1;
  _accelUserOffset = AccelUserOffset{};
  _fifoConfig = FifoConfig{};
  _accelBias = Axes{};
  _gyroBias = Axes{};

  if (config.i2cWrite == nullptr || config.i2cWriteRead == nullptr) {
    return Status::Error(Err::INVALID_CONFIG, "I2C callbacks not set");
  }
  if (config.i2cTimeoutMs == 0) {
    return Status::Error(Err::INVALID_CONFIG, "I2C timeout must be > 0");
  }
  if (config.i2cAddress != 0x6A && config.i2cAddress != 0x6B) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid I2C address");
  }
  if (!isValidAccelFs(config.fsXl)) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid accel full-scale");
  }
  if (!isValidGyroFs(config.fsG)) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid gyro full-scale");
  }
  if (!isValidAccelPowerMode(config.accelPowerMode)) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid accel power mode");
  }
  if (!isValidGyroPowerMode(config.gyroPowerMode)) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid gyro power mode");
  }
  if (!isValidAccelOdr(config.odrXl, config.accelPowerMode)) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid accel ODR/power-mode combination");
  }
  if (!isValidGyroOdr(config.odrG, config.gyroPowerMode)) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid gyro ODR/power-mode combination");
  }

  _config = config;
  if (_config.offlineThreshold == 0) {
    _config.offlineThreshold = 1;
  }
  _accelPowerMode = config.accelPowerMode;
  _gyroPowerMode = config.gyroPowerMode;

  uint8_t chipId = 0;
  Status st = _readRegisterRaw(cmd::REG_WHO_AM_I, chipId);
  if (!st.ok()) {
    return Status::Error(Err::DEVICE_NOT_FOUND, "Device not responding", st.detail);
  }
  if (chipId != cmd::WHO_AM_I_VALUE) {
    return Status::Error(Err::CHIP_ID_MISMATCH, "Chip ID mismatch", chipId);
  }

  st = _applyConfig();
  if (!st.ok()) {
    return st;
  }

  _initialized = true;
  _driverState = DriverState::READY;
  return Status::Ok();
}

void LSM6DS3TR::tick(uint32_t nowMs) {
  (void)nowMs;

  if (!_initialized || !_measurementRequested) {
    return;
  }
  if (_driverState == DriverState::OFFLINE) {
    _measurementRequested = false;
    return;
  }

  uint8_t statusReg = 0;
  Status st = readRegister(cmd::REG_STATUS_REG, statusReg);
  if (!st.ok()) {
    if (_driverState == DriverState::OFFLINE) {
      _measurementRequested = false;
    }
    return;
  }

  const bool accelActive = isActiveOdr(_config.odrXl);
  const bool gyroActive = isActiveOdr(_config.odrG);
  const bool accelReady = !accelActive || ((statusReg & cmd::MASK_XLDA) != 0);
  const bool gyroReady = !gyroActive || ((statusReg & cmd::MASK_GDA) != 0);
  if (!accelReady || !gyroReady) {
    return;
  }

  st = _readRawAll();
  if (!st.ok()) {
    if (_driverState == DriverState::OFFLINE) {
      _measurementRequested = false;
    }
    return;
  }

  _measurementReady = true;
  _measurementRequested = false;
}

void LSM6DS3TR::end() {
  if (_config.i2cWrite != nullptr) {
    const uint8_t payloadG[2] = {cmd::REG_CTRL2_G, 0x00};
    (void)_i2cWriteRaw(payloadG, sizeof(payloadG));
    const uint8_t payloadXl[2] = {cmd::REG_CTRL1_XL, 0x00};
    (void)_i2cWriteRaw(payloadXl, sizeof(payloadXl));
  }

  _initialized = false;
  _driverState = DriverState::UNINIT;
  _measurementRequested = false;
  _measurementReady = false;
  _hasSample = false;
  _rawMeasurement = RawMeasurement{};
}

Status LSM6DS3TR::probe() {
  if (_config.i2cWriteRead == nullptr) {
    return Status::Error(Err::NOT_INITIALIZED, "No transport configured");
  }

  uint8_t chipId = 0;
  Status st = _readRegisterRaw(cmd::REG_WHO_AM_I, chipId);
  if (!st.ok()) {
    return Status::Error(Err::DEVICE_NOT_FOUND, "Device not responding", st.detail);
  }
  if (chipId != cmd::WHO_AM_I_VALUE) {
    return Status::Error(Err::CHIP_ID_MISMATCH, "Chip ID mismatch", chipId);
  }
  return Status::Ok();
}

Status LSM6DS3TR::recover() {
  if (!_initialized) {
    if (_config.i2cWriteRead == nullptr || _config.i2cWrite == nullptr) {
      return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
    }
    const Config config = _config;
    return begin(config);
  }

  const bool startedOffline = _driverState == DriverState::OFFLINE;
  ScopedOfflineI2cAllowance allowOfflineI2c(_allowOfflineI2c, true);
  Status result = [this]() -> Status {
    uint8_t chipId = 0;
    Status st = readRegister(cmd::REG_WHO_AM_I, chipId);
    if (!st.ok()) {
      return st;
    }
    if (chipId != cmd::WHO_AM_I_VALUE) {
      return _recordFailure(Status::Error(Err::CHIP_ID_MISMATCH, "Chip ID mismatch", chipId));
    }

    return _applyConfig();
  }();
  if (startedOffline && !result.ok() && !result.inProgress()) {
    _reassertOfflineLatch();
  }
  return result;
}

Status LSM6DS3TR::requestMeasurement() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (_driverState == DriverState::OFFLINE) {
    return Status::Error(Err::BUSY, "Driver is offline; call recover()");
  }

  const bool accelActive = isActiveOdr(_config.odrXl);
  const bool gyroActive = isActiveOdr(_config.odrG);
  if (!accelActive && !gyroActive) {
    return Status::Error(Err::INVALID_PARAM, "Both sensors powered down");
  }
  if (accelActive && gyroActive) {
    if (!_config.bdu) {
      return Status::Error(Err::INVALID_PARAM, "Async combined measurement requires BDU");
    }
    if (_config.odrXl != _config.odrG) {
      return Status::Error(Err::INVALID_PARAM, "Async combined measurement requires matching ODR");
    }
  }
  if (_measurementRequested && !_measurementReady) {
    return Status::Error(Err::BUSY, "Measurement in progress");
  }

  _measurementReady = false;
  _measurementRequested = true;
  return Status::Error(Err::IN_PROGRESS, "Measurement scheduled");
}

Status LSM6DS3TR::getMeasurement(Measurement& out) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!_measurementReady) {
    return Status::Error(Err::MEASUREMENT_NOT_READY, "Measurement not ready");
  }

  out.accel = convertAccel(_rawMeasurement.accel);
  out.gyro = convertGyro(_rawMeasurement.gyro);
  out.temperatureC = convertTemperature(_rawMeasurement.temperature);
  correctAccel(out.accel);
  correctGyro(out.gyro);
  _measurementReady = false;
  return Status::Ok();
}

Status LSM6DS3TR::getRawMeasurement(RawMeasurement& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!_hasSample) {
    return Status::Error(Err::MEASUREMENT_NOT_READY, "No sample available");
  }

  out = _rawMeasurement;
  return Status::Ok();
}

Status LSM6DS3TR::readAccelRaw(RawAxes& out) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  uint8_t data[cmd::DATA_LEN_ACCEL] = {};
  Status st = readRegs(cmd::REG_DATA_START_ACCEL, data, sizeof(data));
  if (!st.ok()) {
    return st;
  }

  out.x = static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) | data[0]);
  out.y = static_cast<int16_t>((static_cast<uint16_t>(data[3]) << 8) | data[2]);
  out.z = static_cast<int16_t>((static_cast<uint16_t>(data[5]) << 8) | data[4]);
  return Status::Ok();
}

Status LSM6DS3TR::readGyroRaw(RawAxes& out) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  uint8_t data[cmd::DATA_LEN_GYRO] = {};
  Status st = readRegs(cmd::REG_DATA_START_GYRO, data, sizeof(data));
  if (!st.ok()) {
    return st;
  }

  out.x = static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) | data[0]);
  out.y = static_cast<int16_t>((static_cast<uint16_t>(data[3]) << 8) | data[2]);
  out.z = static_cast<int16_t>((static_cast<uint16_t>(data[5]) << 8) | data[4]);
  return Status::Ok();
}

Status LSM6DS3TR::readTemperatureRaw(int16_t& out) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  uint8_t data[2] = {};
  Status st = readRegs(cmd::REG_OUT_TEMP_L, data, sizeof(data));
  if (!st.ok()) {
    return st;
  }

  out = static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) | data[0]);
  return Status::Ok();
}

Status LSM6DS3TR::readAllRaw(RawMeasurement& out) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  uint8_t data[cmd::DATA_LEN_ALL] = {};
  Status st = readRegs(cmd::REG_DATA_START_ALL, data, sizeof(data));
  if (!st.ok()) {
    return st;
  }

  out.temperature = static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) | data[0]);
  out.gyro.x = static_cast<int16_t>((static_cast<uint16_t>(data[3]) << 8) | data[2]);
  out.gyro.y = static_cast<int16_t>((static_cast<uint16_t>(data[5]) << 8) | data[4]);
  out.gyro.z = static_cast<int16_t>((static_cast<uint16_t>(data[7]) << 8) | data[6]);
  out.accel.x = static_cast<int16_t>((static_cast<uint16_t>(data[9]) << 8) | data[8]);
  out.accel.y = static_cast<int16_t>((static_cast<uint16_t>(data[11]) << 8) | data[10]);
  out.accel.z = static_cast<int16_t>((static_cast<uint16_t>(data[13]) << 8) | data[12]);

  _rawMeasurement = out;
  _hasSample = true;
  return Status::Ok();
}

Axes LSM6DS3TR::convertAccel(const RawAxes& raw) const {
  const float sens = accelSensitivityMgLsb(_config.fsXl);
  Axes result;
  result.x = static_cast<float>(raw.x) * sens * 0.001f;
  result.y = static_cast<float>(raw.y) * sens * 0.001f;
  result.z = static_cast<float>(raw.z) * sens * 0.001f;
  return result;
}

Axes LSM6DS3TR::convertGyro(const RawAxes& raw) const {
  const float sens = gyroSensitivityMdpsLsb(_config.fsG);
  Axes result;
  result.x = static_cast<float>(raw.x) * sens * 0.001f;
  result.y = static_cast<float>(raw.y) * sens * 0.001f;
  result.z = static_cast<float>(raw.z) * sens * 0.001f;
  return result;
}

float LSM6DS3TR::convertTemperature(int16_t raw) const {
  return static_cast<float>(raw) / 256.0f + 25.0f;
}

Status LSM6DS3TR::setAccelOdr(Odr odr) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidAccelOdr(odr, _accelPowerMode)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid accel ODR/power-mode combination");
  }
  if ((_pedometerEnabled || _significantMotionEnabled || _tiltEnabled || _wristTiltEnabled) &&
      !requiresAccelOdrAtLeast26Hz(odr)) {
    return Status::Error(Err::INVALID_PARAM, "Embedded functions require accel ODR >= 26 Hz");
  }
  if (_timestampEnabled && odr == Odr::POWER_DOWN && _config.odrG == Odr::POWER_DOWN) {
    return Status::Error(Err::INVALID_PARAM, "Timestamp requires at least one active sensor");
  }

  Status st = writeRegister(cmd::REG_CTRL1_XL, _buildCtrl1Xl(odr, _config.fsXl));
  if (!st.ok()) {
    return st;
  }
  _config.odrXl = odr;
  return Status::Ok();
}

Status LSM6DS3TR::setGyroOdr(Odr odr) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidGyroOdr(odr, _gyroPowerMode)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid gyro ODR/power-mode combination");
  }
  if (_timestampEnabled && odr == Odr::POWER_DOWN && _config.odrXl == Odr::POWER_DOWN) {
    return Status::Error(Err::INVALID_PARAM, "Timestamp requires at least one active sensor");
  }

  Status st = writeRegister(cmd::REG_CTRL2_G, _buildCtrl2G(odr, _config.fsG));
  if (!st.ok()) {
    return st;
  }
  _config.odrG = odr;
  return Status::Ok();
}

Status LSM6DS3TR::setAccelFs(AccelFs fs) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidAccelFs(fs)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid accel full-scale");
  }

  Status st = writeRegister(cmd::REG_CTRL1_XL, _buildCtrl1Xl(_config.odrXl, fs));
  if (!st.ok()) {
    return st;
  }
  _config.fsXl = fs;
  return Status::Ok();
}

Status LSM6DS3TR::setGyroFs(GyroFs fs) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidGyroFs(fs)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid gyro full-scale");
  }

  Status st = writeRegister(cmd::REG_CTRL2_G, _buildCtrl2G(_config.odrG, fs));
  if (!st.ok()) {
    return st;
  }
  _config.fsG = fs;
  return Status::Ok();
}

Status LSM6DS3TR::getAccelOdr(Odr& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  out = _config.odrXl;
  return Status::Ok();
}

Status LSM6DS3TR::getGyroOdr(Odr& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  out = _config.odrG;
  return Status::Ok();
}

Status LSM6DS3TR::getAccelFs(AccelFs& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  out = _config.fsXl;
  return Status::Ok();
}

Status LSM6DS3TR::getGyroFs(GyroFs& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  out = _config.fsG;
  return Status::Ok();
}

Status LSM6DS3TR::readWhoAmI(uint8_t& id) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegister(cmd::REG_WHO_AM_I, id);
}

Status LSM6DS3TR::readStatusReg(uint8_t& status) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegister(cmd::REG_STATUS_REG, status);
}

Status LSM6DS3TR::isAccelDataReady(bool& ready) {
  uint8_t status = 0;
  Status st = readStatusReg(status);
  if (!st.ok()) {
    return st;
  }
  ready = (status & cmd::MASK_XLDA) != 0;
  return Status::Ok();
}

Status LSM6DS3TR::isGyroDataReady(bool& ready) {
  uint8_t status = 0;
  Status st = readStatusReg(status);
  if (!st.ok()) {
    return st;
  }
  ready = (status & cmd::MASK_GDA) != 0;
  return Status::Ok();
}

Status LSM6DS3TR::isTempDataReady(bool& ready) {
  uint8_t status = 0;
  Status st = readStatusReg(status);
  if (!st.ok()) {
    return st;
  }
  ready = (status & cmd::MASK_TDA) != 0;
  return Status::Ok();
}

float LSM6DS3TR::accelSensitivity() const {
  return accelSensitivityMgLsb(_config.fsXl);
}

float LSM6DS3TR::gyroSensitivity() const {
  return gyroSensitivityMdpsLsb(_config.fsG);
}

Status LSM6DS3TR::setAccelPowerMode(AccelPowerMode mode) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidAccelPowerMode(mode)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid accel power mode");
  }
  if (!isValidAccelOdr(_config.odrXl, mode)) {
    return Status::Error(Err::INVALID_PARAM, "Current accel ODR is not valid in requested power mode");
  }

  const AccelPowerMode oldMode = _accelPowerMode;
  _accelPowerMode = mode;
  Status st = writeRegister(cmd::REG_CTRL6_C, _buildCtrl6C());
  if (!st.ok()) {
    _accelPowerMode = oldMode;
  }
  return st;
}

Status LSM6DS3TR::getAccelPowerMode(AccelPowerMode& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  out = _accelPowerMode;
  return Status::Ok();
}

Status LSM6DS3TR::setGyroPowerMode(GyroPowerMode mode) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidGyroPowerMode(mode)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid gyro power mode");
  }
  if (!isValidGyroOdr(_config.odrG, mode)) {
    return Status::Error(Err::INVALID_PARAM, "Current gyro ODR is not valid in requested power mode");
  }

  const GyroPowerMode oldMode = _gyroPowerMode;
  _gyroPowerMode = mode;
  Status st = writeRegister(cmd::REG_CTRL7_G, _buildCtrl7G());
  if (!st.ok()) {
    _gyroPowerMode = oldMode;
  }
  return st;
}

Status LSM6DS3TR::getGyroPowerMode(GyroPowerMode& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  out = _gyroPowerMode;
  return Status::Ok();
}

Status LSM6DS3TR::setGyroSleepEnabled(bool enabled) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  const bool oldValue = _gyroSleepEnabled;
  _gyroSleepEnabled = enabled;
  Status st = writeRegister(cmd::REG_CTRL4_C, _buildCtrl4C());
  if (!st.ok()) {
    _gyroSleepEnabled = oldValue;
  }
  return st;
}

Status LSM6DS3TR::getGyroSleepEnabled(bool& enabled) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  enabled = _gyroSleepEnabled;
  return Status::Ok();
}

Status LSM6DS3TR::setAccelFilterConfig(const AccelFilterConfig& config) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  const AccelFilterConfig oldConfig = _accelFilterConfig;
  _accelFilterConfig = config;
  Status st = writeRegister(cmd::REG_CTRL8_XL, _buildCtrl8Xl());
  if (!st.ok()) {
    _accelFilterConfig = oldConfig;
  }
  return st;
}

Status LSM6DS3TR::getAccelFilterConfig(AccelFilterConfig& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  out = _accelFilterConfig;
  return Status::Ok();
}

Status LSM6DS3TR::setGyroFilterConfig(const GyroFilterConfig& config) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidGyroHpfMode(config.highPassMode)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid gyro high-pass mode");
  }

  const GyroFilterConfig oldConfig = _gyroFilterConfig;
  _gyroFilterConfig = config;
  Status st = writeRegister(cmd::REG_CTRL4_C, _buildCtrl4C());
  if (!st.ok()) {
    _gyroFilterConfig = oldConfig;
    return st;
  }
  st = writeRegister(cmd::REG_CTRL7_G, _buildCtrl7G());
  if (!st.ok()) {
    _gyroFilterConfig = oldConfig;
  }
  return st;
}

Status LSM6DS3TR::getGyroFilterConfig(GyroFilterConfig& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  out = _gyroFilterConfig;
  return Status::Ok();
}

Status LSM6DS3TR::setTimestampEnabled(bool enabled) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (enabled && !isActiveOdr(_config.odrXl) && !isActiveOdr(_config.odrG)) {
    return Status::Error(Err::INVALID_PARAM, "Timestamp requires at least one active sensor");
  }

  const bool oldValue = _timestampEnabled;
  _timestampEnabled = enabled;
  Status st = writeRegister(cmd::REG_CTRL10_C, _buildCtrl10C());
  if (!st.ok()) {
    _timestampEnabled = oldValue;
  }
  return st;
}

Status LSM6DS3TR::getTimestampEnabled(bool& enabled) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  enabled = _timestampEnabled;
  return Status::Ok();
}

Status LSM6DS3TR::setTimestampHighResolution(bool enabled) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  const bool oldValue = _timestampHighResolution;
  _timestampHighResolution = enabled;
  Status st = _updateRegister(cmd::REG_WAKE_UP_DUR, cmd::MASK_TIMER_HR, _buildWakeUpDur());
  if (!st.ok()) {
    _timestampHighResolution = oldValue;
    return st;
  }
  if (enabled) {
    st = resetTimestamp();
    if (!st.ok()) {
      _timestampHighResolution = oldValue;
    }
    return st;
  }
  return Status::Ok();
}

Status LSM6DS3TR::getTimestampHighResolution(bool& enabled) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  enabled = _timestampHighResolution;
  return Status::Ok();
}

Status LSM6DS3TR::readTimestamp(uint32_t& out) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  uint8_t data[3] = {};
  Status st = readRegs(cmd::REG_TIMESTAMP0, data, sizeof(data));
  if (!st.ok()) {
    return st;
  }

  out = static_cast<uint32_t>(data[0]) |
        (static_cast<uint32_t>(data[1]) << 8) |
        (static_cast<uint32_t>(data[2]) << 16);
  return Status::Ok();
}

Status LSM6DS3TR::resetTimestamp() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return writeRegister(cmd::REG_TIMESTAMP2, cmd::TIMESTAMP_RESET_VALUE);
}

Status LSM6DS3TR::setPedometerEnabled(bool enabled) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (enabled && !requiresAccelOdrAtLeast26Hz(_config.odrXl)) {
    return Status::Error(Err::INVALID_PARAM, "Pedometer requires accel ODR >= 26 Hz");
  }

  const bool oldValue = _pedometerEnabled;
  _pedometerEnabled = enabled;
  Status st = writeRegister(cmd::REG_CTRL10_C, _buildCtrl10C());
  if (!st.ok()) {
    _pedometerEnabled = oldValue;
  }
  return st;
}

Status LSM6DS3TR::getPedometerEnabled(bool& enabled) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  enabled = _pedometerEnabled;
  return Status::Ok();
}

Status LSM6DS3TR::setSignificantMotionEnabled(bool enabled) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (enabled && !requiresAccelOdrAtLeast26Hz(_config.odrXl)) {
    return Status::Error(Err::INVALID_PARAM, "Significant motion requires accel ODR >= 26 Hz");
  }

  const bool oldValue = _significantMotionEnabled;
  _significantMotionEnabled = enabled;
  Status st = writeRegister(cmd::REG_CTRL10_C, _buildCtrl10C());
  if (!st.ok()) {
    _significantMotionEnabled = oldValue;
  }
  return st;
}

Status LSM6DS3TR::getSignificantMotionEnabled(bool& enabled) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  enabled = _significantMotionEnabled;
  return Status::Ok();
}

Status LSM6DS3TR::setTiltEnabled(bool enabled) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (enabled && !requiresAccelOdrAtLeast26Hz(_config.odrXl)) {
    return Status::Error(Err::INVALID_PARAM, "Tilt detection requires accel ODR >= 26 Hz");
  }

  const bool oldValue = _tiltEnabled;
  _tiltEnabled = enabled;
  Status st = writeRegister(cmd::REG_CTRL10_C, _buildCtrl10C());
  if (!st.ok()) {
    _tiltEnabled = oldValue;
  }
  return st;
}

Status LSM6DS3TR::getTiltEnabled(bool& enabled) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  enabled = _tiltEnabled;
  return Status::Ok();
}

Status LSM6DS3TR::setWristTiltEnabled(bool enabled) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (enabled && !requiresAccelOdrAtLeast26Hz(_config.odrXl)) {
    return Status::Error(Err::INVALID_PARAM, "Wrist tilt requires accel ODR >= 26 Hz");
  }

  const bool oldValue = _wristTiltEnabled;
  _wristTiltEnabled = enabled;
  Status st = writeRegister(cmd::REG_CTRL10_C, _buildCtrl10C());
  if (!st.ok()) {
    _wristTiltEnabled = oldValue;
  }
  return st;
}

Status LSM6DS3TR::getWristTiltEnabled(bool& enabled) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  enabled = _wristTiltEnabled;
  return Status::Ok();
}

Status LSM6DS3TR::readStepCounter(uint16_t& out) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  uint8_t data[2] = {};
  Status st = readRegs(cmd::REG_STEP_COUNTER_L, data, sizeof(data));
  if (!st.ok()) {
    return st;
  }

  out = static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
  return Status::Ok();
}

Status LSM6DS3TR::readStepTimestamp(uint16_t& out) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  uint8_t data[2] = {};
  Status st = readRegs(cmd::REG_STEP_TIMESTAMP_L, data, sizeof(data));
  if (!st.ok()) {
    return st;
  }

  out = static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
  return Status::Ok();
}

Status LSM6DS3TR::resetStepCounter() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  const uint8_t ctrl10WithReset = static_cast<uint8_t>(_buildCtrl10C() | (1u << cmd::BIT_PEDO_RST_STEP));
  Status st = writeRegister(cmd::REG_CTRL10_C, ctrl10WithReset);
  if (!st.ok()) {
    return st;
  }
  return writeRegister(cmd::REG_CTRL10_C, _buildCtrl10C());
}

Status LSM6DS3TR::setAccelOffsetWeight(AccelOffsetWeight weight) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  if (!isValidAccelOffsetWeight(weight)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid accel offset weight");
  }

  const AccelOffsetWeight oldWeight = _accelOffsetWeight;
  _accelOffsetWeight = weight;
  Status st = writeRegister(cmd::REG_CTRL6_C, _buildCtrl6C());
  if (!st.ok()) {
    _accelOffsetWeight = oldWeight;
  }
  return st;
}

Status LSM6DS3TR::getAccelOffsetWeight(AccelOffsetWeight& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  out = _accelOffsetWeight;
  return Status::Ok();
}

Status LSM6DS3TR::setAccelUserOffset(const AccelUserOffset& offset) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  uint8_t data[3] = {
      static_cast<uint8_t>(offset.x),
      static_cast<uint8_t>(offset.y),
      static_cast<uint8_t>(offset.z),
  };
  Status st = writeRegs(cmd::REG_X_OFS_USR, data, sizeof(data));
  if (!st.ok()) {
    return st;
  }
  _accelUserOffset = offset;
  return Status::Ok();
}

Status LSM6DS3TR::getAccelUserOffset(AccelUserOffset& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  out = _accelUserOffset;
  return Status::Ok();
}

Status LSM6DS3TR::configureFifo(const FifoConfig& config) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (config.threshold > 2047u) {
    return Status::Error(Err::INVALID_PARAM, "FIFO threshold out of range");
  }
  if (!isValidFifoMode(config.mode)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid FIFO mode");
  }
  if (!isValidFifoDecimation(config.accelDecimation) ||
      !isValidFifoDecimation(config.gyroDecimation)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid FIFO decimation");
  }
  if (!isValidFifoOdr(config.odr)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid FIFO ODR");
  }
  if (config.odr == Odr::POWER_DOWN && config.mode != FifoMode::BYPASS) {
    return Status::Error(Err::INVALID_PARAM, "FIFO mode requires active FIFO ODR");
  }
  if (config.mode != FifoMode::BYPASS && !_config.bdu) {
    return Status::Error(Err::INVALID_PARAM, "FIFO requires BDU enabled");
  }

  const FifoConfig oldConfig = _fifoConfig;
  _fifoConfig = config;
  Status st = writeRegister(cmd::REG_FIFO_CTRL1, loByte(_fifoConfig.threshold));
  if (!st.ok()) {
    _fifoConfig = oldConfig;
    return st;
  }
  st = writeRegister(cmd::REG_FIFO_CTRL2, _buildFifoCtrl2());
  if (!st.ok()) {
    _fifoConfig = oldConfig;
    return st;
  }
  st = writeRegister(cmd::REG_FIFO_CTRL3, _buildFifoCtrl3());
  if (!st.ok()) {
    _fifoConfig = oldConfig;
    return st;
  }
  st = writeRegister(cmd::REG_FIFO_CTRL4, _buildFifoCtrl4());
  if (!st.ok()) {
    _fifoConfig = oldConfig;
    return st;
  }
  st = writeRegister(cmd::REG_FIFO_CTRL5, _buildFifoCtrl5());
  if (!st.ok()) {
    _fifoConfig = oldConfig;
  }
  return st;
}

Status LSM6DS3TR::getFifoConfig(FifoConfig& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  out = _fifoConfig;
  return Status::Ok();
}

Status LSM6DS3TR::readFifoStatus(FifoStatus& out) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  uint8_t data[4] = {};
  Status st = readRegs(cmd::REG_FIFO_STATUS1, data, sizeof(data));
  if (!st.ok()) {
    return st;
  }

  out.unreadWords = static_cast<uint16_t>(data[0]) |
                    (static_cast<uint16_t>(data[1] & cmd::MASK_DIFF_FIFO_HI) << 8);
  out.watermark = (data[1] & cmd::MASK_FIFO_WATERM) != 0;
  out.overrun = (data[1] & cmd::MASK_FIFO_OVER_RUN) != 0;
  out.fullSmart = (data[1] & cmd::MASK_FIFO_FULL_SMART) != 0;
  out.empty = (data[1] & cmd::MASK_FIFO_EMPTY) != 0;
  out.pattern = static_cast<uint16_t>(data[2]) |
                (static_cast<uint16_t>(data[3] & 0x03u) << 8);
  return Status::Ok();
}

Status LSM6DS3TR::readFifoWord(uint16_t& out) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  FifoStatus status;
  Status st = readFifoStatus(status);
  if (!st.ok()) {
    return st;
  }
  if (status.empty) {
    return Status::Error(Err::FIFO_EMPTY, "FIFO empty");
  }

  uint8_t data[2] = {};
  st = readRegs(cmd::REG_FIFO_DATA_OUT_L, data, sizeof(data));
  if (!st.ok()) {
    return st;
  }

  out = static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
  return Status::Ok();
}

Status LSM6DS3TR::softReset() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  const bool startedOffline = _driverState == DriverState::OFFLINE;
  ScopedOfflineI2cAllowance allowOfflineI2c(_allowOfflineI2c, true);
  Status result = [this]() -> Status {
    _measurementRequested = false;
    _measurementReady = false;
    _hasSample = false;

    const uint8_t ctrl3 = static_cast<uint8_t>(cmd::MASK_SW_RESET | cmd::MASK_IF_INC |
                                               (_config.bdu ? cmd::MASK_BDU : 0));
    Status st = writeRegister(cmd::REG_CTRL3_C, ctrl3);
    if (!st.ok()) {
      return st;
    }

    const uint32_t deadline = _nowMs() + RESET_TIMEOUT_MS;
    bool resetDone = false;
    for (uint16_t poll = 0; poll < RESET_MAX_POLLS; ++poll) {
      if (deadlineReached(_nowMs(), deadline)) {
        return Status::Error(Err::TIMEOUT, "Reset timeout");
      }
      uint8_t ctrl3Read = 0;
      st = _readRegisterRaw(cmd::REG_CTRL3_C, ctrl3Read);
      if (!st.ok()) {
        return _recordFailure(st);
      }
      if ((ctrl3Read & cmd::MASK_SW_RESET) == 0) {
        resetDone = true;
        break;
      }
    }
    if (!resetDone) {
      return Status::Error(Err::TIMEOUT, "Reset polling limit", RESET_MAX_POLLS);
    }

    return _applyConfig();
  }();
  if (startedOffline && !result.ok() && !result.inProgress()) {
    _reassertOfflineLatch();
  }
  return result;
}

Status LSM6DS3TR::boot() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  const bool startedOffline = _driverState == DriverState::OFFLINE;
  ScopedOfflineI2cAllowance allowOfflineI2c(_allowOfflineI2c, true);
  Status result = [this]() -> Status {
    const uint8_t ctrl3 = static_cast<uint8_t>(cmd::MASK_BOOT | cmd::MASK_IF_INC |
                                               (_config.bdu ? cmd::MASK_BDU : 0));
    Status st = writeRegister(cmd::REG_CTRL3_C, ctrl3);
    if (!st.ok()) {
      return st;
    }

    const uint32_t deadline = _nowMs() + BOOT_TIMEOUT_MS;
    bool bootDone = false;
    for (uint16_t poll = 0; poll < RESET_MAX_POLLS; ++poll) {
      if (deadlineReached(_nowMs(), deadline)) {
        return Status::Error(Err::TIMEOUT, "Boot timeout");
      }
      uint8_t ctrl3Read = 0;
      st = _readRegisterRaw(cmd::REG_CTRL3_C, ctrl3Read);
      if (!st.ok()) {
        return _recordFailure(st);
      }
      if ((ctrl3Read & cmd::MASK_BOOT) == 0) {
        bootDone = true;
        break;
      }
    }
    if (!bootDone) {
      return Status::Error(Err::TIMEOUT, "Boot polling limit", RESET_MAX_POLLS);
    }

    return refreshCachedConfig();
  }();
  if (startedOffline && !result.ok() && !result.inProgress()) {
    _reassertOfflineLatch();
  }
  return result;
}

Status LSM6DS3TR::readRegisterValue(uint8_t reg, uint8_t& value) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidPublicRegisterAddress(reg)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid register address");
  }
  return readRegister(reg, value);
}

Status LSM6DS3TR::writeRegisterValue(uint8_t reg, uint8_t value) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidPublicRegisterAddress(reg)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid register address");
  }
  Status st = writeRegister(reg, value);
  if (!st.ok()) {
    return st;
  }

  switch (reg) {
    case cmd::REG_CTRL1_XL:
    case cmd::REG_CTRL2_G:
    case cmd::REG_CTRL3_C:
    case cmd::REG_CTRL4_C:
    case cmd::REG_CTRL6_C:
    case cmd::REG_CTRL7_G:
    case cmd::REG_CTRL8_XL:
    case cmd::REG_CTRL10_C:
    case cmd::REG_WAKE_UP_DUR:
    case cmd::REG_X_OFS_USR:
    case cmd::REG_Y_OFS_USR:
    case cmd::REG_Z_OFS_USR:
    case cmd::REG_FIFO_CTRL1:
    case cmd::REG_FIFO_CTRL2:
    case cmd::REG_FIFO_CTRL3:
    case cmd::REG_FIFO_CTRL4:
    case cmd::REG_FIFO_CTRL5:
      return refreshCachedConfig();
    default:
      return Status::Ok();
  }
}

Status LSM6DS3TR::readRegisterBlock(uint8_t startReg, uint8_t* buf, size_t len) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidPublicRegisterBlock(startReg, len)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid register block");
  }
  return readRegs(startReg, buf, len);
}

Status LSM6DS3TR::refreshCachedConfig() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  uint8_t ctrl1 = 0;
  uint8_t ctrl2 = 0;
  uint8_t ctrl3 = 0;
  uint8_t ctrl4 = 0;
  uint8_t ctrl6 = 0;
  uint8_t ctrl7 = 0;
  uint8_t ctrl8 = 0;
  uint8_t ctrl10 = 0;
  uint8_t wakeUpDur = 0;

  Status st = readRegister(cmd::REG_CTRL1_XL, ctrl1);
  if (!st.ok()) return st;
  st = readRegister(cmd::REG_CTRL2_G, ctrl2);
  if (!st.ok()) return st;
  st = readRegister(cmd::REG_CTRL3_C, ctrl3);
  if (!st.ok()) return st;
  st = readRegister(cmd::REG_CTRL4_C, ctrl4);
  if (!st.ok()) return st;
  st = readRegister(cmd::REG_CTRL6_C, ctrl6);
  if (!st.ok()) return st;
  st = readRegister(cmd::REG_CTRL7_G, ctrl7);
  if (!st.ok()) return st;
  st = readRegister(cmd::REG_CTRL8_XL, ctrl8);
  if (!st.ok()) return st;
  st = readRegister(cmd::REG_CTRL10_C, ctrl10);
  if (!st.ok()) return st;
  st = readRegister(cmd::REG_WAKE_UP_DUR, wakeUpDur);
  if (!st.ok()) return st;

  _config.odrXl = static_cast<Odr>((ctrl1 & cmd::MASK_ODR_XL) >> cmd::BIT_ODR_XL);
  _config.fsXl = static_cast<AccelFs>((ctrl1 & cmd::MASK_FS_XL) >> cmd::BIT_FS_XL);
  _config.odrG = static_cast<Odr>((ctrl2 & cmd::MASK_ODR_G) >> cmd::BIT_ODR_G);
  _config.fsG = ((ctrl2 & cmd::MASK_FS_125) != 0)
                    ? GyroFs::DPS_125
                    : static_cast<GyroFs>((ctrl2 & cmd::MASK_FS_G) >> cmd::BIT_FS_G);
  _config.bdu = (ctrl3 & cmd::MASK_BDU) != 0;

  _gyroSleepEnabled = (ctrl4 & cmd::MASK_SLEEP_G) != 0;
  _gyroFilterConfig.lpf1Enabled = (ctrl4 & cmd::MASK_LPF1_SEL_G) != 0;

  _accelPowerMode = (ctrl6 & cmd::MASK_XL_HM_MODE) != 0
                        ? AccelPowerMode::LOW_POWER_NORMAL
                        : AccelPowerMode::HIGH_PERFORMANCE;
  _accelOffsetWeight = (ctrl6 & cmd::MASK_USR_OFF_W) != 0
                           ? AccelOffsetWeight::MG_16
                           : AccelOffsetWeight::MG_1;

  _gyroPowerMode = (ctrl7 & cmd::MASK_G_HM_MODE) != 0
                       ? GyroPowerMode::LOW_POWER_NORMAL
                       : GyroPowerMode::HIGH_PERFORMANCE;
  _gyroFilterConfig.highPassEnabled = (ctrl7 & cmd::MASK_HP_EN_G) != 0;
  _gyroFilterConfig.highPassMode =
      static_cast<GyroHpfMode>((ctrl7 & cmd::MASK_HPM_G) >> cmd::BIT_HPM_G);

  _accelFilterConfig.lpf2Enabled = (ctrl8 & cmd::MASK_LPF2_XL_EN) != 0;
  _accelFilterConfig.highPassSlopeEnabled = (ctrl8 & cmd::MASK_HP_SLOPE_XL_EN) != 0;
  _accelFilterConfig.lowPassOn6d = (ctrl8 & cmd::MASK_LOW_PASS_ON_6D) != 0;

  _timestampEnabled = (ctrl10 & (1u << cmd::BIT_TIMER_EN)) != 0;
  _pedometerEnabled = (ctrl10 & (1u << cmd::BIT_PEDO_EN)) != 0;
  _tiltEnabled = (ctrl10 & (1u << cmd::BIT_TILT_EN)) != 0;
  _wristTiltEnabled = (ctrl10 & (1u << cmd::BIT_WRIST_TILT_EN)) != 0;
  _significantMotionEnabled = (ctrl10 & (1u << cmd::BIT_SIGN_MOTION_EN)) != 0;
  _timestampHighResolution = (wakeUpDur & cmd::MASK_TIMER_HR) != 0;

  uint8_t offsetData[3] = {};
  st = readRegs(cmd::REG_X_OFS_USR, offsetData, sizeof(offsetData));
  if (!st.ok()) return st;
  _accelUserOffset.x = static_cast<int8_t>(offsetData[0]);
  _accelUserOffset.y = static_cast<int8_t>(offsetData[1]);
  _accelUserOffset.z = static_cast<int8_t>(offsetData[2]);

  uint8_t fifoCtrl[5] = {};
  st = readRegs(cmd::REG_FIFO_CTRL1, fifoCtrl, sizeof(fifoCtrl));
  if (!st.ok()) return st;
  _fifoConfig.threshold = static_cast<uint16_t>(fifoCtrl[0]) |
                          (static_cast<uint16_t>(fifoCtrl[1] & cmd::MASK_FIFO_THRESHOLD_HI) << 8);
  _fifoConfig.storeTimestampStep = (fifoCtrl[1] & (1u << cmd::BIT_TIMER_PEDO_FIFO_EN)) != 0;
  _fifoConfig.storeTemperature = (fifoCtrl[1] & cmd::MASK_FIFO_TEMP_EN) != 0;
  _fifoConfig.gyroDecimation =
      static_cast<FifoDecimation>((fifoCtrl[2] & cmd::MASK_DEC_FIFO_GYRO) >> cmd::BIT_DEC_FIFO_GYRO);
  _fifoConfig.accelDecimation =
      static_cast<FifoDecimation>((fifoCtrl[2] & cmd::MASK_DEC_FIFO_XL) >> cmd::BIT_DEC_FIFO_XL);
  _fifoConfig.stopOnThreshold = (fifoCtrl[3] & cmd::MASK_STOP_ON_FTH) != 0;
  _fifoConfig.onlyHighData = (fifoCtrl[3] & cmd::MASK_ONLY_HIGH_DATA) != 0;
  _fifoConfig.odr = static_cast<Odr>((fifoCtrl[4] & cmd::MASK_ODR_FIFO) >> cmd::BIT_ODR_FIFO);
  _fifoConfig.mode = static_cast<FifoMode>(fifoCtrl[4] & cmd::MASK_FIFO_MODE);

  return Status::Ok();
}

Status LSM6DS3TR::readWakeUpSource(uint8_t& value) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegister(cmd::REG_WAKE_UP_SRC, value);
}

Status LSM6DS3TR::readTapSource(uint8_t& value) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegister(cmd::REG_TAP_SRC, value);
}

Status LSM6DS3TR::read6dSource(uint8_t& value) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegister(cmd::REG_D6D_SRC, value);
}

Status LSM6DS3TR::readFunctionSource1(uint8_t& value) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegister(cmd::REG_FUNC_SRC1, value);
}

Status LSM6DS3TR::readFunctionSource2(uint8_t& value) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegister(cmd::REG_FUNC_SRC2, value);
}

Status LSM6DS3TR::readWristTiltStatus(uint8_t& value) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  return readRegister(cmd::REG_WRIST_TILT_IA, value);
}

Status LSM6DS3TR::_i2cWriteReadRaw(const uint8_t* txBuf, size_t txLen,
                                   uint8_t* rxBuf, size_t rxLen) {
  if (txBuf == nullptr || txLen == 0 || (rxLen > 0 && rxBuf == nullptr)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid I2C buffer");
  }
  if (_config.i2cWriteRead == nullptr) {
    return Status::Error(Err::INVALID_CONFIG, "I2C write-read not set");
  }
  return _config.i2cWriteRead(_config.i2cAddress, txBuf, txLen, rxBuf, rxLen,
                              _config.i2cTimeoutMs, _config.i2cUser);
}

Status LSM6DS3TR::_i2cWriteRaw(const uint8_t* buf, size_t len) {
  if (buf == nullptr || len == 0) {
    return Status::Error(Err::INVALID_PARAM, "Invalid I2C buffer");
  }
  if (_config.i2cWrite == nullptr) {
    return Status::Error(Err::INVALID_CONFIG, "I2C write not set");
  }
  return _config.i2cWrite(_config.i2cAddress, buf, len, _config.i2cTimeoutMs,
                          _config.i2cUser);
}

Status LSM6DS3TR::_i2cWriteReadTracked(const uint8_t* txBuf, size_t txLen,
                                       uint8_t* rxBuf, size_t rxLen) {
  if (txBuf == nullptr || txLen == 0 || (rxLen > 0 && rxBuf == nullptr)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid I2C buffer");
  }
  Status allowed = _ensureNormalI2cAllowed();
  if (!allowed.ok()) {
    return allowed;
  }

  Status st = _i2cWriteReadRaw(txBuf, txLen, rxBuf, rxLen);
  if (st.code == Err::INVALID_CONFIG || st.code == Err::INVALID_PARAM) {
    return st;
  }
  return _updateHealth(st);
}

Status LSM6DS3TR::_i2cWriteTracked(const uint8_t* buf, size_t len) {
  if (buf == nullptr || len == 0) {
    return Status::Error(Err::INVALID_PARAM, "Invalid I2C buffer");
  }
  Status allowed = _ensureNormalI2cAllowed();
  if (!allowed.ok()) {
    return allowed;
  }

  Status st = _i2cWriteRaw(buf, len);
  if (st.code == Err::INVALID_CONFIG || st.code == Err::INVALID_PARAM) {
    return st;
  }
  return _updateHealth(st);
}

Status LSM6DS3TR::readRegs(uint8_t startReg, uint8_t* buf, size_t len) {
  if (buf == nullptr || len == 0) {
    return Status::Error(Err::INVALID_PARAM, "Invalid read buffer");
  }

  uint8_t reg = startReg;
  return _i2cWriteReadTracked(&reg, 1, buf, len);
}

Status LSM6DS3TR::writeRegs(uint8_t startReg, const uint8_t* buf, size_t len) {
  if (buf == nullptr || len == 0) {
    return Status::Error(Err::INVALID_PARAM, "Invalid write buffer");
  }
  if (len > MAX_WRITE_LEN) {
    return Status::Error(Err::INVALID_PARAM, "Write length too large");
  }

  uint8_t payload[MAX_WRITE_LEN + 1] = {};
  payload[0] = startReg;
  std::memcpy(&payload[1], buf, len);
  return _i2cWriteTracked(payload, len + 1);
}

Status LSM6DS3TR::readRegister(uint8_t reg, uint8_t& value) {
  return readRegs(reg, &value, 1);
}

Status LSM6DS3TR::writeRegister(uint8_t reg, uint8_t value) {
  return writeRegs(reg, &value, 1);
}

Status LSM6DS3TR::_readRegisterRaw(uint8_t reg, uint8_t& value) {
  uint8_t addr = reg;
  return _i2cWriteReadRaw(&addr, 1, &value, 1);
}

Status LSM6DS3TR::_updateRegister(uint8_t reg, uint8_t mask, uint8_t value) {
  uint8_t current = 0;
  Status st = readRegister(reg, current);
  if (!st.ok()) {
    return st;
  }

  current = static_cast<uint8_t>((current & static_cast<uint8_t>(~mask)) | (value & mask));
  return writeRegister(reg, current);
}

Status LSM6DS3TR::_updateHealth(const Status& st) {
  if (!_initialized) {
    return st;
  }

  const uint32_t now = _nowMs();
  const uint32_t maxU32 = std::numeric_limits<uint32_t>::max();
  const uint8_t maxU8 = std::numeric_limits<uint8_t>::max();

  if (st.ok()) {
    _lastOkMs = now;
    if (_totalSuccess < maxU32) {
      _totalSuccess++;
    }
    _consecutiveFailures = 0;
    _driverState = DriverState::READY;
    return st;
  }

  _lastError = st;
  _lastErrorMs = now;
  if (_totalFailures < maxU32) {
    _totalFailures++;
  }
  if (_consecutiveFailures < maxU8) {
    _consecutiveFailures++;
  }

  if (_consecutiveFailures >= _config.offlineThreshold) {
    _driverState = DriverState::OFFLINE;
  } else {
    _driverState = DriverState::DEGRADED;
  }

  return st;
}

Status LSM6DS3TR::_recordFailure(const Status& st) {
  if (!_initialized || st.ok() || st.inProgress()) {
    return st;
  }

  const uint32_t now = _nowMs();
  const uint32_t maxU32 = std::numeric_limits<uint32_t>::max();
  const uint8_t maxU8 = std::numeric_limits<uint8_t>::max();

  _lastError = st;
  _lastErrorMs = now;
  if (_totalFailures < maxU32) {
    _totalFailures++;
  }
  if (_consecutiveFailures < maxU8) {
    _consecutiveFailures++;
  }

  if (_consecutiveFailures >= _config.offlineThreshold) {
    _driverState = DriverState::OFFLINE;
  } else {
    _driverState = DriverState::DEGRADED;
  }

  return st;
}

void LSM6DS3TR::_reassertOfflineLatch() {
  _driverState = DriverState::OFFLINE;
  const uint8_t threshold = _config.offlineThreshold == 0 ? 1 : _config.offlineThreshold;
  if (_consecutiveFailures < threshold) {
    _consecutiveFailures = threshold;
  }
}

Status LSM6DS3TR::_ensureNormalI2cAllowed() const {
  if (_initialized && _driverState == DriverState::OFFLINE && !_allowOfflineI2c) {
    return Status::Error(Err::BUSY, "Driver is offline; call recover()");
  }
  return Status::Ok();
}

Status LSM6DS3TR::_applyConfig() {
  Status st = writeRegister(
      cmd::REG_CTRL3_C,
      static_cast<uint8_t>(cmd::MASK_IF_INC | (_config.bdu ? cmd::MASK_BDU : 0)));
  if (!st.ok()) return st;

  st = writeRegister(cmd::REG_CTRL4_C, _buildCtrl4C());
  if (!st.ok()) return st;
  st = writeRegister(cmd::REG_CTRL6_C, _buildCtrl6C());
  if (!st.ok()) return st;
  st = writeRegister(cmd::REG_CTRL7_G, _buildCtrl7G());
  if (!st.ok()) return st;
  st = writeRegister(cmd::REG_CTRL8_XL, _buildCtrl8Xl());
  if (!st.ok()) return st;
  st = writeRegister(cmd::REG_CTRL1_XL, _buildCtrl1Xl(_config.odrXl, _config.fsXl));
  if (!st.ok()) return st;
  st = writeRegister(cmd::REG_CTRL2_G, _buildCtrl2G(_config.odrG, _config.fsG));
  if (!st.ok()) return st;
  st = _updateRegister(cmd::REG_WAKE_UP_DUR, cmd::MASK_TIMER_HR, _buildWakeUpDur());
  if (!st.ok()) return st;
  st = writeRegister(cmd::REG_CTRL10_C, _buildCtrl10C());
  if (!st.ok()) return st;

  const uint8_t offsets[3] = {
      static_cast<uint8_t>(_accelUserOffset.x),
      static_cast<uint8_t>(_accelUserOffset.y),
      static_cast<uint8_t>(_accelUserOffset.z),
  };
  st = writeRegs(cmd::REG_X_OFS_USR, offsets, sizeof(offsets));
  if (!st.ok()) return st;

  st = writeRegister(cmd::REG_FIFO_CTRL1, loByte(_fifoConfig.threshold));
  if (!st.ok()) return st;
  st = writeRegister(cmd::REG_FIFO_CTRL2, _buildFifoCtrl2());
  if (!st.ok()) return st;
  st = writeRegister(cmd::REG_FIFO_CTRL3, _buildFifoCtrl3());
  if (!st.ok()) return st;
  st = writeRegister(cmd::REG_FIFO_CTRL4, _buildFifoCtrl4());
  if (!st.ok()) return st;
  st = writeRegister(cmd::REG_FIFO_CTRL5, _buildFifoCtrl5());
  if (!st.ok()) return st;

  return Status::Ok();
}

Status LSM6DS3TR::_readRawAll() {
  uint8_t data[cmd::DATA_LEN_ALL] = {};
  Status st = readRegs(cmd::REG_DATA_START_ALL, data, sizeof(data));
  if (!st.ok()) {
    return st;
  }

  _rawMeasurement.temperature =
      static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) | data[0]);
  _rawMeasurement.gyro.x =
      static_cast<int16_t>((static_cast<uint16_t>(data[3]) << 8) | data[2]);
  _rawMeasurement.gyro.y =
      static_cast<int16_t>((static_cast<uint16_t>(data[5]) << 8) | data[4]);
  _rawMeasurement.gyro.z =
      static_cast<int16_t>((static_cast<uint16_t>(data[7]) << 8) | data[6]);
  _rawMeasurement.accel.x =
      static_cast<int16_t>((static_cast<uint16_t>(data[9]) << 8) | data[8]);
  _rawMeasurement.accel.y =
      static_cast<int16_t>((static_cast<uint16_t>(data[11]) << 8) | data[10]);
  _rawMeasurement.accel.z =
      static_cast<int16_t>((static_cast<uint16_t>(data[13]) << 8) | data[12]);
  _hasSample = true;
  return Status::Ok();
}

uint32_t LSM6DS3TR::_nowMs() const {
  if (_config.nowMs != nullptr) {
    return _config.nowMs(_config.timeUser);
  }
  return millis();
}

uint8_t LSM6DS3TR::_buildCtrl1Xl(Odr odr, AccelFs fs) {
  return static_cast<uint8_t>((static_cast<uint8_t>(odr) << cmd::BIT_ODR_XL) |
                              (static_cast<uint8_t>(fs) << cmd::BIT_FS_XL));
}

uint8_t LSM6DS3TR::_buildCtrl2G(Odr odr, GyroFs fs) {
  if (fs == GyroFs::DPS_125) {
    return static_cast<uint8_t>((static_cast<uint8_t>(odr) << cmd::BIT_ODR_G) |
                                cmd::MASK_FS_125);
  }
  return static_cast<uint8_t>((static_cast<uint8_t>(odr) << cmd::BIT_ODR_G) |
                              (static_cast<uint8_t>(fs) << cmd::BIT_FS_G));
}

uint8_t LSM6DS3TR::_buildCtrl4C() const {
  uint8_t value = 0;
  if (_gyroSleepEnabled) value |= cmd::MASK_SLEEP_G;
  if (_gyroFilterConfig.lpf1Enabled) value |= cmd::MASK_LPF1_SEL_G;
  return value;
}

uint8_t LSM6DS3TR::_buildCtrl6C() const {
  uint8_t value = 0;
  if (_accelPowerMode == AccelPowerMode::LOW_POWER_NORMAL) value |= cmd::MASK_XL_HM_MODE;
  if (_accelOffsetWeight == AccelOffsetWeight::MG_16) value |= cmd::MASK_USR_OFF_W;
  return value;
}

uint8_t LSM6DS3TR::_buildCtrl7G() const {
  uint8_t value = 0;
  if (_gyroPowerMode == GyroPowerMode::LOW_POWER_NORMAL) value |= cmd::MASK_G_HM_MODE;
  if (_gyroFilterConfig.highPassEnabled) value |= cmd::MASK_HP_EN_G;
  value |= static_cast<uint8_t>(static_cast<uint8_t>(_gyroFilterConfig.highPassMode)
                                << cmd::BIT_HPM_G);
  return value;
}

uint8_t LSM6DS3TR::_buildCtrl8Xl() const {
  uint8_t value = 0;
  if (_accelFilterConfig.lpf2Enabled) value |= cmd::MASK_LPF2_XL_EN;
  if (_accelFilterConfig.highPassSlopeEnabled) value |= cmd::MASK_HP_SLOPE_XL_EN;
  if (_accelFilterConfig.lowPassOn6d) value |= cmd::MASK_LOW_PASS_ON_6D;
  return value;
}

uint8_t LSM6DS3TR::_buildCtrl10C() const {
  uint8_t value = 0;
  if (_wristTiltEnabled) value |= static_cast<uint8_t>(1u << cmd::BIT_WRIST_TILT_EN);
  if (_timestampEnabled) value |= static_cast<uint8_t>(1u << cmd::BIT_TIMER_EN);
  if (_pedometerEnabled) value |= static_cast<uint8_t>(1u << cmd::BIT_PEDO_EN);
  if (_tiltEnabled) value |= static_cast<uint8_t>(1u << cmd::BIT_TILT_EN);
  if (_significantMotionEnabled) value |= static_cast<uint8_t>(1u << cmd::BIT_SIGN_MOTION_EN);
  if (_pedometerEnabled || _tiltEnabled || _significantMotionEnabled || _wristTiltEnabled) {
    value |= static_cast<uint8_t>(1u << cmd::BIT_FUNC_EN);
  }
  return value;
}

uint8_t LSM6DS3TR::_buildWakeUpDur() const {
  return _timestampHighResolution ? cmd::MASK_TIMER_HR : 0;
}

uint8_t LSM6DS3TR::_buildFifoCtrl2() const {
  uint8_t value = static_cast<uint8_t>((_fifoConfig.threshold >> 8) & cmd::MASK_FIFO_THRESHOLD_HI);
  if (_fifoConfig.storeTimestampStep) value |= static_cast<uint8_t>(1u << cmd::BIT_TIMER_PEDO_FIFO_EN);
  if (_fifoConfig.storeTemperature) value |= cmd::MASK_FIFO_TEMP_EN;
  return value;
}

uint8_t LSM6DS3TR::_buildFifoCtrl3() const {
  return static_cast<uint8_t>((static_cast<uint8_t>(_fifoConfig.gyroDecimation) << cmd::BIT_DEC_FIFO_GYRO) |
                              (static_cast<uint8_t>(_fifoConfig.accelDecimation) << cmd::BIT_DEC_FIFO_XL));
}

uint8_t LSM6DS3TR::_buildFifoCtrl4() const {
  uint8_t value = 0;
  if (_fifoConfig.stopOnThreshold) value |= cmd::MASK_STOP_ON_FTH;
  if (_fifoConfig.onlyHighData) value |= cmd::MASK_ONLY_HIGH_DATA;
  return value;
}

uint8_t LSM6DS3TR::_buildFifoCtrl5() const {
  return static_cast<uint8_t>((static_cast<uint8_t>(_fifoConfig.odr) << cmd::BIT_ODR_FIFO) |
                              static_cast<uint8_t>(_fifoConfig.mode));
}

// ---------------------------------------------------------------------------
// Software bias calibration
// ---------------------------------------------------------------------------

void LSM6DS3TR::setAccelBias(const Axes& bias) {
  _accelBias = bias;
}

Axes LSM6DS3TR::accelBias() const {
  return _accelBias;
}

void LSM6DS3TR::setGyroBias(const Axes& bias) {
  _gyroBias = bias;
}

Axes LSM6DS3TR::gyroBias() const {
  return _gyroBias;
}

void LSM6DS3TR::correctAccel(Axes& inout) const {
  inout.x -= _accelBias.x;
  inout.y -= _accelBias.y;
  inout.z -= _accelBias.z;
}

void LSM6DS3TR::correctGyro(Axes& inout) const {
  inout.x -= _gyroBias.x;
  inout.y -= _gyroBias.y;
  inout.z -= _gyroBias.z;
}

Status LSM6DS3TR::captureAccelBias(uint16_t samples, Axes& out) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (samples == 0 || samples > 10000) {
    return Status::Error(Err::INVALID_PARAM, "samples must be 1..10000");
  }
  if (!isActiveOdr(_config.odrXl)) {
    return Status::Error(Err::INVALID_PARAM, "Accel ODR is POWER_DOWN");
  }

  const uint32_t sampleTimeoutMs = odrIntervalMs(_config.odrXl) * 3 + 100;
  double sumX = 0.0;
  double sumY = 0.0;
  double sumZ = 0.0;

  for (uint16_t i = 0; i < samples; ++i) {
    // Wait for fresh accel data
    const uint32_t deadline = _nowMs() + sampleTimeoutMs;
    bool ready = false;
    uint32_t polls = 0;
    const uint32_t maxPolls = sampleTimeoutMs + 2U;
    while (!ready && polls < maxPolls) {
      Status st = isAccelDataReady(ready);
      if (!st.ok()) {
        return st;
      }
      if (!ready && deadlineReached(_nowMs(), deadline)) {
        return Status::Error(Err::TIMEOUT, "Accel data-ready timeout during calibration");
      }
      polls++;
    }
    if (!ready) {
      return Status::Error(Err::TIMEOUT, "Accel data-ready timeout during calibration");
    }

    RawAxes raw;
    Status st = readAccelRaw(raw);
    if (!st.ok()) {
      return st;
    }
    const Axes phys = convertAccel(raw);
    sumX += static_cast<double>(phys.x);
    sumY += static_cast<double>(phys.y);
    sumZ += static_cast<double>(phys.z);
  }

  const double n = static_cast<double>(samples);
  // Bias = average reading - expected value (Z-up: expect X=0, Y=0, Z=+1g)
  out.x = static_cast<float>(sumX / n);
  out.y = static_cast<float>(sumY / n);
  out.z = static_cast<float>(sumZ / n - 1.0);
  _accelBias = out;
  return Status::Ok();
}

Status LSM6DS3TR::captureGyroBias(uint16_t samples, Axes& out) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (samples == 0 || samples > 10000) {
    return Status::Error(Err::INVALID_PARAM, "samples must be 1..10000");
  }
  if (!isActiveOdr(_config.odrG)) {
    return Status::Error(Err::INVALID_PARAM, "Gyro ODR is POWER_DOWN");
  }

  const uint32_t sampleTimeoutMs = odrIntervalMs(_config.odrG) * 3 + 100;
  double sumX = 0.0;
  double sumY = 0.0;
  double sumZ = 0.0;

  for (uint16_t i = 0; i < samples; ++i) {
    // Wait for fresh gyro data
    const uint32_t deadline = _nowMs() + sampleTimeoutMs;
    bool ready = false;
    uint32_t polls = 0;
    const uint32_t maxPolls = sampleTimeoutMs + 2U;
    while (!ready && polls < maxPolls) {
      Status st = isGyroDataReady(ready);
      if (!st.ok()) {
        return st;
      }
      if (!ready && deadlineReached(_nowMs(), deadline)) {
        return Status::Error(Err::TIMEOUT, "Gyro data-ready timeout during calibration");
      }
      polls++;
    }
    if (!ready) {
      return Status::Error(Err::TIMEOUT, "Gyro data-ready timeout during calibration");
    }

    RawAxes raw;
    Status st = readGyroRaw(raw);
    if (!st.ok()) {
      return st;
    }
    const Axes phys = convertGyro(raw);
    sumX += static_cast<double>(phys.x);
    sumY += static_cast<double>(phys.y);
    sumZ += static_cast<double>(phys.z);
  }

  const double n = static_cast<double>(samples);
  // Bias = average reading (at rest, expected 0 on all axes)
  out.x = static_cast<float>(sumX / n);
  out.y = static_cast<float>(sumY / n);
  out.z = static_cast<float>(sumZ / n);
  _gyroBias = out;
  return Status::Ok();
}

}  // namespace LSM6DS3TR
