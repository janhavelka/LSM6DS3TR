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
static constexpr uint32_t RESET_TIMEOUT_MS = 25;
static constexpr uint16_t RESET_MAX_POLLS = 255;

static bool deadlineReached(uint32_t nowMs, uint32_t deadlineMs) {
  return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}

static bool isValidOdr(Odr odr) {
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

static bool isValidAccelFs(AccelFs fs) {
  switch (fs) {
    case AccelFs::G_2:
    case AccelFs::G_16:
    case AccelFs::G_4:
    case AccelFs::G_8:
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

/// Accelerometer sensitivity in mg/LSB indexed by AccelFs register value
static float accelSensitivityMgLsb(AccelFs fs) {
  switch (fs) {
    case AccelFs::G_2:  return 0.061f;
    case AccelFs::G_4:  return 0.122f;
    case AccelFs::G_8:  return 0.244f;
    case AccelFs::G_16: return 0.488f;
    default:            return 0.061f;
  }
}

/// Gyroscope sensitivity in mdps/LSB indexed by GyroFs
static float gyroSensitivityMdpsLsb(GyroFs fs) {
  switch (fs) {
    case GyroFs::DPS_125:  return 4.375f;
    case GyroFs::DPS_250:  return 8.75f;
    case GyroFs::DPS_500:  return 17.50f;
    case GyroFs::DPS_1000: return 35.0f;
    case GyroFs::DPS_2000: return 70.0f;
    default:               return 8.75f;
  }
}

}  // namespace

// ===========================================================================
// Public: Lifecycle
// ===========================================================================

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

  // --- Validate config ---
  if (config.i2cWrite == nullptr || config.i2cWriteRead == nullptr) {
    return Status::Error(Err::INVALID_CONFIG, "I2C callbacks not set");
  }
  if (config.i2cTimeoutMs == 0) {
    return Status::Error(Err::INVALID_CONFIG, "I2C timeout must be > 0");
  }
  if (config.i2cAddress != 0x6A && config.i2cAddress != 0x6B) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid I2C address");
  }
  if (!isValidOdr(config.odrXl) || !isValidOdr(config.odrG)) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid ODR value");
  }
  if (!isValidAccelFs(config.fsXl)) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid accel full-scale");
  }
  if (!isValidGyroFs(config.fsG)) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid gyro full-scale");
  }

  _config = config;
  if (_config.offlineThreshold == 0) {
    _config.offlineThreshold = 1;
  }

  // --- Read WHO_AM_I (raw — not yet initialized) ---
  uint8_t chipId = 0;
  Status st = _readRegisterRaw(cmd::REG_WHO_AM_I, chipId);
  if (!st.ok()) {
    return Status::Error(Err::DEVICE_NOT_FOUND, "Device not responding", st.detail);
  }
  if (chipId != cmd::WHO_AM_I_VALUE) {
    return Status::Error(Err::CHIP_ID_MISMATCH, "Chip ID mismatch", chipId);
  }

  // --- Apply configuration ---
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

  // Check XLDA (accelerometer data ready) as proxy for full sample available.
  // With BDU enabled and both ODRs the same, all data is updated atomically.
  uint8_t statusReg = 0;
  Status st = readRegister(cmd::REG_STATUS_REG, statusReg);
  if (!st.ok()) {
    if (_driverState == DriverState::OFFLINE) {
      _measurementRequested = false;
    }
    return;
  }

  const bool xlReady = (statusReg & cmd::MASK_XLDA) != 0;
  if (!xlReady) {
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
  _hasSample = true;
  _measurementRequested = false;
}

void LSM6DS3TR::end() {
  if (_initialized) {
    // Best-effort: power down both sensors.
    // Uses raw I2C to avoid health tracking during shutdown.
    const uint8_t payloadXl[2] = { cmd::REG_CTRL1_XL, 0x00 };
    (void)_i2cWriteRaw(payloadXl, sizeof(payloadXl));
    const uint8_t payloadG[2] = { cmd::REG_CTRL2_G, 0x00 };
    (void)_i2cWriteRaw(payloadG, sizeof(payloadG));
  }

  _initialized = false;
  _driverState = DriverState::UNINIT;
  _measurementRequested = false;
  _measurementReady = false;
  _hasSample = false;
  _rawMeasurement = RawMeasurement{};
}

// ===========================================================================
// Public: Diagnostics
// ===========================================================================

Status LSM6DS3TR::probe() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
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
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  uint8_t chipId = 0;
  Status st = readRegister(cmd::REG_WHO_AM_I, chipId);
  if (!st.ok()) {
    return st;
  }
  if (chipId != cmd::WHO_AM_I_VALUE) {
    return Status::Error(Err::CHIP_ID_MISMATCH, "Chip ID mismatch", chipId);
  }

  // Re-apply configuration (device may have reset)
  st = _applyConfig();
  if (!st.ok()) {
    return st;
  }

  return Status::Ok();
}

// ===========================================================================
// Public: Measurement API
// ===========================================================================

Status LSM6DS3TR::requestMeasurement() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (_driverState == DriverState::OFFLINE) {
    return Status::Error(Err::BUSY, "Driver is offline; call recover()");
  }
  if (_config.odrXl == Odr::POWER_DOWN && _config.odrG == Odr::POWER_DOWN) {
    return Status::Error(Err::INVALID_PARAM, "Both sensors powered down");
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

// ===========================================================================
// Public: Direct Read API
// ===========================================================================

Status LSM6DS3TR::readAccelRaw(RawAxes& out) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  uint8_t data[cmd::DATA_LEN_ACCEL] = {};
  Status st = readRegs(cmd::REG_DATA_START_ACCEL, data, sizeof(data));
  if (!st.ok()) {
    return st;
  }

  out.x = static_cast<int16_t>((data[1] << 8) | data[0]);
  out.y = static_cast<int16_t>((data[3] << 8) | data[2]);
  out.z = static_cast<int16_t>((data[5] << 8) | data[4]);
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

  out.x = static_cast<int16_t>((data[1] << 8) | data[0]);
  out.y = static_cast<int16_t>((data[3] << 8) | data[2]);
  out.z = static_cast<int16_t>((data[5] << 8) | data[4]);
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

  out = static_cast<int16_t>((data[1] << 8) | data[0]);
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

  // Bytes 0-1: Temperature
  out.temperature = static_cast<int16_t>((data[1] << 8) | data[0]);
  // Bytes 2-7: Gyroscope X, Y, Z
  out.gyro.x = static_cast<int16_t>((data[3] << 8) | data[2]);
  out.gyro.y = static_cast<int16_t>((data[5] << 8) | data[4]);
  out.gyro.z = static_cast<int16_t>((data[7] << 8) | data[6]);
  // Bytes 8-13: Accelerometer X, Y, Z
  out.accel.x = static_cast<int16_t>((data[9] << 8) | data[8]);
  out.accel.y = static_cast<int16_t>((data[11] << 8) | data[10]);
  out.accel.z = static_cast<int16_t>((data[13] << 8) | data[12]);

  return Status::Ok();
}

Axes LSM6DS3TR::convertAccel(const RawAxes& raw) const {
  const float sens = accelSensitivityMgLsb(_config.fsXl);
  Axes result;
  result.x = static_cast<float>(raw.x) * sens * 0.001f;  // mg -> g
  result.y = static_cast<float>(raw.y) * sens * 0.001f;
  result.z = static_cast<float>(raw.z) * sens * 0.001f;
  return result;
}

Axes LSM6DS3TR::convertGyro(const RawAxes& raw) const {
  const float sens = gyroSensitivityMdpsLsb(_config.fsG);
  Axes result;
  result.x = static_cast<float>(raw.x) * sens * 0.001f;  // mdps -> dps
  result.y = static_cast<float>(raw.y) * sens * 0.001f;
  result.z = static_cast<float>(raw.z) * sens * 0.001f;
  return result;
}

float LSM6DS3TR::convertTemperature(int16_t raw) const {
  return static_cast<float>(raw) / 256.0f + 25.0f;
}

// ===========================================================================
// Public: Configuration
// ===========================================================================

Status LSM6DS3TR::setAccelOdr(Odr odr) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!isValidOdr(odr)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid ODR");
  }

  const uint8_t reg = _buildCtrl1Xl(odr, _config.fsXl);
  Status st = writeRegister(cmd::REG_CTRL1_XL, reg);
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
  if (!isValidOdr(odr)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid ODR");
  }

  const uint8_t reg = _buildCtrl2G(odr, _config.fsG);
  Status st = writeRegister(cmd::REG_CTRL2_G, reg);
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

  const uint8_t reg = _buildCtrl1Xl(_config.odrXl, fs);
  Status st = writeRegister(cmd::REG_CTRL1_XL, reg);
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

  const uint8_t reg = _buildCtrl2G(_config.odrG, fs);
  Status st = writeRegister(cmd::REG_CTRL2_G, reg);
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

Status LSM6DS3TR::softReset() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  _measurementRequested = false;
  _measurementReady = false;
  _hasSample = false;

  Status st = writeRegister(cmd::REG_CTRL3_C, cmd::MASK_SW_RESET);
  if (!st.ok()) {
    return st;
  }

  // Poll for reset completion using RAW reads.
  // During reset (~15 us typ.) the device may NACK — expected.
  const uint32_t deadline = _nowMs() + RESET_TIMEOUT_MS;
  bool resetDone = false;
  for (uint16_t poll = 0; poll < RESET_MAX_POLLS; ++poll) {
    if (deadlineReached(_nowMs(), deadline)) {
      return Status::Error(Err::TIMEOUT, "Reset timeout");
    }
    uint8_t ctrl3 = 0;
    st = _readRegisterRaw(cmd::REG_CTRL3_C, ctrl3);
    if (st.ok() && (ctrl3 & cmd::MASK_SW_RESET) == 0) {
      resetDone = true;
      break;
    }
  }
  if (!resetDone) {
    return Status::Error(Err::TIMEOUT, "Reset polling limit", RESET_MAX_POLLS);
  }

  return _applyConfig();
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
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  uint8_t status = 0;
  Status st = readRegister(cmd::REG_STATUS_REG, status);
  if (!st.ok()) {
    return st;
  }
  ready = (status & cmd::MASK_XLDA) != 0;
  return Status::Ok();
}

Status LSM6DS3TR::isGyroDataReady(bool& ready) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  uint8_t status = 0;
  Status st = readRegister(cmd::REG_STATUS_REG, status);
  if (!st.ok()) {
    return st;
  }
  ready = (status & cmd::MASK_GDA) != 0;
  return Status::Ok();
}

Status LSM6DS3TR::isTempDataReady(bool& ready) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  uint8_t status = 0;
  Status st = readRegister(cmd::REG_STATUS_REG, status);
  if (!st.ok()) {
    return st;
  }
  ready = (status & cmd::MASK_TDA) != 0;
  return Status::Ok();
}

// ===========================================================================
// Public: Sensitivity Helpers
// ===========================================================================

float LSM6DS3TR::accelSensitivity() const {
  return accelSensitivityMgLsb(_config.fsXl);
}

float LSM6DS3TR::gyroSensitivity() const {
  return gyroSensitivityMdpsLsb(_config.fsG);
}

// ===========================================================================
// Private: Transport Wrappers
// ===========================================================================

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

  Status st = _i2cWriteRaw(buf, len);
  if (st.code == Err::INVALID_CONFIG || st.code == Err::INVALID_PARAM) {
    return st;
  }
  return _updateHealth(st);
}

// ===========================================================================
// Private: Register Access
// ===========================================================================

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

// ===========================================================================
// Private: Health Management
// ===========================================================================

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

// ===========================================================================
// Private: Internal Helpers
// ===========================================================================

Status LSM6DS3TR::_applyConfig() {
  // 1. Set CTRL3_C: BDU + IF_INC (mandatory for burst reads)
  uint8_t ctrl3 = cmd::MASK_IF_INC;
  if (_config.bdu) {
    ctrl3 |= cmd::MASK_BDU;
  }
  Status st = writeRegister(cmd::REG_CTRL3_C, ctrl3);
  if (!st.ok()) {
    return st;
  }

  // 2. Set accelerometer ODR + FS
  const uint8_t ctrl1 = _buildCtrl1Xl(_config.odrXl, _config.fsXl);
  st = writeRegister(cmd::REG_CTRL1_XL, ctrl1);
  if (!st.ok()) {
    return st;
  }

  // 3. Set gyroscope ODR + FS
  const uint8_t ctrl2 = _buildCtrl2G(_config.odrG, _config.fsG);
  st = writeRegister(cmd::REG_CTRL2_G, ctrl2);
  if (!st.ok()) {
    return st;
  }

  return Status::Ok();
}

Status LSM6DS3TR::_readRawAll() {
  uint8_t data[cmd::DATA_LEN_ALL] = {};
  Status st = readRegs(cmd::REG_DATA_START_ALL, data, sizeof(data));
  if (!st.ok()) {
    return st;
  }

  // Bytes 0-1: Temperature
  _rawMeasurement.temperature = static_cast<int16_t>((data[1] << 8) | data[0]);
  // Bytes 2-7: Gyroscope X, Y, Z
  _rawMeasurement.gyro.x = static_cast<int16_t>((data[3] << 8) | data[2]);
  _rawMeasurement.gyro.y = static_cast<int16_t>((data[5] << 8) | data[4]);
  _rawMeasurement.gyro.z = static_cast<int16_t>((data[7] << 8) | data[6]);
  // Bytes 8-13: Accelerometer X, Y, Z
  _rawMeasurement.accel.x = static_cast<int16_t>((data[9] << 8) | data[8]);
  _rawMeasurement.accel.y = static_cast<int16_t>((data[11] << 8) | data[10]);
  _rawMeasurement.accel.z = static_cast<int16_t>((data[13] << 8) | data[12]);

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
    // FS_125 bit (bit 1) = 1, FS_G[1:0] = 00
    return static_cast<uint8_t>((static_cast<uint8_t>(odr) << cmd::BIT_ODR_G) |
                                cmd::MASK_FS_125);
  }
  return static_cast<uint8_t>((static_cast<uint8_t>(odr) << cmd::BIT_ODR_G) |
                              (static_cast<uint8_t>(fs) << cmd::BIT_FS_G));
}

}  // namespace LSM6DS3TR
