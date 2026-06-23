/**
 * @file LSM6DS3TR.cpp
 * @brief LSM6DS3TR-C driver implementation.
 */

#include "LSM6DS3TR/LSM6DS3TR.h"

#include <cmath>
#include <cstring>
#include <limits>

namespace LSM6DS3TR {
namespace {

static constexpr size_t MAX_WRITE_LEN = 16;
static constexpr size_t MAX_PUBLIC_READ_LEN = 128;
static constexpr uint32_t RESET_TIMEOUT_MS = 25;
static constexpr uint32_t BOOT_TIMEOUT_MS = 25;
static constexpr uint16_t RESET_MAX_POLLS = 255;
static constexpr uint16_t SAMPLE_READY_MAX_POLLS = 255;
static constexpr uint16_t CALIBRATION_MAX_POLLS_PER_SAMPLE = 255;
static constexpr uint16_t SELF_TEST_MAX_SAMPLES = 100;
static constexpr uint16_t SELF_TEST_READY_MAX_POLLS = 512;
static constexpr uint16_t SELF_TEST_SETTLE_SAMPLES = 5;
static constexpr uint32_t SELF_TEST_ACCEL_SETTLE_MS = 100;
static constexpr uint32_t SELF_TEST_GYRO_SETTLE_MS = 800;
static constexpr uint32_t SELF_TEST_GYRO_STIMULUS_SETTLE_MS = 60;
static constexpr uint32_t SELF_TEST_SETTLE_POLLS_PER_MS = 16;
static constexpr float SELF_TEST_ACCEL_MIN_MG = 90.0f;
static constexpr float SELF_TEST_ACCEL_MAX_MG = 1700.0f;
static constexpr float SELF_TEST_GYRO_MIN_DPS = 150.0f;
static constexpr float SELF_TEST_GYRO_MAX_DPS = 700.0f;
static constexpr float SELF_TEST_ACCEL_SENSITIVITY_G_LSB = 0.000061f;
static constexpr float SELF_TEST_GYRO_SENSITIVITY_DPS_LSB = 0.070f;

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

static bool isFiniteInRange(float value, float minValue, float maxValue) {
  return std::isfinite(value) && value >= minValue && value <= maxValue;
}

static bool isValidCalibrationLimits(const CalibrationLimits& limits) {
  return isFiniteInRange(limits.accelMaxPeakToPeakG, 0.001f, 4.0f) &&
         isFiniteInRange(limits.accelMaxHorizontalMeanG, 0.0f, 4.0f) &&
         isFiniteInRange(limits.accelMinZMeanG, -4.0f, 4.0f) &&
         isFiniteInRange(limits.accelMaxZMeanG, -4.0f, 4.0f) &&
         limits.accelMinZMeanG <= limits.accelMaxZMeanG &&
         isFiniteInRange(limits.gyroMaxPeakToPeakDps, 0.001f, 2000.0f);
}

static bool isCachedConfigRegister(uint8_t reg) {
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
      return true;
    default:
      return false;
  }
}

static bool isSafePublicRegisterWrite(uint8_t reg, uint8_t value) {
  switch (reg) {
    case cmd::REG_FUNC_CFG_ACCESS:
      return (value & (cmd::MASK_FUNC_CFG_EN | cmd::MASK_FUNC_CFG_EN_B)) == 0;
    case cmd::REG_CTRL3_C:
      return (value & (cmd::MASK_BOOT | cmd::MASK_SW_RESET | cmd::MASK_BLE)) == 0 &&
             (value & cmd::MASK_IF_INC) != 0;
    case cmd::REG_CTRL4_C:
      return (value & cmd::MASK_I2C_DISABLE) == 0;
    default:
      return true;
  }
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

static void decodeRawAll(const uint8_t* data, RawMeasurement& out) {
  out.temperature = static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) | data[0]);
  out.gyro.x = static_cast<int16_t>((static_cast<uint16_t>(data[3]) << 8) | data[2]);
  out.gyro.y = static_cast<int16_t>((static_cast<uint16_t>(data[5]) << 8) | data[4]);
  out.gyro.z = static_cast<int16_t>((static_cast<uint16_t>(data[7]) << 8) | data[6]);
  out.accel.x = static_cast<int16_t>((static_cast<uint16_t>(data[9]) << 8) | data[8]);
  out.accel.y = static_cast<int16_t>((static_cast<uint16_t>(data[11]) << 8) | data[10]);
  out.accel.z = static_cast<int16_t>((static_cast<uint16_t>(data[13]) << 8) | data[12]);
}

static void decodeFifoStatus(const uint8_t* data, FifoStatus& out) {
  out.unreadWords = static_cast<uint16_t>(data[0]) |
                    (static_cast<uint16_t>(data[1] & cmd::MASK_DIFF_FIFO_HI) << 8);
  out.watermark = (data[1] & cmd::MASK_FIFO_WATERM) != 0;
  out.overrun = (data[1] & cmd::MASK_FIFO_OVER_RUN) != 0;
  out.fullSmart = (data[1] & cmd::MASK_FIFO_FULL_SMART) != 0;
  out.empty = (data[1] & cmd::MASK_FIFO_EMPTY) != 0;
  out.pattern = static_cast<uint16_t>(data[2]) |
                (static_cast<uint16_t>(data[3] & 0x03u) << 8);
}

static Status normalizeTransportStatus(Status st) {
  if (st.code == Err::IN_PROGRESS) {
    return Status::Error(Err::I2C_BUSY, "Transport busy", st.detail);
  }
  return st;
}

static float absFloat(float value) {
  return value < 0.0f ? -value : value;
}

static bool axisInRange(float value, float minValue, float maxValue) {
  return value >= minValue && value <= maxValue;
}

static void addCalibrationSample(const Axes& sample, uint16_t samplesDone,
                                 double& sumX, double& sumY, double& sumZ,
                                 float& minX, float& maxX,
                                 float& minY, float& maxY,
                                 float& minZ, float& maxZ) {
  sumX += static_cast<double>(sample.x);
  sumY += static_cast<double>(sample.y);
  sumZ += static_cast<double>(sample.z);

  if (samplesDone == 0u) {
    minX = maxX = sample.x;
    minY = maxY = sample.y;
    minZ = maxZ = sample.z;
    return;
  }

  if (sample.x < minX) minX = sample.x;
  if (sample.x > maxX) maxX = sample.x;
  if (sample.y < minY) minY = sample.y;
  if (sample.y > maxY) maxY = sample.y;
  if (sample.z < minZ) minZ = sample.z;
  if (sample.z > maxZ) maxZ = sample.z;
}

static Status finalizeAccelCalibration(uint16_t samples,
                                       double sumX, double sumY, double sumZ,
                                       float minX, float maxX,
                                       float minY, float maxY,
                                       float minZ, float maxZ,
                                       const CalibrationLimits& limits,
                                       Axes& out) {
  const float p2pX = maxX - minX;
  const float p2pY = maxY - minY;
  const float p2pZ = maxZ - minZ;
  if (p2pX > limits.accelMaxPeakToPeakG ||
      p2pY > limits.accelMaxPeakToPeakG ||
      p2pZ > limits.accelMaxPeakToPeakG) {
    return Status::Error(Err::CALIBRATION_UNSTABLE, "Accel calibration unstable");
  }

  const double n = static_cast<double>(samples);
  const float meanX = static_cast<float>(sumX / n);
  const float meanY = static_cast<float>(sumY / n);
  const float meanZ = static_cast<float>(sumZ / n);
  if (absFloat(meanX) > limits.accelMaxHorizontalMeanG ||
      absFloat(meanY) > limits.accelMaxHorizontalMeanG ||
      meanZ < limits.accelMinZMeanG ||
      meanZ > limits.accelMaxZMeanG) {
    return Status::Error(Err::CALIBRATION_ORIENTATION, "Accel calibration orientation");
  }

  out.x = meanX;
  out.y = meanY;
  out.z = meanZ - 1.0f;
  return Status::Ok();
}

static Status finalizeGyroCalibration(uint16_t samples,
                                      double sumX, double sumY, double sumZ,
                                      float minX, float maxX,
                                      float minY, float maxY,
                                      float minZ, float maxZ,
                                      const CalibrationLimits& limits,
                                      Axes& out) {
  const float p2pX = maxX - minX;
  const float p2pY = maxY - minY;
  const float p2pZ = maxZ - minZ;
  if (p2pX > limits.gyroMaxPeakToPeakDps ||
      p2pY > limits.gyroMaxPeakToPeakDps ||
      p2pZ > limits.gyroMaxPeakToPeakDps) {
    return Status::Error(Err::CALIBRATION_UNSTABLE, "Gyro calibration unstable");
  }

  const double n = static_cast<double>(samples);
  out.x = static_cast<float>(sumX / n);
  out.y = static_cast<float>(sumY / n);
  out.z = static_cast<float>(sumZ / n);
  return Status::Ok();
}

static Axes rawToAccelSelfTestAxes(const RawAxes& raw) {
  return Axes{
      static_cast<float>(raw.x) * SELF_TEST_ACCEL_SENSITIVITY_G_LSB,
      static_cast<float>(raw.y) * SELF_TEST_ACCEL_SENSITIVITY_G_LSB,
      static_cast<float>(raw.z) * SELF_TEST_ACCEL_SENSITIVITY_G_LSB,
  };
}

static Axes rawToGyroSelfTestAxes(const RawAxes& raw) {
  return Axes{
      static_cast<float>(raw.x) * SELF_TEST_GYRO_SENSITIVITY_DPS_LSB,
      static_cast<float>(raw.y) * SELF_TEST_GYRO_SENSITIVITY_DPS_LSB,
      static_cast<float>(raw.z) * SELF_TEST_GYRO_SENSITIVITY_DPS_LSB,
  };
}

static Axes deltaAxes(const Axes& stimulus, const Axes& baseline) {
  return Axes{
      absFloat(stimulus.x - baseline.x),
      absFloat(stimulus.y - baseline.y),
      absFloat(stimulus.z - baseline.z),
  };
}

static bool accelSelfTestPass(const Axes& delta) {
  return axisInRange(delta.x * 1000.0f, SELF_TEST_ACCEL_MIN_MG, SELF_TEST_ACCEL_MAX_MG) &&
         axisInRange(delta.y * 1000.0f, SELF_TEST_ACCEL_MIN_MG, SELF_TEST_ACCEL_MAX_MG) &&
         axisInRange(delta.z * 1000.0f, SELF_TEST_ACCEL_MIN_MG, SELF_TEST_ACCEL_MAX_MG);
}

static bool gyroSelfTestPass(const Axes& delta) {
  return axisInRange(delta.x, SELF_TEST_GYRO_MIN_DPS, SELF_TEST_GYRO_MAX_DPS) &&
         axisInRange(delta.y, SELF_TEST_GYRO_MIN_DPS, SELF_TEST_GYRO_MAX_DPS) &&
         axisInRange(delta.z, SELF_TEST_GYRO_MIN_DPS, SELF_TEST_GYRO_MAX_DPS);
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
  _allowOfflineI2c = false;
  _cachedConfigDirty = false;

  _measurementRequested = false;
  _measurementReady = false;
  _hasSample = false;
  _sampleTimestampMs = 0;
  _rawMeasurement = RawMeasurement{};

  _pollJob = PollJob::NONE;
  _pollStep = 0;
  _pollCount = 0;
  _pollNowMs = 0;
  _pollDeadlineMs = 0;
  _pollDeadlineArmed = false;
  _pollInstructionUsed = false;
  _pollStartedOffline = false;
  _lastPollStatus = Status::Ok();
  _pollWakeUpDur = 0;
  _fifoDrainMaxWords = 0;
  _fifoDrainWordsRead = 0;
  _fifoDrainWordsAvailable = 0;
  _refreshCtrl1 = 0;
  _refreshCtrl2 = 0;
  _refreshCtrl3 = 0;
  _refreshCtrl4 = 0;
  _refreshCtrl6 = 0;
  _refreshCtrl7 = 0;
  _refreshCtrl8 = 0;
  _refreshCtrl10 = 0;
  _refreshWakeUpDur = 0;
  std::memset(_refreshOffsetData, 0, sizeof(_refreshOffsetData));
  std::memset(_refreshFifoCtrl, 0, sizeof(_refreshFifoCtrl));
  _calibrationSamplesTarget = 0;
  _calibrationSamplesDone = 0;
  _calibrationPollsForSample = 0;
  _calibrationSumX = 0.0;
  _calibrationSumY = 0.0;
  _calibrationSumZ = 0.0;
  _calibrationMinX = 0.0f;
  _calibrationMaxX = 0.0f;
  _calibrationMinY = 0.0f;
  _calibrationMaxY = 0.0f;
  _calibrationMinZ = 0.0f;
  _calibrationMaxZ = 0.0f;

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
    return Status::Error(Err::INVALID_CONFIG,
                         "Accel 1.6 Hz requires low-power/normal mode; high-performance supports 12.5 Hz+");
  }
  if (!isValidGyroOdr(config.odrG, config.gyroPowerMode)) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid gyro ODR/power-mode combination");
  }
  if (!isValidCalibrationLimits(config.calibrationLimits)) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid calibration limits");
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
    return st;
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
  _cachedConfigDirty = false;
  return Status::Ok();
}

void LSM6DS3TR::tick(uint32_t nowMs) {
  (void)poll(nowMs, 1);
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
  _sampleTimestampMs = 0;
  _rawMeasurement = RawMeasurement{};
  _pollJob = PollJob::NONE;
  _pollStep = 0;
  _pollDeadlineMs = 0;
  _pollDeadlineArmed = false;
  _pollStartedOffline = false;
  _lastPollStatus = Status::Ok();
}

Status LSM6DS3TR::probe() {
  if (_config.i2cWriteRead == nullptr) {
    return Status::Error(Err::NOT_INITIALIZED, "No transport configured");
  }

  uint8_t chipId = 0;
  Status st = _readRegisterRaw(cmd::REG_WHO_AM_I, chipId);
  if (!st.ok()) {
    return st;
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

Status LSM6DS3TR::_validateMeasurementRequest(bool checkReady) const {
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
  if (checkReady && accelActive && gyroActive) {
    if (!_config.bdu) {
      return Status::Error(Err::INVALID_PARAM, "Async combined measurement requires BDU");
    }
    if (_config.odrXl != _config.odrG) {
      return Status::Error(Err::INVALID_PARAM, "Async combined measurement requires matching ODR");
    }
  }
  return Status::Ok();
}

Status LSM6DS3TR::requestMeasurement() {
  return requestMeasurement(true);
}

Status LSM6DS3TR::requestMeasurement(bool checkReady) {
  Status st = _validateMeasurementRequest(checkReady);
  if (!st.ok()) {
    return st;
  }
  if (_measurementRequested && !_measurementReady) {
    return Status::Error(Err::BUSY, "Measurement in progress");
  }
  if (pollBusy()) {
    return Status::Error(Err::BUSY, "Poll job in progress");
  }

  _measurementReady = false;
  _measurementRequested = true;
  _pollJob = PollJob::SAMPLE;
  _pollStep = checkReady ? 0 : 1;
  _pollCount = 0;
  _pollDeadlineMs = 0;
  _pollDeadlineArmed = false;
  _pollInstructionUsed = false;
  _pollStartedOffline = false;
  _lastPollStatus = Status::Error(Err::IN_PROGRESS, "Measurement scheduled");
  return Status::Error(Err::IN_PROGRESS, "Measurement scheduled");
}

bool LSM6DS3TR::pollBusy() const {
  return _pollJob != PollJob::NONE;
}

Status LSM6DS3TR::_startPollJob(uint8_t job, const Status& busyStatus) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  const PollJob pollJob = static_cast<PollJob>(job);
  if (_driverState == DriverState::OFFLINE && pollJob != PollJob::SOFT_RESET &&
      pollJob != PollJob::BOOT) {
    return Status::Error(Err::BUSY, "Driver is offline; call recover()");
  }
  if (pollBusy() || (_measurementRequested && !_measurementReady)) {
    return busyStatus;
  }

  _pollJob = pollJob;
  _pollStep = 0;
  _pollCount = 0;
  _pollDeadlineMs = 0;
  _pollDeadlineArmed = false;
  _pollInstructionUsed = false;
  _pollStartedOffline = _driverState == DriverState::OFFLINE;
  _lastPollStatus = Status::Error(Err::IN_PROGRESS, "Poll job scheduled");
  return _lastPollStatus;
}

Status LSM6DS3TR::startSoftReset() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  Status st = _startPollJob(static_cast<uint8_t>(PollJob::SOFT_RESET),
                            Status::Error(Err::BUSY, "Poll job in progress"));
  if (!st.ok() && !st.inProgress()) {
    return st;
  }
  _measurementRequested = false;
  _measurementReady = false;
  _hasSample = false;
  _sampleTimestampMs = 0;
  return st;
}

Status LSM6DS3TR::startBoot() {
  return _startPollJob(static_cast<uint8_t>(PollJob::BOOT),
                       Status::Error(Err::BUSY, "Poll job in progress"));
}

Status LSM6DS3TR::startRefreshCachedConfig() {
  return _startPollJob(static_cast<uint8_t>(PollJob::REFRESH_CONFIG),
                       Status::Error(Err::BUSY, "Poll job in progress"));
}

Status LSM6DS3TR::runSelfTest(SelfTestResult& out, uint16_t samples) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (samples == 0u || samples > SELF_TEST_MAX_SAMPLES) {
    return Status::Error(Err::INVALID_PARAM, "self-test samples must be 1..100");
  }

  out = SelfTestResult{};

  uint8_t savedCtrl1 = 0;
  uint8_t savedCtrl2 = 0;
  uint8_t savedCtrl5 = 0;
  uint8_t savedCtrl7 = 0;
  uint8_t savedCtrl8 = 0;
  Status st = readRegister(cmd::REG_CTRL1_XL, savedCtrl1);
  if (!st.ok()) return st;
  st = readRegister(cmd::REG_CTRL2_G, savedCtrl2);
  if (!st.ok()) return st;
  st = readRegister(cmd::REG_CTRL5_C, savedCtrl5);
  if (!st.ok()) return st;
  st = readRegister(cmd::REG_CTRL7_G, savedCtrl7);
  if (!st.ok()) return st;
  st = readRegister(cmd::REG_CTRL8_XL, savedCtrl8);
  if (!st.ok()) return st;

  auto restoreRegisters = [this, savedCtrl1, savedCtrl2, savedCtrl5,
                           savedCtrl7, savedCtrl8]() -> Status {
    ScopedOfflineI2cAllowance allowOfflineI2c(_allowOfflineI2c, true);
    Status first = Status::Ok();
    auto keepFirstFailure = [&first](Status restoreStatus) {
      if (!restoreStatus.ok() && first.ok()) {
        first = restoreStatus;
      }
    };
    keepFirstFailure(writeRegister(cmd::REG_CTRL5_C, savedCtrl5));
    keepFirstFailure(writeRegister(cmd::REG_CTRL8_XL, savedCtrl8));
    keepFirstFailure(writeRegister(cmd::REG_CTRL7_G, savedCtrl7));
    keepFirstFailure(writeRegister(cmd::REG_CTRL2_G, savedCtrl2));
    keepFirstFailure(writeRegister(cmd::REG_CTRL1_XL, savedCtrl1));
    return first;
  };

  Status result = Status::Ok();
  RawAxes baselineRaw;
  RawAxes stimulusRaw;
  RawAxes discardRaw;

  st = writeRegister(cmd::REG_CTRL5_C, 0x00);
  if (!st.ok()) {
    result = st;
  }
  if (result.ok()) {
    st = writeRegister(cmd::REG_CTRL8_XL, 0x00);
    if (!st.ok()) result = st;
  }
  if (result.ok()) {
    st = writeRegister(cmd::REG_CTRL1_XL, _buildCtrl1Xl(Odr::HZ_416, AccelFs::G_2));
    if (!st.ok()) result = st;
  }
  if (result.ok()) {
    st = _settleSelfTest(SELF_TEST_ACCEL_SETTLE_MS);
    if (!st.ok()) result = st;
  }
  if (result.ok()) {
    st = _readSelfTestAverage(true, SELF_TEST_SETTLE_SAMPLES, discardRaw);
    if (!st.ok()) result = st;
  }
  if (result.ok()) {
    st = _readSelfTestAverage(true, samples, baselineRaw);
    if (!st.ok()) result = st;
  }
  if (result.ok()) {
    st = writeRegister(cmd::REG_CTRL5_C,
                       static_cast<uint8_t>(1u << cmd::BIT_ST_XL));
    if (!st.ok()) result = st;
  }
  if (result.ok()) {
    st = _settleSelfTest(SELF_TEST_ACCEL_SETTLE_MS);
    if (!st.ok()) result = st;
  }
  if (result.ok()) {
    st = _readSelfTestAverage(true, SELF_TEST_SETTLE_SAMPLES, discardRaw);
    if (!st.ok()) result = st;
  }
  if (result.ok()) {
    st = _readSelfTestAverage(true, samples, stimulusRaw);
    if (!st.ok()) result = st;
  }
  if (result.ok()) {
    out.accelBaseline = rawToAccelSelfTestAxes(baselineRaw);
    out.accelStimulus = rawToAccelSelfTestAxes(stimulusRaw);
    out.accelDelta = deltaAxes(out.accelStimulus, out.accelBaseline);
    out.accelPass = accelSelfTestPass(out.accelDelta);
    if (!out.accelPass) {
      result = Status::Error(Err::SELF_TEST_FAIL, "Accel self-test delta out of range");
    }
  }

  bool gyroSequenceOk = result.ok() || result.code == Err::SELF_TEST_FAIL;
  if (gyroSequenceOk) {
    st = writeRegister(cmd::REG_CTRL5_C, 0x00);
    if (!st.ok()) {
      result = st;
      gyroSequenceOk = false;
    }
  }
  if (gyroSequenceOk) {
    st = writeRegister(cmd::REG_CTRL7_G, 0x00);
    if (!st.ok()) {
      result = st;
      gyroSequenceOk = false;
    }
  }
  if (gyroSequenceOk) {
    st = writeRegister(cmd::REG_CTRL2_G, _buildCtrl2G(Odr::HZ_416, GyroFs::DPS_2000));
    if (!st.ok()) {
      result = st;
      gyroSequenceOk = false;
    }
  }
  if (gyroSequenceOk) {
    st = _settleSelfTest(SELF_TEST_GYRO_SETTLE_MS);
    if (!st.ok()) {
      result = st;
      gyroSequenceOk = false;
    }
  }
  if (gyroSequenceOk) {
    st = _readSelfTestAverage(false, SELF_TEST_SETTLE_SAMPLES, discardRaw);
    if (!st.ok()) {
      result = st;
      gyroSequenceOk = false;
    }
  }
  if (gyroSequenceOk) {
    st = _readSelfTestAverage(false, samples, baselineRaw);
    if (!st.ok()) {
      result = st;
      gyroSequenceOk = false;
    }
  }
  if (gyroSequenceOk) {
    st = writeRegister(cmd::REG_CTRL5_C,
                       static_cast<uint8_t>(1u << cmd::BIT_ST_G));
    if (!st.ok()) {
      result = st;
      gyroSequenceOk = false;
    }
  }
  if (gyroSequenceOk) {
    st = _settleSelfTest(SELF_TEST_GYRO_STIMULUS_SETTLE_MS);
    if (!st.ok()) {
      result = st;
      gyroSequenceOk = false;
    }
  }
  if (gyroSequenceOk) {
    st = _readSelfTestAverage(false, SELF_TEST_SETTLE_SAMPLES, discardRaw);
    if (!st.ok()) {
      result = st;
      gyroSequenceOk = false;
    }
  }
  if (gyroSequenceOk) {
    st = _readSelfTestAverage(false, samples, stimulusRaw);
    if (!st.ok()) {
      result = st;
      gyroSequenceOk = false;
    }
  }
  if (gyroSequenceOk) {
    out.gyroBaseline = rawToGyroSelfTestAxes(baselineRaw);
    out.gyroStimulus = rawToGyroSelfTestAxes(stimulusRaw);
    out.gyroDelta = deltaAxes(out.gyroStimulus, out.gyroBaseline);
    out.gyroPass = gyroSelfTestPass(out.gyroDelta);
    if (!out.gyroPass && result.ok()) {
      result = Status::Error(Err::SELF_TEST_FAIL, "Gyro self-test delta out of range");
    }
  }

  const Status restoreStatus = restoreRegisters();
  if (result.ok() && !restoreStatus.ok()) {
    return restoreStatus;
  }
  return result;
}

Status LSM6DS3TR::startFifoDrain(uint16_t maxWords) {
  if (maxWords == 0) {
    return Status::Error(Err::INVALID_PARAM, "maxWords must be > 0");
  }
  Status st = _startPollJob(static_cast<uint8_t>(PollJob::FIFO_DRAIN),
                            Status::Error(Err::BUSY, "Poll job in progress"));
  if (!st.ok() && !st.inProgress()) {
    return st;
  }
  _fifoDrainMaxWords = maxWords;
  _fifoDrainWordsRead = 0;
  _fifoDrainWordsAvailable = 0;
  return st;
}

Status LSM6DS3TR::startAccelBiasCapture(uint16_t samples) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (samples == 0 || samples > 10000) {
    return Status::Error(Err::INVALID_PARAM, "samples must be 1..10000");
  }
  if (!isActiveOdr(_config.odrXl)) {
    return Status::Error(Err::INVALID_PARAM, "Accel ODR is POWER_DOWN");
  }
  Status st = _startPollJob(static_cast<uint8_t>(PollJob::ACCEL_CALIBRATION),
                            Status::Error(Err::BUSY, "Poll job in progress"));
  if (!st.ok() && !st.inProgress()) {
    return st;
  }
  _calibrationSamplesTarget = samples;
  _calibrationSamplesDone = 0;
  _calibrationPollsForSample = 0;
  _calibrationSumX = 0.0;
  _calibrationSumY = 0.0;
  _calibrationSumZ = 0.0;
  _calibrationMinX = 0.0f;
  _calibrationMaxX = 0.0f;
  _calibrationMinY = 0.0f;
  _calibrationMaxY = 0.0f;
  _calibrationMinZ = 0.0f;
  _calibrationMaxZ = 0.0f;
  _pollDeadlineMs = 0;
  _pollDeadlineArmed = false;
  return st;
}

Status LSM6DS3TR::startGyroBiasCapture(uint16_t samples) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (samples == 0 || samples > 10000) {
    return Status::Error(Err::INVALID_PARAM, "samples must be 1..10000");
  }
  if (!isActiveOdr(_config.odrG)) {
    return Status::Error(Err::INVALID_PARAM, "Gyro ODR is POWER_DOWN");
  }
  Status st = _startPollJob(static_cast<uint8_t>(PollJob::GYRO_CALIBRATION),
                            Status::Error(Err::BUSY, "Poll job in progress"));
  if (!st.ok() && !st.inProgress()) {
    return st;
  }
  _calibrationSamplesTarget = samples;
  _calibrationSamplesDone = 0;
  _calibrationPollsForSample = 0;
  _calibrationSumX = 0.0;
  _calibrationSumY = 0.0;
  _calibrationSumZ = 0.0;
  _calibrationMinX = 0.0f;
  _calibrationMaxX = 0.0f;
  _calibrationMinY = 0.0f;
  _calibrationMaxY = 0.0f;
  _calibrationMinZ = 0.0f;
  _calibrationMaxZ = 0.0f;
  _pollDeadlineMs = 0;
  _pollDeadlineArmed = false;
  return st;
}

Status LSM6DS3TR::_finishPollJob(const Status& st) {
  const PollJob finishedJob = _pollJob;
  const bool startedOffline = _pollStartedOffline;
  _lastPollStatus = st;
  if (st.inProgress()) {
    return st;
  }

  _pollJob = PollJob::NONE;
  _pollStep = 0;
  _pollCount = 0;
  _pollDeadlineMs = 0;
  _pollDeadlineArmed = false;
  _pollInstructionUsed = false;
  _pollStartedOffline = false;
  if (startedOffline && !st.ok() &&
      (finishedJob == PollJob::SOFT_RESET || finishedJob == PollJob::BOOT)) {
    _reassertOfflineLatch();
  }
  return st;
}

Status LSM6DS3TR::poll(uint32_t nowMs, uint8_t maxInstructions) {
  _pollNowMs = nowMs;
  if (!pollBusy()) {
    return _lastPollStatus;
  }
  if (maxInstructions == 0) {
    _lastPollStatus = Status::Error(Err::IN_PROGRESS, "Poll job in progress");
    return _lastPollStatus;
  }

  const bool allowOffline =
      _pollJob == PollJob::SOFT_RESET || _pollJob == PollJob::BOOT;
  ScopedOfflineI2cAllowance allowOfflineI2c(_allowOfflineI2c, allowOffline);

  uint8_t instructions = 0;
  while (pollBusy() && instructions < maxInstructions) {
    _pollInstructionUsed = false;
    Status st = Status::Ok();
    switch (_pollJob) {
      case PollJob::SAMPLE:
        st = _pollSampleStep();
        break;
      case PollJob::SOFT_RESET:
        st = _pollResetOrBootStep(false);
        break;
      case PollJob::BOOT:
        st = _pollResetOrBootStep(true);
        break;
      case PollJob::REFRESH_CONFIG: {
        bool done = false;
        st = _pollRefreshStep(_pollStep, done);
        if (st.ok() && done) {
          st = _finishPollJob(Status::Ok());
        }
        break;
      }
      case PollJob::FIFO_DRAIN:
        st = _pollFifoDrainStep();
        break;
      case PollJob::ACCEL_CALIBRATION:
        st = _pollCalibrationStep(true);
        break;
      case PollJob::GYRO_CALIBRATION:
        st = _pollCalibrationStep(false);
        break;
      case PollJob::NONE:
      default:
        return _lastPollStatus;
    }

    if (!st.ok()) {
      if (st.inProgress()) {
        _lastPollStatus = st;
        if (_pollInstructionUsed) {
          ++instructions;
        }
        break;
      }
      return _finishPollJob(st);
    }
    if (_pollInstructionUsed) {
      ++instructions;
    } else if (st.inProgress()) {
      break;
    }
  }

  if (pollBusy()) {
    _lastPollStatus = Status::Error(Err::IN_PROGRESS, "Poll job in progress");
    return _lastPollStatus;
  }
  return _lastPollStatus;
}

Status LSM6DS3TR::_pollSampleStep() {
  if (!_initialized) {
    _measurementRequested = false;
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (_driverState == DriverState::OFFLINE) {
    _measurementRequested = false;
    return Status::Error(Err::BUSY, "Driver is offline; call recover()");
  }

  if (_pollStep == 0) {
    if (!_pollDeadlineArmed) {
      const Odr odr = isActiveOdr(_config.odrXl) ? _config.odrXl : _config.odrG;
      _pollDeadlineMs = _pollNowMs + odrIntervalMs(odr) * 3U + 100U;
      _pollDeadlineArmed = true;
    }
    if (deadlineReached(_pollNowMs, _pollDeadlineMs)) {
      _measurementRequested = false;
      return Status::Error(Err::TIMEOUT, "Measurement ready timeout");
    }
    if (_pollCount >= SAMPLE_READY_MAX_POLLS) {
      _measurementRequested = false;
      return Status::Error(Err::TIMEOUT, "Measurement ready polling limit",
                           SAMPLE_READY_MAX_POLLS);
    }

    uint8_t statusReg = 0;
    _pollInstructionUsed = true;
    ++_pollCount;
    Status st = readRegister(cmd::REG_STATUS_REG, statusReg);
    if (!st.ok()) {
      _measurementRequested = false;
      return st;
    }

    const bool accelActive = isActiveOdr(_config.odrXl);
    const bool gyroActive = isActiveOdr(_config.odrG);
    const bool accelReady = !accelActive || ((statusReg & cmd::MASK_XLDA) != 0);
    const bool gyroReady = !gyroActive || ((statusReg & cmd::MASK_GDA) != 0);
    if (!accelReady || !gyroReady) {
      return Status::Error(Err::IN_PROGRESS, "Measurement not ready");
    }
    _pollStep = 1;
    return Status::Ok();
  }

  _pollInstructionUsed = true;
  Status st = _readRawAllWithTimestamp(_pollNowMs);
  if (!st.ok()) {
    _measurementRequested = false;
    return st;
  }
  _measurementReady = true;
  _measurementRequested = false;
  return _finishPollJob(Status::Ok());
}

Status LSM6DS3TR::_pollApplyConfigStep(uint8_t step, bool& done) {
  done = false;
  _pollInstructionUsed = true;
  Status st = Status::Ok();
  switch (step) {
    case 0:
      st = writeRegister(
          cmd::REG_CTRL3_C,
          static_cast<uint8_t>(cmd::MASK_IF_INC | (_config.bdu ? cmd::MASK_BDU : 0)));
      break;
    case 1:
      st = writeRegister(cmd::REG_CTRL4_C, _buildCtrl4C());
      break;
    case 2:
      st = writeRegister(cmd::REG_CTRL6_C, _buildCtrl6C());
      break;
    case 3:
      st = writeRegister(cmd::REG_CTRL7_G, _buildCtrl7G());
      break;
    case 4:
      st = writeRegister(cmd::REG_CTRL8_XL, _buildCtrl8Xl());
      break;
    case 5:
      st = writeRegister(cmd::REG_CTRL1_XL, _buildCtrl1Xl(_config.odrXl, _config.fsXl));
      break;
    case 6:
      st = writeRegister(cmd::REG_CTRL2_G, _buildCtrl2G(_config.odrG, _config.fsG));
      break;
    case 7:
      st = readRegister(cmd::REG_WAKE_UP_DUR, _pollWakeUpDur);
      break;
    case 8:
      _pollWakeUpDur = static_cast<uint8_t>(
          (_pollWakeUpDur & static_cast<uint8_t>(~cmd::MASK_TIMER_HR)) |
          (_buildWakeUpDur() & cmd::MASK_TIMER_HR));
      st = writeRegister(cmd::REG_WAKE_UP_DUR, _pollWakeUpDur);
      break;
    case 9:
      st = writeRegister(cmd::REG_CTRL10_C, _buildCtrl10C());
      break;
    case 10: {
      const uint8_t offsets[3] = {
          static_cast<uint8_t>(_accelUserOffset.x),
          static_cast<uint8_t>(_accelUserOffset.y),
          static_cast<uint8_t>(_accelUserOffset.z),
      };
      st = writeRegs(cmd::REG_X_OFS_USR, offsets, sizeof(offsets));
      break;
    }
    case 11:
      st = writeRegister(cmd::REG_FIFO_CTRL1, loByte(_fifoConfig.threshold));
      break;
    case 12:
      st = writeRegister(cmd::REG_FIFO_CTRL2, _buildFifoCtrl2());
      break;
    case 13:
      st = writeRegister(cmd::REG_FIFO_CTRL3, _buildFifoCtrl3());
      break;
    case 14:
      st = writeRegister(cmd::REG_FIFO_CTRL4, _buildFifoCtrl4());
      break;
    case 15:
      st = writeRegister(cmd::REG_FIFO_CTRL5, _buildFifoCtrl5());
      break;
    default:
      _pollInstructionUsed = false;
      _cachedConfigDirty = false;
      done = true;
      return Status::Ok();
  }

  if (!st.ok()) {
    _cachedConfigDirty = true;
    done = true;
    return st;
  }
  _pollStep = static_cast<uint8_t>(step + 1u);
  if (_pollStep > 15u) {
    _cachedConfigDirty = false;
    done = true;
  }
  return Status::Ok();
}

Status LSM6DS3TR::_pollResetOrBootStep(bool bootJob) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }

  if (_pollStep == 0) {
    if (!bootJob) {
      _measurementRequested = false;
      _measurementReady = false;
      _hasSample = false;
      _sampleTimestampMs = 0;
    }

    const uint8_t ctrl3 = static_cast<uint8_t>((bootJob ? cmd::MASK_BOOT : cmd::MASK_SW_RESET) |
                                               cmd::MASK_IF_INC |
                                               (_config.bdu ? cmd::MASK_BDU : 0));
    _pollInstructionUsed = true;
    Status st = writeRegister(cmd::REG_CTRL3_C, ctrl3);
    if (!st.ok()) {
      return st;
    }
    _pollDeadlineMs = _pollNowMs + (bootJob ? BOOT_TIMEOUT_MS : RESET_TIMEOUT_MS);
    _pollDeadlineArmed = true;
    _pollCount = 0;
    _pollStep = 1;
    return Status::Ok();
  }

  if (_pollStep == 1) {
    if (deadlineReached(_pollNowMs, _pollDeadlineMs)) {
      _cachedConfigDirty = true;
      return _recordFailure(
          Status::Error(Err::TIMEOUT, bootJob ? "Boot timeout" : "Reset timeout"));
    }
    if (_pollCount >= RESET_MAX_POLLS) {
      _cachedConfigDirty = true;
      return _recordFailure(Status::Error(
          Err::TIMEOUT, bootJob ? "Boot polling limit" : "Reset polling limit", RESET_MAX_POLLS));
    }

    uint8_t ctrl3Read = 0;
    _pollInstructionUsed = true;
    ++_pollCount;
    Status st = _readRegisterRaw(cmd::REG_CTRL3_C, ctrl3Read);
    if (!st.ok()) {
      return _recordFailure(st);
    }
    const uint8_t bit = bootJob ? cmd::MASK_BOOT : cmd::MASK_SW_RESET;
    if ((ctrl3Read & bit) == 0) {
      _pollStep = 2;
      _pollCount = 0;
    }
    return Status::Ok();
  }

  if (bootJob) {
    bool done = false;
    const uint8_t refreshStep = static_cast<uint8_t>(_pollStep - 2u);
    Status st = _pollRefreshStep(refreshStep, done);
    if (st.ok() && done) {
      return _finishPollJob(Status::Ok());
    }
    if (!st.ok()) {
      return st;
    }
    _pollStep = static_cast<uint8_t>(refreshStep + 3u);
    return Status::Ok();
  }

  bool done = false;
  const uint8_t applyStep = static_cast<uint8_t>(_pollStep - 2u);
  Status st = _pollApplyConfigStep(applyStep, done);
  if (st.ok() && done) {
    return _finishPollJob(Status::Ok());
  }
  if (!st.ok()) {
    return st;
  }
  _pollStep = static_cast<uint8_t>(applyStep + 3u);
  return Status::Ok();
}

Status LSM6DS3TR::_pollRefreshStep(uint8_t step, bool& done) {
  done = false;
  _pollInstructionUsed = true;
  Status st = Status::Ok();
  switch (step) {
    case 0:
      st = readRegister(cmd::REG_CTRL1_XL, _refreshCtrl1);
      break;
    case 1:
      st = readRegister(cmd::REG_CTRL2_G, _refreshCtrl2);
      break;
    case 2:
      st = readRegister(cmd::REG_CTRL3_C, _refreshCtrl3);
      break;
    case 3:
      st = readRegister(cmd::REG_CTRL4_C, _refreshCtrl4);
      break;
    case 4:
      st = readRegister(cmd::REG_CTRL6_C, _refreshCtrl6);
      break;
    case 5:
      st = readRegister(cmd::REG_CTRL7_G, _refreshCtrl7);
      break;
    case 6:
      st = readRegister(cmd::REG_CTRL8_XL, _refreshCtrl8);
      break;
    case 7:
      st = readRegister(cmd::REG_CTRL10_C, _refreshCtrl10);
      break;
    case 8:
      st = readRegister(cmd::REG_WAKE_UP_DUR, _refreshWakeUpDur);
      break;
    case 9:
      st = readRegs(cmd::REG_X_OFS_USR, _refreshOffsetData, sizeof(_refreshOffsetData));
      break;
    case 10:
      st = readRegs(cmd::REG_FIFO_CTRL1, _refreshFifoCtrl, sizeof(_refreshFifoCtrl));
      break;
    default:
      _pollInstructionUsed = false;
      done = true;
      return _commitStagedCachedConfig();
  }

  if (!st.ok()) {
    _cachedConfigDirty = true;
    done = true;
    return st;
  }
  _pollStep = static_cast<uint8_t>(step + 1u);
  if (_pollStep > 10u) {
    done = true;
    return _commitStagedCachedConfig();
  }
  return Status::Ok();
}

Status LSM6DS3TR::_commitStagedCachedConfig() {
  const Odr odrXl = static_cast<Odr>((_refreshCtrl1 & cmd::MASK_ODR_XL) >> cmd::BIT_ODR_XL);
  const AccelFs fsXl = static_cast<AccelFs>((_refreshCtrl1 & cmd::MASK_FS_XL) >>
                                            cmd::BIT_FS_XL);
  const Odr odrG = static_cast<Odr>((_refreshCtrl2 & cmd::MASK_ODR_G) >> cmd::BIT_ODR_G);
  const GyroFs fsG = ((_refreshCtrl2 & cmd::MASK_FS_125) != 0)
                         ? GyroFs::DPS_125
                         : static_cast<GyroFs>((_refreshCtrl2 & cmd::MASK_FS_G) >>
                                               cmd::BIT_FS_G);
  const AccelPowerMode accelPowerMode = (_refreshCtrl6 & cmd::MASK_XL_HM_MODE) != 0
                                            ? AccelPowerMode::LOW_POWER_NORMAL
                                            : AccelPowerMode::HIGH_PERFORMANCE;
  const GyroPowerMode gyroPowerMode = (_refreshCtrl7 & cmd::MASK_G_HM_MODE) != 0
                                          ? GyroPowerMode::LOW_POWER_NORMAL
                                          : GyroPowerMode::HIGH_PERFORMANCE;
  const Odr fifoOdr = static_cast<Odr>((_refreshFifoCtrl[4] & cmd::MASK_ODR_FIFO) >>
                                       cmd::BIT_ODR_FIFO);
  const FifoMode fifoMode = static_cast<FifoMode>(_refreshFifoCtrl[4] & cmd::MASK_FIFO_MODE);

  if (!isValidAccelFs(fsXl) || !isValidAccelOdr(odrXl, accelPowerMode) ||
      !isValidGyroFs(fsG) || !isValidGyroOdr(odrG, gyroPowerMode) ||
      !isValidFifoOdr(fifoOdr) || !isValidFifoMode(fifoMode)) {
    _cachedConfigDirty = true;
    return Status::Error(Err::INVALID_CONFIG, "Invalid cached register state");
  }

  _config.odrXl = odrXl;
  _config.fsXl = fsXl;
  _config.odrG = odrG;
  _config.fsG = fsG;
  _config.bdu = (_refreshCtrl3 & cmd::MASK_BDU) != 0;

  _gyroSleepEnabled = (_refreshCtrl4 & cmd::MASK_SLEEP_G) != 0;
  _gyroFilterConfig.lpf1Enabled = (_refreshCtrl4 & cmd::MASK_LPF1_SEL_G) != 0;

  _accelPowerMode = accelPowerMode;
  _accelOffsetWeight = (_refreshCtrl6 & cmd::MASK_USR_OFF_W) != 0
                           ? AccelOffsetWeight::MG_16
                           : AccelOffsetWeight::MG_1;

  _gyroPowerMode = gyroPowerMode;
  _gyroFilterConfig.highPassEnabled = (_refreshCtrl7 & cmd::MASK_HP_EN_G) != 0;
  _gyroFilterConfig.highPassMode =
      static_cast<GyroHpfMode>((_refreshCtrl7 & cmd::MASK_HPM_G) >> cmd::BIT_HPM_G);

  _accelFilterConfig.lpf2Enabled = (_refreshCtrl8 & cmd::MASK_LPF2_XL_EN) != 0;
  _accelFilterConfig.highPassSlopeEnabled = (_refreshCtrl8 & cmd::MASK_HP_SLOPE_XL_EN) != 0;
  _accelFilterConfig.lowPassOn6d = (_refreshCtrl8 & cmd::MASK_LOW_PASS_ON_6D) != 0;

  _timestampEnabled = (_refreshCtrl10 & (1u << cmd::BIT_TIMER_EN)) != 0;
  _pedometerEnabled = (_refreshCtrl10 & (1u << cmd::BIT_PEDO_EN)) != 0;
  _tiltEnabled = (_refreshCtrl10 & (1u << cmd::BIT_TILT_EN)) != 0;
  _wristTiltEnabled = (_refreshCtrl10 & (1u << cmd::BIT_WRIST_TILT_EN)) != 0;
  _significantMotionEnabled = (_refreshCtrl10 & (1u << cmd::BIT_SIGN_MOTION_EN)) != 0;
  _timestampHighResolution = (_refreshWakeUpDur & cmd::MASK_TIMER_HR) != 0;

  _accelUserOffset.x = static_cast<int8_t>(_refreshOffsetData[0]);
  _accelUserOffset.y = static_cast<int8_t>(_refreshOffsetData[1]);
  _accelUserOffset.z = static_cast<int8_t>(_refreshOffsetData[2]);

  _fifoConfig.threshold = static_cast<uint16_t>(_refreshFifoCtrl[0]) |
                          (static_cast<uint16_t>(_refreshFifoCtrl[1] &
                                                 cmd::MASK_FIFO_THRESHOLD_HI) << 8);
  _fifoConfig.storeTimestampStep =
      (_refreshFifoCtrl[1] & (1u << cmd::BIT_TIMER_PEDO_FIFO_EN)) != 0;
  _fifoConfig.storeTemperature = (_refreshFifoCtrl[1] & cmd::MASK_FIFO_TEMP_EN) != 0;
  _fifoConfig.gyroDecimation =
      static_cast<FifoDecimation>((_refreshFifoCtrl[2] & cmd::MASK_DEC_FIFO_GYRO) >>
                                  cmd::BIT_DEC_FIFO_GYRO);
  _fifoConfig.accelDecimation =
      static_cast<FifoDecimation>((_refreshFifoCtrl[2] & cmd::MASK_DEC_FIFO_XL) >>
                                  cmd::BIT_DEC_FIFO_XL);
  _fifoConfig.stopOnThreshold = (_refreshFifoCtrl[3] & cmd::MASK_STOP_ON_FTH) != 0;
  _fifoConfig.onlyHighData = (_refreshFifoCtrl[3] & cmd::MASK_ONLY_HIGH_DATA) != 0;
  _fifoConfig.odr = fifoOdr;
  _fifoConfig.mode = fifoMode;

  _cachedConfigDirty = false;
  return Status::Ok();
}

Status LSM6DS3TR::_pollFifoDrainStep() {
  if (_pollStep == 0) {
    uint8_t data[4] = {};
    _pollInstructionUsed = true;
    Status st = readRegs(cmd::REG_FIFO_STATUS1, data, sizeof(data));
    if (!st.ok()) {
      return st;
    }
    FifoStatus status;
    decodeFifoStatus(data, status);
    if (status.overrun) {
      return Status::Error(Err::FIFO_OVERRUN, "FIFO overrun");
    }
    if (status.empty || status.unreadWords == 0u) {
      _fifoDrainWordsAvailable = 0;
      return _finishPollJob(Status::Ok());
    }
    _fifoDrainWordsAvailable = status.unreadWords;
    _pollStep = 1;
    return Status::Ok();
  }

  if (_fifoDrainWordsRead >= _fifoDrainMaxWords ||
      _fifoDrainWordsRead >= _fifoDrainWordsAvailable) {
    return _finishPollJob(Status::Ok());
  }

  uint8_t data[2] = {};
  _pollInstructionUsed = true;
  Status st = readRegs(cmd::REG_FIFO_DATA_OUT_L, data, sizeof(data));
  if (!st.ok()) {
    return st;
  }
  ++_fifoDrainWordsRead;
  if (_fifoDrainWordsRead >= _fifoDrainMaxWords ||
      _fifoDrainWordsRead >= _fifoDrainWordsAvailable) {
    return _finishPollJob(Status::Ok());
  }
  return Status::Ok();
}

Status LSM6DS3TR::_pollCalibrationStep(bool accelJob) {
  const Odr odr = accelJob ? _config.odrXl : _config.odrG;
  if (_pollStep == 0) {
    if (!_pollDeadlineArmed) {
      _pollDeadlineMs = _pollNowMs + odrIntervalMs(odr) * 3 + 100;
      _pollDeadlineArmed = true;
    }
    bool ready = false;
    _pollInstructionUsed = true;
    Status st = accelJob ? isAccelDataReady(ready) : isGyroDataReady(ready);
    if (!st.ok()) {
      return st;
    }
    if (!ready) {
      ++_calibrationPollsForSample;
      if (deadlineReached(_pollNowMs, _pollDeadlineMs) ||
          _calibrationPollsForSample >= CALIBRATION_MAX_POLLS_PER_SAMPLE) {
        return Status::Error(Err::TIMEOUT,
                             accelJob ? "Accel data-ready timeout during calibration"
                                      : "Gyro data-ready timeout during calibration",
                             _calibrationPollsForSample);
      }
      return Status::Error(Err::IN_PROGRESS, "Calibration sample not ready");
    }
    _pollStep = 1;
    return Status::Ok();
  }

  RawAxes raw;
  _pollInstructionUsed = true;
  Status st = accelJob ? readAccelRaw(raw) : readGyroRaw(raw);
  if (!st.ok()) {
    return st;
  }
  const Axes phys = accelJob ? convertAccel(raw) : convertGyro(raw);
  addCalibrationSample(phys, _calibrationSamplesDone,
                       _calibrationSumX, _calibrationSumY, _calibrationSumZ,
                       _calibrationMinX, _calibrationMaxX,
                       _calibrationMinY, _calibrationMaxY,
                       _calibrationMinZ, _calibrationMaxZ);
  ++_calibrationSamplesDone;

  if (_calibrationSamplesDone >= _calibrationSamplesTarget) {
    Axes bias;
    Status quality = accelJob
        ? finalizeAccelCalibration(_calibrationSamplesTarget,
                                   _calibrationSumX, _calibrationSumY, _calibrationSumZ,
                                   _calibrationMinX, _calibrationMaxX,
                                   _calibrationMinY, _calibrationMaxY,
                                   _calibrationMinZ, _calibrationMaxZ,
                                   _config.calibrationLimits, bias)
        : finalizeGyroCalibration(_calibrationSamplesTarget,
                                  _calibrationSumX, _calibrationSumY, _calibrationSumZ,
                                  _calibrationMinX, _calibrationMaxX,
                                  _calibrationMinY, _calibrationMaxY,
                                  _calibrationMinZ, _calibrationMaxZ,
                                  _config.calibrationLimits, bias);
    if (!quality.ok()) {
      return quality;
    }
    if (accelJob) {
      _accelBias = bias;
    } else {
      _gyroBias = bias;
    }
    return _finishPollJob(Status::Ok());
  }

  _pollStep = 0;
  _calibrationPollsForSample = 0;
  _pollDeadlineMs = _pollNowMs + odrIntervalMs(odr) * 3 + 100;
  _pollDeadlineArmed = true;
  return Status::Ok();
}

Status LSM6DS3TR::getMeasurement(Measurement& out) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (!_hasSample) {
    return Status::Error(Err::MEASUREMENT_NOT_READY, "No sample available");
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
  _sampleTimestampMs = _nowMs();
  return Status::Ok();
}

Status LSM6DS3TR::getSettings(SettingsSnapshot& out) const {
  out.initialized = _initialized;
  out.state = _driverState;
  out.i2cAddress = _config.i2cAddress;
  out.i2cTimeoutMs = _config.i2cTimeoutMs;
  out.offlineThreshold = _config.offlineThreshold;
  out.hasNowMsHook = (_config.nowMs != nullptr);
  out.odrXl = _config.odrXl;
  out.odrG = _config.odrG;
  out.fsXl = _config.fsXl;
  out.fsG = _config.fsG;
  out.bdu = _config.bdu;
  out.accelPowerMode = _accelPowerMode;
  out.gyroPowerMode = _gyroPowerMode;
  out.gyroSleepEnabled = _gyroSleepEnabled;
  out.accelFilter = _accelFilterConfig;
  out.gyroFilter = _gyroFilterConfig;
  out.timestampEnabled = _timestampEnabled;
  out.timestampHighResolution = _timestampHighResolution;
  out.pedometerEnabled = _pedometerEnabled;
  out.significantMotionEnabled = _significantMotionEnabled;
  out.tiltEnabled = _tiltEnabled;
  out.wristTiltEnabled = _wristTiltEnabled;
  out.accelOffsetWeight = _accelOffsetWeight;
  out.accelUserOffset = _accelUserOffset;
  out.fifo = _fifoConfig;
  out.accelBias = _accelBias;
  out.gyroBias = _gyroBias;
  out.measurementPending = _measurementRequested && !_measurementReady;
  out.measurementReady = _measurementReady;
  out.hasSample = _hasSample;
  out.sampleTimestampMs = _sampleTimestampMs;
  out.rawMeasurement = _rawMeasurement;
  out.cachedConfigDirty = _cachedConfigDirty;
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
    return Status::Error(Err::INVALID_PARAM,
                         "Accel 1.6 Hz requires low-power/normal mode; run 'apm lpn' first");
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
    _cachedConfigDirty = true;
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
    _cachedConfigDirty = true;
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
    _cachedConfigDirty = true;
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
    _cachedConfigDirty = true;
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

Status LSM6DS3TR::readStatus(StatusReg& out) {
  uint8_t raw = 0;
  Status st = readStatusReg(raw);
  if (!st.ok()) {
    return st;
  }

  out.raw = raw;
  out.accelDataReady = (raw & cmd::MASK_XLDA) != 0;
  out.gyroDataReady = (raw & cmd::MASK_GDA) != 0;
  out.tempDataReady = (raw & cmd::MASK_TDA) != 0;
  return Status::Ok();
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
    return Status::Error(Err::INVALID_PARAM,
                         "Current accel ODR is not valid in requested power mode; use 12.5 Hz+ for high-performance");
  }

  const AccelPowerMode oldMode = _accelPowerMode;
  _accelPowerMode = mode;
  Status st = writeRegister(cmd::REG_CTRL6_C, _buildCtrl6C());
  if (!st.ok()) {
    _accelPowerMode = oldMode;
    _cachedConfigDirty = true;
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
    _cachedConfigDirty = true;
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
    _cachedConfigDirty = true;
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
    _cachedConfigDirty = true;
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
    _cachedConfigDirty = true;
    return st;
  }
  st = writeRegister(cmd::REG_CTRL7_G, _buildCtrl7G());
  if (!st.ok()) {
    _gyroFilterConfig = oldConfig;
    _cachedConfigDirty = true;
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
    _cachedConfigDirty = true;
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
    _cachedConfigDirty = true;
    return st;
  }
  if (enabled) {
    st = resetTimestamp();
    if (!st.ok()) {
      _timestampHighResolution = oldValue;
      _cachedConfigDirty = true;
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
    _cachedConfigDirty = true;
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
    _cachedConfigDirty = true;
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
    _cachedConfigDirty = true;
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
    _cachedConfigDirty = true;
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
    _cachedConfigDirty = true;
    return st;
  }
  st = writeRegister(cmd::REG_CTRL10_C, _buildCtrl10C());
  if (!st.ok()) {
    _cachedConfigDirty = true;
  }
  return st;
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
    _cachedConfigDirty = true;
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
    _cachedConfigDirty = true;
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
    _cachedConfigDirty = true;
    return st;
  }
  st = writeRegister(cmd::REG_FIFO_CTRL2, _buildFifoCtrl2());
  if (!st.ok()) {
    _fifoConfig = oldConfig;
    _cachedConfigDirty = true;
    return st;
  }
  st = writeRegister(cmd::REG_FIFO_CTRL3, _buildFifoCtrl3());
  if (!st.ok()) {
    _fifoConfig = oldConfig;
    _cachedConfigDirty = true;
    return st;
  }
  st = writeRegister(cmd::REG_FIFO_CTRL4, _buildFifoCtrl4());
  if (!st.ok()) {
    _fifoConfig = oldConfig;
    _cachedConfigDirty = true;
    return st;
  }
  st = writeRegister(cmd::REG_FIFO_CTRL5, _buildFifoCtrl5());
  if (!st.ok()) {
    _fifoConfig = oldConfig;
    _cachedConfigDirty = true;
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

  decodeFifoStatus(data, out);
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
  if (status.empty || status.unreadWords == 0u) {
    return Status::Error(Err::FIFO_EMPTY, "FIFO empty");
  }
  if (status.overrun) {
    return Status::Error(Err::FIFO_OVERRUN, "FIFO overrun");
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
    _sampleTimestampMs = 0;

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
        _cachedConfigDirty = true;
        return _recordFailure(Status::Error(Err::TIMEOUT, "Reset timeout"));
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
      _cachedConfigDirty = true;
      return _recordFailure(Status::Error(Err::TIMEOUT, "Reset polling limit", RESET_MAX_POLLS));
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
        _cachedConfigDirty = true;
        return _recordFailure(Status::Error(Err::TIMEOUT, "Boot timeout"));
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
      _cachedConfigDirty = true;
      return _recordFailure(Status::Error(Err::TIMEOUT, "Boot polling limit", RESET_MAX_POLLS));
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
  if (!isSafePublicRegisterWrite(reg, value)) {
    return Status::Error(Err::INVALID_PARAM, "Unsafe direct register write");
  }
  Status st = writeRegister(reg, value);
  if (!st.ok()) {
    if (isCachedConfigRegister(reg)) {
      _cachedConfigDirty = true;
    }
    return st;
  }

  if (isCachedConfigRegister(reg)) {
    return refreshCachedConfig();
  }
  return Status::Ok();
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

  Status st = readRegister(cmd::REG_CTRL1_XL, _refreshCtrl1);
  if (!st.ok()) {
    _cachedConfigDirty = true;
    return st;
  }
  st = readRegister(cmd::REG_CTRL2_G, _refreshCtrl2);
  if (!st.ok()) {
    _cachedConfigDirty = true;
    return st;
  }
  st = readRegister(cmd::REG_CTRL3_C, _refreshCtrl3);
  if (!st.ok()) {
    _cachedConfigDirty = true;
    return st;
  }
  st = readRegister(cmd::REG_CTRL4_C, _refreshCtrl4);
  if (!st.ok()) {
    _cachedConfigDirty = true;
    return st;
  }
  st = readRegister(cmd::REG_CTRL6_C, _refreshCtrl6);
  if (!st.ok()) {
    _cachedConfigDirty = true;
    return st;
  }
  st = readRegister(cmd::REG_CTRL7_G, _refreshCtrl7);
  if (!st.ok()) {
    _cachedConfigDirty = true;
    return st;
  }
  st = readRegister(cmd::REG_CTRL8_XL, _refreshCtrl8);
  if (!st.ok()) {
    _cachedConfigDirty = true;
    return st;
  }
  st = readRegister(cmd::REG_CTRL10_C, _refreshCtrl10);
  if (!st.ok()) {
    _cachedConfigDirty = true;
    return st;
  }
  st = readRegister(cmd::REG_WAKE_UP_DUR, _refreshWakeUpDur);
  if (!st.ok()) {
    _cachedConfigDirty = true;
    return st;
  }
  st = readRegs(cmd::REG_X_OFS_USR, _refreshOffsetData, sizeof(_refreshOffsetData));
  if (!st.ok()) {
    _cachedConfigDirty = true;
    return st;
  }
  st = readRegs(cmd::REG_FIFO_CTRL1, _refreshFifoCtrl, sizeof(_refreshFifoCtrl));
  if (!st.ok()) {
    _cachedConfigDirty = true;
    return st;
  }

  return _commitStagedCachedConfig();
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

Status LSM6DS3TR::readSensorHub(SensorHubData& out, uint8_t count) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "begin() not called");
  }
  if (count == 0u || count > 12u) {
    return Status::Error(Err::INVALID_PARAM, "Sensor hub count must be 1..12");
  }

  out = SensorHubData{};
  Status st = readRegs(cmd::REG_SENSORHUB1, out.bytes, count);
  if (!st.ok()) {
    return st;
  }
  out.count = count;
  return Status::Ok();
}

Status LSM6DS3TR::_i2cWriteReadRaw(const uint8_t* txBuf, size_t txLen,
                                   uint8_t* rxBuf, size_t rxLen) {
  if (txBuf == nullptr || txLen == 0 || (rxLen > 0 && rxBuf == nullptr)) {
    return Status::Error(Err::INVALID_PARAM, "Invalid I2C buffer");
  }
  if (_config.i2cWriteRead == nullptr) {
    return Status::Error(Err::INVALID_CONFIG, "I2C write-read not set");
  }
  return normalizeTransportStatus(
      _config.i2cWriteRead(_config.i2cAddress, txBuf, txLen, rxBuf, rxLen,
                           _config.i2cTimeoutMs, _config.i2cUser));
}

Status LSM6DS3TR::_i2cWriteRaw(const uint8_t* buf, size_t len) {
  if (buf == nullptr || len == 0) {
    return Status::Error(Err::INVALID_PARAM, "Invalid I2C buffer");
  }
  if (_config.i2cWrite == nullptr) {
    return Status::Error(Err::INVALID_CONFIG, "I2C write not set");
  }
  return normalizeTransportStatus(
      _config.i2cWrite(_config.i2cAddress, buf, len, _config.i2cTimeoutMs,
                       _config.i2cUser));
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
  if (!_initialized || st.inProgress()) {
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
  const auto failDirty = [this](Status st) -> Status {
    if (_initialized && !st.ok()) {
      _cachedConfigDirty = true;
    }
    return st;
  };

  Status st = writeRegister(
      cmd::REG_CTRL3_C,
      static_cast<uint8_t>(cmd::MASK_IF_INC | (_config.bdu ? cmd::MASK_BDU : 0)));
  if (!st.ok()) return failDirty(st);

  st = writeRegister(cmd::REG_CTRL4_C, _buildCtrl4C());
  if (!st.ok()) return failDirty(st);
  st = writeRegister(cmd::REG_CTRL6_C, _buildCtrl6C());
  if (!st.ok()) return failDirty(st);
  st = writeRegister(cmd::REG_CTRL7_G, _buildCtrl7G());
  if (!st.ok()) return failDirty(st);
  st = writeRegister(cmd::REG_CTRL8_XL, _buildCtrl8Xl());
  if (!st.ok()) return failDirty(st);
  st = writeRegister(cmd::REG_CTRL1_XL, _buildCtrl1Xl(_config.odrXl, _config.fsXl));
  if (!st.ok()) return failDirty(st);
  st = writeRegister(cmd::REG_CTRL2_G, _buildCtrl2G(_config.odrG, _config.fsG));
  if (!st.ok()) return failDirty(st);
  st = _updateRegister(cmd::REG_WAKE_UP_DUR, cmd::MASK_TIMER_HR, _buildWakeUpDur());
  if (!st.ok()) return failDirty(st);
  st = writeRegister(cmd::REG_CTRL10_C, _buildCtrl10C());
  if (!st.ok()) return failDirty(st);

  const uint8_t offsets[3] = {
      static_cast<uint8_t>(_accelUserOffset.x),
      static_cast<uint8_t>(_accelUserOffset.y),
      static_cast<uint8_t>(_accelUserOffset.z),
  };
  st = writeRegs(cmd::REG_X_OFS_USR, offsets, sizeof(offsets));
  if (!st.ok()) return failDirty(st);

  st = writeRegister(cmd::REG_FIFO_CTRL1, loByte(_fifoConfig.threshold));
  if (!st.ok()) return failDirty(st);
  st = writeRegister(cmd::REG_FIFO_CTRL2, _buildFifoCtrl2());
  if (!st.ok()) return failDirty(st);
  st = writeRegister(cmd::REG_FIFO_CTRL3, _buildFifoCtrl3());
  if (!st.ok()) return failDirty(st);
  st = writeRegister(cmd::REG_FIFO_CTRL4, _buildFifoCtrl4());
  if (!st.ok()) return failDirty(st);
  st = writeRegister(cmd::REG_FIFO_CTRL5, _buildFifoCtrl5());
  if (!st.ok()) return failDirty(st);

  _cachedConfigDirty = false;
  return Status::Ok();
}

Status LSM6DS3TR::_readRawAllWithTimestamp(uint32_t sampleTimestampMs) {
  uint8_t data[cmd::DATA_LEN_ALL] = {};
  Status st = readRegs(cmd::REG_DATA_START_ALL, data, sizeof(data));
  if (!st.ok()) {
    return st;
  }

  decodeRawAll(data, _rawMeasurement);
  _hasSample = true;
  _sampleTimestampMs = sampleTimestampMs;
  return Status::Ok();
}

Status LSM6DS3TR::_readRawAll() {
  return _readRawAllWithTimestamp(_nowMs());
}

Status LSM6DS3TR::_settleSelfTest(uint32_t settleMs) {
  if (settleMs == 0u) {
    return Status::Ok();
  }

  const uint32_t deadline = _nowMs() + settleMs;
  const uint32_t maxPolls =
      settleMs * SELF_TEST_SETTLE_POLLS_PER_MS + SELF_TEST_SETTLE_POLLS_PER_MS;
  for (uint32_t poll = 0; poll < maxPolls; ++poll) {
    uint8_t ignored = 0;
    Status st = readStatusReg(ignored);
    if (!st.ok()) {
      return st;
    }
    if (_config.nowMs == nullptr || deadlineReached(_nowMs(), deadline)) {
      return Status::Ok();
    }
  }
  return Status::Error(Err::TIMEOUT, "Self-test settle timeout",
                       static_cast<int32_t>(settleMs));
}

Status LSM6DS3TR::_waitForSelfTestReady(bool accel) {
  const uint32_t timeoutMs = odrIntervalMs(Odr::HZ_416) * 3U + 100U;
  const uint32_t deadline = _nowMs() + timeoutMs;
  for (uint16_t poll = 0; poll < SELF_TEST_READY_MAX_POLLS; ++poll) {
    bool ready = false;
    Status st = accel ? isAccelDataReady(ready) : isGyroDataReady(ready);
    if (!st.ok()) {
      return st;
    }
    if (ready) {
      return Status::Ok();
    }
    if (deadlineReached(_nowMs(), deadline)) {
      return Status::Error(Err::TIMEOUT,
                           accel ? "Accel self-test data-ready timeout"
                                 : "Gyro self-test data-ready timeout",
                           poll);
    }
  }
  return Status::Error(Err::TIMEOUT,
                       accel ? "Accel self-test polling limit"
                             : "Gyro self-test polling limit",
                       SELF_TEST_READY_MAX_POLLS);
}

Status LSM6DS3TR::_readSelfTestAverage(bool accel, uint16_t samples, RawAxes& out) {
  int32_t sumX = 0;
  int32_t sumY = 0;
  int32_t sumZ = 0;
  for (uint16_t i = 0; i < samples; ++i) {
    Status st = _waitForSelfTestReady(accel);
    if (!st.ok()) {
      return st;
    }
    RawAxes raw;
    st = accel ? readAccelRaw(raw) : readGyroRaw(raw);
    if (!st.ok()) {
      return st;
    }
    sumX += raw.x;
    sumY += raw.y;
    sumZ += raw.z;
  }
  out.x = static_cast<int16_t>(sumX / static_cast<int32_t>(samples));
  out.y = static_cast<int16_t>(sumY / static_cast<int32_t>(samples));
  out.z = static_cast<int16_t>(sumZ / static_cast<int32_t>(samples));
  return Status::Ok();
}

uint32_t LSM6DS3TR::_nowMs() const {
  if (_config.nowMs != nullptr) {
    return _config.nowMs(_config.timeUser);
  }
  return 0;
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
  float minX = 0.0f;
  float maxX = 0.0f;
  float minY = 0.0f;
  float maxY = 0.0f;
  float minZ = 0.0f;
  float maxZ = 0.0f;

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
    addCalibrationSample(phys, i, sumX, sumY, sumZ,
                         minX, maxX, minY, maxY, minZ, maxZ);
  }

  Status st = finalizeAccelCalibration(samples, sumX, sumY, sumZ,
                                       minX, maxX, minY, maxY, minZ, maxZ,
                                       _config.calibrationLimits, out);
  if (!st.ok()) {
    return st;
  }
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
  float minX = 0.0f;
  float maxX = 0.0f;
  float minY = 0.0f;
  float maxY = 0.0f;
  float minZ = 0.0f;
  float maxZ = 0.0f;

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
    addCalibrationSample(phys, i, sumX, sumY, sumZ,
                         minX, maxX, minY, maxY, minZ, maxZ);
  }

  Status st = finalizeGyroCalibration(samples, sumX, sumY, sumZ,
                                      minX, maxX, minY, maxY, minZ, maxZ,
                                      _config.calibrationLimits, out);
  if (!st.ok()) {
    return st;
  }
  _gyroBias = out;
  return Status::Ok();
}

}  // namespace LSM6DS3TR
