#include "LSM6DS3TR/LSM6DS3TR.h"

#include <cmath>
#include <cstdint>
#include <limits>

namespace LSM6DS3TR {
namespace {

constexpr uint8_t SELF_TEST_DISCARD_SAMPLES = 1;
constexpr uint16_t SELF_TEST_MIN_SAMPLES = 5;
constexpr uint16_t SELF_TEST_MAX_SAMPLES = 100;
constexpr uint16_t CALIBRATION_MIN_SAMPLES = 1;
constexpr uint16_t CALIBRATION_MAX_SAMPLES = 1000;
constexpr uint16_t FIFO_CAPACITY_WORDS = 2048;
constexpr uint64_t SELF_TEST_ACCEL_SETTLE_MS = 100;
constexpr uint64_t SELF_TEST_GYRO_SETTLE_MS = 150;
constexpr uint64_t SELF_TEST_GYRO_STIMULUS_SETTLE_MS = 50;
constexpr uint64_t CLOCK_QUANTIZATION_MARGIN_MS = 1;
constexpr uint32_t SELF_TEST_FIXED_TRANSACTIONS = 106;
// Datasheet pp.97/103/104: fixed bank-A pedometer policy, modified only in
// proved sensor power-down. No sensor-hub or general embedded-bank API.
constexpr uint8_t PEDOMETER_REGISTERS[] = {0x0F, 0x14, 0x15};
constexpr uint8_t PEDOMETER_VALUES[] = {0x10, 0x6E, 0x00};
constexpr uint8_t COUNTER_ENABLE = 0x34;  // FUNC_EN | PEDO_EN | TIMER_EN.
constexpr uint8_t PEDOMETER_RESET = 0x02;
constexpr float SELF_TEST_ACCEL_MIN_G = 0.090f;
constexpr float SELF_TEST_ACCEL_MAX_G = 1.700f;
constexpr float SELF_TEST_GYRO_MIN_DPS = 150.0f;
constexpr float SELF_TEST_GYRO_MAX_DPS = 700.0f;
constexpr float MAX_ABS_BIAS = 10000.0f;

FifoStatusSnapshot decodeFifoStatus(const uint8_t* data, uint64_t nowMs) {
  FifoStatusSnapshot result{};
  result.readUptimeMs = nowMs;
  result.valid = true;
  result.unreadWords = static_cast<uint16_t>(data[0]) |
      (static_cast<uint16_t>(data[1] & cmd::MASK_DIFF_FIFO_HI) << 8U);
  result.pattern = static_cast<uint16_t>(data[2]) |
      (static_cast<uint16_t>(data[3] & 0x03U) << 8U);
  result.empty = (data[1] & cmd::MASK_FIFO_EMPTY) != 0U;
  result.overrun = (data[1] & cmd::MASK_FIFO_OVER_RUN) != 0U;
  result.full = (data[1] & cmd::MASK_FIFO_FULL_SMART) != 0U;
  result.watermark = (data[1] & cmd::MASK_FIFO_WATERM) != 0U;
  // Repository ambiguity policy: OVER_RUN/count-zero means 2048, never empty.
  // EMPTY disagreement is checked by every consumer before a destructive read.
  if (result.overrun && result.unreadWords == 0U)
    result.unreadWords = FIFO_CAPACITY_WORDS;
  return result;
}

bool consistentFifoStatus(const FifoStatusSnapshot& snapshot) {
  return snapshot.empty == (snapshot.unreadWords == 0U);
}

uint32_t saturatingIncrement(uint32_t value) {
  return value == std::numeric_limits<uint32_t>::max() ? value : value + 1U;
}

uint64_t saturatingIncrement64(uint64_t value) {
  return value == std::numeric_limits<uint64_t>::max() ? value : value + 1U;
}

uint64_t saturatingAdd(uint64_t lhs, uint64_t rhs) {
  const uint64_t maximum = std::numeric_limits<uint64_t>::max();
  return rhs > maximum - lhs ? maximum : lhs + rhs;
}

uint64_t saturatingMultiply(uint64_t lhs, uint64_t rhs) {
  if (lhs == 0U || rhs == 0U) return 0U;
  const uint64_t maximum = std::numeric_limits<uint64_t>::max();
  return lhs > maximum / rhs ? maximum : lhs * rhs;
}

bool finiteAxes(const Axes& axes) {
  return std::isfinite(axes.x) && std::isfinite(axes.y) && std::isfinite(axes.z);
}

bool boundedAxes(const Axes& axes, float maximum) {
  return finiteAxes(axes) && std::fabs(axes.x) <= maximum &&
         std::fabs(axes.y) <= maximum && std::fabs(axes.z) <= maximum;
}

float axesMagnitudeSquared(const Axes& axes) {
  return axes.x * axes.x + axes.y * axes.y + axes.z * axes.z;
}

bool validOdrValue(Odr odr) {
  return static_cast<uint8_t>(odr) <= static_cast<uint8_t>(Odr::HZ_1_6);
}

bool validAccelFs(AccelFs fullScale) {
  switch (fullScale) {
    case AccelFs::G_2:
    case AccelFs::G_4:
    case AccelFs::G_8:
    case AccelFs::G_16:
      return true;
    default:
      return false;
  }
}

bool validGyroFs(GyroFs fullScale) {
  switch (fullScale) {
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

bool validAccelPower(AccelPowerMode mode) {
  return mode == AccelPowerMode::HIGH_PERFORMANCE ||
         mode == AccelPowerMode::LOW_POWER_NORMAL;
}

bool validGyroPower(GyroPowerMode mode) {
  return mode == GyroPowerMode::HIGH_PERFORMANCE ||
         mode == GyroPowerMode::LOW_POWER_NORMAL;
}

bool accelOdrAllowed(Odr odr, AccelPowerMode mode) {
  if (!validOdrValue(odr) || !validAccelPower(mode)) {
    return false;
  }
  if (odr == Odr::HZ_1_6) {
    return mode == AccelPowerMode::LOW_POWER_NORMAL;
  }
  if (mode == AccelPowerMode::LOW_POWER_NORMAL) {
    return static_cast<uint8_t>(odr) <= static_cast<uint8_t>(Odr::HZ_208);
  }
  return true;
}

bool gyroOdrAllowed(Odr odr, GyroPowerMode mode) {
  if (!validOdrValue(odr) || !validGyroPower(mode) || odr == Odr::HZ_1_6) {
    return false;
  }
  if (mode == GyroPowerMode::LOW_POWER_NORMAL) {
    return static_cast<uint8_t>(odr) <= static_cast<uint8_t>(Odr::HZ_208);
  }
  return true;
}

bool validHpf(GyroHpfMode mode) {
  return static_cast<uint8_t>(mode) <= static_cast<uint8_t>(GyroHpfMode::HZ_1_040);
}

bool validSampleQuality(SampleQuality quality) {
  switch (quality) {
    case SampleQuality::READY_CHECKED:
    case SampleQuality::DIRECT_UNVERIFIED:
    case SampleQuality::CONFIG_UNKNOWN:
    case SampleQuality::SETTLING:
      return true;
    default:
      return false;
  }
}

uint16_t gyroSettleSamples(const DeviceProfile& profile) {
  // buildCtrl6() never sets CTRL6_C.FTYPE, so this device runs FTYPE=00 only.
  // LPF1 off -> AN5130 Table 15; LPF1 on -> the FTYPE=00 column of AN5130
  // Table 16. Not a generic LPF1 bound: FTYPE 01/10 raise 208/416/833 Hz.
  switch (profile.gyroOdr) {
    case Odr::HZ_12_5: return 2;
    case Odr::HZ_26: return 3;
    case Odr::HZ_52: return 3;
    case Odr::HZ_104: return profile.gyroFilter.lpf1Enabled ? 4 : 3;
    case Odr::HZ_208: return profile.gyroFilter.lpf1Enabled ? 4 : 3;
    case Odr::HZ_416: return profile.gyroFilter.lpf1Enabled ? 5 : 3;
    case Odr::HZ_833: return profile.gyroFilter.lpf1Enabled ? 7 : 3;
    case Odr::HZ_1660: return 135;
    case Odr::HZ_3330: return 270;
    case Odr::HZ_6660: return 540;
    case Odr::POWER_DOWN:
    case Odr::HZ_1_6:
    default: return 0;
  }
}

uint64_t sampleReadyPollDelayMs(const DeviceProfile& profile,
                                const SampleRequest& request) {
  uint64_t slowestPeriodUs = 0;
  auto include = [&slowestPeriodUs](Odr odr) {
    const uint64_t period = odrPeriodUs(odr);
    if (period > slowestPeriodUs) slowestPeriodUs = period;
  };
  if ((request.quantityMask & SAMPLE_ACCELERATION) != 0U) {
    include(profile.accelOdr);
  }
  if ((request.quantityMask & SAMPLE_ANGULAR_RATE) != 0U) {
    include(profile.gyroOdr);
  }
  if ((request.quantityMask & SAMPLE_TEMPERATURE) != 0U) {
    uint64_t temperaturePeriodUs = odrPeriodUs(Odr::HZ_52);
    if (profile.gyroOdr == Odr::POWER_DOWN &&
        profile.accelPowerMode == AccelPowerMode::LOW_POWER_NORMAL) {
      if (profile.accelOdr == Odr::HZ_12_5) {
        temperaturePeriodUs = odrPeriodUs(Odr::HZ_12_5);
      } else if (profile.accelOdr == Odr::HZ_26) {
        temperaturePeriodUs = odrPeriodUs(Odr::HZ_26);
      }
    }
    if (temperaturePeriodUs > slowestPeriodUs) {
      slowestPeriodUs = temperaturePeriodUs;
    }
  }
  const uint64_t roundedMs = (slowestPeriodUs + 999U) / 1000U;
  return roundedMs == 0U ? 1U : roundedMs;
}

bool diagnosticWritableMask(uint8_t reg, uint8_t& mask) {
  switch (reg) {
    case cmd::REG_FUNC_CFG_ACCESS: mask = 0xA0; return true;
    case cmd::REG_SENSOR_SYNC_TIME_FRAME: mask = 0x0F; return true;
    case cmd::REG_SENSOR_SYNC_RES_RATIO: mask = 0x03; return true;
    case cmd::REG_FIFO_CTRL1: mask = 0xFF; return true;
    case cmd::REG_FIFO_CTRL2: mask = 0xCF; return true;
    case cmd::REG_FIFO_CTRL3: mask = 0x3F; return true;
    case cmd::REG_FIFO_CTRL4: mask = 0xFF; return true;
    case cmd::REG_FIFO_CTRL5: mask = 0x7F; return true;
    case cmd::REG_DRDY_PULSE_CFG_G:
      // Datasheet Table 43: only DRDY_PULSED (bit 7) and INT2_WRIST_TILT
      // (bit 0) are writable; bits 6:1 must stay zero.
      mask = static_cast<uint8_t>(cmd::MASK_DRDY_PULSED | cmd::MASK_INT2_WRIST_TILT);
      return true;
    case cmd::REG_INT1_CTRL:
    case cmd::REG_INT2_CTRL:
    case cmd::REG_CTRL1_XL:
    case cmd::REG_CTRL3_C:
    case cmd::REG_CTRL5_C:
    case cmd::REG_TAP_CFG:
    case cmd::REG_TAP_THS_6D:
    case cmd::REG_INT_DUR2:
    case cmd::REG_WAKE_UP_DUR:
    case cmd::REG_FREE_FALL:
    case cmd::REG_MD1_CFG:
    case cmd::REG_MD2_CFG:
      mask = 0xFF;
      return true;
    case cmd::REG_WAKE_UP_THS: mask = 0xBF; return true;
    case cmd::REG_CTRL2_G:
    case cmd::REG_CTRL4_C: mask = 0xFE; return true;
    case cmd::REG_CTRL6_C: mask = 0xFB; return true;
    case cmd::REG_CTRL7_G: mask = 0xF8; return true;
    case cmd::REG_CTRL8_XL: mask = 0xFD; return true;
    case cmd::REG_CTRL9_XL: mask = 0xF4; return true;
    case cmd::REG_CTRL10_C: mask = 0xBF; return true;
    case cmd::REG_MASTER_CONFIG: mask = 0xDF; return true;
    case cmd::REG_X_OFS_USR:
    case cmd::REG_Y_OFS_USR:
    case cmd::REG_Z_OFS_USR:
      mask = 0xFF;
      return true;
    default:
      mask = 0;
      return false;
  }
}

bool validOffsetWeight(AccelOffsetWeight weight) {
  return weight == AccelOffsetWeight::MG_1 || weight == AccelOffsetWeight::MG_16;
}

uint8_t fifoDecimationFactor(FifoDecimation decimation) {
  static constexpr uint8_t FACTORS[] = {0, 1, 2, 3, 4, 8, 16, 32};
  const uint8_t index = static_cast<uint8_t>(decimation);
  return index < sizeof(FACTORS) ? FACTORS[index] : 0;
}

bool fifoCollecting(const DeviceProfile& profile) {
  return profile.fifo.enabled && profile.fifo.mode != FifoMode::BYPASS;
}

bool counterProfile(const DeviceProfile& profile) {
  return profile.fifo.enabled &&
         profile.fifo.fourthDataset == FifoFourthDataset::TIMESTAMP_STEP;
}

uint8_t buildCounterControl(const DeviceProfile& profile) {
  return counterProfile(profile) ? COUNTER_ENABLE : 0U;
}

uint8_t buildTimerResolution(const DeviceProfile& profile) {
  return counterProfile(profile) &&
                 profile.fifo.timestampResolution == TimestampResolution::US_25
             ? cmd::MASK_TIMER_HR : 0U;
}

uint8_t buildFifoCtrl2(const DeviceProfile& profile) {
  return static_cast<uint8_t>(((profile.fifo.watermarkWords >> 8U) & 7U) |
      (profile.fifo.fourthDataset == FifoFourthDataset::TEMPERATURE ? 0x08U : 0U) |
      (counterProfile(profile) ? 0x80U : 0U) |
      (profile.fifo.trigger == FifoTrigger::STEP_EVENT ? 0x40U : 0U));
}

uint16_t fifoOverwriteFloor(const DeviceProfile& profile) {
  const uint16_t datasets =
      (profile.fifo.gyroDecimation != FifoDecimation::OFF ? 1U : 0U) +
      (profile.fifo.accelDecimation != FifoDecimation::OFF ? 1U : 0U) +
      (profile.fifo.fourthDataset != FifoFourthDataset::OFF ? 1U : 0U);
  return static_cast<uint16_t>(FIFO_CAPACITY_WORDS - 3U * datasets);
}

uint8_t buildFifoCtrl5(const DeviceProfile& profile) {
  return profile.fifo.enabled
             ? static_cast<uint8_t>((static_cast<uint8_t>(profile.fifo.odr)
                                     << cmd::BIT_ODR_FIFO) |
                                    static_cast<uint8_t>(profile.fifo.mode))
             : 0U;
}

uint8_t buildCtrl1(const DeviceProfile& profile) {
  return static_cast<uint8_t>((static_cast<uint8_t>(profile.accelOdr) << cmd::BIT_ODR_XL) |
                              (static_cast<uint8_t>(profile.accelFullScale) << cmd::BIT_FS_XL));
}

uint8_t buildCtrl2(const DeviceProfile& profile) {
  uint8_t value = static_cast<uint8_t>(static_cast<uint8_t>(profile.gyroOdr)
                                       << cmd::BIT_ODR_G);
  if (profile.gyroFullScale == GyroFs::DPS_125) {
    value = static_cast<uint8_t>(value | cmd::MASK_FS_125);
  } else {
    value = static_cast<uint8_t>(value |
                                 (static_cast<uint8_t>(profile.gyroFullScale)
                                  << cmd::BIT_FS_G));
  }
  return value;
}

uint8_t buildCtrl3(const DeviceProfile& profile) {
  return static_cast<uint8_t>(cmd::MASK_IF_INC |
                              (profile.blockDataUpdate ? cmd::MASK_BDU : 0U));
}

uint8_t buildCtrl4(const DeviceProfile& profile) {
  return static_cast<uint8_t>((profile.gyroSleepEnabled ? cmd::MASK_SLEEP_G : 0U) |
                              (profile.gyroFilter.lpf1Enabled
                                   ? cmd::MASK_LPF1_SEL_G
                                   : 0U));
}

uint8_t buildCtrl6(const DeviceProfile& profile) {
  return static_cast<uint8_t>(
      (profile.accelPowerMode == AccelPowerMode::LOW_POWER_NORMAL
           ? cmd::MASK_XL_HM_MODE
           : 0U) |
      (profile.accelOffsetWeight == AccelOffsetWeight::MG_16
           ? cmd::MASK_USR_OFF_W
           : 0U));
}

uint8_t buildCtrl7(const DeviceProfile& profile) {
  return static_cast<uint8_t>(
      (profile.gyroPowerMode == GyroPowerMode::LOW_POWER_NORMAL
           ? cmd::MASK_G_HM_MODE
           : 0U) |
      (profile.gyroFilter.highPassEnabled ? cmd::MASK_HP_EN_G : 0U) |
      (static_cast<uint8_t>(profile.gyroFilter.highPassMode) << cmd::BIT_HPM_G));
}

uint8_t buildCtrl8(const DeviceProfile& profile) {
  return static_cast<uint8_t>(
      (profile.accelFilter.lpf2Enabled ? cmd::MASK_LPF2_XL_EN : 0U) |
      (profile.accelFilter.highPassSlopeEnabled ? cmd::MASK_HP_SLOPE_XL_EN : 0U) |
      (profile.accelFilter.lowPassOn6d ? cmd::MASK_LOW_PASS_ON_6D : 0U));
}

int16_t decodeInt16(const uint8_t* data) {
  return static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
                              (static_cast<uint16_t>(data[1]) << 8U));
}

RawAxes decodeRawAxes(const uint8_t* data) {
  return RawAxes{decodeInt16(&data[0]), decodeInt16(&data[2]),
                 decodeInt16(&data[4])};
}

Axes rawAxesToFloat(const RawAxes& raw, float sensitivity) {
  return Axes{static_cast<float>(raw.x) * sensitivity,
              static_cast<float>(raw.y) * sensitivity,
              static_cast<float>(raw.z) * sensitivity};
}

Axes meanRawAxesToFloat(int64_t sumX, int64_t sumY, int64_t sumZ,
                        uint16_t count, float sensitivity) {
  const float meanScale = sensitivity / static_cast<float>(count);
  return Axes{static_cast<float>(sumX) * meanScale,
              static_cast<float>(sumY) * meanScale,
              static_cast<float>(sumZ) * meanScale};
}

Axes subtractAxes(const Axes& lhs, const Axes& rhs) {
  return Axes{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Axes absoluteAxes(const Axes& axes) {
  return Axes{std::fabs(axes.x), std::fabs(axes.y), std::fabs(axes.z)};
}

bool axesWithin(const Axes& axes, float minimum, float maximum) {
  return std::fabs(axes.x) >= minimum && std::fabs(axes.x) <= maximum &&
         std::fabs(axes.y) >= minimum && std::fabs(axes.y) <= maximum &&
         std::fabs(axes.z) >= minimum && std::fabs(axes.z) <= maximum;
}

Status inProgressStatus() {
  return Status::Error(Err::IN_PROGRESS, "Operation in progress");
}

Status normalizeTransport(Status status) {
  if (status.code == Err::IN_PROGRESS) {
    return Status::Error(Err::I2C_BUSY, "Transport callback returned IN_PROGRESS",
                         status.detail);
  }
  return status;
}

}  // namespace

Status validateDriverConfig(const DriverConfig& config) {
  if (config.i2cWrite == nullptr || config.i2cWriteRead == nullptr) {
    return Status::Error(Err::INVALID_CONFIG, "I2C callbacks are required");
  }
  const uint8_t address = static_cast<uint8_t>(config.address);
  if (address != static_cast<uint8_t>(SensorAddress::SA0_GND) &&
      address != static_cast<uint8_t>(SensorAddress::SA0_VDD)) {
    return Status::Error(Err::INVALID_CONFIG, "I2C address must be 0x6A or 0x6B");
  }
  if (config.i2cTimeoutMs == 0U) {
    return Status::Error(Err::INVALID_CONFIG, "I2C timeout must be nonzero");
  }
  return Status::Ok();
}

Status validateProfile(const DeviceProfile& profile) {
  if (!validAccelFs(profile.accelFullScale) ||
      !validGyroFs(profile.gyroFullScale) ||
      !accelOdrAllowed(profile.accelOdr, profile.accelPowerMode) ||
      !gyroOdrAllowed(profile.gyroOdr, profile.gyroPowerMode) ||
      !validHpf(profile.gyroFilter.highPassMode) ||
      !validOffsetWeight(profile.accelOffsetWeight)) {
    return Status::Error(Err::INVALID_CONFIG, "Unsupported ODR, range, power, or filter");
  }
  if (!profile.blockDataUpdate) {
    return Status::Error(Err::UNSUPPORTED_PROFILE,
                         "Production profiles require block data update");
  }
  if (profile.accelUserOffset.x == std::numeric_limits<int8_t>::min() ||
      profile.accelUserOffset.y == std::numeric_limits<int8_t>::min() ||
      profile.accelUserOffset.z == std::numeric_limits<int8_t>::min()) {
    return Status::Error(Err::INVALID_CONFIG,
                         "Accelerometer user offsets must be -127..127");
  }
  if (profile.gyroPowerMode == GyroPowerMode::LOW_POWER_NORMAL &&
      (profile.gyroFilter.lpf1Enabled || profile.gyroFilter.highPassEnabled)) {
    return Status::Error(Err::UNSUPPORTED_PROFILE,
                         "Gyroscope filters require high-performance mode");
  }
  if (profile.gyroFilter.highPassEnabled) {
    return Status::Error(
        Err::UNSUPPORTED_PROFILE,
        "Gyroscope high-pass settling is not a production profile contract");
  }
  if (profile.accelFilter.highPassSlopeEnabled ||
      profile.accelFilter.lowPassOn6d) {
    return Status::Error(
        Err::UNSUPPORTED_PROFILE,
        "Slope/high-pass output and 6D filtering are not production sample profiles");
  }
  if (profile.interrupts.enabled) {
    return Status::Error(Err::UNSUPPORTED_PROFILE,
                         "Interrupt routing is not an enabled profile");
  }
  const FifoProfile& fifo = profile.fifo;
  if (fifo.mode != FifoMode::BYPASS && fifo.mode != FifoMode::FIFO &&
      fifo.mode != FifoMode::CONTINUOUS) {
    return Status::Error(Err::UNSUPPORTED_PROFILE, "Unsupported FIFO mode");
  }
  if (fifo.watermarkWords > 2047U ||
      static_cast<uint8_t>(fifo.gyroDecimation) > 7U ||
      static_cast<uint8_t>(fifo.accelDecimation) > 7U ||
      static_cast<uint8_t>(fifo.fourthDecimation) > 7U ||
      static_cast<uint8_t>(fifo.fourthDataset) > 2U ||
      static_cast<uint8_t>(fifo.trigger) > 1U ||
      static_cast<uint8_t>(fifo.timestampResolution) > 1U ||
      static_cast<uint8_t>(fifo.odr) > static_cast<uint8_t>(Odr::HZ_6660)) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid FIFO ODR, decimation or watermark");
  }
  if (!fifo.enabled) {
    if (fifo.mode != FifoMode::BYPASS || fifo.odr != Odr::POWER_DOWN ||
        fifo.gyroDecimation != FifoDecimation::OFF ||
        fifo.accelDecimation != FifoDecimation::OFF || fifo.watermarkWords != 0U ||
        fifo.fourthDataset != FifoFourthDataset::OFF ||
        fifo.fourthDecimation != FifoDecimation::OFF ||
        fifo.trigger != FifoTrigger::INTERNAL ||
        fifo.timestampResolution != TimestampResolution::MS_6_4) {
      return Status::Error(Err::INVALID_CONFIG, "Disabled FIFO requires canonical bypass");
    }
    return Status::Ok();
  }
  const uint8_t gyroFactor = fifoDecimationFactor(fifo.gyroDecimation);
  const uint8_t accelFactor = fifoDecimationFactor(fifo.accelDecimation);
  const uint8_t fourthFactor = fifoDecimationFactor(fifo.fourthDecimation);
  const bool fourthEnabled = fifo.fourthDataset != FifoFourthDataset::OFF;
  if (fourthEnabled != (fourthFactor != 0U) ||
      (!counterProfile(profile) &&
       (fifo.trigger != FifoTrigger::INTERNAL ||
        fifo.timestampResolution != TimestampResolution::MS_6_4)) ||
      (counterProfile(profile) &&
       static_cast<uint8_t>(profile.accelOdr) < static_cast<uint8_t>(Odr::HZ_26))) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid fourth source, trigger or counter dependency");
  }
  if ((fifo.trigger == FifoTrigger::INTERNAL && fifo.odr == Odr::POWER_DOWN) ||
      (gyroFactor == 0U && accelFactor == 0U && !fourthEnabled) ||
      profile.accelOdr == Odr::HZ_1_6 || profile.gyroSleepEnabled ||
      (profile.accelOdr == Odr::POWER_DOWN && profile.gyroOdr == Odr::POWER_DOWN) ||
      (gyroFactor != 0U && profile.gyroOdr == Odr::POWER_DOWN) ||
      (accelFactor != 0U && profile.accelOdr == Odr::POWER_DOWN)) {
    return Status::Error(Err::INVALID_CONFIG, "FIFO requires active sensor sources");
  }
  // Step events replace the trigger for every dataset. Their captured motion
  // values have no promised periodic cadence or per-event freshness.
  if (fifo.trigger == FifoTrigger::STEP_EVENT) return Status::Ok();
  // AN5130 Figure 32: min(max(sensor ODR), ODR_FIFO). Compare nominal
  // cadence classes, whose successive encodings double, instead of treating
  // rounded labels such as 416/833 Hz as exact independent oscillators.
  uint8_t trigger = static_cast<uint8_t>(profile.accelOdr);
  if (static_cast<uint8_t>(profile.gyroOdr) > trigger)
    trigger = static_cast<uint8_t>(profile.gyroOdr);
  if (static_cast<uint8_t>(fifo.odr) < trigger)
    trigger = static_cast<uint8_t>(fifo.odr);
  const uint32_t triggerCadence = 1UL << trigger;
  if ((gyroFactor != 0U && triggerCadence >
       (1UL << static_cast<uint8_t>(profile.gyroOdr)) * gyroFactor) ||
      (accelFactor != 0U && triggerCadence >
       (1UL << static_cast<uint8_t>(profile.accelOdr)) * accelFactor)) {
    return Status::Error(Err::INVALID_CONFIG, "FIFO cadence exceeds its motion source");
  }
  return Status::Ok();
}

Status fifoPatternWord(const DeviceProfile& profile, uint16_t index,
                       FifoPatternWord& out) {
  const Status valid = validateProfile(profile);
  if (!valid.ok()) return valid;
  if (!profile.fifo.enabled) {
    return Status::Error(Err::INVALID_PARAM, "FIFO pattern requires enabled profile");
  }
  const uint8_t factors[3] = {
      fifoDecimationFactor(profile.fifo.gyroDecimation),
      fifoDecimationFactor(profile.fifo.accelDecimation),
      fifoDecimationFactor(profile.fifo.fourthDecimation)};
  uint16_t ticks = 1;
  for (uint8_t factor : factors) {
    if (factor == 0U) continue;
    uint16_t common = 1;
    for (uint8_t divisor = 1; divisor <= 32U; ++divisor) {
      if (ticks % divisor == 0U && factor % divisor == 0U) common = divisor;
    }
    ticks = static_cast<uint16_t>(ticks * factor / common);
  }
  FifoPatternWord candidate{};
  uint16_t words = 0;
  // At most 96 trigger positions, including the relatively-prime 3/32 case.
  // Each enabled dataset contributes a complete XYZ in gyro/accel order.
  for (uint16_t tick = 0; tick < ticks; ++tick) {
    const bool temperature = profile.fifo.fourthDataset == FifoFourthDataset::TEMPERATURE;
    const uint8_t quantities[3] = {SAMPLE_ANGULAR_RATE, SAMPLE_ACCELERATION,
        static_cast<uint8_t>(temperature ? SAMPLE_TEMPERATURE : 0U)};
    const FifoRecordKind kinds[3] = {FifoRecordKind::ANGULAR_RATE,
        FifoRecordKind::ACCELERATION,
        temperature ? FifoRecordKind::TEMPERATURE : FifoRecordKind::TIMESTAMP_STEP};
    for (uint8_t sensor = 0; sensor < 3U; ++sensor) {
      if (factors[sensor] == 0U || tick % factors[sensor] != 0U) continue;
      if (index >= words && index < words + 3U) {
        candidate.quantityMask = quantities[sensor];
        candidate.axis = static_cast<uint8_t>(index - words);
        candidate.kind = kinds[sensor];
      }
      words = static_cast<uint16_t>(words + 3U);
    }
  }
  if (index >= words) {
    return Status::Error(Err::INVALID_PARAM, "FIFO pattern index is out of range");
  }
  candidate.patternWords = words;
  out = candidate;
  return Status::Ok();
}

uint64_t odrPeriodUs(Odr odr) {
  uint64_t milliHz = 0;
  switch (odr) {
    case Odr::HZ_1_6: milliHz = 1600; break;
    case Odr::HZ_12_5: milliHz = 12500; break;
    case Odr::HZ_26: milliHz = 26000; break;
    case Odr::HZ_52: milliHz = 52000; break;
    case Odr::HZ_104: milliHz = 104000; break;
    case Odr::HZ_208: milliHz = 208000; break;
    case Odr::HZ_416: milliHz = 416000; break;
    case Odr::HZ_833: milliHz = 833000; break;
    case Odr::HZ_1660: milliHz = 1660000; break;
    case Odr::HZ_3330: milliHz = 3330000; break;
    case Odr::HZ_6660: milliHz = 6660000; break;
    case Odr::POWER_DOWN:
    default: return 0;
  }
  return (1000000000ULL + milliHz - 1ULL) / milliHz;
}

uint64_t requiredSettleUs(const DeviceProfile& profile) {
  uint64_t settle = 0U;
  const uint64_t accelPeriod = odrPeriodUs(profile.accelOdr);
  if (accelPeriod != 0U) {
    const uint64_t accelSamples = profile.accelFilter.lpf2Enabled ? 40U : 14U;
    settle = saturatingMultiply(accelPeriod, accelSamples);
  }
  if (profile.gyroOdr != Odr::POWER_DOWN) {
    uint64_t gyroSettle = 70000U;
    if (!profile.gyroSleepEnabled) {
      gyroSettle = saturatingAdd(
          gyroSettle,
          saturatingMultiply(odrPeriodUs(profile.gyroOdr),
                             gyroSettleSamples(profile)));
    }
    if (gyroSettle > settle) settle = gyroSettle;
  }
  return settle;
}

uint32_t maximumSelfTestTransactions(uint16_t samples) {
  if (samples < SELF_TEST_MIN_SAMPLES || samples > SELF_TEST_MAX_SAMPLES) return 0U;
  // Each sample reserves three ready checks and one burst in four phases.
  // Fixed work reserves the 20 normal test writes, one failure-cleanup write,
  // and the 66-callback profile restore.
  return 16U * (static_cast<uint32_t>(samples) + SELF_TEST_DISCARD_SAMPLES) +
         SELF_TEST_FIXED_TRANSACTIONS;
}

uint32_t maximumCalibrationTransactions(uint16_t samples) {
  if (samples < CALIBRATION_MIN_SAMPLES || samples > CALIBRATION_MAX_SAMPLES) return 0U;
  return 4U * static_cast<uint32_t>(samples);
}

uint32_t maximumFifoPurgeTransactions(uint16_t maxWords, bool stopCollection) {
  if (maxWords == 0U || maxWords > FIFO_CAPACITY_WORDS) return 0U;
  return static_cast<uint32_t>(maxWords) + (stopCollection ? 7U : 5U);
}

uint32_t maximumFifoReadTransactions(uint16_t maxWords) {
  if (maxWords == 0U || maxWords > FIFO_CAPACITY_WORDS) return 0U;
  return 2U * static_cast<uint32_t>(maxWords) + 3U;
}

Status accelSensitivityMicroGPerLsb(AccelFs fullScale, int32_t& out) {
  switch (fullScale) {
    case AccelFs::G_2: out = 61; break;
    case AccelFs::G_4: out = 122; break;
    case AccelFs::G_8: out = 244; break;
    case AccelFs::G_16: out = 488; break;
    default:
      return Status::Error(Err::INVALID_PARAM, "Invalid accelerometer full scale");
  }
  return Status::Ok();
}

Status gyroSensitivityMicroDpsPerLsb(GyroFs fullScale, int32_t& out) {
  switch (fullScale) {
    case GyroFs::DPS_125: out = 4375; break;
    case GyroFs::DPS_250: out = 8750; break;
    case GyroFs::DPS_500: out = 17500; break;
    case GyroFs::DPS_1000: out = 35000; break;
    case GyroFs::DPS_2000: out = 70000; break;
    default:
      return Status::Error(Err::INVALID_PARAM, "Invalid gyroscope full scale");
  }
  return Status::Ok();
}

Status decodeAcceleration(const RawAxes& raw, AccelFs fullScale, IntegerAxes& outMicroG) {
  int32_t sensitivity = 0;
  const Status status = accelSensitivityMicroGPerLsb(fullScale, sensitivity);
  if (!status.ok()) {
    return status;
  }
  outMicroG = IntegerAxes{static_cast<int64_t>(raw.x) * sensitivity,
                          static_cast<int64_t>(raw.y) * sensitivity,
                          static_cast<int64_t>(raw.z) * sensitivity};
  return Status::Ok();
}

Status decodeAngularRate(const RawAxes& raw, GyroFs fullScale, IntegerAxes& outMicroDps) {
  int32_t sensitivity = 0;
  const Status status = gyroSensitivityMicroDpsPerLsb(fullScale, sensitivity);
  if (!status.ok()) {
    return status;
  }
  outMicroDps = IntegerAxes{static_cast<int64_t>(raw.x) * sensitivity,
                            static_cast<int64_t>(raw.y) * sensitivity,
                            static_cast<int64_t>(raw.z) * sensitivity};
  return Status::Ok();
}

int32_t decodeTemperatureMilliC(int16_t raw) {
  return 25000 + static_cast<int32_t>((static_cast<int64_t>(raw) * 1000) / 256);
}

Status convertSample(const RawSampleResult& raw, ConvertedSample& out) {
  const bool qualityConsistent =
      (raw.quality != SampleQuality::READY_CHECKED ||
       raw.freshMask == raw.validMask) &&
      (raw.quality != SampleQuality::DIRECT_UNVERIFIED ||
       raw.freshMask == 0U);
  if (raw.validMask == 0U || (raw.validMask & ~SAMPLE_ALL) != 0U ||
      (raw.freshMask & ~raw.validMask) != 0U ||
      !validSampleQuality(raw.quality) || !qualityConsistent) {
    return Status::Error(Err::INVALID_PARAM, "Invalid raw sample provenance");
  }
  ConvertedSample candidate{};
  candidate.validMask = raw.validMask;
  candidate.freshMask = raw.freshMask;
  candidate.quality = raw.quality;
  candidate.sequence = raw.sequence;
  candidate.configGeneration = raw.configGeneration;
  candidate.readUptimeMs = raw.readUptimeMs;
  if ((raw.validMask & SAMPLE_ACCELERATION) != 0U) {
    const Status status =
        decodeAcceleration(raw.accel, raw.accelFullScale, candidate.accelMicroG);
    if (!status.ok()) return status;
  }
  if ((raw.validMask & SAMPLE_ANGULAR_RATE) != 0U) {
    const Status status =
        decodeAngularRate(raw.gyro, raw.gyroFullScale, candidate.gyroMicroDps);
    if (!status.ok()) return status;
  }
  if ((raw.validMask & SAMPLE_TEMPERATURE) != 0U) {
    candidate.temperatureMilliC = decodeTemperatureMilliC(raw.temperatureRaw);
  }
  out = candidate;
  return Status::Ok();
}

Status validateCalibrationRequest(const CalibrationRequest& request) {
  if (request.samples < CALIBRATION_MIN_SAMPLES ||
      request.samples > CALIBRATION_MAX_SAMPLES) {
    return Status::Error(Err::INVALID_PARAM, "Calibration samples must be 1..1000");
  }
  if (request.kind != CalibrationKind::ACCELEROMETER_BIAS &&
      request.kind != CalibrationKind::GYROSCOPE_BIAS) {
    return Status::Error(Err::INVALID_PARAM, "Invalid calibration kind");
  }
  if (!boundedAxes(request.expectedAccelerationG, 16.0f) ||
      !std::isfinite(request.limits.accelMaxPeakToPeakG) ||
      request.limits.accelMaxPeakToPeakG <= 0.0f ||
      request.limits.accelMaxPeakToPeakG > 4.0f ||
      !std::isfinite(request.limits.gyroMaxPeakToPeakDps) ||
      request.limits.gyroMaxPeakToPeakDps <= 0.0f ||
      request.limits.gyroMaxPeakToPeakDps > 2000.0f) {
    return Status::Error(Err::INVALID_PARAM, "Invalid calibration limits or fixture vector");
  }
  if (request.kind == CalibrationKind::ACCELEROMETER_BIAS) {
    const float magnitudeSquared = axesMagnitudeSquared(request.expectedAccelerationG);
    if (magnitudeSquared < 0.64f || magnitudeSquared > 1.44f) {
      return Status::Error(Err::INVALID_PARAM,
                           "Expected acceleration magnitude must be 0.8..1.2 g");
    }
  }
  return Status::Ok();
}

Status applyBias(Axes& sample, const Axes& bias) {
  if (!boundedAxes(sample, MAX_ABS_BIAS) || !boundedAxes(bias, MAX_ABS_BIAS)) {
    return Status::Error(Err::INVALID_PARAM, "Bias and sample must be finite and bounded");
  }
  const Axes candidate{sample.x - bias.x, sample.y - bias.y, sample.z - bias.z};
  if (!boundedAxes(candidate, MAX_ABS_BIAS)) {
    return Status::Error(Err::INVALID_PARAM, "Bias correction exceeds bounds");
  }
  sample = candidate;
  return Status::Ok();
}

Status LSM6DS3TR::bind(const DriverConfig& config) {
  const Status validation = validateDriverConfig(config);
  if (!validation.ok()) return validation;
  if (_active) return Status::Error(Err::BUSY, "Operation is active");
  if (_resultPending)
    return Status::Error(Err::RESULT_PENDING, "Terminal result must be taken");
  _driverConfig = config;
  _bound = true;
  _hasDesiredProfile = false;
  _hasVerifiedProfile = false;
  _configurationState = ConfigurationState::UNCONFIGURED;
  _configGeneration = 0;
  _validAfterUptimeMs = 0;
  _breakCounterContinuity();
  _mismatchRegister = 0;
  _mismatchExpected = 0;
  _mismatchObserved = 0;
  _transportSuccesses = 0;
  _transportFailures = 0;
  _lastTransportError = Status::Ok();
  _lastTransportErrorUptimeMs = 0;
  return Status::Ok();
}

void LSM6DS3TR::unbind() {
  _breakCounterContinuity();
  _clearFifoOutput();
  _fifoReadRequest = {};
  _bound = false;
  _active = false;
  _resultPending = false;
  _token = {};
  _job = JobKind::NONE;
  _hasDesiredProfile = false;
  _hasVerifiedProfile = false;
  _configurationState = ConfigurationState::UNCONFIGURED;
  _configGeneration = 0;
  _validAfterUptimeMs = 0;
  _workingResult = {};
  _terminalResult = {};
  _driverConfig = {};
}

Status LSM6DS3TR::_checkStart(const OperationTiming& timing) const {
  if (!_bound) return Status::Error(Err::NOT_BOUND, "Driver is not bound");
  if (_active) return Status::Error(Err::BUSY, "Operation is active");
  if (_resultPending)
    return Status::Error(Err::RESULT_PENDING, "Terminal result must be taken");
  if (_tokenExhausted) {
    return Status::Error(Err::RESULT_NOT_AVAILABLE, "Operation token space exhausted");
  }
  if (timing.deadlineMs <= timing.nowMs) {
    return Status::Error(Err::INVALID_PARAM, "Deadline must be after start time");
  }
  return Status::Ok();
}

Status LSM6DS3TR::_start(JobKind kind, const OperationTiming& timing,
                        OperationToken& token) {
  token = {};
  const Status check = _checkStart(timing);
  if (!check.ok()) return check;
  _token = OperationToken{_nextToken};
  if (_nextToken == std::numeric_limits<uint64_t>::max()) {
    _tokenExhausted = true;
  } else {
    ++_nextToken;
  }
  token = _token;
  _active = true;
  _job = kind;
  _step = 0;
  _substep = 0;
  _deadlineMs = timing.deadlineMs;
  _waitUntilMs = 0;
  _operationTransactions = 0;
  switch (kind) {
    case JobKind::PROBE: _operationTransactionLimit = MAX_PROBE_TRANSACTIONS; break;
    case JobKind::CONFIGURE: _operationTransactionLimit = MAX_CONFIGURE_TRANSACTIONS; break;
    case JobKind::SAMPLE: _operationTransactionLimit = MAX_SAMPLE_TRANSACTIONS; break;
    case JobKind::RESET:
    case JobKind::BOOT: _operationTransactionLimit = MAX_RESET_TRANSACTIONS; break;
    case JobKind::RECOVER: _operationTransactionLimit = MAX_RECOVER_TRANSACTIONS; break;
    case JobKind::RECONCILE: _operationTransactionLimit = MAX_RECONCILE_TRANSACTIONS; break;
    case JobKind::POWER_DOWN: _operationTransactionLimit = MAX_POWER_DOWN_TRANSACTIONS; break;
    default: _operationTransactionLimit = 1U; break;
  }
  _transactionUsed = false;
  _waiting = false;
  _pollBoundary = false;
  _hardwareStateMayHaveChanged = false;
  _configurationMayBeUnknown = false;
  // Resolve SETTLING->KNOWN once, at capture time, so the snapshot always
  // agrees with what configurationState() reports to the caller.
  _configurationStateBeforeOperation = configurationState(timing.nowMs);
  _validAfterBeforeOperationMs = _validAfterUptimeMs;
  _workingResult = {};
  _workingResult.token = _token;
  _workingResult.kind = kind;
  _workingResult.state = OperationState::ACTIVE;
  _workingResult.status = inProgressStatus();
  _workingResult.startedUptimeMs = timing.nowMs;
  _counterSetupStep = 0;
  _timestampResetNeeded = false;
  return inProgressStatus();
}

Status LSM6DS3TR::startProbe(const OperationTiming& timing, OperationToken& token) {
  token = {};
  return _start(JobKind::PROBE, timing, token);
}

Status LSM6DS3TR::startConfigure(const DeviceProfile& profile,
                                const OperationTiming& timing,
                                OperationToken& token) {
  token = {};
  const Status admission = _checkStart(timing);
  if (!admission.ok()) return admission;
  const Status validation = validateProfile(profile);
  if (!validation.ok()) return validation;
  const Status status = _start(JobKind::CONFIGURE, timing, token);
  if (!status.inProgress()) return status;
  _desiredProfile = profile;
  _hasDesiredProfile = true;
  _configurationState = ConfigurationState::APPLYING;
  _prepareManagedImage(profile);
  return status;
}

Status LSM6DS3TR::_checkReadyForKnownConfiguration(uint64_t nowMs) const {
  const ConfigurationState state = configurationState(nowMs);
  if (state == ConfigurationState::SETTLING) {
    return Status::Error(Err::SETTLING, "Configuration is settling");
  }
  if (state != ConfigurationState::KNOWN || !_hasVerifiedProfile) {
    return Status::Error(Err::CONFIGURATION_UNKNOWN, "Configuration is not verified");
  }
  return Status::Ok();
}

Status LSM6DS3TR::_checkFifoBypass(uint64_t nowMs) {
  uint8_t ctrl5 = 0;
  const Status status = _read(cmd::REG_FIFO_CTRL5, &ctrl5, 1, nowMs);
  if (!status.ok()) return status;
  if ((ctrl5 & cmd::MASK_FIFO_MODE) != 0U) {
    return Status::Error(Err::FIFO_MAINTENANCE_REQUIRED,
                         "Stop FIFO through bounded maintenance before changing interpretation");
  }
  return Status::Ok();
}

Status LSM6DS3TR::startSample(const SampleRequest& request,
                             const OperationTiming& timing,
                             OperationToken& token) {
  token = {};
  const Status admission = _checkStart(timing);
  if (!admission.ok()) return admission;
  if (request.quantityMask == 0U || (request.quantityMask & ~SAMPLE_ALL) != 0U) {
    return Status::Error(Err::INVALID_PARAM, "Invalid sample quantity mask");
  }
  const Status ready = _checkReadyForKnownConfiguration(timing.nowMs);
  if (!ready.ok()) return ready;
  if (((request.quantityMask & SAMPLE_ACCELERATION) != 0U &&
       _verifiedProfile.accelOdr == Odr::POWER_DOWN) ||
      ((request.quantityMask & SAMPLE_ANGULAR_RATE) != 0U &&
       (_verifiedProfile.gyroOdr == Odr::POWER_DOWN ||
        _verifiedProfile.gyroSleepEnabled)) ||
      ((request.quantityMask & SAMPLE_TEMPERATURE) != 0U &&
       _verifiedProfile.accelOdr == Odr::POWER_DOWN &&
       _verifiedProfile.gyroOdr == Odr::POWER_DOWN)) {
    return Status::Error(Err::INVALID_PARAM,
                         "Requested quantity is powered down or sleeping");
  }
  // A verified profile has already passed validateProfile(), including BDU.
  const Status status = _start(JobKind::SAMPLE, timing, token);
  if (status.inProgress()) {
    _sampleRequest = request;
    _readyPolls = 0;
  }
  return status;
}

Status LSM6DS3TR::startReset(const OperationTiming& timing, OperationToken& token) {
  token = {};
  const Status admission = _checkStart(timing);
  if (!admission.ok()) return admission;
  if (!_hasDesiredProfile) {
    return Status::Error(Err::CONFIGURATION_UNKNOWN, "No desired profile to restore");
  }
  const Status status = _start(JobKind::RESET, timing, token);
  if (status.inProgress()) {
    _configurationState = ConfigurationState::APPLYING;
    _prepareManagedImage(_desiredProfile);
    _substep = 0;
    _readyPolls = 0;
  }
  return status;
}

Status LSM6DS3TR::startBoot(const OperationTiming& timing, OperationToken& token) {
  token = {};
  const Status admission = _checkStart(timing);
  if (!admission.ok()) return admission;
  if (!_hasDesiredProfile) {
    return Status::Error(Err::CONFIGURATION_UNKNOWN, "No desired profile to restore");
  }
  const Status status = _start(JobKind::BOOT, timing, token);
  if (status.inProgress()) {
    _configurationState = ConfigurationState::APPLYING;
    _prepareManagedImage(_desiredProfile);
    _substep = 0;
    _readyPolls = 0;
  }
  return status;
}

Status LSM6DS3TR::startRecover(const OperationTiming& timing, OperationToken& token) {
  token = {};
  const Status admission = _checkStart(timing);
  if (!admission.ok()) return admission;
  if (!_hasDesiredProfile) {
    return Status::Error(Err::CONFIGURATION_UNKNOWN, "No desired profile to restore");
  }
  const Status status = _start(JobKind::RECOVER, timing, token);
  if (status.inProgress()) {
    _configurationState = ConfigurationState::APPLYING;
    _prepareManagedImage(_desiredProfile);
    _substep = 0;
    _readyPolls = 0;
  }
  return status;
}

Status LSM6DS3TR::startReconcile(const OperationTiming& timing,
                                OperationToken& token) {
  token = {};
  const Status admission = _checkStart(timing);
  if (!admission.ok()) return admission;
  if (!_hasDesiredProfile) {
    return Status::Error(Err::CONFIGURATION_UNKNOWN, "No desired profile to reconcile");
  }
  if ((fifoCollecting(_desiredProfile) || counterProfile(_desiredProfile)) &&
      !_hasVerifiedProfile) {
    return Status::Error(Err::FIFO_MAINTENANCE_REQUIRED,
                         "Readback cannot establish buffered FIFO provenance");
  }
  const Status status = _start(JobKind::RECONCILE, timing, token);
  if (status.inProgress()) {
    _configurationState = ConfigurationState::APPLYING;
    _prepareManagedImage(_desiredProfile);
  }
  return status;
}

Status LSM6DS3TR::startPowerDown(const OperationTiming& timing,
                                OperationToken& token) {
  token = {};
  const Status admission = _checkStart(timing);
  if (!admission.ok()) return admission;
  const Status status = _start(JobKind::POWER_DOWN, timing, token);
  if (status.inProgress()) _configurationState = ConfigurationState::APPLYING;
  return status;
}

Status LSM6DS3TR::startSelfTest(const SelfTestRequest& request,
                               const OperationTiming& timing,
                               OperationToken& token) {
  token = {};
  const Status admission = _checkStart(timing);
  if (!admission.ok()) return admission;
  if (request.samples < SELF_TEST_MIN_SAMPLES ||
      request.samples > SELF_TEST_MAX_SAMPLES) {
    return Status::Error(Err::INVALID_PARAM, "Self-test samples must be 5..100");
  }
  const Status ready = _checkReadyForKnownConfiguration(timing.nowMs);
  if (!ready.ok()) return ready;
  if (fifoCollecting(_verifiedProfile)) {
    return Status::Error(Err::FIFO_MAINTENANCE_REQUIRED,
                         "Self-test requires a verified FIFO bypass profile");
  }
  const Status status = _start(JobKind::SELF_TEST, timing, token);
  if (status.inProgress()) {
    _operationTransactionLimit = maximumSelfTestTransactions(request.samples);
    _selfTestRequest = request;
    _selfTestRestoreProfile = _verifiedProfile;
    _configurationState = ConfigurationState::APPLYING;
    _primaryStatus = Status::Ok();
    _readyPolls = 0;
  }
  return status;
}

Status LSM6DS3TR::startCalibration(const CalibrationRequest& request,
                                  const OperationTiming& timing,
                                  OperationToken& token) {
  token = {};
  const Status admission = _checkStart(timing);
  if (!admission.ok()) return admission;
  const Status validation = validateCalibrationRequest(request);
  if (!validation.ok()) return validation;
  const Status ready = _checkReadyForKnownConfiguration(timing.nowMs);
  if (!ready.ok()) return ready;
  if ((request.kind == CalibrationKind::ACCELEROMETER_BIAS &&
       _verifiedProfile.accelOdr == Odr::POWER_DOWN) ||
      (request.kind == CalibrationKind::GYROSCOPE_BIAS &&
       (_verifiedProfile.gyroOdr == Odr::POWER_DOWN ||
        _verifiedProfile.gyroSleepEnabled))) {
    return Status::Error(Err::INVALID_PARAM,
                         "Calibration sensor is powered down or sleeping");
  }
  const Status status = _start(JobKind::CALIBRATION, timing, token);
  if (status.inProgress()) {
    _operationTransactionLimit = maximumCalibrationTransactions(request.samples);
    _calibrationRequest = request;
    _samplesDone = 0;
    _sumX = _sumY = _sumZ = 0;
    _rawMin = RawAxes{std::numeric_limits<int16_t>::max(),
                      std::numeric_limits<int16_t>::max(),
                      std::numeric_limits<int16_t>::max()};
    _rawMax = RawAxes{std::numeric_limits<int16_t>::min(),
                      std::numeric_limits<int16_t>::min(),
                      std::numeric_limits<int16_t>::min()};
    _readyPolls = 0;
  }
  return status;
}

Status LSM6DS3TR::startFifoPurge(const FifoPurgeRequest& request,
                                const OperationTiming& timing,
                                OperationToken& token) {
  token = {};
  const Status admission = _checkStart(timing);
  if (!admission.ok()) return admission;
  if (request.maxWords == 0U || request.maxWords > FIFO_CAPACITY_WORDS) {
    return Status::Error(Err::INVALID_PARAM, "FIFO purge maximum must be 1..2048");
  }
  const Status status = _start(JobKind::FIFO_PURGE, timing, token);
  if (status.inProgress()) {
    _fifoPurgeRequest = request;
    _operationTransactionLimit = maximumFifoPurgeTransactions(request.maxWords,
                                                            request.stopCollection);
  }
  return status;
}

void LSM6DS3TR::_clearFifoOutput() {
  for (uint16_t i = 0; i < _fifoReadRequest.capacity; ++i)
    _fifoReadRequest.records[i] = {};
}

Status LSM6DS3TR::startFifoRead(const FifoReadRequest& request,
                              const OperationTiming& timing,
                              OperationToken& token) {
  token = {};
  const Status admission = _checkStart(timing);
  if (!admission.ok()) return admission;
  if (request.records == nullptr || request.capacity == 0U ||
      request.capacity > FIFO_CAPACITY_WORDS / 3U ||
      maximumFifoReadTransactions(request.maxWords) == 0U)
    return Status::Error(Err::INVALID_PARAM, "Invalid FIFO output buffer or word bound");
  const Status ready = _checkReadyForKnownConfiguration(timing.nowMs);
  if (!ready.ok()) return ready;
  if (!_verifiedProfile.fifo.enabled ||
      _verifiedProfile.fifo.mode == FifoMode::BYPASS)
    return Status::Error(Err::UNSUPPORTED_PROFILE, "Motion FIFO is not collecting");
  FifoPatternWord word{};
  const Status schema = fifoPatternWord(_verifiedProfile, 0, word);
  if (!schema.ok()) return schema;
  const Status status = _start(JobKind::FIFO_READ, timing, token);
  if (status.inProgress()) {
    _fifoReadRequest = request;
    _clearFifoOutput();
    _fifoAxes = {};
    _fifoAxesCount = 0;
    _fifoReadTarget = 0;
    _fifoPatternWords = word.patternWords;
    _workingResult.fifoRead.continuityLost = _fifoContinuityLost;
    _operationTransactionLimit = maximumFifoReadTransactions(request.maxWords);
  }
  return status;
}

void LSM6DS3TR::_prepareManagedImage(const DeviceProfile& profile) {
  const uint8_t registers[MANAGED_REGISTER_COUNT] = {
      cmd::REG_FUNC_CFG_ACCESS, cmd::REG_SENSOR_SYNC_TIME_FRAME,
      cmd::REG_SENSOR_SYNC_RES_RATIO, cmd::REG_FIFO_CTRL1, cmd::REG_FIFO_CTRL2,
      cmd::REG_FIFO_CTRL3, cmd::REG_FIFO_CTRL4, cmd::REG_FIFO_CTRL5,
      cmd::REG_DRDY_PULSE_CFG_G, cmd::REG_INT1_CTRL, cmd::REG_INT2_CTRL,
      cmd::REG_CTRL1_XL, cmd::REG_CTRL2_G, cmd::REG_CTRL3_C,
      cmd::REG_CTRL4_C, cmd::REG_CTRL5_C, cmd::REG_CTRL6_C,
      cmd::REG_CTRL7_G, cmd::REG_CTRL8_XL, cmd::REG_CTRL9_XL,
      cmd::REG_CTRL10_C, cmd::REG_MASTER_CONFIG, cmd::REG_TAP_CFG,
      cmd::REG_TAP_THS_6D, cmd::REG_INT_DUR2, cmd::REG_WAKE_UP_THS,
      cmd::REG_WAKE_UP_DUR, cmd::REG_FREE_FALL, cmd::REG_MD1_CFG,
      cmd::REG_MD2_CFG, cmd::REG_X_OFS_USR, cmd::REG_Y_OFS_USR,
      cmd::REG_Z_OFS_USR};
  const uint8_t values[MANAGED_REGISTER_COUNT] = {
      0, 0, 0,
      static_cast<uint8_t>(profile.fifo.watermarkWords & 0xFFU),
      buildFifoCtrl2(profile),
      static_cast<uint8_t>((static_cast<uint8_t>(profile.fifo.gyroDecimation) << 3U) |
                           static_cast<uint8_t>(profile.fifo.accelDecimation)),
      static_cast<uint8_t>(static_cast<uint8_t>(profile.fifo.fourthDecimation) << 3U),
      buildFifoCtrl5(profile), 0, 0, 0,
      buildCtrl1(profile), buildCtrl2(profile), buildCtrl3(profile),
      buildCtrl4(profile), 0, buildCtrl6(profile), buildCtrl7(profile),
      buildCtrl8(profile), 0, buildCounterControl(profile), 0, 0, 0, 0, 0,
      buildTimerResolution(profile), 0, 0, 0,
      static_cast<uint8_t>(profile.accelUserOffset.x),
      static_cast<uint8_t>(profile.accelUserOffset.y),
      static_cast<uint8_t>(profile.accelUserOffset.z)};
  for (uint8_t index = 0; index < MANAGED_REGISTER_COUNT; ++index) {
    _managedRegisters[index] = registers[index];
    _managedValues[index] = values[index];
  }
}

Status LSM6DS3TR::_read(uint8_t reg, uint8_t* data, size_t length,
                       uint64_t nowMs) {
  if (data == nullptr || length == 0U || length > MAX_TRANSPORT_READ_BYTES) {
    return Status::Error(Err::INVALID_PARAM,
                         "Read exceeds fixed transaction buffer");
  }
  if (_operationTransactionLimit != 0U &&
      _operationTransactions >= _operationTransactionLimit) {
    return Status::Error(Err::TRANSACTION_LIMIT_EXCEEDED,
                         "Operation transaction limit exceeded");
  }
  uint8_t tx = reg;
  _transactionUsed = true;
  _operationTransactions = saturatingIncrement(_operationTransactions);
  Status status = normalizeTransport(_driverConfig.i2cWriteRead(
      static_cast<uint8_t>(_driverConfig.address), &tx, 1, data, length,
      _driverConfig.i2cTimeoutMs, _driverConfig.i2cUser));
  if (status.ok()) {
    _transportSuccesses = saturatingIncrement(_transportSuccesses);
    if (reg == cmd::REG_FUNC_CFG_ACCESS)
      _bankMayBeUnknown = (data[0] & (cmd::MASK_FUNC_CFG_EN | cmd::MASK_FUNC_CFG_EN_B)) != 0U;
  } else {
    _transportFailures = saturatingIncrement(_transportFailures);
    _lastTransportError = status;
    _lastTransportErrorUptimeMs = nowMs;
  }
  return status;
}

Status LSM6DS3TR::_writeByte(uint8_t reg, uint8_t value, uint64_t nowMs) {
  if (_operationTransactionLimit != 0U &&
      _operationTransactions >= _operationTransactionLimit) {
    return Status::Error(Err::TRANSACTION_LIMIT_EXCEEDED,
                         "Operation transaction limit exceeded");
  }
  const uint8_t payload[2] = {reg, value};
  // Every managed write can change hardware state; the flags must be raised
  // only after the transaction-limit guard above has let the write through.
  if (!_hardwareStateMayHaveChanged && _job != JobKind::COUNTER_RESET)
    _breakCounterContinuity();
  if (reg == cmd::REG_FUNC_CFG_ACCESS) _bankMayBeUnknown = true;
  _hardwareStateMayHaveChanged = true;
  _configurationMayBeUnknown = true;
  _transactionUsed = true;
  _operationTransactions = saturatingIncrement(_operationTransactions);
  Status status = normalizeTransport(_driverConfig.i2cWrite(
      static_cast<uint8_t>(_driverConfig.address), payload, sizeof(payload),
      _driverConfig.i2cTimeoutMs, _driverConfig.i2cUser));
  if (status.ok()) {
    _transportSuccesses = saturatingIncrement(_transportSuccesses);
  } else {
    _transportFailures = saturatingIncrement(_transportFailures);
    _lastTransportError = status;
    _lastTransportErrorUptimeMs = nowMs;
  }
  return status;
}

void LSM6DS3TR::_recordMismatch(uint8_t reg, uint8_t expected,
                               uint8_t observed) {
  _mismatchRegister = reg;
  _mismatchExpected = expected;
  _mismatchObserved = observed;
  _workingResult.configuration.mismatchRegister = reg;
  _workingResult.configuration.expectedValue = expected;
  _workingResult.configuration.observedValue = observed;
}

void LSM6DS3TR::_invalidateConfiguration() {
  _configurationState = _hasDesiredProfile ? ConfigurationState::UNKNOWN
                                            : ConfigurationState::UNCONFIGURED;
  _hasVerifiedProfile = false;
  _validAfterUptimeMs = 0;
  _fifoContinuityLost = true;
  _breakCounterContinuity();
}

void LSM6DS3TR::_breakCounterContinuity() {
  _counterEpoch = saturatingIncrement64(_counterEpoch);
  _counterContinuityLost = true;
}

Status LSM6DS3TR::startCounterReset(const CounterResetRequest& request,
                                   const OperationTiming& timing,
                                   OperationToken& token) {
  token = {};
  const Status admission = _checkStart(timing);
  if (!admission.ok()) return admission;
  if (!request.timestamp && !request.steps)
    return Status::Error(Err::INVALID_PARAM, "Select at least one counter reset");
  const Status ready = _checkReadyForKnownConfiguration(timing.nowMs);
  if (!ready.ok()) return ready;
  if (!counterProfile(_verifiedProfile))
    return Status::Error(Err::UNSUPPORTED_PROFILE, "Counter reset requires timestamp/step profile");
  if (fifoCollecting(_verifiedProfile))
    return Status::Error(Err::FIFO_MAINTENANCE_REQUIRED, "Counter reset requires FIFO Bypass");
  const Status status = _start(JobKind::COUNTER_RESET, timing, token);
  if (status.inProgress()) {
    _counterResetRequest = request;
    _operationTransactionLimit = MAX_COUNTER_RESET_TRANSACTIONS;
  }
  return status;
}

Status LSM6DS3TR::_stepCounterReset(uint64_t nowMs) {
  if (_step < 2U) {
    uint8_t value = 0;
    const uint8_t reg = _step == 0U ? cmd::REG_FUNC_CFG_ACCESS : cmd::REG_WHO_AM_I;
    const Status status = _read(reg, &value, 1, nowMs);
    if (!status.ok()) return status;
    if ((_step == 0U && value != 0U) ||
        (_step == 1U && value != cmd::WHO_AM_I_VALUE)) {
      _invalidateConfiguration();
      return Status::Error(_step == 0U ? Err::CONFIGURATION_UNKNOWN : Err::CHIP_ID_MISMATCH,
                           "Counter reset bank or identity mismatch", value);
    }
    if (_step == 1U)
      _workingResult.probe = {static_cast<uint8_t>(_driverConfig.address), value};
    ++_step;
    return inProgressStatus();
  }
  if (_step == 2U) {
    const Status status = _checkFifoBypass(nowMs);
    if (!status.ok()) return status;
    _step = 3;
    return inProgressStatus();
  }
  if (_step == 3U) {
    // The epoch changes once before the first possible reset effect.
    _breakCounterContinuity();
    _fifoContinuityLost = true;
    _step = 4;
    if (_counterResetRequest.timestamp) {
      const uint32_t before = _operationTransactions;
      const Status status = _writeByte(cmd::REG_TIMESTAMP2, cmd::TIMESTAMP_RESET_VALUE, nowMs);
      _workingResult.counters.timestampResetMayHaveOccurred = _operationTransactions != before;
      if (!status.ok()) return status;
      _workingResult.counters.timestampResetConfirmed = true;
      // The command changes counter state, not the stable profile image.
      _configurationMayBeUnknown = false;
      return inProgressStatus();
    }
  }
  if (_step == 4U) {
    _step = 5;
    if (_counterResetRequest.steps) {
      const uint32_t before = _operationTransactions;
      const Status status = _writeByte(cmd::REG_CTRL10_C,
          static_cast<uint8_t>(COUNTER_ENABLE | PEDOMETER_RESET), nowMs);
      _workingResult.counters.stepResetMayHaveOccurred = _operationTransactions != before;
      if (!status.ok()) return status;
      return inProgressStatus();
    }
  }
  if (_step == 5U && _counterResetRequest.steps) {
    const Status status = _writeByte(cmd::REG_CTRL10_C, COUNTER_ENABLE, nowMs);
    if (!status.ok()) return status;
    _step = 6;
    return inProgressStatus();
  }
  if (_step == 6U && _counterResetRequest.steps) {
    uint8_t observed = 0;
    const Status status = _read(cmd::REG_CTRL10_C, &observed, 1, nowMs);
    if (!status.ok()) return status;
    if (observed != COUNTER_ENABLE) {
      _recordMismatch(cmd::REG_CTRL10_C, COUNTER_ENABLE, observed);
      return Status::Error(Err::CONFIGURATION_MISMATCH, "Pedometer reset clear mismatch");
    }
    _workingResult.counters.stepResetConfirmed = true;
    _configurationMayBeUnknown = false;
  }
  if (_counterResetRequest.timestamp && _counterResetRequest.steps)
    _counterContinuityLost = false;
  return _finish(Status::Ok(), OperationState::SUCCEEDED);
}

Status LSM6DS3TR::_stepCounterSetup(const DeviceProfile& target, uint64_t nowMs) {
  // All sixteen phases perform at most one callback. A partial bank transition
  // leaves explicit unknown evidence; cancel never performs cleanup I/O.
  const uint8_t phase = _counterSetupStep;
  uint8_t observed = 0;
  Status status = Status::Ok();
  if (phase == 0U) {
    status = _read(cmd::REG_WAKE_UP_DUR, &observed, 1, nowMs);
    if (status.ok()) _timestampResetNeeded =
        (observed & cmd::MASK_TIMER_HR) != buildTimerResolution(target);
  } else if (phase == 1U) {
    status = _writeByte(cmd::REG_CTRL10_C, 0, nowMs);
  } else if (phase == 2U || phase == 3U) {
    status = _writeByte(phase == 2U ? cmd::REG_CTRL1_XL : cmd::REG_CTRL2_G, 0, nowMs);
  } else if (phase == 4U || phase == 5U) {
    const uint8_t reg = phase == 4U ? cmd::REG_CTRL1_XL : cmd::REG_CTRL2_G;
    status = _read(reg, &observed, 1, nowMs);
    if (status.ok() && observed != 0U) {
      _recordMismatch(reg, 0, observed);
      status = Status::Error(Err::CONFIGURATION_MISMATCH, "Bank policy requires sensor power-down", reg);
    }
  } else if (phase == 6U || phase == 14U) {
    const uint32_t before = _operationTransactions;
    status = _writeByte(cmd::REG_FUNC_CFG_ACCESS,
                         phase == 6U ? cmd::MASK_FUNC_CFG_EN : 0U, nowMs);
    if (_operationTransactions != before) _bankMayBeUnknown = true;
  } else if (phase == 7U || phase == 15U) {
    const uint8_t expected = phase == 7U ? cmd::MASK_FUNC_CFG_EN : 0U;
    status = _read(cmd::REG_FUNC_CFG_ACCESS, &observed, 1, nowMs);
    if (status.ok() && observed != expected) {
      _recordMismatch(cmd::REG_FUNC_CFG_ACCESS, expected, observed);
      status = Status::Error(Err::CONFIGURATION_MISMATCH, "Counter policy bank readback mismatch");
    }
    if (status.ok() && phase == 15U) _bankMayBeUnknown = false;
  } else if (phase >= 8U && phase <= 10U) {
    const uint8_t index = phase - 8U;
    status = _writeByte(PEDOMETER_REGISTERS[index], PEDOMETER_VALUES[index], nowMs);
  } else if (phase >= 11U && phase <= 13U) {
    const uint8_t index = phase - 11U;
    status = _read(PEDOMETER_REGISTERS[index], &observed, 1, nowMs);
    if (status.ok() && observed != PEDOMETER_VALUES[index]) {
      _recordMismatch(PEDOMETER_REGISTERS[index], PEDOMETER_VALUES[index], observed);
      status = Status::Error(Err::CONFIGURATION_MISMATCH, "Fixed pedometer policy mismatch");
    }
  }
  if (status.ok()) ++_counterSetupStep;
  return status.ok() ? inProgressStatus() : status;
}

Status LSM6DS3TR::_stepProbe(uint64_t nowMs) {
  if (_step == 0U) {
    uint8_t access = 0;
    const Status status = _read(cmd::REG_FUNC_CFG_ACCESS, &access, 1, nowMs);
    if (!status.ok()) return status;
    if ((access & (cmd::MASK_FUNC_CFG_EN | cmd::MASK_FUNC_CFG_EN_B)) != 0U) {
      _invalidateConfiguration();
      return Status::Error(Err::CONFIGURATION_UNKNOWN,
                           "WHO_AM_I unavailable outside the main register bank");
    }
    _step = 1;
    return inProgressStatus();
  }
  uint8_t whoAmI = 0;
  const Status status = _read(cmd::REG_WHO_AM_I, &whoAmI, 1, nowMs);
  if (!status.ok()) return status;
  _workingResult.probe.address = static_cast<uint8_t>(_driverConfig.address);
  _workingResult.probe.whoAmI = whoAmI;
  if (whoAmI != cmd::WHO_AM_I_VALUE) {
    _invalidateConfiguration();
    return Status::Error(Err::CHIP_ID_MISMATCH, "WHO_AM_I mismatch", whoAmI);
  }
  return _finish(Status::Ok(), OperationState::SUCCEEDED);
}

Status LSM6DS3TR::_stepConfigure(uint64_t nowMs, bool reconcileOnly) {
  const DeviceProfile& target = _job == JobKind::SELF_TEST
                                    ? _selfTestRestoreProfile : _desiredProfile;
  const bool activateFifo = !reconcileOnly && fifoCollecting(target);
  if (((!reconcileOnly && _job == JobKind::CONFIGURE) ||
       (reconcileOnly && _job == JobKind::RECONCILE)) &&
      _substep == 0U) {
    uint8_t access = 0;
    const Status status =
        _read(cmd::REG_FUNC_CFG_ACCESS, &access, 1, nowMs);
    if (!status.ok()) return status;
    if ((access & (cmd::MASK_FUNC_CFG_EN | cmd::MASK_FUNC_CFG_EN_B)) != 0U) {
      _invalidateConfiguration();
      return Status::Error(Err::CONFIGURATION_UNKNOWN,
                           "Configuration requires the main register bank");
    }
    _substep = 1;
    return inProgressStatus();
  }
  if (((!reconcileOnly && _job == JobKind::CONFIGURE) ||
       (reconcileOnly && _job == JobKind::RECONCILE)) &&
      _substep == 1U) {
    uint8_t whoAmI = 0;
    const Status status = _read(cmd::REG_WHO_AM_I, &whoAmI, 1, nowMs);
    if (!status.ok()) return status;
    _workingResult.probe =
        ProbeResult{static_cast<uint8_t>(_driverConfig.address), whoAmI};
    if (whoAmI != cmd::WHO_AM_I_VALUE) {
      _invalidateConfiguration();
      return Status::Error(Err::CHIP_ID_MISMATCH, "WHO_AM_I mismatch", whoAmI);
    }
    _substep = 2;
    return inProgressStatus();
  }

  if (!reconcileOnly && _job == JobKind::CONFIGURE && _substep == 2U) {
    // AN5130 8.3.1: never alter interpretation while collection can append
    // old-profile data. Even an observed empty active FIFO can refill between
    // callbacks; explicit stopCollection maintenance owns that possible loss.
    const Status status = _checkFifoBypass(nowMs);
    if (!status.ok()) return status;
    _substep = 3;
    return inProgressStatus();
  }

  if (!reconcileOnly && counterProfile(target) && _counterSetupStep < 16U)
    return _stepCounterSetup(target, nowMs);
  if (!reconcileOnly && !counterProfile(target) && _counterSetupStep == 0U) {
    // Disabling a counter profile can also change TIMER_HR. Observe the actual
    // bit, including after raw access, and apply the same required reset rule.
    uint8_t observed = 0;
    const Status status = _read(cmd::REG_WAKE_UP_DUR, &observed, 1, nowMs);
    if (!status.ok()) return status;
    _timestampResetNeeded = (observed & cmd::MASK_TIMER_HR) != 0U;
    _counterSetupStep = 16;
    return inProgressStatus();
  }

  if (reconcileOnly && _step < MANAGED_REGISTER_COUNT) {
    uint8_t observed = 0;
    const uint8_t index = static_cast<uint8_t>(_step);
    const Status status = _read(_managedRegisters[index], &observed, 1, nowMs);
    if (!status.ok()) return status;
    if (observed != _managedValues[index]) {
      _recordMismatch(_managedRegisters[index], _managedValues[index], observed);
      _invalidateConfiguration();
      return Status::Error(Err::CONFIGURATION_MISMATCH,
                           "Managed register readback mismatch",
                           _managedRegisters[index]);
    }
    ++_step;
    if (_step == MANAGED_REGISTER_COUNT) {
      _step = CONFIGURE_SETTLE_ARM;
      _pollBoundary = true;
    }
    return inProgressStatus();
  }

  if (!reconcileOnly && _step < MANAGED_REGISTER_COUNT) {
    const uint8_t index = static_cast<uint8_t>(_step);
    const uint8_t value = _managedRegisters[index] == cmd::REG_CTRL10_C ? 0U :
                         _managedRegisters[index] == cmd::REG_FIFO_CTRL5
                              ? static_cast<uint8_t>(_managedValues[index] &
                                                     ~cmd::MASK_FIFO_MODE)
                              : _managedValues[index];
    const Status status =
        _writeByte(_managedRegisters[index], value, nowMs);
    if (!status.ok()) return status;
    ++_step;
    return inProgressStatus();
  }

  if (!reconcileOnly && _step < CONFIGURE_SETTLE_ARM) {
    const uint8_t index = static_cast<uint8_t>(_step - MANAGED_REGISTER_COUNT);
    const uint8_t expected = _managedRegisters[index] == cmd::REG_CTRL10_C ? 0U :
                            _managedRegisters[index] == cmd::REG_FIFO_CTRL5
                                 ? static_cast<uint8_t>(_managedValues[index] &
                                                        ~cmd::MASK_FIFO_MODE)
                                 : _managedValues[index];
    uint8_t observed = 0;
    const Status status = _read(_managedRegisters[index], &observed, 1, nowMs);
    if (!status.ok()) return status;
    if (observed != expected) {
      _recordMismatch(_managedRegisters[index], expected, observed);
      _invalidateConfiguration();
      return Status::Error(Err::CONFIGURATION_MISMATCH,
                           "Managed register readback mismatch",
                           _managedRegisters[index]);
    }
    ++_step;
    if (_step == CONFIGURE_SETTLE_ARM) {
      // Finalize from a fresh caller clock after the complete readback. A
      // changing poll budget can put earlier profile writes in this same poll.
      _pollBoundary = true;
    }
    return inProgressStatus();
  }

  if (!reconcileOnly && !counterProfile(target) && !_timestampResetNeeded &&
      _step == CONFIGURE_SETTLE_ARM)
    _counterSetupStep = 19;

  if (!reconcileOnly &&
      _step == CONFIGURE_SETTLE_ARM && _counterSetupStep < 19U) {
    if (_counterSetupStep == 16U) {
      _counterSetupStep = 17;
      if (_timestampResetNeeded) {
        const uint32_t before = _operationTransactions;
        const Status status = _writeByte(cmd::REG_TIMESTAMP2,
                                         cmd::TIMESTAMP_RESET_VALUE, nowMs);
        _workingResult.counters.timestampResetMayHaveOccurred =
            _operationTransactions != before;
        if (!status.ok()) return status;
        _workingResult.counters.timestampResetConfirmed = true;
        _timestampResetNeeded = false;
        return inProgressStatus();
      }
    }
    if (_counterSetupStep == 17U) {
      if (!counterProfile(target)) {
        _counterSetupStep = 19;
        _pollBoundary = true;
        return inProgressStatus();
      }
      const Status status = _writeByte(cmd::REG_CTRL10_C, COUNTER_ENABLE, nowMs);
      if (!status.ok()) return status;
      _counterSetupStep = 18;
      return inProgressStatus();
    }
    uint8_t observed = 0;
    const Status status = _read(cmd::REG_CTRL10_C, &observed, 1, nowMs);
    if (!status.ok()) return status;
    if (observed != COUNTER_ENABLE) {
      _recordMismatch(cmd::REG_CTRL10_C, COUNTER_ENABLE, observed);
      return Status::Error(Err::CONFIGURATION_MISMATCH, "Counter enable readback mismatch");
    }
    _counterSetupStep = 19;
    _pollBoundary = true;
    return inProgressStatus();
  }

  if (_step == CONFIGURE_SETTLE_ARM) {
    _mismatchRegister = 0;
    _mismatchExpected = 0;
    _mismatchObserved = 0;
    if (!activateFifo) {
      _verifiedProfile = target;
      _hasVerifiedProfile = true;
      if (!reconcileOnly) {
        _configGeneration = saturatingIncrement(_configGeneration);
        _fifoDiscardMask = 0;
        _fifoContinuityLost = false;
      }
    } else {
      // The disabled collection image is proved, but the desired active mode
      // has not been written/read back. Do not publish its generation yet.
      _hasVerifiedProfile = false;
    }
    uint64_t settleMs = 0U;
    const bool priorGateTrusted =
        _configurationStateBeforeOperation == ConfigurationState::KNOWN ||
        _configurationStateBeforeOperation == ConfigurationState::SETTLING;
    if (reconcileOnly && priorGateTrusted) {
      _validAfterUptimeMs = _validAfterBeforeOperationMs;
      _configurationState = _configurationStateBeforeOperation;
    } else {
      const uint64_t settleUs = requiredSettleUs(target);
      settleMs = settleUs == std::numeric_limits<uint64_t>::max()
                     ? settleUs
                     : (settleUs + 999U) / 1000U;
      if (settleMs != 0U) {
        settleMs = saturatingAdd(settleMs, CLOCK_QUANTIZATION_MARGIN_MS);
      }
      _validAfterUptimeMs = saturatingAdd(nowMs, settleMs);
      _configurationState = activateFifo ? ConfigurationState::APPLYING
                            : settleMs == 0U ? ConfigurationState::KNOWN
                                             : ConfigurationState::SETTLING;
    }
    _configurationMayBeUnknown = activateFifo;
    _step = CONFIGURE_SETTLE_WAIT;
    if (settleMs != 0U) {
      _waiting = true;
      return inProgressStatus();
    }
  }

  if (_step == CONFIGURE_SETTLE_WAIT &&
      nowMs < _validAfterUptimeMs) {
    _waiting = true;
    return inProgressStatus();
  }
  if (activateFifo) {
    if (_step == CONFIGURE_SETTLE_WAIT) {
      _step = CONFIGURE_FIFO_ACTIVATE;
      return inProgressStatus();
    }
    if (_step == CONFIGURE_FIFO_ACTIVATE) {
      const Status status = _writeByte(cmd::REG_FIFO_CTRL5, buildFifoCtrl5(target), nowMs);
      if (!status.ok()) return status;
      _step = CONFIGURE_FIFO_VERIFY;
      return inProgressStatus();
    }
    if (_step == CONFIGURE_FIFO_VERIFY) {
      uint8_t observed = 0;
      const Status status = _read(cmd::REG_FIFO_CTRL5, &observed, 1, nowMs);
      if (!status.ok()) return status;
      const uint8_t expected = buildFifoCtrl5(target);
      if (observed != expected) {
        _recordMismatch(cmd::REG_FIFO_CTRL5, expected, observed);
        _invalidateConfiguration();
        return Status::Error(Err::CONFIGURATION_MISMATCH,
                             "FIFO activation readback mismatch", cmd::REG_FIFO_CTRL5);
      }
      _step = CONFIGURE_COMMIT;
      return inProgressStatus();
    }
    _verifiedProfile = target;
    _hasVerifiedProfile = true;
    _configGeneration = saturatingIncrement(_configGeneration);
    _configurationMayBeUnknown = false;
    _fifoDiscardMask = static_cast<uint8_t>(
        (target.fifo.gyroDecimation != FifoDecimation::OFF ? SAMPLE_ANGULAR_RATE : 0U) |
        (target.fifo.accelDecimation != FifoDecimation::OFF ? SAMPLE_ACCELERATION : 0U));
    _fifoContinuityLost = false;
  }
  _configurationState = ConfigurationState::KNOWN;
  if (_job == JobKind::SELF_TEST) {
    _workingResult.selfTest.primaryStatus = _primaryStatus;
    _workingResult.selfTest.restorationStatus = Status::Ok();
    if (!_primaryStatus.ok()) {
      return _finish(_primaryStatus, OperationState::FAILED);
    }
  }
  return _finish(Status::Ok(), OperationState::SUCCEEDED);
}

Status LSM6DS3TR::_stepSample(uint64_t nowMs) {
  if (_step == 2U) {
    const uint64_t delayMs =
        sampleReadyPollDelayMs(_verifiedProfile, _sampleRequest);
    _waitUntilMs = saturatingAdd(
        nowMs, delayMs + CLOCK_QUANTIZATION_MARGIN_MS);
    _step = 0;
    _waiting = true;
    return inProgressStatus();
  }
  if (_step == 0U && _sampleRequest.checkDataReady) {
    if (_waitUntilMs != 0U && nowMs < _waitUntilMs) {
      _waiting = true;
      return inProgressStatus();
    }
    const Status status = _read(cmd::REG_STATUS_REG, &_sampleStatus, 1, nowMs);
    if (!status.ok()) return status;
    uint8_t required = 0;
    if ((_sampleRequest.quantityMask & SAMPLE_ACCELERATION) != 0U)
      required = cmd::MASK_XLDA;
    if ((_sampleRequest.quantityMask & SAMPLE_ANGULAR_RATE) != 0U)
      required = static_cast<uint8_t>(required | cmd::MASK_GDA);
    if ((_sampleRequest.quantityMask & SAMPLE_TEMPERATURE) != 0U)
      required = static_cast<uint8_t>(required | cmd::MASK_TDA);
    if ((_sampleStatus & required) != required) {
      ++_readyPolls;
      if (_readyPolls >= 65U) {
        return Status::Error(Err::DATA_NOT_READY,
                             "Sample data was not ready within 65 checks");
      }
      // Arm from a fresh caller timestamp after the callback has returned;
      // the callback duration must not consume the bus-silent cadence gate.
      _step = 2;
      _waitUntilMs = 0;
      _waiting = true;
      return inProgressStatus();
    }
    _readyPolls = 0;
    _waitUntilMs = 0;
    _step = 1;
    return inProgressStatus();
  }
  if (_step == 0U) _step = 1;

  uint8_t data[cmd::DATA_LEN_ALL] = {};
  const Status status = _read(cmd::REG_DATA_START_ALL, data, sizeof(data), nowMs);
  if (!status.ok()) return status;
  RawSampleResult& sample = _workingResult.sample;
  sample.temperatureRaw = decodeInt16(&data[0]);
  sample.gyro = decodeRawAxes(&data[2]);
  sample.accel = decodeRawAxes(&data[8]);
  sample.validMask = _sampleRequest.quantityMask;
  sample.freshMask = 0;
  if (_sampleRequest.checkDataReady) {
    if ((_sampleStatus & cmd::MASK_XLDA) != 0U)
      sample.freshMask = SAMPLE_ACCELERATION;
    if ((_sampleStatus & cmd::MASK_GDA) != 0U)
      sample.freshMask = static_cast<uint8_t>(sample.freshMask | SAMPLE_ANGULAR_RATE);
    if ((_sampleStatus & cmd::MASK_TDA) != 0U)
      sample.freshMask = static_cast<uint8_t>(sample.freshMask | SAMPLE_TEMPERATURE);
    sample.freshMask = static_cast<uint8_t>(sample.freshMask & sample.validMask);
  }
  sample.quality = _sampleRequest.checkDataReady
                       ? SampleQuality::READY_CHECKED
                       : SampleQuality::DIRECT_UNVERIFIED;
  _sampleSequence = saturatingIncrement64(_sampleSequence);
  sample.sequence = _sampleSequence;
  sample.configGeneration = _configGeneration;
  sample.readUptimeMs = nowMs;
  sample.accelFullScale = _verifiedProfile.accelFullScale;
  sample.gyroFullScale = _verifiedProfile.gyroFullScale;
  return _finish(Status::Ok(), OperationState::SUCCEEDED);
}

Status LSM6DS3TR::_stepResetBoot(uint64_t nowMs, bool boot, bool recovery) {
  if (_substep == 0U) {
    if (recovery) {
      uint8_t access = 0;
      const Status status =
          _read(cmd::REG_FUNC_CFG_ACCESS, &access, 1, nowMs);
      if (!status.ok()) return status;
      if ((access & (cmd::MASK_FUNC_CFG_EN | cmd::MASK_FUNC_CFG_EN_B)) != 0U) {
        _invalidateConfiguration();
        return Status::Error(Err::CONFIGURATION_UNKNOWN,
                             "Recovery requires the main register bank");
      }
      _substep = 1;
    } else {
      const Status status =
          _writeByte(cmd::REG_FUNC_CFG_ACCESS, 0, nowMs);
      if (!status.ok()) return status;
      _substep = 1;
    }
    return inProgressStatus();
  }
  if (recovery && _substep == 1U) {
    uint8_t whoAmI = 0;
    const Status status = _read(cmd::REG_WHO_AM_I, &whoAmI, 1, nowMs);
    if (!status.ok()) return status;
    _workingResult.probe =
        ProbeResult{static_cast<uint8_t>(_driverConfig.address), whoAmI};
    if (whoAmI != cmd::WHO_AM_I_VALUE) {
      _invalidateConfiguration();
      return Status::Error(Err::CHIP_ID_MISMATCH, "WHO_AM_I mismatch", whoAmI);
    }
    _substep = 9;
    return inProgressStatus();
  }
  if (!recovery && _substep == 1U) {
    uint8_t access = 0;
    const Status status =
        _read(cmd::REG_FUNC_CFG_ACCESS, &access, 1, nowMs);
    if (!status.ok()) return status;
    if ((access & (cmd::MASK_FUNC_CFG_EN | cmd::MASK_FUNC_CFG_EN_B)) != 0U) {
      _recordMismatch(cmd::REG_FUNC_CFG_ACCESS, 0, access);
      return Status::Error(Err::CONFIGURATION_MISMATCH,
                           "Main register bank clear did not verify",
                           cmd::REG_FUNC_CFG_ACCESS);
    }
    _substep = 2;
    return inProgressStatus();
  }
  if (_substep == 2U) {
    uint8_t whoAmI = 0;
    const Status status = _read(cmd::REG_WHO_AM_I, &whoAmI, 1, nowMs);
    if (!status.ok()) return status;
    _workingResult.probe =
        ProbeResult{static_cast<uint8_t>(_driverConfig.address), whoAmI};
    if (whoAmI != cmd::WHO_AM_I_VALUE) {
      _invalidateConfiguration();
      return Status::Error(Err::CHIP_ID_MISMATCH, "WHO_AM_I mismatch", whoAmI);
    }
    _substep = 9;
    return inProgressStatus();
  }
  if (_substep == 9U) {
    const Status status = _checkFifoBypass(nowMs);
    if (!status.ok()) return status;
    _substep = 3;
    return inProgressStatus();
  }
  if (_substep == 3U) {
    const uint8_t gyroOff = static_cast<uint8_t>(buildCtrl2(_desiredProfile) &
                                                 ~cmd::MASK_ODR_G);
    const Status status = _writeByte(cmd::REG_CTRL2_G, gyroOff, nowMs);
    if (!status.ok()) return status;
    _substep = 4;
    return inProgressStatus();
  }
  if (_substep == 4U) {
    const uint8_t accelHighPerformance =
        static_cast<uint8_t>(buildCtrl6(_desiredProfile) & ~cmd::MASK_XL_HM_MODE);
    const Status status =
        _writeByte(cmd::REG_CTRL6_C, accelHighPerformance, nowMs);
    if (!status.ok()) return status;
    _substep = 5;
    return inProgressStatus();
  }
  if (_substep == 5U) {
    DeviceProfile activeAccel = _desiredProfile;
    activeAccel.accelPowerMode = AccelPowerMode::HIGH_PERFORMANCE;
    if (activeAccel.accelOdr == Odr::POWER_DOWN ||
        activeAccel.accelOdr == Odr::HZ_1_6) {
      activeAccel.accelOdr = Odr::HZ_12_5;
    }
    const Status status =
        _writeByte(cmd::REG_CTRL1_XL, buildCtrl1(activeAccel), nowMs);
    if (!status.ok()) return status;
    _substep = 6;
    return inProgressStatus();
  }
  if (_substep == 6U) {
    const uint8_t command = static_cast<uint8_t>(buildCtrl3(_desiredProfile) |
                                                 (boot ? cmd::MASK_BOOT
                                                       : cmd::MASK_SW_RESET));
    const Status status = _writeByte(cmd::REG_CTRL3_C, command, nowMs);
    if (!status.ok()) return status;
    // Arm from a clock sampled after this write. The caller's nowMs may have
    // been sampled before earlier transactions in the same poll invocation.
    _waitUntilMs = 0;
    _substep = 7;
    _waiting = true;
    return inProgressStatus();
  }
  if (_substep == 7U) {
    if (_waitUntilMs == 0U) {
      // BOOT_TIME_MS is AN5130's 15 ms BOOT register-inaccessible interval.
      // For SW_RESET (~50 us in AN5130), this guard is library policy.
      // One extra tick accounts for the examples' millisecond clock truncation.
      _waitUntilMs = saturatingAdd(
          nowMs, static_cast<uint64_t>(cmd::BOOT_TIME_MS) +
                     CLOCK_QUANTIZATION_MARGIN_MS);
      _waiting = true;
      return inProgressStatus();
    }
    if (nowMs < _waitUntilMs) {
      _waiting = true;
      return inProgressStatus();
    }
    uint8_t ctrl3 = 0;
    const Status status = _read(cmd::REG_CTRL3_C, &ctrl3, 1, nowMs);
    if (!status.ok()) return status;
    const uint8_t mask = boot ? cmd::MASK_BOOT : cmd::MASK_SW_RESET;
    if ((ctrl3 & mask) != 0U) {
      ++_readyPolls;
      if (_readyPolls >= 16U) {
        return Status::Error(Err::TRANSACTION_LIMIT_EXCEEDED,
                             "Reset or boot completion poll limit exceeded");
      }
      _waitUntilMs = saturatingAdd(nowMs, 1U);
      _waiting = true;
      return inProgressStatus();
    }
    _readyPolls = 0;
    _step = 0;
    _substep = 8;
    _prepareManagedImage(_desiredProfile);
    return inProgressStatus();
  }
  return _stepConfigure(nowMs, false);
}

Status LSM6DS3TR::_stepPowerDown(uint64_t nowMs) {
  constexpr uint8_t ctrl1 = 0;
  constexpr uint8_t ctrl2 = 0;
  if (_step == 0U) {
    uint8_t access = 0;
    const Status status =
        _read(cmd::REG_FUNC_CFG_ACCESS, &access, 1, nowMs);
    if (!status.ok()) return status;
    _step = (access & (cmd::MASK_FUNC_CFG_EN | cmd::MASK_FUNC_CFG_EN_B)) != 0U
                ? 1U
                : 3U;
    return inProgressStatus();
  }
  if (_step == 1U) {
    const Status status =
        _writeByte(cmd::REG_FUNC_CFG_ACCESS, 0, nowMs);
    if (!status.ok()) return status;
    _step = 2;
    return inProgressStatus();
  }
  if (_step == 2U) {
    uint8_t access = 0;
    const Status status =
        _read(cmd::REG_FUNC_CFG_ACCESS, &access, 1, nowMs);
    if (!status.ok()) return status;
    if ((access & (cmd::MASK_FUNC_CFG_EN | cmd::MASK_FUNC_CFG_EN_B)) != 0U) {
      _recordMismatch(cmd::REG_FUNC_CFG_ACCESS, 0, access);
      return Status::Error(Err::CONFIGURATION_MISMATCH,
                           "Main register bank clear did not verify",
                           cmd::REG_FUNC_CFG_ACCESS);
    }
    _step = 3;
    return inProgressStatus();
  }
  if (_step == 3U) {
    uint8_t whoAmI = 0;
    const Status status = _read(cmd::REG_WHO_AM_I, &whoAmI, 1, nowMs);
    if (!status.ok()) return status;
    _workingResult.probe =
        ProbeResult{static_cast<uint8_t>(_driverConfig.address), whoAmI};
    if (whoAmI != cmd::WHO_AM_I_VALUE) {
      _invalidateConfiguration();
      return Status::Error(Err::CHIP_ID_MISMATCH, "WHO_AM_I mismatch", whoAmI);
    }
    _step = 8;
    return inProgressStatus();
  }
  if (_step == 8U) {
    const Status status = _checkFifoBypass(nowMs);
    if (!status.ok()) return status;
    _step = 4;
    return inProgressStatus();
  }
  if (_step == 4U) {
    const Status status = _writeByte(cmd::REG_CTRL1_XL, ctrl1, nowMs);
    if (!status.ok()) return status;
    ++_step;
    return inProgressStatus();
  }
  if (_step == 5U) {
    const Status status = _writeByte(cmd::REG_CTRL2_G, ctrl2, nowMs);
    if (!status.ok()) return status;
    ++_step;
    return inProgressStatus();
  }
  uint8_t observed = 0;
  if (_step == 6U) {
    const Status status = _read(cmd::REG_CTRL1_XL, &observed, 1, nowMs);
    if (!status.ok()) return status;
    if (observed != ctrl1) {
      _recordMismatch(cmd::REG_CTRL1_XL, ctrl1, observed);
      return Status::Error(Err::CONFIGURATION_MISMATCH,
                           "Power-down readback mismatch", cmd::REG_CTRL1_XL);
    }
    ++_step;
    return inProgressStatus();
  }
  if (_step == 7U) {
    const Status status = _read(cmd::REG_CTRL2_G, &observed, 1, nowMs);
    if (!status.ok()) return status;
    if (observed != ctrl2) {
      _recordMismatch(cmd::REG_CTRL2_G, ctrl2, observed);
      return Status::Error(Err::CONFIGURATION_MISMATCH,
                           "Power-down readback mismatch", cmd::REG_CTRL2_G);
    }
    _hasVerifiedProfile = false;
    _configurationMayBeUnknown = false;
    _configurationState = ConfigurationState::UNCONFIGURED;
    _validAfterUptimeMs = 0;
    _configGeneration = saturatingIncrement(_configGeneration);
    return _finish(Status::Ok(), OperationState::SUCCEEDED);
  }
  return Status::Error(Err::INVALID_PARAM, "Invalid power-down step");
}

Status LSM6DS3TR::_stepSelfTest(uint64_t nowMs) {
  auto beginRestore = [this]() {
    _workingResult.selfTest.primaryStatus = _primaryStatus;
    _substep = 100;
    _step = 0;
    _waitUntilMs = 0;
    _prepareManagedImage(_selfTestRestoreProfile);
  };

  auto routeFailureToRestore =
      [this, &beginRestore](const Status& status) -> Status {
    if (status.ok() || status.inProgress()) return status;
    if (_primaryStatus.ok()) _primaryStatus = status;
    _waitUntilMs = 0;

    // If stimulus may be active, preserve the vendor's terminal order on the
    // failure path too: power down the tested sensor, then disable self-test.
    // A failure in either cleanup write proceeds to bounded profile restore;
    // there is no loop or unbounded retry.
    if (_substep >= 6U && _substep <= 10U) {
      _substep = 90;
      _step = 0;
    } else if (_substep == 11U) {
      _substep = 91;
      _step = 0;
    } else if (_substep >= 16U && _substep < 20U) {
      _substep = 92;
      _step = 0;
    } else if (_substep == 20U && _step == 0U) {
      _substep = 92;
      _step = 0;
    } else if (_substep == 20U) {
      _substep = 93;
      _step = 0;
    } else {
      beginRestore();
      _pollBoundary = true;
    }
    return inProgressStatus();
  };

  auto averageStep = [this, nowMs](bool accel, uint16_t target, bool accumulate,
                                   bool& complete) -> Status {
    complete = false;
    const uint8_t readyMask = accel ? cmd::MASK_XLDA : cmd::MASK_GDA;
    const uint64_t samplePeriodMs = accel ? 20U : 5U;
    if (_step == 2U) {
      _waitUntilMs = saturatingAdd(
          nowMs, samplePeriodMs + CLOCK_QUANTIZATION_MARGIN_MS);
      _step = 0;
      _waiting = true;
      return inProgressStatus();
    }
    if (_step == 0U) {
      if (_waitUntilMs != 0U && nowMs < _waitUntilMs) {
        _waiting = true;
        return inProgressStatus();
      }
      uint8_t statusReg = 0;
      const Status status = _read(cmd::REG_STATUS_REG, &statusReg, 1, nowMs);
      if (!status.ok()) return status;
      if ((statusReg & readyMask) == 0U) {
        ++_readyPolls;
        if (_readyPolls >= 3U) {
          return Status::Error(Err::DATA_NOT_READY,
                               "Self-test data was not ready within three checks");
        }
        // Use a later CPU-only state to start the complete cadence interval
        // after this readiness callback has returned.
        _step = 2;
        _waitUntilMs = 0;
        _waiting = true;
        return inProgressStatus();
      }
      _readyPolls = 0;
      _waitUntilMs = 0;
      _step = 1;
      return inProgressStatus();
    }
    uint8_t data[6] = {};
    const Status status = _read(accel ? cmd::REG_DATA_START_ACCEL
                                      : cmd::REG_DATA_START_GYRO,
                                data, sizeof(data), nowMs);
    if (!status.ok()) return status;
    const RawAxes raw = decodeRawAxes(data);
    if (accumulate) {
      _sumX += raw.x;
      _sumY += raw.y;
      _sumZ += raw.z;
    }
    ++_samplesDone;
    // The enclosing final-average path clears this pending gate. Discards and
    // non-final averages arm it from fresh time on the owner's next poll.
    _step = 2;
    _waitUntilMs = 0;
    _waiting = true;
    if (_samplesDone >= target) complete = true;
    return inProgressStatus();
  };

  if (_substep == 100U) {
    return _stepConfigure(nowMs, false);
  }
  if (_substep == 0U) {
    const Status status = _writeByte(cmd::REG_CTRL5_C, 0, nowMs);
    if (!status.ok()) return routeFailureToRestore(status);
    ++_substep;
    return inProgressStatus();
  }
  if (_substep == 1U) {
    const Status status = _writeByte(cmd::REG_CTRL8_XL, 0, nowMs);
    if (!status.ok()) return routeFailureToRestore(status);
    ++_substep;
    return inProgressStatus();
  }
  if (_substep == 2U) {
    if (_step == 0U) {
      const Status status = _writeByte(cmd::REG_CTRL6_C, 0, nowMs);
      if (!status.ok()) return routeFailureToRestore(status);
      ++_step;
      return inProgressStatus();
    }
    if (_step == 1U) {
      const Status status = _writeByte(cmd::REG_CTRL2_G, 0, nowMs);
      if (!status.ok()) return routeFailureToRestore(status);
      ++_step;
      return inProgressStatus();
    }
    if (_step == 2U) {
      const uint8_t ctrl3 = static_cast<uint8_t>(cmd::MASK_BDU |
                                                 cmd::MASK_IF_INC);
      const Status status = _writeByte(cmd::REG_CTRL3_C, ctrl3, nowMs);
      if (!status.ok()) return routeFailureToRestore(status);
      ++_step;
      return inProgressStatus();
    }
    if (_step == 3U) {
      const Status status = _writeByte(cmd::REG_CTRL4_C, 0, nowMs);
      if (!status.ok()) return routeFailureToRestore(status);
      ++_step;
      return inProgressStatus();
    }
    if (_step == 4U) {
      const Status status = _writeByte(cmd::REG_CTRL7_G, 0, nowMs);
      if (!status.ok()) return routeFailureToRestore(status);
      ++_step;
      return inProgressStatus();
    }
    if (_step == 5U) {
      const Status status = _writeByte(cmd::REG_CTRL9_XL, 0, nowMs);
      if (!status.ok()) return routeFailureToRestore(status);
      ++_step;
      return inProgressStatus();
    }
    if (_step == 6U) {
      const Status status = _writeByte(cmd::REG_CTRL10_C, 0, nowMs);
      if (!status.ok()) return routeFailureToRestore(status);
      ++_step;
      return inProgressStatus();
    }
    if (_step >= 7U && _step <= 9U) {
      const uint8_t reg = static_cast<uint8_t>(
          cmd::REG_X_OFS_USR + static_cast<uint8_t>(_step - 7U));
      const Status status = _writeByte(reg, 0, nowMs);
      if (!status.ok()) return routeFailureToRestore(status);
      ++_step;
      return inProgressStatus();
    }
    DeviceProfile testProfile = _selfTestRestoreProfile;
    testProfile.accelOdr = Odr::HZ_52;
    testProfile.accelFullScale = AccelFs::G_4;
    const Status status =
        _writeByte(cmd::REG_CTRL1_XL, buildCtrl1(testProfile), nowMs);
    if (!status.ok()) return routeFailureToRestore(status);
    _step = 0;
    _waitUntilMs = 0;
    _substep = 3;
    _waiting = true;
    return inProgressStatus();
  }
  if (_substep == 3U || _substep == 7U || _substep == 13U || _substep == 17U) {
    if (_waitUntilMs == 0U) {
      uint64_t settleMs = SELF_TEST_GYRO_STIMULUS_SETTLE_MS;
      if (_substep == 3U || _substep == 7U) {
        settleMs = SELF_TEST_ACCEL_SETTLE_MS;
      } else if (_substep == 13U) {
        settleMs = SELF_TEST_GYRO_SETTLE_MS;
      }
      _waitUntilMs = saturatingAdd(
          nowMs, settleMs + CLOCK_QUANTIZATION_MARGIN_MS);
      _waiting = true;
      return inProgressStatus();
    }
    if (nowMs < _waitUntilMs) {
      _waiting = true;
      return inProgressStatus();
    }
    _samplesDone = 0;
    _readyPolls = 0;
    _step = 0;
    ++_substep;
    return inProgressStatus();
  }
  if (_substep == 4U || _substep == 8U || _substep == 14U || _substep == 18U) {
    bool complete = false;
    const bool accel = _substep < 10U;
    const Status status =
        averageStep(accel, SELF_TEST_DISCARD_SAMPLES, false, complete);
    if (!status.ok() && !status.inProgress()) return routeFailureToRestore(status);
    if (complete) {
      _samplesDone = 0;
      _sumX = _sumY = _sumZ = 0;
      ++_substep;
    }
    return status;
  }
  if (_substep == 5U || _substep == 9U || _substep == 15U || _substep == 19U) {
    bool complete = false;
    const bool accel = _substep < 10U;
    const Status status = averageStep(accel, _selfTestRequest.samples, true, complete);
    if (!status.ok() && !status.inProgress()) return routeFailureToRestore(status);
    if (!complete) return status;
    const RawAxes average{
        static_cast<int16_t>(_sumX / _selfTestRequest.samples),
        static_cast<int16_t>(_sumY / _selfTestRequest.samples),
        static_cast<int16_t>(_sumZ / _selfTestRequest.samples)};
    if (_substep == 5U || _substep == 15U) _phaseBaseline = average;
    else _phaseStimulus = average;

    if (_substep == 5U) {
      _workingResult.selfTest.accelBaselineG = rawAxesToFloat(_phaseBaseline, 0.000122f);
      _substep = 6;
    } else if (_substep == 9U) {
      _workingResult.selfTest.accelStimulusG = rawAxesToFloat(_phaseStimulus, 0.000122f);
      _workingResult.selfTest.accelDeltaG = absoluteAxes(
          subtractAxes(_workingResult.selfTest.accelStimulusG,
                       _workingResult.selfTest.accelBaselineG));
      _workingResult.selfTest.accelPass =
          axesWithin(_workingResult.selfTest.accelDeltaG,
                     SELF_TEST_ACCEL_MIN_G, SELF_TEST_ACCEL_MAX_G);
      if (!_workingResult.selfTest.accelPass && _primaryStatus.ok()) {
        _primaryStatus =
            Status::Error(Err::SELF_TEST_FAIL, "Accelerometer self-test failed");
      }
      _substep = 10;
    } else if (_substep == 15U) {
      _workingResult.selfTest.gyroBaselineDps = rawAxesToFloat(_phaseBaseline, 0.070f);
      _substep = 16;
    } else {
      _workingResult.selfTest.gyroStimulusDps = rawAxesToFloat(_phaseStimulus, 0.070f);
      _workingResult.selfTest.gyroDeltaDps = absoluteAxes(
          subtractAxes(_workingResult.selfTest.gyroStimulusDps,
                       _workingResult.selfTest.gyroBaselineDps));
      _workingResult.selfTest.gyroPass =
          axesWithin(_workingResult.selfTest.gyroDeltaDps,
                     SELF_TEST_GYRO_MIN_DPS, SELF_TEST_GYRO_MAX_DPS);
      if (!_workingResult.selfTest.gyroPass && _primaryStatus.ok()) {
        _primaryStatus = Status::Error(Err::SELF_TEST_FAIL, "Gyroscope self-test failed");
      }
      _substep = 20;
    }
    _samplesDone = 0;
    _sumX = _sumY = _sumZ = 0;
    _step = 0;
    _waitUntilMs = 0;
    _waiting = false;
    return inProgressStatus();
  }
  if (_substep == 6U) {
    const Status status = _writeByte(cmd::REG_CTRL5_C, 1U << cmd::BIT_ST_XL, nowMs);
    if (!status.ok()) return routeFailureToRestore(status);
    _waitUntilMs = 0;
    _substep = 7;
    _waiting = true;
    return inProgressStatus();
  }
  if (_substep == 10U) {
    const Status status = _writeByte(cmd::REG_CTRL1_XL, 0, nowMs);
    if (!status.ok()) return routeFailureToRestore(status);
    ++_substep;
    return inProgressStatus();
  }
  if (_substep == 11U) {
    const Status status = _writeByte(cmd::REG_CTRL5_C, 0, nowMs);
    if (!status.ok()) return routeFailureToRestore(status);
    ++_substep;
    return inProgressStatus();
  }
  if (_substep == 12U) {
    DeviceProfile testProfile = _selfTestRestoreProfile;
    testProfile.gyroOdr = Odr::HZ_208;
    testProfile.gyroFullScale = GyroFs::DPS_2000;
    const Status status =
        _writeByte(cmd::REG_CTRL2_G, buildCtrl2(testProfile), nowMs);
    if (!status.ok()) return routeFailureToRestore(status);
    _waitUntilMs = 0;
    _substep = 13;
    _waiting = true;
    return inProgressStatus();
  }
  if (_substep == 16U) {
    const Status status = _writeByte(cmd::REG_CTRL5_C, 1U << cmd::BIT_ST_G, nowMs);
    if (!status.ok()) return routeFailureToRestore(status);
    _waitUntilMs = 0;
    _substep = 17;
    _waiting = true;
    return inProgressStatus();
  }
  if (_substep == 20U) {
    if (_step == 0U) {
      const Status status = _writeByte(cmd::REG_CTRL2_G, 0, nowMs);
      if (!status.ok()) return routeFailureToRestore(status);
      _step = 1;
      return inProgressStatus();
    }
    const Status status = _writeByte(cmd::REG_CTRL5_C, 0, nowMs);
    if (!status.ok()) return routeFailureToRestore(status);
    beginRestore();
    if (!_primaryStatus.ok()) _pollBoundary = true;
    return inProgressStatus();
  }
  if (_substep == 90U) {
    const Status status = _writeByte(cmd::REG_CTRL1_XL, 0, nowMs);
    if (!status.ok()) return routeFailureToRestore(status);
    _substep = 91;
    return inProgressStatus();
  }
  if (_substep == 91U) {
    const Status status = _writeByte(cmd::REG_CTRL5_C, 0, nowMs);
    if (!status.ok()) return routeFailureToRestore(status);
    beginRestore();
    _pollBoundary = true;
    return inProgressStatus();
  }
  if (_substep == 92U) {
    const Status status = _writeByte(cmd::REG_CTRL2_G, 0, nowMs);
    if (!status.ok()) return routeFailureToRestore(status);
    _substep = 93;
    return inProgressStatus();
  }
  if (_substep == 93U) {
    const Status status = _writeByte(cmd::REG_CTRL5_C, 0, nowMs);
    if (!status.ok()) return routeFailureToRestore(status);
    beginRestore();
    _pollBoundary = true;
    return inProgressStatus();
  }
  return routeFailureToRestore(
      Status::Error(Err::INVALID_PARAM, "Invalid self-test step"));
}

Status LSM6DS3TR::_stepCalibration(uint64_t nowMs) {
  const bool accel =
      _calibrationRequest.kind == CalibrationKind::ACCELEROMETER_BIAS;
  if (_step == 2U) {
    const uint64_t periodUs = odrPeriodUs(
        accel ? _verifiedProfile.accelOdr : _verifiedProfile.gyroOdr);
    const uint64_t periodMs = (periodUs + 999U) / 1000U;
    _waitUntilMs = saturatingAdd(
        nowMs, periodMs + CLOCK_QUANTIZATION_MARGIN_MS);
    _step = 0;
    _waiting = true;
    return inProgressStatus();
  }
  if (_step == 0U) {
    if (_waitUntilMs != 0U && nowMs < _waitUntilMs) {
      _waiting = true;
      return inProgressStatus();
    }
    uint8_t statusReg = 0;
    const Status status = _read(cmd::REG_STATUS_REG, &statusReg, 1, nowMs);
    if (!status.ok()) return status;
    if ((statusReg & (accel ? cmd::MASK_XLDA : cmd::MASK_GDA)) == 0U) {
      ++_readyPolls;
      if (_readyPolls >= 3U) {
        return Status::Error(Err::DATA_NOT_READY,
                             "Calibration data was not ready within three checks");
      }
      _step = 2;
      _waitUntilMs = 0;
      _waiting = true;
      return inProgressStatus();
    }
    _readyPolls = 0;
    _waitUntilMs = 0;
    _step = 1;
    return inProgressStatus();
  }
  uint8_t data[6] = {};
  const Status status = _read(accel ? cmd::REG_DATA_START_ACCEL
                                    : cmd::REG_DATA_START_GYRO,
                              data, sizeof(data), nowMs);
  if (!status.ok()) return status;
  const RawAxes raw = decodeRawAxes(data);
  _sumX += raw.x;
  _sumY += raw.y;
  _sumZ += raw.z;
  if (raw.x < _rawMin.x) _rawMin.x = raw.x;
  if (raw.y < _rawMin.y) _rawMin.y = raw.y;
  if (raw.z < _rawMin.z) _rawMin.z = raw.z;
  if (raw.x > _rawMax.x) _rawMax.x = raw.x;
  if (raw.y > _rawMax.y) _rawMax.y = raw.y;
  if (raw.z > _rawMax.z) _rawMax.z = raw.z;
  ++_samplesDone;
  if (_samplesDone < _calibrationRequest.samples) {
    _step = 2;
    _waitUntilMs = 0;
    _waiting = true;
    return inProgressStatus();
  }
  _step = 0;

  CalibrationResult& result = _workingResult.calibration;
  result.kind = _calibrationRequest.kind;
  result.samples = _samplesDone;
  if (accel) {
    int32_t sensitivity = 0;
    const Status sensitivityStatus =
        accelSensitivityMicroGPerLsb(_verifiedProfile.accelFullScale,
                                     sensitivity);
    if (!sensitivityStatus.ok()) return sensitivityStatus;
    const float scale = static_cast<float>(sensitivity) / 1000000.0f;
    const Axes measured = meanRawAxesToFloat(
        _sumX, _sumY, _sumZ, _samplesDone, scale);
    result.bias = subtractAxes(measured, _calibrationRequest.expectedAccelerationG);
    result.peakToPeak = Axes{
        static_cast<float>(static_cast<int32_t>(_rawMax.x) - _rawMin.x) * scale,
        static_cast<float>(static_cast<int32_t>(_rawMax.y) - _rawMin.y) * scale,
        static_cast<float>(static_cast<int32_t>(_rawMax.z) - _rawMin.z) * scale};
    if (result.peakToPeak.x > _calibrationRequest.limits.accelMaxPeakToPeakG ||
        result.peakToPeak.y > _calibrationRequest.limits.accelMaxPeakToPeakG ||
        result.peakToPeak.z > _calibrationRequest.limits.accelMaxPeakToPeakG) {
      return Status::Error(Err::CALIBRATION_UNSTABLE,
                           "Accelerometer calibration is unstable");
    }
    if (!finiteAxes(result.bias) || axesMagnitudeSquared(result.bias) > 0.25f) {
      return Status::Error(Err::CALIBRATION_ORIENTATION,
                           "Fixture vector does not match measured orientation");
    }
  } else {
    int32_t sensitivity = 0;
    const Status sensitivityStatus =
        gyroSensitivityMicroDpsPerLsb(_verifiedProfile.gyroFullScale, sensitivity);
    if (!sensitivityStatus.ok()) return sensitivityStatus;
    const float scale = static_cast<float>(sensitivity) / 1000000.0f;
    result.bias = meanRawAxesToFloat(
        _sumX, _sumY, _sumZ, _samplesDone, scale);
    result.peakToPeak = Axes{
        static_cast<float>(static_cast<int32_t>(_rawMax.x) - _rawMin.x) * scale,
        static_cast<float>(static_cast<int32_t>(_rawMax.y) - _rawMin.y) * scale,
        static_cast<float>(static_cast<int32_t>(_rawMax.z) - _rawMin.z) * scale};
    if (result.peakToPeak.x > _calibrationRequest.limits.gyroMaxPeakToPeakDps ||
        result.peakToPeak.y > _calibrationRequest.limits.gyroMaxPeakToPeakDps ||
        result.peakToPeak.z > _calibrationRequest.limits.gyroMaxPeakToPeakDps) {
      return Status::Error(Err::CALIBRATION_UNSTABLE,
                           "Gyroscope calibration is unstable");
    }
  }
  return _finish(Status::Ok(), OperationState::SUCCEEDED);
}

Status LSM6DS3TR::_stepFifoRead(uint64_t nowMs) {
  FifoReadResult& result = _workingResult.fifoRead;
  if (_step == 0U) {
    uint8_t access = 0;
    const Status status = _read(cmd::REG_FUNC_CFG_ACCESS, &access, 1, nowMs);
    if (!status.ok()) return status;
    if ((access & (cmd::MASK_FUNC_CFG_EN | cmd::MASK_FUNC_CFG_EN_B)) != 0U) {
      _invalidateConfiguration();
      return Status::Error(Err::CONFIGURATION_UNKNOWN, "FIFO read requires the main bank");
    }
    _step = 1;
    return inProgressStatus();
  }
  if (_step == 1U) {
    uint8_t identity = 0;
    const Status status = _read(cmd::REG_WHO_AM_I, &identity, 1, nowMs);
    if (!status.ok()) return status;
    _workingResult.probe = {static_cast<uint8_t>(_driverConfig.address), identity};
    if (identity != cmd::WHO_AM_I_VALUE) {
      _invalidateConfiguration();
      return Status::Error(Err::CHIP_ID_MISMATCH, "WHO_AM_I mismatch", identity);
    }
    _step = 2;
    return inProgressStatus();
  }
  if (_step == 2U || _step == 4U) {
    uint8_t data[4] = {};
    const Status status = _read(cmd::REG_FIFO_STATUS1, data, sizeof(data), nowMs);
    if (!status.ok()) return status;
    result.after = decodeFifoStatus(data, nowMs);
    if (_step == 2U) {
      result.before = result.after;
      _fifoReadTarget = result.before.unreadWords < _fifoReadRequest.maxWords
          ? result.before.unreadWords : _fifoReadRequest.maxWords;
      result.truncated = result.before.unreadWords > _fifoReadTarget;
    }
    result.overrunObserved = result.overrunObserved || result.after.overrun;
    if (_verifiedProfile.fifo.mode == FifoMode::FIFO && result.after.full) {
      // FIFO mode stops collecting at full; subsequent arrivals may be lost
      // even though the retained words remain coherent and cannot overwrite.
      _fifoContinuityLost = true;
    }
    if (!consistentFifoStatus(result.after)) {
      _fifoContinuityLost = true;
      return Status::Error(Err::OPERATION_INDETERMINATE,
                           "FIFO count and EMPTY status disagreed");
    }
    if (result.overrunObserved) {
      _fifoContinuityLost = true;
      return Status::Error(Err::FIFO_OVERRUN, "FIFO overrun prevents a coherent batch");
    }
    if (_step == 4U && _verifiedProfile.fifo.mode == FifoMode::CONTINUOUS &&
        result.after.unreadWords >= fifoOverwriteFloor(_verifiedProfile) - _substep) {
      // AN5130 p79 does not promise sticky OVER_RUN: consumption can remove
      // the full condition before this observation. Engineering inference:
      // any overwrite would fill the queue, and this sole consumer has removed
      // at most _substep words since the preceding status. A lower occupancy
      // therefore proves that no overwrite occurred, without assuming host
      // latency or sensor-clock accuracy. Reject the unprovable near-full case.
      _fifoContinuityLost = true;
      return Status::Error(Err::OPERATION_INDETERMINATE,
                           "FIFO occupancy cannot exclude an intervening overwrite");
    }
    const uint16_t expected = static_cast<uint16_t>(
        (result.before.pattern + result.wordsConsumed) % _fifoPatternWords);
    if (result.after.pattern >= _fifoPatternWords ||
        (_step == 4U && (result.after.pattern != expected ||
         result.after.unreadWords < result.before.unreadWords - result.wordsConsumed))) {
      result.patternDiscontinuity = true;
      _fifoContinuityLost = true;
      return Status::Error(Err::OPERATION_INDETERMINATE, "FIFO pattern/progress changed");
    }
    if (result.wordsConsumed >= _fifoReadTarget || result.after.empty ||
        result.records == _fifoReadRequest.capacity) {
      result.truncated = result.truncated || result.after.unreadWords != 0U;
      return _finish(Status::Ok(), OperationState::SUCCEEDED);
    }
    _step = 3;
    return inProgressStatus();
  }

  // Plan at most 16 words, stopping before an XYZ whose output would not fit.
  // Never consume beyond the initial observed backlog; arrivals remain queued.
  uint16_t words = static_cast<uint16_t>(_fifoReadTarget - result.wordsConsumed);
  if (words > result.after.unreadWords) words = result.after.unreadWords;
  if (words > MAX_TRANSPORT_READ_BYTES / 2U)
    words = static_cast<uint16_t>(MAX_TRANSPORT_READ_BYTES / 2U);
  uint16_t slots = static_cast<uint16_t>(_fifoReadRequest.capacity - result.records);
  uint8_t discardMask = _fifoDiscardMask;
  uint8_t axesCount = _fifoAxesCount;
  for (uint16_t i = 0; i < words; ++i) {
    FifoPatternWord word{};
    const Status schema = fifoPatternWord(_verifiedProfile,
        static_cast<uint16_t>((result.after.pattern + i) % _fifoPatternWords), word);
    if (!schema.ok()) return schema;
    const bool discard = (discardMask & word.quantityMask) != 0U;
    if (word.axis == 0U) {
      if (slots == 0U && !discard) { words = i; break; }
      axesCount = 1;
    } else if (word.axis == axesCount && axesCount != 0U) {
      ++axesCount;
      if (axesCount == 3U) {
        if (discard) discardMask = static_cast<uint8_t>(discardMask & ~word.quantityMask);
        else --slots;
        axesCount = 0;
      }
    }
  }
  if (words == 0U) {
    result.truncated = result.truncated || result.after.unreadWords != 0U;
    return _finish(Status::Ok(), OperationState::SUCCEEDED);
  }
  uint8_t data[MAX_TRANSPORT_READ_BYTES] = {};
  _substep = words;
  const uint32_t previousTransactions = _operationTransactions;
  const Status status = _read(cmd::REG_FIFO_DATA_OUT_L, data, 2U * words, nowMs);
  if (_operationTransactions != previousTransactions) {
    _hardwareStateMayHaveChanged = true;
    result.wordsPossiblyConsumed = static_cast<uint16_t>(result.wordsConsumed + words);
  }
  if (!status.ok()) return status;
  result.wordsConsumed = static_cast<uint16_t>(result.wordsConsumed + words);
  for (uint16_t i = 0; i < words; ++i) {
    FifoPatternWord word{};
    const Status schema = fifoPatternWord(_verifiedProfile,
        static_cast<uint16_t>((result.after.pattern + i) % _fifoPatternWords), word);
    if (!schema.ok()) return schema;
    const int16_t value = static_cast<int16_t>(static_cast<uint16_t>(data[2U * i]) |
        (static_cast<uint16_t>(data[2U * i + 1U]) << 8U));
    if (word.axis == 0U) {
      _fifoAxes = RawAxes{value, 0, 0};
      _fifoAxesCount = 1;
    } else if (word.axis == 1U && _fifoAxesCount == 1U) {
      _fifoAxes.y = value;
      _fifoAxesCount = 2;
    } else if (word.axis == 2U && _fifoAxesCount == 2U) {
      _fifoAxes.z = value;
      _fifoAxesCount = 0;
      if ((_fifoDiscardMask & word.quantityMask) != 0U) {
        _fifoDiscardMask = static_cast<uint8_t>(_fifoDiscardMask & ~word.quantityMask);
      } else {
        FifoMotionRecord& record = _fifoReadRequest.records[result.records++];
        record.raw = _fifoAxes;
        record.readUptimeMs = nowMs;
        record.configGeneration = _configGeneration;
        record.quantityMask = word.quantityMask;
        record.accelFullScale = _verifiedProfile.accelFullScale;
        record.gyroFullScale = _verifiedProfile.gyroFullScale;
        record.kind = word.kind;
        if (word.kind == FifoRecordKind::TEMPERATURE) {
          // AN5130 Table 74: only bytes 2/3 are temperature. The other two
          // words are unused, and a FIFO trigger proves no new conversion.
          record.temperatureRaw = _fifoAxes.y;
          record.raw = {};
        } else if (word.kind == FifoRecordKind::TIMESTAMP_STEP) {
          // AN5130 Table 73, deliberately not contiguous LE24:
          // bytes: ts[15:8],ts[23:16],unused,ts[7:0],stepsL,stepsH.
          const uint16_t first = static_cast<uint16_t>(_fifoAxes.x);
          const uint16_t middle = static_cast<uint16_t>(_fifoAxes.y);
          record.timestampTicks = (static_cast<uint32_t>(first >> 8U) << 16U) |
              (static_cast<uint32_t>(first & 0xFFU) << 8U) | (middle >> 8U);
          record.stepCount = static_cast<uint16_t>(_fifoAxes.z);
          record.timestampResolution = _verifiedProfile.fifo.timestampResolution;
          record.counterEpoch = _counterEpoch;
          record.counterContinuityLost = _counterContinuityLost;
          record.raw = {};
        }
      }
    }
  }
  _step = 4;
  return inProgressStatus();
}

Status LSM6DS3TR::_stepFifoPurge(uint64_t nowMs) {
  auto decodeStatus = [](const uint8_t* data, uint16_t& unread,
                         uint16_t& pattern, bool& overrun, bool& empty) {
    const FifoStatusSnapshot snapshot = decodeFifoStatus(data, 0);
    unread = snapshot.unreadWords;
    pattern = snapshot.pattern;
    overrun = snapshot.overrun;
    empty = snapshot.empty;
  };

  if (_step == 6U) {
    const uint32_t before = _operationTransactions;
    const Status status = _writeByte(cmd::REG_FIFO_CTRL5, 0, nowMs);
    if (_operationTransactions != before) {
      // A fixed drain cannot rule out arrivals between the last observation
      // and Bypass. This explicitly requested flush never claims losslessness.
      _workingResult.fifoPurge.residualLossPossible = true;
      _fifoContinuityLost = true;
      _invalidateConfiguration();
    }
    if (!status.ok()) return status;
    _step = 7;
    return inProgressStatus();
  }
  if (_step == 7U) {
    uint8_t mode = 0;
    const Status status = _read(cmd::REG_FIFO_CTRL5, &mode, 1, nowMs);
    if (!status.ok()) return status;
    if (mode != 0U) {
      _recordMismatch(cmd::REG_FIFO_CTRL5, 0, mode);
      return Status::Error(Err::CONFIGURATION_MISMATCH, "FIFO Bypass verification failed");
    }
    _workingResult.fifoPurge.collectionStopped = true;
    return _finish(_workingResult.fifoPurge.overrunObserved
        ? Status::Error(Err::FIFO_OVERRUN, "FIFO overrun observed while purging")
        : Status::Ok(), _workingResult.fifoPurge.overrunObserved
        ? OperationState::FAILED : OperationState::SUCCEEDED);
  }

  if (_step == 0U) {
    uint8_t access = 0;
    const Status status =
        _read(cmd::REG_FUNC_CFG_ACCESS, &access, 1, nowMs);
    if (!status.ok()) return status;
    if ((access & (cmd::MASK_FUNC_CFG_EN | cmd::MASK_FUNC_CFG_EN_B)) != 0U) {
      _invalidateConfiguration();
      return Status::Error(Err::CONFIGURATION_UNKNOWN,
                           "FIFO purge requires the main register bank");
    }
    _step = 1;
    return inProgressStatus();
  }
  if (_step == 1U) {
    uint8_t whoAmI = 0;
    const Status status = _read(cmd::REG_WHO_AM_I, &whoAmI, 1, nowMs);
    if (!status.ok()) return status;
    _workingResult.probe =
        ProbeResult{static_cast<uint8_t>(_driverConfig.address), whoAmI};
    if (whoAmI != cmd::WHO_AM_I_VALUE) {
      _invalidateConfiguration();
      return Status::Error(Err::CHIP_ID_MISMATCH, "WHO_AM_I mismatch", whoAmI);
    }
    _step = 2;
    return inProgressStatus();
  }
  if (_step == 2U) {
    uint8_t ctrl3 = 0;
    const Status status = _read(cmd::REG_CTRL3_C, &ctrl3, 1, nowMs);
    if (!status.ok()) return status;
    constexpr uint8_t prerequisites = cmd::MASK_IF_INC | cmd::MASK_BDU;
    if ((ctrl3 & prerequisites) != prerequisites) {
      _invalidateConfiguration();
      return Status::Error(Err::CONFIGURATION_UNKNOWN,
                           "FIFO purge requires verified IF_INC and BDU");
    }
    _step = 3;
    return inProgressStatus();
  }
  if (_step == 3U) {
    uint8_t data[4] = {};
    const Status status = _read(cmd::REG_FIFO_STATUS1, data, sizeof(data), nowMs);
    if (!status.ok()) return status;
    uint16_t unread = 0;
    uint16_t pattern = 0;
    bool overrun = false;
    bool empty = false;
    decodeStatus(data, unread, pattern, overrun, empty);
    const bool statusConsistent = empty == (unread == 0U);
    if (overrun || !statusConsistent) _fifoContinuityLost = true;
    _workingResult.fifoPurge.initialUnreadWords = unread;
    _workingResult.fifoPurge.initialPattern = pattern;
    _workingResult.fifoPurge.overrunObserved = overrun;
    _workingResult.fifoPurge.truncated =
        unread > _fifoPurgeRequest.maxWords || !statusConsistent;
    if (!statusConsistent) {
      return Status::Error(Err::OPERATION_INDETERMINATE,
                           "FIFO count and EMPTY status disagreed");
    }
    _step = 4;
    if (unread == 0U) _step = 5;
    return inProgressStatus();
  }
  if (_step == 4U &&
      _workingResult.fifoPurge.wordsDiscarded <
          _workingResult.fifoPurge.initialUnreadWords &&
      _workingResult.fifoPurge.wordsDiscarded < _fifoPurgeRequest.maxWords) {
    uint8_t discarded[2] = {};
    const uint32_t before = _operationTransactions;
    const Status status =
        _read(cmd::REG_FIFO_DATA_OUT_L, discarded, sizeof(discarded), nowMs);
    if (_operationTransactions != before) {
      _hardwareStateMayHaveChanged = true;
      _fifoContinuityLost = true;
      _workingResult.fifoPurge.wordsPossiblyDiscarded =
          static_cast<uint16_t>(_workingResult.fifoPurge.wordsDiscarded + 1U);
    }
    if (!status.ok()) return status;
    ++_workingResult.fifoPurge.wordsDiscarded;
    if (_workingResult.fifoPurge.wordsDiscarded <
            _workingResult.fifoPurge.initialUnreadWords &&
        _workingResult.fifoPurge.wordsDiscarded < _fifoPurgeRequest.maxWords) {
      return inProgressStatus();
    }
    _step = 5;
    return inProgressStatus();
  }
  uint8_t data[4] = {};
  const Status status = _read(cmd::REG_FIFO_STATUS1, data, sizeof(data), nowMs);
  if (!status.ok()) return status;
  uint16_t pattern = 0;
  bool overrun = false;
  bool empty = false;
  decodeStatus(data, _workingResult.fifoPurge.finalUnreadWords, pattern, overrun,
               empty);
  const bool statusConsistent =
      empty == (_workingResult.fifoPurge.finalUnreadWords == 0U);
  if (overrun || !statusConsistent) _fifoContinuityLost = true;
  _workingResult.fifoPurge.overrunObserved =
      _workingResult.fifoPurge.overrunObserved || overrun;
  _workingResult.fifoPurge.truncated =
      _workingResult.fifoPurge.truncated ||
      _workingResult.fifoPurge.finalUnreadWords != 0U || !statusConsistent;
  if (_fifoPurgeRequest.stopCollection && statusConsistent) {
    _step = 6;
    return inProgressStatus();
  }
  if (_workingResult.fifoPurge.overrunObserved) {
    return _finish(Status::Error(Err::FIFO_OVERRUN,
                                 "FIFO overrun observed while purging"),
                   OperationState::FAILED);
  }
  if (!statusConsistent) {
    return _finish(Status::Error(Err::OPERATION_INDETERMINATE,
                                 "FIFO count and EMPTY status disagreed"),
                   _hardwareStateMayHaveChanged
                       ? OperationState::INDETERMINATE
                       : OperationState::FAILED);
  }
  return _finish(Status::Ok(), OperationState::SUCCEEDED);
}

Status LSM6DS3TR::_finish(const Status& status, OperationState state) {
  if (_job == JobKind::FIFO_READ) {
    FifoReadResult& result = _workingResult.fifoRead;
    if (!status.ok()) {
      _clearFifoOutput();
      result.records = 0;
      _fifoContinuityLost = true;
    } else {
      for (uint16_t i = 0; i < result.records; ++i)
        _fifoReadRequest.records[i].token = _token;
    }
    result.wordsDiscarded = static_cast<uint16_t>(result.wordsConsumed - 3U * result.records);
    if (result.wordsDiscarded != 0U) {
      _fifoContinuityLost = true;
      result.truncated = true;
    }
    const bool newLoss = !status.ok() || result.wordsDiscarded != 0U ||
                         (_fifoContinuityLost && !result.continuityLost);
    result.continuityLost = _fifoContinuityLost;
    if (newLoss) {
      _breakCounterContinuity();
      // Publication follows all post-status and terminal evidence. Every
      // counter record in the batch reports the same final interval evidence.
      for (uint16_t i = 0; i < result.records; ++i) {
        if (_fifoReadRequest.records[i].kind == FifoRecordKind::TIMESTAMP_STEP) {
          _fifoReadRequest.records[i].counterEpoch = _counterEpoch;
          _fifoReadRequest.records[i].counterContinuityLost = true;
        }
      }
    }
  }
  if (_job == JobKind::FIFO_PURGE) _breakCounterContinuity();
  if (_configurationState == ConfigurationState::SETTLING &&
      _pollNowMs >= _validAfterUptimeMs) {
    _configurationState = ConfigurationState::KNOWN;
  }
  _workingResult.status = status;
  _workingResult.state = state;
  _workingResult.hardwareStateMayHaveChanged = _hardwareStateMayHaveChanged;
  _workingResult.transactions = _operationTransactions;
  _workingResult.transactionLimit = _operationTransactionLimit;
  _workingResult.completedUptimeMs = _pollNowMs;
  _workingResult.configuration.state = configurationState(_pollNowMs);
  _workingResult.configuration.generation = _configGeneration;
  _workingResult.configuration.validAfterUptimeMs = _validAfterUptimeMs;
  _workingResult.counters.epoch = _counterEpoch;
  _workingResult.counters.continuityLost = _counterContinuityLost;
  _workingResult.registerBankMayBeUnknown = _bankMayBeUnknown;
  _terminalResult = _workingResult;
  _resultPending = true;
  _clearActive();
  return status;
}

Status LSM6DS3TR::_fail(const Status& status) {
  if (_job == JobKind::SELF_TEST && _substep == 100U) {
    _workingResult.selfTest.primaryStatus = _primaryStatus;
    _workingResult.selfTest.restorationStatus = status;
  }
  if (_configurationMayBeUnknown) {
    _invalidateConfiguration();
  } else if (_configurationState == ConfigurationState::APPLYING) {
    _configurationState = _configurationStateBeforeOperation;
    _validAfterUptimeMs = _validAfterBeforeOperationMs;
  }
  const bool determinateMismatch = status.code == Err::CONFIGURATION_MISMATCH;
  const bool indeterminate = !determinateMismatch && _hardwareStateMayHaveChanged &&
                             (_job != JobKind::SAMPLE &&
                              _job != JobKind::CALIBRATION &&
                              _job != JobKind::PROBE);
  Status terminalStatus = status;
  if (_job == JobKind::SELF_TEST && !_primaryStatus.ok()) {
    terminalStatus = _primaryStatus;
  }
  return _finish(terminalStatus, indeterminate ? OperationState::INDETERMINATE
                                               : OperationState::FAILED);
}

void LSM6DS3TR::_clearActive() {
  _active = false;
  _token = {};
  _job = JobKind::NONE;
  _step = 0;
  _substep = 0;
  _deadlineMs = 0;
  _waitUntilMs = 0;
  _configurationMayBeUnknown = false;
}

PollResult LSM6DS3TR::_pollOne(uint64_t nowMs) {
  Status status = Status::Error(Err::INVALID_PARAM, "Unknown operation");
  switch (_job) {
    case JobKind::PROBE: status = _stepProbe(nowMs); break;
    case JobKind::CONFIGURE: status = _stepConfigure(nowMs, false); break;
    case JobKind::SAMPLE: status = _stepSample(nowMs); break;
    case JobKind::RESET: status = _stepResetBoot(nowMs, false, false); break;
    case JobKind::BOOT: status = _stepResetBoot(nowMs, true, false); break;
    case JobKind::RECOVER: status = _stepResetBoot(nowMs, false, true); break;
    case JobKind::RECONCILE: status = _stepConfigure(nowMs, true); break;
    case JobKind::POWER_DOWN: status = _stepPowerDown(nowMs); break;
    case JobKind::SELF_TEST: status = _stepSelfTest(nowMs); break;
    case JobKind::CALIBRATION: status = _stepCalibration(nowMs); break;
    case JobKind::FIFO_PURGE: status = _stepFifoPurge(nowMs); break;
    case JobKind::FIFO_READ: status = _stepFifoRead(nowMs); break;
    case JobKind::COUNTER_RESET: status = _stepCounterReset(nowMs); break;
    case JobKind::NONE: break;
  }
  if (_active && !status.ok() && !status.inProgress()) _fail(status);
  PollResult result{};
  if (_resultPending && !_active) {
    result.token = _terminalResult.token;
    result.kind = _terminalResult.kind;
    result.state = _terminalResult.state;
    result.status = _terminalResult.status;
    result.transactions = static_cast<uint16_t>(_terminalResult.transactions);
    result.transactionLimit =
        static_cast<uint16_t>(_terminalResult.transactionLimit);
  } else {
    result.token = _token;
    result.kind = _job;
    result.state = _active ? OperationState::ACTIVE : OperationState::IDLE;
    result.status = _active ? inProgressStatus() : Status::Ok();
    if (_active && _job == JobKind::SELF_TEST && _substep == 100U &&
        !_primaryStatus.ok()) {
      result.status = _primaryStatus;
    }
    result.transactions = static_cast<uint16_t>(_operationTransactions);
    result.transactionLimit = static_cast<uint16_t>(_operationTransactionLimit);
  }
  result.waiting = _waiting;
  return result;
}

PollResult LSM6DS3TR::poll(uint64_t nowMs, uint8_t maxTransactions) {
  _pollNowMs = nowMs;
  if (!_bound) {
    PollResult result{};
    result.status = Status::Error(Err::NOT_BOUND, "Driver is not bound");
    return result;
  }
  if (!_active) {
    if (_resultPending) {
      PollResult result{};
      result.token = _terminalResult.token;
      result.kind = _terminalResult.kind;
      result.state = _terminalResult.state;
      result.status = _terminalResult.status;
      result.transactions = static_cast<uint16_t>(_terminalResult.transactions);
      result.transactionLimit =
          static_cast<uint16_t>(_terminalResult.transactionLimit);
      return result;
    }
    return PollResult{};
  }
  if (nowMs >= _deadlineMs) {
    const bool selfTestUntouched =
        _job == JobKind::SELF_TEST && !_hardwareStateMayHaveChanged;
    const bool selfTestRestored =
        _job == JobKind::SELF_TEST && _substep == 100U &&
        _step >= CONFIGURE_SETTLE_WAIT && _hasVerifiedProfile &&
        !_configurationMayBeUnknown;
    if (_configurationMayBeUnknown) {
      _invalidateConfiguration();
    } else if (_configurationState == ConfigurationState::APPLYING) {
      _configurationState = _configurationStateBeforeOperation;
      _validAfterUptimeMs = _validAfterBeforeOperationMs;
    }
    if (_job == JobKind::SELF_TEST) {
      if (_primaryStatus.ok()) {
        _primaryStatus =
            Status::Error(Err::DEADLINE_EXPIRED, "Operation deadline expired");
      }
      _workingResult.selfTest.primaryStatus = _primaryStatus;
      if (selfTestUntouched) {
        _workingResult.selfTest.restorationStatus = Status::Ok();
      } else if (selfTestRestored) {
        _workingResult.selfTest.restorationStatus =
            configurationState(nowMs) == ConfigurationState::SETTLING
                ? Status::Error(Err::SETTLING,
                                "Restored profile is still settling")
                : Status::Ok();
      } else {
        _workingResult.selfTest.restorationStatus =
            Status::Error(Err::OPERATION_INDETERMINATE,
                          "Restoration not completed before deadline");
      }
    }
    _finish(Status::Error(Err::DEADLINE_EXPIRED, "Operation deadline expired"),
            OperationState::TIMED_OUT);
    PollResult result{};
    result.token = _terminalResult.token;
    result.kind = _terminalResult.kind;
    result.state = _terminalResult.state;
    result.status = _terminalResult.status;
    result.transactions = static_cast<uint16_t>(_terminalResult.transactions);
    result.transactionLimit =
        static_cast<uint16_t>(_terminalResult.transactionLimit);
    return result;
  }
  if (maxTransactions == 0U) {
    _transactionUsed = false;
    _waiting = false;
    const DeviceProfile& target = _job == JobKind::SELF_TEST
                                      ? _selfTestRestoreProfile : _desiredProfile;
    const bool counterWorkPending = _job != JobKind::RECONCILE &&
        _counterSetupStep < 19U && (counterProfile(target) || _timestampResetNeeded);
    const bool configurationCompute =
        (_step == CONFIGURE_SETTLE_ARM && !counterWorkPending) ||
        _step == CONFIGURE_SETTLE_WAIT || _step == CONFIGURE_COMMIT;
    bool safeComputeStep = false;
    if ((_job == JobKind::CONFIGURE || _job == JobKind::RECONCILE) &&
        configurationCompute) {
      safeComputeStep = true;
    } else if ((_job == JobKind::RESET || _job == JobKind::BOOT ||
                _job == JobKind::RECOVER) &&
               _substep == 8U && configurationCompute) {
      safeComputeStep = true;
    } else if ((_job == JobKind::SAMPLE ||
                _job == JobKind::CALIBRATION) &&
               _step == 2U) {
      safeComputeStep = true;
    } else if (_job == JobKind::SELF_TEST && _step == 2U &&
               (_substep == 4U || _substep == 5U || _substep == 8U ||
                _substep == 9U || _substep == 14U || _substep == 15U ||
                _substep == 18U || _substep == 19U)) {
      safeComputeStep = true;
    } else if (_job == JobKind::SELF_TEST &&
               ((_substep == 3U || _substep == 7U || _substep == 13U ||
                  _substep == 17U) ||
                 (_substep == 100U && configurationCompute))) {
      safeComputeStep = true;
    } else if ((_job == JobKind::RESET || _job == JobKind::BOOT ||
                _job == JobKind::RECOVER) &&
               _substep == 7U && _waitUntilMs == 0U) {
      safeComputeStep = true;
    }
    if (safeComputeStep) {
      PollResult result = _pollOne(nowMs);
      result.transactionsUsed = 0;
      return result;
    }
    if ((_job == JobKind::RESET || _job == JobKind::BOOT ||
         _job == JobKind::RECOVER) &&
        _substep == 7U) {
      _waiting = nowMs < _waitUntilMs;
    } else if ((_job == JobKind::SAMPLE || _job == JobKind::SELF_TEST ||
                _job == JobKind::CALIBRATION) &&
               _waitUntilMs != 0U) {
      _waiting = nowMs < _waitUntilMs;
    }
    PollResult result{};
    result.token = _token;
    result.kind = _job;
    result.state = OperationState::ACTIVE;
    result.status = (_job == JobKind::SELF_TEST && _substep == 100U &&
                     !_primaryStatus.ok())
                        ? _primaryStatus
                        : inProgressStatus();
    result.transactions = static_cast<uint16_t>(_operationTransactions);
    result.transactionLimit = static_cast<uint16_t>(_operationTransactionLimit);
    result.waiting = _waiting;
    return result;
  }

  uint8_t used = 0;
  PollResult result{};
  for (uint8_t iteration = 0; iteration < 64U && _active; ++iteration) {
    if (used >= maxTransactions) break;
    _transactionUsed = false;
    _waiting = false;
    _pollBoundary = false;
    result = _pollOne(nowMs);
    if (_transactionUsed) {
      ++used;
      if (used >= maxTransactions) break;
    }
    if (!_active || _waiting || _pollBoundary) break;
  }
  if (_active) {
    result.token = _token;
    result.kind = _job;
    result.state = OperationState::ACTIVE;
    result.status = (_job == JobKind::SELF_TEST && _substep == 100U &&
                     !_primaryStatus.ok())
                        ? _primaryStatus
                        : inProgressStatus();
    result.transactions = static_cast<uint16_t>(_operationTransactions);
    result.transactionLimit = static_cast<uint16_t>(_operationTransactionLimit);
    result.waiting = _waiting;
  } else if (_resultPending) {
    result.token = _terminalResult.token;
    result.kind = _terminalResult.kind;
    result.state = _terminalResult.state;
    result.status = _terminalResult.status;
    result.transactions = static_cast<uint16_t>(_terminalResult.transactions);
    result.transactionLimit =
        static_cast<uint16_t>(_terminalResult.transactionLimit);
    result.waiting = false;
  }
  result.transactionsUsed = used;
  return result;
}

Status LSM6DS3TR::cancelActiveJob(uint64_t nowMs) {
  if (!_bound) return Status::Error(Err::NOT_BOUND, "Driver is not bound");
  if (!_active) return Status::Error(Err::RESULT_NOT_AVAILABLE, "No active operation");
  _pollNowMs = nowMs;
  const bool selfTestUntouched =
      _job == JobKind::SELF_TEST && !_hardwareStateMayHaveChanged;
  const bool selfTestRestored =
      _job == JobKind::SELF_TEST && _substep == 100U &&
      _step >= CONFIGURE_SETTLE_WAIT && _hasVerifiedProfile &&
      !_configurationMayBeUnknown;
  if (_configurationMayBeUnknown) {
    _invalidateConfiguration();
  } else if (_configurationState == ConfigurationState::APPLYING) {
    _configurationState = _configurationStateBeforeOperation;
    _validAfterUptimeMs = _validAfterBeforeOperationMs;
  }
  if (_job == JobKind::SELF_TEST) {
    if (_primaryStatus.ok()) {
      _primaryStatus = Status::Error(Err::CANCELLED, "Operation cancelled");
    }
    _workingResult.selfTest.primaryStatus = _primaryStatus;
    if (selfTestUntouched) {
      _workingResult.selfTest.restorationStatus = Status::Ok();
    } else if (selfTestRestored) {
      _workingResult.selfTest.restorationStatus =
          configurationState(nowMs) == ConfigurationState::SETTLING
              ? Status::Error(Err::SETTLING,
                              "Restored profile is still settling")
              : Status::Ok();
    } else {
      _workingResult.selfTest.restorationStatus =
          Status::Error(Err::OPERATION_INDETERMINATE,
                        "Restoration not completed before cancellation");
    }
  }
  (void)_finish(Status::Error(Err::CANCELLED, "Operation cancelled"),
                OperationState::CANCELLED);
  return Status::Ok();
}

Status LSM6DS3TR::takeResult(OperationToken token, OperationResult& out) {
  if (!_resultPending) {
    return Status::Error(Err::RESULT_NOT_AVAILABLE, "No terminal result is pending");
  }
  if (!token.valid() || token != _terminalResult.token) {
    return Status::Error(Err::STALE_RESULT, "Operation token does not match");
  }
  out = _terminalResult;
  if (_terminalResult.kind == JobKind::FIFO_READ) _fifoReadRequest = {};
  _terminalResult = {};
  _resultPending = false;
  return Status::Ok();
}

ConfigurationState LSM6DS3TR::configurationState(uint64_t nowMs) const {
  if (_configurationState == ConfigurationState::SETTLING &&
      nowMs >= _validAfterUptimeMs) {
    return ConfigurationState::KNOWN;
  }
  return _configurationState;
}

Status LSM6DS3TR::getDesiredProfile(DeviceProfile& out) const {
  if (!_bound) return Status::Error(Err::NOT_BOUND, "Driver is not bound");
  if (!_hasDesiredProfile) {
    return Status::Error(Err::CONFIGURATION_UNKNOWN, "No desired profile");
  }
  out = _desiredProfile;
  return Status::Ok();
}

Status LSM6DS3TR::getVerifiedProfile(DeviceProfile& out, uint64_t nowMs) const {
  if (!_bound) return Status::Error(Err::NOT_BOUND, "Driver is not bound");
  if (!_hasVerifiedProfile || configurationState(nowMs) != ConfigurationState::KNOWN) {
    return Status::Error(configurationState(nowMs) == ConfigurationState::SETTLING
                             ? Err::SETTLING
                             : Err::CONFIGURATION_UNKNOWN,
                         "Verified profile is unavailable");
  }
  out = _verifiedProfile;
  return Status::Ok();
}

DriverDiagnostics LSM6DS3TR::diagnostics(uint64_t nowMs) const {
  DriverDiagnostics result{};
  result.transportSuccesses = _transportSuccesses;
  result.transportFailures = _transportFailures;
  result.lastTransportError = _lastTransportError;
  result.lastTransportErrorUptimeMs = _lastTransportErrorUptimeMs;
  result.configGeneration = _configGeneration;
  result.configurationState = configurationState(nowMs);
  result.validAfterUptimeMs = _validAfterUptimeMs;
  result.mismatchRegister = _mismatchRegister;
  result.mismatchExpected = _mismatchExpected;
  result.mismatchObserved = _mismatchObserved;
  return result;
}

bool LSM6DS3TR::_validDiagnosticRange(uint8_t startReg, size_t length) {
  if (length == 0U || length > MAX_TRANSPORT_READ_BYTES) return false;
  const uint16_t end = static_cast<uint16_t>(startReg) +
                       static_cast<uint16_t>(length - 1U);
  return end <= cmd::REG_Z_OFS_USR;
}

bool LSM6DS3TR::_safeDiagnosticWrite(uint8_t reg, uint8_t value) {
  if (reg == cmd::REG_TIMESTAMP2) return value == cmd::TIMESTAMP_RESET_VALUE;
  uint8_t writableMask = 0;
  if (!diagnosticWritableMask(reg, writableMask) ||
      (value & static_cast<uint8_t>(~writableMask)) != 0U) {
    return false;
  }
  if (reg == cmd::REG_FUNC_CFG_ACCESS &&
      (value & (cmd::MASK_FUNC_CFG_EN | cmd::MASK_FUNC_CFG_EN_B)) != 0U)
    return false;
  if (reg == cmd::REG_FIFO_CTRL5) {
    const uint8_t mode = value & cmd::MASK_FIFO_MODE;
    const uint8_t odr = static_cast<uint8_t>(
        (value & cmd::MASK_ODR_FIFO) >> cmd::BIT_ODR_FIFO);
    if (mode == 2U || mode == 5U || mode == 7U || odr > 10U) return false;
  }
  if (reg == cmd::REG_CTRL1_XL &&
      ((value & cmd::MASK_ODR_XL) >> cmd::BIT_ODR_XL) > 11U)
    return false;
  if (reg == cmd::REG_CTRL2_G) {
    const uint8_t odr = static_cast<uint8_t>(
        (value & cmd::MASK_ODR_G) >> cmd::BIT_ODR_G);
    if (odr > 10U ||
        ((value & cmd::MASK_FS_125) != 0U &&
         (value & cmd::MASK_FS_G) != 0U))
      return false;
  }
  if (reg == cmd::REG_CTRL5_C) {
    const uint8_t gyroSelfTest = static_cast<uint8_t>(
        (value & cmd::MASK_ST_G) >> cmd::BIT_ST_G);
    if ((value & cmd::MASK_ST_XL) == cmd::MASK_ST_XL ||
        gyroSelfTest == 2U)
      return false;
  }
  if (reg == cmd::REG_CTRL6_C) {
    constexpr uint8_t TRIGGER_MODE_MASK = 0xE0U;
    const uint8_t trigger = static_cast<uint8_t>(value & TRIGGER_MODE_MASK);
    if (trigger != 0x00U && trigger != 0x40U && trigger != 0x60U &&
        trigger != 0x80U && trigger != 0xC0U)
      return false;
  }
  if (reg == cmd::REG_CTRL3_C &&
      ((value & (cmd::MASK_BOOT | cmd::MASK_SW_RESET | cmd::MASK_BLE)) != 0U ||
       (value & cmd::MASK_IF_INC) == 0U))
    return false;
  if (reg == cmd::REG_CTRL4_C && (value & cmd::MASK_I2C_DISABLE) != 0U)
    return false;
  if ((reg == cmd::REG_X_OFS_USR || reg == cmd::REG_Y_OFS_USR ||
       reg == cmd::REG_Z_OFS_USR) && value == 0x80U)
    return false;
  return true;
}

Status LSM6DS3TR::diagnosticReadRegister(uint8_t reg, uint8_t& value,
                                        uint64_t nowMs) {
  return diagnosticReadBlock(reg, &value, 1, nowMs);
}

Status LSM6DS3TR::diagnosticReadBlock(uint8_t startReg, uint8_t* data,
                                     size_t length, uint64_t nowMs) {
  if (!_bound) return Status::Error(Err::NOT_BOUND, "Driver is not bound");
  if (_active) return Status::Error(Err::BUSY, "Operation is active");
  if (_resultPending)
    return Status::Error(Err::RESULT_PENDING, "Terminal result must be taken");
  if (data == nullptr || !_validDiagnosticRange(startReg, length))
    return Status::Error(Err::INVALID_PARAM, "Invalid diagnostic read range");
  if (_bankMayBeUnknown && !(startReg == cmd::REG_FUNC_CFG_ACCESS && length == 1U))
    return Status::Error(Err::CONFIGURATION_UNKNOWN, "Diagnostic access requires proven main bank");
  _operationTransactions = 0;
  _operationTransactionLimit = 1;
  _transactionUsed = false;
  if (startReg <= cmd::REG_FIFO_DATA_OUT_H &&
      static_cast<size_t>(startReg) + length > cmd::REG_FIFO_DATA_OUT_L) {
    _fifoContinuityLost = true;
    _breakCounterContinuity();
  }
  return _read(startReg, data, length, nowMs);
}

Status LSM6DS3TR::diagnosticWriteRegister(uint8_t reg, uint8_t value,
                                         uint64_t nowMs) {
  if (!_bound) return Status::Error(Err::NOT_BOUND, "Driver is not bound");
  if (_active) return Status::Error(Err::BUSY, "Operation is active");
  if (_resultPending)
    return Status::Error(Err::RESULT_PENDING, "Terminal result must be taken");
  if (!_safeDiagnosticWrite(reg, value))
    return Status::Error(Err::INVALID_PARAM, "Unsafe or read-only diagnostic write");
  if (_bankMayBeUnknown && !(reg == cmd::REG_FUNC_CFG_ACCESS && value == 0U))
    return Status::Error(Err::CONFIGURATION_UNKNOWN, "Diagnostic access requires proven main bank");
  _operationTransactions = 0;
  _operationTransactionLimit = 1;
  _transactionUsed = false;
  _hardwareStateMayHaveChanged = true;
  _configurationMayBeUnknown = true;
  _invalidateConfiguration();
  return _writeByte(reg, value, nowMs);
}

}  // namespace LSM6DS3TR
