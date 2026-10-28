/**
 * @file ProfileCli.h
 * @brief Framework-neutral typed profile helpers for the CLI examples.
 *
 * This example-only helper keeps the Arduino and native ESP-IDF command
 * surfaces identical. It performs no I2C and commits a draft change only when
 * the complete resulting profile passes the production driver's validation.
 * It covers every DeviceProfile field: supported ODR/range/power/filter/sleep
 * and offset values, inactive gyro HPF mode bits, and the invariant fields
 * that production validation deliberately keeps at BDU-on,
 * and unsupported-filter-off values.
 *
 * @note Boolean grammar is 0/1, off/on, or false/true. Parsing an unsupported
 *       invariant value succeeds syntactically but complete-profile validation
 *       rejects it without changing the caller's draft.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "LSM6DS3TR/LSM6DS3TR.h"

namespace profile_cli {

using namespace LSM6DS3TR;

/// Add in-operation profile settling to the bus-work/scheduling allowance.
inline uint32_t settleBudgetMs(const DeviceProfile& profile, uint32_t fixedMs) {
  const uint64_t settleMs = (requiredSettleUs(profile) + 999U) / 1000U;
  const uint64_t totalMs = settleMs + fixedMs;
  return totalMs > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(totalMs);
}

inline const char* boolName(bool value) { return value ? "on" : "off"; }

inline const char* odrName(Odr value) {
  switch (value) {
    case Odr::POWER_DOWN: return "pd";
    case Odr::HZ_1_6: return "1.6";
    case Odr::HZ_12_5: return "12.5";
    case Odr::HZ_26: return "26";
    case Odr::HZ_52: return "52";
    case Odr::HZ_104: return "104";
    case Odr::HZ_208: return "208";
    case Odr::HZ_416: return "416";
    case Odr::HZ_833: return "833";
    case Odr::HZ_1660: return "1660";
    case Odr::HZ_3330: return "3330";
    case Odr::HZ_6660: return "6660";
    default: return "invalid";
  }
}

inline const char* accelFsName(AccelFs value) {
  switch (value) {
    case AccelFs::G_2: return "2";
    case AccelFs::G_4: return "4";
    case AccelFs::G_8: return "8";
    case AccelFs::G_16: return "16";
    default: return "invalid";
  }
}

inline const char* gyroFsName(GyroFs value) {
  switch (value) {
    case GyroFs::DPS_125: return "125";
    case GyroFs::DPS_250: return "250";
    case GyroFs::DPS_500: return "500";
    case GyroFs::DPS_1000: return "1000";
    case GyroFs::DPS_2000: return "2000";
    default: return "invalid";
  }
}

inline const char* accelPowerName(AccelPowerMode value) {
  switch (value) {
    case AccelPowerMode::HIGH_PERFORMANCE: return "hp";
    case AccelPowerMode::LOW_POWER_NORMAL: return "lp";
    default: return "invalid";
  }
}

inline const char* gyroPowerName(GyroPowerMode value) {
  switch (value) {
    case GyroPowerMode::HIGH_PERFORMANCE: return "hp";
    case GyroPowerMode::LOW_POWER_NORMAL: return "lp";
    default: return "invalid";
  }
}

inline const char* gyroHpfModeName(GyroHpfMode value) {
  switch (value) {
    case GyroHpfMode::HZ_0_016: return "0.016";
    case GyroHpfMode::HZ_0_065: return "0.065";
    case GyroHpfMode::HZ_0_260: return "0.260";
    case GyroHpfMode::HZ_1_040: return "1.040";
    default: return "invalid";
  }
}

inline const char* offsetWeightName(AccelOffsetWeight value) {
  switch (value) {
    case AccelOffsetWeight::MG_1: return "1";
    case AccelOffsetWeight::MG_16: return "16";
    default: return "invalid";
  }
}

inline const char* fifoModeName(FifoMode value) {
  switch (value) {
    case FifoMode::BYPASS: return "bypass";
    case FifoMode::FIFO: return "fifo";
    case FifoMode::CONTINUOUS: return "continuous";
    default: return "invalid";
  }
}

inline const char* fifoDecimationName(FifoDecimation value) {
  switch (value) {
    case FifoDecimation::OFF: return "off";
    case FifoDecimation::EVERY_1: return "1";
    case FifoDecimation::EVERY_2: return "2";
    case FifoDecimation::EVERY_3: return "3";
    case FifoDecimation::EVERY_4: return "4";
    case FifoDecimation::EVERY_8: return "8";
    case FifoDecimation::EVERY_16: return "16";
    case FifoDecimation::EVERY_32: return "32";
    default: return "invalid";
  }
}

inline const char* fifoFourthDatasetName(FifoFourthDataset value) {
  switch (value) {
    case FifoFourthDataset::OFF: return "off";
    case FifoFourthDataset::TEMPERATURE: return "temperature";
    case FifoFourthDataset::TIMESTAMP_STEP: return "timestamp_step";
    default: return "invalid";
  }
}

inline const char* fifoTriggerName(FifoTrigger value) {
  switch (value) {
    case FifoTrigger::INTERNAL: return "internal";
    case FifoTrigger::STEP_EVENT: return "step";
    default: return "invalid";
  }
}

inline const char* timestampResolutionName(TimestampResolution value) {
  switch (value) {
    case TimestampResolution::MS_6_4: return "6.4ms";
    case TimestampResolution::US_25: return "25us";
    default: return "invalid";
  }
}

inline const char* fifoRecordKindName(FifoRecordKind value) {
  switch (value) {
    case FifoRecordKind::ANGULAR_RATE: return "gyro";
    case FifoRecordKind::ACCELERATION: return "accel";
    case FifoRecordKind::TEMPERATURE: return "temperature";
    case FifoRecordKind::TIMESTAMP_STEP: return "timestamp_step";
    default: return "invalid";
  }
}

inline const char* interruptPolarityName(InterruptPolarity value) {
  switch (value) {
    case InterruptPolarity::ACTIVE_HIGH: return "high";
    case InterruptPolarity::ACTIVE_LOW: return "low";
    default: return "invalid";
  }
}

inline const char* interruptDriveName(InterruptDrive value) {
  switch (value) {
    case InterruptDrive::PUSH_PULL: return "push_pull";
    case InterruptDrive::OPEN_DRAIN_PAD: return "open_drain";
    default: return "invalid";
  }
}

inline const char* dataReadyModeName(DataReadyMode value) {
  switch (value) {
    case DataReadyMode::LATCHED: return "latched";
    case DataReadyMode::PULSED: return "pulsed";
    default: return "invalid";
  }
}

inline const char* orientationModeName(OrientationMode value) {
  switch (value) {
    case OrientationMode::OFF: return "off";
    case OrientationMode::SIX_D: return "6d";
    case OrientationMode::FOUR_D: return "4d";
    default: return "invalid";
  }
}

inline const char* orientationThresholdName(OrientationThreshold value) {
  switch (value) {
    case OrientationThreshold::DEG_80: return "80";
    case OrientationThreshold::DEG_70: return "70";
    case OrientationThreshold::DEG_60: return "60";
    case OrientationThreshold::DEG_50: return "50";
    default: return "invalid";
  }
}

inline bool equalRoutes(const InterruptRoutes& lhs, const InterruptRoutes& rhs) {
  return lhs.accelReady == rhs.accelReady && lhs.gyroReady == rhs.gyroReady &&
         lhs.temperatureReady == rhs.temperatureReady &&
         lhs.fifoWatermark == rhs.fifoWatermark && lhs.fifoFull == rhs.fifoFull &&
         lhs.fifoOverrun == rhs.fifoOverrun && lhs.stepDetected == rhs.stepDetected &&
         lhs.stepOverflow == rhs.stepOverflow &&
         lhs.timestampEndCounter == rhs.timestampEndCounter &&
         lhs.orientation == rhs.orientation;
}

inline bool equal(const DeviceProfile& lhs, const DeviceProfile& rhs) {
  return lhs.accelOdr == rhs.accelOdr &&
         lhs.accelFullScale == rhs.accelFullScale &&
         lhs.accelPowerMode == rhs.accelPowerMode &&
         lhs.gyroOdr == rhs.gyroOdr &&
         lhs.gyroFullScale == rhs.gyroFullScale &&
         lhs.gyroPowerMode == rhs.gyroPowerMode &&
         lhs.accelFilter.lpf2Enabled == rhs.accelFilter.lpf2Enabled &&
         lhs.accelFilter.highPassSlopeEnabled == rhs.accelFilter.highPassSlopeEnabled &&
         lhs.accelFilter.lowPassOn6d == rhs.accelFilter.lowPassOn6d &&
         lhs.gyroFilter.lpf1Enabled == rhs.gyroFilter.lpf1Enabled &&
         lhs.gyroFilter.highPassEnabled == rhs.gyroFilter.highPassEnabled &&
         lhs.gyroFilter.highPassMode == rhs.gyroFilter.highPassMode &&
         lhs.gyroSleepEnabled == rhs.gyroSleepEnabled &&
         lhs.blockDataUpdate == rhs.blockDataUpdate &&
         lhs.accelOffsetWeight == rhs.accelOffsetWeight &&
         lhs.accelUserOffset.x == rhs.accelUserOffset.x &&
         lhs.accelUserOffset.y == rhs.accelUserOffset.y &&
         lhs.accelUserOffset.z == rhs.accelUserOffset.z &&
         lhs.fifo.enabled == rhs.fifo.enabled &&
         lhs.fifo.mode == rhs.fifo.mode &&
         lhs.fifo.odr == rhs.fifo.odr &&
         lhs.fifo.gyroDecimation == rhs.fifo.gyroDecimation &&
         lhs.fifo.accelDecimation == rhs.fifo.accelDecimation &&
         lhs.fifo.fourthDataset == rhs.fifo.fourthDataset &&
         lhs.fifo.fourthDecimation == rhs.fifo.fourthDecimation &&
         lhs.fifo.trigger == rhs.fifo.trigger &&
         lhs.fifo.timestampResolution == rhs.fifo.timestampResolution &&
         lhs.fifo.watermarkWords == rhs.fifo.watermarkWords &&
         lhs.interrupts.enabled == rhs.interrupts.enabled &&
         lhs.interrupts.polarity == rhs.interrupts.polarity &&
         lhs.interrupts.drive == rhs.interrupts.drive &&
         lhs.interrupts.dataReadyMode == rhs.interrupts.dataReadyMode &&
         lhs.interrupts.latchEvents == rhs.interrupts.latchEvents &&
         lhs.interrupts.int2OnInt1 == rhs.interrupts.int2OnInt1 &&
         equalRoutes(lhs.interrupts.int1, rhs.interrupts.int1) &&
         equalRoutes(lhs.interrupts.int2, rhs.interrupts.int2) &&
         lhs.interrupts.orientation.mode == rhs.interrupts.orientation.mode &&
         lhs.interrupts.orientation.threshold == rhs.interrupts.orientation.threshold;
}

inline bool parseBool(const char* text, bool& value) {
  if (text == nullptr) return false;
  if (std::strcmp(text, "1") == 0 || std::strcmp(text, "on") == 0 ||
      std::strcmp(text, "true") == 0) {
    value = true;
    return true;
  }
  if (std::strcmp(text, "0") == 0 || std::strcmp(text, "off") == 0 ||
      std::strcmp(text, "false") == 0) {
    value = false;
    return true;
  }
  return false;
}

inline bool parseSigned(const char* text, int32_t minimum, int32_t maximum,
                        int32_t& value) {
  if (text == nullptr || *text == '\0') return false;
  char* end = nullptr;
  const long parsed = std::strtol(text, &end, 0);
  if (*end != '\0' || parsed < minimum || parsed > maximum) return false;
  value = static_cast<int32_t>(parsed);
  return true;
}

inline bool parseOdr(const char* text, Odr& value) {
  if (text == nullptr) return false;
  if (std::strcmp(text, "pd") == 0 || std::strcmp(text, "powerdown") == 0) {
    value = Odr::POWER_DOWN;
    return true;
  }
  struct Entry { const char* name; Odr odr; };
  static constexpr Entry ENTRIES[] = {
      {"1.6", Odr::HZ_1_6}, {"12.5", Odr::HZ_12_5},
      {"26", Odr::HZ_26}, {"52", Odr::HZ_52}, {"104", Odr::HZ_104},
      {"208", Odr::HZ_208}, {"416", Odr::HZ_416}, {"833", Odr::HZ_833},
      {"1660", Odr::HZ_1660}, {"3330", Odr::HZ_3330},
      {"6660", Odr::HZ_6660},
  };
  for (const Entry& entry : ENTRIES) {
    if (std::strcmp(text, entry.name) == 0) {
      value = entry.odr;
      return true;
    }
  }
  return false;
}

inline bool parseAccelFs(const char* text, AccelFs& value) {
  if (text == nullptr) return false;
  if (std::strcmp(text, "2") == 0) value = AccelFs::G_2;
  else if (std::strcmp(text, "4") == 0) value = AccelFs::G_4;
  else if (std::strcmp(text, "8") == 0) value = AccelFs::G_8;
  else if (std::strcmp(text, "16") == 0) value = AccelFs::G_16;
  else return false;
  return true;
}

inline bool parseGyroFs(const char* text, GyroFs& value) {
  if (text == nullptr) return false;
  if (std::strcmp(text, "125") == 0) value = GyroFs::DPS_125;
  else if (std::strcmp(text, "250") == 0) value = GyroFs::DPS_250;
  else if (std::strcmp(text, "500") == 0) value = GyroFs::DPS_500;
  else if (std::strcmp(text, "1000") == 0) value = GyroFs::DPS_1000;
  else if (std::strcmp(text, "2000") == 0) value = GyroFs::DPS_2000;
  else return false;
  return true;
}

inline bool parseAccelPower(const char* text, AccelPowerMode& value) {
  if (text == nullptr) return false;
  if (std::strcmp(text, "hp") == 0 || std::strcmp(text, "high") == 0) {
    value = AccelPowerMode::HIGH_PERFORMANCE;
  } else if (std::strcmp(text, "lp") == 0 || std::strcmp(text, "low") == 0) {
    value = AccelPowerMode::LOW_POWER_NORMAL;
  } else {
    return false;
  }
  return true;
}

inline bool parseGyroPower(const char* text, GyroPowerMode& value) {
  if (text == nullptr) return false;
  if (std::strcmp(text, "hp") == 0 || std::strcmp(text, "high") == 0) {
    value = GyroPowerMode::HIGH_PERFORMANCE;
  } else if (std::strcmp(text, "lp") == 0 || std::strcmp(text, "low") == 0) {
    value = GyroPowerMode::LOW_POWER_NORMAL;
  } else {
    return false;
  }
  return true;
}

inline bool parseGyroHpfMode(const char* text, GyroHpfMode& value) {
  if (text == nullptr) return false;
  if (std::strcmp(text, "0.016") == 0) value = GyroHpfMode::HZ_0_016;
  else if (std::strcmp(text, "0.065") == 0) value = GyroHpfMode::HZ_0_065;
  else if (std::strcmp(text, "0.260") == 0) value = GyroHpfMode::HZ_0_260;
  else if (std::strcmp(text, "1.040") == 0) value = GyroHpfMode::HZ_1_040;
  else return false;
  return true;
}

inline Status invalidValue(const char* message) {
  return Status::Error(Err::INVALID_PARAM, message);
}

inline bool parseFifoMode(const char* text, FifoMode& value) {
  if (text == nullptr) return false;
  if (std::strcmp(text, "bypass") == 0) value = FifoMode::BYPASS;
  else if (std::strcmp(text, "fifo") == 0) value = FifoMode::FIFO;
  else if (std::strcmp(text, "continuous") == 0) value = FifoMode::CONTINUOUS;
  else return false;
  return true;
}

inline bool parseFifoDecimation(const char* text, FifoDecimation& value) {
  if (text == nullptr) return false;
  static constexpr FifoDecimation VALUES[] = {
      FifoDecimation::OFF, FifoDecimation::EVERY_1, FifoDecimation::EVERY_2,
      FifoDecimation::EVERY_3, FifoDecimation::EVERY_4, FifoDecimation::EVERY_8,
      FifoDecimation::EVERY_16, FifoDecimation::EVERY_32};
  for (const FifoDecimation candidate : VALUES) {
    if (std::strcmp(text, fifoDecimationName(candidate)) == 0) {
      value = candidate;
      return true;
    }
  }
  return false;
}

inline bool parseFifoFourthDataset(const char* text, FifoFourthDataset& value) {
  if (text == nullptr) return false;
  if (std::strcmp(text, "off") == 0) value = FifoFourthDataset::OFF;
  else if (std::strcmp(text, "temperature") == 0) value = FifoFourthDataset::TEMPERATURE;
  else if (std::strcmp(text, "timestamp_step") == 0) value = FifoFourthDataset::TIMESTAMP_STEP;
  else return false;
  return true;
}

inline bool parseFifoTrigger(const char* text, FifoTrigger& value) {
  if (text == nullptr) return false;
  if (std::strcmp(text, "internal") == 0) value = FifoTrigger::INTERNAL;
  else if (std::strcmp(text, "step") == 0) value = FifoTrigger::STEP_EVENT;
  else return false;
  return true;
}

inline bool parseTimestampResolution(const char* text, TimestampResolution& value) {
  if (text == nullptr) return false;
  if (std::strcmp(text, "6.4ms") == 0) value = TimestampResolution::MS_6_4;
  else if (std::strcmp(text, "25us") == 0) value = TimestampResolution::US_25;
  else return false;
  return true;
}

inline bool parseInterruptPolarity(const char* text, InterruptPolarity& value) {
  if (text == nullptr) return false;
  if (std::strcmp(text, "high") == 0) value = InterruptPolarity::ACTIVE_HIGH;
  else if (std::strcmp(text, "low") == 0) value = InterruptPolarity::ACTIVE_LOW;
  else return false;
  return true;
}

inline bool parseInterruptDrive(const char* text, InterruptDrive& value) {
  if (text == nullptr) return false;
  if (std::strcmp(text, "push_pull") == 0) value = InterruptDrive::PUSH_PULL;
  else if (std::strcmp(text, "open_drain") == 0) value = InterruptDrive::OPEN_DRAIN_PAD;
  else return false;
  return true;
}

inline bool parseDataReadyMode(const char* text, DataReadyMode& value) {
  if (text == nullptr) return false;
  if (std::strcmp(text, "latched") == 0) value = DataReadyMode::LATCHED;
  else if (std::strcmp(text, "pulsed") == 0) value = DataReadyMode::PULSED;
  else return false;
  return true;
}

inline bool parseOrientationMode(const char* text, OrientationMode& value) {
  if (text == nullptr) return false;
  if (std::strcmp(text, "off") == 0) value = OrientationMode::OFF;
  else if (std::strcmp(text, "6d") == 0) value = OrientationMode::SIX_D;
  else if (std::strcmp(text, "4d") == 0) value = OrientationMode::FOUR_D;
  else return false;
  return true;
}

inline bool parseOrientationThreshold(const char* text, OrientationThreshold& value) {
  if (text == nullptr) return false;
  if (std::strcmp(text, "80") == 0) value = OrientationThreshold::DEG_80;
  else if (std::strcmp(text, "70") == 0) value = OrientationThreshold::DEG_70;
  else if (std::strcmp(text, "60") == 0) value = OrientationThreshold::DEG_60;
  else if (std::strcmp(text, "50") == 0) value = OrientationThreshold::DEG_50;
  else return false;
  return true;
}

inline bool setRoute(InterruptRoutes& routes, const char* source, const char* text) {
  if (source == nullptr) return false;
  bool value = false;
  if (!parseBool(text, value)) return false;
  if (std::strcmp(source, "accel") == 0) routes.accelReady = value;
  else if (std::strcmp(source, "gyro") == 0) routes.gyroReady = value;
  else if (std::strcmp(source, "temp") == 0) routes.temperatureReady = value;
  else if (std::strcmp(source, "watermark") == 0) routes.fifoWatermark = value;
  else if (std::strcmp(source, "full") == 0) routes.fifoFull = value;
  else if (std::strcmp(source, "overrun") == 0) routes.fifoOverrun = value;
  else if (std::strcmp(source, "step") == 0) routes.stepDetected = value;
  else if (std::strcmp(source, "step_overflow") == 0) routes.stepOverflow = value;
  else if (std::strcmp(source, "timestamp") == 0) routes.timestampEndCounter = value;
  else if (std::strcmp(source, "orientation") == 0) routes.orientation = value;
  else return false;
  return true;
}

// "all" means every configured observation group, not every chip engine.
// There is no readable timestamp-end status bit; GPIO evidence is external.
inline bool eventRequest(const char* selection, const DeviceProfile& profile,
                          EventReadRequest& out) {
  if (selection == nullptr) return false;
  EventReadRequest candidate{};
  candidate.dataReady = false;
  if (std::strcmp(selection, "ready") == 0) candidate.dataReady = true;
  else if (std::strcmp(selection, "fifo") == 0) candidate.fifo = true;
  else if (std::strcmp(selection, "embedded") == 0) candidate.embedded = true;
  else if (std::strcmp(selection, "orientation") == 0) candidate.orientation = true;
  else if (std::strcmp(selection, "all") == 0) {
    candidate.dataReady = true;
    candidate.fifo = profile.fifo.enabled;
    candidate.embedded = profile.fifo.enabled &&
        profile.fifo.fourthDataset == FifoFourthDataset::TIMESTAMP_STEP;
    candidate.orientation = profile.interrupts.enabled &&
        profile.interrupts.orientation.mode != OrientationMode::OFF;
  } else return false;
  out = candidate;
  return true;
}

/**
 * @brief Apply one typed field update atomically to a valid profile draft.
 * @param profile Current valid draft; unchanged on any error.
 * @param field Field name from the CLI contract.
 * @param values Field value tokens.
 * @param valueCount Number of tokens at @p values.
 * @return OK when committed, otherwise a parse or complete-profile validation error.
 * @note Cross-field constraints are checked on every edit. Coupled ODR/power
 *       changes therefore need an order in which every intermediate draft is
 *       valid; profile defaults provide a deterministic clean draft.
 */
inline Status setField(DeviceProfile& profile, const char* field,
                       const char* const* values, size_t valueCount) {
  if (field == nullptr || values == nullptr) {
    return invalidValue("Profile field/value missing");
  }
  for (size_t index = 0; index < valueCount; ++index) {
    if (values[index] == nullptr)
      return invalidValue("Profile field/value missing");
  }
  DeviceProfile candidate = profile;
  if (std::strcmp(field, "xl_odr") == 0) {
    if (valueCount != 1U || !parseOdr(values[0], candidate.accelOdr))
      return invalidValue("xl_odr expects pd|1.6|12.5|26|52|104|208|416|833|1660|3330|6660");
  } else if (std::strcmp(field, "xl_fs") == 0) {
    if (valueCount != 1U || !parseAccelFs(values[0], candidate.accelFullScale))
      return invalidValue("xl_fs expects 2|4|8|16");
  } else if (std::strcmp(field, "xl_power") == 0) {
    if (valueCount != 1U || !parseAccelPower(values[0], candidate.accelPowerMode))
      return invalidValue("xl_power expects hp|lp");
  } else if (std::strcmp(field, "xl_lpf2") == 0) {
    if (valueCount != 1U || !parseBool(values[0], candidate.accelFilter.lpf2Enabled))
      return invalidValue("xl_lpf2 expects 0|1");
  } else if (std::strcmp(field, "xl_slope_hp") == 0) {
    if (valueCount != 1U || !parseBool(values[0], candidate.accelFilter.highPassSlopeEnabled))
      return invalidValue("xl_slope_hp expects 0|1");
  } else if (std::strcmp(field, "xl_6d_lpf") == 0) {
    if (valueCount != 1U || !parseBool(values[0], candidate.accelFilter.lowPassOn6d))
      return invalidValue("xl_6d_lpf expects 0|1");
  } else if (std::strcmp(field, "g_odr") == 0) {
    if (valueCount != 1U || !parseOdr(values[0], candidate.gyroOdr) ||
        candidate.gyroOdr == Odr::HZ_1_6)
      return invalidValue("g_odr expects pd|12.5|26|52|104|208|416|833|1660|3330|6660");
  } else if (std::strcmp(field, "g_fs") == 0) {
    if (valueCount != 1U || !parseGyroFs(values[0], candidate.gyroFullScale))
      return invalidValue("g_fs expects 125|250|500|1000|2000");
  } else if (std::strcmp(field, "g_power") == 0) {
    if (valueCount != 1U || !parseGyroPower(values[0], candidate.gyroPowerMode))
      return invalidValue("g_power expects hp|lp");
  } else if (std::strcmp(field, "g_lpf1") == 0) {
    if (valueCount != 1U || !parseBool(values[0], candidate.gyroFilter.lpf1Enabled))
      return invalidValue("g_lpf1 expects 0|1");
  } else if (std::strcmp(field, "g_hpf") == 0) {
    if (valueCount != 1U || !parseBool(values[0], candidate.gyroFilter.highPassEnabled))
      return invalidValue("g_hpf expects 0|1");
  } else if (std::strcmp(field, "g_hpf_mode") == 0) {
    if (valueCount != 1U ||
        !parseGyroHpfMode(values[0], candidate.gyroFilter.highPassMode))
      return invalidValue("g_hpf_mode expects 0.016|0.065|0.260|1.040");
  } else if (std::strcmp(field, "g_sleep") == 0) {
    if (valueCount != 1U || !parseBool(values[0], candidate.gyroSleepEnabled))
      return invalidValue("g_sleep expects 0|1");
  } else if (std::strcmp(field, "bdu") == 0) {
    if (valueCount != 1U || !parseBool(values[0], candidate.blockDataUpdate))
      return invalidValue("bdu expects 0|1");
  } else if (std::strcmp(field, "offset_weight") == 0) {
    if (valueCount != 1U) return invalidValue("offset_weight expects 1|16");
    if (std::strcmp(values[0], "1") == 0) candidate.accelOffsetWeight = AccelOffsetWeight::MG_1;
    else if (std::strcmp(values[0], "16") == 0) candidate.accelOffsetWeight = AccelOffsetWeight::MG_16;
    else return invalidValue("offset_weight expects 1|16");
  } else if (std::strcmp(field, "offset") == 0) {
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;
    if (valueCount != 3U || !parseSigned(values[0], -127, 127, x) ||
        !parseSigned(values[1], -127, 127, y) ||
        !parseSigned(values[2], -127, 127, z))
      return invalidValue("offset expects x y z in -127..127");
    candidate.accelUserOffset = {static_cast<int8_t>(x), static_cast<int8_t>(y),
                                 static_cast<int8_t>(z)};
  } else if (std::strcmp(field, "fifo") == 0) {
    if (valueCount == 1U) {
      if (!parseBool(values[0], candidate.fifo.enabled))
        return invalidValue("fifo expects off or <bypass|fifo|continuous> <odr> <g_dec> <xl_dec> <watermark>");
      if (!candidate.fifo.enabled) candidate.fifo = {};
    } else {
      int32_t watermark = 0;
      if (valueCount != 5U || !parseFifoMode(values[0], candidate.fifo.mode) ||
          !parseOdr(values[1], candidate.fifo.odr) ||
          !parseFifoDecimation(values[2], candidate.fifo.gyroDecimation) ||
          !parseFifoDecimation(values[3], candidate.fifo.accelDecimation) ||
          !parseSigned(values[4], 0, 2047, watermark))
        return invalidValue("fifo expects <bypass|fifo|continuous> <odr> <g_dec> <xl_dec> <0..2047>");
      candidate.fifo.enabled = true;
      candidate.fifo.watermarkWords = static_cast<uint16_t>(watermark);
    }
  } else if (std::strcmp(field, "fifo_mode") == 0) {
    if (valueCount != 1U || !parseFifoMode(values[0], candidate.fifo.mode))
      return invalidValue("fifo_mode expects bypass|fifo|continuous");
  } else if (std::strcmp(field, "fifo_odr") == 0) {
    if (valueCount != 1U || !parseOdr(values[0], candidate.fifo.odr))
      return invalidValue("fifo_odr expects 12.5|26|52|104|208|416|833|1660|3330|6660");
  } else if (std::strcmp(field, "fifo_g_dec") == 0) {
    if (valueCount != 1U || !parseFifoDecimation(values[0], candidate.fifo.gyroDecimation))
      return invalidValue("fifo_g_dec expects off|1|2|3|4|8|16|32");
  } else if (std::strcmp(field, "fifo_xl_dec") == 0) {
    if (valueCount != 1U || !parseFifoDecimation(values[0], candidate.fifo.accelDecimation))
      return invalidValue("fifo_xl_dec expects off|1|2|3|4|8|16|32");
  } else if (std::strcmp(field, "fifo_fourth") == 0) {
    if (valueCount == 1U && std::strcmp(values[0], "off") == 0) {
      candidate.fifo.fourthDataset = FifoFourthDataset::OFF;
      candidate.fifo.fourthDecimation = FifoDecimation::OFF;
      candidate.fifo.trigger = FifoTrigger::INTERNAL;
      candidate.fifo.timestampResolution = TimestampResolution::MS_6_4;
    } else if (valueCount != 4U ||
        !parseFifoFourthDataset(values[0], candidate.fifo.fourthDataset) ||
        !parseFifoDecimation(values[1], candidate.fifo.fourthDecimation) ||
        !parseFifoTrigger(values[2], candidate.fifo.trigger) ||
        !parseTimestampResolution(values[3], candidate.fifo.timestampResolution)) {
      return invalidValue("fifo_fourth expects off or <temperature|timestamp_step> <dec> <internal|step> <6.4ms|25us>");
    }
  } else if (std::strcmp(field, "fifo_fourth_dec") == 0) {
    if (valueCount != 1U || !parseFifoDecimation(values[0], candidate.fifo.fourthDecimation))
      return invalidValue("fifo_fourth_dec expects 1|2|3|4|8|16|32, or off only when disabled");
  } else if (std::strcmp(field, "fifo_trigger") == 0) {
    if (valueCount != 1U || !parseFifoTrigger(values[0], candidate.fifo.trigger))
      return invalidValue("fifo_trigger expects internal|step; step requires timestamp_step");
  } else if (std::strcmp(field, "timestamp_resolution") == 0) {
    if (valueCount != 1U || !parseTimestampResolution(values[0], candidate.fifo.timestampResolution))
      return invalidValue("timestamp_resolution expects 6.4ms|25us");
  } else if (std::strcmp(field, "fifo_watermark") == 0) {
    int32_t watermark = 0;
    if (valueCount != 1U || !parseSigned(values[0], 0, 2047, watermark))
      return invalidValue("fifo_watermark expects 0..2047 words");
    candidate.fifo.watermarkWords = static_cast<uint16_t>(watermark);
  } else if (std::strcmp(field, "interrupts") == 0) {
    if (valueCount != 1U || !parseBool(values[0], candidate.interrupts.enabled))
      return invalidValue("interrupts expects 0|1");
    if (!candidate.interrupts.enabled) candidate.interrupts = {};
  } else if (std::strcmp(field, "irq") == 0) {
    if (valueCount != 5U ||
        !parseInterruptPolarity(values[0], candidate.interrupts.polarity) ||
        !parseInterruptDrive(values[1], candidate.interrupts.drive) ||
        !parseDataReadyMode(values[2], candidate.interrupts.dataReadyMode) ||
        !parseBool(values[3], candidate.interrupts.latchEvents) ||
        !parseBool(values[4], candidate.interrupts.int2OnInt1))
      return invalidValue("irq expects <high|low> <push_pull|open_drain> <latched|pulsed> <latch_events> <combine>");
    candidate.interrupts.enabled = true;
  } else if (std::strcmp(field, "irq_polarity") == 0) {
    if (valueCount != 1U || !parseInterruptPolarity(values[0], candidate.interrupts.polarity))
      return invalidValue("irq_polarity expects high|low");
  } else if (std::strcmp(field, "irq_drive") == 0) {
    if (valueCount != 1U || !parseInterruptDrive(values[0], candidate.interrupts.drive))
      return invalidValue("irq_drive expects push_pull|open_drain");
  } else if (std::strcmp(field, "irq_drdy") == 0) {
    if (valueCount != 1U || !parseDataReadyMode(values[0], candidate.interrupts.dataReadyMode))
      return invalidValue("irq_drdy expects latched|pulsed");
  } else if (std::strcmp(field, "irq_latch") == 0) {
    if (valueCount != 1U || !parseBool(values[0], candidate.interrupts.latchEvents))
      return invalidValue("irq_latch expects 0|1");
  } else if (std::strcmp(field, "irq_combine") == 0) {
    if (valueCount != 1U || !parseBool(values[0], candidate.interrupts.int2OnInt1))
      return invalidValue("irq_combine expects 0|1");
  } else if (std::strcmp(field, "int1") == 0 || std::strcmp(field, "int2") == 0) {
    InterruptRoutes& routes = std::strcmp(field, "int1") == 0
                                  ? candidate.interrupts.int1 : candidate.interrupts.int2;
    if (valueCount != 2U || !setRoute(routes, values[0], values[1]))
      return invalidValue("int1|int2 expects <accel|gyro|temp|watermark|full|overrun|step|step_overflow|timestamp|orientation> <0|1>");
  } else if (std::strcmp(field, "orientation") == 0) {
    if (valueCount == 1U && std::strcmp(values[0], "off") == 0) {
      candidate.interrupts.orientation = {};
      candidate.interrupts.int1.orientation = false;
      candidate.interrupts.int2.orientation = false;
    } else {
      if (valueCount != 3U ||
          !parseOrientationMode(values[0], candidate.interrupts.orientation.mode) ||
          candidate.interrupts.orientation.mode == OrientationMode::OFF ||
          !parseOrientationThreshold(values[1], candidate.interrupts.orientation.threshold) ||
          (std::strcmp(values[2], "int1") != 0 && std::strcmp(values[2], "int2") != 0 &&
           std::strcmp(values[2], "both") != 0))
        return invalidValue("orientation expects off or <6d|4d> <80|70|60|50> <int1|int2|both>");
      candidate.interrupts.enabled = true;
      candidate.interrupts.int1.orientation = std::strcmp(values[2], "int2") != 0;
      candidate.interrupts.int2.orientation = std::strcmp(values[2], "int1") != 0;
    }
  } else {
    return invalidValue("Unknown profile field");
  }

  const Status valid = validateProfile(candidate);
  if (!valid.ok()) return valid;
  profile = candidate;
  return Status::Ok();
}

}  // namespace profile_cli
