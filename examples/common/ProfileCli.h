/**
 * @file ProfileCli.h
 * @brief Framework-neutral typed DeviceProfile parsing for the CLI examples.
 *
 * This example-only helper keeps the Arduino and native ESP-IDF command
 * surfaces identical. It performs no I2C and commits a draft change only when
 * the complete resulting profile passes the production driver's validation.
 * It covers every DeviceProfile field: supported ODR/range/power/filter/sleep
 * and offset values, inactive gyro HPF mode bits, and the invariant fields
 * that production validation deliberately keeps at BDU-on, FIFO/interrupt-off,
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
         lhs.interrupts.enabled == rhs.interrupts.enabled;
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
    if (valueCount != 1U || !parseBool(values[0], candidate.fifo.enabled))
      return invalidValue("fifo expects 0|1");
  } else if (std::strcmp(field, "interrupts") == 0) {
    if (valueCount != 1U || !parseBool(values[0], candidate.interrupts.enabled))
      return invalidValue("interrupts expects 0|1");
  } else {
    return invalidValue("Unknown profile field");
  }

  const Status valid = validateProfile(candidate);
  if (!valid.ok()) return valid;
  profile = candidate;
  return Status::Ok();
}

}  // namespace profile_cli
