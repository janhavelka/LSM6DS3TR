/// @file test_profile_cli.cpp
/// @brief Exhaustive native tests for the shared example profile CLI helper.

#include <unity.h>

#include <cstddef>
#include <cstdint>

#include "../examples/common/ProfileCli.h"

using namespace LSM6DS3TR;

namespace {

template <typename Enum>
void assertEnumEqual(Enum expected, Enum actual) {
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expected),
                          static_cast<uint8_t>(actual));
}

void assertStatusCode(Err expected, const Status& actual) {
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expected),
                          static_cast<uint8_t>(actual.code));
}

Status setOne(DeviceProfile& profile, const char* field, const char* value) {
  const char* values[] = {value};
  return profile_cli::setField(profile, field, values, 1U);
}

void assertAtomicError(DeviceProfile& profile, const char* field,
                       const char* const* values, size_t valueCount,
                       Err expected) {
  const DeviceProfile before = profile;
  const Status status = profile_cli::setField(profile, field, values, valueCount);
  assertStatusCode(expected, status);
  TEST_ASSERT_TRUE(profile_cli::equal(before, profile));
}

void assertValid(const DeviceProfile& profile) {
  TEST_ASSERT_TRUE(validateProfile(profile).ok());
}

void assertDifferent(const DeviceProfile& lhs, const DeviceProfile& rhs) {
  TEST_ASSERT_FALSE(profile_cli::equal(lhs, rhs));
  TEST_ASSERT_FALSE(profile_cli::equal(rhs, lhs));
}

void test_profile_cli_names_cover_every_enum_value() {
  TEST_ASSERT_EQUAL_STRING("off", profile_cli::boolName(false));
  TEST_ASSERT_EQUAL_STRING("on", profile_cli::boolName(true));

  struct OdrCase {
    Odr value;
    const char* name;
  };
  static constexpr OdrCase ODR_CASES[] = {
      {Odr::POWER_DOWN, "pd"}, {Odr::HZ_1_6, "1.6"},
      {Odr::HZ_12_5, "12.5"}, {Odr::HZ_26, "26"},
      {Odr::HZ_52, "52"},     {Odr::HZ_104, "104"},
      {Odr::HZ_208, "208"},   {Odr::HZ_416, "416"},
      {Odr::HZ_833, "833"},   {Odr::HZ_1660, "1660"},
      {Odr::HZ_3330, "3330"}, {Odr::HZ_6660, "6660"},
  };
  for (const OdrCase& entry : ODR_CASES) {
    TEST_ASSERT_EQUAL_STRING(entry.name, profile_cli::odrName(entry.value));
  }
  TEST_ASSERT_EQUAL_STRING("invalid",
                           profile_cli::odrName(static_cast<Odr>(0xFE)));

  struct AccelFsCase {
    AccelFs value;
    const char* name;
  };
  static constexpr AccelFsCase ACCEL_FS_CASES[] = {
      {AccelFs::G_2, "2"}, {AccelFs::G_4, "4"},
      {AccelFs::G_8, "8"}, {AccelFs::G_16, "16"},
  };
  for (const AccelFsCase& entry : ACCEL_FS_CASES) {
    TEST_ASSERT_EQUAL_STRING(entry.name, profile_cli::accelFsName(entry.value));
  }
  TEST_ASSERT_EQUAL_STRING(
      "invalid", profile_cli::accelFsName(static_cast<AccelFs>(0xFE)));

  struct GyroFsCase {
    GyroFs value;
    const char* name;
  };
  static constexpr GyroFsCase GYRO_FS_CASES[] = {
      {GyroFs::DPS_125, "125"},   {GyroFs::DPS_250, "250"},
      {GyroFs::DPS_500, "500"},   {GyroFs::DPS_1000, "1000"},
      {GyroFs::DPS_2000, "2000"},
  };
  for (const GyroFsCase& entry : GYRO_FS_CASES) {
    TEST_ASSERT_EQUAL_STRING(entry.name, profile_cli::gyroFsName(entry.value));
  }
  TEST_ASSERT_EQUAL_STRING(
      "invalid", profile_cli::gyroFsName(static_cast<GyroFs>(0xFE)));

  TEST_ASSERT_EQUAL_STRING(
      "hp", profile_cli::accelPowerName(AccelPowerMode::HIGH_PERFORMANCE));
  TEST_ASSERT_EQUAL_STRING(
      "lp", profile_cli::accelPowerName(AccelPowerMode::LOW_POWER_NORMAL));
  TEST_ASSERT_EQUAL_STRING(
      "invalid",
      profile_cli::accelPowerName(static_cast<AccelPowerMode>(0xFE)));
  TEST_ASSERT_EQUAL_STRING(
      "hp", profile_cli::gyroPowerName(GyroPowerMode::HIGH_PERFORMANCE));
  TEST_ASSERT_EQUAL_STRING(
      "lp", profile_cli::gyroPowerName(GyroPowerMode::LOW_POWER_NORMAL));
  TEST_ASSERT_EQUAL_STRING(
      "invalid",
      profile_cli::gyroPowerName(static_cast<GyroPowerMode>(0xFE)));

  struct HpfCase {
    GyroHpfMode value;
    const char* name;
  };
  static constexpr HpfCase HPF_CASES[] = {
      {GyroHpfMode::HZ_0_016, "0.016"},
      {GyroHpfMode::HZ_0_065, "0.065"},
      {GyroHpfMode::HZ_0_260, "0.260"},
      {GyroHpfMode::HZ_1_040, "1.040"},
  };
  for (const HpfCase& entry : HPF_CASES) {
    TEST_ASSERT_EQUAL_STRING(entry.name,
                             profile_cli::gyroHpfModeName(entry.value));
  }
  TEST_ASSERT_EQUAL_STRING(
      "invalid",
      profile_cli::gyroHpfModeName(static_cast<GyroHpfMode>(0xFE)));

  TEST_ASSERT_EQUAL_STRING(
      "1", profile_cli::offsetWeightName(AccelOffsetWeight::MG_1));
  TEST_ASSERT_EQUAL_STRING(
      "16", profile_cli::offsetWeightName(AccelOffsetWeight::MG_16));
  TEST_ASSERT_EQUAL_STRING(
      "invalid",
      profile_cli::offsetWeightName(static_cast<AccelOffsetWeight>(0xFE)));
}

void test_profile_cli_equal_compares_every_profile_field() {
  const DeviceProfile baseline;
  DeviceProfile candidate = baseline;
  TEST_ASSERT_TRUE(profile_cli::equal(baseline, candidate));

  candidate.accelOdr = Odr::HZ_208;
  assertDifferent(baseline, candidate);
  candidate = baseline;
  candidate.accelFullScale = AccelFs::G_4;
  assertDifferent(baseline, candidate);
  candidate = baseline;
  candidate.accelPowerMode = AccelPowerMode::LOW_POWER_NORMAL;
  assertDifferent(baseline, candidate);
  candidate = baseline;
  candidate.gyroOdr = Odr::HZ_208;
  assertDifferent(baseline, candidate);
  candidate = baseline;
  candidate.gyroFullScale = GyroFs::DPS_500;
  assertDifferent(baseline, candidate);
  candidate = baseline;
  candidate.gyroPowerMode = GyroPowerMode::LOW_POWER_NORMAL;
  assertDifferent(baseline, candidate);
  candidate = baseline;
  candidate.accelFilter.lpf2Enabled = true;
  assertDifferent(baseline, candidate);
  candidate = baseline;
  candidate.accelFilter.highPassSlopeEnabled = true;
  assertDifferent(baseline, candidate);
  candidate = baseline;
  candidate.accelFilter.lowPassOn6d = true;
  assertDifferent(baseline, candidate);
  candidate = baseline;
  candidate.gyroFilter.lpf1Enabled = true;
  assertDifferent(baseline, candidate);
  candidate = baseline;
  candidate.gyroFilter.highPassEnabled = true;
  assertDifferent(baseline, candidate);
  candidate = baseline;
  candidate.gyroFilter.highPassMode = GyroHpfMode::HZ_0_065;
  assertDifferent(baseline, candidate);
  candidate = baseline;
  candidate.gyroSleepEnabled = true;
  assertDifferent(baseline, candidate);
  candidate = baseline;
  candidate.blockDataUpdate = false;
  assertDifferent(baseline, candidate);
  candidate = baseline;
  candidate.accelOffsetWeight = AccelOffsetWeight::MG_16;
  assertDifferent(baseline, candidate);
  candidate = baseline;
  candidate.accelUserOffset.x = 1;
  assertDifferent(baseline, candidate);
  candidate = baseline;
  candidate.accelUserOffset.y = -1;
  assertDifferent(baseline, candidate);
  candidate = baseline;
  candidate.accelUserOffset.z = 127;
  assertDifferent(baseline, candidate);
  candidate = baseline;
  candidate.fifo.enabled = true;
  assertDifferent(baseline, candidate);
  candidate = baseline;
  candidate.interrupts.enabled = true;
  assertDifferent(baseline, candidate);

  candidate = baseline;
  TEST_ASSERT_TRUE(profile_cli::equal(candidate, baseline));

  DeviceProfile nonDefault;
  nonDefault.accelOdr = Odr::HZ_208;
  nonDefault.accelFullScale = AccelFs::G_8;
  nonDefault.accelPowerMode = AccelPowerMode::LOW_POWER_NORMAL;
  nonDefault.accelFilter.lpf2Enabled = true;
  nonDefault.gyroOdr = Odr::HZ_416;
  nonDefault.gyroFullScale = GyroFs::DPS_1000;
  nonDefault.gyroFilter.lpf1Enabled = true;
  nonDefault.gyroFilter.highPassMode = GyroHpfMode::HZ_1_040;
  nonDefault.gyroSleepEnabled = true;
  nonDefault.accelOffsetWeight = AccelOffsetWeight::MG_16;
  nonDefault.accelUserOffset = {-127, 42, 127};
  const DeviceProfile identical = nonDefault;
  TEST_ASSERT_TRUE(profile_cli::equal(nonDefault, identical));
}

void test_profile_cli_bool_and_signed_parsers_are_total_and_atomic() {
  struct BoolCase {
    const char* text;
    bool expected;
  };
  static constexpr BoolCase BOOL_CASES[] = {
      {"1", true},   {"on", true},   {"true", true},
      {"0", false}, {"off", false}, {"false", false},
  };
  for (const BoolCase& entry : BOOL_CASES) {
    bool value = !entry.expected;
    TEST_ASSERT_TRUE(profile_cli::parseBool(entry.text, value));
    TEST_ASSERT_EQUAL(entry.expected, value);
  }
  const char* invalidBools[] = {nullptr, "", "2", "ON", " true", "false ",
                                "yes"};
  for (const char* text : invalidBools) {
    bool value = true;
    TEST_ASSERT_FALSE(profile_cli::parseBool(text, value));
    TEST_ASSERT_TRUE(value);
  }

  struct SignedCase {
    const char* text;
    int32_t expected;
  };
  static constexpr SignedCase SIGNED_CASES[] = {
      {"-127", -127}, {"127", 127}, {"0", 0}, {"+42", 42},
      {"017", 15},    {"0x7f", 127}, {"-0x7f", -127}, {" 12", 12},
  };
  for (const SignedCase& entry : SIGNED_CASES) {
    int32_t value = 99;
    TEST_ASSERT_TRUE(profile_cli::parseSigned(entry.text, -127, 127, value));
    TEST_ASSERT_EQUAL_INT32(entry.expected, value);
  }
  const char* invalidSigned[] = {nullptr,
                                 "",
                                 "-128",
                                 "128",
                                 "12x",
                                 "12 ",
                                 "999999999999999999999999999999999"};
  for (const char* text : invalidSigned) {
    int32_t value = 37;
    TEST_ASSERT_FALSE(profile_cli::parseSigned(text, -127, 127, value));
    TEST_ASSERT_EQUAL_INT32(37, value);
  }
  int32_t reversedRangeValue = 17;
  TEST_ASSERT_FALSE(
      profile_cli::parseSigned("0", 1, -1, reversedRangeValue));
  TEST_ASSERT_EQUAL_INT32(17, reversedRangeValue);

  const Status invalid = profile_cli::invalidValue("bad value");
  assertStatusCode(Err::INVALID_PARAM, invalid);
  TEST_ASSERT_EQUAL_STRING("bad value", invalid.msg);
}

void test_profile_cli_odr_parser_covers_all_tokens_and_is_atomic() {
  struct Case {
    const char* text;
    Odr expected;
  };
  static constexpr Case CASES[] = {
      {"pd", Odr::POWER_DOWN}, {"powerdown", Odr::POWER_DOWN},
      {"1.6", Odr::HZ_1_6},   {"12.5", Odr::HZ_12_5},
      {"26", Odr::HZ_26},     {"52", Odr::HZ_52},
      {"104", Odr::HZ_104},   {"208", Odr::HZ_208},
      {"416", Odr::HZ_416},   {"833", Odr::HZ_833},
      {"1660", Odr::HZ_1660}, {"3330", Odr::HZ_3330},
      {"6660", Odr::HZ_6660},
  };
  for (const Case& entry : CASES) {
    Odr value = Odr::HZ_104;
    TEST_ASSERT_TRUE(profile_cli::parseOdr(entry.text, value));
    assertEnumEqual(entry.expected, value);
  }
  const char* invalid[] = {nullptr, "", "PD", "0", "1", "12", "104Hz",
                           "6660 "};
  for (const char* text : invalid) {
    Odr value = Odr::HZ_104;
    TEST_ASSERT_FALSE(profile_cli::parseOdr(text, value));
    assertEnumEqual(Odr::HZ_104, value);
  }
}

void test_profile_cli_range_power_and_hpf_parsers_cover_all_tokens() {
  struct AccelFsCase {
    const char* text;
    AccelFs expected;
  };
  static constexpr AccelFsCase ACCEL_CASES[] = {
      {"2", AccelFs::G_2}, {"4", AccelFs::G_4},
      {"8", AccelFs::G_8}, {"16", AccelFs::G_16},
  };
  for (const AccelFsCase& entry : ACCEL_CASES) {
    AccelFs value = AccelFs::G_2;
    TEST_ASSERT_TRUE(profile_cli::parseAccelFs(entry.text, value));
    assertEnumEqual(entry.expected, value);
  }
  AccelFs accelFs = AccelFs::G_8;
  TEST_ASSERT_FALSE(profile_cli::parseAccelFs(nullptr, accelFs));
  assertEnumEqual(AccelFs::G_8, accelFs);
  TEST_ASSERT_FALSE(profile_cli::parseAccelFs("3", accelFs));
  assertEnumEqual(AccelFs::G_8, accelFs);

  struct GyroFsCase {
    const char* text;
    GyroFs expected;
  };
  static constexpr GyroFsCase GYRO_CASES[] = {
      {"125", GyroFs::DPS_125},   {"250", GyroFs::DPS_250},
      {"500", GyroFs::DPS_500},   {"1000", GyroFs::DPS_1000},
      {"2000", GyroFs::DPS_2000},
  };
  for (const GyroFsCase& entry : GYRO_CASES) {
    GyroFs value = GyroFs::DPS_250;
    TEST_ASSERT_TRUE(profile_cli::parseGyroFs(entry.text, value));
    assertEnumEqual(entry.expected, value);
  }
  GyroFs gyroFs = GyroFs::DPS_500;
  TEST_ASSERT_FALSE(profile_cli::parseGyroFs(nullptr, gyroFs));
  assertEnumEqual(GyroFs::DPS_500, gyroFs);
  TEST_ASSERT_FALSE(profile_cli::parseGyroFs("124", gyroFs));
  assertEnumEqual(GyroFs::DPS_500, gyroFs);

  struct AccelPowerCase {
    const char* text;
    AccelPowerMode expected;
  };
  static constexpr AccelPowerCase ACCEL_POWER_CASES[] = {
      {"hp", AccelPowerMode::HIGH_PERFORMANCE},
      {"high", AccelPowerMode::HIGH_PERFORMANCE},
      {"lp", AccelPowerMode::LOW_POWER_NORMAL},
      {"low", AccelPowerMode::LOW_POWER_NORMAL},
  };
  for (const AccelPowerCase& entry : ACCEL_POWER_CASES) {
    AccelPowerMode value = AccelPowerMode::HIGH_PERFORMANCE;
    TEST_ASSERT_TRUE(profile_cli::parseAccelPower(entry.text, value));
    assertEnumEqual(entry.expected, value);
  }
  AccelPowerMode accelPower = AccelPowerMode::LOW_POWER_NORMAL;
  TEST_ASSERT_FALSE(profile_cli::parseAccelPower(nullptr, accelPower));
  assertEnumEqual(AccelPowerMode::LOW_POWER_NORMAL, accelPower);
  TEST_ASSERT_FALSE(profile_cli::parseAccelPower("normal", accelPower));
  assertEnumEqual(AccelPowerMode::LOW_POWER_NORMAL, accelPower);

  struct GyroPowerCase {
    const char* text;
    GyroPowerMode expected;
  };
  static constexpr GyroPowerCase GYRO_POWER_CASES[] = {
      {"hp", GyroPowerMode::HIGH_PERFORMANCE},
      {"high", GyroPowerMode::HIGH_PERFORMANCE},
      {"lp", GyroPowerMode::LOW_POWER_NORMAL},
      {"low", GyroPowerMode::LOW_POWER_NORMAL},
  };
  for (const GyroPowerCase& entry : GYRO_POWER_CASES) {
    GyroPowerMode value = GyroPowerMode::HIGH_PERFORMANCE;
    TEST_ASSERT_TRUE(profile_cli::parseGyroPower(entry.text, value));
    assertEnumEqual(entry.expected, value);
  }
  GyroPowerMode gyroPower = GyroPowerMode::LOW_POWER_NORMAL;
  TEST_ASSERT_FALSE(profile_cli::parseGyroPower(nullptr, gyroPower));
  assertEnumEqual(GyroPowerMode::LOW_POWER_NORMAL, gyroPower);
  TEST_ASSERT_FALSE(profile_cli::parseGyroPower("normal", gyroPower));
  assertEnumEqual(GyroPowerMode::LOW_POWER_NORMAL, gyroPower);

  struct HpfCase {
    const char* text;
    GyroHpfMode expected;
  };
  static constexpr HpfCase HPF_CASES[] = {
      {"0.016", GyroHpfMode::HZ_0_016},
      {"0.065", GyroHpfMode::HZ_0_065},
      {"0.260", GyroHpfMode::HZ_0_260},
      {"1.040", GyroHpfMode::HZ_1_040},
  };
  for (const HpfCase& entry : HPF_CASES) {
    GyroHpfMode value = GyroHpfMode::HZ_0_016;
    TEST_ASSERT_TRUE(profile_cli::parseGyroHpfMode(entry.text, value));
    assertEnumEqual(entry.expected, value);
  }
  GyroHpfMode hpf = GyroHpfMode::HZ_0_260;
  TEST_ASSERT_FALSE(profile_cli::parseGyroHpfMode(nullptr, hpf));
  assertEnumEqual(GyroHpfMode::HZ_0_260, hpf);
  TEST_ASSERT_FALSE(profile_cli::parseGyroHpfMode("0.26", hpf));
  assertEnumEqual(GyroHpfMode::HZ_0_260, hpf);
}

void test_profile_cli_set_field_accepts_every_odr_range_and_power_value() {
  struct OdrCase {
    const char* text;
    Odr expected;
  };
  static constexpr OdrCase ACCEL_ODR_CASES[] = {
      {"pd", Odr::POWER_DOWN}, {"powerdown", Odr::POWER_DOWN},
      {"1.6", Odr::HZ_1_6},   {"12.5", Odr::HZ_12_5},
      {"26", Odr::HZ_26},     {"52", Odr::HZ_52},
      {"104", Odr::HZ_104},   {"208", Odr::HZ_208},
      {"416", Odr::HZ_416},   {"833", Odr::HZ_833},
      {"1660", Odr::HZ_1660}, {"3330", Odr::HZ_3330},
      {"6660", Odr::HZ_6660},
  };
  for (const OdrCase& entry : ACCEL_ODR_CASES) {
    DeviceProfile profile;
    if (entry.expected == Odr::HZ_1_6) {
      profile.accelPowerMode = AccelPowerMode::LOW_POWER_NORMAL;
    }
    TEST_ASSERT_TRUE(setOne(profile, "xl_odr", entry.text).ok());
    assertEnumEqual(entry.expected, profile.accelOdr);
    assertValid(profile);
  }

  static constexpr OdrCase GYRO_ODR_CASES[] = {
      {"pd", Odr::POWER_DOWN}, {"powerdown", Odr::POWER_DOWN},
      {"12.5", Odr::HZ_12_5}, {"26", Odr::HZ_26},
      {"52", Odr::HZ_52},     {"104", Odr::HZ_104},
      {"208", Odr::HZ_208},   {"416", Odr::HZ_416},
      {"833", Odr::HZ_833},   {"1660", Odr::HZ_1660},
      {"3330", Odr::HZ_3330}, {"6660", Odr::HZ_6660},
  };
  for (const OdrCase& entry : GYRO_ODR_CASES) {
    DeviceProfile profile;
    TEST_ASSERT_TRUE(setOne(profile, "g_odr", entry.text).ok());
    assertEnumEqual(entry.expected, profile.gyroOdr);
    assertValid(profile);
  }

  struct AccelFsCase {
    const char* text;
    AccelFs expected;
  };
  static constexpr AccelFsCase ACCEL_FS_CASES[] = {
      {"2", AccelFs::G_2}, {"4", AccelFs::G_4},
      {"8", AccelFs::G_8}, {"16", AccelFs::G_16},
  };
  for (const AccelFsCase& entry : ACCEL_FS_CASES) {
    DeviceProfile profile;
    TEST_ASSERT_TRUE(setOne(profile, "xl_fs", entry.text).ok());
    assertEnumEqual(entry.expected, profile.accelFullScale);
    assertValid(profile);
  }

  struct GyroFsCase {
    const char* text;
    GyroFs expected;
  };
  static constexpr GyroFsCase GYRO_FS_CASES[] = {
      {"125", GyroFs::DPS_125},   {"250", GyroFs::DPS_250},
      {"500", GyroFs::DPS_500},   {"1000", GyroFs::DPS_1000},
      {"2000", GyroFs::DPS_2000},
  };
  for (const GyroFsCase& entry : GYRO_FS_CASES) {
    DeviceProfile profile;
    TEST_ASSERT_TRUE(setOne(profile, "g_fs", entry.text).ok());
    assertEnumEqual(entry.expected, profile.gyroFullScale);
    assertValid(profile);
  }

  struct AccelPowerCase {
    const char* text;
    AccelPowerMode expected;
  };
  static constexpr AccelPowerCase ACCEL_POWER_CASES[] = {
      {"hp", AccelPowerMode::HIGH_PERFORMANCE},
      {"high", AccelPowerMode::HIGH_PERFORMANCE},
      {"lp", AccelPowerMode::LOW_POWER_NORMAL},
      {"low", AccelPowerMode::LOW_POWER_NORMAL},
  };
  for (const AccelPowerCase& entry : ACCEL_POWER_CASES) {
    DeviceProfile profile;
    TEST_ASSERT_TRUE(setOne(profile, "xl_power", entry.text).ok());
    assertEnumEqual(entry.expected, profile.accelPowerMode);
    assertValid(profile);
  }

  struct GyroPowerCase {
    const char* text;
    GyroPowerMode expected;
  };
  static constexpr GyroPowerCase GYRO_POWER_CASES[] = {
      {"hp", GyroPowerMode::HIGH_PERFORMANCE},
      {"high", GyroPowerMode::HIGH_PERFORMANCE},
      {"lp", GyroPowerMode::LOW_POWER_NORMAL},
      {"low", GyroPowerMode::LOW_POWER_NORMAL},
  };
  for (const GyroPowerCase& entry : GYRO_POWER_CASES) {
    DeviceProfile profile;
    TEST_ASSERT_TRUE(setOne(profile, "g_power", entry.text).ok());
    assertEnumEqual(entry.expected, profile.gyroPowerMode);
    assertValid(profile);
  }
}

void test_profile_cli_set_field_accepts_every_filter_offset_and_policy_value() {
  static constexpr const char* FALSE_TOKENS[] = {"0", "off", "false"};
  static constexpr const char* TRUE_TOKENS[] = {"1", "on", "true"};

  for (size_t enabled = 0; enabled < 2U; ++enabled) {
    const char* const* tokens = enabled == 0U ? FALSE_TOKENS : TRUE_TOKENS;
    for (size_t index = 0; index < 3U; ++index) {
      DeviceProfile profile;
      TEST_ASSERT_TRUE(setOne(profile, "xl_lpf2", tokens[index]).ok());
      TEST_ASSERT_EQUAL(enabled != 0U, profile.accelFilter.lpf2Enabled);
      assertValid(profile);

      profile = {};
      TEST_ASSERT_TRUE(setOne(profile, "g_lpf1", tokens[index]).ok());
      TEST_ASSERT_EQUAL(enabled != 0U, profile.gyroFilter.lpf1Enabled);
      assertValid(profile);

      profile = {};
      TEST_ASSERT_TRUE(setOne(profile, "g_sleep", tokens[index]).ok());
      TEST_ASSERT_EQUAL(enabled != 0U, profile.gyroSleepEnabled);
      assertValid(profile);
    }
  }

  const char* falseInvariantFields[] = {"xl_slope_hp", "xl_6d_lpf", "g_hpf",
                                        "fifo", "interrupts"};
  for (const char* field : falseInvariantFields) {
    for (const char* token : FALSE_TOKENS) {
      DeviceProfile profile;
      TEST_ASSERT_TRUE(setOne(profile, field, token).ok());
      assertValid(profile);
    }
  }
  for (const char* token : TRUE_TOKENS) {
    DeviceProfile profile;
    TEST_ASSERT_TRUE(setOne(profile, "bdu", token).ok());
    TEST_ASSERT_TRUE(profile.blockDataUpdate);
    assertValid(profile);
  }

  struct HpfCase {
    const char* text;
    GyroHpfMode expected;
  };
  static constexpr HpfCase HPF_CASES[] = {
      {"0.016", GyroHpfMode::HZ_0_016},
      {"0.065", GyroHpfMode::HZ_0_065},
      {"0.260", GyroHpfMode::HZ_0_260},
      {"1.040", GyroHpfMode::HZ_1_040},
  };
  for (const HpfCase& entry : HPF_CASES) {
    DeviceProfile profile;
    TEST_ASSERT_TRUE(setOne(profile, "g_hpf_mode", entry.text).ok());
    assertEnumEqual(entry.expected, profile.gyroFilter.highPassMode);
    TEST_ASSERT_FALSE(profile.gyroFilter.highPassEnabled);
    assertValid(profile);
  }

  DeviceProfile profile;
  TEST_ASSERT_TRUE(setOne(profile, "offset_weight", "1").ok());
  assertEnumEqual(AccelOffsetWeight::MG_1, profile.accelOffsetWeight);
  TEST_ASSERT_TRUE(setOne(profile, "offset_weight", "16").ok());
  assertEnumEqual(AccelOffsetWeight::MG_16, profile.accelOffsetWeight);

  const char* minimumOffset[] = {"-127", "-0x7f", "-127"};
  TEST_ASSERT_TRUE(
      profile_cli::setField(profile, "offset", minimumOffset, 3U).ok());
  TEST_ASSERT_EQUAL_INT8(-127, profile.accelUserOffset.x);
  TEST_ASSERT_EQUAL_INT8(-127, profile.accelUserOffset.y);
  TEST_ASSERT_EQUAL_INT8(-127, profile.accelUserOffset.z);
  const char* maximumOffset[] = {"127", "0x7f", "+127"};
  TEST_ASSERT_TRUE(
      profile_cli::setField(profile, "offset", maximumOffset, 3U).ok());
  TEST_ASSERT_EQUAL_INT8(127, profile.accelUserOffset.x);
  TEST_ASSERT_EQUAL_INT8(127, profile.accelUserOffset.y);
  TEST_ASSERT_EQUAL_INT8(127, profile.accelUserOffset.z);
  const char* mixedOffset[] = {"-1", "0", "1"};
  TEST_ASSERT_TRUE(
      profile_cli::setField(profile, "offset", mixedOffset, 3U).ok());
  TEST_ASSERT_EQUAL_INT8(-1, profile.accelUserOffset.x);
  TEST_ASSERT_EQUAL_INT8(0, profile.accelUserOffset.y);
  TEST_ASSERT_EQUAL_INT8(1, profile.accelUserOffset.z);
  assertValid(profile);
}

void test_profile_cli_set_field_rejects_every_bad_arity_atomically() {
  struct OneValueField {
    const char* field;
    const char* valid;
  };
  static constexpr OneValueField FIELDS[] = {
      {"xl_odr", "104"},       {"xl_fs", "2"},
      {"xl_power", "hp"},      {"xl_lpf2", "0"},
      {"xl_slope_hp", "0"},    {"xl_6d_lpf", "0"},
      {"g_odr", "104"},        {"g_fs", "250"},
      {"g_power", "hp"},       {"g_lpf1", "0"},
      {"g_hpf", "0"},          {"g_hpf_mode", "0.016"},
      {"g_sleep", "0"},        {"bdu", "1"},
      {"offset_weight", "1"},  {"fifo", "0"},
      {"interrupts", "0"},
  };
  for (const OneValueField& entry : FIELDS) {
    DeviceProfile profile;
    const char* values[] = {entry.valid, entry.valid, entry.valid};
    assertAtomicError(profile, entry.field, values, 0U, Err::INVALID_PARAM);
    assertAtomicError(profile, entry.field, values, 2U, Err::INVALID_PARAM);
    assertAtomicError(profile, entry.field, values, 3U, Err::INVALID_PARAM);
  }

  DeviceProfile profile;
  const char* offsetValues[] = {"0", "0", "0", "0"};
  assertAtomicError(profile, "offset", offsetValues, 0U, Err::INVALID_PARAM);
  assertAtomicError(profile, "offset", offsetValues, 1U, Err::INVALID_PARAM);
  assertAtomicError(profile, "offset", offsetValues, 2U, Err::INVALID_PARAM);
  assertAtomicError(profile, "offset", offsetValues, 4U, Err::INVALID_PARAM);

  const char* oneValue[] = {"104"};
  assertAtomicError(profile, nullptr, oneValue, 1U, Err::INVALID_PARAM);
  assertAtomicError(profile, "xl_odr", nullptr, 1U, Err::INVALID_PARAM);
  assertAtomicError(profile, "", oneValue, 1U, Err::INVALID_PARAM);
  assertAtomicError(profile, "unknown", oneValue, 1U, Err::INVALID_PARAM);
}

void test_profile_cli_set_field_rejects_every_bad_token_atomically() {
  struct InvalidCase {
    const char* field;
    const char* invalid;
  };
  static constexpr InvalidCase CASES[] = {
      {"xl_odr", "0"},          {"xl_fs", "3"},
      {"xl_power", "normal"},   {"xl_lpf2", "2"},
      {"xl_slope_hp", "2"},     {"xl_6d_lpf", "2"},
      {"g_odr", "0"},           {"g_fs", "124"},
      {"g_power", "normal"},    {"g_lpf1", "2"},
      {"g_hpf", "2"},           {"g_hpf_mode", "0.26"},
      {"g_sleep", "2"},         {"bdu", "2"},
      {"offset_weight", "2"},   {"fifo", "2"},
      {"interrupts", "2"},
  };
  for (const InvalidCase& entry : CASES) {
    DeviceProfile profile;
    const char* value[] = {entry.invalid};
    assertAtomicError(profile, entry.field, value, 1U, Err::INVALID_PARAM);
  }

  const char* invalidOffset[][3] = {
      {"-128", "0", "0"}, {"0", "128", "0"}, {"0", "0", "bad"}};
  for (const auto& values : invalidOffset) {
    DeviceProfile profile;
    assertAtomicError(profile, "offset", values, 3U, Err::INVALID_PARAM);
  }

  const char* fields[] = {
      "xl_odr",      "xl_fs",     "xl_power",  "xl_lpf2",
      "xl_slope_hp", "xl_6d_lpf", "g_odr",     "g_fs",
      "g_power",     "g_lpf1",    "g_hpf",     "g_hpf_mode",
      "g_sleep",     "bdu",       "offset_weight", "fifo",
      "interrupts",
  };
  const char* nullValue[] = {nullptr};
  for (const char* field : fields) {
    DeviceProfile profile;
    assertAtomicError(profile, field, nullValue, 1U, Err::INVALID_PARAM);
  }
  const char* nullOffsets[][3] = {
      {nullptr, "0", "0"}, {"0", nullptr, "0"}, {"0", "0", nullptr}};
  for (const auto& values : nullOffsets) {
    DeviceProfile profile;
    assertAtomicError(profile, "offset", values, 3U, Err::INVALID_PARAM);
  }
}

void test_profile_cli_set_field_rejects_all_production_invariants_atomically() {
  const char* fields[] = {"xl_slope_hp", "xl_6d_lpf", "g_hpf", "fifo",
                          "interrupts"};
  const char* trueTokens[] = {"1", "on", "true"};
  for (const char* field : fields) {
    for (const char* token : trueTokens) {
      DeviceProfile profile;
      const char* values[] = {token};
      assertAtomicError(profile, field, values, 1U,
                        Err::UNSUPPORTED_PROFILE);
    }
  }
  const char* falseTokens[] = {"0", "off", "false"};
  for (const char* token : falseTokens) {
    DeviceProfile profile;
    const char* values[] = {token};
    assertAtomicError(profile, "bdu", values, 1U,
                      Err::UNSUPPORTED_PROFILE);
  }
}

void test_profile_cli_set_field_enforces_all_cross_field_rules_atomically() {
  {
    DeviceProfile profile;
    TEST_ASSERT_TRUE(setOne(profile, "xl_power", "lp").ok());
    TEST_ASSERT_TRUE(setOne(profile, "xl_odr", "208").ok());
    assertValid(profile);
  }
  {
    DeviceProfile profile;
    TEST_ASSERT_TRUE(setOne(profile, "xl_odr", "208").ok());
    TEST_ASSERT_TRUE(setOne(profile, "xl_power", "lp").ok());
    assertValid(profile);
  }
  {
    DeviceProfile profile;
    const char* value[] = {"1.6"};
    assertAtomicError(profile, "xl_odr", value, 1U, Err::INVALID_CONFIG);
  }
  {
    DeviceProfile profile;
    TEST_ASSERT_TRUE(setOne(profile, "xl_power", "lp").ok());
    TEST_ASSERT_TRUE(setOne(profile, "xl_odr", "1.6").ok());
    const char* value[] = {"hp"};
    assertAtomicError(profile, "xl_power", value, 1U, Err::INVALID_CONFIG);
  }
  const char* highOdrs[] = {"416", "833", "1660", "3330", "6660"};
  for (const char* highOdr : highOdrs) {
    DeviceProfile lowPower;
    TEST_ASSERT_TRUE(setOne(lowPower, "xl_power", "lp").ok());
    const char* value[] = {highOdr};
    assertAtomicError(lowPower, "xl_odr", value, 1U, Err::INVALID_CONFIG);

    DeviceProfile highRate;
    TEST_ASSERT_TRUE(setOne(highRate, "xl_odr", highOdr).ok());
    const char* lp[] = {"lp"};
    assertAtomicError(highRate, "xl_power", lp, 1U, Err::INVALID_CONFIG);
  }
  {
    DeviceProfile profile;
    const char* value[] = {"1.6"};
    assertAtomicError(profile, "g_odr", value, 1U, Err::INVALID_PARAM);
  }
  {
    DeviceProfile profile;
    TEST_ASSERT_TRUE(setOne(profile, "g_power", "lp").ok());
    TEST_ASSERT_TRUE(setOne(profile, "g_odr", "208").ok());
    assertValid(profile);
  }
  {
    DeviceProfile profile;
    TEST_ASSERT_TRUE(setOne(profile, "g_odr", "208").ok());
    TEST_ASSERT_TRUE(setOne(profile, "g_power", "lp").ok());
    assertValid(profile);
  }
  for (const char* highOdr : highOdrs) {
    DeviceProfile lowPower;
    TEST_ASSERT_TRUE(setOne(lowPower, "g_power", "lp").ok());
    const char* value[] = {highOdr};
    assertAtomicError(lowPower, "g_odr", value, 1U, Err::INVALID_CONFIG);

    DeviceProfile highRate;
    TEST_ASSERT_TRUE(setOne(highRate, "g_odr", highOdr).ok());
    const char* lp[] = {"lp"};
    assertAtomicError(highRate, "g_power", lp, 1U, Err::INVALID_CONFIG);
  }
  {
    DeviceProfile profile;
    TEST_ASSERT_TRUE(setOne(profile, "g_lpf1", "1").ok());
    const char* value[] = {"lp"};
    assertAtomicError(profile, "g_power", value, 1U,
                      Err::UNSUPPORTED_PROFILE);
  }
  {
    DeviceProfile profile;
    TEST_ASSERT_TRUE(setOne(profile, "g_power", "lp").ok());
    const char* value[] = {"1"};
    assertAtomicError(profile, "g_lpf1", value, 1U,
                      Err::UNSUPPORTED_PROFILE);
  }
}

}  // namespace

void runProfileCliTests() {
  RUN_TEST(test_profile_cli_names_cover_every_enum_value);
  RUN_TEST(test_profile_cli_equal_compares_every_profile_field);
  RUN_TEST(test_profile_cli_bool_and_signed_parsers_are_total_and_atomic);
  RUN_TEST(test_profile_cli_odr_parser_covers_all_tokens_and_is_atomic);
  RUN_TEST(test_profile_cli_range_power_and_hpf_parsers_cover_all_tokens);
  RUN_TEST(test_profile_cli_set_field_accepts_every_odr_range_and_power_value);
  RUN_TEST(test_profile_cli_set_field_accepts_every_filter_offset_and_policy_value);
  RUN_TEST(test_profile_cli_set_field_rejects_every_bad_arity_atomically);
  RUN_TEST(test_profile_cli_set_field_rejects_every_bad_token_atomically);
  RUN_TEST(test_profile_cli_set_field_rejects_all_production_invariants_atomically);
  RUN_TEST(test_profile_cli_set_field_enforces_all_cross_field_rules_atomically);
}
