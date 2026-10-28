/// @file test_filters.cpp
/// @brief Independent vendor-table, wire-image and filter-provenance regressions.
#include <unity.h>

#include <cstdint>
#include <limits>

#include "LSM6DS3TR/LSM6DS3TR.h"
#include "support/FakeBus.h"

using namespace LSM6DS3TR;
using namespace LSM6DS3TRTest;

namespace {

OperationResult configureFilter(LSM6DS3TR::LSM6DS3TR& driver, FakeBus& bus,
                                const DeviceProfile& profile) {
  OperationToken token{};
  TEST_ASSERT_TRUE(driver.startConfigure(profile,
      {bus.nowMs, bus.nowMs + 1000000U}, token).inProgress());
  for (uint16_t i = 0; i < 2000U && driver.operationActive(); ++i) {
    const uint32_t before = bus.transferCalls;
    const PollResult result = driver.poll(bus.nowMs, 7);
    TEST_ASSERT_EQUAL_UINT32(bus.transferCalls - before, result.transactionsUsed);
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(7, result.transactionsUsed);
    bus.nowMs += 1000U;
  }
  TEST_ASSERT_FALSE(driver.operationActive());
  OperationResult result{};
  TEST_ASSERT_TRUE(driver.takeResult(token, result).ok());
  TEST_ASSERT_TRUE(result.status.ok());
  return result;
}

void rejectFilter(const DeviceProfile& profile) {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  OperationToken token{99};
  TEST_ASSERT_FALSE(validateProfile(profile).ok());
  TEST_ASSERT_FALSE(driver.startConfigure(profile,
      {bus.nowMs, bus.nowMs + 1000000U}, token).inProgress());
  TEST_ASSERT_FALSE(token.valid());
  TEST_ASSERT_EQUAL_UINT32(0, bus.transferCalls);
  TEST_ASSERT_EQUAL_UINT64(std::numeric_limits<uint64_t>::max(), requiredSettleUs(profile));
}

void test_accel_filter_table_wire_images_and_settle_counts() {
  // Independently transcribed AN5130 Table 9, p.14; CTRL8_XL bits p.67.
  struct Row { bool lpf2; bool highPass; uint8_t hpcf; uint8_t ctrl8; uint16_t samples; };
  const Row rows[] = {
      {false, false, 0, 0x00, 14},
      {true, false, 0, 0x80, 40}, {true, false, 1, 0xA0, 80},
      {true, false, 2, 0xC0, 15}, {true, false, 3, 0xE0, 320},
      {false, true, 0, 0x04, 14}, {false, true, 1, 0x24, 80},
      {false, true, 2, 0x44, 15}, {false, true, 3, 0x64, 320}};
  for (const Row& row : rows) {
    for (uint8_t input = 0; input <= (row.lpf2 ? 1U : 0U); ++input) {
      DeviceProfile profile{};
      profile.gyroOdr = Odr::POWER_DOWN;
      profile.accelFilter.lpf2Enabled = row.lpf2;
      profile.accelFilter.highPassSlopeEnabled = row.highPass;
      profile.accelFilter.hpcf = static_cast<AccelHpcf>(row.hpcf);
      profile.accelFilter.compositeInput = static_cast<AccelCompositeInput>(input);
      TEST_ASSERT_TRUE(validateProfile(profile).ok());
      TEST_ASSERT_EQUAL_UINT64(9616ULL * row.samples, requiredSettleUs(profile));
      FakeBus bus;
      LSM6DS3TR::LSM6DS3TR driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      configureFilter(driver, bus, profile);
      TEST_ASSERT_EQUAL_HEX8(row.ctrl8 | (input ? 0x08 : 0), bus.regs[0x17]);
      TEST_ASSERT_EQUAL_HEX8(0x40, bus.regs[0x10]);
      TEST_ASSERT_EQUAL_HEX8(0, bus.regs[0x15]);
      TEST_ASSERT_EQUAL_HEX8(0, bus.regs[0x16]);
      TEST_ASSERT_EQUAL_HEX8(0, bus.regs[0x17] & 0x02U); // Reserved bit stays zero.
    }
  }
  DeviceProfile profile{};
  profile.gyroOdr = Odr::POWER_DOWN;
  profile.accelOdr = Odr::HZ_1660;
  profile.accelFilter.lpf1Bandwidth = AccelLpf1Bandwidth::ODR_DIV_4;
  profile.accelFilter.analogBandwidth = AccelAnalogBandwidth::HZ_400;
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  configureFilter(driver, bus, profile);
  TEST_ASSERT_EQUAL_HEX8(0x83, bus.regs[0x10]);
  TEST_ASSERT_EQUAL_HEX8(0, bus.regs[0x17]);
}

void test_accel_filter_power_odr_legality_matrix() {
  for (uint8_t mode = 0; mode < 2U; ++mode) {
    for (uint8_t odr = 0; odr < 12U; ++odr) {
      const bool legal = odr == 11U ? mode == 1U : mode == 0U || odr <= 5U;
      for (uint8_t path = 0; path < 3U; ++path) {
        for (uint8_t hpcf = 0; hpcf < 4U; ++hpcf) {
          DeviceProfile profile{};
          profile.gyroOdr = Odr::POWER_DOWN;
          profile.accelOdr = static_cast<Odr>(odr);
          profile.accelPowerMode = static_cast<AccelPowerMode>(mode);
          profile.accelFilter.lpf2Enabled = path == 1U;
          profile.accelFilter.highPassSlopeEnabled = path == 2U;
          profile.accelFilter.hpcf = static_cast<AccelHpcf>(hpcf);
          TEST_ASSERT_EQUAL(legal && (path != 0U || hpcf == 0U), validateProfile(profile).ok());
        }
      }
    }
  }
}

void test_gyro_ftype_table_wire_images_and_settle_counts() {
  // AN5130 Tables 15/16 p.21: no production helper computes these fixtures.
  const uint16_t counts[10][4] = {
      {2,2,2,2}, {3,3,3,3}, {3,3,3,3}, {4,4,4,4}, {4,4,5,4},
      {5,6,6,5}, {7,8,9,6}, {135,135,135,135}, {270,270,270,270},
      {540,540,540,540}};
  const uint32_t periods[] = {80000,38462,19231,9616,4808,2404,1201,603,301,151};
  for (uint8_t odr = 1; odr <= 10U; ++odr) {
    for (uint8_t ftype = 0; ftype < 4U; ++ftype) {
      DeviceProfile profile{};
      profile.accelOdr = Odr::POWER_DOWN;
      profile.gyroOdr = static_cast<Odr>(odr);
      profile.gyroFilter.lpf1Enabled = true;
      profile.gyroFilter.lpf1Bandwidth = static_cast<GyroLpf1Bandwidth>(ftype);
      TEST_ASSERT_EQUAL_UINT64(70000ULL + uint64_t(periods[odr-1]) * counts[odr-1][ftype],
                               requiredSettleUs(profile));
      FakeBus bus;
      LSM6DS3TR::LSM6DS3TR driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      configureFilter(driver, bus, profile);
      TEST_ASSERT_EQUAL_HEX8(0x02, bus.regs[0x13]);
      TEST_ASSERT_EQUAL_HEX8(ftype, bus.regs[0x15]);
      TEST_ASSERT_EQUAL_HEX8(0, bus.regs[0x01]); // FTYPE never belongs in bank selection.
      TEST_ASSERT_EQUAL_HEX8(0, bus.regs[0x15] & 0xE4U);
    }
  }
}

void test_gyro_hpf_and_power_legality_matrix() {
  for (uint8_t mode = 0; mode < 2U; ++mode) {
    for (uint8_t odr = 0; odr < 12U; ++odr) {
      for (uint8_t hpf = 0; hpf < 4U; ++hpf) {
        DeviceProfile profile{};
        profile.gyroOdr = static_cast<Odr>(odr);
        profile.gyroPowerMode = static_cast<GyroPowerMode>(mode);
        profile.gyroFilter.highPassEnabled = true;
        profile.gyroFilter.highPassMode = static_cast<GyroHpfMode>(hpf);
        const bool legal = mode == 0U && odr >= 1U && odr <= 10U;
        TEST_ASSERT_EQUAL(legal, validateProfile(profile).ok());
        if (!legal) continue;
        TEST_ASSERT_EQUAL_UINT8(SAMPLE_ACCELERATION | SAMPLE_TEMPERATURE,
                               qualifiedQuantityMask(profile));
        FakeBus bus;
        LSM6DS3TR::LSM6DS3TR driver;
        TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
        configureFilter(driver, bus, profile);
        TEST_ASSERT_EQUAL_HEX8(0x40 | (hpf << 4U), bus.regs[0x16]);
        TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::FILTER_UNQUALIFIED),
                               static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
      }
    }
  }
}

void test_filter_contradictions_and_invalid_enums_reject_without_io() {
  DeviceProfile p{};
  p.accelFilter.lpf2Enabled = p.accelFilter.highPassSlopeEnabled = true; rejectFilter(p);
  p = {}; p.accelFilter.hpcf = AccelHpcf::DIV_400; rejectFilter(p);
  p = {}; p.accelFilter.compositeInput = AccelCompositeInput::LOW_NOISE; rejectFilter(p);
  p = {}; p.accelFilter.lpf2Enabled = true;
  p.accelFilter.lpf1Bandwidth = AccelLpf1Bandwidth::ODR_DIV_4; rejectFilter(p);
  p = {}; p.accelFilter.highPassSlopeEnabled = true;
  p.accelFilter.compositeInput = AccelCompositeInput::LOW_NOISE; rejectFilter(p);
  p = {}; p.accelFilter.referenceEnabled = true; rejectFilter(p);
  p.accelFilter.highPassSlopeEnabled = true; rejectFilter(p);
  p.accelFilter.hpcf = AccelHpcf::DIV_9; rejectFilter(p);
  p.accelFilter.hpcf = AccelHpcf::DIV_100;
  p.accelFilter.lowPassOn6d = true; rejectFilter(p);
  p = {}; p.accelFilter.analogBandwidth = AccelAnalogBandwidth::HZ_400; rejectFilter(p);
  p = {}; p.accelFilter.lowPassOn6d = true; rejectFilter(p);
  p = {}; p.gyroFilter.lpf1Bandwidth = GyroLpf1Bandwidth::FTYPE_01; rejectFilter(p);
  p = {}; p.gyroFilter.highPassMode = GyroHpfMode::HZ_1_040; rejectFilter(p);
  p.gyroFilter.highPassEnabled = true; p.gyroSleepEnabled = true; rejectFilter(p);
  p = {}; p.gyroFilter.highPassEnabled = true;
  p.fifo.enabled = true; p.fifo.odr = Odr::HZ_104;
  p.fifo.gyroDecimation = FifoDecimation::EVERY_1; rejectFilter(p);
  p = {}; p.accelFilter.hpcf = static_cast<AccelHpcf>(4); rejectFilter(p);
  p = {}; p.accelFilter.lpf1Bandwidth = static_cast<AccelLpf1Bandwidth>(2); rejectFilter(p);
  p = {}; p.accelFilter.compositeInput = static_cast<AccelCompositeInput>(2); rejectFilter(p);
  p = {}; p.accelFilter.analogBandwidth = static_cast<AccelAnalogBandwidth>(2); rejectFilter(p);
  p = {}; p.gyroFilter.lpf1Bandwidth = static_cast<GyroLpf1Bandwidth>(4); rejectFilter(p);
  p = {}; p.gyroFilter.highPassMode = static_cast<GyroHpfMode>(4); rejectFilter(p);
}

void test_orientation_filter_is_independent_and_extends_gate() {
  for (uint8_t host = 0; host < 3U; ++host) {
    for (uint8_t hpcf = 0; hpcf < 4U; ++hpcf) {
      DeviceProfile p{};
      p.gyroOdr = Odr::POWER_DOWN;
      p.accelFilter.lpf2Enabled = host == 1U;
      p.accelFilter.highPassSlopeEnabled = host == 2U;
      p.accelFilter.lowPassOn6d = true;
      p.accelFilter.hpcf = static_cast<AccelHpcf>(hpcf);
      p.interrupts.enabled = true;
      p.interrupts.orientation.mode = OrientationMode::SIX_D;
      p.interrupts.int1.orientation = true;
      const uint16_t counts[] = {40,80,15,320};
      TEST_ASSERT_EQUAL_UINT64(9616ULL * counts[hpcf], requiredSettleUs(p));
      FakeBus bus;
      LSM6DS3TR::LSM6DS3TR driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      configureFilter(driver, bus, p);
      TEST_ASSERT_EQUAL_HEX8(1U | (hpcf << 5U) | (host == 1U ? 0x80U : 0U) |
                              (host == 2U ? 0x04U : 0U), bus.regs[0x17]);
      TEST_ASSERT_EQUAL_HEX8(0x04, bus.regs[0x5E]);
      TEST_ASSERT_EQUAL_HEX8(0x80, bus.regs[0x58]);
    }
  }
}

void test_low_odr_and_transition_bounds_are_source_based() {
  DeviceProfile p{};
  p.gyroOdr = Odr::POWER_DOWN;
  p.accelOdr = Odr::HZ_1_6;
  p.accelPowerMode = AccelPowerMode::LOW_POWER_NORMAL;
  p.accelFilter.lpf2Enabled = true;
  p.accelFilter.hpcf = AccelHpcf::DIV_400;
  TEST_ASSERT_EQUAL_UINT64(200000000ULL, requiredSettleUs(p));
  p.accelFilter = {};
  TEST_ASSERT_EQUAL_UINT64(8750000ULL, requiredSettleUs(p));
  p.accelOdr = Odr::HZ_6660; p.accelPowerMode = AccelPowerMode::HIGH_PERFORMANCE;
  TEST_ASSERT_EQUAL_UINT64(2114, requiredSettleUs(p)); // 13+1 transition <=14.
  p.accelOdr = Odr::POWER_DOWN; p.gyroOdr = Odr::HZ_12_5;
  p.gyroPowerMode = GyroPowerMode::LOW_POWER_NORMAL;
  TEST_ASSERT_EQUAL_UINT64(150000, requiredSettleUs(p));
  p.gyroOdr = Odr::HZ_208;
  TEST_ASSERT_EQUAL_UINT64(74808, requiredSettleUs(p));
  p.gyroSleepEnabled = true;
  TEST_ASSERT_EQUAL_UINT64(70000, requiredSettleUs(p));
  p.gyroOdr = Odr::POWER_DOWN;
  TEST_ASSERT_EQUAL_UINT64(0, requiredSettleUs(p));
}

void test_conversion_preserves_filter_provenance_and_rejects_false_hpf_quality() {
  RawSampleResult raw{};
  raw.validMask = SAMPLE_ALL;
  raw.quality = SampleQuality::FILTER_UNVERIFIED;
  raw.qualifiedMask = SAMPLE_ACCELERATION | SAMPLE_TEMPERATURE;
  raw.filters.gyro.highPassEnabled = true;
  raw.filters.gyro.highPassMode = GyroHpfMode::HZ_0_260;
  raw.filters.accel.lpf2Enabled = true;
  raw.filters.accel.hpcf = AccelHpcf::DIV_400;
  raw.gyro = {12,-123,32767};
  raw.accel = {-32768,2,3};
  raw.temperatureRaw = -256;
  ConvertedSample output{};
  TEST_ASSERT_TRUE(convertSample(raw, output).ok());
  TEST_ASSERT_EQUAL_UINT8(raw.qualifiedMask, output.qualifiedMask);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelHpcf::DIV_400),
      static_cast<uint8_t>(output.filters.accel.hpcf));
  TEST_ASSERT_EQUAL_INT64(-123 * 8750LL, output.gyroMicroDps.y);
  TEST_ASSERT_EQUAL_INT32(24000, output.temperatureMilliC);
  raw.freshMask = raw.validMask;
  TEST_ASSERT_FALSE(convertSample(raw, output).ok());
  raw.quality = SampleQuality::READY_CHECKED;
  raw.qualifiedMask = SAMPLE_ALL;
  TEST_ASSERT_FALSE(convertSample(raw, output).ok());
  raw.quality = SampleQuality::DIRECT_UNVERIFIED;
  raw.freshMask = 0;
  TEST_ASSERT_FALSE(convertSample(raw, output).ok());
  raw.filters.gyro.highPassEnabled = false;
  raw.filters.accel.hpcf = static_cast<AccelHpcf>(255);
  TEST_ASSERT_FALSE(convertSample(raw, output).ok());
}

} // namespace

void runFilterProfileTests() {
  RUN_TEST(test_accel_filter_table_wire_images_and_settle_counts);
  RUN_TEST(test_accel_filter_power_odr_legality_matrix);
  RUN_TEST(test_gyro_ftype_table_wire_images_and_settle_counts);
  RUN_TEST(test_gyro_hpf_and_power_legality_matrix);
  RUN_TEST(test_filter_contradictions_and_invalid_enums_reject_without_io);
  RUN_TEST(test_orientation_filter_is_independent_and_extends_gate);
  RUN_TEST(test_low_odr_and_transition_bounds_are_source_based);
  RUN_TEST(test_conversion_preserves_filter_provenance_and_rejects_false_hpf_quality);
}
