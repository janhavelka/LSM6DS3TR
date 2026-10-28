/// @file test_filter_operations.cpp
/// @brief Filter qualification, discard, restoration and immutable provenance.
#include <unity.h>

#include <cstdint>
#include <initializer_list>
#include <limits>

#include "LSM6DS3TR/LSM6DS3TR.h"
#include "support/FakeBus.h"

using namespace LSM6DS3TR;
using namespace LSM6DS3TRTest;

namespace {
using Driver = LSM6DS3TR::LSM6DS3TR;

OperationTiming filterTiming(const FakeBus& bus) {
  return {bus.nowMs, bus.nowMs + 600000U};
}

void filterRun(Driver& driver, FakeBus& bus, uint32_t increment = 1U) {
  const uint8_t budgets[] = {0, 1, 255, 2, 0, 7};
  for (uint32_t i = 0; driver.operationActive() && i < 610000U; ++i) {
    const uint8_t budget = budgets[i % 6U];
    const uint32_t before = bus.transferCalls;
    const PollResult result = driver.poll(bus.nowMs, budget);
    TEST_ASSERT_EQUAL_UINT32(bus.transferCalls - before, result.transactionsUsed);
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(budget, result.transactionsUsed);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(result.transactionLimit, result.transactions);
    bus.nowMs += increment;
  }
  TEST_ASSERT_FALSE(driver.operationActive());
  TEST_ASSERT_TRUE(driver.resultPending());
}

OperationResult filterTake(Driver& driver, OperationToken token) {
  OperationResult result;
  TEST_ASSERT_TRUE(driver.takeResult(token, result).ok());
  TEST_ASSERT_EQUAL_UINT64(token.value, result.token.value);
  OperationResult untouched;
  TEST_ASSERT_FALSE(driver.takeResult(token, untouched).ok());
  return result;
}

OperationResult filterConfigure(Driver& driver, FakeBus& bus,
                                const DeviceProfile& profile) {
  OperationToken token;
  TEST_ASSERT_TRUE_MESSAGE(driver.startConfigure(profile, filterTiming(bus), token).inProgress(),
                           "filter profile rejected");
  filterRun(driver, bus);
  const OperationResult result = filterTake(driver, token);
  TEST_ASSERT_TRUE_MESSAGE(result.status.ok(), result.status.msg);
  return result;
}

DeviceProfile referenceProfile(bool fifo = false, bool counters = false) {
  DeviceProfile profile;
  profile.accelFilter.highPassSlopeEnabled = true;
  profile.accelFilter.hpcf = AccelHpcf::DIV_100;
  profile.accelFilter.referenceEnabled = true;
  if (fifo || counters) {
    profile.fifo.enabled = true;
    profile.fifo.mode = fifo ? FifoMode::CONTINUOUS : FifoMode::BYPASS;
    profile.fifo.odr = Odr::HZ_104;
    profile.fifo.accelDecimation = FifoDecimation::EVERY_1;
  }
  if (counters) {
    profile.fifo.fourthDataset = FifoFourthDataset::TIMESTAMP_STEP;
    profile.fifo.fourthDecimation = FifoDecimation::EVERY_1;
    profile.fifo.timestampResolution = TimestampResolution::US_25;
  }
  profile.interrupts.enabled = true;
  profile.interrupts.orientation.mode = OrientationMode::SIX_D;
  profile.interrupts.int1.orientation = true;
  return profile;
}

DeviceProfile hpfProfile() {
  DeviceProfile profile;
  profile.gyroFilter.highPassEnabled = true;
  profile.gyroFilter.highPassMode = GyroHpfMode::HZ_0_260;
  profile.gyroFilter.lpf1Enabled = true;
  profile.gyroFilter.lpf1Bandwidth = GyroLpf1Bandwidth::FTYPE_10;
  return profile;
}

size_t filterFindRead(const FakeBus& bus, uint8_t reg, size_t first = 0) {
  for (size_t i = first; i < bus.traceCount; ++i)
    if (bus.trace[i].kind == TransferKind::WRITE_READ && bus.trace[i].startReg == reg)
      return i;
  return bus.traceCount;
}

size_t filterFindWrite(const FakeBus& bus, uint8_t reg, uint8_t value,
                       size_t first = 0) {
  for (size_t i = first; i < bus.traceCount; ++i)
    if (bus.trace[i].kind == TransferKind::WRITE && bus.trace[i].startReg == reg &&
        bus.trace[i].firstValue == value) return i;
  return bus.traceCount;
}

OperationResult filterSample(Driver& driver, FakeBus& bus, SampleRequest request) {
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startSample(request, filterTiming(bus), token).inProgress());
  filterRun(driver, bus);
  return filterTake(driver, token);
}

void test_filter_hpf_requires_explicit_direct_opt_in_for_every_gyro_request() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  const OperationResult configured = filterConfigure(driver, bus, hpfProfile());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::FILTER_UNQUALIFIED),
                        static_cast<uint8_t>(configured.configuration.state));
  TEST_ASSERT_EQUAL_UINT8(SAMPLE_ACCELERATION | SAMPLE_TEMPERATURE,
                        configured.configuration.qualifiedMask);
  const uint32_t before = bus.transferCalls;
  for (uint8_t mask : {SAMPLE_ANGULAR_RATE,
      static_cast<uint8_t>(SAMPLE_ANGULAR_RATE | SAMPLE_ACCELERATION), SAMPLE_ALL}) {
    for (bool ready : {false, true}) {
      OperationToken token{999};
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::UNSUPPORTED_PROFILE),
          static_cast<uint8_t>(driver.startSample({mask, ready, false}, filterTiming(bus), token).code));
      TEST_ASSERT_FALSE(token.valid());
    }
    OperationToken token;
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
        static_cast<uint8_t>(driver.startSample({mask, true, true}, filterTiming(bus), token).code));
  }
  TEST_ASSERT_EQUAL_UINT32(before, bus.transferCalls);
  const OperationResult sample = filterSample(driver, bus, {SAMPLE_ALL, false, true});
  TEST_ASSERT_TRUE(sample.status.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleQuality::FILTER_UNVERIFIED),
                        static_cast<uint8_t>(sample.sample.quality));
  TEST_ASSERT_EQUAL_UINT8(SAMPLE_ALL, sample.sample.validMask);
  TEST_ASSERT_EQUAL_UINT8(0, sample.sample.freshMask);
  TEST_ASSERT_EQUAL_UINT8(SAMPLE_ACCELERATION | SAMPLE_TEMPERATURE,
                        sample.sample.qualifiedMask);
  TEST_ASSERT_EQUAL_INT16(1000, sample.sample.gyro.x);
  TEST_ASSERT_TRUE(sample.sample.filters.gyro.highPassEnabled);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(GyroHpfMode::HZ_0_260),
                        static_cast<uint8_t>(sample.sample.filters.gyro.highPassMode));

  const OperationResult accel = filterSample(driver, bus,
      {static_cast<uint8_t>(SAMPLE_ACCELERATION | SAMPLE_TEMPERATURE), true, false});
  TEST_ASSERT_TRUE(accel.status.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleQuality::READY_CHECKED),
                        static_cast<uint8_t>(accel.sample.quality));
  TEST_ASSERT_EQUAL_UINT8(SAMPLE_ACCELERATION | SAMPLE_TEMPERATURE, accel.sample.freshMask);
  TEST_ASSERT_EQUAL_UINT8(accel.sample.validMask, accel.sample.qualifiedMask);
  bus.regs[0x1E] = 0x03; // motion ready must not manufacture temperature freshness
  const OperationResult noTemperature = filterSample(driver, bus,
      {static_cast<uint8_t>(SAMPLE_ACCELERATION | SAMPLE_TEMPERATURE), true, false});
  TEST_ASSERT_FALSE(noTemperature.status.ok());
  TEST_ASSERT_EQUAL_UINT8(0, noTemperature.sample.validMask);
}

void test_filter_old_sample_conversion_retains_qualification_and_path() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  DeviceProfile original = hpfProfile();
  original.accelFilter.lpf2Enabled = true;
  original.accelFilter.hpcf = AccelHpcf::DIV_400;
  original.accelFullScale = AccelFs::G_8;
  original.gyroFullScale = GyroFs::DPS_1000;
  filterConfigure(driver, bus, original);
  const RawSampleResult old = filterSample(driver, bus, {SAMPLE_ALL, false, true}).sample;
  ConvertedSample first;
  TEST_ASSERT_TRUE(convertSample(old, first).ok());
  filterConfigure(driver, bus, DeviceProfile{});
  ConvertedSample after;
  TEST_ASSERT_TRUE(convertSample(old, after).ok());
  TEST_ASSERT_EQUAL_INT32(first.gyroMicroDps.x, after.gyroMicroDps.x);
  TEST_ASSERT_EQUAL_INT32(first.accelMicroG.z, after.accelMicroG.z);
  TEST_ASSERT_EQUAL_UINT32(old.configGeneration, after.configGeneration);
  TEST_ASSERT_TRUE(after.filters.gyro.highPassEnabled);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelHpcf::DIV_400),
                        static_cast<uint8_t>(after.filters.accel.hpcf));
  TEST_ASSERT_EQUAL_UINT8(old.qualifiedMask, after.qualifiedMask);
  TEST_ASSERT_EQUAL_UINT8(0, after.freshMask);
  RawSampleResult forged = old;
  forged.quality = SampleQuality::READY_CHECKED;
  forged.freshMask = forged.validMask;
  TEST_ASSERT_FALSE(convertSample(forged, after).ok());
  forged = old;
  forged.qualifiedMask = SAMPLE_ALL;
  TEST_ASSERT_FALSE(convertSample(forged, after).ok());
}

void test_filter_hpf_fifo_exclusion_is_per_channel_and_records_keep_provenance() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  DeviceProfile profile = hpfProfile();
  profile.fifo.enabled = true;
  profile.fifo.mode = FifoMode::CONTINUOUS;
  profile.fifo.odr = Odr::HZ_104;
  profile.fifo.gyroDecimation = FifoDecimation::EVERY_1;
  OperationToken token;
  TEST_ASSERT_FALSE(driver.startConfigure(profile, filterTiming(bus), token).inProgress());
  TEST_ASSERT_EQUAL_UINT32(0, bus.transferCalls);
  profile.fifo.gyroDecimation = FifoDecimation::OFF;
  profile.fifo.accelDecimation = FifoDecimation::EVERY_1;
  profile.accelFilter.highPassSlopeEnabled = true;
  profile.accelFilter.hpcf = AccelHpcf::DIV_9;
  const OperationResult configured = filterConfigure(driver, bus, profile);
  const int16_t words[] = {111, 222, 333, -101, 202, -303};
  bus.setFifoWords(words, 6, 0, 3);
  FifoRecord records[2];
  TEST_ASSERT_TRUE(driver.startFifoRead({records, 2, 6}, filterTiming(bus), token).inProgress());
  filterRun(driver, bus);
  const OperationResult read = filterTake(driver, token);
  TEST_ASSERT_TRUE(read.status.ok());
  TEST_ASSERT_EQUAL_UINT16(1, read.fifoRead.records);
  TEST_ASSERT_EQUAL_UINT16(6, read.fifoRead.wordsConsumed);
  TEST_ASSERT_EQUAL_INT16(-101, records[0].raw.x);
  TEST_ASSERT_EQUAL_UINT8(SAMPLE_ACCELERATION, records[0].qualifiedMask);
  TEST_ASSERT_TRUE(records[0].filters.gyro.highPassEnabled);
  TEST_ASSERT_TRUE(records[0].filters.accel.highPassSlopeEnabled);
  TEST_ASSERT_EQUAL_UINT32(configured.configuration.generation, records[0].configGeneration);
  const FifoRecord retained = records[0];
  TEST_ASSERT_TRUE(driver.startFifoPurge({1, true}, filterTiming(bus), token).inProgress());
  filterRun(driver, bus);
  TEST_ASSERT_TRUE(filterTake(driver, token).status.ok());
  filterConfigure(driver, bus, DeviceProfile{});
  TEST_ASSERT_TRUE(retained.filters.gyro.highPassEnabled);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelHpcf::DIV_9),
                        static_cast<uint8_t>(retained.filters.accel.hpcf));
}

void test_filter_calibration_rejects_non_dc_paths_without_io() {
  for (uint8_t path = 0; path < 4; ++path) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    DeviceProfile profile = path == 3 ? hpfProfile() : referenceProfile();
    if (path < 2) {
      profile.accelFilter.referenceEnabled = false;
      profile.accelFilter.hpcf = path == 0 ? AccelHpcf::DIV_50 : AccelHpcf::DIV_400;
    }
    filterConfigure(driver, bus, profile);
    CalibrationRequest request;
    request.kind = path == 3 ? CalibrationKind::GYROSCOPE_BIAS :
                              CalibrationKind::ACCELEROMETER_BIAS;
    request.expectedAccelerationG = {0, 0, 1};
    const uint32_t before = bus.transferCalls;
    OperationToken token;
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::UNSUPPORTED_PROFILE),
        static_cast<uint8_t>(driver.startCalibration(request, filterTiming(bus), token).code));
    TEST_ASSERT_FALSE(token.valid());
    TEST_ASSERT_EQUAL_UINT32(before, bus.transferCalls);
    // A filtered other sensor does not prohibit DC-preserving accel calibration.
    if (path == 3) {
      request.kind = CalibrationKind::ACCELEROMETER_BIAS;
      request.samples = 1;
      TEST_ASSERT_TRUE(driver.startCalibration(request, filterTiming(bus), token).inProgress());
      filterRun(driver, bus);
      const OperationResult result = filterTake(driver, token);
      TEST_ASSERT_TRUE(result.status.ok());
      TEST_ASSERT_TRUE(result.calibration.filters.gyro.highPassEnabled);
      TEST_ASSERT_EQUAL_UINT32(driver.configGeneration(), result.calibration.configGeneration);
    }
  }
}

void test_filter_self_test_normalizes_and_restores_actual_hpf_profile() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  DeviceProfile original = hpfProfile();
  original.accelFilter.lpf2Enabled = true;
  original.accelFilter.hpcf = AccelHpcf::DIV_100;
  original.accelFilter.lowPassOn6d = true;
  original.interrupts.enabled = true;
  original.interrupts.orientation.mode = OrientationMode::FOUR_D;
  original.interrupts.int1.orientation = true;
  filterConfigure(driver, bus, original);
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startConfigure(DeviceProfile{}, filterTiming(bus), token).inProgress());
  TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
  filterTake(driver, token);
  bus.clearTrace();
  TEST_ASSERT_TRUE(driver.startSelfTest({5}, filterTiming(bus), token).inProgress());
  filterRun(driver, bus);
  const OperationResult result = filterTake(driver, token);
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_TRUE(result.selfTest.primaryStatus.ok());
  TEST_ASSERT_TRUE(result.selfTest.restorationStatus.ok());
  TEST_ASSERT_LESS_THAN(bus.traceCount, filterFindWrite(bus, 0x16, 0));
  TEST_ASSERT_LESS_THAN(bus.traceCount, filterFindWrite(bus, 0x17, 0));
  TEST_ASSERT_EQUAL_HEX8(0x02, bus.regs[0x15]);
  TEST_ASSERT_EQUAL_HEX8(0x60, bus.regs[0x16]);
  TEST_ASSERT_EQUAL_HEX8(0xA1, bus.regs[0x17]);
  TEST_ASSERT_EQUAL_HEX8(0x04, bus.regs[0x5E]);
  DeviceProfile restored;
  TEST_ASSERT_TRUE(driver.getVerifiedProfile(restored, bus.nowMs).ok());
  TEST_ASSERT_TRUE(restored.gyroFilter.highPassEnabled);
  TEST_ASSERT_TRUE(restored.accelFilter.lowPassOn6d);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::FILTER_UNQUALIFIED),
                        static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
}

void test_filter_reference_discards_after_readback_before_fifo_and_routes() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  bus.notReadyStatusReads = 2;
  const DeviceProfile profile = referenceProfile(true, true);
  const OperationResult result = filterConfigure(driver, bus, profile);
  TEST_ASSERT_EQUAL_UINT32(110, result.transactionLimit);
  TEST_ASSERT_EQUAL_UINT32(110, result.transactions);
  TEST_ASSERT_TRUE(result.configuration.referenceDiscardMayHaveOccurred);
  TEST_ASSERT_TRUE(result.configuration.referenceDiscardConfirmed);
  const size_t finalImage = filterFindRead(bus, 0x75);
  const size_t ready = filterFindRead(bus, 0x1E);
  const size_t discard = filterFindRead(bus, 0x28);
  const size_t fifo = filterFindWrite(bus, 0x0A, 0x26);
  const size_t route = filterFindWrite(bus, 0x5E, 0x04);
  TEST_ASSERT_LESS_THAN(bus.traceCount, finalImage);
  TEST_ASSERT_LESS_THAN(discard, ready);
  TEST_ASSERT_LESS_THAN(fifo, discard);
  TEST_ASSERT_LESS_THAN(route, discard);
  // 14 * ceil(1e6/104) = 134624 us; source table envelope plus clock tick.
  TEST_ASSERT_GREATER_OR_EQUAL_UINT64(bus.trace[finalImage].atMs + 136U, bus.trace[ready].atMs);
  TEST_ASSERT_GREATER_OR_EQUAL_UINT64(bus.trace[discard].atMs + 11U, bus.trace[fifo].atMs);
  size_t statusCount = 0;
  size_t discardCount = 0;
  for (size_t i = 0; i < bus.traceCount; ++i) {
    if (bus.trace[i].kind != TransferKind::WRITE_READ) continue;
    if (bus.trace[i].startReg == 0x1E) ++statusCount;
    if (bus.trace[i].startReg == 0x28) ++discardCount;
  }
  TEST_ASSERT_EQUAL_UINT32(3, statusCount);
  TEST_ASSERT_EQUAL_UINT32(1, discardCount);
  TEST_ASSERT_EQUAL_HEX8(0x34, bus.regs[0x17]);
}

void test_filter_reference_unready_is_bounded_and_unpublished() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  bus.notReadyStatusReads = 99;
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startConfigure(referenceProfile(true), filterTiming(bus), token).inProgress());
  filterRun(driver, bus);
  const OperationResult result = filterTake(driver, token);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::DATA_NOT_READY),
                        static_cast<uint8_t>(result.status.code));
  TEST_ASSERT_FALSE(result.configuration.referenceDiscardMayHaveOccurred);
  TEST_ASSERT_FALSE(result.configuration.referenceDiscardConfirmed);
  TEST_ASSERT_EQUAL_UINT32(96, bus.notReadyStatusReads);
  TEST_ASSERT_EQUAL_UINT8(0, bus.regs[0x0A] & 7);
  TEST_ASSERT_EQUAL_UINT8(0, bus.regs[0x5E]);
  TEST_ASSERT_EQUAL_UINT32(0, driver.configGeneration());
}

void test_filter_reference_fault_cancel_and_deadline_each_new_callback() {
  FakeBus baselineBus;
  Driver baseline;
  TEST_ASSERT_TRUE(baseline.bind(makeDriverConfig(baselineBus)).ok());
  baselineBus.notReadyStatusReads = 2;
  filterConfigure(baseline, baselineBus, referenceProfile(true));
  const uint32_t first = static_cast<uint32_t>(filterFindRead(baselineBus, 0x1E)) + 1U;
  const uint32_t last = static_cast<uint32_t>(filterFindRead(baselineBus, 0x28)) + 1U;
  TEST_ASSERT_EQUAL_UINT32(4, last - first + 1U);
  for (uint32_t call = first; call <= last; ++call) {
    for (uint8_t failure = 0; failure < 4; ++failure) {
      FakeBus bus;
      Driver driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      bus.notReadyStatusReads = 2;
      OperationToken token;
      const OperationTiming timing = filterTiming(bus);
      TEST_ASSERT_TRUE(driver.startConfigure(referenceProfile(true), timing, token).inProgress());
      if (failure < 2) {
        bus.addFault(call, Status::Error(Err::I2C_TIMEOUT, "reference phase", -94), failure == 1);
        filterRun(driver, bus);
      } else {
        for (uint32_t poll = 0; poll < 10000U && bus.transferCalls < call - 1U; ++poll)
          driver.poll(bus.nowMs++, 1);
        TEST_ASSERT_EQUAL_UINT32(call - 1U, bus.transferCalls);
        const uint32_t before = bus.transferCalls;
        if (failure == 2) TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
        else {
          bus.nowMs = timing.deadlineMs;
          driver.poll(bus.nowMs, 255);
        }
        TEST_ASSERT_EQUAL_UINT32(before, bus.transferCalls);
      }
      const OperationResult result = filterTake(driver, token);
      TEST_ASSERT_FALSE(result.status.ok());
      TEST_ASSERT_TRUE(result.hardwareStateMayHaveChanged);
      TEST_ASSERT_FALSE(result.configuration.referenceDiscardConfirmed);
      TEST_ASSERT_EQUAL(call == last && failure < 2,
                        result.configuration.referenceDiscardMayHaveOccurred);
      TEST_ASSERT_EQUAL_UINT8(0, bus.regs[0x0A] & 7U);
      TEST_ASSERT_EQUAL_UINT8(0, bus.regs[0x5E]);
      TEST_ASSERT_EQUAL_UINT32(0, driver.configGeneration());
      TEST_ASSERT_FALSE(driver.operationActive());
    }
  }
}

void test_filter_reference_cancel_and_deadline_after_discard_preserve_effect() {
  for (bool cancel : {false, true}) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    OperationToken token;
    const OperationTiming timing = filterTiming(bus);
    TEST_ASSERT_TRUE(driver.startConfigure(referenceProfile(true), timing, token).inProgress());
    for (uint32_t poll = 0; poll < 10000U && filterFindRead(bus, 0x28) == bus.traceCount; ++poll)
      driver.poll(bus.nowMs++, 1);
    TEST_ASSERT_LESS_THAN(bus.traceCount, filterFindRead(bus, 0x28));
    const uint32_t before = bus.transferCalls;
    if (cancel) TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
    else {
      bus.nowMs = timing.deadlineMs;
      driver.poll(bus.nowMs, 0);
    }
    TEST_ASSERT_EQUAL_UINT32(before, bus.transferCalls);
    const OperationResult result = filterTake(driver, token);
    TEST_ASSERT_FALSE(result.status.ok());
    TEST_ASSERT_TRUE(result.configuration.referenceDiscardConfirmed);
    TEST_ASSERT_TRUE(result.configuration.referenceDiscardMayHaveOccurred);
    TEST_ASSERT_EQUAL_UINT8(0, bus.regs[0x0A] & 7U);
    TEST_ASSERT_EQUAL_UINT32(0, driver.configGeneration());
  }
}

void test_filter_trusted_reconcile_preserves_gate_generation_and_reference() {
  for (bool reference : {false, true}) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    const OperationResult configured = filterConfigure(driver, bus,
        reference ? referenceProfile() : hpfProfile());
    bus.clearTrace();
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startReconcile(filterTiming(bus), token).inProgress());
    filterRun(driver, bus);
    const OperationResult reconciled = filterTake(driver, token);
    TEST_ASSERT_TRUE(reconciled.status.ok());
    TEST_ASSERT_EQUAL_UINT32(configured.configuration.generation, reconciled.configuration.generation);
    TEST_ASSERT_EQUAL_UINT64(configured.configuration.validAfterUptimeMs,
                            reconciled.configuration.validAfterUptimeMs);
    TEST_ASSERT_EQUAL_UINT32(0, bus.writeCalls);
    TEST_ASSERT_EQUAL_UINT32(bus.traceCount, filterFindRead(bus, 0x28));
    TEST_ASSERT_FALSE(reconciled.configuration.referenceDiscardMayHaveOccurred);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(configured.configuration.state),
                          static_cast<uint8_t>(reconciled.configuration.state));
    // Register-only reconciliation cannot re-establish an invalidated reference.
    if (reference) {
      TEST_ASSERT_TRUE(driver.diagnosticWriteRegister(0x17, 0x34, bus.nowMs).ok());
      const uint32_t before = bus.transferCalls;
      TEST_ASSERT_FALSE(driver.startReconcile(filterTiming(bus), token).inProgress());
      TEST_ASSERT_EQUAL_UINT32(before, bus.transferCalls);
    }
  }
}

void test_filter_reference_low_odr_and_uint64_gate_never_shortens() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  DeviceProfile profile = referenceProfile();
  profile.accelOdr = Odr::HZ_1_6;
  profile.accelPowerMode = AccelPowerMode::LOW_POWER_NORMAL;
  profile.gyroOdr = Odr::POWER_DOWN;
  const uint64_t began = bus.nowMs;
  const OperationResult result = filterConfigure(driver, bus, profile);
  TEST_ASSERT_GREATER_OR_EQUAL_UINT64(began + 8751U + 626U,
                                     result.configuration.validAfterUptimeMs);
  TEST_ASSERT_TRUE(result.configuration.referenceDiscardConfirmed);

  FakeBus near;
  Driver nearDriver;
  TEST_ASSERT_TRUE(nearDriver.bind(makeDriverConfig(near)).ok());
  near.nowMs = std::numeric_limits<uint64_t>::max() - 1000U;
  OperationToken token;
  TEST_ASSERT_TRUE(nearDriver.startConfigure(profile,
      {near.nowMs, std::numeric_limits<uint64_t>::max()}, token).inProgress());
  for (uint32_t i = 0; i < 200U && nearDriver.validAfterUptimeMs() !=
       std::numeric_limits<uint64_t>::max(); ++i) nearDriver.poll(near.nowMs++, 255);
  TEST_ASSERT_EQUAL_UINT64(std::numeric_limits<uint64_t>::max(), nearDriver.validAfterUptimeMs());
  const uint32_t before = near.transferCalls;
  nearDriver.poll(std::numeric_limits<uint64_t>::max() - 1U, 255);
  TEST_ASSERT_TRUE(nearDriver.operationActive());
  TEST_ASSERT_EQUAL_UINT32(before, near.transferCalls);
  nearDriver.poll(std::numeric_limits<uint64_t>::max(), 0);
  TEST_ASSERT_FALSE(filterTake(nearDriver, token).status.ok());
  TEST_ASSERT_EQUAL_UINT32(before, near.transferCalls);
}

void test_filter_orientation_result_preserves_lpf2_source_after_reconfigure() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  DeviceProfile profile;
  profile.interrupts.enabled = true;
  profile.interrupts.orientation.mode = OrientationMode::SIX_D;
  profile.interrupts.int1.orientation = true;
  profile.accelFilter.lowPassOn6d = true;
  profile.accelFilter.hpcf = AccelHpcf::DIV_400;
  profile.accelFilter.compositeInput = AccelCompositeInput::LOW_NOISE;
  filterConfigure(driver, bus, profile);
  bus.setEventSources(0x41, 0);
  EventReadRequest request;
  request.orientation = true;
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startEventRead(request, filterTiming(bus), token).inProgress());
  filterRun(driver, bus);
  const OperationResult event = filterTake(driver, token);
  TEST_ASSERT_TRUE(event.status.ok());
  filterConfigure(driver, bus, DeviceProfile{});
  TEST_ASSERT_TRUE(event.events.filters.accel.lowPassOn6d);
  TEST_ASSERT_FALSE(event.events.filters.accel.lpf2Enabled);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelHpcf::DIV_400),
                        static_cast<uint8_t>(event.events.filters.accel.hpcf));
  TEST_ASSERT_EQUAL_UINT8(SAMPLE_ALL, event.events.qualifiedMask);
}

void test_filter_reference_lifecycle_replay_reserves_exact_callback_ceiling() {
  for (uint8_t operation = 0; operation < 3; ++operation) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    const DeviceProfile profile = referenceProfile(true, true);
    filterConfigure(driver, bus, profile);
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startFifoPurge({1, true}, filterTiming(bus), token).inProgress());
    filterRun(driver, bus);
    TEST_ASSERT_TRUE(filterTake(driver, token).status.ok());
    bus.clearTrace();
    bus.notReadyStatusReads = 2;
    bus.resetRegistersOnSoftwareReset = true;
    if (operation == 1) bus.bootBitReadsRemaining = 15;
    else bus.resetBitReadsRemaining = 15;
    const Status started = operation == 0 ? driver.startReset(filterTiming(bus), token) :
                           operation == 1 ? driver.startBoot(filterTiming(bus), token) :
                                            driver.startRecover(filterTiming(bus), token);
    TEST_ASSERT_TRUE(started.inProgress());
    filterRun(driver, bus);
    const OperationResult result = filterTake(driver, token);
    TEST_ASSERT_TRUE_MESSAGE(result.status.ok(), result.status.msg);
    TEST_ASSERT_EQUAL_UINT32(operation == 2 ? 130 : 131, result.transactionLimit);
    // Boot retains timestamp resolution, omitting its otherwise required reset.
    TEST_ASSERT_EQUAL_UINT32(operation == 0 ? 131 : 130, result.transactions);
    TEST_ASSERT_TRUE(result.configuration.referenceDiscardConfirmed);
    TEST_ASSERT_EQUAL_HEX8(0x34, bus.regs[0x17]);
    TEST_ASSERT_EQUAL_HEX8(0x26, bus.regs[0x0A]);
    TEST_ASSERT_EQUAL_HEX8(0x04, bus.regs[0x5E]);
  }
}

void test_filter_self_test_reference_restore_and_primary_failure_keep_distinct_status() {
  for (bool failShutdown : {false, true}) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    filterConfigure(driver, bus, referenceProfile(false, true));
    bus.clearTrace();
    bus.notReadyPeriod = 3;
    if (failShutdown)
      bus.addFault(16U * 6U + 27U, Status::Error(Err::I2C_TIMEOUT, "primary shutdown", -801));
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startSelfTest({5}, filterTiming(bus), token).inProgress());
    filterRun(driver, bus);
    const OperationResult result = filterTake(driver, token);
    TEST_ASSERT_EQUAL(!failShutdown, result.status.ok());
    TEST_ASSERT_EQUAL(!failShutdown, result.selfTest.primaryStatus.ok());
    TEST_ASSERT_TRUE(result.selfTest.restorationStatus.ok());
    TEST_ASSERT_EQUAL_UINT32(16U * 6U + 126U, result.transactionLimit);
    // No timer resolution change during self-test: one optional reset unused;
    // successful path additionally leaves its failure-cleanup reserve unused.
    TEST_ASSERT_EQUAL_UINT32(16U * 6U + (failShutdown ? 125U : 124U), result.transactions);
    TEST_ASSERT_TRUE(result.configuration.referenceDiscardConfirmed);
    TEST_ASSERT_EQUAL_HEX8(0x34, bus.regs[0x17]);
    TEST_ASSERT_EQUAL_HEX8(0x04, bus.regs[0x5E]);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::KNOWN),
                          static_cast<uint8_t>(result.configuration.state));
  }
}

void test_filter_self_test_reference_restoration_failure_each_discard_callback() {
  for (bool failPrimary : {false, true}) {
    FakeBus baselineBus;
    Driver baseline;
    TEST_ASSERT_TRUE(baseline.bind(makeDriverConfig(baselineBus)).ok());
    filterConfigure(baseline, baselineBus, referenceProfile());
    baselineBus.clearTrace();
    baselineBus.notReadyPeriod = 3;
    if (failPrimary)
      baselineBus.addFault(16U * 6U + 27U, Status::Error(Err::I2C_TIMEOUT, "primary shutdown", -811));
    OperationToken token;
    TEST_ASSERT_TRUE(baseline.startSelfTest({5}, filterTiming(baselineBus), token).inProgress());
    filterRun(baseline, baselineBus);
    filterTake(baseline, token);
    const size_t image = filterFindRead(baselineBus, 0x75);
    const uint32_t first = static_cast<uint32_t>(filterFindRead(baselineBus, 0x1E, image)) + 1U;
    const uint32_t last = static_cast<uint32_t>(filterFindRead(baselineBus, 0x28, image)) + 1U;
    TEST_ASSERT_EQUAL_UINT32(4, last - first + 1U);
    for (uint32_t call = first; call <= last; ++call) {
      for (bool ambiguous : {false, true}) {
        FakeBus bus;
        Driver driver;
        TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
        filterConfigure(driver, bus, referenceProfile());
        bus.clearTrace();
        bus.notReadyPeriod = 3;
        if (failPrimary)
          bus.addFault(16U * 6U + 27U, Status::Error(Err::I2C_TIMEOUT, "primary shutdown", -811));
        bus.addFault(call, Status::Error(Err::I2C_NACK_DATA, "reference restore", -812), ambiguous);
        TEST_ASSERT_TRUE(driver.startSelfTest({5}, filterTiming(bus), token).inProgress());
        filterRun(driver, bus);
        const OperationResult result = filterTake(driver, token);
        TEST_ASSERT_FALSE(result.status.ok());
        TEST_ASSERT_EQUAL(!failPrimary, result.selfTest.primaryStatus.ok());
        if (failPrimary) TEST_ASSERT_EQUAL_INT32(-811, result.selfTest.primaryStatus.detail);
        TEST_ASSERT_EQUAL_INT32(-812, result.selfTest.restorationStatus.detail);
        TEST_ASSERT_FALSE(result.configuration.referenceDiscardConfirmed);
        TEST_ASSERT_EQUAL(call == last, result.configuration.referenceDiscardMayHaveOccurred);
        TEST_ASSERT_EQUAL_UINT32(call, result.transactions);
        TEST_ASSERT_EQUAL_UINT8(0, bus.regs[0x5E]);
        TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNKNOWN),
                              static_cast<uint8_t>(result.configuration.state));
      }
    }
  }
}

void test_filter_reference_callback_time_and_zero_budget_preserve_both_gates() {
  for (uint8_t gateBudget : {0U, 255U}) {
    FakeBus bus;
    Driver driver;
    DriverConfig config = makeDriverConfig(bus);
    config.i2cWrite = [](uint8_t address, const uint8_t* data, size_t length,
                          uint32_t timeoutMs, void* user) {
      const Status status = fakeWrite(address, data, length, timeoutMs, user);
      static_cast<FakeBus*>(user)->nowMs += 7U;
      return status;
    };
    config.i2cWriteRead = [](uint8_t address, const uint8_t* txData, size_t txLength,
                              uint8_t* rxData, size_t rxLength, uint32_t timeoutMs,
                              void* user) {
      const Status status = fakeWriteRead(address, txData, txLength, rxData, rxLength,
                                           timeoutMs, user);
      static_cast<FakeBus*>(user)->nowMs += 7U;
      return status;
    };
    TEST_ASSERT_TRUE(driver.bind(config).ok());
    const DeviceProfile profile = referenceProfile(true);
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startConfigure(profile, filterTiming(bus), token).inProgress());
    for (uint32_t i = 0; i < 200U && filterFindRead(bus, 0x75) == bus.traceCount; ++i)
      driver.poll(bus.nowMs, 255);
    TEST_ASSERT_LESS_THAN(bus.traceCount, filterFindRead(bus, 0x75));
    bus.nowMs += 23U;
    const uint64_t armAt = bus.nowMs;
    const uint32_t before = bus.transferCalls;
    TEST_ASSERT_EQUAL_UINT8(0, driver.poll(bus.nowMs, gateBudget).transactionsUsed);
    TEST_ASSERT_EQUAL_UINT64(armAt + 136U, driver.validAfterUptimeMs());
    bus.nowMs = armAt + 135U;
    driver.poll(bus.nowMs, 255);
    TEST_ASSERT_EQUAL_UINT32(before, bus.transferCalls);
    bus.nowMs = armAt + 136U;
    for (uint8_t i = 0; i < 8U && filterFindRead(bus, 0x28) == bus.traceCount; ++i)
      driver.poll(bus.nowMs, 255);
    const size_t discard = filterFindRead(bus, 0x28);
    TEST_ASSERT_LESS_THAN(bus.traceCount, discard);
    bus.nowMs += 23U;
    const uint64_t discardArmAt = bus.nowMs;
    const uint32_t afterDiscard = bus.transferCalls;
    driver.poll(bus.nowMs, gateBudget);
    TEST_ASSERT_EQUAL_UINT32(afterDiscard, bus.transferCalls);
    TEST_ASSERT_EQUAL_UINT64(discardArmAt + 11U, driver.validAfterUptimeMs());
    bus.nowMs = discardArmAt + 10U;
    driver.poll(bus.nowMs, 255);
    TEST_ASSERT_EQUAL_UINT32(afterDiscard, bus.transferCalls);
    bus.nowMs = discardArmAt + 11U;
    filterRun(driver, bus);
    const OperationResult result = filterTake(driver, token);
    TEST_ASSERT_TRUE(result.status.ok());
    const size_t activate = filterFindWrite(bus, 0x0A, 0x26);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT64(discardArmAt + 11U, bus.trace[activate].atMs);
  }
}

void test_filter_reference_mismatched_image_never_discards_or_activates() {
  for (uint8_t mismatch : {0x10, 0x15, 0x16, 0x17}) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    bus.corruptReadRegister = mismatch;
    bus.corruptReadValue = 0xFF;
    bus.corruptReadRemaining = 1;
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startConfigure(referenceProfile(true), filterTiming(bus), token).inProgress());
    filterRun(driver, bus);
    const OperationResult result = filterTake(driver, token);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CONFIGURATION_MISMATCH),
                          static_cast<uint8_t>(result.status.code));
    TEST_ASSERT_EQUAL_UINT8(mismatch, result.configuration.mismatchRegister);
    TEST_ASSERT_FALSE(result.configuration.referenceDiscardMayHaveOccurred);
    TEST_ASSERT_FALSE(result.configuration.referenceDiscardConfirmed);
    TEST_ASSERT_EQUAL_UINT8(0, bus.regs[0x0A] & 7U);
    TEST_ASSERT_EQUAL_UINT32(0, driver.configGeneration());
  }
}

void test_filter_self_test_clears_direct_bandwidth_and_analog_override_recipe() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  DeviceProfile profile;
  profile.accelOdr = Odr::HZ_1660;
  profile.accelFilter.lpf1Bandwidth = AccelLpf1Bandwidth::ODR_DIV_4;
  profile.accelFilter.analogBandwidth = AccelAnalogBandwidth::HZ_400;
  filterConfigure(driver, bus, profile);
  TEST_ASSERT_EQUAL_HEX8(0x83, bus.regs[0x10]);
  bus.clearTrace();
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startSelfTest({5}, filterTiming(bus), token).inProgress());
  filterRun(driver, bus);
  const OperationResult result = filterTake(driver, token);
  TEST_ASSERT_TRUE(result.status.ok());
  // AN5130 Fig37: exactly 52 Hz /4g with both filter-selector bits zero.
  TEST_ASSERT_LESS_THAN(bus.traceCount, filterFindWrite(bus, 0x10, 0x38));
  TEST_ASSERT_EQUAL_UINT32(bus.traceCount, filterFindWrite(bus, 0x10, 0x3B));
  TEST_ASSERT_EQUAL_HEX8(0x83, bus.regs[0x10]);
  DeviceProfile restored;
  TEST_ASSERT_TRUE(driver.getVerifiedProfile(restored, bus.nowMs).ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelLpf1Bandwidth::ODR_DIV_4),
                        static_cast<uint8_t>(restored.accelFilter.lpf1Bandwidth));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelAnalogBandwidth::HZ_400),
                        static_cast<uint8_t>(restored.accelFilter.analogBandwidth));
}

void test_filter_reference_fifo_discards_transition_and_requires_stop_for_new_path() {
  for (bool reference : {false, true}) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    DeviceProfile original = referenceProfile(true);
    if (!reference) {
      original.accelFilter.referenceEnabled = false;
      original.accelFilter.hpcf = AccelHpcf::DIV_9;
    }
    const OperationResult configured = filterConfigure(driver, bus, original);
    TEST_ASSERT_EQUAL(reference, configured.configuration.referenceDiscardConfirmed);
    TEST_ASSERT_EQUAL_UINT8(0, configured.sample.validMask);
    // The reference output-register discard precedes FIFO activation, and is
    // distinct from consuming the FIFO's first complete motion vector.
    const size_t referenceRead = filterFindRead(bus, 0x28);
    if (reference) {
      TEST_ASSERT_LESS_THAN(filterFindWrite(bus, 0x0A, 0x26), referenceRead);
      TEST_ASSERT_EQUAL_UINT32(bus.traceCount, filterFindRead(bus, 0x28, referenceRead + 1U));
    } else {
      TEST_ASSERT_EQUAL_UINT32(bus.traceCount, referenceRead);
    }
    const int16_t words[] = {32767, 32766, 32765, -111, 222, -333, 444, 555, 666};
    bus.setFifoWords(words, 9, 0, 3);
    FifoRecord record;
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startFifoRead({&record, 1, 9}, filterTiming(bus), token).inProgress());
    filterRun(driver, bus);
    const OperationResult firstRead = filterTake(driver, token);
    TEST_ASSERT_TRUE(firstRead.status.ok());
    TEST_ASSERT_EQUAL_UINT16(6, firstRead.fifoRead.wordsConsumed);
    TEST_ASSERT_EQUAL_UINT16(3, firstRead.fifoRead.wordsDiscarded);
    TEST_ASSERT_EQUAL_UINT16(1, firstRead.fifoRead.records);
    TEST_ASSERT_EQUAL_INT16(-111, record.raw.x);
    TEST_ASSERT_EQUAL_INT16(-333, record.raw.z);
    TEST_ASSERT_EQUAL_UINT8(SAMPLE_ACCELERATION, record.qualifiedMask);
    TEST_ASSERT_EQUAL(reference, record.filters.accel.referenceEnabled);
    TEST_ASSERT_TRUE(record.filters.accel.highPassSlopeEnabled);
    TEST_ASSERT_EQUAL_UINT32(configured.configuration.generation, record.configGeneration);
    const FifoRecord retained = record;

    DeviceProfile changed = original;
    changed.accelFilter = {};
    changed.accelFilter.lpf2Enabled = true;
    changed.accelFilter.hpcf = AccelHpcf::DIV_400;
    changed.accelFilter.compositeInput = AccelCompositeInput::LOW_NOISE;
    const uint32_t writes = bus.writeCalls;
    TEST_ASSERT_TRUE(driver.startConfigure(changed, filterTiming(bus), token).inProgress());
    filterRun(driver, bus);
    const OperationResult rejected = filterTake(driver, token);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::FIFO_MAINTENANCE_REQUIRED),
                          static_cast<uint8_t>(rejected.status.code));
    TEST_ASSERT_EQUAL_UINT32(writes, bus.writeCalls);
    TEST_ASSERT_EQUAL_UINT16(3, bus.fifoUnreadWords);
    TEST_ASSERT_EQUAL_UINT32(configured.configuration.generation, driver.configGeneration());
    TEST_ASSERT_TRUE(driver.startFifoPurge({1, true}, filterTiming(bus), token).inProgress());
    filterRun(driver, bus);
    const OperationResult stopped = filterTake(driver, token);
    TEST_ASSERT_TRUE(stopped.status.ok());
    TEST_ASSERT_TRUE(stopped.fifoPurge.collectionStopped);
    TEST_ASSERT_TRUE(stopped.fifoPurge.residualLossPossible);
    TEST_ASSERT_EQUAL_UINT16(0, bus.fifoUnreadWords);
    const OperationResult applied = filterConfigure(driver, bus, changed);
    TEST_ASSERT_GREATER_THAN_UINT32(retained.configGeneration, applied.configuration.generation);
    TEST_ASSERT_EQUAL_HEX8(0xE8, bus.regs[0x17]);
    const int16_t replacement[] = {321, 654, 987, -444, 555, -666};
    bus.setFifoWords(replacement, 6, 0, 3);
    TEST_ASSERT_TRUE(driver.startFifoRead({&record, 1, 6}, filterTiming(bus), token).inProgress());
    filterRun(driver, bus);
    const OperationResult secondRead = filterTake(driver, token);
    TEST_ASSERT_TRUE(secondRead.status.ok());
    TEST_ASSERT_EQUAL_UINT16(3, secondRead.fifoRead.wordsDiscarded);
    TEST_ASSERT_EQUAL_INT16(-444, record.raw.x);
    TEST_ASSERT_TRUE(record.filters.accel.lpf2Enabled);
    TEST_ASSERT_FALSE(record.filters.accel.referenceEnabled);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelHpcf::DIV_400),
                          static_cast<uint8_t>(record.filters.accel.hpcf));
    TEST_ASSERT_EQUAL(reference, retained.filters.accel.referenceEnabled);
    TEST_ASSERT_TRUE(retained.filters.accel.highPassSlopeEnabled);
    TEST_ASSERT_FALSE(retained.filters.accel.lpf2Enabled);
    TEST_ASSERT_EQUAL_INT16(-111, retained.raw.x);
    TEST_ASSERT_EQUAL_UINT8(SAMPLE_ACCELERATION, retained.qualifiedMask);
  }
}
} // namespace

void runFilterOperationTests() {
  RUN_TEST(test_filter_hpf_requires_explicit_direct_opt_in_for_every_gyro_request);
  RUN_TEST(test_filter_old_sample_conversion_retains_qualification_and_path);
  RUN_TEST(test_filter_hpf_fifo_exclusion_is_per_channel_and_records_keep_provenance);
  RUN_TEST(test_filter_calibration_rejects_non_dc_paths_without_io);
  RUN_TEST(test_filter_self_test_normalizes_and_restores_actual_hpf_profile);
  RUN_TEST(test_filter_reference_discards_after_readback_before_fifo_and_routes);
  RUN_TEST(test_filter_reference_unready_is_bounded_and_unpublished);
  RUN_TEST(test_filter_reference_fault_cancel_and_deadline_each_new_callback);
  RUN_TEST(test_filter_reference_cancel_and_deadline_after_discard_preserve_effect);
  RUN_TEST(test_filter_trusted_reconcile_preserves_gate_generation_and_reference);
  RUN_TEST(test_filter_reference_low_odr_and_uint64_gate_never_shortens);
  RUN_TEST(test_filter_orientation_result_preserves_lpf2_source_after_reconfigure);
  RUN_TEST(test_filter_reference_lifecycle_replay_reserves_exact_callback_ceiling);
  RUN_TEST(test_filter_self_test_reference_restore_and_primary_failure_keep_distinct_status);
  RUN_TEST(test_filter_self_test_reference_restoration_failure_each_discard_callback);
  RUN_TEST(test_filter_reference_callback_time_and_zero_budget_preserve_both_gates);
  RUN_TEST(test_filter_reference_mismatched_image_never_discards_or_activates);
  RUN_TEST(test_filter_self_test_clears_direct_bandwidth_and_analog_override_recipe);
  RUN_TEST(test_filter_reference_fifo_discards_transition_and_requires_stop_for_new_path);
}
