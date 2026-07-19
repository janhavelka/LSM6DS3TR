/// @file test_basic.cpp
/// @brief Native production-contract and fault-injection tests.

#include <unity.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include "LSM6DS3TR/LSM6DS3TR.h"
#include "support/FakeBus.h"

using namespace LSM6DS3TR;
using namespace LSM6DS3TRTest;

namespace {

DeviceProfile makeProfile() {
  DeviceProfile profile;
  profile.accelOdr = Odr::HZ_104;
  profile.gyroOdr = Odr::HZ_104;
  profile.blockDataUpdate = true;
  return profile;
}

OperationTiming timing(const FakeBus& bus, uint64_t durationMs = 10000u) {
  return OperationTiming{bus.nowMs, bus.nowMs + durationMs};
}

PollResult runToTerminal(LSM6DS3TR::LSM6DS3TR& driver, FakeBus& bus,
                         uint8_t budget = 1, uint32_t maxPolls = 20000) {
  PollResult result;
  for (uint32_t pollCount = 0; pollCount < maxPolls; ++pollCount) {
    const uint32_t before = bus.transferCalls;
    result = driver.poll(bus.nowMs, budget);
    TEST_ASSERT_EQUAL_UINT32(bus.transferCalls - before, result.transactionsUsed);
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(budget, result.transactionsUsed);
    if (result.state != OperationState::ACTIVE) {
      return result;
    }
    bus.nowMs++;
  }
  TEST_FAIL_MESSAGE("operation did not reach a bounded terminal state");
  return result;
}

OperationResult take(LSM6DS3TR::LSM6DS3TR& driver, OperationToken token) {
  OperationResult result;
  const Status status = driver.takeResult(token, result);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::OK),
                          static_cast<uint8_t>(status.code));
  TEST_ASSERT_EQUAL_UINT64(token.value, result.token.value);
  return result;
}

OperationResult configure(LSM6DS3TR::LSM6DS3TR& driver, FakeBus& bus,
                          const DeviceProfile& profile = makeProfile()) {
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startConfigure(profile, timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(terminal.state));
  OperationResult result = take(driver, token);
  TEST_ASSERT_TRUE(result.status.ok());
  if (bus.nowMs < result.configuration.validAfterUptimeMs) {
    bus.nowMs = result.configuration.validAfterUptimeMs;
  }
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::KNOWN),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
  return result;
}

size_t findWrite(const FakeBus& bus, uint8_t reg, uint8_t requiredMask = 0,
                 uint8_t requiredValue = 0, size_t start = 0) {
  for (size_t i = start; i < bus.traceCount; ++i) {
    const Transfer& transfer = bus.trace[i];
    if (transfer.kind == TransferKind::WRITE && transfer.startReg == reg &&
        (transfer.firstValue & requiredMask) == requiredValue) {
      return i;
    }
  }
  return bus.traceCount;
}

void setFifoPurgePrerequisites(FakeBus& bus) {
  bus.regs[cmd::REG_CTRL3_C] =
      static_cast<uint8_t>(cmd::MASK_IF_INC | cmd::MASK_BDU);
}

void assertTerminalFailure(const PollResult& terminal, const OperationResult& result,
                           Err expectedStatus, OperationState expectedState) {
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expectedState),
                          static_cast<uint8_t>(terminal.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expectedStatus),
                          static_cast<uint8_t>(result.status.code));
}

enum class MaintenanceCase : uint8_t {
  RESET,
  BOOT,
  RECOVER,
};

Status startMaintenance(LSM6DS3TR::LSM6DS3TR& driver, MaintenanceCase operation,
                        const OperationTiming& operationTiming,
                        OperationToken& token) {
  switch (operation) {
    case MaintenanceCase::RESET:
      return driver.startReset(operationTiming, token);
    case MaintenanceCase::BOOT:
      return driver.startBoot(operationTiming, token);
    case MaintenanceCase::RECOVER:
      return driver.startRecover(operationTiming, token);
  }
  token = {};
  return Status::Error(Err::INVALID_PARAM, "invalid maintenance test case");
}

}  // namespace

void setUp() {}
void tearDown() {}

// --------------------------------------------------------------------------
// Pure validation, timing, conversion, and portability behavior
// --------------------------------------------------------------------------

void test_status_contract() {
  TEST_ASSERT_TRUE(Status::Ok().ok());
  const Status error = Status::Error(Err::I2C_TIMEOUT, "timeout", -42);
  TEST_ASSERT_FALSE(error.ok());
  TEST_ASSERT_TRUE(error.is(Err::I2C_TIMEOUT));
  TEST_ASSERT_EQUAL_INT32(-42, error.detail);
  TEST_ASSERT_NOT_NULL(error.msg);
}

void test_validate_driver_config_and_address() {
  FakeBus bus;
  DriverConfig config = makeDriverConfig(bus);
  TEST_ASSERT_TRUE(validateDriverConfig(config).ok());

  config.i2cWrite = nullptr;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(validateDriverConfig(config).code));
  config = makeDriverConfig(bus);
  config.i2cWriteRead = nullptr;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(validateDriverConfig(config).code));
  config = makeDriverConfig(bus);
  config.i2cTimeoutMs = 0;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(validateDriverConfig(config).code));
  config = makeDriverConfig(bus);
  config.address = static_cast<SensorAddress>(0x69);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(validateDriverConfig(config).code));
}

void test_validate_profile_rejects_all_unsupported_or_unsafe_values() {
  DeviceProfile profile = makeProfile();
  TEST_ASSERT_TRUE(validateProfile(profile).ok());

  profile.blockDataUpdate = false;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::UNSUPPORTED_PROFILE),
                          static_cast<uint8_t>(validateProfile(profile).code));
  profile = makeProfile();
  profile.fifo.enabled = true;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::UNSUPPORTED_PROFILE),
                          static_cast<uint8_t>(validateProfile(profile).code));
  profile = makeProfile();
  profile.interrupts.enabled = true;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::UNSUPPORTED_PROFILE),
                          static_cast<uint8_t>(validateProfile(profile).code));
  profile = makeProfile();
  profile.accelOdr = Odr::HZ_1_6;
  profile.accelPowerMode = AccelPowerMode::HIGH_PERFORMANCE;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(validateProfile(profile).code));
  profile = makeProfile();
  profile.gyroOdr = Odr::HZ_1_6;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(validateProfile(profile).code));
  profile = makeProfile();
  profile.accelFullScale = static_cast<AccelFs>(0x7F);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(validateProfile(profile).code));
  profile = makeProfile();
  profile.gyroFullScale = static_cast<GyroFs>(0x7E);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(validateProfile(profile).code));
}

void test_all_odr_periods_are_bounded_and_ordered() {
  TEST_ASSERT_EQUAL_UINT64(0u, odrPeriodUs(Odr::POWER_DOWN));
  TEST_ASSERT_EQUAL_UINT64(625000u, odrPeriodUs(Odr::HZ_1_6));
  TEST_ASSERT_EQUAL_UINT64(80000u, odrPeriodUs(Odr::HZ_12_5));
  const Odr increasing[] = {Odr::HZ_12_5, Odr::HZ_26,   Odr::HZ_52,  Odr::HZ_104,
                            Odr::HZ_208,  Odr::HZ_416,  Odr::HZ_833, Odr::HZ_1660,
                            Odr::HZ_3330, Odr::HZ_6660};
  uint64_t previous = odrPeriodUs(increasing[0]);
  for (size_t i = 1; i < sizeof(increasing) / sizeof(increasing[0]); ++i) {
    const uint64_t current = odrPeriodUs(increasing[i]);
    TEST_ASSERT_GREATER_THAN_UINT64(0u, current);
    TEST_ASSERT_LESS_THAN_UINT64(previous, current);
    previous = current;
  }
  TEST_ASSERT_EQUAL_UINT64(0u, odrPeriodUs(static_cast<Odr>(0xFF)));
}

void test_all_accel_sensitivities_and_edges() {
  const AccelFs scales[] = {AccelFs::G_2, AccelFs::G_4, AccelFs::G_8, AccelFs::G_16};
  const int32_t expected[] = {61, 122, 244, 488};
  for (size_t i = 0; i < 4; ++i) {
    int32_t sensitivity = -1;
    TEST_ASSERT_TRUE(accelSensitivityMicroGPerLsb(scales[i], sensitivity).ok());
    TEST_ASSERT_EQUAL_INT32(expected[i], sensitivity);
    IntegerAxes out;
    TEST_ASSERT_TRUE(decodeAcceleration(RawAxes{32767, -32768, 0}, scales[i], out).ok());
    TEST_ASSERT_EQUAL_INT32(32767 * expected[i], out.x);
    TEST_ASSERT_EQUAL_INT32(-32768 * expected[i], out.y);
  }
  int32_t unchanged = 123;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_PARAM),
      static_cast<uint8_t>(
          accelSensitivityMicroGPerLsb(static_cast<AccelFs>(0x7F), unchanged).code));
  TEST_ASSERT_EQUAL_INT32(123, unchanged);
}

void test_all_gyro_sensitivities_and_full_int16_range() {
  const GyroFs scales[] = {GyroFs::DPS_125, GyroFs::DPS_250, GyroFs::DPS_500,
                           GyroFs::DPS_1000, GyroFs::DPS_2000};
  const int32_t expected[] = {4375, 8750, 17500, 35000, 70000};
  for (size_t i = 0; i < 5; ++i) {
    int32_t sensitivity = -1;
    TEST_ASSERT_TRUE(gyroSensitivityMicroDpsPerLsb(scales[i], sensitivity).ok());
    TEST_ASSERT_EQUAL_INT32(expected[i], sensitivity);
    IntegerAxes out;
    TEST_ASSERT_TRUE(decodeAngularRate(RawAxes{32767, -32768, 0}, scales[i], out).ok());
    TEST_ASSERT_EQUAL_INT64(static_cast<int64_t>(32767) * expected[i], out.x);
    TEST_ASSERT_EQUAL_INT64(static_cast<int64_t>(-32768) * expected[i], out.y);
  }
  TEST_ASSERT_GREATER_THAN_INT64(
      std::numeric_limits<int32_t>::max(), static_cast<int64_t>(32767) * 70000);
}

void test_temperature_decode_boundaries() {
  TEST_ASSERT_EQUAL_INT32(25000, decodeTemperatureMilliC(0));
  TEST_ASSERT_EQUAL_INT32(26000, decodeTemperatureMilliC(256));
  TEST_ASSERT_EQUAL_INT32(24000, decodeTemperatureMilliC(-256));
  TEST_ASSERT_EQUAL_INT32(152996, decodeTemperatureMilliC(32767));
  TEST_ASSERT_EQUAL_INT32(-103000, decodeTemperatureMilliC(-32768));
}

void test_convert_sample_is_self_contained_and_preserves_provenance() {
  RawSampleResult raw;
  raw.accel = RawAxes{100, -100, 1};
  raw.gyro = RawAxes{100, -100, 1};
  raw.temperatureRaw = 256;
  raw.validMask = SAMPLE_ALL;
  raw.freshMask = SAMPLE_ACCELERATION;
  raw.sequence = 77;
  raw.configGeneration = 42;
  raw.readUptimeMs = UINT64_C(0x100000001);
  raw.accelFullScale = AccelFs::G_16;
  raw.gyroFullScale = GyroFs::DPS_2000;

  ConvertedSample converted;
  TEST_ASSERT_TRUE(convertSample(raw, converted).ok());
  TEST_ASSERT_EQUAL_INT32(48800, converted.accelMicroG.x);
  TEST_ASSERT_EQUAL_INT64(7000000, converted.gyroMicroDps.x);
  TEST_ASSERT_EQUAL_INT32(26000, converted.temperatureMilliC);
  TEST_ASSERT_EQUAL_UINT8(raw.validMask, converted.validMask);
  TEST_ASSERT_EQUAL_UINT8(raw.freshMask, converted.freshMask);
  TEST_ASSERT_EQUAL_UINT64(raw.sequence, converted.sequence);
  TEST_ASSERT_EQUAL_UINT32(raw.configGeneration, converted.configGeneration);
  TEST_ASSERT_EQUAL_UINT64(raw.readUptimeMs, converted.readUptimeMs);
}

void test_bias_and_calibration_validation_reject_non_finite_values() {
  CalibrationRequest request;
  request.samples = 0;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(validateCalibrationRequest(request).code));
  request.samples = 1001;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(validateCalibrationRequest(request).code));
  request.samples = 10;
  request.expectedAccelerationG.x = std::numeric_limits<float>::quiet_NaN();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(validateCalibrationRequest(request).code));

  Axes sample{1.0f, 2.0f, 3.0f};
  const Axes original = sample;
  const Axes invalidBias{std::numeric_limits<float>::infinity(), 0.0f, 0.0f};
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(applyBias(sample, invalidBias).code));
  TEST_ASSERT_EQUAL_FLOAT(original.x, sample.x);
  TEST_ASSERT_EQUAL_FLOAT(original.y, sample.y);
  TEST_ASSERT_EQUAL_FLOAT(original.z, sample.z);
  TEST_ASSERT_TRUE(applyBias(sample, Axes{0.5f, 1.0f, -1.0f}).ok());
  TEST_ASSERT_EQUAL_FLOAT(0.5f, sample.x);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, sample.y);
  TEST_ASSERT_EQUAL_FLOAT(4.0f, sample.z);
}

// --------------------------------------------------------------------------
// Zero-I/O lifecycle, result identity, and owner scheduling
// --------------------------------------------------------------------------

void test_bind_unbind_and_rebind_are_bus_silent() {
  FakeBus first;
  FakeBus second;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(first)).ok());
  TEST_ASSERT_TRUE(driver.isBound());
  TEST_ASSERT_EQUAL_UINT32(0u, first.transferCalls);

  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(second)).ok());
  TEST_ASSERT_EQUAL_UINT32(0u, first.transferCalls);
  TEST_ASSERT_EQUAL_UINT32(0u, second.transferCalls);
  driver.unbind();
  TEST_ASSERT_FALSE(driver.isBound());
  TEST_ASSERT_FALSE(driver.operationActive());
  TEST_ASSERT_FALSE(driver.resultPending());
  TEST_ASSERT_EQUAL_UINT32(0u, first.transferCalls + second.transferCalls);
}

void test_invalid_rebind_preserves_working_binding() {
  FakeBus first;
  FakeBus rejected;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(first)).ok());
  DriverConfig invalid = makeDriverConfig(rejected);
  invalid.i2cWriteRead = nullptr;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(driver.bind(invalid).code));

  OperationToken token;
  TEST_ASSERT_TRUE(driver.startProbe(timing(first), token).inProgress());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(runToTerminal(driver, first).state));
  TEST_ASSERT_EQUAL_UINT32(MAX_PROBE_TRANSACTIONS, first.transferCalls);
  TEST_ASSERT_EQUAL_UINT32(0u, rejected.transferCalls);
  (void)take(driver, token);
}

void test_unbound_and_invalid_timing_starts_are_zero_io() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  OperationToken token{99};
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_BOUND),
                          static_cast<uint8_t>(driver.startProbe(timing(bus), token).code));
  TEST_ASSERT_FALSE(token.valid());
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  token.value = 99;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_PARAM),
      static_cast<uint8_t>(
          driver.startProbe(OperationTiming{bus.nowMs, bus.nowMs}, token).code));
  TEST_ASSERT_FALSE(token.valid());
  TEST_ASSERT_EQUAL_UINT32(0u, bus.transferCalls);
}

void test_probe_budget_zero_then_one_and_exactly_once_result() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startProbe(timing(bus), token).inProgress());
  TEST_ASSERT_TRUE(token.valid());

  PollResult poll = driver.poll(bus.nowMs, 0);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::ACTIVE),
                          static_cast<uint8_t>(poll.state));
  TEST_ASSERT_EQUAL_UINT8(0u, poll.transactionsUsed);
  TEST_ASSERT_EQUAL_UINT32(0u, bus.transferCalls);
  poll = driver.poll(bus.nowMs, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::ACTIVE),
                          static_cast<uint8_t>(poll.state));
  TEST_ASSERT_EQUAL_UINT8(1u, poll.transactionsUsed);
  poll = driver.poll(bus.nowMs, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(poll.state));
  TEST_ASSERT_EQUAL_UINT8(1u, poll.transactionsUsed);

  OperationResult result = take(driver, token);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(JobKind::PROBE),
                          static_cast<uint8_t>(result.kind));
  TEST_ASSERT_EQUAL_HEX8(cmd::WHO_AM_I_VALUE, result.probe.whoAmI);
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(SensorAddress::SA0_GND),
                         result.probe.address);
  TEST_ASSERT_EQUAL_UINT32(MAX_PROBE_TRANSACTIONS, result.transactionLimit);
  TEST_ASSERT_EQUAL_UINT32(MAX_PROBE_TRANSACTIONS, result.transactions);
  OperationResult duplicate;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::RESULT_NOT_AVAILABLE),
                          static_cast<uint8_t>(driver.takeResult(token, duplicate).code));
}

void test_wrong_token_does_not_consume_pending_result() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startProbe(timing(bus), token).inProgress());
  (void)runToTerminal(driver, bus);
  OperationResult wrong;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::STALE_RESULT),
                          static_cast<uint8_t>(
                              driver.takeResult(OperationToken{token.value + 1u}, wrong).code));
  TEST_ASSERT_TRUE(driver.resultPending());
  (void)take(driver, token);
}

void test_active_and_pending_operations_cannot_be_overwritten() {
  FakeBus bus;
  FakeBus replacementBus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  OperationToken first;
  OperationToken rejected{55};
  TEST_ASSERT_TRUE(driver.startProbe(timing(bus), first).inProgress());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(driver.startProbe(timing(bus), rejected).code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(driver.bind(makeDriverConfig(replacementBus)).code));
  TEST_ASSERT_FALSE(rejected.valid());
  (void)runToTerminal(driver, bus);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::RESULT_PENDING),
                          static_cast<uint8_t>(driver.startProbe(timing(bus), rejected).code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::RESULT_PENDING),
                          static_cast<uint8_t>(driver.bind(makeDriverConfig(replacementBus)).code));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::RESULT_PENDING),
      static_cast<uint8_t>(
          driver.diagnosticWriteRegister(cmd::REG_CTRL1_XL, 0, bus.nowMs).code));
  TEST_ASSERT_EQUAL_UINT32(0u, replacementBus.transferCalls);
  TEST_ASSERT_FALSE(rejected.valid());
  (void)take(driver, first);
  TEST_ASSERT_TRUE(driver.startProbe(timing(bus), rejected).inProgress());
  TEST_ASSERT_TRUE(rejected.valid());
  TEST_ASSERT_NOT_EQUAL(first.value, rejected.value);
}

void test_probe_failures_are_precise_and_never_gate_later_work() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  bus.addFault(1, Status::Error(Err::I2C_NACK_ADDR, "absent", -77));
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startProbe(timing(bus), token).inProgress());
  PollResult terminal = runToTerminal(driver, bus);
  OperationResult result = take(driver, token);
  assertTerminalFailure(terminal, result, Err::I2C_NACK_ADDR, OperationState::FAILED);
  TEST_ASSERT_EQUAL_INT32(-77, result.status.detail);
  DriverDiagnostics diagnostics = driver.diagnostics(bus.nowMs);
  TEST_ASSERT_EQUAL_UINT32(1u, diagnostics.transportFailures);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_ADDR),
                          static_cast<uint8_t>(diagnostics.lastTransportError.code));
  TEST_ASSERT_EQUAL_UINT64(bus.trace[0].atMs,
                           diagnostics.lastTransportErrorUptimeMs);

  bus.clearTrace();
  TEST_ASSERT_TRUE(driver.startProbe(timing(bus), token).inProgress());
  terminal = runToTerminal(driver, bus);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(terminal.state));
  (void)take(driver, token);
  diagnostics = driver.diagnostics(bus.nowMs);
  TEST_ASSERT_EQUAL_UINT32(1u, diagnostics.transportFailures);
  TEST_ASSERT_EQUAL_UINT32(MAX_PROBE_TRANSACTIONS,
                           diagnostics.transportSuccesses);
}

void test_probe_deadline_and_uint64_boundary_are_safe() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  OperationToken token;
  TEST_ASSERT_TRUE(driver
                       .startProbe(OperationTiming{bus.nowMs, bus.nowMs + 1u}, token)
                       .inProgress());
  PollResult terminal = driver.poll(bus.nowMs + 1u, 1);
  TEST_ASSERT_EQUAL_UINT32(0u, bus.transferCalls);
  OperationResult result = take(driver, token);
  assertTerminalFailure(terminal, result, Err::DEADLINE_EXPIRED,
                        OperationState::TIMED_OUT);

  bus.nowMs = std::numeric_limits<uint64_t>::max() - 2u;
  TEST_ASSERT_TRUE(driver
                       .startProbe(OperationTiming{
                                       bus.nowMs,
                                       std::numeric_limits<uint64_t>::max()},
                                   token)
                       .inProgress());
  terminal = driver.poll(bus.nowMs + 1u, 2);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(terminal.state));
  (void)take(driver, token);
}

// --------------------------------------------------------------------------
// Configure, readback, cancellation, ambiguity, and cache provenance
// --------------------------------------------------------------------------

void test_configure_enforces_budget_and_verifies_before_generation_increment() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startConfigure(makeProfile(), timing(bus), token).inProgress());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::APPLYING),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
  TEST_ASSERT_EQUAL_UINT32(0u, driver.configGeneration());
  TEST_ASSERT_EQUAL_UINT32(0u, bus.transferCalls);

  const PollResult zero = driver.poll(bus.nowMs, 0);
  TEST_ASSERT_EQUAL_UINT8(0u, zero.transactionsUsed);
  TEST_ASSERT_EQUAL_UINT32(0u, bus.transferCalls);
  const PollResult terminal = runToTerminal(driver, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(terminal.state));
  const OperationResult result = take(driver, token);
  TEST_ASSERT_EQUAL_UINT32(1u, result.configuration.generation);
  TEST_ASSERT_EQUAL_UINT32(1u, driver.configGeneration());
  TEST_ASSERT_GREATER_THAN_UINT32(0u, result.transactions);
  TEST_ASSERT_EQUAL_UINT32(bus.transferCalls, result.transactions);
  TEST_ASSERT_EQUAL_UINT32(MAX_CONFIGURE_TRANSACTIONS, result.transactionLimit);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(result.transactionLimit, result.transactions);
  TEST_ASSERT_TRUE(result.configuration.state == ConfigurationState::SETTLING ||
                   result.configuration.state == ConfigurationState::KNOWN);
}

void test_complete_profile_clears_all_disabled_subsystems() {
  FakeBus bus;
  bus.regs[cmd::REG_SENSOR_SYNC_TIME_FRAME] = 0xFF;
  bus.regs[cmd::REG_SENSOR_SYNC_RES_RATIO] = 0xFF;
  bus.regs[cmd::REG_FIFO_CTRL1] = 0xFF;
  bus.regs[cmd::REG_FIFO_CTRL2] = 0xFF;
  bus.regs[cmd::REG_FIFO_CTRL3] = 0xFF;
  bus.regs[cmd::REG_FIFO_CTRL4] = 0xFF;
  bus.regs[cmd::REG_FIFO_CTRL5] = 0xFF;
  bus.regs[cmd::REG_DRDY_PULSE_CFG_G] = 0xFF;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  for (uint8_t reg = cmd::REG_FIFO_CTRL1; reg <= cmd::REG_FIFO_CTRL5; ++reg) {
    TEST_ASSERT_EQUAL_HEX8(0u, bus.regs[reg]);
    TEST_ASSERT_LESS_THAN(bus.traceCount, findWrite(bus, reg));
  }
  TEST_ASSERT_EQUAL_HEX8(0u, bus.regs[cmd::REG_SENSOR_SYNC_TIME_FRAME]);
  TEST_ASSERT_EQUAL_HEX8(0u, bus.regs[cmd::REG_SENSOR_SYNC_RES_RATIO]);
  TEST_ASSERT_EQUAL_HEX8(0u, bus.regs[cmd::REG_DRDY_PULSE_CFG_G]);
  TEST_ASSERT_LESS_THAN(bus.traceCount,
                        findWrite(bus, cmd::REG_SENSOR_SYNC_TIME_FRAME));
  TEST_ASSERT_LESS_THAN(bus.traceCount,
                        findWrite(bus, cmd::REG_SENSOR_SYNC_RES_RATIO));
  TEST_ASSERT_LESS_THAN(bus.traceCount,
                        findWrite(bus, cmd::REG_DRDY_PULSE_CFG_G));
}

void test_configure_failure_at_every_transport_stage_has_precise_effect_state() {
  FakeBus referenceBus;
  LSM6DS3TR::LSM6DS3TR referenceDriver;
  TEST_ASSERT_TRUE(referenceDriver.bind(makeDriverConfig(referenceBus)).ok());
  OperationToken referenceToken;
  TEST_ASSERT_TRUE(referenceDriver
                       .startConfigure(makeProfile(), timing(referenceBus), referenceToken)
                       .inProgress());
  (void)runToTerminal(referenceDriver, referenceBus, 4);
  const uint32_t stageCount = referenceBus.transferCalls;
  TEST_ASSERT_GREATER_THAN_UINT32(1u, stageCount);
  (void)take(referenceDriver, referenceToken);

  for (uint32_t stage = 1; stage <= stageCount; ++stage) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    bus.addFault(stage, Status::Error(Err::I2C_TIMEOUT, "stage failure",
                                     static_cast<int32_t>(stage)));
    OperationToken token;
    TEST_ASSERT_TRUE(
        driver.startConfigure(makeProfile(), timing(bus), token).inProgress());
    const PollResult terminal = runToTerminal(driver, bus, 3);
    const OperationResult result = take(driver, token);
    assertTerminalFailure(
        terminal, result, Err::I2C_TIMEOUT,
        stage <= 2u ? OperationState::FAILED : OperationState::INDETERMINATE);
    TEST_ASSERT_EQUAL(stage > 2u, result.hardwareStateMayHaveChanged);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(stage <= 2u ? ConfigurationState::UNCONFIGURED
                                         : ConfigurationState::UNKNOWN),
        static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
    TEST_ASSERT_EQUAL_UINT32(0u, driver.configGeneration());
  }
}

void test_ambiguous_write_effect_is_observable_and_never_retried() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  bus.addFault(3, Status::Error(Err::I2C_TIMEOUT, "ambiguous", -88), true);
  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startConfigure(makeProfile(), timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 4);
  const OperationResult result = take(driver, token);
  assertTerminalFailure(terminal, result, Err::I2C_TIMEOUT,
                        OperationState::INDETERMINATE);
  TEST_ASSERT_EQUAL_UINT32(3u, bus.transferCalls);
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_FUNC_CFG_ACCESS, bus.trace[0].startReg);
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_WHO_AM_I, bus.trace[1].startReg);
  TEST_ASSERT_TRUE(bus.trace[2].effectApplied);
  TEST_ASSERT_TRUE(result.hardwareStateMayHaveChanged);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNKNOWN),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
}

void test_configuration_readback_mismatch_reports_exact_register_values() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  bus.corruptReadRegister = cmd::REG_CTRL1_XL;
  bus.corruptReadValue = 0xFF;
  bus.corruptReadRemaining = 1;
  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startConfigure(makeProfile(), timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 2);
  const OperationResult result = take(driver, token);
  assertTerminalFailure(terminal, result, Err::CONFIGURATION_MISMATCH,
                        OperationState::FAILED);
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_CTRL1_XL, result.configuration.mismatchRegister);
  TEST_ASSERT_EQUAL_HEX8(0xFF, result.configuration.observedValue);
  TEST_ASSERT_NOT_EQUAL(result.configuration.expectedValue,
                        result.configuration.observedValue);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNKNOWN),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
  TEST_ASSERT_EQUAL_UINT32(0u, driver.configGeneration());

  DriverDiagnostics diagnostics = driver.diagnostics(bus.nowMs);
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_CTRL1_XL, diagnostics.mismatchRegister);
  TEST_ASSERT_EQUAL_HEX8(result.configuration.expectedValue,
                         diagnostics.mismatchExpected);
  TEST_ASSERT_EQUAL_HEX8(0xFF, diagnostics.mismatchObserved);

  TEST_ASSERT_TRUE(driver.startReconcile(timing(bus), token).inProgress());
  const PollResult repairedTerminal = runToTerminal(driver, bus, 2);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(repairedTerminal.state));
  const OperationResult repaired = take(driver, token);
  TEST_ASSERT_EQUAL_HEX8(0u, repaired.configuration.mismatchRegister);
  TEST_ASSERT_EQUAL_HEX8(0u, repaired.configuration.expectedValue);
  TEST_ASSERT_EQUAL_HEX8(0u, repaired.configuration.observedValue);
  diagnostics = driver.diagnostics(bus.nowMs);
  TEST_ASSERT_EQUAL_HEX8(0u, diagnostics.mismatchRegister);
  TEST_ASSERT_EQUAL_HEX8(0u, diagnostics.mismatchExpected);
  TEST_ASSERT_EQUAL_HEX8(0u, diagnostics.mismatchObserved);
}

void test_cancel_before_and_after_configuration_effect_is_bus_silent() {
  FakeBus beforeBus;
  LSM6DS3TR::LSM6DS3TR beforeDriver;
  TEST_ASSERT_TRUE(beforeDriver.bind(makeDriverConfig(beforeBus)).ok());
  OperationToken token;
  TEST_ASSERT_TRUE(beforeDriver
                       .startConfigure(makeProfile(), timing(beforeBus), token)
                       .inProgress());
  TEST_ASSERT_TRUE(beforeDriver.cancelActiveJob(beforeBus.nowMs).ok());
  TEST_ASSERT_EQUAL_UINT32(0u, beforeBus.transferCalls);
  OperationResult result = take(beforeDriver, token);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::CANCELLED),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CANCELLED),
                          static_cast<uint8_t>(result.status.code));
  TEST_ASSERT_FALSE(result.hardwareStateMayHaveChanged);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNCONFIGURED),
                          static_cast<uint8_t>(beforeDriver.configurationState(beforeBus.nowMs)));

  FakeBus afterBus;
  LSM6DS3TR::LSM6DS3TR afterDriver;
  TEST_ASSERT_TRUE(afterDriver.bind(makeDriverConfig(afterBus)).ok());
  TEST_ASSERT_TRUE(afterDriver
                       .startConfigure(makeProfile(), timing(afterBus), token)
                       .inProgress());
  PollResult poll = afterDriver.poll(afterBus.nowMs, 1);
  TEST_ASSERT_EQUAL_UINT8(1u, poll.transactionsUsed);
  const uint32_t transfers = afterBus.transferCalls;
  TEST_ASSERT_TRUE(afterDriver.cancelActiveJob(afterBus.nowMs).ok());
  TEST_ASSERT_EQUAL_UINT32(transfers, afterBus.transferCalls);
  result = take(afterDriver, token);
  TEST_ASSERT_FALSE(result.hardwareStateMayHaveChanged);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNCONFIGURED),
                          static_cast<uint8_t>(afterDriver.configurationState(afterBus.nowMs)));
}

void test_configure_cancellation_is_safe_after_every_transfer_stage() {
  for (uint32_t completed = 0; completed <= MAX_CONFIGURE_TRANSACTIONS; ++completed) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    OperationToken token;
    TEST_ASSERT_TRUE(
        driver.startConfigure(makeProfile(), timing(bus), token).inProgress());
    for (uint32_t transfer = 0; transfer < completed; ++transfer) {
      const PollResult poll = driver.poll(bus.nowMs, 1);
      TEST_ASSERT_EQUAL_UINT8(1u, poll.transactionsUsed);
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::ACTIVE),
                              static_cast<uint8_t>(poll.state));
      bus.nowMs++;
    }
    const uint32_t transfers = bus.transferCalls;
    TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
    TEST_ASSERT_EQUAL_UINT32(transfers, bus.transferCalls);
    const OperationResult result = take(driver, token);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::CANCELLED),
                            static_cast<uint8_t>(result.state));
    TEST_ASSERT_EQUAL_UINT32(completed, result.transactions);
    if (completed <= 2u) {
      TEST_ASSERT_FALSE(result.hardwareStateMayHaveChanged);
      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(ConfigurationState::UNCONFIGURED),
          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
    } else {
      TEST_ASSERT_TRUE(result.hardwareStateMayHaveChanged);
      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(ConfigurationState::UNKNOWN),
          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
    }
  }
}

void test_timeout_after_partial_configuration_exposes_unknown_state() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  OperationToken token;
  TEST_ASSERT_TRUE(driver
                       .startConfigure(makeProfile(),
                                       OperationTiming{bus.nowMs, bus.nowMs + 1u}, token)
                       .inProgress());
  TEST_ASSERT_EQUAL_UINT8(3u, driver.poll(bus.nowMs, 3).transactionsUsed);
  const uint32_t transfers = bus.transferCalls;
  const PollResult terminal = driver.poll(bus.nowMs + 1u, 4);
  TEST_ASSERT_EQUAL_UINT32(transfers, bus.transferCalls);
  const OperationResult result = take(driver, token);
  assertTerminalFailure(terminal, result, Err::DEADLINE_EXPIRED,
                        OperationState::TIMED_OUT);
  TEST_ASSERT_TRUE(result.hardwareStateMayHaveChanged);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNKNOWN),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs + 1u)));
}

void test_configure_terminal_publishes_only_after_settled_and_verified() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  const OperationResult configured = configure(driver, bus);
  DeviceProfile desired;
  TEST_ASSERT_TRUE(driver.getDesiredProfile(desired).ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(makeProfile().accelOdr),
                          static_cast<uint8_t>(desired.accelOdr));
  DeviceProfile verified;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::KNOWN),
                          static_cast<uint8_t>(configured.configuration.state));
  TEST_ASSERT_TRUE(driver.getVerifiedProfile(verified, bus.nowMs).ok());
}

void test_reconcile_is_read_only_and_only_full_verification_clears_unknown() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  const uint8_t desiredCtrl1 = bus.regs[cmd::REG_CTRL1_XL];
  TEST_ASSERT_TRUE(
      driver.diagnosticWriteRegister(cmd::REG_CTRL1_XL, desiredCtrl1, bus.nowMs).ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNKNOWN),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
  bus.clearTrace();

  OperationToken token;
  TEST_ASSERT_TRUE(driver.startReconcile(timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(terminal.state));
  (void)take(driver, token);
  for (size_t i = 0; i < bus.traceCount; ++i) {
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TransferKind::WRITE_READ),
                            static_cast<uint8_t>(bus.trace[i].kind));
  }
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::KNOWN),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
}

void test_identity_mismatch_invalidates_all_identity_dependent_jobs() {
  {
    FakeBus bus;
    bus.regs[cmd::REG_WHO_AM_I] = 0x00;
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    OperationToken token;
    TEST_ASSERT_TRUE(
        driver.startConfigure(makeProfile(), timing(bus), token).inProgress());
    const PollResult terminal = runToTerminal(driver, bus, 1);
    const OperationResult result = take(driver, token);
    assertTerminalFailure(terminal, result, Err::CHIP_ID_MISMATCH,
                          OperationState::FAILED);
    TEST_ASSERT_FALSE(result.hardwareStateMayHaveChanged);
    TEST_ASSERT_EQUAL_HEX8(0x00, result.probe.whoAmI);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(ConfigurationState::UNKNOWN),
        static_cast<uint8_t>(driver.configurationState(bus.nowMs)));

    // A failed configure still retains the desired profile. Reset must not use
    // that fact as a substitute for proving the live device identity.
    bus.clearTrace();
    bus.regs[cmd::REG_WHO_AM_I] = 0x66;
    TEST_ASSERT_TRUE(driver.startReset(timing(bus), token).inProgress());
    const PollResult resetTerminal = runToTerminal(driver, bus, 1);
    const OperationResult resetResult = take(driver, token);
    assertTerminalFailure(resetTerminal, resetResult, Err::CHIP_ID_MISMATCH,
                          OperationState::INDETERMINATE);
    TEST_ASSERT_TRUE(resetResult.hardwareStateMayHaveChanged);
    TEST_ASSERT_EQUAL_HEX8(0x66, resetResult.probe.whoAmI);
    TEST_ASSERT_EQUAL_UINT32(3u, bus.transferCalls);
    TEST_ASSERT_EQUAL_HEX8(cmd::REG_FUNC_CFG_ACCESS, bus.trace[0].startReg);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TransferKind::WRITE),
                            static_cast<uint8_t>(bus.trace[0].kind));
    TEST_ASSERT_EQUAL_HEX8(cmd::REG_FUNC_CFG_ACCESS, bus.trace[1].startReg);
    TEST_ASSERT_EQUAL_HEX8(cmd::REG_WHO_AM_I, bus.trace[2].startReg);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(ConfigurationState::UNKNOWN),
        static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
  }

  for (uint8_t operation = 0; operation < 3u; ++operation) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    (void)configure(driver, bus);
    bus.clearTrace();
    bus.regs[cmd::REG_WHO_AM_I] = 0x55;
    OperationToken token;
    Status started;
    if (operation == 0u) {
      started = driver.startProbe(timing(bus), token);
    } else if (operation == 1u) {
      started = driver.startReconcile(timing(bus), token);
    } else {
      started = driver.startRecover(timing(bus), token);
    }
    TEST_ASSERT_TRUE(started.inProgress());
    const PollResult terminal = runToTerminal(driver, bus, 1);
    const OperationResult result = take(driver, token);
    assertTerminalFailure(terminal, result, Err::CHIP_ID_MISMATCH,
                          OperationState::FAILED);
    TEST_ASSERT_FALSE(result.hardwareStateMayHaveChanged);
    TEST_ASSERT_EQUAL_HEX8(0x55, result.probe.whoAmI);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(ConfigurationState::UNKNOWN),
        static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
    DeviceProfile unavailable;
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::CONFIGURATION_UNKNOWN),
        static_cast<uint8_t>(
            driver.getVerifiedProfile(unavailable, bus.nowMs).code));
  }

  {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    (void)configure(driver, bus);
    bus.clearTrace();
    bus.regs[cmd::REG_WHO_AM_I] = 0x77;
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startBoot(timing(bus), token).inProgress());
    const PollResult terminal = runToTerminal(driver, bus, 1);
    const OperationResult result = take(driver, token);
    assertTerminalFailure(terminal, result, Err::CHIP_ID_MISMATCH,
                          OperationState::INDETERMINATE);
    TEST_ASSERT_TRUE(result.hardwareStateMayHaveChanged);
    TEST_ASSERT_EQUAL_HEX8(0x77, result.probe.whoAmI);
    TEST_ASSERT_EQUAL_UINT32(3u, bus.transferCalls);
    TEST_ASSERT_EQUAL_HEX8(cmd::REG_WHO_AM_I, bus.trace[2].startReg);
  }
}

// --------------------------------------------------------------------------
// Atomic samples, field masks, freshness, and immutable conversion provenance
// --------------------------------------------------------------------------

void test_sample_is_atomic_tokened_and_validity_aware() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  bus.setRawSample(6400, 1000, -500, 0, 16384, -1, -32768);

  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startSample(SampleRequest{}, timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(terminal.state));
  const OperationResult result = take(driver, token);
  TEST_ASSERT_EQUAL_UINT8(SAMPLE_ALL, result.sample.validMask);
  TEST_ASSERT_EQUAL_UINT8(SAMPLE_ALL, result.sample.freshMask);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleQuality::READY_CHECKED),
                          static_cast<uint8_t>(result.sample.quality));
  TEST_ASSERT_EQUAL_INT16(16384, result.sample.accel.x);
  TEST_ASSERT_EQUAL_INT16(-32768, result.sample.accel.z);
  TEST_ASSERT_EQUAL_INT16(1000, result.sample.gyro.x);
  TEST_ASSERT_EQUAL_INT16(6400, result.sample.temperatureRaw);
  TEST_ASSERT_EQUAL_UINT64(1u, result.sample.sequence);
  TEST_ASSERT_EQUAL_UINT32(driver.configGeneration(), result.sample.configGeneration);
  TEST_ASSERT_EQUAL_UINT64(bus.trace[bus.traceCount - 1u].atMs,
                           result.sample.readUptimeMs);
}

void test_direct_unverified_sample_has_no_false_freshness() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  SampleRequest request;
  request.checkDataReady = false;
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startSample(request, timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(terminal.state));
  const OperationResult result = take(driver, token);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleQuality::DIRECT_UNVERIFIED),
                          static_cast<uint8_t>(result.sample.quality));
  TEST_ASSERT_EQUAL_UINT8(0u, result.sample.freshMask);
  TEST_ASSERT_EQUAL_UINT32(1u, bus.transferCalls);
}

void test_powered_down_sensor_combinations_have_exact_valid_masks() {
  struct Case {
    Odr accel;
    Odr gyro;
    uint8_t expected;
  };
  const Case cases[] = {
      {Odr::HZ_104, Odr::POWER_DOWN, SAMPLE_ACCELERATION | SAMPLE_TEMPERATURE},
      {Odr::POWER_DOWN, Odr::HZ_104, SAMPLE_ANGULAR_RATE | SAMPLE_TEMPERATURE},
  };
  for (const Case& current : cases) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    DeviceProfile profile = makeProfile();
    profile.accelOdr = current.accel;
    profile.gyroOdr = current.gyro;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    (void)configure(driver, bus, profile);
    bus.clearTrace();
    OperationToken token;
    SampleRequest request;
    request.quantityMask = current.expected;
    TEST_ASSERT_TRUE(driver.startSample(request, timing(bus), token).inProgress());
    (void)runToTerminal(driver, bus, 1);
    const OperationResult result = take(driver, token);
    TEST_ASSERT_EQUAL_UINT8(current.expected, result.sample.validMask);
    TEST_ASSERT_EQUAL_UINT8(current.expected, result.sample.freshMask);
  }

  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  DeviceProfile poweredDown = makeProfile();
  poweredDown.accelOdr = Odr::POWER_DOWN;
  poweredDown.gyroOdr = Odr::POWER_DOWN;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus, poweredDown);
  bus.clearTrace();
  OperationToken token{77};
  SampleRequest temperatureOnly;
  temperatureOnly.quantityMask = SAMPLE_TEMPERATURE;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_PARAM),
      static_cast<uint8_t>(
          driver.startSample(temperatureOnly, timing(bus), token).code));
  TEST_ASSERT_FALSE(token.valid());
  TEST_ASSERT_EQUAL_UINT32(0u, bus.transferCalls);
}

void test_sample_wait_steps_use_zero_i2c_and_timeout_without_stale_data() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  bus.notReadyStatusReads = 100;
  OperationToken token;
  TEST_ASSERT_TRUE(driver
                       .startSample(SampleRequest{},
                                    OperationTiming{bus.nowMs, bus.nowMs + 3u}, token)
                       .inProgress());
  PollResult poll = driver.poll(bus.nowMs, 4);
  TEST_ASSERT_EQUAL_UINT8(1u, poll.transactionsUsed);
  TEST_ASSERT_TRUE(poll.waiting);
  const uint32_t afterStatus = bus.transferCalls;
  poll = driver.poll(bus.nowMs, 0);
  TEST_ASSERT_EQUAL_UINT8(0u, poll.transactionsUsed);
  TEST_ASSERT_EQUAL_UINT32(afterStatus, bus.transferCalls);
  const PollResult terminal = driver.poll(bus.nowMs + 3u, 0);
  const OperationResult result = take(driver, token);
  assertTerminalFailure(terminal, result, Err::DEADLINE_EXPIRED,
                        OperationState::TIMED_OUT);
  TEST_ASSERT_EQUAL_UINT8(0u, result.sample.validMask);
  TEST_ASSERT_EQUAL_UINT8(0u, result.sample.freshMask);
}

void test_combined_sample_does_not_claim_fresh_temperature_without_tda() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  bus.regs[cmd::REG_STATUS_REG] =
      static_cast<uint8_t>(cmd::MASK_XLDA | cmd::MASK_GDA);
  OperationToken token;
  TEST_ASSERT_TRUE(driver
                       .startSample(SampleRequest{},
                                    OperationTiming{bus.nowMs, bus.nowMs + 2u}, token)
                       .inProgress());
  const PollResult first = driver.poll(bus.nowMs, 2);
  TEST_ASSERT_TRUE(first.waiting);
  TEST_ASSERT_EQUAL_UINT8(1u, first.transactionsUsed);
  TEST_ASSERT_EQUAL_UINT32(1u, bus.transferCalls);
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_STATUS_REG, bus.trace[0].startReg);
  const PollResult terminal = driver.poll(bus.nowMs + 2u, 0);
  const OperationResult result = take(driver, token);
  assertTerminalFailure(terminal, result, Err::DEADLINE_EXPIRED,
                        OperationState::TIMED_OUT);
  TEST_ASSERT_EQUAL_UINT8(0u, result.sample.validMask);
}

void test_sample_ready_on_last_check_succeeds_and_stuck_ready_is_bounded() {
  {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    (void)configure(driver, bus);
    bus.clearTrace();
    bus.notReadyStatusReads = 64;
    OperationToken token;
    TEST_ASSERT_TRUE(
        driver.startSample(SampleRequest{}, timing(bus), token).inProgress());
    const PollResult terminal = runToTerminal(driver, bus, 1);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                            static_cast<uint8_t>(terminal.state));
    const OperationResult result = take(driver, token);
    TEST_ASSERT_EQUAL_UINT32(MAX_SAMPLE_TRANSACTIONS, result.transactionLimit);
    TEST_ASSERT_EQUAL_UINT32(MAX_SAMPLE_TRANSACTIONS, result.transactions);
    TEST_ASSERT_EQUAL_UINT8(SAMPLE_ALL, result.sample.validMask);
    TEST_ASSERT_EQUAL_UINT8(SAMPLE_ALL, result.sample.freshMask);
    TEST_ASSERT_EQUAL_HEX8(cmd::REG_DATA_START_ALL,
                           bus.trace[bus.traceCount - 1u].startReg);
  }

  {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    (void)configure(driver, bus);
    bus.clearTrace();
    bus.notReadyStatusReads = 100;
    OperationToken token;
    TEST_ASSERT_TRUE(
        driver.startSample(SampleRequest{}, timing(bus), token).inProgress());
    const PollResult terminal = runToTerminal(driver, bus, 1);
    const OperationResult result = take(driver, token);
    assertTerminalFailure(terminal, result, Err::DATA_NOT_READY,
                          OperationState::FAILED);
    TEST_ASSERT_EQUAL_UINT32(MAX_SAMPLE_TRANSACTIONS, result.transactionLimit);
    TEST_ASSERT_EQUAL_UINT32(65u, result.transactions);
    TEST_ASSERT_EQUAL_UINT8(0u, result.sample.validMask);
    TEST_ASSERT_EQUAL_UINT8(0u, result.sample.freshMask);
    for (size_t i = 0; i < bus.traceCount; ++i) {
      TEST_ASSERT_EQUAL_HEX8(cmd::REG_STATUS_REG, bus.trace[i].startReg);
    }
  }
}

void test_failed_new_sample_cannot_publish_previous_sample() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  OperationToken goodToken;
  TEST_ASSERT_TRUE(
      driver.startSample(SampleRequest{}, timing(bus), goodToken).inProgress());
  (void)runToTerminal(driver, bus, 2);
  const OperationResult good = take(driver, goodToken);
  TEST_ASSERT_EQUAL_UINT64(1u, good.sample.sequence);

  bus.clearTrace();
  bus.addFault(1, Status::Error(Err::I2C_TIMEOUT, "sample fail", -55));
  OperationToken failedToken;
  TEST_ASSERT_TRUE(
      driver.startSample(SampleRequest{}, timing(bus), failedToken).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 2);
  const OperationResult failed = take(driver, failedToken);
  assertTerminalFailure(terminal, failed, Err::I2C_TIMEOUT, OperationState::FAILED);
  TEST_ASSERT_EQUAL_UINT8(0u, failed.sample.validMask);
  TEST_ASSERT_EQUAL_UINT8(0u, failed.sample.freshMask);
  OperationResult stale;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::RESULT_NOT_AVAILABLE),
                          static_cast<uint8_t>(driver.takeResult(goodToken, stale).code));
}

void test_sample_conversion_keeps_original_scale_after_reconfigure() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  bus.setRawSample(6400, 1000, -500, 0, 16384, 0, 0);
  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startSample(SampleRequest{}, timing(bus), token).inProgress());
  (void)runToTerminal(driver, bus, 2);
  const RawSampleResult original = take(driver, token).sample;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelFs::G_2),
                          static_cast<uint8_t>(original.accelFullScale));

  DeviceProfile changed = makeProfile();
  changed.accelFullScale = AccelFs::G_16;
  (void)configure(driver, bus, changed);
  ConvertedSample converted;
  TEST_ASSERT_TRUE(convertSample(original, converted).ok());
  TEST_ASSERT_EQUAL_INT32(16384 * 61, converted.accelMicroG.x);
}

void test_unknown_and_settling_configuration_gate_samples_without_i2c() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startConfigure(makeProfile(), timing(bus), token).inProgress());
  (void)runToTerminal(driver, bus, 2);
  const OperationResult configured = take(driver, token);
  bus.clearTrace();
  bus.nowMs = configured.configuration.validAfterUptimeMs;
  TEST_ASSERT_TRUE(driver.diagnosticWriteRegister(cmd::REG_CTRL1_XL, 0, bus.nowMs).ok());
  bus.clearTrace();
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::CONFIGURATION_UNKNOWN),
      static_cast<uint8_t>(driver.startSample(SampleRequest{}, timing(bus), token).code));
  TEST_ASSERT_FALSE(token.valid());
  TEST_ASSERT_EQUAL_UINT32(0u, bus.transferCalls);
}

// --------------------------------------------------------------------------
// Reset/boot sequencing, diagnostic exclusion, recovery, and maintenance
// --------------------------------------------------------------------------

void test_reset_orders_required_modes_and_enforces_no_i2c_gate() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  bus.commandInaccessibleMs = 15;
  bus.resetRegistersOnSoftwareReset = true;
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startReset(timing(bus), token).inProgress());

  size_t commandIndex = bus.traceCount;
  for (uint32_t i = 0; i < 20 && commandIndex == bus.traceCount; ++i) {
    (void)driver.poll(bus.nowMs, 1);
    commandIndex = findWrite(bus, cmd::REG_CTRL3_C, cmd::MASK_SW_RESET,
                             cmd::MASK_SW_RESET);
  }
  TEST_ASSERT_LESS_THAN(bus.traceCount, commandIndex);
  const size_t gyroDown = findWrite(bus, cmd::REG_CTRL2_G, 0xF0u, 0u);
  const size_t accelHighPerformance = findWrite(bus, cmd::REG_CTRL6_C,
                                                cmd::MASK_XL_HM_MODE, 0u);
  TEST_ASSERT_LESS_THAN(commandIndex, gyroDown);
  TEST_ASSERT_LESS_THAN(commandIndex, accelHighPerformance);
  const uint64_t commandTime = bus.trace[commandIndex].atMs;
  const uint32_t transfers = bus.transferCalls;
  const PollResult waiting = driver.poll(bus.nowMs, 4);
  TEST_ASSERT_TRUE(waiting.waiting);
  TEST_ASSERT_EQUAL_UINT8(0u, waiting.transactionsUsed);
  TEST_ASSERT_EQUAL_UINT32(transfers, bus.transferCalls);

  bus.nowMs = commandTime + 14u;
  TEST_ASSERT_EQUAL_UINT8(0u, driver.poll(bus.nowMs, 4).transactionsUsed);
  TEST_ASSERT_EQUAL_UINT32(transfers, bus.transferCalls);
  bus.nowMs = commandTime + 15u;
  const PollResult terminal = runToTerminal(driver, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(terminal.state));
  const OperationResult result = take(driver, token);
  TEST_ASSERT_TRUE(result.hardwareStateMayHaveChanged);
  TEST_ASSERT_EQUAL_UINT32(MAX_RESET_TRANSACTIONS, result.transactionLimit);
  for (size_t i = commandIndex + 1u; i < bus.traceCount; ++i) {
    TEST_ASSERT_GREATER_OR_EQUAL_UINT64(commandTime + 15u, bus.trace[i].atMs);
  }
}

void test_boot_has_same_bounded_inaccessible_window() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  bus.commandInaccessibleMs = 15;
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startBoot(timing(bus), token).inProgress());
  size_t commandIndex = bus.traceCount;
  for (uint32_t i = 0; i < 20 && commandIndex == bus.traceCount; ++i) {
    (void)driver.poll(bus.nowMs, 1);
    commandIndex = findWrite(bus, cmd::REG_CTRL3_C, cmd::MASK_BOOT, cmd::MASK_BOOT);
  }
  TEST_ASSERT_LESS_THAN(bus.traceCount, commandIndex);
  const uint64_t commandTime = bus.trace[commandIndex].atMs;
  const uint32_t transfers = bus.transferCalls;
  TEST_ASSERT_EQUAL_UINT8(0u, driver.poll(commandTime + 14u, 8).transactionsUsed);
  TEST_ASSERT_EQUAL_UINT32(transfers, bus.transferCalls);
  bus.nowMs = commandTime + 15u;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(runToTerminal(driver, bus, 2).state));
  (void)take(driver, token);
}

void test_cancel_reset_after_command_marks_configuration_unknown_without_i2c() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startReset(timing(bus), token).inProgress());
  while (findWrite(bus, cmd::REG_CTRL3_C, cmd::MASK_SW_RESET, cmd::MASK_SW_RESET) ==
         bus.traceCount) {
    (void)driver.poll(bus.nowMs, 1);
  }
  const uint32_t transfers = bus.transferCalls;
  TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
  TEST_ASSERT_EQUAL_UINT32(transfers, bus.transferCalls);
  const OperationResult result = take(driver, token);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::CANCELLED),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_TRUE(result.hardwareStateMayHaveChanged);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNKNOWN),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
}

void test_recover_reprobes_and_reapplies_verified_profile_without_retry_policy() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  const uint32_t oldGeneration = configure(driver, bus).configuration.generation;
  bus.resetRegisters();
  bus.clearTrace();
  bus.commandInaccessibleMs = 15;
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startRecover(timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(terminal.state));
  const OperationResult result = take(driver, token);
  TEST_ASSERT_GREATER_THAN_UINT32(oldGeneration, result.configuration.generation);
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_FUNC_CFG_ACCESS, bus.trace[0].startReg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TransferKind::WRITE_READ),
                          static_cast<uint8_t>(bus.trace[0].kind));
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_WHO_AM_I, bus.trace[1].startReg);
  if (bus.nowMs < result.configuration.validAfterUptimeMs) {
    bus.nowMs = result.configuration.validAfterUptimeMs;
  }
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::KNOWN),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
}

void test_reset_boot_and_recover_failures_are_bounded_at_every_transfer_stage() {
  const MaintenanceCase operations[] = {MaintenanceCase::RESET,
                                        MaintenanceCase::BOOT,
                                        MaintenanceCase::RECOVER};
  for (MaintenanceCase operation : operations) {
    FakeBus referenceBus;
    LSM6DS3TR::LSM6DS3TR referenceDriver;
    TEST_ASSERT_TRUE(referenceDriver.bind(makeDriverConfig(referenceBus)).ok());
    (void)configure(referenceDriver, referenceBus);
    referenceBus.clearTrace();
    OperationToken referenceToken;
    TEST_ASSERT_TRUE(startMaintenance(referenceDriver, operation,
                                      timing(referenceBus), referenceToken)
                         .inProgress());
    (void)runToTerminal(referenceDriver, referenceBus, 4);
    const uint32_t stageCount = referenceBus.transferCalls;
    TEST_ASSERT_GREATER_THAN_UINT32(1u, stageCount);
    (void)take(referenceDriver, referenceToken);

    for (uint32_t stage = 1; stage <= stageCount; ++stage) {
      FakeBus bus;
      LSM6DS3TR::LSM6DS3TR driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      (void)configure(driver, bus);
      bus.clearTrace();
      bus.addFault(stage, Status::Error(Err::I2C_TIMEOUT, "maintenance stage",
                                       static_cast<int32_t>(stage)));
      OperationToken token;
      TEST_ASSERT_TRUE(
          startMaintenance(driver, operation, timing(bus), token).inProgress());
      const PollResult terminal = runToTerminal(driver, bus, 3);
      const OperationResult result = take(driver, token);
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                              static_cast<uint8_t>(result.status.code));
      TEST_ASSERT_LESS_OR_EQUAL_UINT32(result.transactionLimit,
                                       result.transactions);
      if (operation == MaintenanceCase::RECOVER && stage <= 2u) {
        TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::FAILED),
                                static_cast<uint8_t>(terminal.state));
        TEST_ASSERT_FALSE(result.hardwareStateMayHaveChanged);
        TEST_ASSERT_EQUAL_UINT8(
            static_cast<uint8_t>(ConfigurationState::KNOWN),
            static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
      } else {
        TEST_ASSERT_EQUAL_UINT8(
            static_cast<uint8_t>(OperationState::INDETERMINATE),
            static_cast<uint8_t>(terminal.state));
        TEST_ASSERT_TRUE(result.hardwareStateMayHaveChanged);
        TEST_ASSERT_EQUAL_UINT8(
            static_cast<uint8_t>(ConfigurationState::UNKNOWN),
            static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
      }
    }
  }
}

void test_reset_boot_and_recover_completion_poll_boundaries_are_exact() {
  const MaintenanceCase operations[] = {MaintenanceCase::RESET,
                                        MaintenanceCase::BOOT,
                                        MaintenanceCase::RECOVER};
  for (MaintenanceCase operation : operations) {
    {
      FakeBus bus;
      LSM6DS3TR::LSM6DS3TR driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      (void)configure(driver, bus);
      bus.clearTrace();
      if (operation == MaintenanceCase::BOOT) {
        bus.bootBitReadsRemaining = 15;
      } else {
        bus.resetBitReadsRemaining = 15;
      }
      OperationToken token;
      TEST_ASSERT_TRUE(
          startMaintenance(driver, operation, timing(bus), token).inProgress());
      const PollResult terminal = runToTerminal(driver, bus, 1);
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                              static_cast<uint8_t>(terminal.state));
      const OperationResult result = take(driver, token);
      const uint32_t limit = operation == MaintenanceCase::RECOVER
                                 ? MAX_RECOVER_TRANSACTIONS
                                 : MAX_RESET_TRANSACTIONS;
      TEST_ASSERT_EQUAL_UINT32(limit, result.transactionLimit);
      TEST_ASSERT_EQUAL_UINT32(limit, result.transactions);
      uint32_t commandReads = 0;
      bool commandWritten = false;
      const uint8_t commandMask = operation == MaintenanceCase::BOOT
                                      ? cmd::MASK_BOOT
                                      : cmd::MASK_SW_RESET;
      for (size_t i = 0; i < bus.traceCount; ++i) {
        if (bus.trace[i].kind == TransferKind::WRITE &&
            bus.trace[i].startReg == cmd::REG_CTRL3_C &&
            (bus.trace[i].firstValue & commandMask) != 0u) {
          commandWritten = true;
          continue;
        }
        if (commandWritten && bus.trace[i].kind == TransferKind::WRITE) {
          break;
        }
        if (commandWritten && bus.trace[i].kind == TransferKind::WRITE_READ &&
            bus.trace[i].startReg == cmd::REG_CTRL3_C) {
          ++commandReads;
        }
      }
      TEST_ASSERT_EQUAL_UINT32(16u, commandReads);
    }

    {
      FakeBus bus;
      LSM6DS3TR::LSM6DS3TR driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      (void)configure(driver, bus);
      bus.clearTrace();
      if (operation == MaintenanceCase::BOOT) {
        bus.holdBootBit = true;
      } else {
        bus.holdResetBit = true;
      }
      OperationToken token;
      TEST_ASSERT_TRUE(
          startMaintenance(driver, operation, timing(bus), token).inProgress());
      const PollResult terminal = runToTerminal(driver, bus, 1);
      const OperationResult result = take(driver, token);
      assertTerminalFailure(terminal, result, Err::TRANSACTION_LIMIT_EXCEEDED,
                            OperationState::INDETERMINATE);
      TEST_ASSERT_TRUE(result.hardwareStateMayHaveChanged);
      TEST_ASSERT_LESS_OR_EQUAL_UINT32(result.transactionLimit,
                                       result.transactions);
      TEST_ASSERT_EQUAL_UINT32(operation == MaintenanceCase::RECOVER ? 21u : 22u,
                               result.transactions);
      uint32_t commandReads = 0;
      bool commandWritten = false;
      const uint8_t commandMask = operation == MaintenanceCase::BOOT
                                      ? cmd::MASK_BOOT
                                      : cmd::MASK_SW_RESET;
      for (size_t i = 0; i < bus.traceCount; ++i) {
        if (bus.trace[i].kind == TransferKind::WRITE &&
            bus.trace[i].startReg == cmd::REG_CTRL3_C &&
            (bus.trace[i].firstValue & commandMask) != 0u) {
          commandWritten = true;
          continue;
        }
        if (commandWritten && bus.trace[i].kind == TransferKind::WRITE) {
          break;
        }
        if (commandWritten && bus.trace[i].kind == TransferKind::WRITE_READ &&
            bus.trace[i].startReg == cmd::REG_CTRL3_C) {
          ++commandReads;
        }
      }
      TEST_ASSERT_EQUAL_UINT32(16u, commandReads);
      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(ConfigurationState::UNKNOWN),
          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
    }
  }
}

void test_reset_and_boot_wait_deadlines_saturate_at_uint64_max() {
  const MaintenanceCase operations[] = {MaintenanceCase::RESET,
                                        MaintenanceCase::BOOT};
  for (MaintenanceCase operation : operations) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    (void)configure(driver, bus);
    bus.clearTrace();
    bus.nowMs = std::numeric_limits<uint64_t>::max() - 10u;
    OperationToken token;
    TEST_ASSERT_TRUE(startMaintenance(
                         driver, operation,
                         OperationTiming{bus.nowMs,
                                         std::numeric_limits<uint64_t>::max()},
                         token)
                         .inProgress());
    for (uint8_t transfer = 0; transfer < 6u; ++transfer) {
      TEST_ASSERT_EQUAL_UINT8(1u, driver.poll(bus.nowMs, 1).transactionsUsed);
    }
    const uint32_t transfers = bus.transferCalls;
    bus.nowMs = std::numeric_limits<uint64_t>::max() - 1u;
    const PollResult waiting = driver.poll(bus.nowMs, 1);
    TEST_ASSERT_TRUE(waiting.waiting);
    TEST_ASSERT_EQUAL_UINT8(0u, waiting.transactionsUsed);
    TEST_ASSERT_EQUAL_UINT32(transfers, bus.transferCalls);
    bus.nowMs = std::numeric_limits<uint64_t>::max();
    const PollResult terminal = driver.poll(bus.nowMs, 1);
    const OperationResult result = take(driver, token);
    assertTerminalFailure(terminal, result, Err::DEADLINE_EXPIRED,
                          OperationState::TIMED_OUT);
    TEST_ASSERT_TRUE(result.hardwareStateMayHaveChanged);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(ConfigurationState::UNKNOWN),
        static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
  }
}

void test_self_test_is_staged_bounded_and_restores_configuration() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  const uint32_t generation = configure(driver, bus).configuration.generation;
  bus.clearTrace();
  SelfTestRequest request;
  request.samples = 5;
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startSelfTest(request, timing(bus), token).inProgress());
  bool observedZeroI2cWait = false;
  PollResult terminal;
  for (uint32_t i = 0; i < 5000; ++i) {
    const uint32_t before = bus.transferCalls;
    terminal = driver.poll(bus.nowMs, 1);
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(1u, terminal.transactionsUsed);
    if (terminal.waiting && terminal.transactionsUsed == 0u) {
      observedZeroI2cWait = true;
    }
    TEST_ASSERT_EQUAL_UINT32(bus.transferCalls - before, terminal.transactionsUsed);
    if (terminal.state != OperationState::ACTIVE) {
      break;
    }
    bus.nowMs++;
  }
  TEST_ASSERT_TRUE(observedZeroI2cWait);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(terminal.state));
  const OperationResult result = take(driver, token);
  TEST_ASSERT_TRUE(result.selfTest.accelPass);
  TEST_ASSERT_TRUE(result.selfTest.gyroPass);
  TEST_ASSERT_TRUE(result.selfTest.primaryStatus.ok());
  TEST_ASSERT_TRUE(result.selfTest.restorationStatus.ok());
  TEST_ASSERT_EQUAL_UINT32(maximumSelfTestTransactions(request.samples),
                           result.transactionLimit);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(result.transactionLimit, result.transactions);
  TEST_ASSERT_GREATER_THAN_UINT32(generation, driver.configGeneration());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::KNOWN),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
}

void test_self_test_wakes_and_restores_a_sleeping_gyro_profile() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  DeviceProfile profile = makeProfile();
  profile.gyroSleepEnabled = true;
  (void)configure(driver, bus, profile);
  bus.clearTrace();

  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startSelfTest(SelfTestRequest{5}, timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(terminal.state));
  const OperationResult result = take(driver, token);
  TEST_ASSERT_TRUE(result.selfTest.accelPass);
  TEST_ASSERT_TRUE(result.selfTest.gyroPass);

  const size_t wakeWrite =
      findWrite(bus, cmd::REG_CTRL4_C, cmd::MASK_SLEEP_G, 0U);
  TEST_ASSERT_LESS_THAN(bus.traceCount, wakeWrite);
  const size_t restoreWrite = findWrite(bus, cmd::REG_CTRL4_C,
                                        cmd::MASK_SLEEP_G,
                                        cmd::MASK_SLEEP_G, wakeWrite + 1U);
  TEST_ASSERT_LESS_THAN(bus.traceCount, restoreWrite);
  DeviceProfile verified;
  TEST_ASSERT_TRUE(driver.getVerifiedProfile(verified, bus.nowMs).ok());
  TEST_ASSERT_TRUE(verified.gyroSleepEnabled);
}

void test_self_test_failures_are_bounded_at_every_transfer_stage() {
  FakeBus referenceBus;
  LSM6DS3TR::LSM6DS3TR referenceDriver;
  TEST_ASSERT_TRUE(referenceDriver.bind(makeDriverConfig(referenceBus)).ok());
  (void)configure(referenceDriver, referenceBus);
  referenceBus.clearTrace();
  OperationToken referenceToken;
  TEST_ASSERT_TRUE(referenceDriver
                       .startSelfTest(SelfTestRequest{5}, timing(referenceBus),
                                      referenceToken)
                       .inProgress());
  (void)runToTerminal(referenceDriver, referenceBus, 4);
  const uint32_t stageCount = referenceBus.transferCalls;
  TEST_ASSERT_GREATER_THAN_UINT32(1u, stageCount);
  (void)take(referenceDriver, referenceToken);

  for (uint32_t stage = 1; stage <= stageCount; ++stage) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    (void)configure(driver, bus);
    bus.clearTrace();
    bus.addFault(stage, Status::Error(Err::I2C_TIMEOUT, "self-test stage",
                                     static_cast<int32_t>(stage)));
    OperationToken token;
    TEST_ASSERT_TRUE(driver
                         .startSelfTest(SelfTestRequest{5}, timing(bus), token)
                         .inProgress());
    const PollResult terminal = runToTerminal(driver, bus, 3);
    const OperationResult result = take(driver, token);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                            static_cast<uint8_t>(result.status.code));
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(result.transactionLimit,
                                     result.transactions);
    if (result.selfTest.restorationStatus.ok()) {
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::FAILED),
                              static_cast<uint8_t>(terminal.state));
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                              static_cast<uint8_t>(
                                  result.selfTest.primaryStatus.code));
      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(ConfigurationState::KNOWN),
          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
    } else {
      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(OperationState::INDETERMINATE),
          static_cast<uint8_t>(terminal.state));
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                              static_cast<uint8_t>(
                                  result.selfTest.restorationStatus.code));
      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(ConfigurationState::UNKNOWN),
          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
    }
  }
}

void test_self_test_preserves_primary_and_restoration_failures() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  bus.addFault(1, Status::Error(Err::I2C_NACK_DATA, "primary", -201));
  bus.addFault(2, Status::Error(Err::I2C_TIMEOUT, "restore", -202));
  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startSelfTest(SelfTestRequest{5}, timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 1);
  const OperationResult result = take(driver, token);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::INDETERMINATE),
                          static_cast<uint8_t>(terminal.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_DATA),
                          static_cast<uint8_t>(result.status.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_DATA),
                          static_cast<uint8_t>(result.selfTest.primaryStatus.code));
  TEST_ASSERT_EQUAL_INT32(-201, result.selfTest.primaryStatus.detail);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::I2C_TIMEOUT),
      static_cast<uint8_t>(result.selfTest.restorationStatus.code));
  TEST_ASSERT_EQUAL_INT32(-202, result.selfTest.restorationStatus.detail);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNKNOWN),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
}

void test_self_test_restoration_deadline_preserves_primary_failure() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  bus.addFault(1, Status::Error(Err::I2C_NACK_DATA, "primary", -301));
  const uint64_t deadline = bus.nowMs + 100U;
  OperationToken token;
  TEST_ASSERT_TRUE(driver
                        .startSelfTest(SelfTestRequest{5},
                                      OperationTiming{bus.nowMs, deadline}, token)
                       .inProgress());
  const PollResult restoring = driver.poll(bus.nowMs, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::ACTIVE),
                          static_cast<uint8_t>(restoring.state));

  bus.nowMs = deadline;
  const PollResult terminal = driver.poll(bus.nowMs, 1);
  const OperationResult result = take(driver, token);
  assertTerminalFailure(terminal, result, Err::DEADLINE_EXPIRED,
                        OperationState::TIMED_OUT);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_DATA),
                          static_cast<uint8_t>(result.selfTest.primaryStatus.code));
  TEST_ASSERT_EQUAL_INT32(-301, result.selfTest.primaryStatus.detail);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::OPERATION_INDETERMINATE),
      static_cast<uint8_t>(result.selfTest.restorationStatus.code));
  TEST_ASSERT_TRUE(result.hardwareStateMayHaveChanged);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNKNOWN),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
}

void test_self_test_negative_stimulus_reports_absolute_deltas() {
  FakeBus bus;
  bus.negativeSelfTestStimulus = true;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startSelfTest(SelfTestRequest{5}, timing(bus), token).inProgress());
  (void)runToTerminal(driver, bus, 1);
  const OperationResult result = take(driver, token);
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_TRUE(result.selfTest.accelPass);
  TEST_ASSERT_TRUE(result.selfTest.gyroPass);
  TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, result.selfTest.accelDeltaG.x);
  TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, result.selfTest.accelDeltaG.y);
  TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, result.selfTest.accelDeltaG.z);
  TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, result.selfTest.gyroDeltaDps.x);
  TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, result.selfTest.gyroDeltaDps.y);
  TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, result.selfTest.gyroDeltaDps.z);
}

void test_self_test_cancel_during_settle_is_bus_silent_and_unknown() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startSelfTest(SelfTestRequest{5}, timing(bus), token).inProgress());
  PollResult poll;
  do {
    poll = driver.poll(bus.nowMs, 1);
  } while (!poll.waiting);
  const uint32_t transfers = bus.transferCalls;
  TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
  TEST_ASSERT_EQUAL_UINT32(transfers, bus.transferCalls);
  const OperationResult result = take(driver, token);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::CANCELLED),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_TRUE(result.hardwareStateMayHaveChanged);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNKNOWN),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
}

void test_self_test_intermittent_not_ready_uses_bounded_bus_silent_cadence() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  bus.notReadyStatusReads = 2;
  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startSelfTest(SelfTestRequest{5}, timing(bus), token).inProgress());

  bool observedNotReadyWait = false;
  for (uint32_t pollCount = 0; pollCount < 2000 && !observedNotReadyWait;
       ++pollCount) {
    const size_t traceBefore = bus.traceCount;
    const PollResult poll = driver.poll(bus.nowMs, 1);
    if (bus.traceCount > traceBefore && poll.waiting &&
        bus.trace[bus.traceCount - 1u].startReg == cmd::REG_STATUS_REG) {
      observedNotReadyWait = true;
      const uint32_t transfers = bus.transferCalls;
      const PollResult sameTime = driver.poll(bus.nowMs, 4);
      TEST_ASSERT_TRUE(sameTime.waiting);
      TEST_ASSERT_EQUAL_UINT8(0u, sameTime.transactionsUsed);
      TEST_ASSERT_EQUAL_UINT32(transfers, bus.transferCalls);
    }
    bus.nowMs++;
  }
  TEST_ASSERT_TRUE(observedNotReadyWait);
  const PollResult terminal = runToTerminal(driver, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(terminal.state));
  const OperationResult result = take(driver, token);
  TEST_ASSERT_TRUE(result.selfTest.accelPass);
  TEST_ASSERT_TRUE(result.selfTest.gyroPass);
  TEST_ASSERT_EQUAL_UINT32(maximumSelfTestTransactions(5),
                           result.transactionLimit);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(result.transactionLimit,
                                   result.transactions);

  uint64_t statusTimes[3] = {};
  size_t statusCount = 0;
  for (size_t i = 0; i < bus.traceCount && statusCount < 3u; ++i) {
    if (bus.trace[i].startReg == cmd::REG_STATUS_REG) {
      statusTimes[statusCount++] = bus.trace[i].atMs;
    }
  }
  TEST_ASSERT_EQUAL_UINT32(3u, statusCount);
  TEST_ASSERT_GREATER_OR_EQUAL_UINT64(statusTimes[0] + 3u, statusTimes[1]);
  TEST_ASSERT_GREATER_OR_EQUAL_UINT64(statusTimes[1] + 3u, statusTimes[2]);
}

void test_self_test_three_not_ready_checks_restore_known_configuration() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  bus.notReadyStatusReads = 3;
  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startSelfTest(SelfTestRequest{5}, timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 1);
  const OperationResult result = take(driver, token);
  assertTerminalFailure(terminal, result, Err::DATA_NOT_READY,
                        OperationState::FAILED);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::DATA_NOT_READY),
                          static_cast<uint8_t>(
                              result.selfTest.primaryStatus.code));
  TEST_ASSERT_TRUE(result.selfTest.restorationStatus.ok());
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(result.transactionLimit,
                                   result.transactions);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::KNOWN),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
}

void test_self_test_invalid_request_and_timeout_are_zero_retry() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  OperationToken token{55};
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_PARAM),
      static_cast<uint8_t>(
          driver.startSelfTest(SelfTestRequest{0}, timing(bus), token).code));
  TEST_ASSERT_FALSE(token.valid());
  TEST_ASSERT_EQUAL_UINT32(0u, bus.transferCalls);

  TEST_ASSERT_TRUE(driver
                        .startSelfTest(SelfTestRequest{5},
                                      OperationTiming{bus.nowMs, bus.nowMs + 1u}, token)
                       .inProgress());
  (void)driver.poll(bus.nowMs, 1);
  const uint32_t transfers = bus.transferCalls;
  const PollResult terminal = driver.poll(bus.nowMs + 1u, 8);
  TEST_ASSERT_EQUAL_UINT32(transfers, bus.transferCalls);
  const OperationResult result = take(driver, token);
  assertTerminalFailure(terminal, result, Err::DEADLINE_EXPIRED,
                        OperationState::TIMED_OUT);
}

void test_gyro_calibration_is_staged_fixed_count_and_owner_timed() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  CalibrationRequest request;
  request.kind = CalibrationKind::GYROSCOPE_BIAS;
  request.samples = 3;
  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startCalibration(request, timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(terminal.state));
  const OperationResult result = take(driver, token);
  TEST_ASSERT_EQUAL_UINT16(3u, result.calibration.samples);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CalibrationKind::GYROSCOPE_BIAS),
                          static_cast<uint8_t>(result.calibration.kind));
  TEST_ASSERT_EQUAL_UINT32(maximumCalibrationTransactions(request.samples),
                           result.transactionLimit);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(result.transactionLimit, result.transactions);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 8.75f, result.calibration.bias.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -4.375f, result.calibration.bias.y);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, result.calibration.bias.z);
}

void test_gyro_calibration_rejects_sleeping_sensor_without_i2c() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  DeviceProfile profile = makeProfile();
  profile.gyroSleepEnabled = true;
  (void)configure(driver, bus, profile);
  bus.clearTrace();

  CalibrationRequest request;
  request.kind = CalibrationKind::GYROSCOPE_BIAS;
  request.samples = 1;
  OperationToken token{77};
  const Status status = driver.startCalibration(request, timing(bus), token);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(status.code));
  TEST_ASSERT_FALSE(token.valid());
  TEST_ASSERT_EQUAL_UINT32(0U, bus.transferCalls);
}

void test_accel_calibration_uses_explicit_fixture_vector_and_rejects_orientation() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  CalibrationRequest request;
  request.kind = CalibrationKind::ACCELEROMETER_BIAS;
  request.samples = 3;
  request.expectedAccelerationG = Axes{0.0f, 0.0f, 1.0f};
  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startCalibration(request, timing(bus), token).inProgress());
  (void)runToTerminal(driver, bus, 1);
  OperationResult result = take(driver, token);
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, result.calibration.bias.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, result.calibration.bias.y);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.000576f, result.calibration.bias.z);

  bus.setRawSample(6400, 1000, -500, 0, 16384, 0, 0);
  bus.clearTrace();
  TEST_ASSERT_TRUE(
      driver.startCalibration(request, timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 1);
  result = take(driver, token);
  assertTerminalFailure(terminal, result, Err::CALIBRATION_ORIENTATION,
                        OperationState::FAILED);
}

void test_accel_calibration_int16_span_cannot_overflow_peak_to_peak() {
  FakeBus bus;
  bus.alternateAccelInt16Extremes = true;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  CalibrationRequest request;
  request.kind = CalibrationKind::ACCELEROMETER_BIAS;
  request.samples = 2;
  request.expectedAccelerationG = Axes{0.0f, 0.0f, 1.0f};
  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startCalibration(request, timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 1);
  const OperationResult result = take(driver, token);
  assertTerminalFailure(terminal, result, Err::CALIBRATION_UNSTABLE,
                        OperationState::FAILED);
  TEST_ASSERT_GREATER_THAN_FLOAT(request.limits.accelMaxPeakToPeakG,
                                 result.calibration.peakToPeak.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.997635f,
                           result.calibration.peakToPeak.x);
}

void test_calibration_not_ready_retries_and_post_sample_waits_are_bus_silent() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  bus.notReadyStatusReads = 2;
  CalibrationRequest request;
  request.kind = CalibrationKind::GYROSCOPE_BIAS;
  request.samples = 2;
  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startCalibration(request, timing(bus), token).inProgress());

  PollResult first = driver.poll(bus.nowMs, 1);
  TEST_ASSERT_TRUE(first.waiting);
  TEST_ASSERT_EQUAL_UINT8(1u, first.transactionsUsed);
  const uint32_t transfers = bus.transferCalls;
  const PollResult sameTime = driver.poll(bus.nowMs, 4);
  TEST_ASSERT_TRUE(sameTime.waiting);
  TEST_ASSERT_EQUAL_UINT8(0u, sameTime.transactionsUsed);
  TEST_ASSERT_EQUAL_UINT32(transfers, bus.transferCalls);
  bus.nowMs++;

  const PollResult terminal = runToTerminal(driver, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(terminal.state));
  const OperationResult result = take(driver, token);
  TEST_ASSERT_EQUAL_UINT16(2u, result.calibration.samples);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(result.transactionLimit,
                                   result.transactions);

  uint64_t statusTimes[4] = {};
  size_t statusCount = 0;
  for (size_t i = 0; i < bus.traceCount && statusCount < 4u; ++i) {
    if (bus.trace[i].startReg == cmd::REG_STATUS_REG) {
      statusTimes[statusCount++] = bus.trace[i].atMs;
    }
  }
  TEST_ASSERT_EQUAL_UINT32(4u, statusCount);
  const uint64_t periodMs = (odrPeriodUs(Odr::HZ_104) + 999u) / 1000u;
  TEST_ASSERT_GREATER_OR_EQUAL_UINT64(statusTimes[0] + periodMs,
                                      statusTimes[1]);
  TEST_ASSERT_GREATER_OR_EQUAL_UINT64(statusTimes[1] + periodMs,
                                      statusTimes[2]);
  TEST_ASSERT_GREATER_OR_EQUAL_UINT64(statusTimes[2] + periodMs,
                                      statusTimes[3]);
}

void test_calibration_three_not_ready_checks_fail_without_mutating_configuration() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  bus.notReadyStatusReads = 3;
  CalibrationRequest request;
  request.kind = CalibrationKind::GYROSCOPE_BIAS;
  request.samples = 1;
  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startCalibration(request, timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 1);
  const OperationResult result = take(driver, token);
  assertTerminalFailure(terminal, result, Err::DATA_NOT_READY,
                        OperationState::FAILED);
  TEST_ASSERT_FALSE(result.hardwareStateMayHaveChanged);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::KNOWN),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
}

void test_all_other_hardware_entry_points_are_excluded_by_active_job() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  OperationToken active;
  TEST_ASSERT_TRUE(driver.startProbe(timing(bus), active).inProgress());
  OperationToken rejected{9};
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(driver.startConfigure(makeProfile(), timing(bus), rejected).code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(driver.startSample(SampleRequest{}, timing(bus), rejected).code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(driver.startReset(timing(bus), rejected).code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(driver.startBoot(timing(bus), rejected).code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(driver.startRecover(timing(bus), rejected).code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(driver.startReconcile(timing(bus), rejected).code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(driver.startPowerDown(timing(bus), rejected).code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(driver.startSelfTest(SelfTestRequest{}, timing(bus), rejected).code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(driver.startCalibration(CalibrationRequest{}, timing(bus), rejected).code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(driver.startFifoPurge(FifoPurgeRequest{}, timing(bus), rejected).code));
  uint8_t value = 0;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(driver.diagnosticReadRegister(cmd::REG_WHO_AM_I, value, bus.nowMs).code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY),
                          static_cast<uint8_t>(driver.diagnosticWriteRegister(cmd::REG_CTRL1_XL, 0, bus.nowMs).code));
  TEST_ASSERT_FALSE(rejected.valid());
  TEST_ASSERT_EQUAL_UINT32(0u, bus.transferCalls);
}

void test_diagnostic_bounds_and_raw_write_invalidation_are_explicit() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  uint8_t data[33] = {};
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(driver.diagnosticReadBlock(0x10, data, sizeof(data), bus.nowMs).code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(driver.diagnosticReadBlock(0xFF, data, 2, bus.nowMs).code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(driver.diagnosticWriteRegister(cmd::REG_WHO_AM_I, 0, bus.nowMs).code));
  const struct {
    uint8_t reg;
    uint8_t value;
  } rejectedWrites[] = {
      {cmd::REG_FUNC_CFG_ACCESS,
       static_cast<uint8_t>(cmd::MASK_FUNC_CFG_EN | cmd::MASK_FUNC_CFG_EN_B)},
      {cmd::REG_SENSOR_SYNC_TIME_FRAME, 0x10u},
      {cmd::REG_SENSOR_SYNC_RES_RATIO, 0x04u},
      {cmd::REG_FIFO_CTRL2, 0x30u},
      {cmd::REG_FIFO_CTRL5, 0x02u},
      {cmd::REG_CTRL1_XL, 0xC0u},
      {cmd::REG_CTRL2_G,
       static_cast<uint8_t>(cmd::MASK_FS_125 | cmd::MASK_FS_G)},
      {cmd::REG_CTRL3_C, 0u},
      {cmd::REG_CTRL5_C, cmd::MASK_ST_XL},
      {cmd::REG_CTRL5_C, 0x08u},
      {cmd::REG_CTRL6_C, 0x20u},
      {cmd::REG_CTRL7_G, 0x01u},
      {cmd::REG_MASTER_CONFIG, 0x20u},
      {cmd::REG_WAKE_UP_THS, 0x40u},
      {cmd::REG_X_OFS_USR, 0x80u},
  };
  for (const auto& rejected : rejectedWrites) {
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::INVALID_PARAM),
        static_cast<uint8_t>(
            driver.diagnosticWriteRegister(rejected.reg, rejected.value,
                                           bus.nowMs)
                .code));
  }
  TEST_ASSERT_EQUAL_UINT32(0u, bus.transferCalls);

  const uint32_t successesBefore = driver.diagnostics(bus.nowMs).transportSuccesses;
  TEST_ASSERT_TRUE(driver.diagnosticWriteRegister(cmd::REG_CTRL1_XL, 0, bus.nowMs).ok());
  TEST_ASSERT_TRUE(
      driver.diagnosticWriteRegister(cmd::REG_MASTER_CONFIG, 0x08u,
                                     bus.nowMs).ok());
  TEST_ASSERT_TRUE(
      driver.diagnosticWriteRegister(cmd::REG_CTRL5_C, cmd::MASK_ST_G,
                                     bus.nowMs).ok());
  TEST_ASSERT_TRUE(driver.diagnosticWriteRegister(
      cmd::REG_CTRL7_G, cmd::MASK_ROUNDING_STATUS, bus.nowMs).ok());
  TEST_ASSERT_EQUAL_UINT32(4u, bus.transferCalls);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNKNOWN),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
  const DriverDiagnostics diagnostics = driver.diagnostics(bus.nowMs);
  TEST_ASSERT_EQUAL_UINT32(successesBefore + 4u, diagnostics.transportSuccesses);
  TEST_ASSERT_EQUAL_UINT64(0u, diagnostics.lastTransportErrorUptimeMs);
}

void test_diagnostic_sync_writes_invalidate_and_reconcile_exact_addresses() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_PARAM),
      static_cast<uint8_t>(driver.diagnosticWriteRegister(0x02U, 0x11U,
                                                           bus.nowMs)
                               .code));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_PARAM),
      static_cast<uint8_t>(driver.diagnosticWriteRegister(0x03U, 0x22U,
                                                           bus.nowMs)
                               .code));
  TEST_ASSERT_EQUAL_UINT32(0U, bus.transferCalls);

  TEST_ASSERT_TRUE(driver.diagnosticWriteRegister(
      cmd::REG_SENSOR_SYNC_TIME_FRAME, 0x03U, bus.nowMs).ok());
  TEST_ASSERT_TRUE(driver.diagnosticWriteRegister(
      cmd::REG_SENSOR_SYNC_RES_RATIO, 0x02U, bus.nowMs).ok());
  TEST_ASSERT_TRUE(driver.diagnosticWriteRegister(
      cmd::REG_DRDY_PULSE_CFG_G, 0x80U, bus.nowMs).ok());
  TEST_ASSERT_EQUAL_UINT32(3U, bus.transferCalls);
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_SENSOR_SYNC_TIME_FRAME,
                         bus.trace[0].startReg);
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_SENSOR_SYNC_RES_RATIO,
                         bus.trace[1].startReg);
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_DRDY_PULSE_CFG_G,
                         bus.trace[2].startReg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNKNOWN),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));

  OperationToken token;
  TEST_ASSERT_TRUE(driver.startReconcile(timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus);
  const OperationResult result = take(driver, token);
  assertTerminalFailure(terminal, result, Err::CONFIGURATION_MISMATCH,
                        OperationState::FAILED);
  TEST_ASSERT_EQUAL_HEX8(cmd::REG_SENSOR_SYNC_TIME_FRAME,
                         result.configuration.mismatchRegister);

  (void)configure(driver, bus);
  TEST_ASSERT_EQUAL_HEX8(0U, bus.regs[cmd::REG_SENSOR_SYNC_TIME_FRAME]);
  TEST_ASSERT_EQUAL_HEX8(0U, bus.regs[cmd::REG_SENSOR_SYNC_RES_RATIO]);
  TEST_ASSERT_EQUAL_HEX8(0U, bus.regs[cmd::REG_DRDY_PULSE_CFG_G]);
}

void test_ambiguous_diagnostic_write_invalidates_without_hidden_refresh() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  bus.addFault(1, Status::Error(Err::I2C_TIMEOUT, "ambiguous diagnostic", -100), true);
  const Status status =
      driver.diagnosticWriteRegister(cmd::REG_CTRL1_XL, 0, bus.nowMs);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(status.code));
  TEST_ASSERT_EQUAL_UINT32(1u, bus.transferCalls);
  TEST_ASSERT_TRUE(bus.trace[0].effectApplied);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNKNOWN),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
}

void test_power_down_is_explicit_fallible_and_partial_failure_is_indeterminate() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  bus.addFault(3, Status::Error(Err::I2C_TIMEOUT, "accel power-down ambiguous", -101),
               true);
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startPowerDown(timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 2);
  const OperationResult result = take(driver, token);
  assertTerminalFailure(terminal, result, Err::I2C_TIMEOUT,
                        OperationState::INDETERMINATE);
  TEST_ASSERT_EQUAL_UINT32(3u, bus.transferCalls);
  TEST_ASSERT_TRUE(result.hardwareStateMayHaveChanged);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNKNOWN),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
}

void test_power_down_is_available_from_unconfigured_and_unknown_states() {
  FakeBus unconfiguredBus;
  unconfiguredBus.regs[cmd::REG_CTRL1_XL] = 0xFF;
  unconfiguredBus.regs[cmd::REG_CTRL2_G] = 0xFF;
  LSM6DS3TR::LSM6DS3TR unconfiguredDriver;
  TEST_ASSERT_TRUE(
      unconfiguredDriver.bind(makeDriverConfig(unconfiguredBus)).ok());
  OperationToken token;
  TEST_ASSERT_TRUE(unconfiguredDriver
                       .startPowerDown(timing(unconfiguredBus), token)
                       .inProgress());
  (void)runToTerminal(unconfiguredDriver, unconfiguredBus, 1);
  OperationResult result = take(unconfiguredDriver, token);
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_EQUAL_HEX8(0u, unconfiguredBus.regs[cmd::REG_CTRL1_XL]);
  TEST_ASSERT_EQUAL_HEX8(0u, unconfiguredBus.regs[cmd::REG_CTRL2_G]);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(ConfigurationState::UNCONFIGURED),
      static_cast<uint8_t>(
          unconfiguredDriver.configurationState(unconfiguredBus.nowMs)));

  FakeBus unknownBus;
  LSM6DS3TR::LSM6DS3TR unknownDriver;
  TEST_ASSERT_TRUE(unknownDriver.bind(makeDriverConfig(unknownBus)).ok());
  (void)configure(unknownDriver, unknownBus);
  TEST_ASSERT_TRUE(unknownDriver
                       .diagnosticWriteRegister(cmd::REG_CTRL1_XL, 0x40,
                                                unknownBus.nowMs)
                       .ok());
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(ConfigurationState::UNKNOWN),
      static_cast<uint8_t>(unknownDriver.configurationState(unknownBus.nowMs)));
  unknownBus.clearTrace();
  TEST_ASSERT_TRUE(
      unknownDriver.startPowerDown(timing(unknownBus), token).inProgress());
  (void)runToTerminal(unknownDriver, unknownBus, 1);
  result = take(unknownDriver, token);
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_EQUAL_HEX8(0u, unknownBus.regs[cmd::REG_CTRL1_XL]);
  TEST_ASSERT_EQUAL_HEX8(0u, unknownBus.regs[cmd::REG_CTRL2_G]);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(ConfigurationState::UNCONFIGURED),
      static_cast<uint8_t>(unknownDriver.configurationState(unknownBus.nowMs)));
}

void test_successful_power_down_leaves_no_false_known_configuration() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startPowerDown(timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(terminal.state));
  const OperationResult result = take(driver, token);
  TEST_ASSERT_EQUAL_UINT32(MAX_POWER_DOWN_TRANSACTIONS, result.transactionLimit);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNCONFIGURED),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
}

void test_fifo_purge_is_bounded_reports_loss_and_honors_budget() {
  FakeBus bus;
  setFifoPurgePrerequisites(bus);
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  bus.setFifo(5, 7, 0x1200);
  FifoPurgeRequest request;
  request.maxWords = 3;
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startFifoPurge(request, timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(terminal.state));
  const OperationResult result = take(driver, token);
  TEST_ASSERT_EQUAL_UINT16(5u, result.fifoPurge.initialUnreadWords);
  TEST_ASSERT_EQUAL_UINT16(7u, result.fifoPurge.initialPattern);
  TEST_ASSERT_EQUAL_UINT16(3u, result.fifoPurge.wordsDiscarded);
  TEST_ASSERT_EQUAL_UINT16(2u, result.fifoPurge.finalUnreadWords);
  TEST_ASSERT_TRUE(result.fifoPurge.truncated);
  TEST_ASSERT_TRUE(result.hardwareStateMayHaveChanged);
  TEST_ASSERT_EQUAL_UINT32(maximumFifoPurgeTransactions(request.maxWords),
                           result.transactionLimit);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(8u, bus.transferCalls);
}

void test_partial_fifo_purge_cancellation_preserves_discard_count() {
  FakeBus bus;
  setFifoPurgePrerequisites(bus);
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  bus.setFifo(5, 2, 0x2200);
  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startFifoPurge(FifoPurgeRequest{5}, timing(bus), token).inProgress());
  const PollResult partial = driver.poll(bus.nowMs, 5);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::ACTIVE),
                          static_cast<uint8_t>(partial.state));
  TEST_ASSERT_EQUAL_UINT8(5u, partial.transactionsUsed);
  TEST_ASSERT_EQUAL_UINT16(4u, bus.fifoUnreadWords);
  const uint32_t transfers = bus.transferCalls;
  TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
  TEST_ASSERT_EQUAL_UINT32(transfers, bus.transferCalls);
  const OperationResult result = take(driver, token);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::CANCELLED),
                          static_cast<uint8_t>(result.state));
  TEST_ASSERT_EQUAL_UINT16(5u, result.fifoPurge.initialUnreadWords);
  TEST_ASSERT_EQUAL_UINT16(1u, result.fifoPurge.wordsDiscarded);
  TEST_ASSERT_TRUE(result.hardwareStateMayHaveChanged);
}

void test_fifo_concurrent_arrival_is_reported_as_truncated() {
  FakeBus bus;
  setFifoPurgePrerequisites(bus);
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  bus.setFifo(2, 0, 0x3300);
  bus.fifoConcurrentArrivalWords = 1;
  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startFifoPurge(FifoPurgeRequest{2}, timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(terminal.state));
  const OperationResult result = take(driver, token);
  TEST_ASSERT_EQUAL_UINT16(2u, result.fifoPurge.wordsDiscarded);
  TEST_ASSERT_EQUAL_UINT16(1u, result.fifoPurge.finalUnreadWords);
  TEST_ASSERT_TRUE(result.fifoPurge.truncated);
}

void test_fifo_overrun_is_data_loss_not_a_valid_batch() {
  FakeBus bus;
  setFifoPurgePrerequisites(bus);
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  bus.setFifo(2048, 3, 0x1000, true);
  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startFifoPurge(FifoPurgeRequest{2}, timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 2);
  const OperationResult result = take(driver, token);
  assertTerminalFailure(terminal, result, Err::FIFO_OVERRUN, OperationState::FAILED);
  TEST_ASSERT_TRUE(result.fifoPurge.overrunObserved);
  TEST_ASSERT_EQUAL_UINT16(2u, result.fifoPurge.wordsDiscarded);
  TEST_ASSERT_EQUAL_UINT16(2048u, result.fifoPurge.initialUnreadWords);
  TEST_ASSERT_TRUE(result.fifoPurge.truncated);
  TEST_ASSERT_EQUAL_UINT32(7u, bus.transferCalls);

  bus.clearTrace();
  bus.setFifo(7, 1, 0x2000, true);
  TEST_ASSERT_TRUE(
      driver.startFifoPurge(FifoPurgeRequest{2}, timing(bus), token).inProgress());
  (void)runToTerminal(driver, bus, 2);
  const OperationResult latchedResult = take(driver, token);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::FIFO_OVERRUN),
                          static_cast<uint8_t>(latchedResult.status.code));
  TEST_ASSERT_EQUAL_UINT16(7u, latchedResult.fifoPurge.initialUnreadWords);
  TEST_ASSERT_EQUAL_UINT16(5u, latchedResult.fifoPurge.finalUnreadWords);
  TEST_ASSERT_EQUAL_UINT16(2u, latchedResult.fifoPurge.wordsDiscarded);
}

void test_settle_bounds_match_supported_filter_and_power_profiles() {
  DeviceProfile profile = makeProfile();
  TEST_ASSERT_EQUAL_UINT64(134624u, requiredSettleUs(profile));

  profile.accelFilter.lpf2Enabled = true;
  TEST_ASSERT_EQUAL_UINT64(384640u, requiredSettleUs(profile));

  profile = makeProfile();
  profile.accelOdr = Odr::POWER_DOWN;
  profile.gyroOdr = Odr::HZ_12_5;
  TEST_ASSERT_EQUAL_UINT64(230000u, requiredSettleUs(profile));

  profile.gyroOdr = Odr::HZ_104;
  profile.gyroFilter.lpf1Enabled = true;
  TEST_ASSERT_EQUAL_UINT64(108464u, requiredSettleUs(profile));

  profile.gyroOdr = Odr::HZ_6660;
  profile.gyroFilter.lpf1Enabled = false;
  TEST_ASSERT_EQUAL_UINT64(151540u, requiredSettleUs(profile));

  profile.gyroSleepEnabled = true;
  TEST_ASSERT_EQUAL_UINT64(70000u, requiredSettleUs(profile));

  profile = makeProfile();
  profile.accelOdr = Odr::HZ_1_6;
  profile.accelPowerMode = AccelPowerMode::LOW_POWER_NORMAL;
  profile.gyroOdr = Odr::POWER_DOWN;
  TEST_ASSERT_EQUAL_UINT64(8750000u, requiredSettleUs(profile));
}

void test_profile_rejections_are_complete_and_bus_silent() {
  auto rejected = [](const DeviceProfile& profile) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    OperationToken token{77};
    const Status status = driver.startConfigure(profile, timing(bus), token);
    TEST_ASSERT_FALSE(status.ok());
    TEST_ASSERT_FALSE(status.inProgress());
    TEST_ASSERT_FALSE(token.valid());
    TEST_ASSERT_EQUAL_UINT32(0u, bus.transferCalls);
  };

  DeviceProfile profile = makeProfile();
  profile.gyroPowerMode = GyroPowerMode::LOW_POWER_NORMAL;
  profile.gyroFilter.lpf1Enabled = true;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::UNSUPPORTED_PROFILE),
                          static_cast<uint8_t>(validateProfile(profile).code));
  rejected(profile);

  profile = makeProfile();
  profile.gyroPowerMode = GyroPowerMode::LOW_POWER_NORMAL;
  profile.gyroFilter.highPassEnabled = true;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::UNSUPPORTED_PROFILE),
                          static_cast<uint8_t>(validateProfile(profile).code));
  rejected(profile);

  profile = makeProfile();
  profile.accelFilter.highPassSlopeEnabled = true;
  rejected(profile);
  profile = makeProfile();
  profile.accelFilter.lowPassOn6d = true;
  rejected(profile);

  for (uint8_t axis = 0; axis < 3u; ++axis) {
    profile = makeProfile();
    if (axis == 0u) profile.accelUserOffset.x = INT8_MIN;
    if (axis == 1u) profile.accelUserOffset.y = INT8_MIN;
    if (axis == 2u) profile.accelUserOffset.z = INT8_MIN;
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                            static_cast<uint8_t>(validateProfile(profile).code));
    rejected(profile);
  }

  profile = makeProfile();
  profile.accelPowerMode = AccelPowerMode::LOW_POWER_NORMAL;
  profile.accelOdr = Odr::HZ_416;
  rejected(profile);
  profile = makeProfile();
  profile.gyroPowerMode = GyroPowerMode::LOW_POWER_NORMAL;
  profile.gyroOdr = Odr::HZ_416;
  rejected(profile);

  profile = makeProfile();
  profile.accelPowerMode = static_cast<AccelPowerMode>(0x7Fu);
  rejected(profile);
  profile = makeProfile();
  profile.gyroPowerMode = static_cast<GyroPowerMode>(0x7Fu);
  rejected(profile);
  profile = makeProfile();
  profile.gyroFilter.highPassMode = static_cast<GyroHpfMode>(0x7Fu);
  rejected(profile);
  profile = makeProfile();
  profile.accelOffsetWeight = static_cast<AccelOffsetWeight>(0x7Fu);
  rejected(profile);
}

void test_nondefault_profile_encodes_and_reads_back_every_supported_field() {
  const GyroHpfMode modes[] = {GyroHpfMode::HZ_0_016, GyroHpfMode::HZ_0_065,
                               GyroHpfMode::HZ_0_260, GyroHpfMode::HZ_1_040};
  for (uint8_t mode = 0; mode < 4u; ++mode) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    DeviceProfile profile = makeProfile();
    profile.accelOdr = Odr::HZ_833;
    profile.accelFullScale = AccelFs::G_16;
    profile.gyroOdr = Odr::HZ_416;
    profile.gyroFullScale = GyroFs::DPS_125;
    profile.accelFilter.lpf2Enabled = true;
    profile.gyroFilter.lpf1Enabled = true;
    profile.gyroFilter.highPassEnabled = false;
    profile.gyroFilter.highPassMode = modes[mode];
    profile.gyroSleepEnabled = true;
    profile.accelOffsetWeight = AccelOffsetWeight::MG_16;
    profile.accelUserOffset = AccelUserOffset{11, -22, 33};
    (void)configure(driver, bus, profile);

    TEST_ASSERT_EQUAL_HEX8(0x74u, bus.regs[cmd::REG_CTRL1_XL]);
    TEST_ASSERT_EQUAL_HEX8(0x62u, bus.regs[cmd::REG_CTRL2_G]);
    TEST_ASSERT_EQUAL_HEX8(0x44u, bus.regs[cmd::REG_CTRL3_C]);
    TEST_ASSERT_EQUAL_HEX8(0x42u, bus.regs[cmd::REG_CTRL4_C]);
    TEST_ASSERT_EQUAL_HEX8(0x08u, bus.regs[cmd::REG_CTRL6_C]);
    TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(mode << 4u),
                           bus.regs[cmd::REG_CTRL7_G]);
    TEST_ASSERT_EQUAL_HEX8(0x80u, bus.regs[cmd::REG_CTRL8_XL]);
    TEST_ASSERT_EQUAL_HEX8(11u, bus.regs[cmd::REG_X_OFS_USR]);
    TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(-22),
                           bus.regs[cmd::REG_Y_OFS_USR]);
    TEST_ASSERT_EQUAL_HEX8(33u, bus.regs[cmd::REG_Z_OFS_USR]);
    DeviceProfile verified;
    TEST_ASSERT_TRUE(driver.getVerifiedProfile(verified, bus.nowMs).ok());
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(modes[mode]),
                            static_cast<uint8_t>(verified.gyroFilter.highPassMode));
    TEST_ASSERT_TRUE(verified.gyroSleepEnabled);
  }
}

void test_reconcile_known_profile_preserves_generation_and_settle_evidence() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  const OperationResult configured = configure(driver, bus);
  const uint32_t generation = driver.configGeneration();
  const uint64_t validAfter = driver.validAfterUptimeMs();
  bus.clearTrace();
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startReconcile(timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus, 4);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(terminal.state));
  const OperationResult result = take(driver, token);
  TEST_ASSERT_EQUAL_UINT32(generation, result.configuration.generation);
  TEST_ASSERT_EQUAL_UINT32(configured.configuration.generation,
                           driver.configGeneration());
  TEST_ASSERT_EQUAL_UINT64(validAfter, result.configuration.validAfterUptimeMs);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::KNOWN),
                          static_cast<uint8_t>(result.configuration.state));
  TEST_ASSERT_EQUAL_UINT32(MAX_RECONCILE_TRANSACTIONS, result.transactions);
}

void test_sleeping_gyro_rejects_rate_but_allows_temperature() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  DeviceProfile profile = makeProfile();
  profile.gyroSleepEnabled = true;
  (void)configure(driver, bus, profile);
  bus.clearTrace();

  SampleRequest rate;
  rate.quantityMask = SAMPLE_ANGULAR_RATE;
  OperationToken token{77};
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_PARAM),
      static_cast<uint8_t>(driver.startSample(rate, timing(bus), token).code));
  TEST_ASSERT_FALSE(token.valid());
  TEST_ASSERT_EQUAL_UINT32(0u, bus.transferCalls);

  SampleRequest temperature;
  temperature.quantityMask = SAMPLE_TEMPERATURE;
  TEST_ASSERT_TRUE(driver.startSample(temperature, timing(bus), token).inProgress());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(runToTerminal(driver, bus).state));
  const OperationResult result = take(driver, token);
  TEST_ASSERT_EQUAL_UINT8(SAMPLE_TEMPERATURE, result.sample.validMask);
  TEST_ASSERT_EQUAL_UINT8(SAMPLE_TEMPERATURE, result.sample.freshMask);
}

void test_sample_readiness_cadence_tracks_slowest_requested_source() {
  struct Case {
    Odr accelOdr;
    uint8_t quantityMask;
    uint64_t expectedDelayMs;
  };
  const Case cases[] = {{Odr::HZ_12_5, SAMPLE_ACCELERATION, 80u},
                        {Odr::HZ_1_6, SAMPLE_ACCELERATION, 625u},
                        {Odr::HZ_1_6, SAMPLE_TEMPERATURE, 20u}};
  for (const Case& current : cases) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    DeviceProfile profile = makeProfile();
    profile.accelOdr = current.accelOdr;
    profile.accelPowerMode = AccelPowerMode::LOW_POWER_NORMAL;
    profile.gyroOdr = Odr::POWER_DOWN;
    (void)configure(driver, bus, profile);
    bus.clearTrace();
    bus.notReadyStatusReads = 1;
    SampleRequest request;
    request.quantityMask = current.quantityMask;
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startSample(request, timing(bus), token).inProgress());
    const uint64_t firstAt = bus.nowMs;
    const PollResult first = driver.poll(firstAt, 8);
    TEST_ASSERT_TRUE(first.waiting);
    TEST_ASSERT_EQUAL_UINT8(1u, first.transactionsUsed);
    const uint32_t transfers = bus.transferCalls;
    TEST_ASSERT_EQUAL_UINT8(
        0u, driver.poll(firstAt + current.expectedDelayMs - 1u, 8).transactionsUsed);
    TEST_ASSERT_EQUAL_UINT32(transfers, bus.transferCalls);
    bus.nowMs = firstAt + current.expectedDelayMs;
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                            static_cast<uint8_t>(runToTerminal(driver, bus, 2).state));
    (void)take(driver, token);
    TEST_ASSERT_EQUAL_UINT64(firstAt + current.expectedDelayMs,
                             bus.trace[1].atMs);
  }
}

void test_conversion_preserves_quality_and_rejects_malformed_masks_atomically() {
  RawSampleResult raw;
  raw.validMask = SAMPLE_ACCELERATION;
  raw.freshMask = SAMPLE_ACCELERATION;
  raw.quality = SampleQuality::READY_CHECKED;
  ConvertedSample out;
  TEST_ASSERT_TRUE(convertSample(raw, out).ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(SampleQuality::READY_CHECKED),
                          static_cast<uint8_t>(out.quality));

  ConvertedSample sentinel;
  sentinel.validMask = 0x5Au;
  sentinel.freshMask = 0xA5u;
  sentinel.sequence = 0x12345678u;
  sentinel.quality = SampleQuality::SETTLING;
  const ConvertedSample original = sentinel;

  raw.validMask = 0u;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(convertSample(raw, sentinel).code));
  TEST_ASSERT_EQUAL_UINT8(original.validMask, sentinel.validMask);
  TEST_ASSERT_EQUAL_UINT64(original.sequence, sentinel.sequence);

  raw.validMask = static_cast<uint8_t>(SAMPLE_ACCELERATION | 0x80u);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(convertSample(raw, sentinel).code));
  raw.validMask = SAMPLE_ACCELERATION;
  raw.freshMask = SAMPLE_ANGULAR_RATE;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(convertSample(raw, sentinel).code));
  raw.freshMask = SAMPLE_ACCELERATION;
  raw.quality = static_cast<SampleQuality>(0x7Fu);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(convertSample(raw, sentinel).code));
  TEST_ASSERT_EQUAL_UINT8(original.validMask, sentinel.validMask);
  TEST_ASSERT_EQUAL_UINT8(original.freshMask, sentinel.freshMask);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(original.quality),
                          static_cast<uint8_t>(sentinel.quality));
}

void test_calibration_validates_gravity_magnitude_and_total_bias_norm() {
  CalibrationRequest request;
  request.kind = CalibrationKind::ACCELEROMETER_BIAS;
  request.samples = 2;
  request.expectedAccelerationG = Axes{0.0f, 0.0f, 0.79f};
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(validateCalibrationRequest(request).code));
  request.expectedAccelerationG = Axes{0.0f, 0.0f, 1.21f};
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(validateCalibrationRequest(request).code));
  request.expectedAccelerationG = Axes{0.6f, 0.0f, 0.8f};
  TEST_ASSERT_TRUE(validateCalibrationRequest(request).ok());

  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  bus.setRawSample(6400, 0, 0, 0, 6557, 6557, 16393);
  request.expectedAccelerationG = Axes{0.0f, 0.0f, 1.0f};
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startCalibration(request, timing(bus), token).inProgress());
  const PollResult terminal = runToTerminal(driver, bus);
  const OperationResult result = take(driver, token);
  assertTerminalFailure(terminal, result, Err::CALIBRATION_ORIENTATION,
                        OperationState::FAILED);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.4f, result.calibration.bias.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.4f, result.calibration.bias.y);
  TEST_ASSERT_LESS_THAN_FLOAT(0.01f, std::fabs(result.calibration.bias.z));
}

void test_self_test_normalizes_low_power_offsets_and_restores_exact_profile() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  DeviceProfile profile = makeProfile();
  profile.accelOdr = Odr::HZ_208;
  profile.accelPowerMode = AccelPowerMode::LOW_POWER_NORMAL;
  profile.accelOffsetWeight = AccelOffsetWeight::MG_16;
  profile.accelUserOffset = AccelUserOffset{11, -22, 33};
  (void)configure(driver, bus, profile);
  bus.clearTrace();

  OperationToken token;
  TEST_ASSERT_TRUE(
      driver.startSelfTest(SelfTestRequest{5}, timing(bus), token).inProgress());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(runToTerminal(driver, bus).state));
  const OperationResult result = take(driver, token);
  TEST_ASSERT_TRUE(result.selfTest.accelPass);
  TEST_ASSERT_TRUE(result.selfTest.gyroPass);

  const size_t accelTest = findWrite(bus, cmd::REG_CTRL1_XL, 0xFFu, 0x60u);
  const size_t highPerformance = findWrite(bus, cmd::REG_CTRL6_C, 0xFFu, 0u);
  const size_t offsetX = findWrite(bus, cmd::REG_X_OFS_USR, 0xFFu, 0u);
  const size_t offsetY = findWrite(bus, cmd::REG_Y_OFS_USR, 0xFFu, 0u);
  const size_t offsetZ = findWrite(bus, cmd::REG_Z_OFS_USR, 0xFFu, 0u);
  TEST_ASSERT_LESS_THAN(accelTest, highPerformance);
  TEST_ASSERT_LESS_THAN(accelTest, offsetX);
  TEST_ASSERT_LESS_THAN(accelTest, offsetY);
  TEST_ASSERT_LESS_THAN(accelTest, offsetZ);

  TEST_ASSERT_EQUAL_HEX8(0x18u, bus.regs[cmd::REG_CTRL6_C]);
  TEST_ASSERT_EQUAL_HEX8(11u, bus.regs[cmd::REG_X_OFS_USR]);
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(-22),
                         bus.regs[cmd::REG_Y_OFS_USR]);
  TEST_ASSERT_EQUAL_HEX8(33u, bus.regs[cmd::REG_Z_OFS_USR]);
  DeviceProfile verified;
  TEST_ASSERT_TRUE(driver.getVerifiedProfile(verified, bus.nowMs).ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelPowerMode::LOW_POWER_NORMAL),
                          static_cast<uint8_t>(verified.accelPowerMode));
  TEST_ASSERT_EQUAL_INT8(-22, verified.accelUserOffset.y);
}

void test_self_test_restores_verified_not_newly_desired_cancelled_profile() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  DeviceProfile verifiedProfile = makeProfile();
  verifiedProfile.accelFullScale = AccelFs::G_4;
  (void)configure(driver, bus, verifiedProfile);

  DeviceProfile desiredProfile = makeProfile();
  desiredProfile.accelFullScale = AccelFs::G_16;
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startConfigure(desiredProfile, timing(bus), token).inProgress());
  TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
  (void)take(driver, token);
  bus.clearTrace();

  TEST_ASSERT_TRUE(
      driver.startSelfTest(SelfTestRequest{5}, timing(bus), token).inProgress());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(runToTerminal(driver, bus).state));
  (void)take(driver, token);
  DeviceProfile desired;
  DeviceProfile verified;
  TEST_ASSERT_TRUE(driver.getDesiredProfile(desired).ok());
  TEST_ASSERT_TRUE(driver.getVerifiedProfile(verified, bus.nowMs).ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelFs::G_16),
                          static_cast<uint8_t>(desired.accelFullScale));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelFs::G_4),
                          static_cast<uint8_t>(verified.accelFullScale));
  TEST_ASSERT_EQUAL_HEX8(0x48u, bus.regs[cmd::REG_CTRL1_XL]);
}

void advanceSelfTestToRestoredSettling(LSM6DS3TR::LSM6DS3TR& driver,
                                       FakeBus& bus) {
  for (uint32_t count = 0; count < 10000u; ++count) {
    (void)driver.poll(bus.nowMs, 1);
    if (driver.operationActive() &&
        driver.configurationState(bus.nowMs) == ConfigurationState::SETTLING) {
      return;
    }
    ++bus.nowMs;
  }
  TEST_FAIL_MESSAGE("self-test never reached restored settling state");
}

void test_self_test_deadline_and_cancel_provenance_before_write_and_after_restore() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  OperationToken token;

  TEST_ASSERT_TRUE(
      driver.startSelfTest(SelfTestRequest{5}, timing(bus), token).inProgress());
  TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
  OperationResult result = take(driver, token);
  TEST_ASSERT_FALSE(result.hardwareStateMayHaveChanged);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CANCELLED),
                          static_cast<uint8_t>(result.selfTest.primaryStatus.code));
  TEST_ASSERT_TRUE(result.selfTest.restorationStatus.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::KNOWN),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));

  const uint64_t deadline = bus.nowMs + 1u;
  TEST_ASSERT_TRUE(driver
                       .startSelfTest(SelfTestRequest{5},
                                      OperationTiming{bus.nowMs, deadline}, token)
                       .inProgress());
  const PollResult untouchedTimeout = driver.poll(deadline, 8);
  result = take(driver, token);
  assertTerminalFailure(untouchedTimeout, result, Err::DEADLINE_EXPIRED,
                        OperationState::TIMED_OUT);
  TEST_ASSERT_FALSE(result.hardwareStateMayHaveChanged);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::DEADLINE_EXPIRED),
                          static_cast<uint8_t>(result.selfTest.primaryStatus.code));
  TEST_ASSERT_TRUE(result.selfTest.restorationStatus.ok());

  bus.nowMs = deadline + 1u;
  const uint64_t restoredStart = bus.nowMs;
  TEST_ASSERT_TRUE(
      driver.startSelfTest(SelfTestRequest{5}, timing(bus), token).inProgress());
  advanceSelfTestToRestoredSettling(driver, bus);
  const uint64_t restoredElapsed = bus.nowMs - restoredStart;
  const uint32_t transfers = bus.transferCalls;
  TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
  TEST_ASSERT_EQUAL_UINT32(transfers, bus.transferCalls);
  result = take(driver, token);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::SETTLING),
                          static_cast<uint8_t>(result.selfTest.restorationStatus.code));
  TEST_ASSERT_TRUE(result.hardwareStateMayHaveChanged);

  bus.nowMs = driver.validAfterUptimeMs();
  const uint64_t secondStart = bus.nowMs;
  const uint64_t settlingDeadline = secondStart + restoredElapsed + 1u;
  TEST_ASSERT_TRUE(driver
                       .startSelfTest(SelfTestRequest{5},
                                      OperationTiming{secondStart, settlingDeadline}, token)
                       .inProgress());
  while (bus.nowMs < settlingDeadline && driver.operationActive()) {
    (void)driver.poll(bus.nowMs, 1);
    ++bus.nowMs;
  }
  TEST_ASSERT_TRUE(driver.operationActive());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::SETTLING),
                          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
  const PollResult restoredTimeout = driver.poll(settlingDeadline, 1);
  result = take(driver, token);
  assertTerminalFailure(restoredTimeout, result, Err::DEADLINE_EXPIRED,
                        OperationState::TIMED_OUT);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::SETTLING),
                          static_cast<uint8_t>(result.selfTest.restorationStatus.code));
}

void test_poll_exposes_cumulative_progress_and_stops_on_primary_failure_boundary() {
  FakeBus configureBus;
  LSM6DS3TR::LSM6DS3TR configureDriver;
  TEST_ASSERT_TRUE(configureDriver.bind(makeDriverConfig(configureBus)).ok());
  OperationToken token;
  TEST_ASSERT_TRUE(
      configureDriver.startConfigure(makeProfile(), timing(configureBus), token)
          .inProgress());
  const PollResult progress = configureDriver.poll(configureBus.nowMs, 3);
  TEST_ASSERT_EQUAL_UINT8(3u, progress.transactionsUsed);
  TEST_ASSERT_EQUAL_UINT16(3u, progress.transactions);
  TEST_ASSERT_EQUAL_UINT16(MAX_CONFIGURE_TRANSACTIONS,
                           progress.transactionLimit);
  TEST_ASSERT_TRUE(configureDriver.cancelActiveJob(configureBus.nowMs).ok());
  (void)take(configureDriver, token);

  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)configure(driver, bus);
  bus.clearTrace();
  bus.addFault(1, Status::Error(Err::I2C_TIMEOUT, "primary boundary", -701));
  TEST_ASSERT_TRUE(
      driver.startSelfTest(SelfTestRequest{5}, timing(bus), token).inProgress());
  const PollResult failure = driver.poll(bus.nowMs, 8);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::ACTIVE),
                          static_cast<uint8_t>(failure.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(failure.status.code));
  TEST_ASSERT_EQUAL_UINT8(1u, failure.transactionsUsed);
  TEST_ASSERT_EQUAL_UINT16(1u, failure.transactions);
  TEST_ASSERT_EQUAL_UINT32(1u, bus.transferCalls);
  TEST_ASSERT_EQUAL_UINT16(maximumSelfTestTransactions(5),
                           failure.transactionLimit);
  const PollResult terminal = runToTerminal(driver, bus);
  const OperationResult result = take(driver, token);
  assertTerminalFailure(terminal, result, Err::I2C_TIMEOUT,
                        OperationState::FAILED);
  TEST_ASSERT_TRUE(result.selfTest.restorationStatus.ok());

  FakeBus thresholdBus;
  thresholdBus.simulateSelfTest = false;
  LSM6DS3TR::LSM6DS3TR thresholdDriver;
  TEST_ASSERT_TRUE(
      thresholdDriver.bind(makeDriverConfig(thresholdBus)).ok());
  (void)configure(thresholdDriver, thresholdBus);
  thresholdBus.clearTrace();
  TEST_ASSERT_TRUE(thresholdDriver
                       .startSelfTest(SelfTestRequest{5}, timing(thresholdBus),
                                      token)
                       .inProgress());
  uint8_t gyroDataReads = 0;
  for (uint32_t polls = 0; polls < 2000u && gyroDataReads < 20u; ++polls) {
    const size_t traceBefore = thresholdBus.traceCount;
    (void)thresholdDriver.poll(thresholdBus.nowMs, 1);
    if (thresholdBus.traceCount > traceBefore &&
        thresholdBus.trace[thresholdBus.traceCount - 1u].startReg ==
            cmd::REG_DATA_START_GYRO) {
      ++gyroDataReads;
    }
    ++thresholdBus.nowMs;
  }
  TEST_ASSERT_EQUAL_UINT8(20u, gyroDataReads);
  const uint32_t thresholdTransfers = thresholdBus.transferCalls;
  const PollResult thresholdFailure =
      thresholdDriver.poll(thresholdBus.nowMs, 0);
  TEST_ASSERT_EQUAL_UINT32(thresholdTransfers, thresholdBus.transferCalls);
  TEST_ASSERT_EQUAL_UINT8(0u, thresholdFailure.transactionsUsed);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::ACTIVE),
                          static_cast<uint8_t>(thresholdFailure.state));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::SELF_TEST_FAIL),
                          static_cast<uint8_t>(thresholdFailure.status.code));
  const PollResult thresholdTerminal =
      runToTerminal(thresholdDriver, thresholdBus);
  const OperationResult thresholdResult = take(thresholdDriver, token);
  assertTerminalFailure(thresholdTerminal, thresholdResult,
                        Err::SELF_TEST_FAIL, OperationState::FAILED);
  TEST_ASSERT_TRUE(thresholdResult.selfTest.restorationStatus.ok());
}

void test_maximum_transaction_helpers_cover_exact_boundaries() {
  TEST_ASSERT_EQUAL_UINT32(0u, maximumSelfTestTransactions(4));
  TEST_ASSERT_EQUAL_UINT32(240u, maximumSelfTestTransactions(5));
  TEST_ASSERT_EQUAL_UINT32(1760u, maximumSelfTestTransactions(100));
  TEST_ASSERT_EQUAL_UINT32(0u, maximumSelfTestTransactions(101));
  TEST_ASSERT_EQUAL_UINT32(0u, maximumCalibrationTransactions(0));
  TEST_ASSERT_EQUAL_UINT32(4u, maximumCalibrationTransactions(1));
  TEST_ASSERT_EQUAL_UINT32(4000u, maximumCalibrationTransactions(1000));
  TEST_ASSERT_EQUAL_UINT32(0u, maximumCalibrationTransactions(1001));
  TEST_ASSERT_EQUAL_UINT32(0u, maximumFifoPurgeTransactions(0));
  TEST_ASSERT_EQUAL_UINT32(6u, maximumFifoPurgeTransactions(1));
  TEST_ASSERT_EQUAL_UINT32(2053u, maximumFifoPurgeTransactions(2048));
  TEST_ASSERT_EQUAL_UINT32(0u, maximumFifoPurgeTransactions(2049));
}

void test_sa0_vdd_address_and_timeout_are_forwarded_to_every_probe_transfer() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  DriverConfig config = makeDriverConfig(bus);
  config.address = SensorAddress::SA0_VDD;
  config.i2cTimeoutMs = 37u;
  TEST_ASSERT_TRUE(driver.bind(config).ok());
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startProbe(timing(bus), token).inProgress());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                          static_cast<uint8_t>(runToTerminal(driver, bus, 2).state));
  const OperationResult result = take(driver, token);
  TEST_ASSERT_EQUAL_HEX8(0x6Bu, result.probe.address);
  TEST_ASSERT_EQUAL_UINT32(MAX_PROBE_TRANSACTIONS, bus.transferCalls);
  for (size_t index = 0; index < bus.traceCount; ++index) {
    TEST_ASSERT_EQUAL_HEX8(0x6Bu, bus.trace[index].address);
    TEST_ASSERT_EQUAL_UINT32(37u, bus.trace[index].timeoutMs);
  }
}

void test_main_bank_preconditions_and_explicit_bank_recovery_paths() {
  constexpr uint8_t embeddedBank = cmd::MASK_FUNC_CFG_EN;
  OperationToken token;

  {
    FakeBus bus;
    bus.regs[cmd::REG_FUNC_CFG_ACCESS] = embeddedBank;
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    TEST_ASSERT_TRUE(driver.startProbe(timing(bus), token).inProgress());
    const PollResult terminal = runToTerminal(driver, bus);
    const OperationResult result = take(driver, token);
    assertTerminalFailure(terminal, result, Err::CONFIGURATION_UNKNOWN,
                          OperationState::FAILED);
    TEST_ASSERT_EQUAL_UINT32(1u, bus.transferCalls);
    TEST_ASSERT_EQUAL_HEX8(cmd::REG_FUNC_CFG_ACCESS, bus.trace[0].startReg);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TransferKind::WRITE_READ),
                            static_cast<uint8_t>(bus.trace[0].kind));
  }

  {
    FakeBus bus;
    bus.regs[cmd::REG_FUNC_CFG_ACCESS] = embeddedBank;
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    TEST_ASSERT_TRUE(
        driver.startConfigure(makeProfile(), timing(bus), token).inProgress());
    const PollResult terminal = runToTerminal(driver, bus);
    const OperationResult result = take(driver, token);
    assertTerminalFailure(terminal, result, Err::CONFIGURATION_UNKNOWN,
                          OperationState::FAILED);
    TEST_ASSERT_FALSE(result.hardwareStateMayHaveChanged);
    TEST_ASSERT_EQUAL_UINT32(1u, bus.transferCalls);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNKNOWN),
                            static_cast<uint8_t>(
                                driver.configurationState(bus.nowMs)));
  }

  {
    FakeBus bus;
    setFifoPurgePrerequisites(bus);
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    (void)configure(driver, bus);

    bus.clearTrace();
    bus.regs[cmd::REG_FUNC_CFG_ACCESS] = embeddedBank;
    TEST_ASSERT_TRUE(driver.startReconcile(timing(bus), token).inProgress());
    PollResult terminal = runToTerminal(driver, bus);
    OperationResult result = take(driver, token);
    assertTerminalFailure(terminal, result, Err::CONFIGURATION_UNKNOWN,
                          OperationState::FAILED);
    TEST_ASSERT_EQUAL_UINT32(1u, bus.transferCalls);
    TEST_ASSERT_EQUAL_HEX8(cmd::REG_FUNC_CFG_ACCESS, bus.trace[0].startReg);

    bus.clearTrace();
    bus.regs[cmd::REG_FUNC_CFG_ACCESS] = embeddedBank;
    bus.setFifo(2, 0, 0x1000);
    TEST_ASSERT_TRUE(
        driver.startFifoPurge(FifoPurgeRequest{2}, timing(bus), token)
            .inProgress());
    terminal = runToTerminal(driver, bus);
    result = take(driver, token);
    assertTerminalFailure(terminal, result, Err::CONFIGURATION_UNKNOWN,
                          OperationState::FAILED);
    TEST_ASSERT_EQUAL_UINT32(1u, bus.transferCalls);
    TEST_ASSERT_EQUAL_UINT16(0u, bus.fifoDataReads);

    bus.clearTrace();
    bus.regs[cmd::REG_FUNC_CFG_ACCESS] = embeddedBank;
    TEST_ASSERT_TRUE(driver.startReset(timing(bus), token).inProgress());
    terminal = runToTerminal(driver, bus);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                            static_cast<uint8_t>(terminal.state));
    (void)take(driver, token);
    TEST_ASSERT_EQUAL_HEX8(0u, bus.regs[cmd::REG_FUNC_CFG_ACCESS]);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TransferKind::WRITE),
                            static_cast<uint8_t>(bus.trace[0].kind));
    TEST_ASSERT_EQUAL_HEX8(cmd::REG_FUNC_CFG_ACCESS, bus.trace[0].startReg);
    TEST_ASSERT_EQUAL_HEX8(0u, bus.trace[0].firstValue);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TransferKind::WRITE_READ),
                            static_cast<uint8_t>(bus.trace[1].kind));
    TEST_ASSERT_EQUAL_HEX8(cmd::REG_FUNC_CFG_ACCESS, bus.trace[1].startReg);

    bus.clearTrace();
    bus.regs[cmd::REG_FUNC_CFG_ACCESS] = embeddedBank;
    TEST_ASSERT_TRUE(driver.startPowerDown(timing(bus), token).inProgress());
    terminal = runToTerminal(driver, bus);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                            static_cast<uint8_t>(terminal.state));
    (void)take(driver, token);
    TEST_ASSERT_EQUAL_HEX8(0u, bus.regs[cmd::REG_FUNC_CFG_ACCESS]);
    TEST_ASSERT_EQUAL_HEX8(cmd::REG_FUNC_CFG_ACCESS, bus.trace[0].startReg);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TransferKind::WRITE_READ),
                            static_cast<uint8_t>(bus.trace[0].kind));
    TEST_ASSERT_EQUAL_HEX8(cmd::REG_FUNC_CFG_ACCESS, bus.trace[1].startReg);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TransferKind::WRITE),
                            static_cast<uint8_t>(bus.trace[1].kind));
    TEST_ASSERT_EQUAL_HEX8(cmd::REG_FUNC_CFG_ACCESS, bus.trace[2].startReg);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TransferKind::WRITE_READ),
                            static_cast<uint8_t>(bus.trace[2].kind));
    TEST_ASSERT_EQUAL_HEX8(cmd::REG_WHO_AM_I, bus.trace[3].startReg);

    bus.clearTrace();
    bus.regs[cmd::REG_FUNC_CFG_ACCESS] = embeddedBank;
    TEST_ASSERT_TRUE(driver.startRecover(timing(bus), token).inProgress());
    terminal = runToTerminal(driver, bus);
    result = take(driver, token);
    assertTerminalFailure(terminal, result, Err::CONFIGURATION_UNKNOWN,
                          OperationState::FAILED);
    TEST_ASSERT_FALSE(result.hardwareStateMayHaveChanged);
    TEST_ASSERT_EQUAL_UINT32(1u, bus.transferCalls);
    TEST_ASSERT_EQUAL_HEX8(cmd::REG_FUNC_CFG_ACCESS, bus.trace[0].startReg);
  }
}

void test_fifo_preconditions_overrun_and_ambiguous_consumption_are_precise() {
  {
    FakeBus bus;
    setFifoPurgePrerequisites(bus);
    bus.regs[cmd::REG_WHO_AM_I] = 0x5Au;
    bus.setFifo(3, 0, 0x4000);
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    OperationToken token;
    TEST_ASSERT_TRUE(
        driver.startFifoPurge(FifoPurgeRequest{3}, timing(bus), token)
            .inProgress());
    const PollResult terminal = runToTerminal(driver, bus);
    const OperationResult result = take(driver, token);
    assertTerminalFailure(terminal, result, Err::CHIP_ID_MISMATCH,
                          OperationState::FAILED);
    TEST_ASSERT_FALSE(result.hardwareStateMayHaveChanged);
    TEST_ASSERT_EQUAL_HEX8(0x5Au, result.probe.whoAmI);
    TEST_ASSERT_EQUAL_UINT32(2u, bus.transferCalls);
    TEST_ASSERT_EQUAL_HEX8(cmd::REG_FUNC_CFG_ACCESS, bus.trace[0].startReg);
    TEST_ASSERT_EQUAL_HEX8(cmd::REG_WHO_AM_I, bus.trace[1].startReg);
    TEST_ASSERT_EQUAL_UINT16(0u, bus.fifoDataReads);
  }

  {
    FakeBus bus;
    bus.regs[cmd::REG_CTRL3_C] = cmd::MASK_IF_INC;
    bus.setFifo(3, 0, 0x4100);
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    OperationToken token;
    TEST_ASSERT_TRUE(
        driver.startFifoPurge(FifoPurgeRequest{3}, timing(bus), token)
            .inProgress());
    const PollResult terminal = runToTerminal(driver, bus);
    const OperationResult result = take(driver, token);
    assertTerminalFailure(terminal, result, Err::CONFIGURATION_UNKNOWN,
                          OperationState::FAILED);
    TEST_ASSERT_FALSE(result.hardwareStateMayHaveChanged);
    TEST_ASSERT_EQUAL_UINT32(3u, bus.transferCalls);
    TEST_ASSERT_EQUAL_HEX8(cmd::REG_FUNC_CFG_ACCESS, bus.trace[0].startReg);
    TEST_ASSERT_EQUAL_HEX8(cmd::REG_WHO_AM_I, bus.trace[1].startReg);
    TEST_ASSERT_EQUAL_HEX8(cmd::REG_CTRL3_C, bus.trace[2].startReg);
    TEST_ASSERT_EQUAL_UINT16(0u, bus.fifoDataReads);
  }

  {
    FakeBus bus;
    setFifoPurgePrerequisites(bus);
    bus.setFifo(2, 0, 0x4200);
    bus.addFault(5, Status::Error(Err::I2C_TIMEOUT,
                                  "ambiguous FIFO consumption", -704),
                 true);
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    OperationToken token;
    TEST_ASSERT_TRUE(
        driver.startFifoPurge(FifoPurgeRequest{2}, timing(bus), token)
            .inProgress());
    const PollResult terminal = runToTerminal(driver, bus);
    const OperationResult result = take(driver, token);
    assertTerminalFailure(terminal, result, Err::I2C_TIMEOUT,
                          OperationState::INDETERMINATE);
    TEST_ASSERT_TRUE(result.hardwareStateMayHaveChanged);
    TEST_ASSERT_EQUAL_UINT16(0u, result.fifoPurge.wordsDiscarded);
    TEST_ASSERT_EQUAL_UINT16(1u, bus.fifoUnreadWords);
    TEST_ASSERT_EQUAL_UINT16(1u, bus.fifoDataReads);
    TEST_ASSERT_TRUE(bus.trace[4].effectApplied);
  }
}

void test_read_only_and_destructive_jobs_fail_precisely_at_every_transfer_stage() {
  {
    FakeBus referenceBus;
    LSM6DS3TR::LSM6DS3TR referenceDriver;
    TEST_ASSERT_TRUE(referenceDriver.bind(makeDriverConfig(referenceBus)).ok());
    const uint32_t generation =
        configure(referenceDriver, referenceBus).configuration.generation;
    referenceBus.clearTrace();
    OperationToken token;
    TEST_ASSERT_TRUE(referenceDriver.startReconcile(timing(referenceBus), token)
                         .inProgress());
    (void)runToTerminal(referenceDriver, referenceBus);
    const uint32_t stageCount = referenceBus.transferCalls;
    TEST_ASSERT_EQUAL_UINT32(MAX_RECONCILE_TRANSACTIONS, stageCount);
    (void)take(referenceDriver, token);

    for (uint32_t stage = 1; stage <= stageCount; ++stage) {
      FakeBus bus;
      LSM6DS3TR::LSM6DS3TR driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      (void)configure(driver, bus);
      bus.clearTrace();
      bus.addFault(stage, Status::Error(Err::I2C_TIMEOUT,
                                       "reconcile stage failure",
                                       static_cast<int32_t>(stage)));
      TEST_ASSERT_TRUE(driver.startReconcile(timing(bus), token).inProgress());
      const PollResult terminal = runToTerminal(driver, bus, 4);
      const OperationResult result = take(driver, token);
      assertTerminalFailure(terminal, result, Err::I2C_TIMEOUT,
                            OperationState::FAILED);
      TEST_ASSERT_FALSE(result.hardwareStateMayHaveChanged);
      TEST_ASSERT_EQUAL_UINT32(stage, result.transactions);
      TEST_ASSERT_EQUAL_UINT32(generation, driver.configGeneration());
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::KNOWN),
                              static_cast<uint8_t>(
                                  driver.configurationState(bus.nowMs)));
    }
  }

  {
    FakeBus referenceBus;
    LSM6DS3TR::LSM6DS3TR referenceDriver;
    TEST_ASSERT_TRUE(referenceDriver.bind(makeDriverConfig(referenceBus)).ok());
    (void)configure(referenceDriver, referenceBus);
    referenceBus.clearTrace();
    referenceBus.regs[cmd::REG_FUNC_CFG_ACCESS] = cmd::MASK_FUNC_CFG_EN;
    OperationToken token;
    TEST_ASSERT_TRUE(referenceDriver.startPowerDown(timing(referenceBus), token)
                         .inProgress());
    (void)runToTerminal(referenceDriver, referenceBus);
    const uint32_t stageCount = referenceBus.transferCalls;
    TEST_ASSERT_EQUAL_UINT32(MAX_POWER_DOWN_TRANSACTIONS, stageCount);
    (void)take(referenceDriver, token);

    for (uint32_t stage = 1; stage <= stageCount; ++stage) {
      FakeBus bus;
      LSM6DS3TR::LSM6DS3TR driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      (void)configure(driver, bus);
      bus.clearTrace();
      bus.regs[cmd::REG_FUNC_CFG_ACCESS] = cmd::MASK_FUNC_CFG_EN;
      bus.addFault(stage, Status::Error(Err::I2C_TIMEOUT,
                                       "power-down stage failure",
                                       static_cast<int32_t>(stage)));
      TEST_ASSERT_TRUE(driver.startPowerDown(timing(bus), token).inProgress());
      const PollResult terminal = runToTerminal(driver, bus, 3);
      const OperationResult result = take(driver, token);
      assertTerminalFailure(terminal, result, Err::I2C_TIMEOUT,
                            stage == 1u ? OperationState::FAILED
                                        : OperationState::INDETERMINATE);
      TEST_ASSERT_EQUAL(stage > 1u, result.hardwareStateMayHaveChanged);
      TEST_ASSERT_EQUAL_UINT32(stage, result.transactions);
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(
                                  stage == 1u ? ConfigurationState::KNOWN
                                              : ConfigurationState::UNKNOWN),
                              static_cast<uint8_t>(
                                  driver.configurationState(bus.nowMs)));
    }
  }

  {
    CalibrationRequest request;
    request.kind = CalibrationKind::GYROSCOPE_BIAS;
    request.samples = 2;
    FakeBus referenceBus;
    LSM6DS3TR::LSM6DS3TR referenceDriver;
    TEST_ASSERT_TRUE(referenceDriver.bind(makeDriverConfig(referenceBus)).ok());
    (void)configure(referenceDriver, referenceBus);
    referenceBus.clearTrace();
    OperationToken token;
    TEST_ASSERT_TRUE(referenceDriver
                         .startCalibration(request, timing(referenceBus), token)
                         .inProgress());
    (void)runToTerminal(referenceDriver, referenceBus);
    const uint32_t stageCount = referenceBus.transferCalls;
    TEST_ASSERT_EQUAL_UINT32(2u * request.samples, stageCount);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(
        maximumCalibrationTransactions(request.samples), stageCount);
    (void)take(referenceDriver, token);

    for (uint32_t stage = 1; stage <= stageCount; ++stage) {
      FakeBus bus;
      LSM6DS3TR::LSM6DS3TR driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      (void)configure(driver, bus);
      bus.clearTrace();
      bus.addFault(stage, Status::Error(Err::I2C_TIMEOUT,
                                       "calibration stage failure",
                                       static_cast<int32_t>(stage)));
      TEST_ASSERT_TRUE(
          driver.startCalibration(request, timing(bus), token).inProgress());
      const PollResult terminal = runToTerminal(driver, bus, 4);
      const OperationResult result = take(driver, token);
      assertTerminalFailure(terminal, result, Err::I2C_TIMEOUT,
                            OperationState::FAILED);
      TEST_ASSERT_FALSE(result.hardwareStateMayHaveChanged);
      TEST_ASSERT_EQUAL_UINT32(stage, result.transactions);
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::KNOWN),
                              static_cast<uint8_t>(
                                  driver.configurationState(bus.nowMs)));
    }
  }

  {
    for (uint32_t stage = 1; stage <= maximumFifoPurgeTransactions(2); ++stage) {
      FakeBus bus;
      setFifoPurgePrerequisites(bus);
      bus.setFifo(2, 0, 0x4300);
      LSM6DS3TR::LSM6DS3TR driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      bus.addFault(stage, Status::Error(Err::I2C_TIMEOUT,
                                       "FIFO stage failure",
                                       static_cast<int32_t>(stage)));
      OperationToken token;
      TEST_ASSERT_TRUE(
          driver.startFifoPurge(FifoPurgeRequest{2}, timing(bus), token)
              .inProgress());
      const PollResult terminal = runToTerminal(driver, bus, 4);
      const OperationResult result = take(driver, token);
      assertTerminalFailure(terminal, result, Err::I2C_TIMEOUT,
                            stage <= 4u ? OperationState::FAILED
                                        : OperationState::INDETERMINATE);
      TEST_ASSERT_EQUAL(stage > 4u, result.hardwareStateMayHaveChanged);
      TEST_ASSERT_EQUAL_UINT32(stage, result.transactions);
    }
  }
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_status_contract);
  RUN_TEST(test_validate_driver_config_and_address);
  RUN_TEST(test_validate_profile_rejects_all_unsupported_or_unsafe_values);
  RUN_TEST(test_all_odr_periods_are_bounded_and_ordered);
  RUN_TEST(test_all_accel_sensitivities_and_edges);
  RUN_TEST(test_all_gyro_sensitivities_and_full_int16_range);
  RUN_TEST(test_temperature_decode_boundaries);
  RUN_TEST(test_convert_sample_is_self_contained_and_preserves_provenance);
  RUN_TEST(test_bias_and_calibration_validation_reject_non_finite_values);
  RUN_TEST(test_bind_unbind_and_rebind_are_bus_silent);
  RUN_TEST(test_invalid_rebind_preserves_working_binding);
  RUN_TEST(test_unbound_and_invalid_timing_starts_are_zero_io);
  RUN_TEST(test_probe_budget_zero_then_one_and_exactly_once_result);
  RUN_TEST(test_wrong_token_does_not_consume_pending_result);
  RUN_TEST(test_active_and_pending_operations_cannot_be_overwritten);
  RUN_TEST(test_probe_failures_are_precise_and_never_gate_later_work);
  RUN_TEST(test_probe_deadline_and_uint64_boundary_are_safe);
  RUN_TEST(test_configure_enforces_budget_and_verifies_before_generation_increment);
  RUN_TEST(test_complete_profile_clears_all_disabled_subsystems);
  RUN_TEST(test_configure_failure_at_every_transport_stage_has_precise_effect_state);
  RUN_TEST(test_ambiguous_write_effect_is_observable_and_never_retried);
  RUN_TEST(test_configuration_readback_mismatch_reports_exact_register_values);
  RUN_TEST(test_cancel_before_and_after_configuration_effect_is_bus_silent);
  RUN_TEST(test_configure_cancellation_is_safe_after_every_transfer_stage);
  RUN_TEST(test_timeout_after_partial_configuration_exposes_unknown_state);
  RUN_TEST(test_configure_terminal_publishes_only_after_settled_and_verified);
  RUN_TEST(test_reconcile_is_read_only_and_only_full_verification_clears_unknown);
  RUN_TEST(test_identity_mismatch_invalidates_all_identity_dependent_jobs);
  RUN_TEST(test_sample_is_atomic_tokened_and_validity_aware);
  RUN_TEST(test_direct_unverified_sample_has_no_false_freshness);
  RUN_TEST(test_powered_down_sensor_combinations_have_exact_valid_masks);
  RUN_TEST(test_sample_wait_steps_use_zero_i2c_and_timeout_without_stale_data);
  RUN_TEST(test_combined_sample_does_not_claim_fresh_temperature_without_tda);
  RUN_TEST(test_sample_ready_on_last_check_succeeds_and_stuck_ready_is_bounded);
  RUN_TEST(test_failed_new_sample_cannot_publish_previous_sample);
  RUN_TEST(test_sample_conversion_keeps_original_scale_after_reconfigure);
  RUN_TEST(test_unknown_and_settling_configuration_gate_samples_without_i2c);
  RUN_TEST(test_reset_orders_required_modes_and_enforces_no_i2c_gate);
  RUN_TEST(test_boot_has_same_bounded_inaccessible_window);
  RUN_TEST(test_cancel_reset_after_command_marks_configuration_unknown_without_i2c);
  RUN_TEST(test_recover_reprobes_and_reapplies_verified_profile_without_retry_policy);
  RUN_TEST(test_reset_boot_and_recover_failures_are_bounded_at_every_transfer_stage);
  RUN_TEST(test_reset_boot_and_recover_completion_poll_boundaries_are_exact);
  RUN_TEST(test_reset_and_boot_wait_deadlines_saturate_at_uint64_max);
  RUN_TEST(test_self_test_is_staged_bounded_and_restores_configuration);
  RUN_TEST(test_self_test_wakes_and_restores_a_sleeping_gyro_profile);
  RUN_TEST(test_self_test_failures_are_bounded_at_every_transfer_stage);
  RUN_TEST(test_self_test_preserves_primary_and_restoration_failures);
  RUN_TEST(test_self_test_restoration_deadline_preserves_primary_failure);
  RUN_TEST(test_self_test_negative_stimulus_reports_absolute_deltas);
  RUN_TEST(test_self_test_cancel_during_settle_is_bus_silent_and_unknown);
  RUN_TEST(test_self_test_intermittent_not_ready_uses_bounded_bus_silent_cadence);
  RUN_TEST(test_self_test_three_not_ready_checks_restore_known_configuration);
  RUN_TEST(test_self_test_invalid_request_and_timeout_are_zero_retry);
  RUN_TEST(test_gyro_calibration_is_staged_fixed_count_and_owner_timed);
  RUN_TEST(test_gyro_calibration_rejects_sleeping_sensor_without_i2c);
  RUN_TEST(test_accel_calibration_uses_explicit_fixture_vector_and_rejects_orientation);
  RUN_TEST(test_accel_calibration_int16_span_cannot_overflow_peak_to_peak);
  RUN_TEST(test_calibration_not_ready_retries_and_post_sample_waits_are_bus_silent);
  RUN_TEST(test_calibration_three_not_ready_checks_fail_without_mutating_configuration);
  RUN_TEST(test_all_other_hardware_entry_points_are_excluded_by_active_job);
  RUN_TEST(test_diagnostic_bounds_and_raw_write_invalidation_are_explicit);
  RUN_TEST(test_diagnostic_sync_writes_invalidate_and_reconcile_exact_addresses);
  RUN_TEST(test_ambiguous_diagnostic_write_invalidates_without_hidden_refresh);
  RUN_TEST(test_power_down_is_explicit_fallible_and_partial_failure_is_indeterminate);
  RUN_TEST(test_power_down_is_available_from_unconfigured_and_unknown_states);
  RUN_TEST(test_successful_power_down_leaves_no_false_known_configuration);
  RUN_TEST(test_fifo_purge_is_bounded_reports_loss_and_honors_budget);
  RUN_TEST(test_partial_fifo_purge_cancellation_preserves_discard_count);
  RUN_TEST(test_fifo_concurrent_arrival_is_reported_as_truncated);
  RUN_TEST(test_fifo_overrun_is_data_loss_not_a_valid_batch);
  RUN_TEST(test_settle_bounds_match_supported_filter_and_power_profiles);
  RUN_TEST(test_profile_rejections_are_complete_and_bus_silent);
  RUN_TEST(test_nondefault_profile_encodes_and_reads_back_every_supported_field);
  RUN_TEST(test_reconcile_known_profile_preserves_generation_and_settle_evidence);
  RUN_TEST(test_sleeping_gyro_rejects_rate_but_allows_temperature);
  RUN_TEST(test_sample_readiness_cadence_tracks_slowest_requested_source);
  RUN_TEST(test_conversion_preserves_quality_and_rejects_malformed_masks_atomically);
  RUN_TEST(test_calibration_validates_gravity_magnitude_and_total_bias_norm);
  RUN_TEST(test_self_test_normalizes_low_power_offsets_and_restores_exact_profile);
  RUN_TEST(test_self_test_restores_verified_not_newly_desired_cancelled_profile);
  RUN_TEST(test_self_test_deadline_and_cancel_provenance_before_write_and_after_restore);
  RUN_TEST(test_poll_exposes_cumulative_progress_and_stops_on_primary_failure_boundary);
  RUN_TEST(test_maximum_transaction_helpers_cover_exact_boundaries);
  RUN_TEST(test_sa0_vdd_address_and_timeout_are_forwarded_to_every_probe_transfer);
  RUN_TEST(test_main_bank_preconditions_and_explicit_bank_recovery_paths);
  RUN_TEST(test_fifo_preconditions_overrun_and_ambiguous_consumption_are_precise);
  RUN_TEST(test_read_only_and_destructive_jobs_fail_precisely_at_every_transfer_stage);
  return UNITY_END();
}
