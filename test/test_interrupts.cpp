/// @file test_interrupts.cpp
/// @brief Source-derived interrupt routing and bounded event observation tests.
#include <unity.h>

#include <cstdint>
#include <initializer_list>

#include "LSM6DS3TR/LSM6DS3TR.h"
#include "support/FakeBus.h"

using namespace LSM6DS3TR;
using namespace LSM6DS3TRTest;

void lowerFifoTestCeiling(LSM6DS3TR::LSM6DS3TR& driver, uint32_t limit);

namespace {
using Driver = LSM6DS3TR::LSM6DS3TR;

DeviceProfile interruptProfile(bool collecting = false) {
  DeviceProfile profile;
  profile.fifo.enabled = true;
  profile.fifo.mode = collecting ? FifoMode::CONTINUOUS : FifoMode::BYPASS;
  profile.fifo.odr = Odr::HZ_104;
  profile.fifo.fourthDataset = FifoFourthDataset::TIMESTAMP_STEP;
  profile.fifo.fourthDecimation = FifoDecimation::EVERY_1;
  profile.interrupts.enabled = true;
  profile.interrupts.latchEvents = true;
  profile.interrupts.orientation.mode = OrientationMode::SIX_D;
  profile.interrupts.int1.orientation = true;
  profile.interrupts.int1.stepDetected = true;
  profile.interrupts.int1.timestampEndCounter = true;
  profile.interrupts.int2.stepOverflow = true;
  return profile;
}

DeviceProfile fullInterruptProfile() {
  DeviceProfile profile = interruptProfile(true);
  profile.fifo.gyroDecimation = FifoDecimation::EVERY_1;
  profile.fifo.accelDecimation = FifoDecimation::EVERY_1;
  profile.fifo.timestampResolution = TimestampResolution::US_25;
  profile.fifo.watermarkWords = 0x345;
  profile.interrupts.polarity = InterruptPolarity::ACTIVE_LOW;
  profile.interrupts.drive = InterruptDrive::OPEN_DRAIN_PAD;
  profile.interrupts.dataReadyMode = DataReadyMode::PULSED;
  profile.interrupts.int2OnInt1 = true;
  profile.interrupts.orientation.mode = OrientationMode::FOUR_D;
  profile.interrupts.orientation.threshold = OrientationThreshold::DEG_50;
  for (InterruptRoutes* route : {&profile.interrupts.int1,
                                 &profile.interrupts.int2}) {
    route->accelReady = true;
    route->gyroReady = true;
    route->fifoWatermark = true;
    route->fifoFull = true;
    route->fifoOverrun = true;
    route->orientation = true;
  }
  profile.interrupts.int2.temperatureReady = true;
  return profile;
}

OperationTiming interruptTiming(const FakeBus& bus) {
  return {bus.nowMs, bus.nowMs + 120000U};
}

void interruptRun(Driver& driver, FakeBus& bus) {
  const uint8_t budgets[] = {0, 1, 255, 2, 0, 7};
  for (uint32_t count = 0; count < 150000U && driver.operationActive(); ++count) {
    const uint8_t budget = budgets[count % 6U];
    const uint32_t before = bus.transferCalls;
    const PollResult result = driver.poll(bus.nowMs++, budget);
    TEST_ASSERT_EQUAL_UINT32(bus.transferCalls - before, result.transactionsUsed);
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(budget, result.transactionsUsed);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(result.transactionLimit, result.transactions);
  }
  TEST_ASSERT_FALSE(driver.operationActive());
  TEST_ASSERT_TRUE(driver.resultPending());
}

OperationResult interruptTake(Driver& driver, OperationToken token) {
  OperationResult result;
  TEST_ASSERT_TRUE(driver.takeResult(token, result).ok());
  TEST_ASSERT_EQUAL_UINT64(token.value, result.token.value);
  OperationResult second;
  TEST_ASSERT_FALSE(driver.takeResult(token, second).ok());
  return result;
}

OperationResult interruptConfigure(Driver& driver, FakeBus& bus,
                                    const DeviceProfile& profile) {
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startConfigure(profile, interruptTiming(bus), token).inProgress());
  interruptRun(driver, bus);
  const OperationResult result = interruptTake(driver, token);
  TEST_ASSERT_TRUE_MESSAGE(result.status.ok(), result.status.msg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::KNOWN),
      static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
  return result;
}

EventReadRequest allEvents() {
  EventReadRequest request;
  request.dataReady = true;
  request.fifo = true;
  request.embedded = true;
  request.orientation = true;
  return request;
}

OperationResult interruptRead(Driver& driver, FakeBus& bus,
                               const EventReadRequest& request = allEvents()) {
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startEventRead(request, interruptTiming(bus), token).inProgress());
  interruptRun(driver, bus);
  return interruptTake(driver, token);
}

void advanceInterruptCalls(Driver& driver, FakeBus& bus, uint32_t target) {
  for (uint32_t count = 0; count < 150000U && driver.operationActive() &&
       bus.transferCalls < target; ++count) driver.poll(bus.nowMs++, 1);
  TEST_ASSERT_EQUAL_UINT32(target, bus.transferCalls);
}

size_t interruptFindWrite(const FakeBus& bus, uint8_t reg, uint8_t value,
                           size_t first = 0) {
  for (size_t index = first; index < bus.traceCount; ++index)
    if (bus.trace[index].kind == TransferKind::WRITE &&
        bus.trace[index].startReg == reg && bus.trace[index].firstValue == value)
      return index;
  return bus.traceCount;
}

void test_interrupt_route_pin_matrix_has_independent_wire_images() {
  // Datasheet Tables44..47 and197..200. Unsupported pin/source pairs are
  // deliberately present in the value type and must fail atomically.
  struct RouteFixture { bool InterruptRoutes::*field; uint8_t reg1; uint8_t bit1;
                        uint8_t reg2; uint8_t bit2; };
  const RouteFixture fixtures[] = {
      {&InterruptRoutes::accelReady, 0x0D, 0x01, 0x0E, 0x01},
      {&InterruptRoutes::gyroReady, 0x0D, 0x02, 0x0E, 0x02},
      {&InterruptRoutes::temperatureReady, 0, 0, 0x0E, 0x04},
      {&InterruptRoutes::fifoWatermark, 0x0D, 0x08, 0x0E, 0x08},
      {&InterruptRoutes::fifoOverrun, 0x0D, 0x10, 0x0E, 0x10},
      {&InterruptRoutes::fifoFull, 0x0D, 0x20, 0x0E, 0x20},
      {&InterruptRoutes::stepDetected, 0x0D, 0x80, 0, 0},
      {&InterruptRoutes::stepOverflow, 0, 0, 0x0E, 0x40},
      {&InterruptRoutes::timestampEndCounter, 0x5E, 0x01, 0, 0},
      {&InterruptRoutes::orientation, 0x5E, 0x04, 0x5F, 0x04}};
  for (const RouteFixture& fixture : fixtures) {
    for (uint8_t pin = 0; pin < 2; ++pin) {
      FakeBus bus;
      Driver driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      DeviceProfile profile = interruptProfile(true);
      profile.interrupts.int1 = {};
      profile.interrupts.int2 = {};
      profile.interrupts.orientation.mode = fixture.field == &InterruptRoutes::orientation
          ? OrientationMode::SIX_D : OrientationMode::OFF;
      InterruptRoutes& route = pin == 0 ? profile.interrupts.int1 :
                                          profile.interrupts.int2;
      route.*fixture.field = true;
      const uint8_t reg = pin == 0 ? fixture.reg1 : fixture.reg2;
      OperationToken token;
      if (reg == 0) {
        TEST_ASSERT_FALSE(driver.startConfigure(profile, interruptTiming(bus), token).inProgress());
        TEST_ASSERT_FALSE(token.valid());
        TEST_ASSERT_EQUAL_UINT32(0, bus.transferCalls);
      } else {
        interruptConfigure(driver, bus, profile);
        TEST_ASSERT_EQUAL_HEX8(pin == 0 ? fixture.bit1 : fixture.bit2, bus.regs[reg]);
        const uint8_t routeRegisters[] = {0x0D, 0x0E, 0x5E, 0x5F};
        for (uint8_t other : routeRegisters)
          if (other != reg) TEST_ASSERT_EQUAL_HEX8(0, bus.regs[other]);
      }
    }
  }
}

void test_interrupt_invalid_profiles_are_total_and_bus_silent() {
  DeviceProfile candidates[17];
  for (DeviceProfile& profile : candidates) profile = interruptProfile();
  candidates[0].interrupts.polarity = static_cast<InterruptPolarity>(2);
  candidates[1].interrupts.drive = static_cast<InterruptDrive>(2);
  candidates[2].interrupts.dataReadyMode = static_cast<DataReadyMode>(2);
  candidates[3].interrupts.orientation.mode = static_cast<OrientationMode>(3);
  candidates[4].interrupts.orientation.threshold = static_cast<OrientationThreshold>(4);
  candidates[5].interrupts.enabled = false;
  candidates[6].interrupts.int1.temperatureReady = true;
  candidates[7].interrupts.int2.stepDetected = true;
  candidates[8].interrupts.int1.stepOverflow = true;
  candidates[9].interrupts.int2.timestampEndCounter = true;
  candidates[10].interrupts.orientation.mode = OrientationMode::OFF;
  candidates[11].interrupts.int1.orientation = false;
  candidates[12].accelOdr = Odr::POWER_DOWN;
  candidates[13].accelFilter.lowPassOn6d = true;
  candidates[13].interrupts.orientation.mode = OrientationMode::OFF;
  candidates[14].interrupts.int1.fifoWatermark = true;
  candidates[15].fifo = {};
  candidates[16].interrupts.int1.gyroReady = true;
  candidates[16].gyroSleepEnabled = true;
  for (const DeviceProfile& profile : candidates) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    OperationToken token{123};
    TEST_ASSERT_FALSE(driver.startConfigure(profile, interruptTiming(bus), token).inProgress());
    TEST_ASSERT_FALSE(token.valid());
    TEST_ASSERT_EQUAL_UINT32(0, bus.transferCalls);
  }
}

void test_interrupt_complete_image_composes_shared_fields_and_thresholds() {
  for (uint8_t threshold = 0; threshold < 4; ++threshold) {
    for (OrientationMode mode : {OrientationMode::SIX_D, OrientationMode::FOUR_D}) {
      FakeBus bus;
      Driver driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      DeviceProfile profile = fullInterruptProfile();
      profile.interrupts.orientation.mode = mode;
      profile.interrupts.orientation.threshold = static_cast<OrientationThreshold>(threshold);
      interruptConfigure(driver, bus, profile);
      TEST_ASSERT_EQUAL_HEX8(0x74, bus.regs[0x12]);
      TEST_ASSERT_EQUAL_HEX8(0x20, bus.regs[0x13]);
      TEST_ASSERT_EQUAL_HEX8(0x80, bus.regs[0x0B]);
      TEST_ASSERT_EQUAL_HEX8(0xBB, bus.regs[0x0D]);
      TEST_ASSERT_EQUAL_HEX8(0x7F, bus.regs[0x0E]);
      TEST_ASSERT_EQUAL_HEX8(0x05, bus.regs[0x5E]);
      TEST_ASSERT_EQUAL_HEX8(0x04, bus.regs[0x5F]);
      TEST_ASSERT_EQUAL_HEX8(0x81, bus.regs[0x58]);
      TEST_ASSERT_EQUAL_HEX8((threshold << 5U) | (mode == OrientationMode::FOUR_D ? 0x80U : 0U),
                            bus.regs[0x59]);
      TEST_ASSERT_EQUAL_HEX8(0x83, bus.regs[0x07]);
      TEST_ASSERT_EQUAL_HEX8(0x08, bus.regs[0x09]);
      TEST_ASSERT_EQUAL_HEX8(0x34, bus.regs[0x19]);
      TEST_ASSERT_EQUAL_HEX8(0x10, bus.regs[0x5C]);
      TEST_ASSERT_EQUAL_HEX8(0, bus.regs[0x1A]);
      TEST_ASSERT_EQUAL_HEX8(0, bus.regs[0x17]);
    }
  }
}

void test_interrupt_configure_masks_before_dependencies_and_activates_after_settling() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  for (uint8_t reg : {0x0D, 0x0E, 0x5E, 0x5F}) bus.regs[reg] = 0xFF;
  bus.setEventSources(0x55, 0x18);
  const DeviceProfile profile = fullInterruptProfile();
  const OperationResult result = interruptConfigure(driver, bus, profile);
  TEST_ASSERT_EQUAL_UINT32(106U, result.transactions);
  size_t lastMaskRead = 0;
  size_t firstDependencyWrite = bus.traceCount;
  size_t firstActivation = bus.traceCount;
  uint8_t masked = 0;
  for (size_t i = 0; i < bus.traceCount; ++i) {
    const Transfer& transfer = bus.trace[i];
    const bool route = transfer.startReg == 0x0D || transfer.startReg == 0x0E ||
                       transfer.startReg == 0x5E || transfer.startReg == 0x5F;
    if (route && transfer.kind == TransferKind::WRITE_READ && masked < 4) {
      ++masked;
      lastMaskRead = i;
    }
    if (!route && transfer.kind == TransferKind::WRITE &&
        firstDependencyWrite == bus.traceCount) firstDependencyWrite = i;
    if (route && transfer.kind == TransferKind::WRITE && transfer.firstValue != 0 &&
        firstActivation == bus.traceCount) firstActivation = i;
  }
  TEST_ASSERT_EQUAL_UINT8(4, masked);
  TEST_ASSERT_TRUE(lastMaskRead < firstDependencyWrite);
  TEST_ASSERT_TRUE(firstActivation < bus.traceCount);
  TEST_ASSERT_TRUE(bus.trace[firstActivation].atMs >= result.configuration.validAfterUptimeMs);
  TEST_ASSERT_EQUAL_UINT32(0, bus.orientationSourceReads);
  TEST_ASSERT_EQUAL_UINT32(0, bus.embeddedSourceReads);
  TEST_ASSERT_EQUAL_HEX8(0x55, bus.regs[0x1D]);
  TEST_ASSERT_EQUAL_HEX8(0x18, bus.regs[0x53]);
}

void test_event_reads_keep_independent_times_raw_sources_and_decoded_causes() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  interruptConfigure(driver, bus, interruptProfile(true));
  bus.clearTrace();
  bus.fifoFullSmart = true;
  bus.setFifo(17, 0, 0x1234, true);
  bus.setEventSources(0x7F, 0xFF, 0xFF);
  const OperationResult result = interruptRead(driver, bus);
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_EQUAL_UINT32(6, result.transactions);
  TEST_ASSERT_EQUAL_UINT8(15, result.events.requestedMask);
  TEST_ASSERT_EQUAL_UINT8(15, result.events.readMask);
  TEST_ASSERT_EQUAL_UINT64(result.token.value, result.events.token.value);
  TEST_ASSERT_EQUAL_UINT32(result.configuration.generation, result.events.configGeneration);
  TEST_ASSERT_EQUAL_HEX8(0xFF, result.events.ready.raw);
  TEST_ASSERT_EQUAL_HEX8(0xFF, result.events.embedded.raw);
  TEST_ASSERT_EQUAL_HEX8(0x7F, result.events.orientation.raw);
  TEST_ASSERT_EQUAL_HEX8(0x07, result.events.dataReadyMask);
  TEST_ASSERT_TRUE(result.events.ready.valid);
  TEST_ASSERT_TRUE(result.events.fifo.valid);
  TEST_ASSERT_TRUE(result.events.embedded.valid);
  TEST_ASSERT_TRUE(result.events.orientation.valid);
  TEST_ASSERT_TRUE(result.events.fifoWatermark);
  TEST_ASSERT_TRUE(result.events.fifoOverrun);
  TEST_ASSERT_TRUE(result.events.fifoFull);
  TEST_ASSERT_TRUE(result.events.stepDetected);
  TEST_ASSERT_TRUE(result.events.stepOverflow);
  TEST_ASSERT_TRUE(result.events.orientationDetected);
  TEST_ASSERT_TRUE(result.events.xLow && result.events.xHigh && result.events.yLow &&
                   result.events.yHigh && result.events.zLow && result.events.zHigh);
  TEST_ASSERT_TRUE(result.events.timestampRouted);
  TEST_ASSERT_EQUAL_UINT8(12, result.events.acknowledgedMask);
  TEST_ASSERT_EQUAL_UINT8(12, result.events.mayHaveAcknowledgedMask);
  TEST_ASSERT_EQUAL_HEX8(0x3F, bus.regs[0x1D]);
  TEST_ASSERT_EQUAL_HEX8(0xE7, bus.regs[0x53]);
  TEST_ASSERT_EQUAL_UINT16(0, bus.fifoDataReads);
  TEST_ASSERT_EQUAL_UINT32(0, bus.timestampResetWrites);
  TEST_ASSERT_EQUAL_HEX8(0xFF, bus.regs[0x1E]);
}

void test_event_nonlatched_and_4d_sources_never_claim_unavailable_evidence() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  DeviceProfile profile = interruptProfile();
  profile.interrupts.latchEvents = false;
  profile.interrupts.orientation.mode = OrientationMode::FOUR_D;
  interruptConfigure(driver, bus, profile);
  bus.clearTrace();
  bus.setEventSources(0x7F, 0x18);
  EventReadRequest request;
  request.embedded = true;
  request.orientation = true;
  const OperationResult result = interruptRead(driver, bus, request);
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_FALSE(result.hardwareStateMayHaveChanged);
  TEST_ASSERT_EQUAL_UINT8(0, result.events.mayHaveAcknowledgedMask);
  TEST_ASSERT_EQUAL_UINT8(0, result.events.acknowledgedMask);
  TEST_ASSERT_FALSE(result.events.zLow || result.events.zHigh);
  TEST_ASSERT_EQUAL_HEX8(0x7F, bus.regs[0x1D]);
  TEST_ASSERT_EQUAL_HEX8(0x18, bus.regs[0x53]);
}

void test_event_arbitrary_bytes_decode_only_supported_source_bits() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  DeviceProfile profile = interruptProfile();
  profile.interrupts.latchEvents = false;
  interruptConfigure(driver, bus, profile);
  EventReadRequest request;
  request.embedded = true;
  request.orientation = true;
  for (uint16_t raw = 0; raw <= 255; ++raw) {
    bus.setEventSources(static_cast<uint8_t>(raw), static_cast<uint8_t>(raw),
                         static_cast<uint8_t>(raw));
    const OperationResult result = interruptRead(driver, bus, request);
    TEST_ASSERT_TRUE(result.status.ok());
    TEST_ASSERT_EQUAL(raw & 7U, result.events.dataReadyMask);
    TEST_ASSERT_EQUAL((raw & 0x10U) != 0, result.events.stepDetected);
    TEST_ASSERT_EQUAL((raw & 0x08U) != 0, result.events.stepOverflow);
    TEST_ASSERT_EQUAL((raw & 0x40U) != 0, result.events.orientationDetected);
    TEST_ASSERT_EQUAL((raw & 0x01U) != 0, result.events.xLow);
    TEST_ASSERT_EQUAL((raw & 0x02U) != 0, result.events.xHigh);
    TEST_ASSERT_EQUAL((raw & 0x04U) != 0, result.events.yLow);
    TEST_ASSERT_EQUAL((raw & 0x08U) != 0, result.events.yHigh);
    TEST_ASSERT_EQUAL((raw & 0x10U) != 0, result.events.zLow);
    TEST_ASSERT_EQUAL((raw & 0x20U) != 0, result.events.zHigh);
  }
}

void test_event_arrivals_between_reads_remain_distinct_observations() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  interruptConfigure(driver, bus, interruptProfile());
  bus.clearTrace();
  bus.setEventSources(0, 0, 1);
  EventReadRequest request;
  request.embedded = true;
  request.orientation = true;
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startEventRead(request, interruptTiming(bus), token).inProgress());
  // Explicit single-callback polling ensures each timestamp is independently
  // observed; no same-poll timestamp happens to suggest atomicity.
  driver.poll(bus.nowMs++, 1);
  driver.poll(bus.nowMs++, 1);
  driver.poll(bus.nowMs++, 1);
  bus.eventArrivalCall = bus.transferCalls + 1U;
  bus.arrivingOrientation = 0x42;
  bus.arrivingEmbedded = 0x10;
  for (uint8_t attempt = 0; attempt < 10U && driver.operationActive(); ++attempt)
    driver.poll(bus.nowMs++, 1);
  TEST_ASSERT_FALSE(driver.operationActive());
  const OperationResult result = interruptTake(driver, token);
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_EQUAL_UINT8(1, result.events.dataReadyMask);
  TEST_ASSERT_TRUE(result.events.stepDetected);
  TEST_ASSERT_TRUE(result.events.orientationDetected);
  TEST_ASSERT_TRUE(result.events.ready.readUptimeMs < result.events.embedded.readUptimeMs);
  TEST_ASSERT_TRUE(result.events.ready.readUptimeMs < result.events.orientation.readUptimeMs);
}

void test_event_fault_cancel_deadline_at_every_transfer_preserves_partial_effects() {
  for (uint32_t stop = 0; stop < 6; ++stop) {
    for (uint8_t mode = 0; mode < 4; ++mode) {
      FakeBus bus;
      Driver driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      interruptConfigure(driver, bus, interruptProfile(true));
      bus.setEventSources(0x42, 0x18);
      bus.clearTrace();
      OperationToken token;
      const OperationTiming timing = interruptTiming(bus);
      if (mode < 2) bus.addFault(stop + 1U,
          Status::Error(Err::I2C_TIMEOUT, "event source read", -733), mode == 1);
      TEST_ASSERT_TRUE(driver.startEventRead(allEvents(), timing, token).inProgress());
      if (mode < 2) interruptRun(driver, bus);
      else {
        advanceInterruptCalls(driver, bus, stop);
        const uint32_t before = bus.transferCalls;
        if (mode == 2) TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
        else driver.poll(timing.deadlineMs, 255);
        TEST_ASSERT_EQUAL_UINT32(before, bus.transferCalls);
      }
      const OperationResult result = interruptTake(driver, token);
      TEST_ASSERT_FALSE(result.status.ok());
      TEST_ASSERT_FALSE(result.events.token.valid());
      TEST_ASSERT_LESS_OR_EQUAL_UINT32(6, result.transactions);
      TEST_ASSERT_EQUAL_UINT32(mode < 2 ? stop + 1U : stop, result.transactions);
      TEST_ASSERT_EQUAL_UINT32(0, bus.timestampResetWrites);
      TEST_ASSERT_EQUAL_UINT16(0, bus.fifoDataReads);
      TEST_ASSERT_LESS_OR_EQUAL_UINT32(1, bus.orientationSourceReads);
      TEST_ASSERT_LESS_OR_EQUAL_UINT32(1, bus.embeddedSourceReads);
      if ((result.events.mayHaveAcknowledgedMask & 12U) != 0U)
        TEST_ASSERT_TRUE(result.hardwareStateMayHaveChanged);
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::KNOWN),
          static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
    }
  }
}

void test_event_callback_limits_and_active_pending_ownership_are_exact() {
  for (uint32_t limit = 1; limit < 6; ++limit) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    interruptConfigure(driver, bus, interruptProfile(true));
    bus.setEventSources(0x42, 0x18);
    bus.clearTrace();
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startEventRead(allEvents(), interruptTiming(bus), token).inProgress());
    lowerFifoTestCeiling(driver, limit);
    interruptRun(driver, bus);
    const uint32_t calls = bus.transferCalls;
    OperationToken rejected;
    TEST_ASSERT_FALSE(driver.startEventRead(allEvents(), interruptTiming(bus), rejected).inProgress());
    TEST_ASSERT_EQUAL_UINT32(calls, bus.transferCalls);
    const OperationResult result = interruptTake(driver, token);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TRANSACTION_LIMIT_EXCEEDED),
        static_cast<uint8_t>(result.status.code));
    TEST_ASSERT_EQUAL_UINT32(limit, result.transactions);
    TEST_ASSERT_EQUAL_UINT32(limit, calls);
    TEST_ASSERT_FALSE(result.events.token.valid());
  }
}

void test_event_admission_is_atomic_and_does_not_reuse_stale_source_evidence() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  OperationToken token;
  TEST_ASSERT_FALSE(driver.startEventRead({}, interruptTiming(bus), token).inProgress());
  TEST_ASSERT_EQUAL_UINT32(0, bus.transferCalls);
  interruptConfigure(driver, bus, interruptProfile());
  bus.clearTrace();
  EventReadRequest empty;
  empty.dataReady = false;
  TEST_ASSERT_FALSE(driver.startEventRead(empty, interruptTiming(bus), token).inProgress());
  TEST_ASSERT_FALSE(token.valid());
  TEST_ASSERT_EQUAL_UINT32(0, bus.transferCalls);
  bus.setEventSources(0x42, 0x18);
  EventReadRequest request;
  request.embedded = true;
  request.orientation = true;
  const OperationResult first = interruptRead(driver, bus, request);
  TEST_ASSERT_TRUE(first.status.ok());
  bus.clearTrace();
  bus.addFault(1, Status::Error(Err::I2C_NACK_ADDR, "next event read"));
  const OperationResult failed = interruptRead(driver, bus, request);
  TEST_ASSERT_FALSE(failed.status.ok());
  TEST_ASSERT_FALSE(failed.events.token.valid());
  TEST_ASSERT_EQUAL_UINT8(0, failed.events.readMask);
  TEST_ASSERT_FALSE(failed.events.orientation.valid);
  TEST_ASSERT_FALSE(failed.events.embedded.valid);
  TEST_ASSERT_FALSE(failed.events.orientationDetected);
}

void test_interrupt_configure_faults_cancel_deadlines_cover_every_new_phase() {
  const DeviceProfile profile = fullInterruptProfile();
  uint32_t total = 0;
  {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    total = interruptConfigure(driver, bus, profile).transactions;
    TEST_ASSERT_EQUAL_UINT32(106U, total);
  }
  for (uint32_t stop = 0; stop < total; ++stop) {
    for (uint8_t mode = 0; mode < 4; ++mode) {
      FakeBus bus;
      Driver driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      bus.setEventSources(0x42, 0x18);
      if (mode < 2) bus.addFault(stop + 1U,
          Status::Error(Err::I2C_TIMEOUT, "interrupt configure phase", -744), mode == 1);
      OperationToken token;
      const OperationTiming timing = interruptTiming(bus);
      TEST_ASSERT_TRUE(driver.startConfigure(profile, timing, token).inProgress());
      if (mode < 2) interruptRun(driver, bus);
      else {
        advanceInterruptCalls(driver, bus, stop);
        const uint32_t before = bus.transferCalls;
        if (mode == 2) TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
        else driver.poll(timing.deadlineMs, 255);
        TEST_ASSERT_EQUAL_UINT32(before, bus.transferCalls);
      }
      const OperationResult result = interruptTake(driver, token);
      TEST_ASSERT_FALSE(result.status.ok());
      TEST_ASSERT_EQUAL_UINT32(mode < 2 ? stop + 1U : stop, result.transactions);
      TEST_ASSERT_EQUAL_UINT32(0, driver.configGeneration());
      TEST_ASSERT_EQUAL_UINT32(0, bus.orientationSourceReads);
      TEST_ASSERT_EQUAL_UINT32(0, bus.embeddedSourceReads);
    }
  }
}

void test_interrupt_reconcile_is_read_only_and_names_exact_route_mismatch() {
  const uint8_t registers[] = {0x12, 0x13, 0x0B, 0x0D, 0x0E, 0x58, 0x59, 0x5E, 0x5F};
  for (uint8_t reg : registers) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    interruptConfigure(driver, bus, fullInterruptProfile());
    const uint32_t resets = bus.timestampResetWrites;
    const uint8_t expected = bus.regs[reg];
    bus.regs[reg] ^= 1U;
    bus.setEventSources(0x42, 0x18);
    bus.clearTrace();
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startReconcile(interruptTiming(bus), token).inProgress());
    interruptRun(driver, bus);
    const OperationResult result = interruptTake(driver, token);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CONFIGURATION_MISMATCH),
        static_cast<uint8_t>(result.status.code));
    TEST_ASSERT_EQUAL_HEX8(reg, result.configuration.mismatchRegister);
    TEST_ASSERT_EQUAL_HEX8(expected, result.configuration.expectedValue);
    TEST_ASSERT_EQUAL_HEX8(expected ^ 1U, result.configuration.observedValue);
    TEST_ASSERT_EQUAL_UINT32(0, bus.writeCalls);
    TEST_ASSERT_EQUAL_UINT32(0, bus.orientationSourceReads);
    TEST_ASSERT_EQUAL_UINT32(0, bus.embeddedSourceReads);
    TEST_ASSERT_EQUAL_UINT32(resets, bus.timestampResetWrites);
  }
}

void test_interrupt_raw_route_masks_cannot_enable_unsupported_engines() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  interruptConfigure(driver, bus, interruptProfile());
  bus.clearTrace();
  for (uint8_t reg : {0x0D, 0x0E, 0x5E, 0x5F}) {
    for (uint16_t value = 1; value <= 255; ++value) {
      TEST_ASSERT_FALSE(driver.diagnosticWriteRegister(reg, static_cast<uint8_t>(value), bus.nowMs).ok());
      TEST_ASSERT_EQUAL_UINT32(0, bus.transferCalls);
    }
  }
}

void test_timestamp_acknowledgement_uses_explicit_counter_reset_not_event_read() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  interruptConfigure(driver, bus, interruptProfile());
  bus.setCounters(0xFF1234, 0xABCD);
  bus.timestampEndAsserted = true;
  bus.setEventSources(0, 0);
  bus.clearTrace();
  EventReadRequest request;
  request.embedded = true;
  request.orientation = true;
  const OperationResult read = interruptRead(driver, bus, request);
  TEST_ASSERT_TRUE(read.status.ok());
  TEST_ASSERT_TRUE(read.events.timestampRouted);
  TEST_ASSERT_TRUE(bus.timestampEndAsserted);
  TEST_ASSERT_EQUAL_UINT32(0, bus.timestampResetWrites);
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startCounterReset({true, false}, interruptTiming(bus), token).inProgress());
  interruptRun(driver, bus);
  const OperationResult reset = interruptTake(driver, token);
  TEST_ASSERT_TRUE(reset.status.ok());
  TEST_ASSERT_TRUE(reset.counters.timestampResetConfirmed);
  TEST_ASSERT_TRUE(reset.counters.timestampResetMayHaveOccurred);
  TEST_ASSERT_TRUE(reset.counters.epoch > read.counters.epoch);
  TEST_ASSERT_TRUE(reset.counters.continuityLost);
  TEST_ASSERT_FALSE(bus.timestampEndAsserted);
  TEST_ASSERT_EQUAL_HEX32(0, bus.timestampTicks);
  TEST_ASSERT_EQUAL_HEX16(0xABCD, bus.stepCount);
}

void test_interrupt_disabled_configuration_clears_every_owned_route_and_mode() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  interruptConfigure(driver, bus, interruptProfile());
  bus.clearTrace();
  interruptConfigure(driver, bus, DeviceProfile{});
  for (uint8_t reg : {0x0B, 0x0D, 0x0E, 0x58, 0x59, 0x5E, 0x5F})
    TEST_ASSERT_EQUAL_HEX8(0, bus.regs[reg]);
  TEST_ASSERT_EQUAL_HEX8(0x44, bus.regs[0x12]);
  TEST_ASSERT_EQUAL_HEX8(0, bus.regs[0x13]);
}

void test_interrupt_reset_boot_recovery_restore_routes_without_source_acknowledgement() {
  for (uint8_t operation = 0; operation < 3; ++operation) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    const DeviceProfile profile = interruptProfile();
    interruptConfigure(driver, bus, profile);
    bus.setEventSources(0x42, 0x18);
    bus.resetRegistersOnSoftwareReset = true;
    if (operation == 1U) bus.bootBitReadsRemaining = 15;
    else bus.resetBitReadsRemaining = 15;
    bus.clearTrace();
    OperationToken token;
    const Status started = operation == 0U ? driver.startReset(interruptTiming(bus), token) :
                           operation == 1U ? driver.startBoot(interruptTiming(bus), token) :
                                             driver.startRecover(interruptTiming(bus), token);
    TEST_ASSERT_TRUE(started.inProgress());
    interruptRun(driver, bus);
    const OperationResult result = interruptTake(driver, token);
    TEST_ASSERT_TRUE_MESSAGE(result.status.ok(), result.status.msg);
    TEST_ASSERT_TRUE(result.events.historyMayBeLost);
    TEST_ASSERT_EQUAL_HEX8(0x80, bus.regs[0x0D]);
    TEST_ASSERT_EQUAL_HEX8(0x40, bus.regs[0x0E]);
    TEST_ASSERT_EQUAL_HEX8(0x05, bus.regs[0x5E]);
    TEST_ASSERT_EQUAL_HEX8(0, bus.regs[0x5F]);
    TEST_ASSERT_EQUAL_HEX8(0x81, bus.regs[0x58]);
    TEST_ASSERT_EQUAL_HEX8(0x34, bus.regs[0x19]);
    TEST_ASSERT_EQUAL_UINT32(0, bus.orientationSourceReads);
    TEST_ASSERT_EQUAL_UINT32(0, bus.embeddedSourceReads);
    const size_t firstMode = interruptFindWrite(bus, 0x11, 0);
    for (uint8_t reg : {0x0D, 0x0E, 0x5E, 0x5F})
      TEST_ASSERT_TRUE(interruptFindWrite(bus, reg, 0) < firstMode);
  }
}

void test_interrupt_power_down_masks_routes_and_calibration_preserves_them() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  interruptConfigure(driver, bus, interruptProfile());
  bus.setEventSources(0x42, 0x18);
  bus.clearTrace();
  OperationToken token;
  CalibrationRequest calibration;
  calibration.samples = 1;
  TEST_ASSERT_TRUE(driver.startCalibration(calibration, interruptTiming(bus), token).inProgress());
  interruptRun(driver, bus);
  const OperationResult calibrated = interruptTake(driver, token);
  TEST_ASSERT_TRUE(calibrated.status.ok());
  TEST_ASSERT_EQUAL_UINT32(0, bus.writeCalls);
  TEST_ASSERT_EQUAL_HEX8(0x80, bus.regs[0x0D]);
  TEST_ASSERT_EQUAL_HEX8(0x40, bus.regs[0x0E]);
  TEST_ASSERT_EQUAL_UINT32(0, bus.orientationSourceReads);
  TEST_ASSERT_EQUAL_UINT32(0, bus.embeddedSourceReads);
  bus.clearTrace();
  TEST_ASSERT_TRUE(driver.startPowerDown(interruptTiming(bus), token).inProgress());
  interruptRun(driver, bus);
  const OperationResult stopped = interruptTake(driver, token);
  TEST_ASSERT_TRUE(stopped.status.ok());
  TEST_ASSERT_TRUE(stopped.events.historyMayBeLost);
  TEST_ASSERT_EQUAL_HEX8(0, bus.regs[0x10]);
  TEST_ASSERT_EQUAL_HEX8(0, bus.regs[0x11]);
  for (uint8_t reg : {0x0D, 0x0E, 0x5E, 0x5F}) TEST_ASSERT_EQUAL_HEX8(0, bus.regs[reg]);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNCONFIGURED),
      static_cast<uint8_t>(stopped.configuration.state));
}

void test_interrupt_self_test_masks_before_temporary_writes_and_restores_exact_image() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  interruptConfigure(driver, bus, interruptProfile());
  bus.setEventSources(0x42, 0x18);
  bus.clearTrace();
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startSelfTest({5}, interruptTiming(bus), token).inProgress());
  interruptRun(driver, bus);
  const OperationResult result = interruptTake(driver, token);
  TEST_ASSERT_TRUE_MESSAGE(result.status.ok(), result.status.msg);
  TEST_ASSERT_TRUE(result.selfTest.restorationStatus.ok());
  TEST_ASSERT_TRUE(result.events.historyMayBeLost);
  const size_t firstTest = interruptFindWrite(bus, 0x14, 0);
  for (uint8_t reg : {0x0D, 0x0E, 0x5E, 0x5F})
    TEST_ASSERT_TRUE(interruptFindWrite(bus, reg, 0) < firstTest);
  TEST_ASSERT_EQUAL_HEX8(0x80, bus.regs[0x0D]);
  TEST_ASSERT_EQUAL_HEX8(0x40, bus.regs[0x0E]);
  TEST_ASSERT_EQUAL_HEX8(0x05, bus.regs[0x5E]);
  TEST_ASSERT_EQUAL_HEX8(0x81, bus.regs[0x58]);
  TEST_ASSERT_EQUAL_HEX8(0x34, bus.regs[0x19]);
  TEST_ASSERT_EQUAL_UINT32(0, bus.orientationSourceReads);
  TEST_ASSERT_EQUAL_UINT32(0, bus.embeddedSourceReads);
}

void test_interrupt_power_down_fail_cancel_deadline_cover_masking_and_sensor_phases() {
  for (uint32_t stop = 0; stop < 15; ++stop) {
    for (uint8_t mode = 0; mode < 4; ++mode) {
      FakeBus bus;
      Driver driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      interruptConfigure(driver, bus, interruptProfile());
      bus.setEventSources(0x42, 0x18);
      bus.clearTrace();
      if (mode < 2) bus.addFault(stop + 1U,
          Status::Error(Err::I2C_TIMEOUT, "power down interrupt phase", -745), mode == 1);
      OperationToken token;
      const OperationTiming timing = interruptTiming(bus);
      TEST_ASSERT_TRUE(driver.startPowerDown(timing, token).inProgress());
      if (mode < 2) interruptRun(driver, bus);
      else {
        advanceInterruptCalls(driver, bus, stop);
        const uint32_t before = bus.transferCalls;
        if (mode == 2) TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
        else driver.poll(timing.deadlineMs, 255);
        TEST_ASSERT_EQUAL_UINT32(before, bus.transferCalls);
      }
      const OperationResult result = interruptTake(driver, token);
      TEST_ASSERT_FALSE(result.status.ok());
      TEST_ASSERT_EQUAL_UINT32(mode < 2 ? stop + 1U : stop, result.transactions);
      TEST_ASSERT_EQUAL_UINT32(0, bus.orientationSourceReads);
      TEST_ASSERT_EQUAL_UINT32(0, bus.embeddedSourceReads);
    }
  }
}

void test_interrupt_self_test_restoration_fault_cancel_deadline_cover_every_phase() {
  uint32_t restoreFirst = 0;
  uint32_t total = 0;
  {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    interruptConfigure(driver, bus, interruptProfile());
    bus.clearTrace();
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startSelfTest({5}, interruptTiming(bus), token).inProgress());
    interruptRun(driver, bus);
    const OperationResult reference = interruptTake(driver, token);
    TEST_ASSERT_TRUE(reference.status.ok());
    total = reference.transactions;
    for (size_t index = 0; index < bus.traceCount && restoreFirst == 0; ++index)
      if (bus.trace[index].kind == TransferKind::WRITE_READ &&
          bus.trace[index].startReg == 0x5C) restoreFirst = static_cast<uint32_t>(index + 1U);
    TEST_ASSERT_TRUE(restoreFirst > 8U);
    TEST_ASSERT_TRUE(restoreFirst < total);
  }
  for (uint32_t stop = restoreFirst - 1U; stop < total; ++stop) {
    for (uint8_t mode = 0; mode < 4; ++mode) {
      FakeBus bus;
      Driver driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      interruptConfigure(driver, bus, interruptProfile());
      bus.setEventSources(0x42, 0x18);
      bus.clearTrace();
      if (mode < 2) bus.addFault(stop + 1U,
          Status::Error(Err::I2C_TIMEOUT, "interrupt self-test restoration", -746), mode == 1);
      OperationToken token;
      const OperationTiming timing = interruptTiming(bus);
      TEST_ASSERT_TRUE(driver.startSelfTest({5}, timing, token).inProgress());
      if (mode < 2) interruptRun(driver, bus);
      else {
        advanceInterruptCalls(driver, bus, stop);
        const uint32_t before = bus.transferCalls;
        if (mode == 2) TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
        else driver.poll(timing.deadlineMs, 255);
        TEST_ASSERT_EQUAL_UINT32(before, bus.transferCalls);
      }
      const OperationResult result = interruptTake(driver, token);
      TEST_ASSERT_FALSE(result.status.ok());
      TEST_ASSERT_FALSE(result.selfTest.restorationStatus.ok());
      TEST_ASSERT_LESS_OR_EQUAL_UINT32(maximumSelfTestTransactions(5), result.transactions);
      TEST_ASSERT_EQUAL_UINT32(mode < 2 ? stop + 1U : stop, bus.transferCalls);
      TEST_ASSERT_TRUE(result.events.historyMayBeLost);
      TEST_ASSERT_EQUAL_UINT32(0, bus.orientationSourceReads);
      TEST_ASSERT_EQUAL_UINT32(0, bus.embeddedSourceReads);
    }
  }
}

void test_interrupt_reset_boot_recovery_route_phases_never_retry_or_hide_effects() {
  for (uint8_t operation = 0; operation < 3; ++operation) {
    uint32_t calls[32] = {};
    uint8_t count = 0;
    {
      FakeBus bus;
      Driver driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      interruptConfigure(driver, bus, interruptProfile());
      bus.clearTrace();
      OperationToken token;
      const Status start = operation == 0U ? driver.startReset(interruptTiming(bus), token) :
                           operation == 1U ? driver.startBoot(interruptTiming(bus), token) :
                                             driver.startRecover(interruptTiming(bus), token);
      TEST_ASSERT_TRUE(start.inProgress());
      interruptRun(driver, bus);
      TEST_ASSERT_TRUE(interruptTake(driver, token).status.ok());
      for (size_t index = 0; index < bus.traceCount; ++index) {
        const uint8_t reg = bus.trace[index].startReg;
        if (reg == 0x0D || reg == 0x0E || reg == 0x5E || reg == 0x5F) {
          TEST_ASSERT_LESS_THAN_UINT8(32, count);
          calls[count++] = static_cast<uint32_t>(index + 1U);
        }
      }
      // Eight safe masking callbacks, eight base-image callbacks, eight final
      // route callbacks. No implicit acknowledgement reads are involved.
      TEST_ASSERT_EQUAL_UINT8(24, count);
    }
    for (uint8_t phase = 0; phase < count; ++phase) {
      const uint32_t stop = calls[phase] - 1U;
      for (uint8_t mode = 0; mode < 4; ++mode) {
        FakeBus bus;
        Driver driver;
        TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
        interruptConfigure(driver, bus, interruptProfile());
        bus.setEventSources(0x42, 0x18);
        bus.clearTrace();
        if (mode < 2) bus.addFault(stop + 1U,
            Status::Error(Err::I2C_TIMEOUT, "interrupt lifecycle route phase", -747), mode == 1);
        OperationToken token;
        const OperationTiming timing = interruptTiming(bus);
        const Status start = operation == 0U ? driver.startReset(timing, token) :
                             operation == 1U ? driver.startBoot(timing, token) :
                                               driver.startRecover(timing, token);
        TEST_ASSERT_TRUE(start.inProgress());
        if (mode < 2) interruptRun(driver, bus);
        else {
          advanceInterruptCalls(driver, bus, stop);
          const uint32_t before = bus.transferCalls;
          if (mode == 2) TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
          else driver.poll(timing.deadlineMs, 255);
          TEST_ASSERT_EQUAL_UINT32(before, bus.transferCalls);
        }
        const OperationResult result = interruptTake(driver, token);
        TEST_ASSERT_FALSE(result.status.ok());
        TEST_ASSERT_EQUAL_UINT32(mode < 2 ? stop + 1U : stop, bus.transferCalls);
        TEST_ASSERT_EQUAL_UINT32(0, bus.orientationSourceReads);
        TEST_ASSERT_EQUAL_UINT32(0, bus.embeddedSourceReads);
      }
    }
  }
}

void test_interrupt_power_down_proves_main_bank_at_its_exact_ceiling() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  interruptConfigure(driver, bus, interruptProfile());
  bus.regs[0x01] = 0x80;
  bus.clearTrace();
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startPowerDown(interruptTiming(bus), token).inProgress());
  interruptRun(driver, bus);
  const OperationResult result = interruptTake(driver, token);
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_EQUAL_UINT32(17, result.transactions);
  TEST_ASSERT_EQUAL_UINT32(MAX_POWER_DOWN_TRANSACTIONS, result.transactions);
  TEST_ASSERT_FALSE(result.registerBankMayBeUnknown);
  TEST_ASSERT_EQUAL_HEX8(0, bus.regs[0x01]);
}

void test_event_zero_budget_and_all_active_pending_entry_points_are_bus_silent() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  interruptConfigure(driver, bus, interruptProfile(true));
  bus.setEventSources(0x42, 0x18);
  bus.clearTrace();
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startEventRead(allEvents(), interruptTiming(bus), token).inProgress());
  const auto assertExcluded = [&driver, &bus]() {
    const uint32_t calls = bus.transferCalls;
    OperationToken rejected;
    TEST_ASSERT_FALSE(driver.startProbe(interruptTiming(bus), rejected).inProgress());
    TEST_ASSERT_FALSE(driver.startConfigure(interruptProfile(), interruptTiming(bus), rejected).inProgress());
    TEST_ASSERT_FALSE(driver.startSample({}, interruptTiming(bus), rejected).inProgress());
    TEST_ASSERT_FALSE(driver.startEventRead(allEvents(), interruptTiming(bus), rejected).inProgress());
    TEST_ASSERT_FALSE(driver.startReset(interruptTiming(bus), rejected).inProgress());
    TEST_ASSERT_FALSE(driver.startBoot(interruptTiming(bus), rejected).inProgress());
    TEST_ASSERT_FALSE(driver.startRecover(interruptTiming(bus), rejected).inProgress());
    TEST_ASSERT_FALSE(driver.startReconcile(interruptTiming(bus), rejected).inProgress());
    TEST_ASSERT_FALSE(driver.startPowerDown(interruptTiming(bus), rejected).inProgress());
    TEST_ASSERT_FALSE(driver.startCounterReset({}, interruptTiming(bus), rejected).inProgress());
    TEST_ASSERT_FALSE(driver.startSelfTest({5}, interruptTiming(bus), rejected).inProgress());
    TEST_ASSERT_FALSE(driver.startCalibration({}, interruptTiming(bus), rejected).inProgress());
    TEST_ASSERT_FALSE(driver.startFifoPurge({1}, interruptTiming(bus), rejected).inProgress());
    FifoRecord record;
    TEST_ASSERT_FALSE(driver.startFifoRead({&record, 1, 1}, interruptTiming(bus), rejected).inProgress());
    uint8_t raw = 0;
    TEST_ASSERT_FALSE(driver.diagnosticReadRegister(0x1D, raw, bus.nowMs).ok());
    TEST_ASSERT_FALSE(driver.diagnosticWriteRegister(0x5E, 0, bus.nowMs).ok());
    TEST_ASSERT_EQUAL_UINT32(calls, bus.transferCalls);
  };
  for (uint8_t stage = 0; stage < 6; ++stage) {
    const uint32_t calls = bus.transferCalls;
    const PollResult idle = driver.poll(bus.nowMs++, 0);
    TEST_ASSERT_EQUAL_UINT8(0, idle.transactionsUsed);
    TEST_ASSERT_EQUAL_UINT32(calls, bus.transferCalls);
    assertExcluded();
    driver.poll(bus.nowMs++, 1);
  }
  if (driver.operationActive()) driver.poll(bus.nowMs++, 1);
  TEST_ASSERT_TRUE(driver.resultPending());
  assertExcluded();
  TEST_ASSERT_TRUE(interruptTake(driver, token).status.ok());
  const OperationResult next = interruptRead(driver, bus);
  TEST_ASSERT_TRUE(next.status.ok());
  TEST_ASSERT_FALSE(next.events.orientationDetected);
  TEST_ASSERT_FALSE(next.events.stepDetected || next.events.stepOverflow);
}

void test_event_terminal_publication_needs_no_transaction_budget() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  interruptConfigure(driver, bus, DeviceProfile{});
  bus.clearTrace();
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startEventRead({}, interruptTiming(bus), token).inProgress());
  for (uint8_t call = 0; call < 3U; ++call) {
    const PollResult progress = driver.poll(bus.nowMs++, 1);
    TEST_ASSERT_EQUAL_UINT8(1, progress.transactionsUsed);
  }
  TEST_ASSERT_TRUE(driver.operationActive());
  const uint32_t calls = bus.transferCalls;
  const PollResult terminal = driver.poll(bus.nowMs, 0);
  TEST_ASSERT_EQUAL_UINT32(calls, bus.transferCalls);
  TEST_ASSERT_EQUAL_UINT8(0, terminal.transactionsUsed);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(OperationState::SUCCEEDED),
                         static_cast<uint8_t>(terminal.state));
  const OperationResult result = interruptTake(driver, token);
  TEST_ASSERT_EQUAL_UINT8(EVENT_DATA_READY, result.events.readMask);
  TEST_ASSERT_EQUAL_UINT64(token.value, result.events.token.value);
}

void test_event_observed_fifo_loss_survives_a_later_cleared_status() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  interruptConfigure(driver, bus, interruptProfile(true));
  bus.clearTrace();
  bus.setFifo(3, 0, 0x1234, true);
  EventReadRequest request;
  request.dataReady = false;
  request.fifo = true;
  const OperationResult observed = interruptRead(driver, bus, request);
  TEST_ASSERT_TRUE(observed.events.fifoOverrun);
  TEST_ASSERT_TRUE(observed.events.historyMayBeLost);
  TEST_ASSERT_FALSE(observed.hardwareStateMayHaveChanged);
  TEST_ASSERT_EQUAL_UINT16(0, bus.fifoDataReads);
  bus.setFifo(3, 0, 0x1234, false);
  bus.fifoPatternWords = 3;
  FifoRecord record;
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startFifoRead({&record, 1, 3}, interruptTiming(bus), token).inProgress());
  interruptRun(driver, bus);
  const OperationResult batch = interruptTake(driver, token);
  TEST_ASSERT_TRUE(batch.status.ok());
  TEST_ASSERT_TRUE(batch.fifoRead.continuityLost);
  TEST_ASSERT_FALSE(batch.fifoRead.overrunObserved);
  TEST_ASSERT_EQUAL_UINT64(observed.counters.epoch, batch.counters.epoch);
}

void test_diagnostic_auxiliary_pad_routes_cannot_bypass_typed_sources() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  const uint8_t registers[] = {0x0B, 0x13, 0x1A};
  const uint8_t forbidden[] = {0x01, 0x10, 0x80};
  for (uint8_t i = 0; i < 3U; ++i) {
    for (uint16_t raw = 0; raw <= 255U; ++raw) {
      if ((raw & forbidden[i]) == 0U) continue;
      const uint32_t calls = bus.transferCalls;
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
          static_cast<uint8_t>(driver.diagnosticWriteRegister(registers[i],
              static_cast<uint8_t>(raw), bus.nowMs).code));
      TEST_ASSERT_EQUAL_UINT32(calls, bus.transferCalls);
    }
  }
  TEST_ASSERT_TRUE(driver.diagnosticWriteRegister(0x0B, 0x80, bus.nowMs).ok());
  TEST_ASSERT_TRUE(driver.diagnosticWriteRegister(0x13, 0x20, bus.nowMs).ok());
  TEST_ASSERT_TRUE(driver.diagnosticWriteRegister(0x1A, 0, bus.nowMs).ok());
}

}  // namespace

void runInterruptTests() {
  RUN_TEST(test_interrupt_route_pin_matrix_has_independent_wire_images);
  RUN_TEST(test_interrupt_invalid_profiles_are_total_and_bus_silent);
  RUN_TEST(test_interrupt_complete_image_composes_shared_fields_and_thresholds);
  RUN_TEST(test_interrupt_configure_masks_before_dependencies_and_activates_after_settling);
  RUN_TEST(test_event_reads_keep_independent_times_raw_sources_and_decoded_causes);
  RUN_TEST(test_event_nonlatched_and_4d_sources_never_claim_unavailable_evidence);
  RUN_TEST(test_event_arbitrary_bytes_decode_only_supported_source_bits);
  RUN_TEST(test_event_arrivals_between_reads_remain_distinct_observations);
  RUN_TEST(test_event_fault_cancel_deadline_at_every_transfer_preserves_partial_effects);
  RUN_TEST(test_event_callback_limits_and_active_pending_ownership_are_exact);
  RUN_TEST(test_event_admission_is_atomic_and_does_not_reuse_stale_source_evidence);
  RUN_TEST(test_interrupt_configure_faults_cancel_deadlines_cover_every_new_phase);
  RUN_TEST(test_interrupt_reconcile_is_read_only_and_names_exact_route_mismatch);
  RUN_TEST(test_interrupt_raw_route_masks_cannot_enable_unsupported_engines);
  RUN_TEST(test_timestamp_acknowledgement_uses_explicit_counter_reset_not_event_read);
  RUN_TEST(test_interrupt_disabled_configuration_clears_every_owned_route_and_mode);
  RUN_TEST(test_interrupt_reset_boot_recovery_restore_routes_without_source_acknowledgement);
  RUN_TEST(test_interrupt_power_down_masks_routes_and_calibration_preserves_them);
  RUN_TEST(test_interrupt_self_test_masks_before_temporary_writes_and_restores_exact_image);
  RUN_TEST(test_interrupt_power_down_fail_cancel_deadline_cover_masking_and_sensor_phases);
  RUN_TEST(test_interrupt_self_test_restoration_fault_cancel_deadline_cover_every_phase);
  RUN_TEST(test_interrupt_reset_boot_recovery_route_phases_never_retry_or_hide_effects);
  RUN_TEST(test_interrupt_power_down_proves_main_bank_at_its_exact_ceiling);
  RUN_TEST(test_event_zero_budget_and_all_active_pending_entry_points_are_bus_silent);
  RUN_TEST(test_event_terminal_publication_needs_no_transaction_budget);
  RUN_TEST(test_event_observed_fifo_loss_survives_a_later_cleared_status);
  RUN_TEST(test_diagnostic_auxiliary_pad_routes_cannot_bypass_typed_sources);
}
