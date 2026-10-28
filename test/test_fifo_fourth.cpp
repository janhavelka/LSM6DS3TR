/// @file test_fifo_fourth.cpp
/// @brief Independent fourth-dataset wire fixtures, counter lifetimes and bounded faults.
#include <unity.h>

#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <limits>

#include "LSM6DS3TR/LSM6DS3TR.h"
#include "support/FakeBus.h"

using namespace LSM6DS3TR;
using namespace LSM6DS3TRTest;

void lowerFifoTestCeiling(LSM6DS3TR::LSM6DS3TR& driver, uint32_t limit);

namespace {

using Driver = LSM6DS3TR::LSM6DS3TR;

DeviceProfile fourthProfile(FifoFourthDataset source = FifoFourthDataset::TIMESTAMP_STEP,
                            FifoMode mode = FifoMode::CONTINUOUS) {
  DeviceProfile profile;
  profile.fifo.enabled = true;
  profile.fifo.mode = mode;
  profile.fifo.odr = Odr::HZ_104;
  profile.fifo.fourthDataset = source;
  profile.fifo.fourthDecimation = FifoDecimation::EVERY_1;
  return profile;
}

OperationTiming fourthTiming(const FakeBus& bus) {
  return {bus.nowMs, bus.nowMs + 30000U};
}

void fourthRun(Driver& driver, FakeBus& bus) {
  const uint8_t budgets[] = {0, 1, 255, 0, 2, 7};
  for (uint32_t poll = 0; poll < 40000U && driver.operationActive(); ++poll) {
    const uint8_t budget = budgets[poll % 6U];
    const uint32_t before = bus.transferCalls;
    const PollResult result = driver.poll(bus.nowMs++, budget);
    TEST_ASSERT_EQUAL_UINT32(bus.transferCalls - before, result.transactionsUsed);
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(budget, result.transactionsUsed);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(result.transactionLimit, result.transactions);
  }
  TEST_ASSERT_FALSE(driver.operationActive());
  TEST_ASSERT_TRUE(driver.resultPending());
  TEST_ASSERT_EQUAL_UINT16(0, bus.fifoEmptyReadAttempts);
}

OperationResult fourthTake(Driver& driver, OperationToken token) {
  OperationResult result;
  TEST_ASSERT_TRUE(driver.takeResult(token, result).ok());
  TEST_ASSERT_EQUAL_UINT64(token.value, result.token.value);
  OperationResult second;
  TEST_ASSERT_FALSE(driver.takeResult(token, second).ok());
  return result;
}

OperationResult fourthConfigure(Driver& driver, FakeBus& bus, const DeviceProfile& profile) {
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startConfigure(profile, fourthTiming(bus), token).inProgress());
  fourthRun(driver, bus);
  const OperationResult result = fourthTake(driver, token);
  TEST_ASSERT_TRUE_MESSAGE(result.status.ok(), result.status.msg);
  bus.nowMs = result.configuration.validAfterUptimeMs > bus.nowMs
      ? result.configuration.validAfterUptimeMs : bus.nowMs;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::KNOWN),
      static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
  return result;
}

OperationResult fourthRead(Driver& driver, FakeBus& bus, FifoRecord* records,
                           uint16_t capacity, uint16_t maximumWords) {
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startFifoRead({records, capacity, maximumWords},
                                       fourthTiming(bus), token).inProgress());
  fourthRun(driver, bus);
  return fourthTake(driver, token);
}

size_t fourthFindWrite(const FakeBus& bus, uint8_t reg, uint8_t value, size_t first = 0) {
  for (size_t i = first; i < bus.traceCount; ++i)
    if (bus.trace[i].kind == TransferKind::WRITE && bus.trace[i].startReg == reg &&
        bus.trace[i].firstValue == value) return i;
  return bus.traceCount;
}

void fourthAdvanceToCalls(Driver& driver, FakeBus& bus, uint32_t calls) {
  for (uint32_t polls = 0; polls < 40000U && driver.operationActive() &&
       bus.transferCalls < calls; ++polls) driver.poll(bus.nowMs++, 1);
  TEST_ASSERT_EQUAL_UINT32(calls, bus.transferCalls);
}

void test_fourth_profile_invalid_combinations_are_bus_silent_and_atomic() {
  DeviceProfile candidates[14];
  for (DeviceProfile& profile : candidates) profile = fourthProfile();
  candidates[0].fifo.fourthDataset = static_cast<FifoFourthDataset>(0xFF);
  candidates[1].fifo.fourthDecimation = FifoDecimation::OFF;
  candidates[2].fifo.fourthDecimation = static_cast<FifoDecimation>(8);
  candidates[3].fifo.trigger = static_cast<FifoTrigger>(2);
  candidates[4].fifo.timestampResolution = static_cast<TimestampResolution>(2);
  candidates[5].accelOdr = Odr::POWER_DOWN;
  candidates[6].accelOdr = Odr::HZ_12_5;
  candidates[7].fifo.enabled = false;
  candidates[8].fifo.fourthDataset = FifoFourthDataset::OFF;
  candidates[9].fifo.fourthDataset = FifoFourthDataset::TEMPERATURE;
  candidates[9].fifo.trigger = FifoTrigger::STEP_EVENT;
  candidates[10].fifo.fourthDataset = FifoFourthDataset::TEMPERATURE;
  candidates[10].accelOdr = Odr::POWER_DOWN;
  candidates[10].gyroOdr = Odr::POWER_DOWN;
  candidates[11].fifo.trigger = FifoTrigger::STEP_EVENT;
  candidates[11].fifo.fourthDataset = FifoFourthDataset::OFF;
  candidates[11].fifo.fourthDecimation = FifoDecimation::OFF;
  candidates[12].fifo.odr = Odr::POWER_DOWN;
  candidates[13].fifo.fourthDataset = FifoFourthDataset::TEMPERATURE;
  candidates[13].fifo.timestampResolution = TimestampResolution::US_25;
  for (const DeviceProfile& candidate : candidates) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    OperationToken token{123};
    TEST_ASSERT_FALSE(validateProfile(candidate).ok());
    TEST_ASSERT_FALSE(driver.startConfigure(candidate, fourthTiming(bus), token).inProgress());
    TEST_ASSERT_FALSE(token.valid());
    TEST_ASSERT_EQUAL_UINT32(0, bus.transferCalls);
    DeviceProfile desired;
    TEST_ASSERT_FALSE(driver.getDesiredProfile(desired).ok());
  }
}

void test_fourth_register_image_policy_and_resolution_reset_sequence() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  bus.bankA[0x0F] = 0xFF;
  bus.bankA[0x14] = 0xFF;
  bus.bankA[0x15] = 0xFF;
  bus.setCounters(0x123456, 0xABCD);
  DeviceProfile profile = fourthProfile();
  profile.fifo.timestampResolution = TimestampResolution::US_25;
  profile.fifo.trigger = FifoTrigger::STEP_EVENT;
  profile.fifo.odr = Odr::POWER_DOWN;
  const OperationResult result = fourthConfigure(driver, bus, profile);
  TEST_ASSERT_EQUAL_HEX8(0xC0, bus.regs[cmd::REG_FIFO_CTRL2]);
  TEST_ASSERT_EQUAL_HEX8(0x08, bus.regs[cmd::REG_FIFO_CTRL4]);
  TEST_ASSERT_EQUAL_HEX8(0x34, bus.regs[cmd::REG_CTRL10_C]);
  TEST_ASSERT_EQUAL_HEX8(0x10, bus.regs[cmd::REG_WAKE_UP_DUR]);
  TEST_ASSERT_EQUAL_HEX8(0x10, bus.bankA[0x0F]);
  TEST_ASSERT_EQUAL_HEX8(0x6E, bus.bankA[0x14]);
  TEST_ASSERT_EQUAL_HEX8(0, bus.bankA[0x15]);
  TEST_ASSERT_EQUAL_HEX8(0, bus.regs[cmd::REG_FUNC_CFG_ACCESS]);
  TEST_ASSERT_EQUAL_UINT32(0, bus.bankWritesWhileSensorsActive);
  TEST_ASSERT_EQUAL_UINT32(1, bus.timestampResetWrites);
  TEST_ASSERT_EQUAL_UINT32(0, bus.stepResetAssertions);
  TEST_ASSERT_EQUAL_HEX16(0xABCD, bus.stepCount);
  const size_t resolution = fourthFindWrite(bus, cmd::REG_WAKE_UP_DUR, 0x10);
  const size_t reset = fourthFindWrite(bus, cmd::REG_TIMESTAMP2, 0xAA);
  const size_t enable = fourthFindWrite(bus, cmd::REG_CTRL10_C, 0x34);
  TEST_ASSERT_TRUE(resolution < reset && reset < enable && enable < bus.traceCount);
  TEST_ASSERT_TRUE(result.counters.timestampResetMayHaveOccurred);
  TEST_ASSERT_TRUE(result.counters.timestampResetConfirmed);
  TEST_ASSERT_FALSE(result.counters.stepResetMayHaveOccurred);
  TEST_ASSERT_GREATER_THAN_UINT64(0, result.counters.epoch);
  TEST_ASSERT_TRUE(result.counters.continuityLost);
}

void test_fourth_temperature_pdf_table74_uses_only_middle_signed_pair() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  fourthConfigure(driver, bus, fourthProfile(FifoFourthDataset::TEMPERATURE));
  // Table74 byte positions0/1 and4/5 are unused, deliberately nonzero.
  const int16_t words[] = {static_cast<int16_t>(0xBBAA), -32768,
      static_cast<int16_t>(0xDDCC), static_cast<int16_t>(0x2718), 32767,
      static_cast<int16_t>(0x3654), static_cast<int16_t>(0x0001), -1,
      static_cast<int16_t>(0xFF02), 0x1234, -1, 0x5678};
  bus.setFifoWords(words, 12, 0, 3);
  FifoRecord records[4];
  const OperationResult result = fourthRead(driver, bus, records, 4, 12);
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_EQUAL_UINT16(4, result.fifoRead.records);
  const int16_t expected[] = {-32768, 32767, -1, -1};
  for (uint16_t i = 0; i < 4; ++i) {
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(FifoRecordKind::TEMPERATURE),
                           static_cast<uint8_t>(records[i].kind));
    TEST_ASSERT_EQUAL_INT16(expected[i], records[i].temperatureRaw);
    TEST_ASSERT_EQUAL_UINT64(result.token.value, records[i].token.value);
    TEST_ASSERT_EQUAL_UINT32(0, records[i].timestampTicks);
    TEST_ASSERT_EQUAL_UINT16(0, records[i].stepCount);
  }
  TEST_ASSERT_EQUAL_UINT32(0, bus.timestampResetWrites);
  TEST_ASSERT_EQUAL_UINT32(0, bus.stepResetAssertions);
}

void test_fourth_timestamp_pdf_table73_preserves_raw_wrapping_counters() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  DeviceProfile profile = fourthProfile();
  profile.fifo.timestampResolution = TimestampResolution::US_25;
  fourthConfigure(driver, bus, profile);
  // Table73: ticks=(byte1<<16)|(byte0<<8)|byte3; byte2 is unused.
  const int16_t words[] = {0x1234, 0x56A5, 0x789A, -1, -91, -1,
      0, 0x00EF, 0, 0, 0x0011, 0};
  bus.setFifoWords(words, 12, 0, 3);
  FifoRecord records[4];
  const OperationResult result = fourthRead(driver, bus, records, 4, 12);
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_EQUAL_UINT16(4, result.fifoRead.records);
  const uint32_t ticks[] = {0x123456, 0xFFFFFF, 0, 0};
  const uint16_t steps[] = {0x789A, 0xFFFF, 0, 0};
  for (uint16_t i = 0; i < 4; ++i) {
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(FifoRecordKind::TIMESTAMP_STEP),
                           static_cast<uint8_t>(records[i].kind));
    TEST_ASSERT_EQUAL_HEX32(ticks[i], records[i].timestampTicks);
    TEST_ASSERT_EQUAL_HEX16(steps[i], records[i].stepCount);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(TimestampResolution::US_25),
                           static_cast<uint8_t>(records[i].timestampResolution));
    TEST_ASSERT_EQUAL_UINT64(result.counters.epoch, records[i].counterEpoch);
    TEST_ASSERT_TRUE(records[i].counterContinuityLost);
  }
}

void test_fourth_mixed_decode_handles_every_starting_pattern_position() {
  const int16_t pattern[] = {101, 102, 103, 201, 202, 203, 0x1234, 0x56EF, 0x789A};
  for (uint16_t start = 0; start < 9; ++start) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    DeviceProfile profile = fourthProfile();
    profile.fifo.gyroDecimation = FifoDecimation::EVERY_1;
    profile.fifo.accelDecimation = FifoDecimation::EVERY_1;
    fourthConfigure(driver, bus, profile);
    FifoRecord records[12];
    bus.setFifoWords(pattern, 9, 0, 9);
    TEST_ASSERT_TRUE(fourthRead(driver, bus, records, 12, 9).status.ok());
    int16_t input[27];
    for (uint16_t i = 0; i < 27; ++i) input[i] = pattern[(start + i) % 9U];
    bus.setFifoWords(input, 27, start, 9);
    const OperationResult result = fourthRead(driver, bus, records, 12, 27);
    TEST_ASSERT_TRUE(result.status.ok());
    TEST_ASSERT_EQUAL_UINT16(start % 3U == 0 ? 9 : 8, result.fifoRead.records);
    for (uint16_t i = 0; i < result.fifoRead.records; ++i) {
      const FifoRecord& record = records[i];
      if (record.kind == FifoRecordKind::ANGULAR_RATE) {
        TEST_ASSERT_EQUAL_INT16(101, record.raw.x);
        TEST_ASSERT_EQUAL_INT16(102, record.raw.y);
        TEST_ASSERT_EQUAL_INT16(103, record.raw.z);
      } else if (record.kind == FifoRecordKind::ACCELERATION) {
        TEST_ASSERT_EQUAL_INT16(201, record.raw.x);
        TEST_ASSERT_EQUAL_INT16(202, record.raw.y);
        TEST_ASSERT_EQUAL_INT16(203, record.raw.z);
      } else {
        TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(FifoRecordKind::TIMESTAMP_STEP),
                               static_cast<uint8_t>(record.kind));
        TEST_ASSERT_EQUAL_HEX32(0x123456, record.timestampTicks);
        TEST_ASSERT_EQUAL_HEX16(0x789A, record.stepCount);
      }
    }
  }
}

void test_fourth_decimator_patterns_and_maximum_are_independent() {
  const FifoDecimation codes[] = {FifoDecimation::EVERY_1, FifoDecimation::EVERY_2,
      FifoDecimation::EVERY_3, FifoDecimation::EVERY_4, FifoDecimation::EVERY_8,
      FifoDecimation::EVERY_16, FifoDecimation::EVERY_32};
  const uint8_t factors[] = {1, 2, 3, 4, 8, 16, 32};
  for (uint8_t n = 0; n < 7; ++n) {
    DeviceProfile profile = fourthProfile();
    profile.fifo.gyroDecimation = FifoDecimation::EVERY_1;
    profile.fifo.fourthDecimation = codes[n];
    const uint16_t expectedWords = static_cast<uint16_t>(3U * (factors[n] + 1U));
    for (uint16_t wordIndex = 0; wordIndex < expectedWords; ++wordIndex) {
      FifoPatternWord word;
      TEST_ASSERT_TRUE(fifoPatternWord(profile, wordIndex, word).ok());
      TEST_ASSERT_EQUAL_UINT16(expectedWords, word.patternWords);
      TEST_ASSERT_EQUAL_UINT8(wordIndex % 3U, word.axis);
      const FifoRecordKind kind = wordIndex >= 3 && wordIndex < 6
          ? FifoRecordKind::TIMESTAMP_STEP : FifoRecordKind::ANGULAR_RATE;
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(kind), static_cast<uint8_t>(word.kind));
    }
  }
  DeviceProfile maximum = fourthProfile();
  maximum.fifo.gyroDecimation = FifoDecimation::EVERY_1;
  maximum.fifo.accelDecimation = FifoDecimation::EVERY_3;
  maximum.fifo.fourthDecimation = FifoDecimation::EVERY_32;
  FifoPatternWord word;
  for (uint16_t i = 0; i < 393; ++i) {
    TEST_ASSERT_TRUE(fifoPatternWord(maximum, i, word).ok());
    TEST_ASSERT_EQUAL_UINT16(393, word.patternWords);
  }
  TEST_ASSERT_FALSE(fifoPatternWord(maximum, 393, word).ok());
  TEST_ASSERT_EQUAL_UINT16(393, MAX_FIFO_PATTERN_WORDS);
}

void test_fourth_step_events_and_internal_triggers_use_explicit_fake_inputs() {
  for (bool stepEvent : {false, true}) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    DeviceProfile profile = fourthProfile();
    if (stepEvent) {
      profile.fifo.trigger = FifoTrigger::STEP_EVENT;
      profile.fifo.odr = Odr::POWER_DOWN;
    }
    fourthConfigure(driver, bus, profile);
    bus.fifoPatternWords = 3;
    bus.setCounters(0xFFFFFF, 0xFFFF);
    bus.captureFourthDataset(!stepEvent);
    TEST_ASSERT_EQUAL_UINT16(0, bus.fifoUnreadWords);
    bus.captureFourthDataset(stepEvent);
    bus.advanceCounters(1, 1);
    bus.captureFourthDataset(stepEvent);
    FifoRecord records[2];
    const OperationResult result = fourthRead(driver, bus, records, 2, 6);
    TEST_ASSERT_TRUE(result.status.ok());
    TEST_ASSERT_EQUAL_UINT16(2, result.fifoRead.records);
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFF, records[0].timestampTicks);
    TEST_ASSERT_EQUAL_HEX16(0xFFFF, records[0].stepCount);
    TEST_ASSERT_EQUAL_HEX32(0, records[1].timestampTicks);
    TEST_ASSERT_EQUAL_HEX16(0, records[1].stepCount);
  }
}

void test_fourth_step_trigger_applies_to_mixed_motion_without_periodic_cadence() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  DeviceProfile profile = fourthProfile();
  profile.accelOdr = Odr::HZ_26;
  profile.gyroOdr = Odr::HZ_12_5;
  profile.fifo.trigger = FifoTrigger::STEP_EVENT;
  profile.fifo.odr = Odr::POWER_DOWN;
  profile.fifo.gyroDecimation = FifoDecimation::EVERY_1;
  profile.fifo.accelDecimation = FifoDecimation::EVERY_1;
  fourthConfigure(driver, bus, profile);
  const int16_t words[] = {1, 2, 3, 4, 5, 6, 0x1234, 0x56AA, 9,
                          1, 2, 3, 4, 5, 6, 0x1234, 0x56BB, 9};
  bus.setFifoWords(words, 18, 0, 9);
  FifoRecord records[6];
  const OperationResult result = fourthRead(driver, bus, records, 6, 18);
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_EQUAL_UINT16(4, result.fifoRead.records);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(FifoRecordKind::TIMESTAMP_STEP),
                         static_cast<uint8_t>(records[0].kind));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(FifoRecordKind::ANGULAR_RATE),
                         static_cast<uint8_t>(records[1].kind));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(FifoRecordKind::ACCELERATION),
                         static_cast<uint8_t>(records[2].kind));
  TEST_ASSERT_EQUAL_HEX32(0x123456, records[3].timestampTicks);
}

void test_fourth_maximum_mixed_decimation_decodes_all_393_starting_positions() {
  // Independent source schedule: G every trigger, A every third trigger,
  // counters every32 triggers. There are96 triggers and131 complete records.
  int16_t pattern[393];
  uint16_t used = 0;
  for (uint16_t trigger = 0; trigger < 96; ++trigger) {
    pattern[used++] = 101; pattern[used++] = 102; pattern[used++] = 103;
    if (trigger % 3U == 0U) {
      pattern[used++] = 201; pattern[used++] = 202; pattern[used++] = 203;
    }
    if (trigger % 32U == 0U) {
      pattern[used++] = 0x1234; pattern[used++] = 0x56AA; pattern[used++] = 0x789A;
    }
  }
  TEST_ASSERT_EQUAL_UINT16(393, used);
  for (uint16_t start = 0; start < 393; ++start) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    DeviceProfile profile = fourthProfile();
    profile.fifo.gyroDecimation = FifoDecimation::EVERY_1;
    profile.fifo.accelDecimation = FifoDecimation::EVERY_3;
    profile.fifo.fourthDecimation = FifoDecimation::EVERY_32;
    fourthConfigure(driver, bus, profile);
    FifoRecord records[262];
    bus.setFifoWords(pattern, 393, 0, 393);
    TEST_ASSERT_TRUE(fourthRead(driver, bus, records, 262, 393).status.ok());
    int16_t input[786];
    for (uint16_t i = 0; i < 786; ++i) input[i] = pattern[(start + i) % 393U];
    bus.setFifoWords(input, 786, start, 393);
    const OperationResult result = fourthRead(driver, bus, records, 262, 786);
    TEST_ASSERT_TRUE(result.status.ok());
    TEST_ASSERT_EQUAL_UINT16(start % 3U == 0U ? 262 : 261, result.fifoRead.records);
    for (uint16_t i = 0; i < result.fifoRead.records; ++i) {
      const FifoRecord& record = records[i];
      if (record.kind == FifoRecordKind::ANGULAR_RATE) {
        TEST_ASSERT_EQUAL_INT16(101, record.raw.x);
        TEST_ASSERT_EQUAL_INT16(102, record.raw.y);
        TEST_ASSERT_EQUAL_INT16(103, record.raw.z);
      } else if (record.kind == FifoRecordKind::ACCELERATION) {
        TEST_ASSERT_EQUAL_INT16(201, record.raw.x);
        TEST_ASSERT_EQUAL_INT16(202, record.raw.y);
        TEST_ASSERT_EQUAL_INT16(203, record.raw.z);
      } else {
        TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(FifoRecordKind::TIMESTAMP_STEP),
                               static_cast<uint8_t>(record.kind));
        TEST_ASSERT_EQUAL_HEX32(0x123456, record.timestampTicks);
        TEST_ASSERT_EQUAL_HEX16(0x789A, record.stepCount);
      }
    }
  }
}

void test_fourth_output_capacity_and_partial_records_remain_bounded() {
  for (uint16_t bound = 1; bound <= 7; ++bound) {
    for (uint16_t capacity = 1; capacity <= 2; ++capacity) {
      FakeBus bus;
      Driver driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      fourthConfigure(driver, bus, fourthProfile(FifoFourthDataset::TEMPERATURE));
      const int16_t words[] = {1, 100, 2, 3, 200, 4, 5, 300, 6};
      bus.setFifoWords(words, 9, 0, 3);
      FifoRecord records[2];
      const OperationResult result = fourthRead(driver, bus, records, capacity, bound);
      TEST_ASSERT_TRUE(result.status.ok());
      const uint16_t possible = bound / 3U;
      const uint16_t expected = possible < capacity ? possible : capacity;
      TEST_ASSERT_EQUAL_UINT16(expected, result.fifoRead.records);
      TEST_ASSERT_LESS_OR_EQUAL_UINT16(bound, result.fifoRead.wordsConsumed);
      TEST_ASSERT_EQUAL_UINT16(result.fifoRead.wordsConsumed - 3U * expected,
                               result.fifoRead.wordsDiscarded);
      TEST_ASSERT_TRUE(result.fifoRead.truncated);
      TEST_ASSERT_EQUAL_UINT32(maximumFifoReadTransactions(bound), result.transactionLimit);
    }
  }
}

void test_counter_reset_is_explicit_and_same_resolution_replay_preserves_values() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  const DeviceProfile profile = fourthProfile(FifoFourthDataset::TIMESTAMP_STEP, FifoMode::BYPASS);
  const OperationResult initial = fourthConfigure(driver, bus, profile);
  bus.setCounters(0x654321, 0x7654);
  const OperationResult replay = fourthConfigure(driver, bus, profile);
  TEST_ASSERT_EQUAL_HEX32(0x654321, bus.timestampTicks);
  TEST_ASSERT_EQUAL_HEX16(0x7654, bus.stepCount);
  TEST_ASSERT_EQUAL_UINT32(0, bus.timestampResetWrites);
  TEST_ASSERT_EQUAL_UINT32(0, bus.stepResetAssertions);
  TEST_ASSERT_GREATER_THAN_UINT64(initial.counters.epoch, replay.counters.epoch);
  TEST_ASSERT_TRUE(replay.counters.continuityLost);
  bus.clearTrace();
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startReconcile(fourthTiming(bus), token).inProgress());
  fourthRun(driver, bus);
  const OperationResult reconcile = fourthTake(driver, token);
  TEST_ASSERT_TRUE(reconcile.status.ok());
  TEST_ASSERT_EQUAL_UINT32(0, bus.writeCalls);
  TEST_ASSERT_EQUAL_UINT64(replay.counters.epoch, reconcile.counters.epoch);
  TEST_ASSERT_EQUAL_HEX32(0x654321, bus.timestampTicks);
  const uint32_t generation = driver.configGeneration();
  bus.clearTrace();
  TEST_ASSERT_TRUE(driver.startCounterReset({}, fourthTiming(bus), token).inProgress());
  fourthRun(driver, bus);
  const OperationResult reset = fourthTake(driver, token);
  TEST_ASSERT_TRUE(reset.status.ok());
  TEST_ASSERT_EQUAL_UINT32(7, reset.transactions);
  TEST_ASSERT_TRUE(reset.counters.timestampResetConfirmed);
  TEST_ASSERT_TRUE(reset.counters.stepResetConfirmed);
  TEST_ASSERT_FALSE(reset.counters.continuityLost);
  TEST_ASSERT_GREATER_THAN_UINT64(replay.counters.epoch, reset.counters.epoch);
  TEST_ASSERT_EQUAL_HEX32(0, bus.timestampTicks);
  TEST_ASSERT_EQUAL_HEX16(0, bus.stepCount);
  TEST_ASSERT_EQUAL_HEX8(0x34, bus.regs[cmd::REG_CTRL10_C]);
  TEST_ASSERT_EQUAL_UINT32(generation, driver.configGeneration());
}

void test_counter_reset_selection_and_admission_are_atomic() {
  for (uint8_t selected = 0; selected < 4; ++selected) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    fourthConfigure(driver, bus, fourthProfile(FifoFourthDataset::TIMESTAMP_STEP, FifoMode::BYPASS));
    bus.setCounters(0x123456, 0x789A);
    bus.clearTrace();
    OperationToken token;
    const CounterResetRequest request{(selected & 1U) != 0U, (selected & 2U) != 0U};
    const Status admitted = driver.startCounterReset(request, fourthTiming(bus), token);
    if (selected == 0) {
      TEST_ASSERT_FALSE(admitted.inProgress());
      TEST_ASSERT_FALSE(token.valid());
      TEST_ASSERT_EQUAL_UINT32(0, bus.transferCalls);
      continue;
    }
    TEST_ASSERT_TRUE(admitted.inProgress());
    fourthRun(driver, bus);
    const OperationResult result = fourthTake(driver, token);
    TEST_ASSERT_TRUE(result.status.ok());
    TEST_ASSERT_EQUAL_HEX32((selected & 1U) != 0 ? 0 : 0x123456, bus.timestampTicks);
    TEST_ASSERT_EQUAL_HEX16((selected & 2U) != 0 ? 0 : 0x789A, bus.stepCount);
    TEST_ASSERT_EQUAL(selected != 3, result.counters.continuityLost);
  }
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  OperationToken token;
  TEST_ASSERT_FALSE(driver.startCounterReset({}, fourthTiming(bus), token).inProgress());
  TEST_ASSERT_EQUAL_UINT32(0, bus.transferCalls);
  fourthConfigure(driver, bus, fourthProfile());
  bus.clearTrace();
  TEST_ASSERT_FALSE(driver.startCounterReset({}, fourthTiming(bus), token).inProgress());
  TEST_ASSERT_EQUAL_UINT32(0, bus.transferCalls);
}

void test_counter_reset_failure_cancellation_and_deadline_at_each_transfer() {
  for (uint32_t stopped = 0; stopped < 7; ++stopped) {
    for (uint8_t failure = 0; failure < 4; ++failure) {
      FakeBus bus;
      Driver driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      fourthConfigure(driver, bus, fourthProfile(FifoFourthDataset::TIMESTAMP_STEP, FifoMode::BYPASS));
      bus.setCounters(0x123456, 0x789A);
      bus.clearTrace();
      OperationToken token;
      const OperationTiming timing = fourthTiming(bus);
      if (failure < 2)
        bus.addFault(stopped + 1U, Status::Error(Err::I2C_TIMEOUT, "counter failure", -432), failure == 1);
      TEST_ASSERT_TRUE(driver.startCounterReset({}, timing, token).inProgress());
      if (failure < 2) fourthRun(driver, bus);
      else {
        fourthAdvanceToCalls(driver, bus, stopped);
        const uint32_t before = bus.transferCalls;
        if (failure == 2) TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
        else driver.poll(timing.deadlineMs, 255);
        TEST_ASSERT_EQUAL_UINT32(before, bus.transferCalls);
      }
      const OperationResult result = fourthTake(driver, token);
      TEST_ASSERT_FALSE(result.status.ok());
      TEST_ASSERT_LESS_OR_EQUAL_UINT32(7, result.transactions);
      TEST_ASSERT_TRUE(result.counters.continuityLost);
      if (stopped >= 4 || (stopped == 3 && failure < 2))
        TEST_ASSERT_TRUE(result.counters.timestampResetMayHaveOccurred);
      if ((bus.regs[cmd::REG_CTRL10_C] & 2U) != 0)
        TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNKNOWN),
            static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
    }
  }
}

void test_fourth_configure_fault_cancel_and_deadline_cover_every_transfer() {
  DeviceProfile profile = fourthProfile();
  profile.fifo.timestampResolution = TimestampResolution::US_25;
  uint32_t total = 0;
  {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    total = fourthConfigure(driver, bus, profile).transactions;
    // This counter profile has no enabled interrupt routes (no final eight callbacks).
    TEST_ASSERT_EQUAL_UINT32(98U, total);
  }
  for (uint32_t stopped = 0; stopped < total; ++stopped) {
    for (uint8_t mode = 0; mode < 4; ++mode) {
      FakeBus bus;
      Driver driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      OperationToken token;
      const OperationTiming timing = fourthTiming(bus);
      if (mode < 2)
        bus.addFault(stopped + 1U, Status::Error(Err::I2C_TIMEOUT, "configure phase", -591), mode == 1);
      TEST_ASSERT_TRUE(driver.startConfigure(profile, timing, token).inProgress());
      if (mode < 2) fourthRun(driver, bus);
      else {
        fourthAdvanceToCalls(driver, bus, stopped);
        const uint32_t before = bus.transferCalls;
        if (mode == 2) TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
        else driver.poll(timing.deadlineMs, 255);
        TEST_ASSERT_EQUAL_UINT32(before, bus.transferCalls);
      }
      const OperationResult result = fourthTake(driver, token);
      TEST_ASSERT_FALSE(result.status.ok());
      TEST_ASSERT_LESS_OR_EQUAL_UINT32(MAX_CONFIGURE_TRANSACTIONS, result.transactions);
      if (mode < 2) TEST_ASSERT_EQUAL_INT32(-591, result.status.detail);
      if ((bus.regs[cmd::REG_FUNC_CFG_ACCESS] & 0x80U) != 0U)
        TEST_ASSERT_TRUE(result.registerBankMayBeUnknown);
      TEST_ASSERT_EQUAL_UINT32(0, bus.bankWritesWhileSensorsActive);
    }
  }
}

void test_fourth_ambiguous_consumption_cancel_and_overrun_preserve_effect_evidence() {
  for (uint8_t mode = 0; mode < 4; ++mode) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    const OperationResult configured = fourthConfigure(driver, bus, fourthProfile());
    const int16_t words[] = {0x1234, 0x56AA, 7, 0x2345, 0x67BB, 8};
    bus.setFifoWords(words, 6, 0, 3);
    bus.clearTrace();
    FifoRecord records[2];
    OperationToken token;
    if (mode == 0) bus.addFault(4, Status::Error(Err::I2C_ERROR, "lost FIFO bytes"), true);
    if (mode == 1) bus.fifoOverrun = true;
    TEST_ASSERT_TRUE(driver.startFifoRead({records, 2, 6}, fourthTiming(bus), token).inProgress());
    if (mode < 2) fourthRun(driver, bus);
    else {
      fourthAdvanceToCalls(driver, bus, mode == 2 ? 0 : 4);
      const uint32_t before = bus.transferCalls;
      TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
      TEST_ASSERT_EQUAL_UINT32(before, bus.transferCalls);
    }
    const OperationResult result = fourthTake(driver, token);
    TEST_ASSERT_FALSE(result.status.ok());
    TEST_ASSERT_EQUAL_UINT16(0, result.fifoRead.records);
    TEST_ASSERT_FALSE(records[0].token.valid());
    TEST_ASSERT_FALSE(records[1].token.valid());
    TEST_ASSERT_TRUE(result.counters.continuityLost);
    TEST_ASSERT_GREATER_THAN_UINT64(configured.counters.epoch, result.counters.epoch);
    if (mode == 0) {
      TEST_ASSERT_EQUAL_UINT16(0, result.fifoRead.wordsConsumed);
      TEST_ASSERT_EQUAL_UINT16(6, result.fifoRead.wordsPossiblyConsumed);
      TEST_ASSERT_TRUE(result.hardwareStateMayHaveChanged);
    }
    if (mode == 3) TEST_ASSERT_EQUAL_UINT16(6, result.fifoRead.wordsConsumed);
  }
}

void test_fourth_engine_disable_and_unknown_intervals_never_expand_counters() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  const DeviceProfile profile = fourthProfile(FifoFourthDataset::TIMESTAMP_STEP, FifoMode::BYPASS);
  const OperationResult configured = fourthConfigure(driver, bus, profile);
  bus.setCounters(0xFFFFFE, 0xFFFE);
  // Multiple24-bit wraps are deliberately unknowable from these raw values.
  bus.advanceCounters(0x02000005, 5);
  TEST_ASSERT_EQUAL_HEX32(3, bus.timestampTicks);
  TEST_ASSERT_EQUAL_HEX16(3, bus.stepCount);
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startPowerDown(fourthTiming(bus), token).inProgress());
  fourthRun(driver, bus);
  const OperationResult down = fourthTake(driver, token);
  TEST_ASSERT_TRUE(down.status.ok());
  TEST_ASSERT_TRUE(down.counters.continuityLost);
  TEST_ASSERT_GREATER_THAN_UINT64(configured.counters.epoch, down.counters.epoch);
  TEST_ASSERT_EQUAL_HEX32(3, bus.timestampTicks);
  TEST_ASSERT_EQUAL_HEX16(3, bus.stepCount);
  fourthConfigure(driver, bus, profile);
  TEST_ASSERT_EQUAL_HEX16(3, bus.stepCount);
  bus.regs[cmd::REG_CTRL10_C] = 0;
  bus.advanceCounters(999, 999);
  TEST_ASSERT_EQUAL_HEX32(3, bus.timestampTicks);
  TEST_ASSERT_EQUAL_HEX16(3, bus.stepCount);
}

void test_fourth_continuous_nine_word_headroom_boundary_is_exact() {
  for (uint16_t initial : {2038, 2039}) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    DeviceProfile profile = fourthProfile();
    profile.fifo.gyroDecimation = FifoDecimation::EVERY_1;
    profile.fifo.accelDecimation = FifoDecimation::EVERY_1;
    fourthConfigure(driver, bus, profile);
    const int16_t warmup[] = {1, 2, 3, 4, 5, 6, 0x1234, 0x56AA, 7};
    bus.setFifoWords(warmup, 9, 0, 9);
    FifoRecord records[6];
    TEST_ASSERT_TRUE(fourthRead(driver, bus, records, 6, 9).status.ok());
    bus.setFifo(initial, 0, 100);
    bus.fifoPatternWords = 9;
    const OperationResult result = fourthRead(driver, bus, records, 6, 16);
    TEST_ASSERT_EQUAL_UINT16(16, result.fifoRead.wordsConsumed);
    TEST_ASSERT_EQUAL_UINT16(initial - 16U, result.fifoRead.after.unreadWords);
    if (initial == 2038) {
      TEST_ASSERT_TRUE(result.status.ok());
      TEST_ASSERT_EQUAL_UINT16(5, result.fifoRead.records);
    } else {
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::OPERATION_INDETERMINATE),
                             static_cast<uint8_t>(result.status.code));
      TEST_ASSERT_EQUAL_UINT16(0, result.fifoRead.records);
      TEST_ASSERT_TRUE(result.counters.continuityLost);
    }
  }
}

void test_counter_bank_policy_mismatch_leaves_visible_bank_fence() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  bus.corruptReadRegister = 0x14;
  bus.corruptReadValue = 0x6F;
  bus.corruptReadRemaining = 1;
  bus.corruptReadBankOnly = true;
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startConfigure(fourthProfile(), fourthTiming(bus), token).inProgress());
  fourthRun(driver, bus);
  const OperationResult result = fourthTake(driver, token);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CONFIGURATION_MISMATCH),
                         static_cast<uint8_t>(result.status.code));
  TEST_ASSERT_EQUAL_HEX8(0x14, result.configuration.mismatchRegister);
  TEST_ASSERT_EQUAL_HEX8(0x6E, result.configuration.expectedValue);
  TEST_ASSERT_EQUAL_HEX8(0x6F, result.configuration.observedValue);
  TEST_ASSERT_TRUE(result.registerBankMayBeUnknown);
  TEST_ASSERT_EQUAL_HEX8(0x80, bus.regs[cmd::REG_FUNC_CFG_ACCESS]);
  const uint32_t calls = bus.transferCalls;
  uint8_t untouched = 0xA5;
  TEST_ASSERT_FALSE(driver.diagnosticReadRegister(cmd::REG_CTRL4_C, untouched, bus.nowMs).ok());
  TEST_ASSERT_FALSE(driver.diagnosticWriteRegister(cmd::REG_CTRL4_C, 0, bus.nowMs).ok());
  TEST_ASSERT_EQUAL_HEX8(0xA5, untouched);
  TEST_ASSERT_EQUAL_UINT32(calls, bus.transferCalls);
  TEST_ASSERT_TRUE(driver.diagnosticWriteRegister(cmd::REG_FUNC_CFG_ACCESS, 0, bus.nowMs).ok());
  TEST_ASSERT_TRUE(driver.diagnosticReadRegister(cmd::REG_FUNC_CFG_ACCESS, untouched, bus.nowMs).ok());
  TEST_ASSERT_EQUAL_HEX8(0, untouched);
}

void test_counter_read_only_reconcile_preserves_values_at_every_fault_stage() {
  for (uint32_t stage = 1; stage <= MAX_RECONCILE_TRANSACTIONS; ++stage) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    const OperationResult configured = fourthConfigure(driver, bus,
        fourthProfile(FifoFourthDataset::TIMESTAMP_STEP, FifoMode::BYPASS));
    bus.setCounters(0xFEDCBA, 0x9876);
    bus.clearTrace();
    bus.addFault(stage, Status::Error(Err::I2C_TIMEOUT, "read-only counter reconcile"));
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startReconcile(fourthTiming(bus), token).inProgress());
    fourthRun(driver, bus);
    const OperationResult result = fourthTake(driver, token);
    TEST_ASSERT_FALSE(result.status.ok());
    TEST_ASSERT_EQUAL_UINT32(0, bus.writeCalls);
    TEST_ASSERT_EQUAL_UINT16(0, bus.fifoDataReads);
    TEST_ASSERT_EQUAL_HEX32(0xFEDCBA, bus.timestampTicks);
    TEST_ASSERT_EQUAL_HEX16(0x9876, bus.stepCount);
    TEST_ASSERT_EQUAL_UINT64(configured.counters.epoch, result.counters.epoch);
  }
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  fourthConfigure(driver, bus,
      fourthProfile(FifoFourthDataset::TIMESTAMP_STEP, FifoMode::BYPASS));
  TEST_ASSERT_TRUE(driver.diagnosticWriteRegister(cmd::REG_CTRL4_C, 0, bus.nowMs).ok());
  bus.clearTrace();
  OperationToken token;
  TEST_ASSERT_FALSE(driver.startReconcile(fourthTiming(bus), token).inProgress());
  TEST_ASSERT_EQUAL_UINT32(0, bus.transferCalls);
}

void test_counter_reset_boot_and_recovery_restore_all_owned_controls() {
  for (uint8_t operation = 0; operation < 3; ++operation) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    DeviceProfile profile = fourthProfile();
    profile.fifo.timestampResolution = TimestampResolution::US_25;
    profile.fifo.gyroDecimation = FifoDecimation::EVERY_1;
    const OperationResult configured = fourthConfigure(driver, bus, profile);
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startFifoPurge({1, true}, fourthTiming(bus), token).inProgress());
    fourthRun(driver, bus);
    TEST_ASSERT_TRUE(fourthTake(driver, token).status.ok());
    bus.clearTrace();
    bus.setCounters(0xABCDEF, 0x6789);
    bus.resetRegistersOnSoftwareReset = true;
    if (operation == 1) bus.bootBitReadsRemaining = 15;
    else bus.resetBitReadsRemaining = 15;
    const Status started = operation == 0 ? driver.startReset(fourthTiming(bus), token) :
        operation == 1 ? driver.startBoot(fourthTiming(bus), token) :
                         driver.startRecover(fourthTiming(bus), token);
    TEST_ASSERT_TRUE(started.inProgress());
    fourthRun(driver, bus);
    const OperationResult result = fourthTake(driver, token);
    TEST_ASSERT_TRUE_MESSAGE(result.status.ok(), result.status.msg);
    const uint32_t ceiling = operation == 2 ? MAX_RECOVER_TRANSACTIONS : MAX_RESET_TRANSACTIONS;
    TEST_ASSERT_EQUAL_UINT32(ceiling, result.transactionLimit);
    TEST_ASSERT_EQUAL_UINT32(operation == 1 ? 118U : operation == 2 ? 118U : 119U, result.transactions);
    TEST_ASSERT_EQUAL_HEX8(0x34, bus.regs[cmd::REG_CTRL10_C]);
    TEST_ASSERT_EQUAL_HEX8(0x10, bus.regs[cmd::REG_WAKE_UP_DUR]);
    TEST_ASSERT_EQUAL_HEX8(0x80, bus.regs[cmd::REG_FIFO_CTRL2]);
    TEST_ASSERT_EQUAL_HEX8(0x08, bus.regs[cmd::REG_FIFO_CTRL4]);
    TEST_ASSERT_EQUAL_HEX8(0x26, bus.regs[cmd::REG_FIFO_CTRL5]);
    TEST_ASSERT_EQUAL_HEX16(operation == 1 ? 0x6789 : 0, bus.stepCount);
    TEST_ASSERT_GREATER_THAN_UINT64(configured.counters.epoch, result.counters.epoch);
    TEST_ASSERT_TRUE(result.counters.continuityLost);
  }
}

void test_counter_self_test_restores_profile_without_implicit_resets() {
  for (bool failShutdown : {false, true}) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    DeviceProfile profile = fourthProfile(FifoFourthDataset::TIMESTAMP_STEP, FifoMode::BYPASS);
    profile.fifo.timestampResolution = TimestampResolution::US_25;
    const OperationResult configured = fourthConfigure(driver, bus, profile);
    const uint32_t resets = bus.timestampResetWrites;
    bus.setCounters(0x123456, 0x789A);
    bus.clearTrace();
    bus.notReadyPeriod = 3;
    if (failShutdown)
      bus.addFault(16U * 6U + 27U, Status::Error(Err::I2C_TIMEOUT, "shutdown counter restore"));
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startSelfTest({5}, fourthTiming(bus), token).inProgress());
    fourthRun(driver, bus);
    const OperationResult result = fourthTake(driver, token);
    TEST_ASSERT_EQUAL(!failShutdown, result.status.ok());
    TEST_ASSERT_TRUE(result.selfTest.restorationStatus.ok());
    TEST_ASSERT_EQUAL_UINT32(16U * 6U + (failShutdown ? 113U : 112U), result.transactions);
    TEST_ASSERT_EQUAL_UINT32(16U * 6U + 126U, result.transactionLimit);
    TEST_ASSERT_EQUAL_HEX32(0x123456, bus.timestampTicks);
    TEST_ASSERT_EQUAL_HEX16(0x789A, bus.stepCount);
    TEST_ASSERT_EQUAL_UINT32(resets, bus.timestampResetWrites);
    TEST_ASSERT_EQUAL_UINT32(0, bus.stepResetAssertions);
    TEST_ASSERT_EQUAL_HEX8(0x34, bus.regs[cmd::REG_CTRL10_C]);
    TEST_ASSERT_EQUAL_HEX8(0x10, bus.regs[cmd::REG_WAKE_UP_DUR]);
    TEST_ASSERT_GREATER_THAN_UINT64(configured.counters.epoch, result.counters.epoch);
    TEST_ASSERT_TRUE(result.counters.continuityLost);
  }
}

void test_counter_resolution_transition_through_disabled_profile_resets_at_change() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  DeviceProfile fast = fourthProfile(FifoFourthDataset::TIMESTAMP_STEP, FifoMode::BYPASS);
  fast.fifo.timestampResolution = TimestampResolution::US_25;
  fourthConfigure(driver, bus, fast);
  bus.setCounters(0x123456, 0x789A);
  const uint32_t before = bus.timestampResetWrites;
  const OperationResult disabled = fourthConfigure(driver, bus, DeviceProfile{});
  TEST_ASSERT_EQUAL_UINT32(before + 1U, bus.timestampResetWrites);
  TEST_ASSERT_TRUE(disabled.counters.timestampResetConfirmed);
  TEST_ASSERT_EQUAL_HEX32(0, bus.timestampTicks);
  TEST_ASSERT_EQUAL_HEX16(0x789A, bus.stepCount);
  bus.setCounters(0x654321, 0x789A);
  fourthConfigure(driver, bus,
      fourthProfile(FifoFourthDataset::TIMESTAMP_STEP, FifoMode::BYPASS));
  TEST_ASSERT_EQUAL_UINT32(before + 1U, bus.timestampResetWrites);
  TEST_ASSERT_EQUAL_HEX32(0x654321, bus.timestampTicks);
  TEST_ASSERT_EQUAL_HEX16(0x789A, bus.stepCount);
}

void test_counter_diagnostics_and_rebinding_preserve_distinct_epochs() {
  FakeBus bus;
  Driver driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  const DeviceProfile profile = fourthProfile(FifoFourthDataset::TIMESTAMP_STEP, FifoMode::BYPASS);
  fourthConfigure(driver, bus, profile);
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startCounterReset({}, fourthTiming(bus), token).inProgress());
  fourthRun(driver, bus);
  const OperationResult reset = fourthTake(driver, token);
  TEST_ASSERT_FALSE(reset.counters.continuityLost);
  bus.setCounters(0x123456, 0x789A);
  uint8_t raw[3];
  TEST_ASSERT_TRUE(driver.diagnosticReadBlock(cmd::REG_TIMESTAMP0, raw, 3, bus.nowMs).ok());
  TEST_ASSERT_EQUAL_HEX8(0x56, raw[0]);
  TEST_ASSERT_EQUAL_HEX8(0x34, raw[1]);
  TEST_ASSERT_EQUAL_HEX8(0x12, raw[2]);
  TEST_ASSERT_TRUE(driver.startProbe(fourthTiming(bus), token).inProgress());
  fourthRun(driver, bus);
  const OperationResult readOnly = fourthTake(driver, token);
  TEST_ASSERT_EQUAL_UINT64(reset.counters.epoch, readOnly.counters.epoch);
  TEST_ASSERT_FALSE(readOnly.counters.continuityLost);
  const int16_t queued[] = {0x1234, 0x56AA, 0x789A};
  bus.setFifoWords(queued, 3, 0, 3);
  TEST_ASSERT_TRUE(driver.diagnosticReadBlock(cmd::REG_FIFO_DATA_OUT_L, raw, 2, bus.nowMs).ok());
  TEST_ASSERT_TRUE(driver.startProbe(fourthTiming(bus), token).inProgress());
  fourthRun(driver, bus);
  const OperationResult consumed = fourthTake(driver, token);
  TEST_ASSERT_GREATER_THAN_UINT64(readOnly.counters.epoch, consumed.counters.epoch);
  TEST_ASSERT_TRUE(consumed.counters.continuityLost);
  TEST_ASSERT_TRUE(driver.diagnosticWriteRegister(cmd::REG_TIMESTAMP2, 0xAA, bus.nowMs).ok());
  TEST_ASSERT_EQUAL_HEX32(0, bus.timestampTicks);
  TEST_ASSERT_TRUE(driver.startProbe(fourthTiming(bus), token).inProgress());
  fourthRun(driver, bus);
  const OperationResult mutated = fourthTake(driver, token);
  TEST_ASSERT_GREATER_THAN_UINT64(consumed.counters.epoch, mutated.counters.epoch);
  TEST_ASSERT_TRUE(mutated.counters.continuityLost);
  const uint32_t calls = bus.transferCalls;
  driver.unbind();
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  TEST_ASSERT_EQUAL_UINT32(calls, bus.transferCalls);
  TEST_ASSERT_TRUE(driver.startProbe(fourthTiming(bus), token).inProgress());
  fourthRun(driver, bus);
  const OperationResult rebound = fourthTake(driver, token);
  TEST_ASSERT_GREATER_THAN_UINT64(mutated.counters.epoch, rebound.counters.epoch);
  TEST_ASSERT_TRUE(rebound.counters.continuityLost);
}

void test_counter_self_test_restoration_fault_cancel_deadline_cover_every_transfer() {
  // Four sample phases reserve three readiness checks plus one burst for each of the
  // five averages and one discard; eight mask callbacks and 20 recipe writes precede restoration.
  static constexpr uint32_t RESTORE_START = 16U * 6U + 28U;
  static constexpr uint32_t RESTORE_END = RESTORE_START + 16U + 66U + 2U;
  for (uint32_t stopped = RESTORE_START; stopped < RESTORE_END; ++stopped) {
    for (uint8_t mode = 0; mode < 4; ++mode) {
      FakeBus bus;
      Driver driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      DeviceProfile profile = fourthProfile(FifoFourthDataset::TIMESTAMP_STEP, FifoMode::BYPASS);
      profile.fifo.timestampResolution = TimestampResolution::US_25;
      const OperationResult configured = fourthConfigure(driver, bus, profile);
      bus.clearTrace();
      bus.notReadyPeriod = 3;
      if (mode < 2)
        bus.addFault(stopped + 1U, Status::Error(Err::I2C_TIMEOUT, "self-test counter restore", -719),
                     mode == 1);
      OperationToken token;
      const OperationTiming timing = fourthTiming(bus);
      TEST_ASSERT_TRUE(driver.startSelfTest({5}, timing, token).inProgress());
      if (mode < 2) fourthRun(driver, bus);
      else {
        fourthAdvanceToCalls(driver, bus, stopped);
        const uint32_t before = bus.transferCalls;
        if (mode == 2) TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
        else driver.poll(timing.deadlineMs, 255);
        TEST_ASSERT_EQUAL_UINT32(before, bus.transferCalls);
      }
      const OperationResult result = fourthTake(driver, token);
      TEST_ASSERT_FALSE(result.status.ok());
      TEST_ASSERT_FALSE(result.selfTest.restorationStatus.ok());
      TEST_ASSERT_TRUE(result.counters.continuityLost);
      TEST_ASSERT_GREATER_THAN_UINT64(configured.counters.epoch, result.counters.epoch);
      TEST_ASSERT_LESS_OR_EQUAL_UINT32(maximumSelfTestTransactions(5), result.transactions);
      if (mode < 2) TEST_ASSERT_EQUAL_INT32(-719, result.status.detail);
      if ((bus.regs[cmd::REG_FUNC_CFG_ACCESS] & 0x80U) != 0U)
        TEST_ASSERT_TRUE(result.registerBankMayBeUnknown);
    }
  }
}

void test_counter_reset_callback_ceiling_prevents_unbudgeted_effects() {
  for (uint32_t limit = 1; limit < 7; ++limit) {
    FakeBus bus;
    Driver driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    fourthConfigure(driver, bus,
        fourthProfile(FifoFourthDataset::TIMESTAMP_STEP, FifoMode::BYPASS));
    bus.setCounters(0x123456, 0x789A);
    bus.clearTrace();
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startCounterReset({}, fourthTiming(bus), token).inProgress());
    lowerFifoTestCeiling(driver, limit);
    fourthRun(driver, bus);
    const OperationResult result = fourthTake(driver, token);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TRANSACTION_LIMIT_EXCEEDED),
                           static_cast<uint8_t>(result.status.code));
    TEST_ASSERT_EQUAL_UINT32(limit, result.transactions);
    TEST_ASSERT_EQUAL_UINT32(limit, bus.transferCalls);
    TEST_ASSERT_EQUAL(limit >= 4U, result.counters.timestampResetMayHaveOccurred);
    TEST_ASSERT_EQUAL(limit >= 4U, result.counters.timestampResetConfirmed);
    TEST_ASSERT_EQUAL(limit >= 5U, result.counters.stepResetMayHaveOccurred);
    TEST_ASSERT_FALSE(result.counters.stepResetConfirmed);
    if (limit < 4U) TEST_ASSERT_FALSE(result.hardwareStateMayHaveChanged);
    TEST_ASSERT_TRUE(result.counters.continuityLost);
  }
}

}  // namespace

void runFifoFourthTests() {
  RUN_TEST(test_fourth_profile_invalid_combinations_are_bus_silent_and_atomic);
  RUN_TEST(test_fourth_register_image_policy_and_resolution_reset_sequence);
  RUN_TEST(test_fourth_temperature_pdf_table74_uses_only_middle_signed_pair);
  RUN_TEST(test_fourth_timestamp_pdf_table73_preserves_raw_wrapping_counters);
  RUN_TEST(test_fourth_mixed_decode_handles_every_starting_pattern_position);
  RUN_TEST(test_fourth_decimator_patterns_and_maximum_are_independent);
  RUN_TEST(test_fourth_step_events_and_internal_triggers_use_explicit_fake_inputs);
  RUN_TEST(test_fourth_step_trigger_applies_to_mixed_motion_without_periodic_cadence);
  RUN_TEST(test_fourth_maximum_mixed_decimation_decodes_all_393_starting_positions);
  RUN_TEST(test_fourth_output_capacity_and_partial_records_remain_bounded);
  RUN_TEST(test_counter_reset_is_explicit_and_same_resolution_replay_preserves_values);
  RUN_TEST(test_counter_reset_selection_and_admission_are_atomic);
  RUN_TEST(test_counter_reset_failure_cancellation_and_deadline_at_each_transfer);
  RUN_TEST(test_fourth_configure_fault_cancel_and_deadline_cover_every_transfer);
  RUN_TEST(test_fourth_ambiguous_consumption_cancel_and_overrun_preserve_effect_evidence);
  RUN_TEST(test_fourth_engine_disable_and_unknown_intervals_never_expand_counters);
  RUN_TEST(test_fourth_continuous_nine_word_headroom_boundary_is_exact);
  RUN_TEST(test_counter_bank_policy_mismatch_leaves_visible_bank_fence);
  RUN_TEST(test_counter_read_only_reconcile_preserves_values_at_every_fault_stage);
  RUN_TEST(test_counter_reset_boot_and_recovery_restore_all_owned_controls);
  RUN_TEST(test_counter_self_test_restores_profile_without_implicit_resets);
  RUN_TEST(test_counter_resolution_transition_through_disabled_profile_resets_at_change);
  RUN_TEST(test_counter_diagnostics_and_rebinding_preserve_distinct_epochs);
  RUN_TEST(test_counter_self_test_restoration_fault_cancel_deadline_cover_every_transfer);
  RUN_TEST(test_counter_reset_callback_ceiling_prevents_unbudgeted_effects);
}
