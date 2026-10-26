/// @file test_fifo.cpp
/// @brief Independent motion FIFO wire, schema, ownership and fault regressions.
#include <unity.h>

#include <cstdint>
#include <cstring>
#include <limits>
#include <initializer_list>

#include "LSM6DS3TR/LSM6DS3TR.h"
#include "support/FakeBus.h"

using namespace LSM6DS3TR;
using namespace LSM6DS3TRTest;

void lowerFifoTestCeiling(LSM6DS3TR::LSM6DS3TR& driver, uint32_t limit);

namespace {

DeviceProfile motionProfile() {
  DeviceProfile profile;
  profile.fifo.enabled = true;
  profile.fifo.mode = FifoMode::CONTINUOUS;
  profile.fifo.odr = Odr::HZ_104;
  profile.fifo.gyroDecimation = FifoDecimation::EVERY_1;
  profile.fifo.accelDecimation = FifoDecimation::EVERY_1;
  return profile;
}

OperationTiming fifoTiming(const FakeBus& bus, uint64_t duration = 30000U) {
  return {bus.nowMs, bus.nowMs + duration};
}

OperationResult fifoTake(LSM6DS3TR::LSM6DS3TR& driver, OperationToken token) {
  OperationResult result;
  TEST_ASSERT_TRUE(driver.takeResult(token, result).ok());
  TEST_ASSERT_EQUAL_UINT64(token.value, result.token.value);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::RESULT_NOT_AVAILABLE),
      static_cast<uint8_t>(driver.takeResult(token, result).code));
  return result;
}

void fifoRun(LSM6DS3TR::LSM6DS3TR& driver, FakeBus& bus) {
  // Alternate CPU-only, single-attempt, and large budgets on every operation.
  const uint8_t budgets[] = {0, 1, 255, 2, 0, 7};
  for (uint32_t n = 0; n < 30000U && driver.operationActive(); ++n) {
    const uint8_t budget = budgets[n % 6U];
    const uint32_t before = bus.transferCalls;
    const PollResult result = driver.poll(bus.nowMs++, budget);
    TEST_ASSERT_EQUAL_UINT32(bus.transferCalls - before, result.transactionsUsed);
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(budget, result.transactionsUsed);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(result.transactionLimit, result.transactions);
  }
  TEST_ASSERT_FALSE(driver.operationActive());
  TEST_ASSERT_TRUE(driver.resultPending());
  TEST_ASSERT_EQUAL_UINT16(0, bus.fifoEmptyReadAttempts);
  for (size_t i = 0; i < bus.traceCount; ++i) {
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(32, bus.trace[i].rxLength);
  }
}

OperationResult fifoConfigure(LSM6DS3TR::LSM6DS3TR& driver, FakeBus& bus,
                              const DeviceProfile& profile = motionProfile()) {
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startConfigure(profile, fifoTiming(bus), token).inProgress());
  fifoRun(driver, bus);
  const OperationResult result = fifoTake(driver, token);
  TEST_ASSERT_TRUE_MESSAGE(result.status.ok(), result.status.msg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::KNOWN),
      static_cast<uint8_t>(driver.configurationState(bus.nowMs)));
  return result;
}

void assertPattern(const DeviceProfile& profile, const char* vectors) {
  const uint16_t words = static_cast<uint16_t>(std::strlen(vectors) * 3U);
  TEST_ASSERT_TRUE(validateProfile(profile).ok());
  for (uint16_t i = 0; i < words; ++i) {
    FifoPatternWord word;
    TEST_ASSERT_TRUE(fifoPatternWord(profile, i, word).ok());
    TEST_ASSERT_EQUAL_UINT16(words, word.patternWords);
    TEST_ASSERT_EQUAL_UINT8(i % 3U, word.axis);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(vectors[i / 3U] == 'G' ? SAMPLE_ANGULAR_RATE :
                          SAMPLE_ACCELERATION, word.quantityMask, vectors);
  }
  FifoPatternWord untouched{123, 45, 67};
  TEST_ASSERT_FALSE(fifoPatternWord(profile, words, untouched).ok());
  TEST_ASSERT_EQUAL_UINT16(123, untouched.patternWords);
  TEST_ASSERT_EQUAL_UINT8(45, untouched.quantityMask);
  TEST_ASSERT_EQUAL_UINT8(67, untouched.axis);
}

void test_fifo_patterns_match_independent_vendor_vectors() {
  DeviceProfile profile = motionProfile();
  assertPattern(profile, "GA"); // AN5130 Table 69.
  profile.gyroOdr = Odr::HZ_208;
  profile.fifo.odr = Odr::HZ_208;
  profile.fifo.accelDecimation = FifoDecimation::EVERY_2;
  assertPattern(profile, "GAG"); // AN5130 Table 70, all nine positions.
  profile = motionProfile();
  profile.accelOdr = Odr::HZ_208;
  profile.fifo.odr = Odr::HZ_208;
  profile.fifo.gyroDecimation = FifoDecimation::EVERY_2;
  assertPattern(profile, "GAA"); // Motion-only part of AN5130 example 3.
  profile = motionProfile();
  profile.fifo.accelDecimation = FifoDecimation::OFF;
  assertPattern(profile, "G");
  profile.fifo.accelDecimation = FifoDecimation::EVERY_1;
  profile.fifo.gyroDecimation = FifoDecimation::OFF;
  assertPattern(profile, "A");

  const FifoDecimation decimations[] = {
      FifoDecimation::EVERY_1, FifoDecimation::EVERY_2,
      FifoDecimation::EVERY_3, FifoDecimation::EVERY_4,
      FifoDecimation::EVERY_8, FifoDecimation::EVERY_16,
      FifoDecimation::EVERY_32};
  const char* expected[] = {"GA", "GAG", "GAGG", "GAGGG", "GAGGGGGGG",
      "GAGGGGGGGGGGGGGGG", "GAGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG"};
  for (uint8_t i = 0; i < 7U; ++i) {
    profile = motionProfile();
    profile.fifo.accelDecimation = decimations[i];
    assertPattern(profile, expected[i]);
    profile.fifo.gyroDecimation = decimations[i];
    assertPattern(profile, "GA");
    profile.fifo.accelDecimation = FifoDecimation::OFF;
    assertPattern(profile, "G");
  }
  // Independent schedule: G every 3 ticks, A at ticks 0,32,64 in 96 ticks.
  profile = motionProfile();
  profile.fifo.gyroDecimation = FifoDecimation::EVERY_3;
  profile.fifo.accelDecimation = FifoDecimation::EVERY_32;
  assertPattern(profile, "GAGGGGGGGGGGAGGGGGGGGGGGAGGGGGGGGGG");
}

void test_fifo_profile_validation_is_total_atomic_and_bus_silent() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  for (uint16_t n = 0; n < 256U; ++n) {
    DeviceProfile profile = motionProfile();
    profile.fifo.mode = static_cast<FifoMode>(n);
    const DeviceProfile before = profile;
    const bool expected = n == 0U || n == 1U || n == 6U;
    TEST_ASSERT_EQUAL(expected, validateProfile(profile).ok());
    TEST_ASSERT_EQUAL_MEMORY(&before, &profile, sizeof(profile));
  }
  for (uint8_t mutation = 0; mutation < 13U; ++mutation) {
    DeviceProfile profile = motionProfile();
    switch (mutation) {
      case 0: profile.fifo.odr = Odr::POWER_DOWN; break;
      case 1: profile.fifo.odr = Odr::HZ_1_6; break;
      case 2: profile.fifo.odr = static_cast<Odr>(255); break;
      case 3: profile.fifo.gyroDecimation = static_cast<FifoDecimation>(8); break;
      case 4: profile.fifo.accelDecimation = static_cast<FifoDecimation>(255); break;
      case 5: profile.fifo.gyroDecimation = profile.fifo.accelDecimation =
          FifoDecimation::OFF; break;
      case 6: profile.gyroOdr = Odr::POWER_DOWN; break;
      case 7: profile.accelOdr = Odr::POWER_DOWN; break;
      case 8: profile.gyroSleepEnabled = true; break;
      case 9: profile.gyroOdr = Odr::HZ_208; profile.fifo.odr = Odr::HZ_208; break;
      case 10: profile.fifo.watermarkWords = 2048; break;
      case 11: profile.fifo.enabled = false; break;
      case 12: profile.fifo.gyroDecimation = FifoDecimation::OFF;
          profile.gyroSleepEnabled = true; break;
    }
    const DeviceProfile before = profile;
    TEST_ASSERT_FALSE(validateProfile(profile).ok());
    OperationToken token{42};
    TEST_ASSERT_FALSE(driver.startConfigure(profile, fifoTiming(bus), token).inProgress());
    TEST_ASSERT_FALSE(token.valid());
    TEST_ASSERT_EQUAL_MEMORY(&before, &profile, sizeof(profile));
  }
  DeviceProfile profile = motionProfile();
  profile.fifo.watermarkWords = 2047;
  TEST_ASSERT_TRUE(validateProfile(profile).ok());
  profile.fifo.watermarkWords = 0;
  TEST_ASSERT_TRUE(validateProfile(profile).ok());
  TEST_ASSERT_EQUAL_UINT32(0, bus.transferCalls);
}

void test_fifo_wire_image_and_activation_follow_verified_settling() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  DeviceProfile profile = motionProfile();
  profile.accelOdr = Odr::HZ_208;
  profile.fifo.odr = Odr::HZ_208;
  profile.fifo.gyroDecimation = FifoDecimation::EVERY_2;
  profile.fifo.watermarkWords = 0x5AB;
  const OperationResult result = fifoConfigure(driver, bus, profile);
  const uint8_t expected[] = {0xAB, 0x05, 0x11, 0x00, 0x2E};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, bus.regs + 0x06, 5);
  TEST_ASSERT_EQUAL_HEX8(0, bus.regs[0x1A]); // Internal trigger, no sensor hub.
  uint64_t lastProfileRead = 0;
  uint64_t activation = 0;
  for (size_t i = 0; i < bus.traceCount; ++i) {
    const Transfer& transfer = bus.trace[i];
    if (transfer.kind == TransferKind::WRITE_READ && transfer.startReg == 0x75) {
      lastProfileRead = transfer.atMs;
    }
    if (transfer.kind == TransferKind::WRITE && transfer.startReg == 0x0A &&
        (transfer.firstValue & 7U) != 0U) {
      TEST_ASSERT_EQUAL_UINT64(0, activation);
      TEST_ASSERT_NOT_EQUAL(0, lastProfileRead);
      activation = transfer.atMs;
    }
  }
  TEST_ASSERT_TRUE(activation > lastProfileRead);
  TEST_ASSERT_TRUE(activation >= lastProfileRead +
      (requiredSettleUs(profile) + 999U) / 1000U + 1U);
  TEST_ASSERT_EQUAL_UINT32(MAX_CONFIGURE_TRANSACTIONS, result.transactions);
}

OperationResult fifoRead(LSM6DS3TR::LSM6DS3TR& driver, FakeBus& bus,
                         FifoMotionRecord* records, uint16_t capacity,
                         uint16_t maximum) {
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startFifoRead({records, capacity, maximum},
                                      fifoTiming(bus), token).inProgress());
  fifoRun(driver, bus);
  return fifoTake(driver, token);
}

void warmFifo(LSM6DS3TR::LSM6DS3TR& driver, FakeBus& bus,
              const DeviceProfile& profile = motionProfile()) {
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)fifoConfigure(driver, bus, profile);
  const int16_t first[] = {101, 102, 103, 201, 202, 203};
  bus.setFifoWords(first, 6, 0, 6);
  FifoMotionRecord scratch[2];
  const OperationResult result = fifoRead(driver, bus, scratch, 2, 6);
  TEST_ASSERT_TRUE_MESSAGE(result.status.ok(), result.status.msg);
  TEST_ASSERT_EQUAL_UINT16(0, result.fifoRead.records);
  TEST_ASSERT_EQUAL_UINT16(6, result.fifoRead.wordsDiscarded);
  bus.clearTrace();
}

void assertOutputCleared(const FifoMotionRecord* records, uint16_t count) {
  for (uint16_t i = 0; i < count; ++i) {
    TEST_ASSERT_FALSE(records[i].token.valid());
    TEST_ASSERT_EQUAL_UINT8(0, records[i].quantityMask);
    TEST_ASSERT_EQUAL_INT16(0, records[i].raw.x);
    TEST_ASSERT_EQUAL_INT16(0, records[i].raw.y);
    TEST_ASSERT_EQUAL_INT16(0, records[i].raw.z);
  }
}

void test_fifo_records_are_complete_signed_and_keep_immutable_provenance() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  DeviceProfile profile = motionProfile();
  profile.accelFullScale = AccelFs::G_8;
  profile.gyroFullScale = GyroFs::DPS_1000;
  warmFifo(driver, bus, profile);
  const int16_t words[] = {-32768, -1, 32767, 1, 2, 3, -4, -5, -6,
      7, 8, 9, -10, -11, -12, 13, 14, 15};
  bus.setFifoWords(words, 18, 0, 6);
  FifoMotionRecord records[6];
  const uint64_t started = bus.nowMs;
  const OperationResult result = fifoRead(driver, bus, records, 6, 18);
  TEST_ASSERT_TRUE_MESSAGE(result.status.ok(), result.status.msg);
  TEST_ASSERT_EQUAL_UINT16(6, result.fifoRead.records);
  TEST_ASSERT_EQUAL_UINT16(18, result.fifoRead.wordsConsumed);
  TEST_ASSERT_EQUAL_UINT16(18, result.fifoRead.wordsPossiblyConsumed);
  TEST_ASSERT_EQUAL_UINT16(0, result.fifoRead.wordsDiscarded);
  TEST_ASSERT_TRUE(result.fifoRead.before.valid);
  TEST_ASSERT_TRUE(result.fifoRead.after.valid);
  TEST_ASSERT_EQUAL_UINT16(18, result.fifoRead.before.unreadWords);
  TEST_ASSERT_EQUAL_UINT16(0, result.fifoRead.after.unreadWords);
  TEST_ASSERT_FALSE(result.fifoRead.truncated);
  TEST_ASSERT_FALSE(result.fifoRead.overrunObserved);
  TEST_ASSERT_FALSE(result.fifoRead.patternDiscontinuity);
  for (uint16_t i = 0; i < 6U; ++i) {
    TEST_ASSERT_EQUAL_UINT64(result.token.value, records[i].token.value);
    TEST_ASSERT_EQUAL_UINT32(driver.configGeneration(), records[i].configGeneration);
    TEST_ASSERT_EQUAL_UINT8(i % 2U == 0U ? SAMPLE_ANGULAR_RATE : SAMPLE_ACCELERATION,
                          records[i].quantityMask);
    TEST_ASSERT_EQUAL_INT16(words[3U * i], records[i].raw.x);
    TEST_ASSERT_EQUAL_INT16(words[3U * i + 1U], records[i].raw.y);
    TEST_ASSERT_EQUAL_INT16(words[3U * i + 2U], records[i].raw.z);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelFs::G_8),
                            static_cast<uint8_t>(records[i].accelFullScale));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(GyroFs::DPS_1000),
                            static_cast<uint8_t>(records[i].gyroFullScale));
    TEST_ASSERT_TRUE(records[i].readUptimeMs >= started);
    TEST_ASSERT_TRUE(records[i].readUptimeMs <= result.completedUptimeMs);
  }
  // Capture times are shared within a burst; no artificial ODR timestamps.
  TEST_ASSERT_EQUAL_UINT64(records[0].readUptimeMs, records[1].readUptimeMs);
  IntegerAxes physical;
  TEST_ASSERT_TRUE(decodeAcceleration(records[1].raw, records[1].accelFullScale,
                                      physical).ok());
  TEST_ASSERT_EQUAL_INT64(244, physical.x);
}

void test_fifo_every_starting_axis_and_pattern_wrap_has_explicit_discard() {
  for (uint16_t pattern = 0; pattern < 6U; ++pattern) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    warmFifo(driver, bus);
    bus.setFifo(18, pattern, 100);
    bus.fifoPatternWords = 6;
    FifoMotionRecord records[6];
    const OperationResult result = fifoRead(driver, bus, records, 6, 18);
    TEST_ASSERT_TRUE_MESSAGE(result.status.ok(), result.status.msg);
    const uint16_t leading = static_cast<uint16_t>((3U - pattern % 3U) % 3U);
    const uint16_t complete = static_cast<uint16_t>((18U - leading) / 3U);
    TEST_ASSERT_EQUAL_UINT16(complete, result.fifoRead.records);
    TEST_ASSERT_EQUAL_UINT16(18, result.fifoRead.wordsConsumed);
    TEST_ASSERT_EQUAL_UINT16(18U - 3U * complete, result.fifoRead.wordsDiscarded);
    TEST_ASSERT_EQUAL_UINT16(pattern, result.fifoRead.before.pattern);
    TEST_ASSERT_EQUAL_UINT16(pattern, result.fifoRead.after.pattern);
    for (uint16_t i = 0; i < complete; ++i) {
      TEST_ASSERT_EQUAL_INT16(100U + leading + 3U * i, records[i].raw.x);
      const uint16_t vector = (pattern + leading + 3U * i) % 6U;
      TEST_ASSERT_EQUAL_UINT8(vector == 0U ? SAMPLE_ANGULAR_RATE : SAMPLE_ACCELERATION,
                            records[i].quantityMask);
    }
  }
}

void test_fifo_short_buffer_word_bound_and_partial_vectors_are_bounded() {
  const uint16_t capacities[] = {1, 4, 4};
  const uint16_t limits[] = {12, 4, 2};
  const uint16_t starts[] = {0, 0, 1};
  const uint16_t consumed[] = {3, 4, 2};
  const uint16_t valid[] = {1, 1, 0};
  const uint16_t discarded[] = {0, 1, 2};
  for (uint8_t c = 0; c < 3; ++c) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    warmFifo(driver, bus);
    bus.setFifo(12, starts[c], 10);
    bus.fifoPatternWords = 6;
    FifoMotionRecord records[4];
    const OperationResult result = fifoRead(driver, bus, records, capacities[c], limits[c]);
    TEST_ASSERT_TRUE_MESSAGE(result.status.ok(), result.status.msg);
    TEST_ASSERT_EQUAL_UINT16(consumed[c], result.fifoRead.wordsConsumed);
    TEST_ASSERT_EQUAL_UINT16(valid[c], result.fifoRead.records);
    TEST_ASSERT_EQUAL_UINT16(discarded[c], result.fifoRead.wordsDiscarded);
    TEST_ASSERT_EQUAL_UINT16(12U - consumed[c], bus.fifoUnreadWords);
    TEST_ASSERT_TRUE(result.fifoRead.truncated);
  }
}

void test_fifo_empty_full_overrun_and_contradictions_never_read_empty() {
  for (uint8_t c = 0; c < 6U; ++c) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    DeviceProfile profile = motionProfile();
    if (c == 1) profile.fifo.mode = FifoMode::FIFO;
    warmFifo(driver, bus, profile);
    bus.setFifo(c == 0 ? 0 : c == 1 ? 2047 : c == 2 ? 2048 : c == 3 ? 0 : 6,
                0, 1, c == 2 || c == 5);
    bus.fifoPatternWords = 6;
    if (c == 3) bus.forceFifoEmptyClear = true;
    if (c == 4 || c == 5) bus.forceFifoEmptySet = true;
    FifoMotionRecord records[2];
    const OperationResult result = fifoRead(driver, bus, records, 2, 3);
    if (c <= 1) {
      TEST_ASSERT_TRUE_MESSAGE(result.status.ok(), result.status.msg);
      TEST_ASSERT_EQUAL_UINT16(c == 0 ? 0 : 1, result.fifoRead.records);
      if (c == 1) TEST_ASSERT_TRUE(result.fifoRead.before.full);
    } else {
      TEST_ASSERT_FALSE(result.status.ok());
      TEST_ASSERT_EQUAL_UINT16(0, result.fifoRead.records);
      TEST_ASSERT_EQUAL_UINT16(0, bus.fifoDataReads);
      assertOutputCleared(records, 2);
    }
    TEST_ASSERT_EQUAL_UINT16(0, bus.fifoEmptyReadAttempts);
  }
}

void test_fifo_concurrent_arrivals_remain_for_next_bounded_batch() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  warmFifo(driver, bus);
  bus.setFifo(6, 0, 1);
  bus.fifoPatternWords = 6;
  bus.fifoConcurrentArrivalWords = 6;
  FifoMotionRecord records[4];
  const OperationResult result = fifoRead(driver, bus, records, 4, 12);
  TEST_ASSERT_TRUE_MESSAGE(result.status.ok(), result.status.msg);
  TEST_ASSERT_EQUAL_UINT16(6, result.fifoRead.wordsConsumed);
  TEST_ASSERT_EQUAL_UINT16(6, result.fifoRead.after.unreadWords);
  TEST_ASSERT_EQUAL_UINT16(6, bus.fifoUnreadWords);
  TEST_ASSERT_TRUE(result.fifoRead.truncated);
}

void test_fifo_pattern_discontinuity_invalidates_entire_output() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  warmFifo(driver, bus);
  bus.setFifo(30, 0, 1);
  bus.fifoPatternWords = 6;
  bus.fifoPatternJumpAfterRead = 1;
  bus.fifoPatternJump = 1;
  FifoMotionRecord records[10];
  const OperationResult result = fifoRead(driver, bus, records, 10, 30);
  TEST_ASSERT_FALSE(result.status.ok());
  TEST_ASSERT_TRUE(result.fifoRead.patternDiscontinuity);
  TEST_ASSERT_TRUE(result.fifoRead.continuityLost);
  TEST_ASSERT_EQUAL_UINT16(0, result.fifoRead.records);
  TEST_ASSERT_GREATER_THAN_UINT16(0, result.fifoRead.wordsConsumed);
  assertOutputCleared(records, 10);
}

void test_fifo_transport_failure_at_every_stage_never_retries_or_publishes() {
  FakeBus reference;
  LSM6DS3TR::LSM6DS3TR referenceDriver;
  warmFifo(referenceDriver, reference);
  reference.setFifo(33, 0, 20);
  reference.fifoPatternWords = 6;
  FifoMotionRecord referenceRecords[11];
  const OperationResult good = fifoRead(referenceDriver, reference, referenceRecords, 11, 33);
  TEST_ASSERT_TRUE(good.status.ok());
  bool sawMaximumBurst = false;
  for (size_t i = 0; i < reference.traceCount; ++i) {
    if (reference.trace[i].rxLength == 32U) sawMaximumBurst = true;
  }
  TEST_ASSERT_TRUE(sawMaximumBurst);
  const Err errors[] = {Err::I2C_NACK_ADDR, Err::I2C_ERROR, Err::I2C_TIMEOUT};
  for (Err error : errors) {
    for (uint32_t stage = 1; stage <= good.transactions; ++stage) {
      for (bool applyEffect : {false, true}) {
        FakeBus bus;
        LSM6DS3TR::LSM6DS3TR driver;
        warmFifo(driver, bus);
        bus.setFifo(33, 0, 20);
        bus.fifoPatternWords = 6;
        bus.addFault(stage, Status::Error(error, "FIFO read fault", -71), applyEffect);
        FifoMotionRecord records[11];
        const OperationResult result = fifoRead(driver, bus, records, 11, 33);
        TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(error),
                                static_cast<uint8_t>(result.status.code));
        TEST_ASSERT_EQUAL_INT32(-71, result.status.detail);
        TEST_ASSERT_EQUAL_UINT32(stage, bus.transferCalls);
        TEST_ASSERT_EQUAL_UINT32(stage, result.transactions);
        TEST_ASSERT_EQUAL_UINT16(0, result.fifoRead.records);
        TEST_ASSERT_TRUE(result.fifoRead.wordsPossiblyConsumed >=
                         result.fifoRead.wordsConsumed);
        TEST_ASSERT_TRUE(result.fifoRead.wordsPossiblyConsumed >= bus.fifoDataReads);
        TEST_ASSERT_EQUAL(result.fifoRead.wordsPossiblyConsumed > 0U,
                          result.hardwareStateMayHaveChanged);
        assertOutputCleared(records, 11);
      }
    }
  }
}

void test_fifo_cancellation_deadline_and_invalid_requests_have_no_stale_batch() {
  for (uint8_t stop = 0; stop < 7U; ++stop) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    warmFifo(driver, bus);
    bus.setFifo(30, 0, 20);
    bus.fifoPatternWords = 6;
    FifoMotionRecord records[10];
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startFifoRead({records, 10, 30}, fifoTiming(bus), token).inProgress());
    for (uint8_t n = 0; n < stop; ++n) (void)driver.poll(bus.nowMs++, 1);
    TEST_ASSERT_TRUE(driver.operationActive());
    const uint32_t calls = bus.transferCalls;
    if (stop == 6U) {
      (void)driver.poll(bus.nowMs + 30000U, 255);
    } else {
      TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
    }
    TEST_ASSERT_EQUAL_UINT32(calls, bus.transferCalls);
    const OperationResult result = fifoTake(driver, token);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(stop == 6U ? Err::DEADLINE_EXPIRED :
                                               Err::CANCELLED),
                            static_cast<uint8_t>(result.status.code));
    TEST_ASSERT_EQUAL_UINT16(0, result.fifoRead.records);
    assertOutputCleared(records, 10);
  }
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  warmFifo(driver, bus);
  FifoMotionRecord records[2];
  records[0].token = {999};
  const FifoReadRequest invalid[] = {{nullptr, 1, 1}, {records, 0, 1},
      {records, 683, 1}, {records, 1, 0}, {records, 1, 2049}};
  for (const FifoReadRequest& request : invalid) {
    OperationToken token{99};
    TEST_ASSERT_FALSE(driver.startFifoRead(request, fifoTiming(bus), token).inProgress());
    TEST_ASSERT_FALSE(token.valid());
    TEST_ASSERT_EQUAL_UINT64(999, records[0].token.value);
  }
  TEST_ASSERT_EQUAL_UINT32(0, bus.transferCalls);
  TEST_ASSERT_EQUAL_UINT32(0, maximumFifoReadTransactions(0));
  TEST_ASSERT_EQUAL_UINT32(0, maximumFifoReadTransactions(2049));
  TEST_ASSERT_EQUAL_UINT32(5, maximumFifoReadTransactions(1));
  TEST_ASSERT_EQUAL_UINT32(4099, maximumFifoReadTransactions(2048));
}

void test_fifo_buffer_lifetime_tokens_backpressure_unbind_and_rebind() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  warmFifo(driver, bus);
  bus.setFifo(6, 0, 10);
  bus.fifoPatternWords = 6;
  FifoMotionRecord records[2];
  records[0].token = {77};
  OperationToken token;
  TEST_ASSERT_TRUE(driver.startFifoRead({records, 2, 6}, fifoTiming(bus), token).inProgress());
  assertOutputCleared(records, 2);
  OperationToken refused{99};
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUSY), static_cast<uint8_t>(
      driver.startProbe(fifoTiming(bus), refused).code));
  fifoRun(driver, bus);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::RESULT_PENDING), static_cast<uint8_t>(
      driver.startProbe(fifoTiming(bus), refused).code));
  OperationResult result;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::STALE_RESULT), static_cast<uint8_t>(
      driver.takeResult({token.value + 1}, result).code));
  TEST_ASSERT_TRUE(driver.resultPending());
  const OperationResult taken = fifoTake(driver, token);
  TEST_ASSERT_EQUAL_UINT16(2, taken.fifoRead.records);
  const FifoMotionRecord saved = records[0];
  driver.unbind();
  TEST_ASSERT_EQUAL_UINT64(saved.token.value, records[0].token.value);
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CONFIGURATION_UNKNOWN),
      static_cast<uint8_t>(driver.startFifoRead({records, 2, 6}, fifoTiming(bus), refused).code));

  // A pending successful result still owns its caller buffer until consumed.
  bus.regs[0x0A] = 0; // Independent fixture reset before reconfiguration.
  (void)fifoConfigure(driver, bus);
  bus.setFifo(12, 0, 50);
  bus.fifoPatternWords = 6;
  TEST_ASSERT_TRUE(driver.startFifoRead({records, 2, 12}, fifoTiming(bus), token).inProgress());
  fifoRun(driver, bus);
  driver.unbind();
  assertOutputCleared(records, 2);
  TEST_ASSERT_FALSE(driver.resultPending());
}

void test_fifo_configure_faults_and_cancellation_cover_activation_stages() {
  const Err errors[] = {Err::I2C_NACK_ADDR, Err::I2C_ERROR, Err::I2C_TIMEOUT};
  for (Err error : errors) {
    for (uint32_t stage = 1; stage <= MAX_CONFIGURE_TRANSACTIONS; ++stage) {
      FakeBus bus;
      LSM6DS3TR::LSM6DS3TR driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      bus.addFault(stage, Status::Error(error, "FIFO configure stage", -81), true);
      OperationToken token;
      TEST_ASSERT_TRUE(driver.startConfigure(motionProfile(), fifoTiming(bus), token).inProgress());
      fifoRun(driver, bus);
      const OperationResult result = fifoTake(driver, token);
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(error), static_cast<uint8_t>(result.status.code));
      TEST_ASSERT_EQUAL_UINT32(stage, result.transactions);
      TEST_ASSERT_EQUAL_UINT32(stage, bus.transferCalls);
      TEST_ASSERT_EQUAL(stage > 3U, result.hardwareStateMayHaveChanged);
      TEST_ASSERT_EQUAL_UINT32(0, driver.configGeneration());
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(stage > 3U ? ConfigurationState::UNKNOWN :
                                                             ConfigurationState::UNCONFIGURED),
                            static_cast<uint8_t>(result.configuration.state));
    }
  }
  for (uint32_t stop = 0; stop <= MAX_CONFIGURE_TRANSACTIONS; ++stop) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startConfigure(motionProfile(), fifoTiming(bus), token).inProgress());
    for (uint32_t n = 0; n < 30000U && bus.transferCalls < stop; ++n) {
      (void)driver.poll(bus.nowMs++, 1);
    }
    TEST_ASSERT_EQUAL_UINT32(stop, bus.transferCalls);
    TEST_ASSERT_TRUE(driver.operationActive());
    TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
    const OperationResult result = fifoTake(driver, token);
    TEST_ASSERT_EQUAL_UINT32(stop, bus.transferCalls);
    TEST_ASSERT_EQUAL_UINT32(0, driver.configGeneration());
    TEST_ASSERT_EQUAL(stop > 3U, result.hardwareStateMayHaveChanged);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CANCELLED),
                            static_cast<uint8_t>(result.status.code));
  }
}

void test_fifo_reconcile_is_read_only_at_every_failure_stage_and_cannot_relabel_unknown_data() {
  const Err errors[] = {Err::I2C_NACK_ADDR, Err::I2C_ERROR, Err::I2C_TIMEOUT};
  for (Err error : errors) {
    for (uint32_t stage = 1; stage <= MAX_RECONCILE_TRANSACTIONS; ++stage) {
      FakeBus bus;
      LSM6DS3TR::LSM6DS3TR driver;
      warmFifo(driver, bus);
      bus.setFifo(6, 0, 900);
      bus.addFault(stage, Status::Error(error, "FIFO reconcile stage", -82));
      const uint32_t generation = driver.configGeneration();
      OperationToken token;
      TEST_ASSERT_TRUE(driver.startReconcile(fifoTiming(bus), token).inProgress());
      fifoRun(driver, bus);
      const OperationResult result = fifoTake(driver, token);
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(error), static_cast<uint8_t>(result.status.code));
      TEST_ASSERT_EQUAL_UINT32(stage, bus.transferCalls);
      TEST_ASSERT_EQUAL_UINT32(0, bus.writeCalls);
      TEST_ASSERT_EQUAL_UINT16(0, bus.fifoDataReads);
      TEST_ASSERT_EQUAL_UINT16(6, bus.fifoUnreadWords);
      TEST_ASSERT_EQUAL_UINT32(generation, driver.configGeneration());
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::KNOWN),
                            static_cast<uint8_t>(result.configuration.state));
    }
  }
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  warmFifo(driver, bus);
  TEST_ASSERT_TRUE(driver.diagnosticWriteRegister(0x10, bus.regs[0x10], bus.nowMs).ok());
  bus.clearTrace();
  OperationToken token;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::FIFO_MAINTENANCE_REQUIRED),
      static_cast<uint8_t>(driver.startReconcile(fifoTiming(bus), token).code));
  TEST_ASSERT_FALSE(token.valid());
  TEST_ASSERT_EQUAL_UINT32(0, bus.transferCalls);
}

void test_fifo_live_collection_blocks_profile_and_lifecycle_mutations() {
  for (uint8_t operation = 0; operation < 6U; ++operation) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    warmFifo(driver, bus);
    bus.setFifo(6, 0, 700);
    bus.fifoPatternWords = 6;
    const uint32_t generation = driver.configGeneration();
    OperationToken token;
    DeviceProfile changed = motionProfile();
    changed.accelFullScale = AccelFs::G_16;
    Status start;
    switch (operation) {
      case 0: start = driver.startConfigure(changed, fifoTiming(bus), token); break;
      case 1: start = driver.startReset(fifoTiming(bus), token); break;
      case 2: start = driver.startBoot(fifoTiming(bus), token); break;
      case 3: start = driver.startRecover(fifoTiming(bus), token); break;
      case 4: start = driver.startPowerDown(fifoTiming(bus), token); break;
      default: start = driver.startSelfTest({}, fifoTiming(bus), token); break;
    }
    if (start.inProgress()) {
      fifoRun(driver, bus);
      const OperationResult result = fifoTake(driver, token);
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::FIFO_MAINTENANCE_REQUIRED),
                            static_cast<uint8_t>(result.status.code));
    } else {
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::FIFO_MAINTENANCE_REQUIRED),
                              static_cast<uint8_t>(start.code));
    }
    TEST_ASSERT_EQUAL_UINT16(6, bus.fifoUnreadWords);
    TEST_ASSERT_EQUAL_UINT16(0, bus.fifoDataReads);
    TEST_ASSERT_EQUAL_HEX8(0x40, bus.regs[0x10]);
    TEST_ASSERT_EQUAL_HEX8(0x40, bus.regs[0x11]);
    TEST_ASSERT_EQUAL_HEX8(0x26, bus.regs[0x0A]);
    TEST_ASSERT_EQUAL_UINT32(generation, driver.configGeneration());
    if (operation == 0U) {
      FifoMotionRecord records[2];
      const OperationResult result = fifoRead(driver, bus, records, 2, 6);
      TEST_ASSERT_TRUE(result.status.ok());
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelFs::G_2),
                              static_cast<uint8_t>(records[1].accelFullScale));
    }
  }
}

void test_fifo_explicit_stop_reports_residual_loss_and_recovery_reestablishes_schema() {
  for (bool arrivals : {false, true}) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    warmFifo(driver, bus);
    bus.setFifo(6, 0, 500);
    bus.fifoPatternWords = 6;
    if (arrivals) bus.fifoConcurrentArrivalWords = 3;
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startFifoPurge({3, true}, fifoTiming(bus), token).inProgress());
    fifoRun(driver, bus);
    const OperationResult stopped = fifoTake(driver, token);
    TEST_ASSERT_TRUE_MESSAGE(stopped.status.ok(), stopped.status.msg);
    TEST_ASSERT_TRUE(stopped.fifoPurge.collectionStopped);
    TEST_ASSERT_TRUE(stopped.fifoPurge.residualLossPossible);
    TEST_ASSERT_TRUE(stopped.fifoPurge.truncated);
    TEST_ASSERT_EQUAL_UINT16(3, stopped.fifoPurge.wordsDiscarded);
    TEST_ASSERT_EQUAL_UINT16(0, bus.fifoUnreadWords);
    TEST_ASSERT_EQUAL_HEX8(0, bus.regs[0x0A] & 7U);
    TEST_ASSERT_EQUAL_UINT32(maximumFifoPurgeTransactions(3, true), stopped.transactions);
    const uint32_t generation = driver.configGeneration();
    bus.clearTrace();
    TEST_ASSERT_TRUE(driver.startRecover(fifoTiming(bus), token).inProgress());
    fifoRun(driver, bus);
    const OperationResult recovered = fifoTake(driver, token);
    TEST_ASSERT_TRUE_MESSAGE(recovered.status.ok(), recovered.status.msg);
    TEST_ASSERT_GREATER_THAN_UINT32(generation, driver.configGeneration());
    TEST_ASSERT_EQUAL_HEX8(0x26, bus.regs[0x0A]);
    bus.setFifo(12, 0, 800);
    bus.fifoPatternWords = 6;
    FifoMotionRecord records[4];
    const OperationResult read = fifoRead(driver, bus, records, 4, 12);
    TEST_ASSERT_TRUE(read.status.ok());
    TEST_ASSERT_EQUAL_UINT16(6, read.fifoRead.wordsDiscarded);
    TEST_ASSERT_EQUAL_UINT16(2, read.fifoRead.records);
    TEST_ASSERT_EQUAL_INT16(806, records[0].raw.x);
    TEST_ASSERT_EQUAL_UINT32(driver.configGeneration(), records[0].configGeneration);
  }
}

void test_fifo_transaction_ceiling_guards_consumption_before_callback() {
  for (uint8_t limit = 3; limit <= 5; ++limit) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    warmFifo(driver, bus);
    bus.setFifo(6, 0, 11);
    bus.fifoPatternWords = 6;
    FifoMotionRecord record;
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startFifoRead({&record, 1, 1}, fifoTiming(bus), token).inProgress());
    lowerFifoTestCeiling(driver, limit);
    fifoRun(driver, bus);
    const OperationResult result = fifoTake(driver, token);
    TEST_ASSERT_EQUAL_UINT32(limit, bus.transferCalls);
    TEST_ASSERT_EQUAL_UINT32(limit, result.transactions);
    TEST_ASSERT_EQUAL_UINT32(limit, result.transactionLimit);
    TEST_ASSERT_EQUAL_UINT16(limit == 3U ? 0U : 1U, result.fifoRead.wordsConsumed);
    TEST_ASSERT_EQUAL_UINT16(limit == 3U ? 0U : 1U, result.fifoRead.wordsPossiblyConsumed);
    TEST_ASSERT_EQUAL(limit > 3U, result.hardwareStateMayHaveChanged);
    TEST_ASSERT_EQUAL(limit == 5U, result.status.ok());
  }
}

void test_fifo_single_sensor_wire_images_and_records() {
  for (bool gyro : {false, true}) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    DeviceProfile profile = motionProfile();
    profile.fifo.mode = FifoMode::FIFO;
    if (gyro) {
      profile.fifo.accelDecimation = FifoDecimation::OFF;
      profile.accelOdr = Odr::POWER_DOWN;
    } else {
      profile.fifo.gyroDecimation = FifoDecimation::OFF;
      profile.gyroOdr = Odr::POWER_DOWN;
    }
    (void)fifoConfigure(driver, bus, profile);
    TEST_ASSERT_EQUAL_HEX8(gyro ? 0x08 : 0x01, bus.regs[0x08]);
    TEST_ASSERT_EQUAL_HEX8(0x21, bus.regs[0x0A]);
    const int16_t words[] = {1, 2, 3, -4, -5, -6, 7, 8, 9};
    bus.setFifoWords(words, 9, 0, 3);
    FifoMotionRecord records[3];
    const OperationResult read = fifoRead(driver, bus, records, 3, 9);
    TEST_ASSERT_TRUE(read.status.ok());
    TEST_ASSERT_EQUAL_UINT16(3, read.fifoRead.wordsDiscarded);
    TEST_ASSERT_EQUAL_UINT16(2, read.fifoRead.records);
    TEST_ASSERT_EQUAL_UINT8(gyro ? SAMPLE_ANGULAR_RATE : SAMPLE_ACCELERATION,
                          records[0].quantityMask);
    TEST_ASSERT_EQUAL_INT16(-4, records[0].raw.x);
    TEST_ASSERT_EQUAL_INT16(-6, records[0].raw.z);
    TEST_ASSERT_EQUAL_INT16(7, records[1].raw.x);
  }
}

void test_fifo_transition_discard_and_stream_loss_persist_across_calls() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
  (void)fifoConfigure(driver, bus);
  bus.setFifo(18, 0, 100);
  bus.fifoPatternWords = 6;
  FifoMotionRecord records[6];
  OperationResult result = fifoRead(driver, bus, records, 6, 1);
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_EQUAL_UINT16(0, result.fifoRead.records);
  TEST_ASSERT_EQUAL_UINT16(1, result.fifoRead.wordsDiscarded);
  result = fifoRead(driver, bus, records, 6, 17);
  TEST_ASSERT_TRUE(result.status.ok());
  // Partial first G discarded, first complete A and subsequent complete G
  // still each require their transition discard. No invalid XYZ escapes.
  TEST_ASSERT_EQUAL_UINT16(8, result.fifoRead.wordsDiscarded);
  TEST_ASSERT_EQUAL_UINT16(3, result.fifoRead.records);
  TEST_ASSERT_EQUAL_INT16(109, records[0].raw.x);

  bus.setFifo(6, 0, 500);
  bus.clearTrace();
  bus.addFault(4, Status::Error(Err::I2C_TIMEOUT, "consumed burst"), true);
  result = fifoRead(driver, bus, records, 6, 6);
  TEST_ASSERT_FALSE(result.status.ok());
  TEST_ASSERT_TRUE(result.fifoRead.continuityLost);
  bus.clearTrace();
  bus.setFifo(6, 0, 700);
  result = fifoRead(driver, bus, records, 6, 6);
  TEST_ASSERT_TRUE(result.status.ok());
  TEST_ASSERT_EQUAL_UINT16(2, result.fifoRead.records);
  TEST_ASSERT_TRUE(result.fifoRead.continuityLost);
}

void test_fifo_identity_and_post_read_empty_contradiction_invalidate_batch() {
  for (bool identity : {false, true}) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    warmFifo(driver, bus);
    bus.setFifo(12, 0, 500);
    bus.fifoPatternWords = 6;
    if (identity) bus.regs[0x0F] = 0x69;
    else bus.forceFifoEmptySetAfterDataRead = true;
    FifoMotionRecord records[2];
    const OperationResult result = fifoRead(driver, bus, records, 2, 6);
    TEST_ASSERT_FALSE(result.status.ok());
    TEST_ASSERT_EQUAL_UINT16(0, result.fifoRead.records);
    assertOutputCleared(records, 2);
    if (identity) {
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CHIP_ID_MISMATCH),
                            static_cast<uint8_t>(result.status.code));
      TEST_ASSERT_EQUAL_UINT16(0, bus.fifoDataReads);
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNKNOWN),
                            static_cast<uint8_t>(result.configuration.state));
    } else {
      TEST_ASSERT_TRUE(result.fifoRead.continuityLost);
      TEST_ASSERT_EQUAL_UINT16(6, result.fifoRead.wordsConsumed);
    }
  }
}

void test_fifo_transient_overwrite_with_whole_pattern_skip_never_publishes() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR driver;
  warmFifo(driver, bus);
  bus.setFifo(2042, 0, 100);
  bus.fifoPatternWords = 6;
  bus.fifoOverflowOnDataRead = 1;
  bus.fifoOverflowArrivalWords = 12;
  bus.fifoClearOverrunOnDataRead = true;
  FifoMotionRecord records[10];
  const OperationResult result = fifoRead(driver, bus, records, 10, 30);
  TEST_ASSERT_FALSE(result.status.ok());
  TEST_ASSERT_FALSE(result.fifoRead.overrunObserved);
  // Six overwritten words wrap the pattern exactly. The post-burst headroom
  // guard must reject even though neither pattern nor sticky flags prove it.
  TEST_ASSERT_FALSE(result.fifoRead.patternDiscontinuity);
  TEST_ASSERT_TRUE(result.fifoRead.continuityLost);
  TEST_ASSERT_EQUAL_UINT16(0, result.fifoRead.records);
  assertOutputCleared(records, 10);
}

void test_fifo_explicit_stop_all_fault_cancel_and_ceiling_stages_preserve_effects() {
  const Err errors[] = {Err::I2C_NACK_ADDR, Err::I2C_ERROR, Err::I2C_TIMEOUT};
  for (Err error : errors) {
    for (uint32_t stage = 1; stage <= 10U; ++stage) {
      for (bool applied : {false, true}) {
        FakeBus bus;
        LSM6DS3TR::LSM6DS3TR driver;
        warmFifo(driver, bus);
        bus.setFifo(6, 0, 101);
        bus.fifoPatternWords = 6;
        bus.addFault(stage, Status::Error(error, "FIFO stop stage", -91), applied);
        OperationToken token;
        TEST_ASSERT_TRUE(driver.startFifoPurge({3, true}, fifoTiming(bus), token).inProgress());
        fifoRun(driver, bus);
        const OperationResult result = fifoTake(driver, token);
        TEST_ASSERT_EQUAL_UINT32(stage, result.transactions);
        TEST_ASSERT_EQUAL_UINT32(stage, bus.transferCalls);
        TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(error), static_cast<uint8_t>(result.status.code));
        TEST_ASSERT_FALSE(result.fifoPurge.collectionStopped);
        TEST_ASSERT_TRUE(result.fifoPurge.wordsPossiblyDiscarded >=
                         result.fifoPurge.wordsDiscarded);
        TEST_ASSERT_TRUE(result.fifoPurge.wordsPossiblyDiscarded >= bus.fifoDataReads);
        TEST_ASSERT_EQUAL(stage > 4U, result.hardwareStateMayHaveChanged);
        if (stage >= 9U) {
          TEST_ASSERT_TRUE(result.fifoPurge.residualLossPossible);
          TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ConfigurationState::UNKNOWN),
                                static_cast<uint8_t>(result.configuration.state));
        }
      }
    }
  }
  for (uint8_t stop = 0; stop < 10U; ++stop) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    warmFifo(driver, bus);
    bus.setFifo(6, 0, 101);
    bus.fifoPatternWords = 6;
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startFifoPurge({3, true}, fifoTiming(bus), token).inProgress());
    for (uint8_t n = 0; n < stop; ++n) (void)driver.poll(bus.nowMs++, 1);
    TEST_ASSERT_TRUE(driver.operationActive());
    const uint32_t calls = bus.transferCalls;
    if (stop == 9U) (void)driver.poll(bus.nowMs + 30000U, 255);
    else TEST_ASSERT_TRUE(driver.cancelActiveJob(bus.nowMs).ok());
    const OperationResult result = fifoTake(driver, token);
    TEST_ASSERT_EQUAL_UINT32(calls, bus.transferCalls);
    TEST_ASSERT_FALSE(result.fifoPurge.collectionStopped);
    TEST_ASSERT_EQUAL(stop > 4U, result.hardwareStateMayHaveChanged);
    TEST_ASSERT_EQUAL_UINT16(stop < 5U ? 0U : stop - 4U < 3U ? stop - 4U : 3U,
                            result.fifoPurge.wordsDiscarded);
  }
  for (uint8_t limit = 8; limit <= 9U; ++limit) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    warmFifo(driver, bus);
    bus.setFifo(6, 0, 101);
    bus.fifoPatternWords = 6;
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startFifoPurge({3, true}, fifoTiming(bus), token).inProgress());
    lowerFifoTestCeiling(driver, limit);
    fifoRun(driver, bus);
    const OperationResult result = fifoTake(driver, token);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TRANSACTION_LIMIT_EXCEEDED),
                            static_cast<uint8_t>(result.status.code));
    TEST_ASSERT_EQUAL_UINT32(limit, result.transactions);
    TEST_ASSERT_EQUAL_HEX8(limit == 8U ? 6U : 0U, bus.regs[0x0A] & 7U);
    TEST_ASSERT_FALSE(result.fifoPurge.collectionStopped);
  }
}

void test_fifo_every_motion_decimator_has_an_independent_literal_wire_image() {
  // Datasheet Tables 33/34 and AN5130 Tables 54/55: these expected bytes are
  // literal wire fixtures, independent of enum values and register builders.
  const FifoDecimation factors[] = {FifoDecimation::EVERY_1,
      FifoDecimation::EVERY_2, FifoDecimation::EVERY_3, FifoDecimation::EVERY_4,
      FifoDecimation::EVERY_8, FifoDecimation::EVERY_16, FifoDecimation::EVERY_32};
  const uint8_t gyroBytes[] = {0x08, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38};
  const uint8_t accelBytes[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
  for (bool gyro : {false, true}) {
    for (uint8_t i = 0; i < 7U; ++i) {
      FakeBus bus;
      LSM6DS3TR::LSM6DS3TR driver;
      TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
      DeviceProfile profile = motionProfile();
      profile.fifo.gyroDecimation = gyro ? factors[i] : FifoDecimation::OFF;
      profile.fifo.accelDecimation = gyro ? FifoDecimation::OFF : factors[i];
      (void)fifoConfigure(driver, bus, profile);
      const uint8_t expected[] = {0x00, 0x00, gyro ? gyroBytes[i] : accelBytes[i],
                                  0x00, 0x26};
      TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, bus.regs + 0x06, 5);
      bool writeSeen = false;
      bool readbackSeen = false;
      for (size_t n = 0; n < bus.traceCount; ++n) {
        const Transfer& transfer = bus.trace[n];
        if (transfer.startReg != 0x08) continue;
        if (transfer.kind == TransferKind::WRITE) {
          TEST_ASSERT_EQUAL_HEX8(expected[2], transfer.firstValue);
          writeSeen = true;
        } else {
          TEST_ASSERT_TRUE(writeSeen);
          readbackSeen = true;
        }
      }
      TEST_ASSERT_TRUE(writeSeen);
      TEST_ASSERT_TRUE(readbackSeen);
    }
  }
}

void test_fifo_acquisition_decodes_every_start_in_the_maximum_pattern() {
  // 96 trigger ticks: G at 0,3,...,93; A at 0,32,64. Dataset order at a
  // shared tick is G then A. This explicit 35-vector schedule is independent
  // of fifoPatternWord(), including the asymmetric gap around factor 3.
  static constexpr char VECTORS[] = "GAGGGGGGGGGGAGGGGGGGGGGGAGGGGGGGGGG";
  static_assert(sizeof(VECTORS) == 36U, "Maximum schedule has 35 XYZ vectors");
  for (uint16_t pattern = 0; pattern < 105U; ++pattern) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    TEST_ASSERT_TRUE(driver.bind(makeDriverConfig(bus)).ok());
    DeviceProfile profile = motionProfile();
    profile.fifo.gyroDecimation = FifoDecimation::EVERY_3;
    profile.fifo.accelDecimation = FifoDecimation::EVERY_32;
    (void)fifoConfigure(driver, bus, profile);
    // Both selected sensors occur in the first six words; consume only their
    // required transition vectors before the independently seeded backlog.
    bus.setFifo(6, 0, 10);
    bus.fifoPatternWords = 105;
    FifoMotionRecord records[35];
    const OperationResult transition = fifoRead(driver, bus, records, 35, 6);
    TEST_ASSERT_TRUE(transition.status.ok());
    TEST_ASSERT_EQUAL_UINT16(0, transition.fifoRead.records);
    TEST_ASSERT_EQUAL_UINT16(6, transition.fifoRead.wordsDiscarded);
    bus.clearTrace();
    bus.setFifo(105, pattern, 1000);
    const OperationResult result = fifoRead(driver, bus, records, 35, 105);
    TEST_ASSERT_TRUE_MESSAGE(result.status.ok(), result.status.msg);
    const uint16_t leading = static_cast<uint16_t>((3U - pattern % 3U) % 3U);
    const uint16_t complete = static_cast<uint16_t>((105U - leading) / 3U);
    TEST_ASSERT_EQUAL_UINT16(complete, result.fifoRead.records);
    TEST_ASSERT_EQUAL_UINT16(105, result.fifoRead.wordsConsumed);
    TEST_ASSERT_EQUAL_UINT16(105, result.fifoRead.wordsPossiblyConsumed);
    TEST_ASSERT_EQUAL_UINT16(105U - 3U * complete, result.fifoRead.wordsDiscarded);
    TEST_ASSERT_EQUAL_UINT16(pattern, result.fifoRead.before.pattern);
    TEST_ASSERT_EQUAL_UINT16(pattern, result.fifoRead.after.pattern);
    TEST_ASSERT_FALSE(result.fifoRead.patternDiscontinuity);
    TEST_ASSERT_FALSE(result.fifoRead.overrunObserved);
    for (uint16_t i = 0; i < complete; ++i) {
      const uint16_t vector = static_cast<uint16_t>(
          ((pattern + leading + 3U * i) % 105U) / 3U);
      TEST_ASSERT_EQUAL_UINT8(VECTORS[vector] == 'G' ? SAMPLE_ANGULAR_RATE :
                                                    SAMPLE_ACCELERATION,
                            records[i].quantityMask);
      TEST_ASSERT_EQUAL_INT16(1000U + leading + 3U * i, records[i].raw.x);
      TEST_ASSERT_EQUAL_INT16(1001U + leading + 3U * i, records[i].raw.y);
      TEST_ASSERT_EQUAL_INT16(1002U + leading + 3U * i, records[i].raw.z);
      TEST_ASSERT_EQUAL_UINT64(result.token.value, records[i].token.value);
    }
  }
}

void test_fifo_continuous_post_burst_headroom_boundary_is_exact() {
  for (uint16_t initial : {2041, 2042}) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    warmFifo(driver, bus);
    bus.setFifo(initial, 0, 101);
    bus.fifoPatternWords = 6;
    FifoMotionRecord records[6];
    const OperationResult result = fifoRead(driver, bus, records, 6, 16);
    TEST_ASSERT_EQUAL_UINT16(16, result.fifoRead.wordsConsumed);
    TEST_ASSERT_EQUAL_UINT16(initial - 16U, result.fifoRead.after.unreadWords);
    TEST_ASSERT_FALSE(result.fifoRead.overrunObserved);
    TEST_ASSERT_FALSE(result.fifoRead.patternDiscontinuity);
    if (initial == 2041U) {
      // 2025 retained words proves the 16-word burst did not cross the
      // conservative full floor. Exactly 2026 cannot provide that proof.
      TEST_ASSERT_TRUE_MESSAGE(result.status.ok(), result.status.msg);
      TEST_ASSERT_EQUAL_UINT16(5, result.fifoRead.records);
      TEST_ASSERT_EQUAL_UINT16(1, result.fifoRead.wordsDiscarded);
    } else {
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::OPERATION_INDETERMINATE),
                              static_cast<uint8_t>(result.status.code));
      TEST_ASSERT_EQUAL_UINT16(0, result.fifoRead.records);
      TEST_ASSERT_TRUE(result.fifoRead.continuityLost);
      assertOutputCleared(records, 6);
    }
  }
}

void test_fifo_reset_boot_recover_restore_active_profile_at_exact_ceiling() {
  for (uint8_t operation = 0; operation < 3U; ++operation) {
    FakeBus bus;
    LSM6DS3TR::LSM6DS3TR driver;
    DeviceProfile profile = motionProfile();
    profile.accelFullScale = AccelFs::G_4;
    profile.gyroFullScale = GyroFs::DPS_1000;
    warmFifo(driver, bus, profile);
    bus.setFifo(6, 0, 100);
    bus.fifoPatternWords = 6;
    OperationToken token;
    TEST_ASSERT_TRUE(driver.startFifoPurge({6, true}, fifoTiming(bus), token).inProgress());
    fifoRun(driver, bus);
    const OperationResult stopped = fifoTake(driver, token);
    TEST_ASSERT_TRUE(stopped.status.ok());
    TEST_ASSERT_TRUE(stopped.fifoPurge.collectionStopped);
    const uint32_t generation = driver.configGeneration();
    bus.clearTrace();
    if (operation == 1U) bus.bootBitReadsRemaining = 15;
    else bus.resetBitReadsRemaining = 15;
    bus.resetRegistersOnSoftwareReset = true;
    const Status start = operation == 0U ? driver.startReset(fifoTiming(bus), token) :
                         operation == 1U ? driver.startBoot(fifoTiming(bus), token) :
                                           driver.startRecover(fifoTiming(bus), token);
    TEST_ASSERT_TRUE(start.inProgress());
    fifoRun(driver, bus);
    const OperationResult result = fifoTake(driver, token);
    TEST_ASSERT_TRUE_MESSAGE(result.status.ok(), result.status.msg);
    const uint32_t ceiling = operation == 2U ? MAX_RECOVER_TRANSACTIONS :
                                               MAX_RESET_TRANSACTIONS;
    TEST_ASSERT_EQUAL_UINT32(ceiling, result.transactions);
    TEST_ASSERT_EQUAL_UINT32(ceiling, result.transactionLimit);
    TEST_ASSERT_EQUAL_UINT32(ceiling, bus.transferCalls);
    TEST_ASSERT_EQUAL_UINT32(generation + 1U, driver.configGeneration());
    TEST_ASSERT_EQUAL_HEX8(0x26, bus.regs[0x0A]);
    TEST_ASSERT_EQUAL_HEX8(0x48, bus.regs[0x10]);
    TEST_ASSERT_EQUAL_HEX8(0x48, bus.regs[0x11]);
    TEST_ASSERT_EQUAL_UINT16(0, bus.fifoUnreadWords);
    bus.setFifo(12, 0, 800);
    bus.fifoPatternWords = 6;
    FifoMotionRecord records[4];
    const OperationResult read = fifoRead(driver, bus, records, 4, 12);
    TEST_ASSERT_TRUE(read.status.ok());
    TEST_ASSERT_EQUAL_UINT16(6, read.fifoRead.wordsDiscarded);
    TEST_ASSERT_EQUAL_UINT16(2, read.fifoRead.records);
    TEST_ASSERT_EQUAL_INT16(806, records[0].raw.x);
    TEST_ASSERT_EQUAL_UINT32(result.configuration.generation, records[0].configGeneration);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelFs::G_4),
                            static_cast<uint8_t>(records[1].accelFullScale));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(GyroFs::DPS_1000),
                            static_cast<uint8_t>(records[0].gyroFullScale));
  }
}

} // namespace

void runFifoTests() {
  RUN_TEST(test_fifo_patterns_match_independent_vendor_vectors);
  RUN_TEST(test_fifo_profile_validation_is_total_atomic_and_bus_silent);
  RUN_TEST(test_fifo_wire_image_and_activation_follow_verified_settling);
  RUN_TEST(test_fifo_records_are_complete_signed_and_keep_immutable_provenance);
  RUN_TEST(test_fifo_every_starting_axis_and_pattern_wrap_has_explicit_discard);
  RUN_TEST(test_fifo_short_buffer_word_bound_and_partial_vectors_are_bounded);
  RUN_TEST(test_fifo_empty_full_overrun_and_contradictions_never_read_empty);
  RUN_TEST(test_fifo_concurrent_arrivals_remain_for_next_bounded_batch);
  RUN_TEST(test_fifo_pattern_discontinuity_invalidates_entire_output);
  RUN_TEST(test_fifo_transport_failure_at_every_stage_never_retries_or_publishes);
  RUN_TEST(test_fifo_cancellation_deadline_and_invalid_requests_have_no_stale_batch);
  RUN_TEST(test_fifo_buffer_lifetime_tokens_backpressure_unbind_and_rebind);
  RUN_TEST(test_fifo_configure_faults_and_cancellation_cover_activation_stages);
  RUN_TEST(test_fifo_reconcile_is_read_only_at_every_failure_stage_and_cannot_relabel_unknown_data);
  RUN_TEST(test_fifo_live_collection_blocks_profile_and_lifecycle_mutations);
  RUN_TEST(test_fifo_explicit_stop_reports_residual_loss_and_recovery_reestablishes_schema);
  RUN_TEST(test_fifo_transaction_ceiling_guards_consumption_before_callback);
  RUN_TEST(test_fifo_single_sensor_wire_images_and_records);
  RUN_TEST(test_fifo_transition_discard_and_stream_loss_persist_across_calls);
  RUN_TEST(test_fifo_identity_and_post_read_empty_contradiction_invalidate_batch);
  RUN_TEST(test_fifo_transient_overwrite_with_whole_pattern_skip_never_publishes);
  RUN_TEST(test_fifo_explicit_stop_all_fault_cancel_and_ceiling_stages_preserve_effects);
  RUN_TEST(test_fifo_every_motion_decimator_has_an_independent_literal_wire_image);
  RUN_TEST(test_fifo_acquisition_decodes_every_start_in_the_maximum_pattern);
  RUN_TEST(test_fifo_continuous_post_burst_headroom_boundary_is_exact);
  RUN_TEST(test_fifo_reset_boot_recover_restore_active_profile_at_exact_ceiling);
}
