/// @file test_basic.cpp
/// @brief Native contract tests for LSM6DS3TR lifecycle, validation, and helpers.

#include <unity.h>

#include <cstring>

#include "Arduino.h"
#include "Wire.h"

SerialClass Serial;
TwoWire Wire;

#include "LSM6DS3TR/LSM6DS3TR.h"
#include "common/I2cTransport.h"

using namespace LSM6DS3TR;

namespace {

struct FakeBus {
  uint32_t nowMs = 1000;
  uint32_t writeCalls = 0;
  uint32_t readCalls = 0;

  int readErrorRemaining = 0;
  int writeErrorRemaining = 0;
  uint32_t failWriteCall = 0;
  Status readError = Status::Error(Err::I2C_ERROR, "forced read error", -1);
  Status writeError = Status::Error(Err::I2C_ERROR, "forced write error", -2);

  uint32_t notReadyStatusReads = 0;
  uint8_t regs[256] = {};

  FakeBus() {
    resetRegisters();
  }

  void resetRegisters() {
    std::memset(regs, 0, sizeof(regs));
    regs[cmd::REG_WHO_AM_I] = cmd::WHO_AM_I_VALUE;
    regs[cmd::REG_STATUS_REG] = static_cast<uint8_t>(cmd::MASK_XLDA | cmd::MASK_GDA | cmd::MASK_TDA);
    regs[cmd::REG_CTRL3_C] = static_cast<uint8_t>(cmd::MASK_IF_INC | cmd::MASK_BDU);
    setMeasurementData();
    setTimestamp(0x123456u);
    setStepCounter(0x1234u);
    setStepTimestamp(0x5678u);
    setFifoEmpty();
  }

  void setMeasurementData() {
    setTemperatureRaw(6400);
    setGyroRaw(1000, -500, 0);
    setAccelRaw(16384, 0, 0);
  }

  void setTemperatureRaw(int16_t value) {
    regs[cmd::REG_OUT_TEMP_L] = static_cast<uint8_t>(value & 0xFFu);
    regs[cmd::REG_OUT_TEMP_H] = static_cast<uint8_t>((static_cast<uint16_t>(value) >> 8) & 0xFFu);
  }

  void setGyroRaw(int16_t x, int16_t y, int16_t z) {
    regs[cmd::REG_OUTX_L_G] = static_cast<uint8_t>(x & 0xFF);
    regs[cmd::REG_OUTX_H_G] = static_cast<uint8_t>((static_cast<uint16_t>(x) >> 8) & 0xFFu);
    regs[cmd::REG_OUTY_L_G] = static_cast<uint8_t>(y & 0xFF);
    regs[cmd::REG_OUTY_H_G] = static_cast<uint8_t>((static_cast<uint16_t>(y) >> 8) & 0xFFu);
    regs[cmd::REG_OUTZ_L_G] = static_cast<uint8_t>(z & 0xFF);
    regs[cmd::REG_OUTZ_H_G] = static_cast<uint8_t>((static_cast<uint16_t>(z) >> 8) & 0xFFu);
  }

  void setAccelRaw(int16_t x, int16_t y, int16_t z) {
    regs[cmd::REG_OUTX_L_XL] = static_cast<uint8_t>(x & 0xFF);
    regs[cmd::REG_OUTX_H_XL] = static_cast<uint8_t>((static_cast<uint16_t>(x) >> 8) & 0xFFu);
    regs[cmd::REG_OUTY_L_XL] = static_cast<uint8_t>(y & 0xFF);
    regs[cmd::REG_OUTY_H_XL] = static_cast<uint8_t>((static_cast<uint16_t>(y) >> 8) & 0xFFu);
    regs[cmd::REG_OUTZ_L_XL] = static_cast<uint8_t>(z & 0xFF);
    regs[cmd::REG_OUTZ_H_XL] = static_cast<uint8_t>((static_cast<uint16_t>(z) >> 8) & 0xFFu);
  }

  void setTimestamp(uint32_t value) {
    regs[cmd::REG_TIMESTAMP0] = static_cast<uint8_t>(value & 0xFFu);
    regs[cmd::REG_TIMESTAMP1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
    regs[cmd::REG_TIMESTAMP2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
  }

  void setStepCounter(uint16_t value) {
    regs[cmd::REG_STEP_COUNTER_L] = static_cast<uint8_t>(value & 0xFFu);
    regs[cmd::REG_STEP_COUNTER_H] = static_cast<uint8_t>((value >> 8) & 0xFFu);
  }

  void setStepTimestamp(uint16_t value) {
    regs[cmd::REG_STEP_TIMESTAMP_L] = static_cast<uint8_t>(value & 0xFFu);
    regs[cmd::REG_STEP_TIMESTAMP_H] = static_cast<uint8_t>((value >> 8) & 0xFFu);
  }

  void setFifoEmpty() {
    regs[cmd::REG_FIFO_STATUS1] = 0x00;
    regs[cmd::REG_FIFO_STATUS2] = cmd::MASK_FIFO_EMPTY;
    regs[cmd::REG_FIFO_STATUS3] = 0x00;
    regs[cmd::REG_FIFO_STATUS4] = 0x00;
    regs[cmd::REG_FIFO_DATA_OUT_L] = 0x00;
    regs[cmd::REG_FIFO_DATA_OUT_H] = 0x00;
  }

  void setFifoStatus(uint16_t unreadWords, uint16_t pattern, uint16_t nextWord) {
    regs[cmd::REG_FIFO_STATUS1] = static_cast<uint8_t>(unreadWords & 0xFFu);
    regs[cmd::REG_FIFO_STATUS2] = static_cast<uint8_t>((unreadWords >> 8) & cmd::MASK_DIFF_FIFO_HI);
    if (unreadWords == 0u) {
      regs[cmd::REG_FIFO_STATUS2] |= cmd::MASK_FIFO_EMPTY;
    }
    regs[cmd::REG_FIFO_STATUS3] = static_cast<uint8_t>(pattern & 0xFFu);
    regs[cmd::REG_FIFO_STATUS4] = static_cast<uint8_t>((pattern >> 8) & 0x03u);
    regs[cmd::REG_FIFO_DATA_OUT_L] = static_cast<uint8_t>(nextWord & 0xFFu);
    regs[cmd::REG_FIFO_DATA_OUT_H] = static_cast<uint8_t>((nextWord >> 8) & 0xFFu);
  }
};

Status fakeWrite(uint8_t, const uint8_t* data, size_t len, uint32_t, void* user) {
  FakeBus* bus = static_cast<FakeBus*>(user);
  bus->writeCalls++;
  if (data == nullptr || len < 2u) {
    return Status::Error(Err::INVALID_PARAM, "invalid fake write args");
  }
  if (bus->writeErrorRemaining > 0) {
    bus->writeErrorRemaining--;
    return bus->writeError;
  }
  if (bus->failWriteCall != 0u && bus->writeCalls == bus->failWriteCall) {
    bus->failWriteCall = 0;
    return bus->writeError;
  }

  const uint8_t startReg = data[0];
  for (size_t i = 1; i < len; ++i) {
    const uint8_t reg = static_cast<uint8_t>(startReg + (i - 1u));
    const uint8_t value = data[i];

    if (reg == cmd::REG_CTRL3_C) {
      bus->regs[reg] = static_cast<uint8_t>(value & ~(cmd::MASK_SW_RESET | cmd::MASK_BOOT));
      continue;
    }

    if (reg == cmd::REG_TIMESTAMP2 && value == cmd::TIMESTAMP_RESET_VALUE) {
      bus->setTimestamp(0u);
      continue;
    }

    if (reg == cmd::REG_CTRL10_C && (value & (1u << cmd::BIT_PEDO_RST_STEP)) != 0u) {
      bus->regs[reg] = value;
      bus->setStepCounter(0u);
      continue;
    }

    bus->regs[reg] = value;
  }

  return Status::Ok();
}

Status fakeWriteRead(uint8_t, const uint8_t* txData, size_t txLen, uint8_t* rxData,
                     size_t rxLen, uint32_t, void* user) {
  FakeBus* bus = static_cast<FakeBus*>(user);
  bus->readCalls++;
  if (txData == nullptr || txLen == 0u || (rxLen > 0u && rxData == nullptr)) {
    return Status::Error(Err::INVALID_PARAM, "invalid fake write-read args");
  }
  if (bus->readErrorRemaining > 0) {
    bus->readErrorRemaining--;
    return bus->readError;
  }

  const uint8_t startReg = txData[0];
  for (size_t i = 0; i < rxLen; ++i) {
    rxData[i] = bus->regs[static_cast<uint8_t>(startReg + i)];
  }

  if (startReg == cmd::REG_STATUS_REG && rxLen >= 1u && bus->notReadyStatusReads > 0u) {
    rxData[0] = 0x00;
    bus->notReadyStatusReads--;
  }

  return Status::Ok();
}

uint32_t fakeNowMs(void* user) {
  return static_cast<FakeBus*>(user)->nowMs;
}

Config makeConfig(FakeBus& bus) {
  Config cfg;
  cfg.i2cWrite = fakeWrite;
  cfg.i2cWriteRead = fakeWriteRead;
  cfg.i2cUser = &bus;
  cfg.nowMs = fakeNowMs;
  cfg.timeUser = &bus;
  cfg.i2cTimeoutMs = 10;
  cfg.offlineThreshold = 3;
  cfg.odrXl = Odr::HZ_104;
  cfg.odrG = Odr::HZ_104;
  cfg.fsXl = AccelFs::G_2;
  cfg.fsG = GyroFs::DPS_250;
  cfg.accelPowerMode = AccelPowerMode::HIGH_PERFORMANCE;
  cfg.gyroPowerMode = GyroPowerMode::HIGH_PERFORMANCE;
  cfg.bdu = true;
  return cfg;
}

}  // namespace

void setUp() {
  setMillis(0);
  Wire._clearEndTransmissionResult();
  Wire._clearRequestFromOverride();
}

void tearDown() {}

// ==========================================================================
// Status tests
// ==========================================================================

void test_status_ok() {
  const Status st = Status::Ok();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::OK), static_cast<uint8_t>(st.code));
}

void test_status_error() {
  const Status st = Status::Error(Err::I2C_ERROR, "Test error", 42);
  TEST_ASSERT_FALSE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(42, st.detail);
}

void test_status_in_progress() {
  const Status st{Err::IN_PROGRESS, 0, "In progress"};
  TEST_ASSERT_FALSE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS), static_cast<uint8_t>(st.code));
}

// ==========================================================================
// Config default tests
// ==========================================================================

void test_config_defaults() {
  const Config cfg;
  TEST_ASSERT_NULL(cfg.i2cWrite);
  TEST_ASSERT_NULL(cfg.i2cWriteRead);
  TEST_ASSERT_EQUAL_HEX8(0x6A, cfg.i2cAddress);
  TEST_ASSERT_EQUAL_UINT32(50u, cfg.i2cTimeoutMs);
  TEST_ASSERT_EQUAL_UINT8(5u, cfg.offlineThreshold);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Odr::HZ_104), static_cast<uint8_t>(cfg.odrXl));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Odr::HZ_104), static_cast<uint8_t>(cfg.odrG));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelFs::G_2), static_cast<uint8_t>(cfg.fsXl));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(GyroFs::DPS_250), static_cast<uint8_t>(cfg.fsG));
  TEST_ASSERT_TRUE(cfg.bdu);
}

// ==========================================================================
// Lifecycle and validation tests
// ==========================================================================

void test_begin_rejects_missing_callbacks() {
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg;
  const Status st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::UNINIT),
                          static_cast<uint8_t>(dev.state()));
}

void test_begin_success_sets_ready_and_health() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  const Status st = dev.begin(makeConfig(bus));
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_TRUE(dev.isOnline());
  TEST_ASSERT_EQUAL_UINT32(0u, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(0u, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(0u, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT32(0u, dev.lastOkMs());
}

void test_begin_rejects_invalid_address() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg = makeConfig(bus);
  cfg.i2cAddress = 0x50;
  const Status st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG), static_cast<uint8_t>(st.code));
}

void test_begin_rejects_accel_1_6_in_high_performance() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg = makeConfig(bus);
  cfg.odrXl = Odr::HZ_1_6;
  const Status st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG), static_cast<uint8_t>(st.code));
}

void test_begin_rejects_accel_416_in_low_power_normal() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg = makeConfig(bus);
  cfg.accelPowerMode = AccelPowerMode::LOW_POWER_NORMAL;
  cfg.odrXl = Odr::HZ_416;
  const Status st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG), static_cast<uint8_t>(st.code));
}

void test_begin_rejects_gyro_1_6() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg = makeConfig(bus);
  cfg.odrG = Odr::HZ_1_6;
  const Status st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG), static_cast<uint8_t>(st.code));
}

void test_begin_rejects_gyro_833_in_low_power_normal() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg = makeConfig(bus);
  cfg.gyroPowerMode = GyroPowerMode::LOW_POWER_NORMAL;
  cfg.odrG = Odr::HZ_833;
  const Status st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG), static_cast<uint8_t>(st.code));
}

void test_begin_rejects_invalid_power_mode_enums() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg = makeConfig(bus);
  cfg.accelPowerMode = static_cast<AccelPowerMode>(99);

  Status st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG), static_cast<uint8_t>(st.code));

  cfg = makeConfig(bus);
  cfg.gyroPowerMode = static_cast<GyroPowerMode>(99);
  st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG), static_cast<uint8_t>(st.code));
}

void test_begin_detects_wrong_chip_id() {
  FakeBus bus;
  bus.regs[cmd::REG_WHO_AM_I] = 0x00;
  LSM6DS3TR::LSM6DS3TR dev;
  const Status st = dev.begin(makeConfig(bus));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CHIP_ID_MISMATCH),
                          static_cast<uint8_t>(st.code));
}

void test_probe_after_failed_begin_uses_stored_transport() {
  FakeBus bus;
  bus.regs[cmd::REG_WHO_AM_I] = 0x00;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CHIP_ID_MISMATCH),
                          static_cast<uint8_t>(dev.begin(makeConfig(bus)).code));

  bus.regs[cmd::REG_WHO_AM_I] = cmd::WHO_AM_I_VALUE;
  const Status st = dev.probe();
  TEST_ASSERT_TRUE(st.ok());
}

void test_recover_after_failed_begin_retries_begin() {
  FakeBus bus;
  bus.regs[cmd::REG_WHO_AM_I] = 0x00;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CHIP_ID_MISMATCH),
                          static_cast<uint8_t>(dev.begin(makeConfig(bus)).code));

  bus.regs[cmd::REG_WHO_AM_I] = cmd::WHO_AM_I_VALUE;
  const Status st = dev.recover();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
}

void test_now_ms_fallback_uses_millis_when_callback_missing() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg = makeConfig(bus);
  cfg.nowMs = nullptr;
  cfg.timeUser = nullptr;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  setMillis(4321);
  const Status st = dev.recover();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT32(4321u, dev.lastOkMs());
}

// ==========================================================================
// Diagnostics tests
// ==========================================================================

void test_probe_failure_does_not_update_health() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  const uint32_t beforeSuccess = dev.totalSuccess();
  const uint32_t beforeFailures = dev.totalFailures();
  const DriverState beforeState = dev.state();

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_ERROR, "forced probe error", -7);
  const Status st = dev.probe();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::DEVICE_NOT_FOUND),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(beforeSuccess, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(beforeFailures, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(beforeState),
                          static_cast<uint8_t>(dev.state()));
}

void test_recover_failure_updates_health_once() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_ERROR, "forced recover error", -8);
  const Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(1u, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(1u, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR),
                          static_cast<uint8_t>(dev.lastError().code));
  TEST_ASSERT_EQUAL_UINT32(bus.nowMs, dev.lastErrorMs());
}

void test_recover_chip_id_mismatch_updates_health_once() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.nowMs = 2222;
  bus.regs[cmd::REG_WHO_AM_I] = 0x00;
  const Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CHIP_ID_MISMATCH),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(1u, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(1u, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::CHIP_ID_MISMATCH),
                          static_cast<uint8_t>(dev.lastError().code));
  TEST_ASSERT_EQUAL_INT32(0, dev.lastError().detail);
  TEST_ASSERT_EQUAL_UINT32(2222u, dev.lastErrorMs());
}

void test_recover_success_returns_ready() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_ERROR, "forced recover error", -9);
  (void)dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));

  bus.nowMs = 4321;
  const Status st = dev.recover();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(0u, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT32(4321u, dev.lastOkMs());
}

void test_recover_reaches_offline_when_threshold_is_one() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg = makeConfig(bus);
  cfg.offlineThreshold = 1;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_ERROR, "forced timeout", -10);
  const Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_FALSE(dev.isOnline());
}

// ==========================================================================
// Measurement tests
// ==========================================================================

void test_read_all_raw_returns_data() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  RawMeasurement raw{};
  const Status st = dev.readAllRaw(raw);
  TEST_ASSERT_TRUE(st.ok());

  TEST_ASSERT_EQUAL_INT16(16384, raw.accel.x);
  TEST_ASSERT_EQUAL_INT16(0, raw.accel.y);
  TEST_ASSERT_EQUAL_INT16(0, raw.accel.z);
  TEST_ASSERT_EQUAL_INT16(1000, raw.gyro.x);
  TEST_ASSERT_EQUAL_INT16(-500, raw.gyro.y);
  TEST_ASSERT_EQUAL_INT16(0, raw.gyro.z);
  TEST_ASSERT_EQUAL_INT16(6400, raw.temperature);
}

void test_convert_accel_at_2g() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  const RawAxes raw = {16384, 0, 0};
  const Axes result = dev.convertAccel(raw);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.999f, result.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, result.y);
}

void test_convert_gyro_at_250dps() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  const RawAxes raw = {1000, -500, 0};
  const Axes result = dev.convertGyro(raw);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 8.75f, result.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -4.375f, result.y);
}

void test_convert_temperature() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, dev.convertTemperature(6400));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 25.0f, dev.convertTemperature(0));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, dev.convertTemperature(-6400));
}

void test_request_measurement_and_tick() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  Status st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS), static_cast<uint8_t>(st.code));
  TEST_ASSERT_FALSE(dev.measurementReady());

  dev.tick(bus.nowMs);
  TEST_ASSERT_TRUE(dev.measurementReady());

  Measurement m{};
  st = dev.getMeasurement(m);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_FALSE(dev.measurementReady());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.999f, m.accel.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, m.temperatureC);
}

void test_request_measurement_rejects_without_bdu_for_combined_async() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg = makeConfig(bus);
  cfg.bdu = false;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  const Status st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM), static_cast<uint8_t>(st.code));
}

void test_request_measurement_rejects_mismatched_active_odr() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg = makeConfig(bus);
  cfg.odrG = Odr::HZ_208;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  const Status st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM), static_cast<uint8_t>(st.code));
}

void test_measurement_not_ready_before_tick() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  bus.notReadyStatusReads = 2;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  const Status st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS), static_cast<uint8_t>(st.code));

  dev.tick(bus.nowMs);
  TEST_ASSERT_FALSE(dev.measurementReady());
  dev.tick(bus.nowMs);
  TEST_ASSERT_FALSE(dev.measurementReady());
  dev.tick(bus.nowMs);
  TEST_ASSERT_TRUE(dev.measurementReady());
}

void test_get_raw_measurement_after_read() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  RawMeasurement raw{};
  Status st = dev.getRawMeasurement(raw);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::MEASUREMENT_NOT_READY),
                          static_cast<uint8_t>(st.code));

  st = dev.readAllRaw(raw);
  TEST_ASSERT_TRUE(st.ok());

  st = dev.requestMeasurement();
  dev.tick(bus.nowMs);
  TEST_ASSERT_TRUE(dev.measurementReady());

  st = dev.getRawMeasurement(raw);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_INT16(16384, raw.accel.x);
}

void test_get_measurement_applies_manual_bias() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  Axes accelBias;
  accelBias.x = 0.100f;
  accelBias.y = 0.200f;
  accelBias.z = 0.300f;
  dev.setAccelBias(accelBias);

  Axes gyroBias;
  gyroBias.x = 1.25f;
  gyroBias.y = -0.50f;
  gyroBias.z = 0.25f;
  dev.setGyroBias(gyroBias);

  Status st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS), static_cast<uint8_t>(st.code));

  dev.tick(bus.nowMs);
  TEST_ASSERT_TRUE(dev.measurementReady());

  Measurement m{};
  st = dev.getMeasurement(m);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.899f, m.accel.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.200f, m.accel.y);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.300f, m.accel.z);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 7.50f, m.gyro.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -3.875f, m.gyro.y);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.25f, m.gyro.z);
}

void test_capture_accel_bias_auto_applies_to_measurements() {
  FakeBus bus;
  bus.setAccelRaw(0, 0, 16384);
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  Axes bias{};
  Status st = dev.captureAccelBias(4, bias);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, bias.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, bias.y);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, bias.z);

  const Axes storedBias = dev.accelBias();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, bias.x, storedBias.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, bias.y, storedBias.y);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, bias.z, storedBias.z);

  st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS), static_cast<uint8_t>(st.code));
  dev.tick(bus.nowMs);

  Measurement m{};
  st = dev.getMeasurement(m);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, m.accel.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, m.accel.y);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, m.accel.z);
}

void test_capture_gyro_bias_auto_applies_to_measurements() {
  FakeBus bus;
  bus.setGyroRaw(100, -50, 25);
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  Axes bias{};
  Status st = dev.captureGyroBias(4, bias);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.875f, bias.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.4375f, bias.y);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.21875f, bias.z);

  const Axes storedBias = dev.gyroBias();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, bias.x, storedBias.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, bias.y, storedBias.y);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, bias.z, storedBias.z);

  st = dev.requestMeasurement();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS), static_cast<uint8_t>(st.code));
  dev.tick(bus.nowMs);

  Measurement m{};
  st = dev.getMeasurement(m);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, m.gyro.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, m.gyro.y);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, m.gyro.z);
}

void test_capture_bias_times_out_with_stalled_clock_and_no_data_ready() {
  FakeBus bus;
  bus.regs[cmd::REG_STATUS_REG] = 0x00;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  Axes bias{};
  Status st = dev.captureAccelBias(1, bias);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT), static_cast<uint8_t>(st.code));

  st = dev.captureGyroBias(1, bias);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT), static_cast<uint8_t>(st.code));
}

// ==========================================================================
// Configuration and feature tests
// ==========================================================================

void test_set_accel_odr() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  TEST_ASSERT_TRUE(dev.setAccelOdr(Odr::HZ_208).ok());

  Odr odr = Odr::POWER_DOWN;
  TEST_ASSERT_TRUE(dev.getAccelOdr(odr).ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Odr::HZ_208), static_cast<uint8_t>(odr));
}

void test_set_gyro_fs() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  TEST_ASSERT_TRUE(dev.setGyroFs(GyroFs::DPS_500).ok());

  GyroFs fs = GyroFs::DPS_250;
  TEST_ASSERT_TRUE(dev.getGyroFs(fs).ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(GyroFs::DPS_500), static_cast<uint8_t>(fs));
}

void test_sensitivity_helpers() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg = makeConfig(bus);
  cfg.fsXl = AccelFs::G_4;
  cfg.fsG = GyroFs::DPS_125;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.122f, dev.accelSensitivity());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.375f, dev.gyroSensitivity());
}

void test_set_accel_power_mode_rejects_unsupported_current_odr() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg = makeConfig(bus);
  cfg.odrXl = Odr::HZ_416;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  const Status st = dev.setAccelPowerMode(AccelPowerMode::LOW_POWER_NORMAL);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM), static_cast<uint8_t>(st.code));
}

void test_set_gyro_power_mode_rejects_unsupported_current_odr() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg = makeConfig(bus);
  cfg.odrG = Odr::HZ_833;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  const Status st = dev.setGyroPowerMode(GyroPowerMode::LOW_POWER_NORMAL);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM), static_cast<uint8_t>(st.code));
}

void test_power_mode_setters_reject_invalid_enums() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
  const uint32_t writesBefore = bus.writeCalls;

  Status st = dev.setAccelPowerMode(static_cast<AccelPowerMode>(99));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM), static_cast<uint8_t>(st.code));
  st = dev.setGyroPowerMode(static_cast<GyroPowerMode>(99));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(writesBefore, bus.writeCalls);
}

void test_cached_setters_roll_back_after_i2c_failure() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.writeErrorRemaining = 1;
  Status st = dev.setAccelPowerMode(AccelPowerMode::LOW_POWER_NORMAL);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR), static_cast<uint8_t>(st.code));
  AccelPowerMode accelMode = AccelPowerMode::LOW_POWER_NORMAL;
  TEST_ASSERT_TRUE(dev.getAccelPowerMode(accelMode).ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelPowerMode::HIGH_PERFORMANCE),
                          static_cast<uint8_t>(accelMode));

  TEST_ASSERT_TRUE(dev.setGyroSleepEnabled(true).ok());
  bus.writeErrorRemaining = 1;
  st = dev.setGyroSleepEnabled(false);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR), static_cast<uint8_t>(st.code));
  bool sleepEnabled = false;
  TEST_ASSERT_TRUE(dev.getGyroSleepEnabled(sleepEnabled).ok());
  TEST_ASSERT_TRUE(sleepEnabled);
}

void test_multi_register_setters_roll_back_cache_after_partial_i2c_failure() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  GyroFilterConfig gyroFilter;
  gyroFilter.lpf1Enabled = true;
  gyroFilter.highPassEnabled = true;
  gyroFilter.highPassMode = GyroHpfMode::HZ_2_07;
  bus.failWriteCall = bus.writeCalls + 2u;
  Status st = dev.setGyroFilterConfig(gyroFilter);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR), static_cast<uint8_t>(st.code));

  GyroFilterConfig gyroFilterOut;
  TEST_ASSERT_TRUE(dev.getGyroFilterConfig(gyroFilterOut).ok());
  TEST_ASSERT_FALSE(gyroFilterOut.lpf1Enabled);
  TEST_ASSERT_FALSE(gyroFilterOut.highPassEnabled);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(GyroHpfMode::HZ_0_0081),
                          static_cast<uint8_t>(gyroFilterOut.highPassMode));

  FifoConfig fifo;
  fifo.threshold = 0x120u;
  fifo.odr = Odr::HZ_104;
  fifo.mode = FifoMode::CONTINUOUS;
  fifo.accelDecimation = FifoDecimation::NONE;
  fifo.gyroDecimation = FifoDecimation::DIV_2;
  bus.failWriteCall = bus.writeCalls + 3u;
  st = dev.configureFifo(fifo);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR), static_cast<uint8_t>(st.code));

  FifoConfig fifoOut;
  TEST_ASSERT_TRUE(dev.getFifoConfig(fifoOut).ok());
  TEST_ASSERT_EQUAL_UINT16(0u, fifoOut.threshold);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(FifoMode::BYPASS),
                          static_cast<uint8_t>(fifoOut.mode));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Odr::POWER_DOWN),
                          static_cast<uint8_t>(fifoOut.odr));
}

void test_timestamp_requires_active_sensor() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg = makeConfig(bus);
  cfg.odrXl = Odr::POWER_DOWN;
  cfg.odrG = Odr::POWER_DOWN;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  const Status st = dev.setTimestampEnabled(true);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM), static_cast<uint8_t>(st.code));
}

void test_embedded_functions_require_accel_odr_at_least_26hz() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg = makeConfig(bus);
  cfg.odrXl = Odr::HZ_12_5;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  const Status st = dev.setPedometerEnabled(true);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM), static_cast<uint8_t>(st.code));
}

void test_set_accel_odr_rejects_disabling_embedded_function_support() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
  TEST_ASSERT_TRUE(dev.setPedometerEnabled(true).ok());

  const Status st = dev.setAccelOdr(Odr::HZ_12_5);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM), static_cast<uint8_t>(st.code));
}

void test_timestamp_helpers_work() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  uint32_t timestamp = 0;
  TEST_ASSERT_TRUE(dev.readTimestamp(timestamp).ok());
  TEST_ASSERT_EQUAL_UINT32(0x123456u, timestamp);

  TEST_ASSERT_TRUE(dev.resetTimestamp().ok());
  TEST_ASSERT_TRUE(dev.readTimestamp(timestamp).ok());
  TEST_ASSERT_EQUAL_UINT32(0u, timestamp);
}

void test_step_counter_helpers_work() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
  TEST_ASSERT_TRUE(dev.setPedometerEnabled(true).ok());

  uint16_t steps = 0;
  uint16_t stepTs = 0;
  TEST_ASSERT_TRUE(dev.readStepCounter(steps).ok());
  TEST_ASSERT_TRUE(dev.readStepTimestamp(stepTs).ok());
  TEST_ASSERT_EQUAL_UINT16(0x1234u, steps);
  TEST_ASSERT_EQUAL_UINT16(0x5678u, stepTs);

  TEST_ASSERT_TRUE(dev.resetStepCounter().ok());
  TEST_ASSERT_TRUE(dev.readStepCounter(steps).ok());
  TEST_ASSERT_EQUAL_UINT16(0u, steps);
}

void test_offset_and_filter_helpers_work() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  const AccelFilterConfig accelFilter = {true, true, true};
  const GyroFilterConfig gyroFilter = {true, true, GyroHpfMode::HZ_2_07};
  AccelUserOffset offset;
  offset.x = -4;
  offset.y = 7;
  offset.z = 12;

  TEST_ASSERT_TRUE(dev.setAccelPowerMode(AccelPowerMode::LOW_POWER_NORMAL).ok());
  TEST_ASSERT_TRUE(dev.setGyroPowerMode(GyroPowerMode::LOW_POWER_NORMAL).ok());
  TEST_ASSERT_TRUE(dev.setGyroSleepEnabled(true).ok());
  TEST_ASSERT_TRUE(dev.setAccelFilterConfig(accelFilter).ok());
  TEST_ASSERT_TRUE(dev.setGyroFilterConfig(gyroFilter).ok());
  TEST_ASSERT_TRUE(dev.setAccelOffsetWeight(AccelOffsetWeight::MG_16).ok());
  TEST_ASSERT_TRUE(dev.setAccelUserOffset(offset).ok());

  AccelPowerMode accelPower = AccelPowerMode::HIGH_PERFORMANCE;
  GyroPowerMode gyroPower = GyroPowerMode::HIGH_PERFORMANCE;
  bool gyroSleep = false;
  AccelFilterConfig accelFilterOut;
  GyroFilterConfig gyroFilterOut;
  AccelOffsetWeight weight = AccelOffsetWeight::MG_1;
  AccelUserOffset offsetOut;

  TEST_ASSERT_TRUE(dev.getAccelPowerMode(accelPower).ok());
  TEST_ASSERT_TRUE(dev.getGyroPowerMode(gyroPower).ok());
  TEST_ASSERT_TRUE(dev.getGyroSleepEnabled(gyroSleep).ok());
  TEST_ASSERT_TRUE(dev.getAccelFilterConfig(accelFilterOut).ok());
  TEST_ASSERT_TRUE(dev.getGyroFilterConfig(gyroFilterOut).ok());
  TEST_ASSERT_TRUE(dev.getAccelOffsetWeight(weight).ok());
  TEST_ASSERT_TRUE(dev.getAccelUserOffset(offsetOut).ok());

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelPowerMode::LOW_POWER_NORMAL),
                          static_cast<uint8_t>(accelPower));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(GyroPowerMode::LOW_POWER_NORMAL),
                          static_cast<uint8_t>(gyroPower));
  TEST_ASSERT_TRUE(gyroSleep);
  TEST_ASSERT_TRUE(accelFilterOut.lpf2Enabled);
  TEST_ASSERT_TRUE(accelFilterOut.highPassSlopeEnabled);
  TEST_ASSERT_TRUE(accelFilterOut.lowPassOn6d);
  TEST_ASSERT_TRUE(gyroFilterOut.lpf1Enabled);
  TEST_ASSERT_TRUE(gyroFilterOut.highPassEnabled);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(GyroHpfMode::HZ_2_07),
                          static_cast<uint8_t>(gyroFilterOut.highPassMode));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelOffsetWeight::MG_16),
                          static_cast<uint8_t>(weight));
  TEST_ASSERT_EQUAL_INT8(-4, offsetOut.x);
  TEST_ASSERT_EQUAL_INT8(7, offsetOut.y);
  TEST_ASSERT_EQUAL_INT8(12, offsetOut.z);
}

void test_filter_offset_and_fifo_setters_reject_invalid_enums() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
  const uint32_t writesBefore = bus.writeCalls;

  GyroFilterConfig gyroFilter;
  gyroFilter.highPassMode = static_cast<GyroHpfMode>(99);
  Status st = dev.setGyroFilterConfig(gyroFilter);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM), static_cast<uint8_t>(st.code));

  st = dev.setAccelOffsetWeight(static_cast<AccelOffsetWeight>(99));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM), static_cast<uint8_t>(st.code));

  FifoConfig fifo;
  fifo.odr = Odr::HZ_104;
  fifo.mode = static_cast<FifoMode>(2);
  st = dev.configureFifo(fifo);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM), static_cast<uint8_t>(st.code));

  fifo.mode = FifoMode::FIFO;
  fifo.accelDecimation = static_cast<FifoDecimation>(99);
  st = dev.configureFifo(fifo);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM), static_cast<uint8_t>(st.code));

  TEST_ASSERT_EQUAL_UINT32(writesBefore, bus.writeCalls);
}

void test_fifo_configuration_and_read_helpers_work() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  FifoConfig fifo;
  fifo.threshold = 0x123u;
  fifo.odr = Odr::HZ_104;
  fifo.mode = FifoMode::CONTINUOUS;
  fifo.accelDecimation = FifoDecimation::NONE;
  fifo.gyroDecimation = FifoDecimation::DIV_2;
  fifo.stopOnThreshold = true;
  fifo.onlyHighData = true;
  fifo.storeTemperature = true;
  fifo.storeTimestampStep = true;

  TEST_ASSERT_TRUE(dev.configureFifo(fifo).ok());

  FifoConfig fifoOut;
  TEST_ASSERT_TRUE(dev.getFifoConfig(fifoOut).ok());
  TEST_ASSERT_EQUAL_UINT16(fifo.threshold, fifoOut.threshold);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(fifo.mode), static_cast<uint8_t>(fifoOut.mode));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(fifo.odr), static_cast<uint8_t>(fifoOut.odr));
  TEST_ASSERT_TRUE(fifoOut.stopOnThreshold);
  TEST_ASSERT_TRUE(fifoOut.onlyHighData);
  TEST_ASSERT_TRUE(fifoOut.storeTemperature);
  TEST_ASSERT_TRUE(fifoOut.storeTimestampStep);

  TEST_ASSERT_EQUAL_HEX8(0x23, bus.regs[cmd::REG_FIFO_CTRL1]);
  TEST_ASSERT_EQUAL_HEX8(0x89, bus.regs[cmd::REG_FIFO_CTRL2]);
  TEST_ASSERT_EQUAL_HEX8(0x11, bus.regs[cmd::REG_FIFO_CTRL3]);
  TEST_ASSERT_EQUAL_HEX8(0xC0, bus.regs[cmd::REG_FIFO_CTRL4]);
  TEST_ASSERT_EQUAL_HEX8(0x26, bus.regs[cmd::REG_FIFO_CTRL5]);

  bus.setFifoStatus(3u, 0x12u, 0xBEEFu);
  FifoStatus status;
  TEST_ASSERT_TRUE(dev.readFifoStatus(status).ok());
  TEST_ASSERT_EQUAL_UINT16(3u, status.unreadWords);
  TEST_ASSERT_EQUAL_UINT16(0x12u, status.pattern);
  TEST_ASSERT_FALSE(status.empty);

  uint16_t word = 0;
  TEST_ASSERT_TRUE(dev.readFifoWord(word).ok());
  TEST_ASSERT_EQUAL_UINT16(0xBEEFu, word);
}

void test_fifo_requires_bdu_when_active() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg = makeConfig(bus);
  cfg.bdu = false;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  FifoConfig fifo;
  fifo.odr = Odr::HZ_104;
  fifo.mode = FifoMode::FIFO;
  const Status st = dev.configureFifo(fifo);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM), static_cast<uint8_t>(st.code));
}

void test_read_fifo_word_reports_empty() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  uint16_t word = 0;
  const Status st = dev.readFifoWord(word);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::FIFO_EMPTY), static_cast<uint8_t>(st.code));
}

void test_direct_register_access_and_refresh_work() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.regs[cmd::REG_CTRL4_C] = static_cast<uint8_t>(cmd::MASK_SLEEP_G | cmd::MASK_LPF1_SEL_G);
  bus.regs[cmd::REG_CTRL6_C] = static_cast<uint8_t>(cmd::MASK_XL_HM_MODE | cmd::MASK_USR_OFF_W);
  bus.regs[cmd::REG_CTRL7_G] = static_cast<uint8_t>(cmd::MASK_G_HM_MODE | cmd::MASK_HP_EN_G |
                                                    (2u << cmd::BIT_HPM_G));
  bus.regs[cmd::REG_CTRL8_XL] = static_cast<uint8_t>(cmd::MASK_LPF2_XL_EN |
                                                     cmd::MASK_HP_SLOPE_XL_EN |
                                                     cmd::MASK_LOW_PASS_ON_6D);
  bus.regs[cmd::REG_CTRL10_C] = static_cast<uint8_t>((1u << cmd::BIT_TIMER_EN) |
                                                     (1u << cmd::BIT_PEDO_EN) |
                                                     (1u << cmd::BIT_TILT_EN) |
                                                     (1u << cmd::BIT_SIGN_MOTION_EN) |
                                                     (1u << cmd::BIT_WRIST_TILT_EN));
  bus.regs[cmd::REG_WAKE_UP_DUR] = cmd::MASK_TIMER_HR;
  bus.regs[cmd::REG_X_OFS_USR] = static_cast<uint8_t>(-2);
  bus.regs[cmd::REG_Y_OFS_USR] = 5u;
  bus.regs[cmd::REG_Z_OFS_USR] = static_cast<uint8_t>(-7);

  TEST_ASSERT_TRUE(dev.refreshCachedConfig().ok());

  bool boolOut = false;
  GyroFilterConfig gyroFilter;
  AccelFilterConfig accelFilter;
  AccelOffsetWeight weight = AccelOffsetWeight::MG_1;
  AccelUserOffset offset;

  TEST_ASSERT_TRUE(dev.getGyroSleepEnabled(boolOut).ok());
  TEST_ASSERT_TRUE(boolOut);
  TEST_ASSERT_TRUE(dev.getTimestampEnabled(boolOut).ok());
  TEST_ASSERT_TRUE(boolOut);
  TEST_ASSERT_TRUE(dev.getTimestampHighResolution(boolOut).ok());
  TEST_ASSERT_TRUE(boolOut);
  TEST_ASSERT_TRUE(dev.getPedometerEnabled(boolOut).ok());
  TEST_ASSERT_TRUE(boolOut);
  TEST_ASSERT_TRUE(dev.getSignificantMotionEnabled(boolOut).ok());
  TEST_ASSERT_TRUE(boolOut);
  TEST_ASSERT_TRUE(dev.getTiltEnabled(boolOut).ok());
  TEST_ASSERT_TRUE(boolOut);
  TEST_ASSERT_TRUE(dev.getWristTiltEnabled(boolOut).ok());
  TEST_ASSERT_TRUE(boolOut);
  TEST_ASSERT_TRUE(dev.getGyroFilterConfig(gyroFilter).ok());
  TEST_ASSERT_TRUE(dev.getAccelFilterConfig(accelFilter).ok());
  TEST_ASSERT_TRUE(dev.getAccelOffsetWeight(weight).ok());
  TEST_ASSERT_TRUE(dev.getAccelUserOffset(offset).ok());

  TEST_ASSERT_TRUE(gyroFilter.lpf1Enabled);
  TEST_ASSERT_TRUE(gyroFilter.highPassEnabled);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(GyroHpfMode::HZ_2_07),
                          static_cast<uint8_t>(gyroFilter.highPassMode));
  TEST_ASSERT_TRUE(accelFilter.lpf2Enabled);
  TEST_ASSERT_TRUE(accelFilter.highPassSlopeEnabled);
  TEST_ASSERT_TRUE(accelFilter.lowPassOn6d);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelOffsetWeight::MG_16),
                          static_cast<uint8_t>(weight));
  TEST_ASSERT_EQUAL_INT8(-2, offset.x);
  TEST_ASSERT_EQUAL_INT8(5, offset.y);
  TEST_ASSERT_EQUAL_INT8(-7, offset.z);
}

void test_direct_register_access_rejects_invalid_bounds_without_bus_io() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
  const uint32_t writesBefore = bus.writeCalls;
  const uint32_t readsBefore = bus.readCalls;

  uint8_t value = 0;
  Status st = dev.readRegisterValue(static_cast<uint8_t>(cmd::REG_Z_OFS_USR + 1u), value);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM), static_cast<uint8_t>(st.code));

  st = dev.writeRegisterValue(static_cast<uint8_t>(cmd::REG_Z_OFS_USR + 1u), 0x12u);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM), static_cast<uint8_t>(st.code));

  uint8_t block[2] = {};
  st = dev.readRegisterBlock(cmd::REG_Z_OFS_USR, block, sizeof(block));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM), static_cast<uint8_t>(st.code));

  st = dev.readRegisterBlock(cmd::REG_CTRL1_XL, block, 0);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM), static_cast<uint8_t>(st.code));

  TEST_ASSERT_EQUAL_UINT32(writesBefore, bus.writeCalls);
  TEST_ASSERT_EQUAL_UINT32(readsBefore, bus.readCalls);
}

void test_source_register_helpers_work() {
  FakeBus bus;
  bus.regs[cmd::REG_WAKE_UP_SRC] = 0x11;
  bus.regs[cmd::REG_TAP_SRC] = 0x22;
  bus.regs[cmd::REG_D6D_SRC] = 0x33;
  bus.regs[cmd::REG_FUNC_SRC1] = 0x44;
  bus.regs[cmd::REG_FUNC_SRC2] = 0x55;
  bus.regs[cmd::REG_WRIST_TILT_IA] = 0x66;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  uint8_t value = 0;
  TEST_ASSERT_TRUE(dev.readWakeUpSource(value).ok());
  TEST_ASSERT_EQUAL_HEX8(0x11, value);
  TEST_ASSERT_TRUE(dev.readTapSource(value).ok());
  TEST_ASSERT_EQUAL_HEX8(0x22, value);
  TEST_ASSERT_TRUE(dev.read6dSource(value).ok());
  TEST_ASSERT_EQUAL_HEX8(0x33, value);
  TEST_ASSERT_TRUE(dev.readFunctionSource1(value).ok());
  TEST_ASSERT_EQUAL_HEX8(0x44, value);
  TEST_ASSERT_TRUE(dev.readFunctionSource2(value).ok());
  TEST_ASSERT_EQUAL_HEX8(0x55, value);
  TEST_ASSERT_TRUE(dev.readWristTiltStatus(value).ok());
  TEST_ASSERT_EQUAL_HEX8(0x66, value);
}

// ==========================================================================
// Soft reset and end tests
// ==========================================================================

void test_soft_reset_restores_config() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());
  TEST_ASSERT_TRUE(dev.setAccelOdr(Odr::HZ_208).ok());
  TEST_ASSERT_TRUE(dev.setGyroFs(GyroFs::DPS_500).ok());

  const Status st = dev.softReset();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));

  Odr accelOdr = Odr::POWER_DOWN;
  GyroFs gyroFs = GyroFs::DPS_250;
  TEST_ASSERT_TRUE(dev.getAccelOdr(accelOdr).ok());
  TEST_ASSERT_TRUE(dev.getGyroFs(gyroFs).ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Odr::HZ_208), static_cast<uint8_t>(accelOdr));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(GyroFs::DPS_500), static_cast<uint8_t>(gyroFs));
}

void test_soft_reset_raw_poll_failure_updates_health() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_ERROR, "forced reset poll error", -33);
  const Status st = dev.softReset();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(1u, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(1u, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
}

void test_end_resets_state() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  dev.end();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::UNINIT),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_FALSE(dev.isOnline());
}

// ==========================================================================
// Command table and transport adapter tests
// ==========================================================================

void test_command_table_fix_for_sensor_sync_time_frame() {
  TEST_ASSERT_EQUAL_HEX8(0x02, cmd::REG_SENSOR_SYNC_TIME_FRAME);
}

void test_example_transport_maps_wire_errors() {
  Wire._clearEndTransmissionResult();
  Wire._clearRequestFromOverride();

  TEST_ASSERT_TRUE(transport::initWire(8, 9, 400000, 77));
  TEST_ASSERT_EQUAL_UINT32(77u, Wire.getTimeOut());

  const uint8_t byte = 0x55;

  Wire._setEndTransmissionResult(2);
  Status st = transport::wireWrite(0x6A, &byte, 1, 123, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_ADDR),
                          static_cast<uint8_t>(st.code));

  Wire._setEndTransmissionResult(3);
  st = transport::wireWrite(0x6A, &byte, 1, 999, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_NACK_DATA),
                          static_cast<uint8_t>(st.code));

  Wire._setEndTransmissionResult(4);
  st = transport::wireWrite(0x6A, &byte, 1, 999, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_BUS),
                          static_cast<uint8_t>(st.code));

  Wire._setEndTransmissionResult(5);
  st = transport::wireWrite(0x6A, &byte, 1, 999, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_TIMEOUT),
                          static_cast<uint8_t>(st.code));
}

void test_example_transport_validates_params() {
  const uint8_t tx = 0x00;
  uint8_t rx = 0;

  Status st = transport::wireWrite(0x6A, nullptr, 1, 50, nullptr);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));

  st = transport::wireWrite(0x6A, &tx, 0, 50, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(st.code));

  st = transport::wireWriteRead(0x6A, nullptr, 1, &rx, 1, 50, &Wire);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_PARAM),
                          static_cast<uint8_t>(st.code));
}

// ==========================================================================
// Main
// ==========================================================================

int main() {
  UNITY_BEGIN();

  RUN_TEST(test_status_ok);
  RUN_TEST(test_status_error);
  RUN_TEST(test_status_in_progress);

  RUN_TEST(test_config_defaults);

  RUN_TEST(test_begin_rejects_missing_callbacks);
  RUN_TEST(test_begin_success_sets_ready_and_health);
  RUN_TEST(test_begin_rejects_invalid_address);
  RUN_TEST(test_begin_rejects_accel_1_6_in_high_performance);
  RUN_TEST(test_begin_rejects_accel_416_in_low_power_normal);
  RUN_TEST(test_begin_rejects_gyro_1_6);
  RUN_TEST(test_begin_rejects_gyro_833_in_low_power_normal);
  RUN_TEST(test_begin_rejects_invalid_power_mode_enums);
  RUN_TEST(test_begin_detects_wrong_chip_id);
  RUN_TEST(test_probe_after_failed_begin_uses_stored_transport);
  RUN_TEST(test_recover_after_failed_begin_retries_begin);
  RUN_TEST(test_now_ms_fallback_uses_millis_when_callback_missing);

  RUN_TEST(test_probe_failure_does_not_update_health);
  RUN_TEST(test_recover_failure_updates_health_once);
  RUN_TEST(test_recover_chip_id_mismatch_updates_health_once);
  RUN_TEST(test_recover_success_returns_ready);
  RUN_TEST(test_recover_reaches_offline_when_threshold_is_one);

  RUN_TEST(test_read_all_raw_returns_data);
  RUN_TEST(test_convert_accel_at_2g);
  RUN_TEST(test_convert_gyro_at_250dps);
  RUN_TEST(test_convert_temperature);
  RUN_TEST(test_request_measurement_and_tick);
  RUN_TEST(test_request_measurement_rejects_without_bdu_for_combined_async);
  RUN_TEST(test_request_measurement_rejects_mismatched_active_odr);
  RUN_TEST(test_measurement_not_ready_before_tick);
  RUN_TEST(test_get_raw_measurement_after_read);
  RUN_TEST(test_get_measurement_applies_manual_bias);
  RUN_TEST(test_capture_accel_bias_auto_applies_to_measurements);
  RUN_TEST(test_capture_gyro_bias_auto_applies_to_measurements);
  RUN_TEST(test_capture_bias_times_out_with_stalled_clock_and_no_data_ready);

  RUN_TEST(test_set_accel_odr);
  RUN_TEST(test_set_gyro_fs);
  RUN_TEST(test_sensitivity_helpers);
  RUN_TEST(test_set_accel_power_mode_rejects_unsupported_current_odr);
  RUN_TEST(test_set_gyro_power_mode_rejects_unsupported_current_odr);
  RUN_TEST(test_power_mode_setters_reject_invalid_enums);
  RUN_TEST(test_cached_setters_roll_back_after_i2c_failure);
  RUN_TEST(test_multi_register_setters_roll_back_cache_after_partial_i2c_failure);
  RUN_TEST(test_timestamp_requires_active_sensor);
  RUN_TEST(test_embedded_functions_require_accel_odr_at_least_26hz);
  RUN_TEST(test_set_accel_odr_rejects_disabling_embedded_function_support);
  RUN_TEST(test_timestamp_helpers_work);
  RUN_TEST(test_step_counter_helpers_work);
  RUN_TEST(test_offset_and_filter_helpers_work);
  RUN_TEST(test_filter_offset_and_fifo_setters_reject_invalid_enums);
  RUN_TEST(test_fifo_configuration_and_read_helpers_work);
  RUN_TEST(test_fifo_requires_bdu_when_active);
  RUN_TEST(test_read_fifo_word_reports_empty);
  RUN_TEST(test_direct_register_access_and_refresh_work);
  RUN_TEST(test_direct_register_access_rejects_invalid_bounds_without_bus_io);
  RUN_TEST(test_source_register_helpers_work);

  RUN_TEST(test_soft_reset_restores_config);
  RUN_TEST(test_soft_reset_raw_poll_failure_updates_health);
  RUN_TEST(test_end_resets_state);

  RUN_TEST(test_command_table_fix_for_sensor_sync_time_frame);
  RUN_TEST(test_example_transport_maps_wire_errors);
  RUN_TEST(test_example_transport_validates_params);

  return UNITY_END();
}
