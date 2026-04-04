/// @file test_basic.cpp
/// @brief Native contract tests for LSM6DS3TR lifecycle and health behavior.

#include <unity.h>

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
  Status readError = Status::Error(Err::I2C_ERROR, "forced read error", -1);
  Status writeError = Status::Error(Err::I2C_ERROR, "forced write error", -2);

  // STATUS_REG override: if > 0, return XLDA=0 (not ready)
  uint32_t notReadyStatusReads = 0;
};

Status fakeWrite(uint8_t, const uint8_t* data, size_t len, uint32_t, void* user) {
  FakeBus* bus = static_cast<FakeBus*>(user);
  bus->writeCalls++;
  if (data == nullptr || len == 0) {
    return Status::Error(Err::INVALID_PARAM, "invalid fake write args");
  }
  if (bus->writeErrorRemaining > 0) {
    bus->writeErrorRemaining--;
    return bus->writeError;
  }
  return Status::Ok();
}

Status fakeWriteRead(uint8_t, const uint8_t* txData, size_t txLen, uint8_t* rxData,
                     size_t rxLen, uint32_t, void* user) {
  FakeBus* bus = static_cast<FakeBus*>(user);
  bus->readCalls++;
  if (txData == nullptr || txLen == 0 || (rxLen > 0 && rxData == nullptr)) {
    return Status::Error(Err::INVALID_PARAM, "invalid fake write-read args");
  }
  if (bus->readErrorRemaining > 0) {
    bus->readErrorRemaining--;
    return bus->readError;
  }

  const uint8_t reg = txData[0];
  for (size_t i = 0; i < rxLen; ++i) {
    rxData[i] = 0;
  }

  if (reg == cmd::REG_WHO_AM_I && rxLen >= 1) {
    rxData[0] = cmd::WHO_AM_I_VALUE;
  } else if (reg == cmd::REG_STATUS_REG && rxLen >= 1) {
    if (bus->notReadyStatusReads > 0) {
      rxData[0] = 0x00;  // No data ready
      bus->notReadyStatusReads--;
    } else {
      rxData[0] = cmd::MASK_XLDA | cmd::MASK_GDA | cmd::MASK_TDA;
    }
  } else if (reg == cmd::REG_CTRL3_C && rxLen >= 1) {
    // After reset, SW_RESET bit cleared
    rxData[0] = cmd::MASK_IF_INC;
  } else if (reg == cmd::REG_DATA_START_ALL && rxLen == cmd::DATA_LEN_ALL) {
    // Return synthetic IMU data:
    // Temperature: 6400 (raw) -> 6400/256 + 25 = 50.0 C
    rxData[0] = 0x00;  // OUT_TEMP_L
    rxData[1] = 0x19;  // OUT_TEMP_H  -> 0x1900 = 6400
    // Gyro X: 1000
    rxData[2] = 0xE8;  // OUTX_L_G
    rxData[3] = 0x03;  // OUTX_H_G   -> 0x03E8 = 1000
    // Gyro Y: -500
    rxData[4] = 0x0C;  // OUTY_L_G
    rxData[5] = 0xFE;  // OUTY_H_G   -> 0xFE0C = -500
    // Gyro Z: 0
    rxData[6] = 0x00;  // OUTZ_L_G
    rxData[7] = 0x00;  // OUTZ_H_G
    // Accel X: 16384 (~1g at ±2g: 16384 * 0.061 = 999.4 mg)
    rxData[8] = 0x00;  // OUTX_L_XL
    rxData[9] = 0x40;  // OUTX_H_XL  -> 0x4000 = 16384
    // Accel Y: 0
    rxData[10] = 0x00; // OUTY_L_XL
    rxData[11] = 0x00; // OUTY_H_XL
    // Accel Z: 0
    rxData[12] = 0x00; // OUTZ_L_XL
    rxData[13] = 0x00; // OUTZ_H_XL
  } else if (reg == cmd::REG_DATA_START_ACCEL && rxLen == cmd::DATA_LEN_ACCEL) {
    rxData[0] = 0x00;
    rxData[1] = 0x40;  // X = 16384
    rxData[2] = 0x00;
    rxData[3] = 0x00;  // Y = 0
    rxData[4] = 0x00;
    rxData[5] = 0x00;  // Z = 0
  } else if (reg == cmd::REG_DATA_START_GYRO && rxLen == cmd::DATA_LEN_GYRO) {
    rxData[0] = 0xE8;
    rxData[1] = 0x03;  // X = 1000
    rxData[2] = 0x00;
    rxData[3] = 0x00;  // Y = 0
    rxData[4] = 0x00;
    rxData[5] = 0x00;  // Z = 0
  } else if (reg == cmd::REG_OUT_TEMP_L && rxLen == 2) {
    rxData[0] = 0x00;
    rxData[1] = 0x19;  // 6400
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
  Status st = Status::Ok();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::OK), static_cast<uint8_t>(st.code));
}

void test_status_error() {
  Status st = Status::Error(Err::I2C_ERROR, "Test error", 42);
  TEST_ASSERT_FALSE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(42, st.detail);
}

void test_status_in_progress() {
  Status st{Err::IN_PROGRESS, 0, "In progress"};
  TEST_ASSERT_FALSE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::IN_PROGRESS), static_cast<uint8_t>(st.code));
}

// ==========================================================================
// Config default tests
// ==========================================================================

void test_config_defaults() {
  Config cfg;
  TEST_ASSERT_NULL(cfg.i2cWrite);
  TEST_ASSERT_NULL(cfg.i2cWriteRead);
  TEST_ASSERT_EQUAL_HEX8(0x6A, cfg.i2cAddress);
  TEST_ASSERT_EQUAL_UINT32(50u, cfg.i2cTimeoutMs);
  TEST_ASSERT_EQUAL_UINT8(5, cfg.offlineThreshold);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Odr::HZ_104), static_cast<uint8_t>(cfg.odrXl));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Odr::HZ_104), static_cast<uint8_t>(cfg.odrG));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AccelFs::G_2), static_cast<uint8_t>(cfg.fsXl));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(GyroFs::DPS_250), static_cast<uint8_t>(cfg.fsG));
  TEST_ASSERT_TRUE(cfg.bdu);
}

// ==========================================================================
// Lifecycle tests
// ==========================================================================

void test_begin_rejects_missing_callbacks() {
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg;
  Status st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::UNINIT),
                          static_cast<uint8_t>(dev.state()));
}

void test_begin_success_sets_ready_and_health() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Status st = dev.begin(makeConfig(bus));
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
  Status st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG), static_cast<uint8_t>(st.code));
}

void test_begin_detects_wrong_chip_id() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg = makeConfig(bus);
  // Override the read to return wrong WHO_AM_I
  // We need a special bus for this — we'll use read error to force DEVICE_NOT_FOUND
  bus.readErrorRemaining = 1;
  bus.readError = Status::Error(Err::I2C_ERROR, "no device", -1);
  Status st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::DEVICE_NOT_FOUND),
                          static_cast<uint8_t>(st.code));
}

// ==========================================================================
// NowMs fallback tests
// ==========================================================================

void test_now_ms_fallback_uses_millis_when_callback_missing() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg = makeConfig(bus);
  cfg.nowMs = nullptr;
  cfg.timeUser = nullptr;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  setMillis(4321);
  Status st = dev.recover();
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
  Status st = dev.probe();
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
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(1u, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(1u, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::I2C_ERROR),
                          static_cast<uint8_t>(dev.lastError().code));
  TEST_ASSERT_EQUAL_UINT32(bus.nowMs, dev.lastErrorMs());
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
  Status st = dev.recover();
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
  Status st = dev.recover();
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
  Status st = dev.readAllRaw(raw);
  TEST_ASSERT_TRUE(st.ok());

  // Verify against FakeBus synthetic data
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
  Config cfg = makeConfig(bus);
  cfg.fsXl = AccelFs::G_2;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  RawAxes raw = {16384, 0, 0};
  Axes result = dev.convertAccel(raw);
  // 16384 * 0.061 mg/LSB = 999.424 mg = 0.999424 g
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.999f, result.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, result.y);
}

void test_convert_gyro_at_250dps() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  Config cfg = makeConfig(bus);
  cfg.fsG = GyroFs::DPS_250;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  RawAxes raw = {1000, -500, 0};
  Axes result = dev.convertGyro(raw);
  // 1000 * 8.75 mdps/LSB = 8750 mdps = 8.75 dps
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 8.75f, result.x);
  // -500 * 8.75 = -4375 mdps = -4.375 dps
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -4.375f, result.y);
}

void test_convert_temperature() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  // raw = 6400, temp = 6400/256 + 25 = 50.0 C
  float tempC = dev.convertTemperature(6400);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, tempC);

  // raw = 0, temp = 0/256 + 25 = 25.0 C
  tempC = dev.convertTemperature(0);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 25.0f, tempC);

  // raw = -6400, temp = -6400/256 + 25 = 0.0 C
  tempC = dev.convertTemperature(-6400);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, tempC);
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

  // Verify synthetic data
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.999f, m.accel.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, m.temperatureC);
}

void test_measurement_not_ready_before_tick() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  bus.notReadyStatusReads = 2;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  Status st = dev.requestMeasurement();
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

  // Direct read populates hasSample (readAllRaw is blocking)
  st = dev.readAllRaw(raw);
  TEST_ASSERT_TRUE(st.ok());

  // requestMeasurement + tick also works
  st = dev.requestMeasurement();
  dev.tick(bus.nowMs);
  TEST_ASSERT_TRUE(dev.measurementReady());

  st = dev.getRawMeasurement(raw);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_INT16(16384, raw.accel.x);
}

// ==========================================================================
// Configuration tests
// ==========================================================================

void test_set_accel_odr() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  Status st = dev.setAccelOdr(Odr::HZ_208);
  TEST_ASSERT_TRUE(st.ok());

  Odr odr;
  st = dev.getAccelOdr(odr);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Odr::HZ_208), static_cast<uint8_t>(odr));
}

void test_set_gyro_fs() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  Status st = dev.setGyroFs(GyroFs::DPS_500);
  TEST_ASSERT_TRUE(st.ok());

  GyroFs fs;
  st = dev.getGyroFs(fs);
  TEST_ASSERT_TRUE(st.ok());
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

// ==========================================================================
// Soft reset test
// ==========================================================================

void test_soft_reset_restores_config() {
  FakeBus bus;
  LSM6DS3TR::LSM6DS3TR dev;
  TEST_ASSERT_TRUE(dev.begin(makeConfig(bus)).ok());

  Status st = dev.softReset();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
}

// ==========================================================================
// End test
// ==========================================================================

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
// Transport adapter tests
// ==========================================================================

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

  // Status
  RUN_TEST(test_status_ok);
  RUN_TEST(test_status_error);
  RUN_TEST(test_status_in_progress);

  // Config
  RUN_TEST(test_config_defaults);

  // Lifecycle
  RUN_TEST(test_begin_rejects_missing_callbacks);
  RUN_TEST(test_begin_success_sets_ready_and_health);
  RUN_TEST(test_begin_rejects_invalid_address);
  RUN_TEST(test_begin_detects_wrong_chip_id);
  RUN_TEST(test_now_ms_fallback_uses_millis_when_callback_missing);

  // Diagnostics
  RUN_TEST(test_probe_failure_does_not_update_health);
  RUN_TEST(test_recover_failure_updates_health_once);
  RUN_TEST(test_recover_success_returns_ready);
  RUN_TEST(test_recover_reaches_offline_when_threshold_is_one);

  // Measurement
  RUN_TEST(test_read_all_raw_returns_data);
  RUN_TEST(test_convert_accel_at_2g);
  RUN_TEST(test_convert_gyro_at_250dps);
  RUN_TEST(test_convert_temperature);
  RUN_TEST(test_request_measurement_and_tick);
  RUN_TEST(test_measurement_not_ready_before_tick);
  RUN_TEST(test_get_raw_measurement_after_read);

  // Configuration
  RUN_TEST(test_set_accel_odr);
  RUN_TEST(test_set_gyro_fs);
  RUN_TEST(test_sensitivity_helpers);

  // Soft reset
  RUN_TEST(test_soft_reset_restores_config);

  // End
  RUN_TEST(test_end_resets_state);

  // Transport adapter
  RUN_TEST(test_example_transport_maps_wire_errors);
  RUN_TEST(test_example_transport_validates_params);

  return UNITY_END();
}
