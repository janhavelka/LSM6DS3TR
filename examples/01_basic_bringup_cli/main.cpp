/// @file main.cpp
/// @brief Basic bringup example for LSM6DS3TR-C IMU
/// @note This is an EXAMPLE, not part of the library

#include <Arduino.h>
#include <limits>
#include "examples/common/Log.h"
#include "examples/common/BoardConfig.h"
#include "examples/common/I2cTransport.h"
#include "examples/common/I2cScanner.h"

#include "LSM6DS3TR/LSM6DS3TR.h"

// ============================================================================
// Globals
// ============================================================================

struct StressStats {
  bool active = false;
  uint32_t startMs = 0;
  uint32_t endMs = 0;
  int target = 0;
  int attempts = 0;
  int success = 0;
  uint32_t errors = 0;
  bool hasSample = false;
  float minAx = 0, maxAx = 0;
  float minAy = 0, maxAy = 0;
  float minAz = 0, maxAz = 0;
  float minGx = 0, maxGx = 0;
  float minGy = 0, maxGy = 0;
  float minGz = 0, maxGz = 0;
  float minTemp = 0, maxTemp = 0;
  double sumTemp = 0;
  LSM6DS3TR::Status lastError = LSM6DS3TR::Status::Ok();
};

LSM6DS3TR::LSM6DS3TR device;
bool verboseMode = false;
bool pendingRead = false;
uint32_t pendingStartMs = 0;
int stressRemaining = 0;
StressStats stressStats;

void cancelPending();

// ============================================================================
// Helper Functions
// ============================================================================

uint32_t exampleNowMs(void*) {
  return millis();
}

const char* errToStr(LSM6DS3TR::Err err) {
  using namespace LSM6DS3TR;
  switch (err) {
    case Err::OK:                    return "OK";
    case Err::NOT_INITIALIZED:       return "NOT_INITIALIZED";
    case Err::INVALID_CONFIG:        return "INVALID_CONFIG";
    case Err::I2C_ERROR:             return "I2C_ERROR";
    case Err::I2C_NACK_ADDR:        return "I2C_NACK_ADDR";
    case Err::I2C_NACK_DATA:        return "I2C_NACK_DATA";
    case Err::I2C_BUS:              return "I2C_BUS";
    case Err::I2C_TIMEOUT:          return "I2C_TIMEOUT";
    case Err::TIMEOUT:               return "TIMEOUT";
    case Err::INVALID_PARAM:         return "INVALID_PARAM";
    case Err::DEVICE_NOT_FOUND:      return "DEVICE_NOT_FOUND";
    case Err::CHIP_ID_MISMATCH:      return "CHIP_ID_MISMATCH";
    case Err::MEASUREMENT_NOT_READY: return "MEASUREMENT_NOT_READY";
    case Err::SELF_TEST_FAIL:        return "SELF_TEST_FAIL";
    case Err::BUSY:                  return "BUSY";
    case Err::IN_PROGRESS:           return "IN_PROGRESS";
    default:                         return "UNKNOWN";
  }
}

const char* stateToStr(LSM6DS3TR::DriverState st) {
  using namespace LSM6DS3TR;
  switch (st) {
    case DriverState::UNINIT:   return "UNINIT";
    case DriverState::READY:    return "READY";
    case DriverState::DEGRADED: return "DEGRADED";
    case DriverState::OFFLINE:  return "OFFLINE";
    default:                    return "UNKNOWN";
  }
}

const char* stateColor(LSM6DS3TR::DriverState st, bool online, uint8_t consecutiveFailures) {
  if (st == LSM6DS3TR::DriverState::UNINIT) return LOG_COLOR_RESET;
  return LOG_COLOR_STATE(online, consecutiveFailures);
}

const char* goodIfZeroColor(uint32_t value) {
  return (value == 0U) ? LOG_COLOR_GREEN : LOG_COLOR_RED;
}

const char* goodIfNonZeroColor(uint32_t value) {
  return (value > 0U) ? LOG_COLOR_GREEN : LOG_COLOR_YELLOW;
}

const char* successRateColor(float pct) {
  if (pct >= 99.9f) return LOG_COLOR_GREEN;
  if (pct >= 80.0f) return LOG_COLOR_YELLOW;
  return LOG_COLOR_RED;
}

const char* odrToStr(LSM6DS3TR::Odr odr) {
  using namespace LSM6DS3TR;
  switch (odr) {
    case Odr::POWER_DOWN: return "POWER_DOWN";
    case Odr::HZ_1_6:    return "1.6 Hz";
    case Odr::HZ_12_5:   return "12.5 Hz";
    case Odr::HZ_26:     return "26 Hz";
    case Odr::HZ_52:     return "52 Hz";
    case Odr::HZ_104:    return "104 Hz";
    case Odr::HZ_208:    return "208 Hz";
    case Odr::HZ_416:    return "416 Hz";
    case Odr::HZ_833:    return "833 Hz";
    case Odr::HZ_1660:   return "1660 Hz";
    case Odr::HZ_3330:   return "3330 Hz";
    case Odr::HZ_6660:   return "6660 Hz";
    default:              return "UNKNOWN";
  }
}

const char* accelFsToStr(LSM6DS3TR::AccelFs fs) {
  using namespace LSM6DS3TR;
  switch (fs) {
    case AccelFs::G_2:  return "+/-2g";
    case AccelFs::G_4:  return "+/-4g";
    case AccelFs::G_8:  return "+/-8g";
    case AccelFs::G_16: return "+/-16g";
    default:            return "UNKNOWN";
  }
}

const char* gyroFsToStr(LSM6DS3TR::GyroFs fs) {
  using namespace LSM6DS3TR;
  switch (fs) {
    case GyroFs::DPS_125:  return "+/-125 dps";
    case GyroFs::DPS_250:  return "+/-250 dps";
    case GyroFs::DPS_500:  return "+/-500 dps";
    case GyroFs::DPS_1000: return "+/-1000 dps";
    case GyroFs::DPS_2000: return "+/-2000 dps";
    default:               return "UNKNOWN";
  }
}

LSM6DS3TR::Odr parseOdr(int value) {
  using namespace LSM6DS3TR;
  switch (value) {
    case 0:    return Odr::POWER_DOWN;
    case 1:    return Odr::HZ_1_6;
    case 12:   return Odr::HZ_12_5;
    case 26:   return Odr::HZ_26;
    case 52:   return Odr::HZ_52;
    case 104:  return Odr::HZ_104;
    case 208:  return Odr::HZ_208;
    case 416:  return Odr::HZ_416;
    case 833:  return Odr::HZ_833;
    case 1660: return Odr::HZ_1660;
    case 3330: return Odr::HZ_3330;
    case 6660: return Odr::HZ_6660;
    default:   return Odr::HZ_104;
  }
}

LSM6DS3TR::AccelFs parseAccelFs(int value) {
  using namespace LSM6DS3TR;
  switch (value) {
    case 2:  return AccelFs::G_2;
    case 4:  return AccelFs::G_4;
    case 8:  return AccelFs::G_8;
    case 16: return AccelFs::G_16;
    default: return AccelFs::G_2;
  }
}

LSM6DS3TR::GyroFs parseGyroFs(int value) {
  using namespace LSM6DS3TR;
  switch (value) {
    case 125:  return GyroFs::DPS_125;
    case 250:  return GyroFs::DPS_250;
    case 500:  return GyroFs::DPS_500;
    case 1000: return GyroFs::DPS_1000;
    case 2000: return GyroFs::DPS_2000;
    default:   return GyroFs::DPS_250;
  }
}

void printStatus(const LSM6DS3TR::Status& st) {
  Serial.printf("  Status: %s%s%s (code=%u, detail=%ld)\n",
                LOG_COLOR_RESULT(st.ok()),
                errToStr(st.code),
                LOG_COLOR_RESET,
                static_cast<unsigned>(st.code),
                static_cast<long>(st.detail));
  if (st.msg && st.msg[0]) {
    Serial.printf("  Message: %s%s%s\n", LOG_COLOR_YELLOW, st.msg, LOG_COLOR_RESET);
  }
}

void printDriverHealth() {
  const uint32_t now = millis();
  const uint32_t totalOk = device.totalSuccess();
  const uint32_t totalFail = device.totalFailures();
  const uint32_t total = totalOk + totalFail;
  const float successRate = (total > 0U)
                              ? (100.0f * static_cast<float>(totalOk) / static_cast<float>(total))
                              : 0.0f;
  const LSM6DS3TR::Status lastErr = device.lastError();
  const LSM6DS3TR::DriverState st = device.state();
  const bool online = device.isOnline();

  Serial.println("=== Driver Health ===");
  Serial.printf("  State: %s%s%s\n",
                stateColor(st, online, device.consecutiveFailures()),
                stateToStr(st),
                LOG_COLOR_RESET);
  Serial.printf("  Online: %s%s%s\n",
                online ? LOG_COLOR_GREEN : LOG_COLOR_RED,
                log_bool_str(online),
                LOG_COLOR_RESET);
  Serial.printf("  Consecutive failures: %s%u%s\n",
                goodIfZeroColor(device.consecutiveFailures()),
                device.consecutiveFailures(),
                LOG_COLOR_RESET);
  Serial.printf("  Total success: %s%lu%s\n",
                goodIfNonZeroColor(totalOk),
                static_cast<unsigned long>(totalOk),
                LOG_COLOR_RESET);
  Serial.printf("  Total failures: %s%lu%s\n",
                goodIfZeroColor(totalFail),
                static_cast<unsigned long>(totalFail),
                LOG_COLOR_RESET);
  Serial.printf("  Success rate: %s%.1f%%%s\n",
                successRateColor(successRate),
                successRate,
                LOG_COLOR_RESET);

  const uint32_t lastOkMs = device.lastOkMs();
  if (lastOkMs > 0U) {
    Serial.printf("  Last OK: %lu ms ago (at %lu ms)\n",
                  static_cast<unsigned long>(now - lastOkMs),
                  static_cast<unsigned long>(lastOkMs));
  } else {
    Serial.println("  Last OK: never");
  }

  const uint32_t lastErrorMs = device.lastErrorMs();
  if (lastErrorMs > 0U) {
    Serial.printf("  Last error: %lu ms ago (at %lu ms)\n",
                  static_cast<unsigned long>(now - lastErrorMs),
                  static_cast<unsigned long>(lastErrorMs));
  } else {
    Serial.println("  Last error: never");
  }

  if (!lastErr.ok()) {
    Serial.printf("  Error code: %s%s%s\n",
                  LOG_COLOR_RED,
                  errToStr(lastErr.code),
                  LOG_COLOR_RESET);
    Serial.printf("  Error detail: %ld\n", static_cast<long>(lastErr.detail));
    if (lastErr.msg && lastErr.msg[0]) {
      Serial.printf("  Error msg: %s\n", lastErr.msg);
    }
  }
}

void printSettings() {
  LSM6DS3TR::Odr odrXl, odrG;
  LSM6DS3TR::AccelFs fsXl;
  LSM6DS3TR::GyroFs fsG;

  Serial.println("=== Current Settings ===");

  if (device.getAccelOdr(odrXl).ok()) {
    Serial.printf("  Accel ODR: %s (%u)\n", odrToStr(odrXl), static_cast<unsigned>(odrXl));
  }
  if (device.getGyroOdr(odrG).ok()) {
    Serial.printf("  Gyro ODR:  %s (%u)\n", odrToStr(odrG), static_cast<unsigned>(odrG));
  }
  if (device.getAccelFs(fsXl).ok()) {
    Serial.printf("  Accel FS:  %s\n", accelFsToStr(fsXl));
  }
  if (device.getGyroFs(fsG).ok()) {
    Serial.printf("  Gyro FS:   %s\n", gyroFsToStr(fsG));
  }
  Serial.printf("  Accel sens: %.3f mg/LSB\n", device.accelSensitivity());
  Serial.printf("  Gyro sens:  %.3f mdps/LSB\n", device.gyroSensitivity());
  Serial.printf("  Verbose: %s\n", verboseMode ? "ON" : "OFF");
}

void printMeasurement(const LSM6DS3TR::Measurement& m) {
  Serial.printf("Accel: x=%.3f y=%.3f z=%.3f g\n", m.accel.x, m.accel.y, m.accel.z);
  Serial.printf("Gyro:  x=%.2f y=%.2f z=%.2f dps\n", m.gyro.x, m.gyro.y, m.gyro.z);
  Serial.printf("Temp:  %.2f C\n", m.temperatureC);
}

void printRawSample() {
  LSM6DS3TR::RawMeasurement raw;
  const LSM6DS3TR::Status st = device.readAllRaw(raw);
  if (!st.ok()) {
    printStatus(st);
    return;
  }
  Serial.printf("Raw Accel: x=%d y=%d z=%d\n", raw.accel.x, raw.accel.y, raw.accel.z);
  Serial.printf("Raw Gyro:  x=%d y=%d z=%d\n", raw.gyro.x, raw.gyro.y, raw.gyro.z);
  Serial.printf("Raw Temp:  %d\n", raw.temperature);
}

// ============================================================================
// Stress Testing
// ============================================================================

void resetStressStats(int target) {
  stressStats = StressStats{};
  stressStats.active = true;
  stressStats.startMs = millis();
  stressStats.target = target;
}

void noteStressError(const LSM6DS3TR::Status& st) {
  stressStats.errors++;
  stressStats.lastError = st;
}

void updateStressStats(const LSM6DS3TR::Measurement& m) {
  if (!stressStats.hasSample) {
    stressStats.minAx = stressStats.maxAx = m.accel.x;
    stressStats.minAy = stressStats.maxAy = m.accel.y;
    stressStats.minAz = stressStats.maxAz = m.accel.z;
    stressStats.minGx = stressStats.maxGx = m.gyro.x;
    stressStats.minGy = stressStats.maxGy = m.gyro.y;
    stressStats.minGz = stressStats.maxGz = m.gyro.z;
    stressStats.minTemp = stressStats.maxTemp = m.temperatureC;
    stressStats.hasSample = true;
  } else {
    if (m.accel.x < stressStats.minAx) stressStats.minAx = m.accel.x;
    if (m.accel.x > stressStats.maxAx) stressStats.maxAx = m.accel.x;
    if (m.accel.y < stressStats.minAy) stressStats.minAy = m.accel.y;
    if (m.accel.y > stressStats.maxAy) stressStats.maxAy = m.accel.y;
    if (m.accel.z < stressStats.minAz) stressStats.minAz = m.accel.z;
    if (m.accel.z > stressStats.maxAz) stressStats.maxAz = m.accel.z;
    if (m.gyro.x < stressStats.minGx) stressStats.minGx = m.gyro.x;
    if (m.gyro.x > stressStats.maxGx) stressStats.maxGx = m.gyro.x;
    if (m.gyro.y < stressStats.minGy) stressStats.minGy = m.gyro.y;
    if (m.gyro.y > stressStats.maxGy) stressStats.maxGy = m.gyro.y;
    if (m.gyro.z < stressStats.minGz) stressStats.minGz = m.gyro.z;
    if (m.gyro.z > stressStats.maxGz) stressStats.maxGz = m.gyro.z;
    if (m.temperatureC < stressStats.minTemp) stressStats.minTemp = m.temperatureC;
    if (m.temperatureC > stressStats.maxTemp) stressStats.maxTemp = m.temperatureC;
  }
  stressStats.sumTemp += m.temperatureC;
  stressStats.success++;
}

void finishStressStats() {
  stressStats.active = false;
  stressStats.endMs = millis();
  const uint32_t durationMs = stressStats.endMs - stressStats.startMs;

  Serial.println("=== Stress Summary ===");
  Serial.printf("  Target: %d\n", stressStats.target);
  Serial.printf("  Attempts: %d\n", stressStats.attempts);
  Serial.printf("  Success: %s%d%s\n",
                goodIfNonZeroColor(static_cast<uint32_t>(stressStats.success)),
                stressStats.success, LOG_COLOR_RESET);
  Serial.printf("  Errors: %s%lu%s\n",
                goodIfZeroColor(stressStats.errors),
                static_cast<unsigned long>(stressStats.errors), LOG_COLOR_RESET);
  Serial.printf("  Duration: %lu ms\n", static_cast<unsigned long>(durationMs));
  if (durationMs > 0) {
    const float rate = 1000.0f * static_cast<float>(stressStats.attempts) /
                       static_cast<float>(durationMs);
    Serial.printf("  Rate: %.2f samples/s\n", rate);
  }

  if (stressStats.success > 0) {
    const float avgTemp = static_cast<float>(stressStats.sumTemp / stressStats.success);
    Serial.printf("  Accel X g: min=%.3f max=%.3f\n", stressStats.minAx, stressStats.maxAx);
    Serial.printf("  Accel Y g: min=%.3f max=%.3f\n", stressStats.minAy, stressStats.maxAy);
    Serial.printf("  Accel Z g: min=%.3f max=%.3f\n", stressStats.minAz, stressStats.maxAz);
    Serial.printf("  Gyro X dps: min=%.2f max=%.2f\n", stressStats.minGx, stressStats.maxGx);
    Serial.printf("  Gyro Y dps: min=%.2f max=%.2f\n", stressStats.minGy, stressStats.maxGy);
    Serial.printf("  Gyro Z dps: min=%.2f max=%.2f\n", stressStats.minGz, stressStats.maxGz);
    Serial.printf("  Temp C: min=%.2f avg=%.2f max=%.2f\n",
                  stressStats.minTemp, avgTemp, stressStats.maxTemp);
  } else {
    Serial.println("  No valid samples");
  }

  if (!stressStats.lastError.ok()) {
    Serial.printf("  Last error: %s\n", errToStr(stressStats.lastError.code));
    if (stressStats.lastError.msg && stressStats.lastError.msg[0]) {
      Serial.printf("  Message: %s\n", stressStats.lastError.msg);
    }
  }
}

LSM6DS3TR::Status performReadBlocking(LSM6DS3TR::Measurement& out) {
  LSM6DS3TR::RawMeasurement raw;
  LSM6DS3TR::Status st = device.readAllRaw(raw);
  if (!st.ok()) return st;
  out.accel = device.convertAccel(raw.accel);
  out.gyro = device.convertGyro(raw.gyro);
  out.temperatureC = device.convertTemperature(raw.temperature);
  return LSM6DS3TR::Status::Ok();
}

LSM6DS3TR::Status scheduleMeasurement() {
  LSM6DS3TR::Status st = device.requestMeasurement();
  if (st.inProgress()) {
    pendingRead = true;
    pendingStartMs = millis();
  }
  return st;
}

void handleMeasurementReady() {
  if (!pendingRead || !device.measurementReady()) return;
  pendingRead = false;

  LSM6DS3TR::Measurement m;
  const LSM6DS3TR::Status st = device.getMeasurement(m);
  if (!st.ok()) {
    if (stressStats.active) {
      noteStressError(st);
      stressStats.attempts++;
      stressRemaining--;
      if (stressRemaining == 0) finishStressStats();
    } else {
      printStatus(st);
    }
    return;
  }

  if (stressStats.active) {
    updateStressStats(m);
    stressStats.attempts++;
    stressRemaining--;
    if (stressRemaining == 0) finishStressStats();
  } else {
    printMeasurement(m);
  }
}

void cancelPending() {
  pendingRead = false;
  if (stressStats.active) {
    stressStats.active = false;
    Serial.println("  Cancelled.");
  }
  stressRemaining = 0;
}

// ============================================================================
// Self-Test
// ============================================================================

void selfTest() {
  Serial.println("=== Self-Test ===");

  // 1. Probe
  Serial.print("  Probe: ");
  LSM6DS3TR::Status st = device.probe();
  Serial.printf("%s%s%s\n", LOG_COLOR_RESULT(st.ok()), errToStr(st.code), LOG_COLOR_RESET);
  if (!st.ok()) return;

  // 2. WHO_AM_I
  uint8_t id = 0;
  st = device.readWhoAmI(id);
  Serial.printf("  WHO_AM_I: 0x%02X %s%s%s\n",
                id,
                LOG_COLOR_RESULT(id == LSM6DS3TR::cmd::WHO_AM_I_VALUE),
                (id == LSM6DS3TR::cmd::WHO_AM_I_VALUE) ? "OK" : "FAIL",
                LOG_COLOR_RESET);

  // 3. Burst read
  LSM6DS3TR::RawMeasurement raw;
  st = device.readAllRaw(raw);
  Serial.printf("  Burst read: %s%s%s\n",
                LOG_COLOR_RESULT(st.ok()), errToStr(st.code), LOG_COLOR_RESET);
  if (st.ok()) {
    Serial.printf("    Accel raw: x=%d y=%d z=%d\n", raw.accel.x, raw.accel.y, raw.accel.z);
    Serial.printf("    Gyro raw:  x=%d y=%d z=%d\n", raw.gyro.x, raw.gyro.y, raw.gyro.z);
    Serial.printf("    Temp raw:  %d\n", raw.temperature);
  }

  // 4. Conversion sanity
  if (st.ok()) {
    LSM6DS3TR::Axes accel = device.convertAccel(raw.accel);
    float tempC = device.convertTemperature(raw.temperature);
    float accelMag = sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    bool accelPlausible = (accelMag > 0.5f && accelMag < 2.0f);
    bool tempPlausible = (tempC > -10.0f && tempC < 80.0f);
    Serial.printf("  Accel magnitude: %.3f g %s%s%s\n",
                  accelMag,
                  LOG_COLOR_RESULT(accelPlausible),
                  accelPlausible ? "OK" : "SUSPECT",
                  LOG_COLOR_RESET);
    Serial.printf("  Temperature: %.2f C %s%s%s\n",
                  tempC,
                  LOG_COLOR_RESULT(tempPlausible),
                  tempPlausible ? "OK" : "SUSPECT",
                  LOG_COLOR_RESET);
  }

  // 5. Health
  Serial.printf("  Driver state: %s\n", stateToStr(device.state()));
  Serial.println("=== Self-Test Complete ===");
}

// ============================================================================
// Help
// ============================================================================

void printHelp() {
  Serial.println("\n=== LSM6DS3TR-C CLI Commands ===");
  Serial.println("Measurement:");
  Serial.println("  read          Read accel + gyro + temp (converted)");
  Serial.println("  raw           Read raw ADC values");
  Serial.println("  accel         Read accelerometer only");
  Serial.println("  gyro          Read gyroscope only");
  Serial.println("  temp          Read temperature only");
  Serial.println("Configuration:");
  Serial.println("  odrxl [<hz>]  Get/set accel ODR (0,1,12,26,52,104,208,416,833,1660,3330,6660)");
  Serial.println("  odrg [<hz>]   Get/set gyro ODR");
  Serial.println("  fsxl [<g>]    Get/set accel full-scale (2,4,8,16)");
  Serial.println("  fsg [<dps>]   Get/set gyro full-scale (125,250,500,1000,2000)");
  Serial.println("  cfg           Show all current settings");
  Serial.println("Diagnostics:");
  Serial.println("  drv           Show driver health");
  Serial.println("  probe         Probe device (raw, no health tracking)");
  Serial.println("  recover       Attempt recovery from DEGRADED/OFFLINE");
  Serial.println("  selftest      Run self-test diagnostics");
  Serial.println("  stress [N]    Read N samples, report stats (default 100)");
  Serial.println("  verbose [0|1] Toggle verbose mode");
  Serial.println("Device:");
  Serial.println("  whoami        Read WHO_AM_I register");
  Serial.println("  status        Read STATUS_REG");
  Serial.println("  reset         Software reset");
  Serial.println("Utility:");
  Serial.println("  scan          Scan I2C bus");
  Serial.println("  version       Show library version");
  Serial.println("  help          Print this help");
}

// ============================================================================
// Command Parser Helpers
// ============================================================================

/// Parse integer argument after a command prefix.
/// e.g. parseIntArg("odrxl 104", "odrxl", &val) -> val=104, returns true
bool parseIntArg(const String& input, const char* prefix, int& out) {
  int idx = input.indexOf(' ');
  if (idx < 0) return false;
  String arg = input.substring(idx + 1);
  arg.trim();
  if (arg.length() == 0) return false;
  out = arg.toInt();
  return true;
}

// ============================================================================
// Command Dispatch
// ============================================================================

void processCommand(const String& input) {
  String cmd = input;
  cmd.trim();
  if (cmd.length() == 0) return;

  // --- Measurement ---
  if (cmd == "read") {
    LSM6DS3TR::Measurement m;
    LSM6DS3TR::Status st = performReadBlocking(m);
    if (st.ok()) {
      printMeasurement(m);
    } else {
      printStatus(st);
    }

  } else if (cmd == "raw") {
    printRawSample();

  } else if (cmd == "accel") {
    LSM6DS3TR::RawAxes raw;
    LSM6DS3TR::Status st = device.readAccelRaw(raw);
    if (st.ok()) {
      LSM6DS3TR::Axes a = device.convertAccel(raw);
      Serial.printf("Accel: x=%.3f y=%.3f z=%.3f g (raw: %d %d %d)\n",
                    a.x, a.y, a.z, raw.x, raw.y, raw.z);
    } else {
      printStatus(st);
    }

  } else if (cmd == "gyro") {
    LSM6DS3TR::RawAxes raw;
    LSM6DS3TR::Status st = device.readGyroRaw(raw);
    if (st.ok()) {
      LSM6DS3TR::Axes g = device.convertGyro(raw);
      Serial.printf("Gyro: x=%.2f y=%.2f z=%.2f dps (raw: %d %d %d)\n",
                    g.x, g.y, g.z, raw.x, raw.y, raw.z);
    } else {
      printStatus(st);
    }

  } else if (cmd == "temp") {
    int16_t raw = 0;
    LSM6DS3TR::Status st = device.readTemperatureRaw(raw);
    if (st.ok()) {
      float tempC = device.convertTemperature(raw);
      Serial.printf("Temp: %.2f C (raw: %d)\n", tempC, raw);
    } else {
      printStatus(st);
    }

  // --- Configuration ---
  } else if (cmd.startsWith("odrxl")) {
    int val = 0;
    if (parseIntArg(cmd, "odrxl", val)) {
      LSM6DS3TR::Odr odr = parseOdr(val);
      LSM6DS3TR::Status st = device.setAccelOdr(odr);
      if (st.ok()) {
        Serial.printf("  Accel ODR set to %s\n", odrToStr(odr));
      } else {
        printStatus(st);
      }
    } else {
      LSM6DS3TR::Odr odr;
      if (device.getAccelOdr(odr).ok()) {
        Serial.printf("  Accel ODR: %s (%u)\n", odrToStr(odr), static_cast<unsigned>(odr));
      }
    }

  } else if (cmd.startsWith("odrg")) {
    int val = 0;
    if (parseIntArg(cmd, "odrg", val)) {
      LSM6DS3TR::Odr odr = parseOdr(val);
      LSM6DS3TR::Status st = device.setGyroOdr(odr);
      if (st.ok()) {
        Serial.printf("  Gyro ODR set to %s\n", odrToStr(odr));
      } else {
        printStatus(st);
      }
    } else {
      LSM6DS3TR::Odr odr;
      if (device.getGyroOdr(odr).ok()) {
        Serial.printf("  Gyro ODR: %s (%u)\n", odrToStr(odr), static_cast<unsigned>(odr));
      }
    }

  } else if (cmd.startsWith("fsxl")) {
    int val = 0;
    if (parseIntArg(cmd, "fsxl", val)) {
      LSM6DS3TR::AccelFs fs = parseAccelFs(val);
      LSM6DS3TR::Status st = device.setAccelFs(fs);
      if (st.ok()) {
        Serial.printf("  Accel FS set to %s (sens: %.3f mg/LSB)\n",
                      accelFsToStr(fs), device.accelSensitivity());
      } else {
        printStatus(st);
      }
    } else {
      LSM6DS3TR::AccelFs fs;
      if (device.getAccelFs(fs).ok()) {
        Serial.printf("  Accel FS: %s (sens: %.3f mg/LSB)\n",
                      accelFsToStr(fs), device.accelSensitivity());
      }
    }

  } else if (cmd.startsWith("fsg")) {
    int val = 0;
    if (parseIntArg(cmd, "fsg", val)) {
      LSM6DS3TR::GyroFs fs = parseGyroFs(val);
      LSM6DS3TR::Status st = device.setGyroFs(fs);
      if (st.ok()) {
        Serial.printf("  Gyro FS set to %s (sens: %.3f mdps/LSB)\n",
                      gyroFsToStr(fs), device.gyroSensitivity());
      } else {
        printStatus(st);
      }
    } else {
      LSM6DS3TR::GyroFs fs;
      if (device.getGyroFs(fs).ok()) {
        Serial.printf("  Gyro FS: %s (sens: %.3f mdps/LSB)\n",
                      gyroFsToStr(fs), device.gyroSensitivity());
      }
    }

  } else if (cmd == "cfg" || cmd == "settings") {
    printSettings();

  // --- Diagnostics ---
  } else if (cmd == "drv") {
    printDriverHealth();

  } else if (cmd == "probe") {
    LSM6DS3TR::Status st = device.probe();
    Serial.printf("  Probe: %s%s%s\n",
                  LOG_COLOR_RESULT(st.ok()), errToStr(st.code), LOG_COLOR_RESET);

  } else if (cmd == "recover") {
    LSM6DS3TR::Status st = device.recover();
    Serial.printf("  Recover: %s%s%s\n",
                  LOG_COLOR_RESULT(st.ok()), errToStr(st.code), LOG_COLOR_RESET);
    printDriverHealth();

  } else if (cmd == "selftest") {
    selfTest();

  } else if (cmd.startsWith("stress")) {
    cancelPending();
    int count = 100;
    parseIntArg(cmd, "stress", count);
    if (count <= 0) count = 100;
    Serial.printf("  Starting stress test: %d samples...\n", count);
    stressRemaining = count;
    resetStressStats(count);
    // Kick off first measurement
    LSM6DS3TR::Status st = scheduleMeasurement();
    if (!st.inProgress() && !st.ok()) {
      noteStressError(st);
      stressStats.attempts++;
      stressRemaining--;
      if (stressRemaining == 0) finishStressStats();
    }

  } else if (cmd.startsWith("verbose")) {
    int val = -1;
    if (parseIntArg(cmd, "verbose", val)) {
      verboseMode = (val != 0);
    } else {
      verboseMode = !verboseMode;
    }
    Serial.printf("  Verbose: %s\n", verboseMode ? "ON" : "OFF");

  // --- Device ---
  } else if (cmd == "whoami") {
    uint8_t id = 0;
    LSM6DS3TR::Status st = device.readWhoAmI(id);
    if (st.ok()) {
      Serial.printf("  WHO_AM_I = 0x%02X %s\n", id,
                    id == LSM6DS3TR::cmd::WHO_AM_I_VALUE ? "(correct)" : "(UNEXPECTED!)");
    } else {
      printStatus(st);
    }

  } else if (cmd == "status") {
    uint8_t reg = 0;
    LSM6DS3TR::Status st = device.readStatusReg(reg);
    if (st.ok()) {
      Serial.printf("  STATUS_REG = 0x%02X (XLDA=%d GDA=%d TDA=%d)\n",
                    reg,
                    (reg & 0x01) ? 1 : 0,
                    (reg & 0x02) ? 1 : 0,
                    (reg & 0x04) ? 1 : 0);
    } else {
      printStatus(st);
    }

  } else if (cmd == "reset") {
    LSM6DS3TR::Status st = device.softReset();
    if (st.ok()) {
      Serial.printf("  Reset: %sOK%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
    } else {
      printStatus(st);
    }

  // --- Utility ---
  } else if (cmd == "scan") {
    i2c_scanner::scan(Wire);

  } else if (cmd == "version" || cmd == "ver") {
    Serial.printf("  LSM6DS3TR library v%s\n", LSM6DS3TR::VERSION);

  } else if (cmd == "help" || cmd == "?") {
    printHelp();

  } else {
    Serial.printf("  Unknown: '%s' (type 'help')\n", cmd.c_str());
  }
}

// ============================================================================
// Arduino Setup / Loop
// ============================================================================

void setup() {
  log_begin(115200);
  delay(100);

  LOGI("=== LSM6DS3TR-C Bringup CLI ===");
  LOGI("Library v%s", LSM6DS3TR::VERSION);

  board::initI2c();
  LOGI("I2C initialized (SDA=%d, SCL=%d)", board::I2C_SDA, board::I2C_SCL);

  i2c_scanner::scan(Wire);

  LSM6DS3TR::Config cfg;
  cfg.i2cWrite     = transport::wireWrite;
  cfg.i2cWriteRead = transport::wireWriteRead;
  cfg.i2cUser      = &Wire;
  cfg.i2cAddress   = 0x6A;
  cfg.i2cTimeoutMs = board::I2C_TIMEOUT_MS;
  cfg.nowMs        = exampleNowMs;
  cfg.offlineThreshold = 5;
  cfg.odrXl = LSM6DS3TR::Odr::HZ_104;
  cfg.odrG  = LSM6DS3TR::Odr::HZ_104;
  cfg.fsXl  = LSM6DS3TR::AccelFs::G_2;
  cfg.fsG   = LSM6DS3TR::GyroFs::DPS_250;
  cfg.bdu   = true;

  auto st = device.begin(cfg);
  if (!st.ok()) {
    LOGE("Failed to initialize device");
    printStatus(st);
    return;
  }

  LOGI("Device initialized successfully");
  printDriverHealth();

  Serial.println("\nType 'help' for commands");
  Serial.print("> ");
}

void loop() {
  device.tick(millis());

  // Stress test: schedule next measurement
  if (stressStats.active && stressRemaining > 0 && !pendingRead) {
    LSM6DS3TR::Status st = scheduleMeasurement();
    if (!st.inProgress() && !st.ok()) {
      noteStressError(st);
      stressStats.attempts++;
      stressRemaining--;
      if (stressRemaining == 0) finishStressStats();
    }
  }

  handleMeasurementReady();

  // Serial input
  static String inputBuffer;
  static constexpr size_t kMaxInputLen = 128;
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
        Serial.print("> ");
      }
    } else if (inputBuffer.length() < kMaxInputLen) {
      inputBuffer += c;
    }
  }
}
