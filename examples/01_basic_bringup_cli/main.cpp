/// @file main.cpp
/// @brief Interactive bringup and diagnostics CLI for LSM6DS3TR-C
/// @note This is an EXAMPLE, not part of the library

#include <Arduino.h>

#include <cmath>
#include <cstdlib>

#include "examples/common/BoardConfig.h"
#include "examples/common/BusDiag.h"
#include "examples/common/CliShell.h"
#include "examples/common/HealthView.h"
#include "examples/common/Log.h"
#include "examples/common/TransportAdapter.h"

#include "LSM6DS3TR/LSM6DS3TR.h"

namespace {

using LSM6DS3TR::AccelFs;
using LSM6DS3TR::AccelOffsetWeight;
using LSM6DS3TR::Config;
using LSM6DS3TR::GyroFs;
using LSM6DS3TR::Odr;

struct StressStats {
  bool active = false;
  uint32_t startMs = 0;
  uint32_t endMs = 0;
  int target = 0;
  int attempts = 0;
  int success = 0;
  uint32_t errors = 0;
  bool hasSample = false;
  float minAx = 0.0f;
  float maxAx = 0.0f;
  float minAy = 0.0f;
  float maxAy = 0.0f;
  float minAz = 0.0f;
  float maxAz = 0.0f;
  float minGx = 0.0f;
  float maxGx = 0.0f;
  float minGy = 0.0f;
  float maxGy = 0.0f;
  float minGz = 0.0f;
  float maxGz = 0.0f;
  float minTemp = 0.0f;
  float maxTemp = 0.0f;
  double sumTemp = 0.0;
  LSM6DS3TR::Status lastError = LSM6DS3TR::Status::Ok();
};

LSM6DS3TR::LSM6DS3TR device;
bool verboseMode = false;
bool continuousMode = false;
bool pendingRead = false;
uint32_t pendingStartMs = 0;
int stressRemaining = 0;
StressStats stressStats;

constexpr int DEFAULT_STRESS_COUNT = 100;
constexpr int MAX_STRESS_COUNT = 100000;
constexpr int DEFAULT_DUMP_START = 0x10;
constexpr int DEFAULT_DUMP_LEN = 32;
constexpr int MAX_DUMP_LEN = 128;
constexpr int SELF_TEST_SAMPLES = 5;
constexpr uint8_t SELF_TEST_CTRL1_XL = 0x60;
constexpr uint8_t SELF_TEST_CTRL2_G = 0x60;
constexpr uint8_t SELF_TEST_CTRL5_XL_POS = 0x01;
constexpr uint8_t SELF_TEST_CTRL5_G_POS = 0x04;

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
    case Err::TIMEOUT:               return "TIMEOUT";
    case Err::INVALID_PARAM:         return "INVALID_PARAM";
    case Err::DEVICE_NOT_FOUND:      return "DEVICE_NOT_FOUND";
    case Err::CHIP_ID_MISMATCH:      return "CHIP_ID_MISMATCH";
    case Err::MEASUREMENT_NOT_READY: return "MEASUREMENT_NOT_READY";
    case Err::BUSY:                  return "BUSY";
    case Err::IN_PROGRESS:           return "IN_PROGRESS";
    case Err::SELF_TEST_FAIL:        return "SELF_TEST_FAIL";
    case Err::I2C_NACK_ADDR:         return "I2C_NACK_ADDR";
    case Err::I2C_NACK_DATA:         return "I2C_NACK_DATA";
    case Err::I2C_TIMEOUT:           return "I2C_TIMEOUT";
    case Err::I2C_BUS:               return "I2C_BUS";
    case Err::FIFO_EMPTY:            return "FIFO_EMPTY";
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
  if (st == LSM6DS3TR::DriverState::UNINIT) {
    return LOG_COLOR_RESET;
  }
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
    case Odr::HZ_1_6:     return "1.6 Hz";
    case Odr::HZ_12_5:    return "12.5 Hz";
    case Odr::HZ_26:      return "26 Hz";
    case Odr::HZ_52:      return "52 Hz";
    case Odr::HZ_104:     return "104 Hz";
    case Odr::HZ_208:     return "208 Hz";
    case Odr::HZ_416:     return "416 Hz";
    case Odr::HZ_833:     return "833 Hz";
    case Odr::HZ_1660:    return "1660 Hz";
    case Odr::HZ_3330:    return "3330 Hz";
    case Odr::HZ_6660:    return "6660 Hz";
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

const char* accelPowerModeToStr(LSM6DS3TR::AccelPowerMode mode) {
  switch (mode) {
    case LSM6DS3TR::AccelPowerMode::HIGH_PERFORMANCE: return "high-performance";
    case LSM6DS3TR::AccelPowerMode::LOW_POWER_NORMAL: return "low-power/normal";
    default:                                          return "UNKNOWN";
  }
}

const char* gyroPowerModeToStr(LSM6DS3TR::GyroPowerMode mode) {
  switch (mode) {
    case LSM6DS3TR::GyroPowerMode::HIGH_PERFORMANCE: return "high-performance";
    case LSM6DS3TR::GyroPowerMode::LOW_POWER_NORMAL: return "low-power/normal";
    default:                                         return "UNKNOWN";
  }
}

const char* gyroHpfModeToStr(LSM6DS3TR::GyroHpfMode mode) {
  switch (mode) {
    case LSM6DS3TR::GyroHpfMode::HZ_0_0081: return "0.0081";
    case LSM6DS3TR::GyroHpfMode::HZ_0_0324: return "0.0324";
    case LSM6DS3TR::GyroHpfMode::HZ_2_07:   return "2.07";
    case LSM6DS3TR::GyroHpfMode::HZ_16_32:  return "16.32";
    default:                                return "UNKNOWN";
  }
}

const char* fifoModeToStr(LSM6DS3TR::FifoMode mode) {
  switch (mode) {
    case LSM6DS3TR::FifoMode::BYPASS:               return "bypass";
    case LSM6DS3TR::FifoMode::FIFO:                 return "fifo";
    case LSM6DS3TR::FifoMode::CONTINUOUS_TO_FIFO:   return "continuous-to-fifo";
    case LSM6DS3TR::FifoMode::BYPASS_TO_CONTINUOUS: return "bypass-to-continuous";
    case LSM6DS3TR::FifoMode::CONTINUOUS:           return "continuous";
    default:                                        return "UNKNOWN";
  }
}

const char* fifoDecimationToStr(LSM6DS3TR::FifoDecimation decimation) {
  switch (decimation) {
    case LSM6DS3TR::FifoDecimation::DISABLED: return "off";
    case LSM6DS3TR::FifoDecimation::NONE:     return "1";
    case LSM6DS3TR::FifoDecimation::DIV_2:    return "2";
    case LSM6DS3TR::FifoDecimation::DIV_3:    return "3";
    case LSM6DS3TR::FifoDecimation::DIV_4:    return "4";
    case LSM6DS3TR::FifoDecimation::DIV_8:    return "8";
    case LSM6DS3TR::FifoDecimation::DIV_16:   return "16";
    case LSM6DS3TR::FifoDecimation::DIV_32:   return "32";
    default:                                  return "UNKNOWN";
  }
}

const char* offsetWeightToStr(AccelOffsetWeight weight) {
  switch (weight) {
    case AccelOffsetWeight::MG_1:  return "1 mg/LSB";
    case AccelOffsetWeight::MG_16: return "16 mg/LSB";
    default:                       return "UNKNOWN";
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

void printVersionInfo() {
  Serial.println("=== Version ===");
  Serial.printf("  Version: %s\n", LSM6DS3TR::VERSION);
  Serial.printf("  Full:    %s\n", LSM6DS3TR::VERSION_FULL);
  Serial.printf("  Built:   %s\n", LSM6DS3TR::BUILD_TIMESTAMP);
  Serial.printf("  Commit:  %s\n", LSM6DS3TR::GIT_COMMIT);
  Serial.printf("  Status:  %s\n", LSM6DS3TR::GIT_STATUS);
}

bool isReservedRegister(uint8_t reg) {
  return reg >= 0x43u && reg <= 0x48u;
}

size_t splitTokens(const String& line, String* tokens, size_t maxTokens) {
  String work = line;
  work.trim();
  size_t count = 0;
  int start = 0;
  while (start < work.length() && count < maxTokens) {
    while (start < work.length() && work[start] == ' ') {
      start++;
    }
    if (start >= work.length()) {
      break;
    }
    int end = work.indexOf(' ', start);
    if (end < 0) {
      tokens[count++] = work.substring(start);
      break;
    }
    tokens[count++] = work.substring(start, end);
    start = end + 1;
  }
  return count;
}

bool parseUnsignedToken(const String& token, unsigned long maxValue, unsigned long& outValue) {
  if (token.length() == 0) {
    return false;
  }
  char* end = nullptr;
  const bool isHex = token.startsWith("0x") || token.startsWith("0X");
  const unsigned long value = strtoul(token.c_str(), &end, isHex ? 16 : 10);
  if (end == token.c_str() || *end != '\0' || value > maxValue) {
    return false;
  }
  outValue = value;
  return true;
}

bool parseSignedToken(const String& token, long minValue, long maxValue, long& outValue) {
  if (token.length() == 0) {
    return false;
  }
  char* end = nullptr;
  const long value = strtol(token.c_str(), &end, 10);
  if (end == token.c_str() || *end != '\0' || value < minValue || value > maxValue) {
    return false;
  }
  outValue = value;
  return true;
}

bool parseBoolToken(String token, bool& outValue) {
  token.toLowerCase();
  if (token == "1" || token == "on" || token == "true" || token == "yes" || token == "enable") {
    outValue = true;
    return true;
  }
  if (token == "0" || token == "off" || token == "false" || token == "no" || token == "disable") {
    outValue = false;
    return true;
  }
  return false;
}

bool parseOdrToken(String token, LSM6DS3TR::Odr& outValue) {
  token.toLowerCase();
  if (token == "0" || token == "off" || token == "pd" || token == "powerdown") { outValue = LSM6DS3TR::Odr::POWER_DOWN; return true; }
  if (token == "1" || token == "1.6" || token == "1p6") { outValue = LSM6DS3TR::Odr::HZ_1_6; return true; }
  if (token == "12" || token == "12.5" || token == "12p5") { outValue = LSM6DS3TR::Odr::HZ_12_5; return true; }
  if (token == "26") { outValue = LSM6DS3TR::Odr::HZ_26; return true; }
  if (token == "52") { outValue = LSM6DS3TR::Odr::HZ_52; return true; }
  if (token == "104") { outValue = LSM6DS3TR::Odr::HZ_104; return true; }
  if (token == "208") { outValue = LSM6DS3TR::Odr::HZ_208; return true; }
  if (token == "416") { outValue = LSM6DS3TR::Odr::HZ_416; return true; }
  if (token == "833") { outValue = LSM6DS3TR::Odr::HZ_833; return true; }
  if (token == "1660") { outValue = LSM6DS3TR::Odr::HZ_1660; return true; }
  if (token == "3330") { outValue = LSM6DS3TR::Odr::HZ_3330; return true; }
  if (token == "6660") { outValue = LSM6DS3TR::Odr::HZ_6660; return true; }
  return false;
}

bool parseAccelFsToken(String token, LSM6DS3TR::AccelFs& outValue) {
  token.toLowerCase();
  if (token == "2" || token == "2g") { outValue = LSM6DS3TR::AccelFs::G_2; return true; }
  if (token == "4" || token == "4g") { outValue = LSM6DS3TR::AccelFs::G_4; return true; }
  if (token == "8" || token == "8g") { outValue = LSM6DS3TR::AccelFs::G_8; return true; }
  if (token == "16" || token == "16g") { outValue = LSM6DS3TR::AccelFs::G_16; return true; }
  return false;
}

bool parseGyroFsToken(String token, LSM6DS3TR::GyroFs& outValue) {
  token.toLowerCase();
  if (token == "125") { outValue = LSM6DS3TR::GyroFs::DPS_125; return true; }
  if (token == "250") { outValue = LSM6DS3TR::GyroFs::DPS_250; return true; }
  if (token == "500") { outValue = LSM6DS3TR::GyroFs::DPS_500; return true; }
  if (token == "1000") { outValue = LSM6DS3TR::GyroFs::DPS_1000; return true; }
  if (token == "2000") { outValue = LSM6DS3TR::GyroFs::DPS_2000; return true; }
  return false;
}

bool parseAccelPowerModeToken(String token, LSM6DS3TR::AccelPowerMode& outValue) {
  token.toLowerCase();
  if (token == "hp" || token == "high") { outValue = LSM6DS3TR::AccelPowerMode::HIGH_PERFORMANCE; return true; }
  if (token == "lpn" || token == "lp" || token == "normal") { outValue = LSM6DS3TR::AccelPowerMode::LOW_POWER_NORMAL; return true; }
  return false;
}

bool parseGyroPowerModeToken(String token, LSM6DS3TR::GyroPowerMode& outValue) {
  token.toLowerCase();
  if (token == "hp" || token == "high") { outValue = LSM6DS3TR::GyroPowerMode::HIGH_PERFORMANCE; return true; }
  if (token == "lpn" || token == "lp" || token == "normal") { outValue = LSM6DS3TR::GyroPowerMode::LOW_POWER_NORMAL; return true; }
  return false;
}

bool parseGyroHpfModeToken(String token, LSM6DS3TR::GyroHpfMode& outValue) {
  token.toLowerCase();
  if (token == "0") { outValue = LSM6DS3TR::GyroHpfMode::HZ_0_0081; return true; }
  if (token == "1") { outValue = LSM6DS3TR::GyroHpfMode::HZ_0_0324; return true; }
  if (token == "2") { outValue = LSM6DS3TR::GyroHpfMode::HZ_2_07; return true; }
  if (token == "3") { outValue = LSM6DS3TR::GyroHpfMode::HZ_16_32; return true; }
  return false;
}

bool parseOffsetWeightToken(String token, AccelOffsetWeight& outValue) {
  token.toLowerCase();
  if (token == "1") { outValue = AccelOffsetWeight::MG_1; return true; }
  if (token == "16") { outValue = AccelOffsetWeight::MG_16; return true; }
  return false;
}

bool parseFifoModeToken(String token, LSM6DS3TR::FifoMode& outValue) {
  token.toLowerCase();
  if (token == "bypass") { outValue = LSM6DS3TR::FifoMode::BYPASS; return true; }
  if (token == "fifo") { outValue = LSM6DS3TR::FifoMode::FIFO; return true; }
  if (token == "c2f" || token == "continuous-to-fifo") { outValue = LSM6DS3TR::FifoMode::CONTINUOUS_TO_FIFO; return true; }
  if (token == "b2c" || token == "bypass-to-continuous") { outValue = LSM6DS3TR::FifoMode::BYPASS_TO_CONTINUOUS; return true; }
  if (token == "cont" || token == "continuous") { outValue = LSM6DS3TR::FifoMode::CONTINUOUS; return true; }
  return false;
}

bool parseFifoDecimationToken(String token, LSM6DS3TR::FifoDecimation& outValue) {
  token.toLowerCase();
  if (token == "off" || token == "0" || token == "disabled") { outValue = LSM6DS3TR::FifoDecimation::DISABLED; return true; }
  if (token == "1" || token == "none") { outValue = LSM6DS3TR::FifoDecimation::NONE; return true; }
  if (token == "2") { outValue = LSM6DS3TR::FifoDecimation::DIV_2; return true; }
  if (token == "3") { outValue = LSM6DS3TR::FifoDecimation::DIV_3; return true; }
  if (token == "4") { outValue = LSM6DS3TR::FifoDecimation::DIV_4; return true; }
  if (token == "8") { outValue = LSM6DS3TR::FifoDecimation::DIV_8; return true; }
  if (token == "16") { outValue = LSM6DS3TR::FifoDecimation::DIV_16; return true; }
  if (token == "32") { outValue = LSM6DS3TR::FifoDecimation::DIV_32; return true; }
  return false;
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
  if (device.lastOkMs() > 0U) {
    Serial.printf("  Last OK: %lu ms ago\n", static_cast<unsigned long>(now - device.lastOkMs()));
  } else {
    Serial.println("  Last OK: never");
  }
  if (device.lastErrorMs() > 0U) {
    Serial.printf("  Last error: %lu ms ago\n", static_cast<unsigned long>(now - device.lastErrorMs()));
  } else {
    Serial.println("  Last error: never");
  }
  if (!lastErr.ok()) {
    printStatus(lastErr);
  }
}

void printMeasurement(const LSM6DS3TR::Measurement& m) {
  Serial.printf("Sample: ax=%+.3f ay=%+.3f az=%+.3f g | gx=%+.2f gy=%+.2f gz=%+.2f dps | t=%.2f C\n",
                m.accel.x, m.accel.y, m.accel.z,
                m.gyro.x, m.gyro.y, m.gyro.z,
                m.temperatureC);
}

void printRawMeasurement(const LSM6DS3TR::RawMeasurement& raw) {
  Serial.printf("Raw: ax=%d ay=%d az=%d | gx=%d gy=%d gz=%d | t=%d\n",
                raw.accel.x, raw.accel.y, raw.accel.z,
                raw.gyro.x, raw.gyro.y, raw.gyro.z,
                raw.temperature);
}

void printHexDump(uint8_t startReg, const uint8_t* data, size_t len) {
  for (size_t offset = 0; offset < len; offset += 16) {
    const size_t lineLen = ((len - offset) > 16U) ? 16U : (len - offset);
    Serial.printf("  %02X: ", static_cast<unsigned>(startReg + offset));
    for (size_t i = 0; i < lineLen; ++i) {
      Serial.printf("%02X ", data[offset + i]);
    }
    for (size_t i = lineLen; i < 16U; ++i) {
      Serial.print("   ");
    }
    Serial.print(" |");
    for (size_t i = 0; i < lineLen; ++i) {
      const char c = static_cast<char>(data[offset + i]);
      Serial.print((c >= 0x20 && c < 0x7F) ? c : '.');
    }
    Serial.println("|");
  }
}

void printSettings() {
  LSM6DS3TR::Odr odrXl = LSM6DS3TR::Odr::POWER_DOWN;
  LSM6DS3TR::Odr odrG = LSM6DS3TR::Odr::POWER_DOWN;
  LSM6DS3TR::AccelFs fsXl = LSM6DS3TR::AccelFs::G_2;
  LSM6DS3TR::GyroFs fsG = LSM6DS3TR::GyroFs::DPS_250;
  LSM6DS3TR::AccelPowerMode accelPower = LSM6DS3TR::AccelPowerMode::HIGH_PERFORMANCE;
  LSM6DS3TR::GyroPowerMode gyroPower = LSM6DS3TR::GyroPowerMode::HIGH_PERFORMANCE;
  bool gyroSleep = false;
  bool timestampEnabled = false;
  bool timestampHighResolution = false;
  bool pedometer = false;
  bool significantMotion = false;
  bool tilt = false;
  bool wristTilt = false;
  LSM6DS3TR::AccelFilterConfig accelFilter;
  LSM6DS3TR::GyroFilterConfig gyroFilter;
  AccelOffsetWeight offsetWeight = AccelOffsetWeight::MG_1;
  LSM6DS3TR::AccelUserOffset offset;
  LSM6DS3TR::FifoConfig fifo;

  LSM6DS3TR::Status st = device.getAccelOdr(odrXl);
  if (!st.ok()) {
    printStatus(st);
    return;
  }
  (void)device.getGyroOdr(odrG);
  (void)device.getAccelFs(fsXl);
  (void)device.getGyroFs(fsG);
  (void)device.getAccelPowerMode(accelPower);
  (void)device.getGyroPowerMode(gyroPower);
  (void)device.getGyroSleepEnabled(gyroSleep);
  (void)device.getTimestampEnabled(timestampEnabled);
  (void)device.getTimestampHighResolution(timestampHighResolution);
  (void)device.getPedometerEnabled(pedometer);
  (void)device.getSignificantMotionEnabled(significantMotion);
  (void)device.getTiltEnabled(tilt);
  (void)device.getWristTiltEnabled(wristTilt);
  (void)device.getAccelFilterConfig(accelFilter);
  (void)device.getGyroFilterConfig(gyroFilter);
  (void)device.getAccelOffsetWeight(offsetWeight);
  (void)device.getAccelUserOffset(offset);
  (void)device.getFifoConfig(fifo);

  Serial.println("=== Current Settings ===");
  Serial.printf("  Accel ODR:          %s\n", odrToStr(odrXl));
  Serial.printf("  Gyro ODR:           %s\n", odrToStr(odrG));
  Serial.printf("  Accel FS:           %s\n", accelFsToStr(fsXl));
  Serial.printf("  Gyro FS:            %s\n", gyroFsToStr(fsG));
  Serial.printf("  Accel power:        %s\n", accelPowerModeToStr(accelPower));
  Serial.printf("  Gyro power:         %s\n", gyroPowerModeToStr(gyroPower));
  Serial.printf("  Gyro sleep:         %s\n", log_bool_str(gyroSleep));
  Serial.printf("  Accel LPF2:         %s\n", log_bool_str(accelFilter.lpf2Enabled));
  Serial.printf("  Accel HP/slope:     %s\n", log_bool_str(accelFilter.highPassSlopeEnabled));
  Serial.printf("  Accel LPF on 6D:    %s\n", log_bool_str(accelFilter.lowPassOn6d));
  Serial.printf("  Gyro LPF1:          %s\n", log_bool_str(gyroFilter.lpf1Enabled));
  Serial.printf("  Gyro HPF:           %s\n", log_bool_str(gyroFilter.highPassEnabled));
  Serial.printf("  Gyro HPF mode:      %s\n", gyroHpfModeToStr(gyroFilter.highPassMode));
  Serial.printf("  Timestamp:          %s\n", log_bool_str(timestampEnabled));
  Serial.printf("  Timestamp HR:       %s\n", log_bool_str(timestampHighResolution));
  Serial.printf("  Pedometer:          %s\n", log_bool_str(pedometer));
  Serial.printf("  Significant motion: %s\n", log_bool_str(significantMotion));
  Serial.printf("  Tilt:               %s\n", log_bool_str(tilt));
  Serial.printf("  Wrist tilt:         %s\n", log_bool_str(wristTilt));
  Serial.printf("  Offset weight:      %s\n", offsetWeightToStr(offsetWeight));
  Serial.printf("  User offset:        x=%d y=%d z=%d\n", offset.x, offset.y, offset.z);
  Serial.printf("  FIFO mode:          %s\n", fifoModeToStr(fifo.mode));
  Serial.printf("  FIFO ODR:           %s\n", odrToStr(fifo.odr));
  Serial.printf("  FIFO threshold:     %u\n", fifo.threshold);
  Serial.printf("  FIFO accel decim:   %s\n", fifoDecimationToStr(fifo.accelDecimation));
  Serial.printf("  FIFO gyro decim:    %s\n", fifoDecimationToStr(fifo.gyroDecimation));
  Serial.printf("  FIFO temp:          %s\n", log_bool_str(fifo.storeTemperature));
  Serial.printf("  FIFO step/time:     %s\n", log_bool_str(fifo.storeTimestampStep));
  Serial.printf("  FIFO stop@th:       %s\n", log_bool_str(fifo.stopOnThreshold));
  Serial.printf("  FIFO high-only:     %s\n", log_bool_str(fifo.onlyHighData));
}

LSM6DS3TR::Status performReadBlocking(LSM6DS3TR::Measurement& out) {
  LSM6DS3TR::RawMeasurement raw;
  LSM6DS3TR::Status st = device.readAllRaw(raw);
  if (!st.ok()) {
    return st;
  }
  out.accel = device.convertAccel(raw.accel);
  out.gyro = device.convertGyro(raw.gyro);
  out.temperatureC = device.convertTemperature(raw.temperature);
  device.correctAccel(out.accel);
  device.correctGyro(out.gyro);
  return LSM6DS3TR::Status::Ok();
}

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
  Serial.printf("  Target:   %d\n", stressStats.target);
  Serial.printf("  Attempts: %d\n", stressStats.attempts);
  Serial.printf("  Success:  %d\n", stressStats.success);
  Serial.printf("  Errors:   %lu\n", static_cast<unsigned long>(stressStats.errors));
  Serial.printf("  Duration: %lu ms\n", static_cast<unsigned long>(durationMs));
  if (durationMs > 0U) {
    Serial.printf("  Rate:     %.2f samples/s\n",
                  1000.0f * static_cast<float>(stressStats.attempts) / static_cast<float>(durationMs));
  }
  if (!stressStats.lastError.ok()) {
    printStatus(stressStats.lastError);
  }
}

void cancelPending() {
  pendingRead = false;
  stressRemaining = 0;
  continuousMode = false;
  if (stressStats.active) {
    stressStats.active = false;
    Serial.println("  Cancelled.");
  }
}

LSM6DS3TR::Status scheduleMeasurement() {
  const LSM6DS3TR::Status st = device.requestMeasurement();
  if (st.inProgress()) {
    pendingRead = true;
    pendingStartMs = millis();
  }
  return st;
}

void handleMeasurementReady() {
  if (!pendingRead || !device.measurementReady()) {
    return;
  }
  pendingRead = false;
  LSM6DS3TR::Measurement m;
  const LSM6DS3TR::Status st = device.getMeasurement(m);
  if (!st.ok()) {
    if (stressStats.active) {
      noteStressError(st);
      stressStats.attempts++;
      stressRemaining--;
      if (stressRemaining == 0) {
        finishStressStats();
      }
    } else {
      printStatus(st);
    }
    return;
  }
  if (stressStats.active) {
    updateStressStats(m);
    stressStats.attempts++;
    stressRemaining--;
    if (stressRemaining == 0) {
      finishStressStats();
    }
  } else {
    if (verboseMode) {
      Serial.printf("  Async latency: %lu ms\n",
                    static_cast<unsigned long>(millis() - pendingStartMs));
    }
    printMeasurement(m);
  }
}

bool readRegisterSnapshot(const uint8_t* regs, uint8_t* values, size_t count) {
  for (size_t i = 0; i < count; ++i) {
    LSM6DS3TR::Status st = device.readRegisterValue(regs[i], values[i]);
    if (!st.ok()) {
      printStatus(st);
      return false;
    }
  }
  return true;
}

void restoreRegisterSnapshot(const uint8_t* regs, const uint8_t* values, size_t count) {
  for (size_t i = 0; i < count; ++i) {
    const size_t index = count - 1U - i;
    const LSM6DS3TR::Status st = device.writeRegisterValue(regs[index], values[index]);
    if (!st.ok()) {
      printStatus(st);
    }
  }
  (void)device.refreshCachedConfig();
}

bool averageAccelSamples(int sampleCount, LSM6DS3TR::RawAxes& out) {
  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;
  for (int i = 0; i < sampleCount; ++i) {
    LSM6DS3TR::RawAxes raw;
    const LSM6DS3TR::Status st = device.readAccelRaw(raw);
    if (!st.ok()) {
      printStatus(st);
      return false;
    }
    sumX += raw.x;
    sumY += raw.y;
    sumZ += raw.z;
    delay(10);
  }
  out.x = static_cast<int16_t>(sumX / sampleCount);
  out.y = static_cast<int16_t>(sumY / sampleCount);
  out.z = static_cast<int16_t>(sumZ / sampleCount);
  return true;
}

bool averageGyroSamples(int sampleCount, LSM6DS3TR::RawAxes& out) {
  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;
  for (int i = 0; i < sampleCount; ++i) {
    LSM6DS3TR::RawAxes raw;
    const LSM6DS3TR::Status st = device.readGyroRaw(raw);
    if (!st.ok()) {
      printStatus(st);
      return false;
    }
    sumX += raw.x;
    sumY += raw.y;
    sumZ += raw.z;
    delay(10);
  }
  out.x = static_cast<int16_t>(sumX / sampleCount);
  out.y = static_cast<int16_t>(sumY / sampleCount);
  out.z = static_cast<int16_t>(sumZ / sampleCount);
  return true;
}

void selfTest() {
  Serial.println("=== Self-Test ===");
  const LSM6DS3TR::Status probeStatus = device.probe();
  Serial.printf("  Probe: %s%s%s\n", LOG_COLOR_RESULT(probeStatus.ok()), errToStr(probeStatus.code), LOG_COLOR_RESET);
  if (!probeStatus.ok()) {
    return;
  }

  uint8_t whoAmI = 0;
  LSM6DS3TR::Status st = device.readWhoAmI(whoAmI);
  if (!st.ok()) {
    printStatus(st);
    return;
  }
  Serial.printf("  WHO_AM_I: 0x%02X\n", whoAmI);

  bool accelPass = false;
  bool gyroPass = false;
  static const uint8_t accelRegs[] = {LSM6DS3TR::cmd::REG_CTRL1_XL, LSM6DS3TR::cmd::REG_CTRL5_C, LSM6DS3TR::cmd::REG_CTRL8_XL};
  static const uint8_t gyroRegs[] = {LSM6DS3TR::cmd::REG_CTRL2_G, LSM6DS3TR::cmd::REG_CTRL5_C, LSM6DS3TR::cmd::REG_CTRL7_G};
  uint8_t accelSaved[sizeof(accelRegs)] = {};
  uint8_t gyroSaved[sizeof(gyroRegs)] = {};

  if (readRegisterSnapshot(accelRegs, accelSaved, sizeof(accelRegs))) {
    if (device.writeRegisterValue(LSM6DS3TR::cmd::REG_CTRL1_XL, SELF_TEST_CTRL1_XL).ok() &&
        device.writeRegisterValue(LSM6DS3TR::cmd::REG_CTRL8_XL, 0x00).ok() &&
        device.writeRegisterValue(LSM6DS3TR::cmd::REG_CTRL5_C, 0x00).ok()) {
      delay(100);
      LSM6DS3TR::RawAxes nost;
      LSM6DS3TR::RawAxes test;
      if (averageAccelSamples(SELF_TEST_SAMPLES, nost) &&
          device.writeRegisterValue(LSM6DS3TR::cmd::REG_CTRL5_C, SELF_TEST_CTRL5_XL_POS).ok()) {
        delay(100);
        if (averageAccelSamples(SELF_TEST_SAMPLES, test)) {
          const float dx = fabsf(static_cast<float>(test.x - nost.x)) * 0.061f;
          const float dy = fabsf(static_cast<float>(test.y - nost.y)) * 0.061f;
          const float dz = fabsf(static_cast<float>(test.z - nost.z)) * 0.061f;
          accelPass = (dx >= 90.0f && dx <= 1700.0f) &&
                      (dy >= 90.0f && dy <= 1700.0f) &&
                      (dz >= 90.0f && dz <= 1700.0f);
          Serial.printf("  Accel delta: x=%.1f y=%.1f z=%.1f mg %s%s%s\n",
                        dx, dy, dz, LOG_COLOR_RESULT(accelPass), accelPass ? "PASS" : "FAIL", LOG_COLOR_RESET);
        }
      }
    }
    restoreRegisterSnapshot(accelRegs, accelSaved, sizeof(accelRegs));
  }

  if (readRegisterSnapshot(gyroRegs, gyroSaved, sizeof(gyroRegs))) {
    if (device.writeRegisterValue(LSM6DS3TR::cmd::REG_CTRL2_G, SELF_TEST_CTRL2_G).ok() &&
        device.writeRegisterValue(LSM6DS3TR::cmd::REG_CTRL7_G, 0x00).ok() &&
        device.writeRegisterValue(LSM6DS3TR::cmd::REG_CTRL5_C, 0x00).ok()) {
      delay(800);
      LSM6DS3TR::RawAxes nost;
      LSM6DS3TR::RawAxes test;
      if (averageGyroSamples(SELF_TEST_SAMPLES, nost) &&
          device.writeRegisterValue(LSM6DS3TR::cmd::REG_CTRL5_C, SELF_TEST_CTRL5_G_POS).ok()) {
        delay(60);
        if (averageGyroSamples(SELF_TEST_SAMPLES, test)) {
          const float dx = fabsf(static_cast<float>(test.x - nost.x)) * 0.00875f;
          const float dy = fabsf(static_cast<float>(test.y - nost.y)) * 0.00875f;
          const float dz = fabsf(static_cast<float>(test.z - nost.z)) * 0.00875f;
          gyroPass = (dx >= 20.0f && dx <= 80.0f) &&
                     (dy >= 20.0f && dy <= 80.0f) &&
                     (dz >= 20.0f && dz <= 80.0f);
          Serial.printf("  Gyro delta:  x=%.2f y=%.2f z=%.2f dps %s%s%s\n",
                        dx, dy, dz, LOG_COLOR_RESULT(gyroPass), gyroPass ? "PASS" : "FAIL", LOG_COLOR_RESET);
        }
      }
    }
    restoreRegisterSnapshot(gyroRegs, gyroSaved, sizeof(gyroRegs));
  }

  const bool overallPass = accelPass && gyroPass;
  Serial.printf("  Overall: %s%s%s\n", LOG_COLOR_RESULT(overallPass), overallPass ? "PASS" : "FAIL", LOG_COLOR_RESET);
}

void runStressMix(int count) {
  if (count <= 0) count = DEFAULT_STRESS_COUNT;
  if (count > MAX_STRESS_COUNT) count = MAX_STRESS_COUNT;
  uint32_t errors = 0;
  LSM6DS3TR::Status lastError = LSM6DS3TR::Status::Ok();
  const uint32_t startMs = millis();
  for (int i = 0; i < count; ++i) {
    LSM6DS3TR::Status st = LSM6DS3TR::Status::Ok();
    switch (i % 6) {
      case 0: { LSM6DS3TR::RawMeasurement raw; st = device.readAllRaw(raw); break; }
      case 1: { LSM6DS3TR::RawAxes raw; st = device.readAccelRaw(raw); break; }
      case 2: { LSM6DS3TR::RawAxes raw; st = device.readGyroRaw(raw); break; }
      case 3: { int16_t raw = 0; st = device.readTemperatureRaw(raw); break; }
      case 4: { uint8_t statusReg = 0; st = device.readStatusReg(statusReg); break; }
      default: { uint8_t id = 0; st = device.readWhoAmI(id); break; }
    }
    if (!st.ok()) {
      errors++;
      lastError = st;
    }
  }
  const uint32_t durationMs = millis() - startMs;
  Serial.println("=== Mixed Stress Summary ===");
  Serial.printf("  Iterations: %d\n", count);
  Serial.printf("  Errors:     %lu\n", static_cast<unsigned long>(errors));
  Serial.printf("  Duration:   %lu ms\n", static_cast<unsigned long>(durationMs));
  if (!lastError.ok()) {
    printStatus(lastError);
  }
}

void printRegisterValue(const char* label, uint8_t value) {
  Serial.printf("  %s = 0x%02X\n", label, value);
}

void printHelp() {
  auto helpSection = [](const char* title) {
    Serial.printf("\n%s[%s]%s\n", LOG_COLOR_GREEN, title, LOG_COLOR_RESET);
  };
  auto helpItem = [](const char* cmd, const char* desc) {
    Serial.printf("  %s%-24s%s - %s\n", LOG_COLOR_CYAN, cmd, LOG_COLOR_RESET, desc);
  };

  Serial.println();
  Serial.printf("%s=== LSM6DS3TR-C CLI Help ===%s\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  Serial.printf("Version: %s\n", LSM6DS3TR::VERSION_FULL);

  helpSection("Common");
  helpItem("help / ?", "Show this help");
  helpItem("version / ver", "Print version and build metadata");
  helpItem("scan", "Scan I2C bus");
  helpItem("drv", "Show driver health");
  helpItem("drv1", "Show one-line health view");
  helpItem("cfg / settings", "Show current cached configuration");
  helpItem("refresh", "Refresh cached config from device registers");
  helpItem("verbose [0|1]", "Toggle or set verbose mode");

  helpSection("Measurement");
  helpItem("read", "Read accel + gyro + temp (converted)");
  helpItem("raw", "Read accel + gyro + temp (raw)");
  helpItem("accel", "Read accelerometer only");
  helpItem("gyro", "Read gyroscope only");
  helpItem("temp", "Read temperature only");
  helpItem("status", "Read STATUS_REG with decoded data-ready bits");
  helpItem("tsread", "Read timestamp counter");
  helpItem("steps", "Read step counter and step timestamp");
  helpItem("fifo", "Read FIFO config and status");
  helpItem("fifo_read [N]", "Read up to N FIFO words");
  helpItem("stream", "Toggle continuous output (one line per sample at sensor ODR)");

  helpSection("Core Config");
  helpItem("odrxl [hz]", "Get/set accel ODR (off|1.6|12.5|26|52|104|208|416|833|1660|3330|6660)");
  helpItem("odrg [hz]", "Get/set gyro ODR (off|12.5|26|52|104|208|416|833|1660|3330|6660)");
  helpItem("fsxl [g]", "Get/set accel full-scale (2|4|8|16)");
  helpItem("fsg [dps]", "Get/set gyro full-scale (125|250|500|1000|2000)");
  helpItem("apm [hp|lpn]", "Get/set accel power mode");
  helpItem("gpm [hp|lpn]", "Get/set gyro power mode");
  helpItem("gsleep [0|1]", "Get/set gyro sleep");
  helpItem("reset", "Software reset and re-apply cached config");
  helpItem("boot", "Issue boot command and refresh cached config");

  helpSection("Filters And Embedded");
  helpItem("alpf2 [0|1]", "Get/set accel LPF2");
  helpItem("aslope [0|1]", "Get/set accel high-pass/slope");
  helpItem("a6d [0|1]", "Get/set accel low-pass on 6D");
  helpItem("glpf1 [0|1]", "Get/set gyro LPF1");
  helpItem("ghpf [0|1]", "Get/set gyro HPF");
  helpItem("ghpfmode [0..3]", "Get/set gyro HPF cutoff");
  helpItem("ts [0|1]", "Get/set timestamp enable");
  helpItem("tshr [0|1]", "Get/set timestamp high resolution");
  helpItem("tsreset", "Reset timestamp counter");
  helpItem("pedo [0|1]", "Get/set pedometer");
  helpItem("sigmot [0|1]", "Get/set significant motion");
  helpItem("tilt [0|1]", "Get/set tilt detection");
  helpItem("wtilt [0|1]", "Get/set wrist tilt");
  helpItem("stepreset", "Reset step counter");

  helpSection("Offsets And FIFO");
  helpItem("ofswt [1|16]", "Get/set accel offset weight");
  helpItem("offset [x y z]", "Get/set accel user offsets");
  helpItem("fifo_mode [mode]", "Get/set FIFO mode");
  helpItem("fifo_odr [hz]", "Get/set FIFO ODR");
  helpItem("fifo_xl [off|1|2|3|4|8|16|32]", "Get/set FIFO accel decimation");
  helpItem("fifo_g [off|1|2|3|4|8|16|32]", "Get/set FIFO gyro decimation");
  helpItem("fifo_th [N]", "Get/set FIFO threshold (0..2047)");
  helpItem("fifo_temp [0|1]", "Get/set FIFO temperature storage");
  helpItem("fifo_step [0|1]", "Get/set FIFO step/timestamp storage");
  helpItem("fifo_stop [0|1]", "Get/set FIFO stop on threshold");
  helpItem("fifo_high [0|1]", "Get/set FIFO high-byte-only mode");

  helpSection("Calibration");
  helpItem("cal [N]", "Calibrate accel+gyro at rest (Z-up, default 100 samples)");
  helpItem("calxl [N]", "Calibrate accel only (Z-up, default 100 samples)");
  helpItem("calg [N]", "Calibrate gyro only (stationary, default 100 samples)");
  helpItem("biasxl [x y z]", "Get/set accel software bias (g)");
  helpItem("biasg [x y z]", "Get/set gyro software bias (dps)");
  helpItem("biasreset", "Clear all software biases to zero");

  helpSection("Diagnostics");
  helpItem("probe", "Probe WHO_AM_I without health tracking");
  helpItem("recover", "Retry initialization/recovery");
  helpItem("whoami", "Read WHO_AM_I register");
  helpItem("wusrc", "Read WAKE_UP_SRC");
  helpItem("tapsrc", "Read TAP_SRC");
  helpItem("6dsrc", "Read D6D_SRC");
  helpItem("funcsrc1", "Read FUNC_SRC1");
  helpItem("funcsrc2", "Read FUNC_SRC2");
  helpItem("wtstatus", "Read WRIST_TILT_IA");
  helpItem("selftest", "Run accelerometer and gyroscope self-test");
  helpItem("stress [N]", "Async converted-sample stress test");
  helpItem("stress_mix [N]", "Blocking mixed-operation stress test");

  helpSection("Registers");
  helpItem("rreg <addr>", "Read register byte (reserved 0x43-0x48 blocked)");
  helpItem("wreg <addr> <val>", "Write register byte and refresh cache when needed");
  helpItem("dump [addr len]", "Hex dump register range outside 0x43-0x48");
}

void processCommand(const String& line) {
  String tokens[8];
  const size_t count = splitTokens(line, tokens, 8);
  if (count == 0) {
    return;
  }

  String cmd = tokens[0];
  cmd.toLowerCase();

  if (cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "version" || cmd == "ver") {
    printVersionInfo();
  } else if (cmd == "scan") {
    bus_diag::scan();
  } else if (cmd == "drv") {
    printDriverHealth();
  } else if (cmd == "drv1") {
    printHealthView(device);
  } else if (cmd == "cfg" || cmd == "settings") {
    printSettings();
  } else if (cmd == "refresh") {
    printStatus(device.refreshCachedConfig());
  } else if (cmd == "verbose") {
    bool value = !verboseMode;
    if (count >= 2 && !parseBoolToken(tokens[1], value)) {
      Serial.println("  Expected 0 or 1");
      return;
    }
    verboseMode = value;
    Serial.printf("  Verbose: %s\n", verboseMode ? "ON" : "OFF");
  } else if (cmd == "read") {
    LSM6DS3TR::Measurement m;
    const LSM6DS3TR::Status st = performReadBlocking(m);
    if (!st.ok()) { printStatus(st); return; }
    printMeasurement(m);
  } else if (cmd == "raw") {
    LSM6DS3TR::RawMeasurement raw;
    const LSM6DS3TR::Status st = device.readAllRaw(raw);
    if (!st.ok()) { printStatus(st); return; }
    printRawMeasurement(raw);
  } else if (cmd == "accel") {
    LSM6DS3TR::RawAxes raw;
    const LSM6DS3TR::Status st = device.readAccelRaw(raw);
    if (!st.ok()) { printStatus(st); return; }
    LSM6DS3TR::Axes accel = device.convertAccel(raw);
    device.correctAccel(accel);
    Serial.printf("Accel: x=%.3f y=%.3f z=%.3f g (raw: %d %d %d)\n", accel.x, accel.y, accel.z, raw.x, raw.y, raw.z);
  } else if (cmd == "gyro") {
    LSM6DS3TR::RawAxes raw;
    const LSM6DS3TR::Status st = device.readGyroRaw(raw);
    if (!st.ok()) { printStatus(st); return; }
    LSM6DS3TR::Axes gyro = device.convertGyro(raw);
    device.correctGyro(gyro);
    Serial.printf("Gyro: x=%.2f y=%.2f z=%.2f dps (raw: %d %d %d)\n", gyro.x, gyro.y, gyro.z, raw.x, raw.y, raw.z);
  } else if (cmd == "temp") {
    int16_t raw = 0;
    const LSM6DS3TR::Status st = device.readTemperatureRaw(raw);
    if (!st.ok()) { printStatus(st); return; }
    Serial.printf("Temp: %.2f C (raw: %d)\n", device.convertTemperature(raw), raw);
  } else if (cmd == "status") {
    uint8_t statusReg = 0;
    const LSM6DS3TR::Status st = device.readStatusReg(statusReg);
    if (!st.ok()) { printStatus(st); return; }
    Serial.printf("  STATUS_REG = 0x%02X (XLDA=%d GDA=%d TDA=%d)\n",
                  statusReg,
                  (statusReg & LSM6DS3TR::cmd::MASK_XLDA) ? 1 : 0,
                  (statusReg & LSM6DS3TR::cmd::MASK_GDA) ? 1 : 0,
                  (statusReg & LSM6DS3TR::cmd::MASK_TDA) ? 1 : 0);
  } else if (cmd == "tsread") {
    uint32_t timestamp = 0;
    const LSM6DS3TR::Status st = device.readTimestamp(timestamp);
    if (!st.ok()) { printStatus(st); return; }
    Serial.printf("  Timestamp: %lu\n", static_cast<unsigned long>(timestamp));
  } else if (cmd == "steps") {
    uint16_t counter = 0;
    uint16_t stepTimestamp = 0;
    LSM6DS3TR::Status st = device.readStepCounter(counter);
    if (!st.ok()) { printStatus(st); return; }
    st = device.readStepTimestamp(stepTimestamp);
    if (!st.ok()) { printStatus(st); return; }
    Serial.printf("  Steps: %u\n", counter);
    Serial.printf("  Step timestamp: %u\n", stepTimestamp);
  } else if (cmd == "fifo") {
    LSM6DS3TR::FifoConfig fifo;
    LSM6DS3TR::FifoStatus status;
    LSM6DS3TR::Status st = device.getFifoConfig(fifo);
    if (!st.ok()) { printStatus(st); return; }
    st = device.readFifoStatus(status);
    if (!st.ok()) { printStatus(st); return; }
    Serial.println("=== FIFO ===");
    Serial.printf("  Mode: %s\n", fifoModeToStr(fifo.mode));
    Serial.printf("  ODR:  %s\n", odrToStr(fifo.odr));
    Serial.printf("  Threshold: %u\n", fifo.threshold);
    Serial.printf("  Unread words: %u\n", status.unreadWords);
    Serial.printf("  Pattern: %u\n", status.pattern);
    Serial.printf("  Empty: %s\n", log_bool_str(status.empty));
  } else if (cmd == "fifo_read") {
    int countToRead = 1;
    if (count >= 2) {
      long parsed = 0;
      if (!parseSignedToken(tokens[1], 1, 1024, parsed)) {
        Serial.println("  Expected fifo_read [1..1024]");
        return;
      }
      countToRead = static_cast<int>(parsed);
    }
    for (int i = 0; i < countToRead; ++i) {
      uint16_t word = 0;
      const LSM6DS3TR::Status st = device.readFifoWord(word);
      if (!st.ok()) {
        printStatus(st);
        break;
      }
      Serial.printf("  FIFO[%d] = 0x%04X\n", i, word);
    }
  } else if (cmd == "probe") {
    printStatus(device.probe());
  } else if (cmd == "recover") {
    printStatus(device.recover());
    printDriverHealth();
  } else if (cmd == "whoami") {
    uint8_t id = 0;
    const LSM6DS3TR::Status st = device.readWhoAmI(id);
    if (!st.ok()) { printStatus(st); return; }
    Serial.printf("  WHO_AM_I = 0x%02X\n", id);
  } else if (cmd == "wusrc" || cmd == "tapsrc" || cmd == "6dsrc" ||
             cmd == "funcsrc1" || cmd == "funcsrc2" || cmd == "wtstatus") {
    uint8_t value = 0;
    LSM6DS3TR::Status st = LSM6DS3TR::Status::Ok();
    if (cmd == "wusrc") st = device.readWakeUpSource(value);
    else if (cmd == "tapsrc") st = device.readTapSource(value);
    else if (cmd == "6dsrc") st = device.read6dSource(value);
    else if (cmd == "funcsrc1") st = device.readFunctionSource1(value);
    else if (cmd == "funcsrc2") st = device.readFunctionSource2(value);
    else st = device.readWristTiltStatus(value);
    if (!st.ok()) { printStatus(st); return; }
    printRegisterValue(cmd.c_str(), value);
  } else if (cmd == "stream") {
    continuousMode = !continuousMode;
    Serial.printf("  Continuous output: %s\n", continuousMode ? "ON (send 'stream' to stop)" : "OFF");
  } else if (cmd == "cal" || cmd == "calxl" || cmd == "calg") {
    int sampleCount = 100;
    if (count >= 2) {
      long parsed = 0;
      if (!parseSignedToken(tokens[1], 1, 10000, parsed)) {
        Serial.println("  Expected sample count [1..10000]");
        return;
      }
      sampleCount = static_cast<int>(parsed);
    }
    const bool doAccel = (cmd == "cal" || cmd == "calxl");
    const bool doGyro = (cmd == "cal" || cmd == "calg");
    Serial.printf("  Calibrating with %d samples (keep sensor still, Z-up)...\n", sampleCount);
    if (doAccel) {
      LSM6DS3TR::Axes bias;
      const LSM6DS3TR::Status st = device.captureAccelBias(static_cast<uint16_t>(sampleCount), bias);
      if (!st.ok()) { printStatus(st); return; }
      Serial.printf("  Accel bias: x=%.6f y=%.6f z=%.6f g\n", bias.x, bias.y, bias.z);
    }
    if (doGyro) {
      LSM6DS3TR::Axes bias;
      const LSM6DS3TR::Status st = device.captureGyroBias(static_cast<uint16_t>(sampleCount), bias);
      if (!st.ok()) { printStatus(st); return; }
      Serial.printf("  Gyro bias:  x=%.4f y=%.4f z=%.4f dps\n", bias.x, bias.y, bias.z);
    }
    Serial.println("  Calibration applied.");
  } else if (cmd == "biasxl" || cmd == "biasg") {
    if (count == 1) {
      // Show current bias
      const LSM6DS3TR::Axes bias = (cmd == "biasxl") ? device.accelBias() : device.gyroBias();
      const char* unit = (cmd == "biasxl") ? "g" : "dps";
      Serial.printf("  %s bias: x=%.6f y=%.6f z=%.6f %s\n",
                    (cmd == "biasxl") ? "Accel" : "Gyro", bias.x, bias.y, bias.z, unit);
    } else if (count == 4) {
      // Set bias manually
      char* endX = nullptr;
      char* endY = nullptr;
      char* endZ = nullptr;
      const float x = strtof(tokens[1].c_str(), &endX);
      const float y = strtof(tokens[2].c_str(), &endY);
      const float z = strtof(tokens[3].c_str(), &endZ);
      if (*endX != '\0' || *endY != '\0' || *endZ != '\0') {
        Serial.printf("  Expected %s <x> <y> <z>\n", cmd.c_str());
        return;
      }
      LSM6DS3TR::Axes bias;
      bias.x = x;
      bias.y = y;
      bias.z = z;
      if (cmd == "biasxl") {
        device.setAccelBias(bias);
        Serial.printf("  Accel bias set: x=%.6f y=%.6f z=%.6f g\n", x, y, z);
      } else {
        device.setGyroBias(bias);
        Serial.printf("  Gyro bias set: x=%.6f y=%.6f z=%.6f dps\n", x, y, z);
      }
    } else {
      Serial.printf("  Expected %s or %s <x> <y> <z>\n", cmd.c_str(), cmd.c_str());
    }
  } else if (cmd == "biasreset") {
    LSM6DS3TR::Axes zeroBias;
    zeroBias.x = 0.0f;
    zeroBias.y = 0.0f;
    zeroBias.z = 0.0f;
    device.setAccelBias(zeroBias);
    device.setGyroBias(zeroBias);
    Serial.println("  All software biases cleared.");
  } else if (cmd == "selftest") {
    selfTest();
  } else if (cmd == "stress") {
    cancelPending();
    int sampleCount = DEFAULT_STRESS_COUNT;
    if (count >= 2) {
      long parsed = 0;
      if (!parseSignedToken(tokens[1], 1, MAX_STRESS_COUNT, parsed)) {
        Serial.printf("  Expected stress [1..%d]\n", MAX_STRESS_COUNT);
        return;
      }
      sampleCount = static_cast<int>(parsed);
    }
    resetStressStats(sampleCount);
    stressRemaining = sampleCount;
    const LSM6DS3TR::Status st = scheduleMeasurement();
    if (!st.inProgress() && !st.ok()) {
      noteStressError(st);
      stressStats.attempts++;
      stressRemaining--;
      if (stressRemaining == 0) {
        finishStressStats();
      }
    }
  } else if (cmd == "stress_mix") {
    int sampleCount = DEFAULT_STRESS_COUNT;
    if (count >= 2) {
      long parsed = 0;
      if (!parseSignedToken(tokens[1], 1, MAX_STRESS_COUNT, parsed)) {
        Serial.printf("  Expected stress_mix [1..%d]\n", MAX_STRESS_COUNT);
        return;
      }
      sampleCount = static_cast<int>(parsed);
    }
    runStressMix(sampleCount);
  } else if (cmd == "reset") {
    printStatus(device.softReset());
  } else if (cmd == "boot") {
    printStatus(device.boot());
  } else if (cmd == "stepreset") {
    printStatus(device.resetStepCounter());
  } else if (cmd == "tsreset") {
    printStatus(device.resetTimestamp());
  } else if (cmd == "odrxl" || cmd == "odrg" || cmd == "fsxl" || cmd == "fsg" ||
             cmd == "apm" || cmd == "gpm" || cmd == "gsleep" || cmd == "alpf2" ||
             cmd == "aslope" || cmd == "a6d" || cmd == "glpf1" || cmd == "ghpf" ||
             cmd == "ghpfmode" || cmd == "ts" || cmd == "tshr" || cmd == "pedo" ||
             cmd == "sigmot" || cmd == "tilt" || cmd == "wtilt" || cmd == "ofswt" ||
             cmd == "offset" || cmd == "fifo_mode" || cmd == "fifo_odr" ||
             cmd == "fifo_xl" || cmd == "fifo_g" || cmd == "fifo_th" ||
             cmd == "fifo_temp" || cmd == "fifo_step" || cmd == "fifo_stop" ||
             cmd == "fifo_high") {
    LSM6DS3TR::AccelFilterConfig accelFilter;
    LSM6DS3TR::GyroFilterConfig gyroFilter;
    LSM6DS3TR::FifoConfig fifo;
    (void)device.getAccelFilterConfig(accelFilter);
    (void)device.getGyroFilterConfig(gyroFilter);
    (void)device.getFifoConfig(fifo);
    LSM6DS3TR::Status st = LSM6DS3TR::Status::Ok();

    if (cmd == "odrxl" || cmd == "odrg") {
      if (count == 1) {
        LSM6DS3TR::Odr odr;
        st = (cmd == "odrxl") ? device.getAccelOdr(odr) : device.getGyroOdr(odr);
        if (st.ok()) Serial.printf("  %s: %s\n", cmd.c_str(), odrToStr(odr));
      } else {
        LSM6DS3TR::Odr odr;
        if (!parseOdrToken(tokens[1], odr)) {
          Serial.println("  Invalid ODR token");
          return;
        }
        st = (cmd == "odrxl") ? device.setAccelOdr(odr) : device.setGyroOdr(odr);
      }
    } else if (cmd == "fsxl" || cmd == "fsg") {
      if (count == 1) {
        if (cmd == "fsxl") {
          LSM6DS3TR::AccelFs fs;
          st = device.getAccelFs(fs);
          if (st.ok()) Serial.printf("  Accel FS: %s\n", accelFsToStr(fs));
        } else {
          LSM6DS3TR::GyroFs fs;
          st = device.getGyroFs(fs);
          if (st.ok()) Serial.printf("  Gyro FS: %s\n", gyroFsToStr(fs));
        }
      } else if (cmd == "fsxl") {
        LSM6DS3TR::AccelFs fs;
        if (!parseAccelFsToken(tokens[1], fs)) {
          Serial.println("  Expected fsxl [2|4|8|16]");
          return;
        }
        st = device.setAccelFs(fs);
      } else {
        LSM6DS3TR::GyroFs fs;
        if (!parseGyroFsToken(tokens[1], fs)) {
          Serial.println("  Expected fsg [125|250|500|1000|2000]");
          return;
        }
        st = device.setGyroFs(fs);
      }
    } else if (cmd == "apm" || cmd == "gpm") {
      if (count == 1) {
        if (cmd == "apm") {
          LSM6DS3TR::AccelPowerMode mode;
          st = device.getAccelPowerMode(mode);
          if (st.ok()) Serial.printf("  Accel power: %s\n", accelPowerModeToStr(mode));
        } else {
          LSM6DS3TR::GyroPowerMode mode;
          st = device.getGyroPowerMode(mode);
          if (st.ok()) Serial.printf("  Gyro power: %s\n", gyroPowerModeToStr(mode));
        }
      } else if (cmd == "apm") {
        LSM6DS3TR::AccelPowerMode mode;
        if (!parseAccelPowerModeToken(tokens[1], mode)) {
          Serial.println("  Expected apm [hp|lpn]");
          return;
        }
        st = device.setAccelPowerMode(mode);
      } else {
        LSM6DS3TR::GyroPowerMode mode;
        if (!parseGyroPowerModeToken(tokens[1], mode)) {
          Serial.println("  Expected gpm [hp|lpn]");
          return;
        }
        st = device.setGyroPowerMode(mode);
      }
    } else if (cmd == "gsleep") {
      if (count == 1) {
        bool value = false;
        st = device.getGyroSleepEnabled(value);
        if (st.ok()) Serial.printf("  Gyro sleep: %s\n", log_bool_str(value));
      } else {
        bool value = false;
        if (!parseBoolToken(tokens[1], value)) { Serial.println("  Expected gsleep [0|1]"); return; }
        st = device.setGyroSleepEnabled(value);
      }
    } else if (cmd == "alpf2" || cmd == "aslope" || cmd == "a6d") {
      if (count == 1) {
        if (cmd == "alpf2") Serial.printf("  Accel LPF2: %s\n", log_bool_str(accelFilter.lpf2Enabled));
        if (cmd == "aslope") Serial.printf("  Accel HP/slope: %s\n", log_bool_str(accelFilter.highPassSlopeEnabled));
        if (cmd == "a6d") Serial.printf("  Accel low-pass on 6D: %s\n", log_bool_str(accelFilter.lowPassOn6d));
      } else {
        bool value = false;
        if (!parseBoolToken(tokens[1], value)) { Serial.println("  Expected [0|1]"); return; }
        if (cmd == "alpf2") accelFilter.lpf2Enabled = value;
        if (cmd == "aslope") accelFilter.highPassSlopeEnabled = value;
        if (cmd == "a6d") accelFilter.lowPassOn6d = value;
        st = device.setAccelFilterConfig(accelFilter);
      }
    } else if (cmd == "glpf1" || cmd == "ghpf" || cmd == "ghpfmode") {
      if (count == 1) {
        if (cmd == "glpf1") Serial.printf("  Gyro LPF1: %s\n", log_bool_str(gyroFilter.lpf1Enabled));
        if (cmd == "ghpf") Serial.printf("  Gyro HPF: %s\n", log_bool_str(gyroFilter.highPassEnabled));
        if (cmd == "ghpfmode") Serial.printf("  Gyro HPF mode: %s\n", gyroHpfModeToStr(gyroFilter.highPassMode));
      } else if (cmd == "ghpfmode") {
        LSM6DS3TR::GyroHpfMode mode;
        if (!parseGyroHpfModeToken(tokens[1], mode)) { Serial.println("  Expected ghpfmode [0|1|2|3]"); return; }
        gyroFilter.highPassMode = mode;
        st = device.setGyroFilterConfig(gyroFilter);
      } else {
        bool value = false;
        if (!parseBoolToken(tokens[1], value)) { Serial.println("  Expected [0|1]"); return; }
        if (cmd == "glpf1") gyroFilter.lpf1Enabled = value;
        if (cmd == "ghpf") gyroFilter.highPassEnabled = value;
        st = device.setGyroFilterConfig(gyroFilter);
      }
    } else if (cmd == "ts" || cmd == "tshr" || cmd == "pedo" || cmd == "sigmot" || cmd == "tilt" || cmd == "wtilt") {
      if (count == 1) {
        bool value = false;
        if (cmd == "ts") st = device.getTimestampEnabled(value);
        else if (cmd == "tshr") st = device.getTimestampHighResolution(value);
        else if (cmd == "pedo") st = device.getPedometerEnabled(value);
        else if (cmd == "sigmot") st = device.getSignificantMotionEnabled(value);
        else if (cmd == "tilt") st = device.getTiltEnabled(value);
        else st = device.getWristTiltEnabled(value);
        if (st.ok()) Serial.printf("  %s: %s\n", cmd.c_str(), log_bool_str(value));
      } else {
        bool value = false;
        if (!parseBoolToken(tokens[1], value)) { Serial.println("  Expected [0|1]"); return; }
        if (cmd == "ts") st = device.setTimestampEnabled(value);
        else if (cmd == "tshr") st = device.setTimestampHighResolution(value);
        else if (cmd == "pedo") st = device.setPedometerEnabled(value);
        else if (cmd == "sigmot") st = device.setSignificantMotionEnabled(value);
        else if (cmd == "tilt") st = device.setTiltEnabled(value);
        else st = device.setWristTiltEnabled(value);
      }
    } else if (cmd == "ofswt") {
      if (count == 1) {
        AccelOffsetWeight weight;
        st = device.getAccelOffsetWeight(weight);
        if (st.ok()) Serial.printf("  Offset weight: %s\n", offsetWeightToStr(weight));
      } else {
        AccelOffsetWeight weight;
        if (!parseOffsetWeightToken(tokens[1], weight)) { Serial.println("  Expected ofswt [1|16]"); return; }
        st = device.setAccelOffsetWeight(weight);
      }
    } else if (cmd == "offset") {
      if (count == 1) {
        LSM6DS3TR::AccelUserOffset offset;
        st = device.getAccelUserOffset(offset);
        if (st.ok()) Serial.printf("  Offset: x=%d y=%d z=%d\n", offset.x, offset.y, offset.z);
      } else if (count == 4) {
        long x = 0, y = 0, z = 0;
        if (!parseSignedToken(tokens[1], -128, 127, x) ||
            !parseSignedToken(tokens[2], -128, 127, y) ||
            !parseSignedToken(tokens[3], -128, 127, z)) {
          Serial.println("  Expected offset <x> <y> <z>");
          return;
        }
        LSM6DS3TR::AccelUserOffset offset;
        offset.x = static_cast<int8_t>(x);
        offset.y = static_cast<int8_t>(y);
        offset.z = static_cast<int8_t>(z);
        st = device.setAccelUserOffset(offset);
      } else {
        Serial.println("  Expected offset or offset <x> <y> <z>");
        return;
      }
    } else if (cmd == "fifo_mode" || cmd == "fifo_odr" || cmd == "fifo_xl" || cmd == "fifo_g" ||
               cmd == "fifo_th" || cmd == "fifo_temp" || cmd == "fifo_step" || cmd == "fifo_stop" || cmd == "fifo_high") {
      if (cmd == "fifo_mode") {
        if (count == 1) Serial.printf("  FIFO mode: %s\n", fifoModeToStr(fifo.mode));
        else {
          LSM6DS3TR::FifoMode mode;
          if (!parseFifoModeToken(tokens[1], mode)) { Serial.println("  Invalid FIFO mode"); return; }
          fifo.mode = mode;
          st = device.configureFifo(fifo);
        }
      } else if (cmd == "fifo_odr") {
        if (count == 1) Serial.printf("  FIFO ODR: %s\n", odrToStr(fifo.odr));
        else {
          LSM6DS3TR::Odr odr;
          if (!parseOdrToken(tokens[1], odr)) { Serial.println("  Invalid FIFO ODR"); return; }
          fifo.odr = odr;
          st = device.configureFifo(fifo);
        }
      } else if (cmd == "fifo_xl" || cmd == "fifo_g") {
        if (count == 1) {
          Serial.printf("  %s: %s\n", cmd.c_str(), fifoDecimationToStr(cmd == "fifo_xl" ? fifo.accelDecimation : fifo.gyroDecimation));
        } else {
          LSM6DS3TR::FifoDecimation decimation;
          if (!parseFifoDecimationToken(tokens[1], decimation)) { Serial.println("  Invalid FIFO decimation"); return; }
          if (cmd == "fifo_xl") fifo.accelDecimation = decimation;
          else fifo.gyroDecimation = decimation;
          st = device.configureFifo(fifo);
        }
      } else if (cmd == "fifo_th") {
        if (count == 1) Serial.printf("  FIFO threshold: %u\n", fifo.threshold);
        else {
          unsigned long value = 0;
          if (!parseUnsignedToken(tokens[1], 2047UL, value)) { Serial.println("  Expected fifo_th [0..2047]"); return; }
          fifo.threshold = static_cast<uint16_t>(value);
          st = device.configureFifo(fifo);
        }
      } else {
        bool value = false;
        if (count == 1) {
          if (cmd == "fifo_temp") Serial.printf("  FIFO temperature: %s\n", log_bool_str(fifo.storeTemperature));
          if (cmd == "fifo_step") Serial.printf("  FIFO step/timestamp: %s\n", log_bool_str(fifo.storeTimestampStep));
          if (cmd == "fifo_stop") Serial.printf("  FIFO stop on threshold: %s\n", log_bool_str(fifo.stopOnThreshold));
          if (cmd == "fifo_high") Serial.printf("  FIFO high-only: %s\n", log_bool_str(fifo.onlyHighData));
        } else {
          if (!parseBoolToken(tokens[1], value)) { Serial.println("  Expected [0|1]"); return; }
          if (cmd == "fifo_temp") fifo.storeTemperature = value;
          if (cmd == "fifo_step") fifo.storeTimestampStep = value;
          if (cmd == "fifo_stop") fifo.stopOnThreshold = value;
          if (cmd == "fifo_high") fifo.onlyHighData = value;
          st = device.configureFifo(fifo);
        }
      }
    }
    if (!st.ok()) {
      printStatus(st);
    }
  } else if (cmd == "rreg" || cmd == "wreg" || cmd == "dump") {
    if (cmd == "rreg") {
      if (count != 2) { Serial.println("  Expected rreg <addr>"); return; }
      unsigned long reg = 0;
      if (!parseUnsignedToken(tokens[1], 0xFFUL, reg)) { Serial.println("  Invalid register"); return; }
      if (isReservedRegister(static_cast<uint8_t>(reg))) { Serial.println("  Reserved register range 0x43-0x48 is blocked"); return; }
      uint8_t value = 0;
      const LSM6DS3TR::Status st = device.readRegisterValue(static_cast<uint8_t>(reg), value);
      if (!st.ok()) { printStatus(st); return; }
      Serial.printf("  [0x%02lX] = 0x%02X\n", reg, value);
    } else if (cmd == "wreg") {
      if (count != 3) { Serial.println("  Expected wreg <addr> <val>"); return; }
      unsigned long reg = 0;
      unsigned long value = 0;
      if (!parseUnsignedToken(tokens[1], 0xFFUL, reg) || !parseUnsignedToken(tokens[2], 0xFFUL, value)) {
        Serial.println("  Invalid register or value");
        return;
      }
      if (isReservedRegister(static_cast<uint8_t>(reg))) { Serial.println("  Reserved register range 0x43-0x48 is blocked"); return; }
      printStatus(device.writeRegisterValue(static_cast<uint8_t>(reg), static_cast<uint8_t>(value)));
    } else {
      unsigned long start = DEFAULT_DUMP_START;
      unsigned long len = DEFAULT_DUMP_LEN;
      if (count >= 2 && !parseUnsignedToken(tokens[1], 0xFFUL, start)) { Serial.println("  Invalid start register"); return; }
      if (count >= 3 && !parseUnsignedToken(tokens[2], MAX_DUMP_LEN, len)) { Serial.printf("  Invalid dump length (max %d)\n", MAX_DUMP_LEN); return; }
      if (len == 0UL || (start + len - 1UL) > 0xFFUL) { Serial.println("  Invalid dump range"); return; }
      for (unsigned long reg = start; reg < (start + len); ++reg) {
        if (isReservedRegister(static_cast<uint8_t>(reg))) { Serial.println("  Dump range intersects reserved 0x43-0x48"); return; }
      }
      uint8_t buffer[MAX_DUMP_LEN] = {};
      const LSM6DS3TR::Status st = device.readRegisterBlock(static_cast<uint8_t>(start), buffer, static_cast<size_t>(len));
      if (!st.ok()) { printStatus(st); return; }
      printHexDump(static_cast<uint8_t>(start), buffer, static_cast<size_t>(len));
    }
  } else {
    Serial.printf("  Unknown command: '%s' (type 'help')\n", cmd.c_str());
  }
}

}  // namespace

void setup() {
  log_begin(115200);
  delay(100);

  LOGI("=== LSM6DS3TR-C Bringup CLI ===");
  printVersionInfo();

  board::initI2c();
  LOGI("I2C initialized (SDA=%d, SCL=%d)", board::I2C_SDA, board::I2C_SCL);
  bus_diag::scan();

  LSM6DS3TR::Config cfg;
  cfg.i2cWrite = transport_adapter::wireWrite;
  cfg.i2cWriteRead = transport_adapter::wireWriteRead;
  cfg.i2cUser = &Wire;
  cfg.i2cAddress = 0x6A;
  cfg.i2cTimeoutMs = board::I2C_TIMEOUT_MS;
  cfg.nowMs = exampleNowMs;
  cfg.offlineThreshold = 5;
  cfg.odrXl = LSM6DS3TR::Odr::HZ_104;
  cfg.odrG = LSM6DS3TR::Odr::HZ_104;
  cfg.fsXl = LSM6DS3TR::AccelFs::G_2;
  cfg.fsG = LSM6DS3TR::GyroFs::DPS_250;
  cfg.bdu = true;

  const LSM6DS3TR::Status st = device.begin(cfg);
  if (!st.ok()) {
    LOGE("Device initialization failed; CLI remains available for probe/recover");
    printStatus(st);
  } else {
    LOGI("Device initialized successfully");
    printDriverHealth();
  }

  Serial.println();
  Serial.println("Type 'help' for commands");
  Serial.print("> ");
}

void loop() {
  device.tick(millis());

  if (stressStats.active && stressRemaining > 0 && !pendingRead) {
    const LSM6DS3TR::Status st = scheduleMeasurement();
    if (!st.inProgress() && !st.ok()) {
      noteStressError(st);
      stressStats.attempts++;
      stressRemaining--;
      if (stressRemaining == 0) {
        finishStressStats();
      }
    }
  }

  handleMeasurementReady();

  if (continuousMode) {
    uint8_t statusReg = 0;
    if (device.readStatusReg(statusReg).ok() &&
        (statusReg & LSM6DS3TR::cmd::MASK_XLDA)) {
      LSM6DS3TR::Measurement m;
      const LSM6DS3TR::Status st = performReadBlocking(m);
      if (st.ok()) {
        printMeasurement(m);
      }
    }
  }

  String line;
  if (cli_shell::readLine(line)) {
    processCommand(line);
    Serial.print("> ");
  }
}
