/// @file main.cpp
/// @brief Native ESP-IDF bring-up CLI for LSM6DS3TR-C.

#include <cctype>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

#include "LSM6DS3TR/LSM6DS3TR.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr gpio_num_t I2C_SDA = GPIO_NUM_8;
constexpr gpio_num_t I2C_SCL = GPIO_NUM_9;
constexpr uint8_t I2C_ADDRESS = 0x6A;
constexpr uint32_t I2C_FREQ_HZ = 400000;
constexpr uint32_t I2C_TIMEOUT_MS = 50;
constexpr size_t INPUT_MAX = 160;

struct I2cContext {
  i2c_master_bus_handle_t bus = nullptr;
  i2c_master_dev_handle_t dev = nullptr;
  uint8_t address = I2C_ADDRESS;
};

LSM6DS3TR::LSM6DS3TR device;
I2cContext i2c;
bool verboseMode = false;
bool streamMode = false;

uint32_t nowMs(void*) {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000LL);
}

int timeoutArg(uint32_t timeoutMs) {
  const uint32_t maxInt = static_cast<uint32_t>(std::numeric_limits<int>::max());
  return static_cast<int>(timeoutMs > maxInt ? maxInt : timeoutMs);
}

LSM6DS3TR::Status mapEsp(esp_err_t err, const char* msg) {
  if (err == ESP_OK) return LSM6DS3TR::Status::Ok();
  if (err == ESP_ERR_TIMEOUT) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_TIMEOUT, "I2C timeout",
                                    static_cast<int32_t>(err));
  }
  if (err == ESP_ERR_INVALID_RESPONSE) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_NACK_ADDR, "I2C NACK",
                                    static_cast<int32_t>(err));
  }
  return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_BUS, msg, static_cast<int32_t>(err));
}

LSM6DS3TR::Status i2cWrite(uint8_t addr, const uint8_t* data, size_t len,
                           uint32_t timeoutMs, void* user) {
  auto* ctx = static_cast<I2cContext*>(user);
  if (ctx == nullptr || ctx->dev == nullptr || addr != ctx->address) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::INVALID_CONFIG, "I2C context invalid");
  }
  if (data == nullptr || len == 0) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::INVALID_PARAM, "I2C write buffer invalid");
  }
  return mapEsp(i2c_master_transmit(ctx->dev, data, len, timeoutArg(timeoutMs)),
                "I2C write failed");
}

LSM6DS3TR::Status i2cWriteRead(uint8_t addr, const uint8_t* txData, size_t txLen,
                               uint8_t* rxData, size_t rxLen, uint32_t timeoutMs,
                               void* user) {
  auto* ctx = static_cast<I2cContext*>(user);
  if (ctx == nullptr || ctx->dev == nullptr || addr != ctx->address) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::INVALID_CONFIG, "I2C context invalid");
  }
  if (txData == nullptr || txLen == 0 || rxData == nullptr || rxLen == 0) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::INVALID_PARAM, "I2C read buffer invalid");
  }
  return mapEsp(i2c_master_transmit_receive(ctx->dev, txData, txLen, rxData, rxLen,
                                            timeoutArg(timeoutMs)),
                "I2C write-read failed");
}

LSM6DS3TR::Config makeConfig() {
  LSM6DS3TR::Config cfg{};
  cfg.i2cWrite = i2cWrite;
  cfg.i2cWriteRead = i2cWriteRead;
  cfg.i2cUser = &i2c;
  cfg.nowMs = nowMs;
  cfg.i2cAddress = I2C_ADDRESS;
  cfg.i2cTimeoutMs = I2C_TIMEOUT_MS;
  cfg.offlineThreshold = 5;
  return cfg;
}

const char* errToStr(LSM6DS3TR::Err err) {
  switch (err) {
    case LSM6DS3TR::Err::OK: return "OK";
    case LSM6DS3TR::Err::NOT_INITIALIZED: return "NOT_INITIALIZED";
    case LSM6DS3TR::Err::INVALID_CONFIG: return "INVALID_CONFIG";
    case LSM6DS3TR::Err::I2C_ERROR: return "I2C_ERROR";
    case LSM6DS3TR::Err::TIMEOUT: return "TIMEOUT";
    case LSM6DS3TR::Err::INVALID_PARAM: return "INVALID_PARAM";
    case LSM6DS3TR::Err::DEVICE_NOT_FOUND: return "DEVICE_NOT_FOUND";
    case LSM6DS3TR::Err::CHIP_ID_MISMATCH: return "CHIP_ID_MISMATCH";
    case LSM6DS3TR::Err::MEASUREMENT_NOT_READY: return "MEASUREMENT_NOT_READY";
    case LSM6DS3TR::Err::BUSY: return "BUSY";
    case LSM6DS3TR::Err::IN_PROGRESS: return "IN_PROGRESS";
    case LSM6DS3TR::Err::SELF_TEST_FAIL: return "SELF_TEST_FAIL";
    case LSM6DS3TR::Err::I2C_NACK_ADDR: return "I2C_NACK_ADDR";
    case LSM6DS3TR::Err::I2C_NACK_DATA: return "I2C_NACK_DATA";
    case LSM6DS3TR::Err::I2C_TIMEOUT: return "I2C_TIMEOUT";
    case LSM6DS3TR::Err::I2C_BUS: return "I2C_BUS";
    case LSM6DS3TR::Err::FIFO_EMPTY: return "FIFO_EMPTY";
    default: return "UNKNOWN";
  }
}

const char* stateToStr(LSM6DS3TR::DriverState state) {
  switch (state) {
    case LSM6DS3TR::DriverState::UNINIT: return "UNINIT";
    case LSM6DS3TR::DriverState::READY: return "READY";
    case LSM6DS3TR::DriverState::DEGRADED: return "DEGRADED";
    case LSM6DS3TR::DriverState::OFFLINE: return "OFFLINE";
    default: return "UNKNOWN";
  }
}

void printStatus(LSM6DS3TR::Status st) {
  printf("  Status: %s (code=%u, detail=%ld)\n", errToStr(st.code),
         static_cast<unsigned>(st.code), static_cast<long>(st.detail));
  if (st.msg != nullptr && st.msg[0] != '\0') {
    printf("  Message: %s\n", st.msg);
  }
}

char* nextToken(char** save) {
  return strtok_r(nullptr, " \t", save);
}

bool parseU32(const char* text, uint32_t& out) {
  if (text == nullptr || text[0] == '\0') return false;
  char* end = nullptr;
  const unsigned long value = strtoul(text, &end, 0);
  if (end == text || *end != '\0') return false;
  out = static_cast<uint32_t>(value);
  return true;
}

bool parseI32(const char* text, int32_t& out) {
  if (text == nullptr || text[0] == '\0') return false;
  char* end = nullptr;
  const long value = strtol(text, &end, 0);
  if (end == text || *end != '\0') return false;
  out = static_cast<int32_t>(value);
  return true;
}

bool parseBool(const char* text, bool& out) {
  if (text == nullptr) return false;
  if (strcmp(text, "1") == 0 || strcmp(text, "on") == 0 || strcmp(text, "true") == 0) {
    out = true;
    return true;
  }
  if (strcmp(text, "0") == 0 || strcmp(text, "off") == 0 || strcmp(text, "false") == 0) {
    out = false;
    return true;
  }
  return false;
}

void lowerInPlace(char* text) {
  for (; text != nullptr && *text != '\0'; ++text) {
    *text = static_cast<char>(tolower(static_cast<unsigned char>(*text)));
  }
}

void printHelp() {
  puts("\n=== LSM6DS3TR native ESP-IDF CLI ===");
  puts("Common: help ? version ver scan begin init drv drv1 cfg settings refresh verbose <0|1>");
  puts("Data: read raw accel gyro temp status tsread steps fifo fifo_read <n> stream");
  puts("Config: odrxl <n> odrg <n> fsxl <2|4|8|16> fsg <125|250|500|1000|2000>");
  puts("Power/filter: apm <hp|lpn> gpm <hp|lpn> gsleep <0|1> alpf2 <0|1> aslope <0|1>");
  puts("Power/filter: a6d <0|1> glpf1 <0|1> ghpf <0|1> ghpfmode <0|1|2|3>");
  puts("Features: ts <0|1> tshr <0|1> tsreset pedo <0|1> sigmot <0|1> tilt <0|1>");
  puts("Features: wtilt <0|1> stepreset ofswt <1|16> offset [x y z]");
  puts("FIFO: fifo_mode <0|1|3|4|6> fifo_odr <n> fifo_xl <0..7> fifo_g <0..7>");
  puts("FIFO: fifo_th <0..2047> fifo_temp <0|1> fifo_step <0|1> fifo_stop <0|1> fifo_high <0|1>");
  puts("Diagnostics: cal calxl [n] calg [n] biasxl [x y z] biasg [x y z] biasreset");
  puts("Diagnostics: probe recover whoami id wusrc tapsrc 6dsrc funcsrc1 funcsrc2 wtstatus shub [n]");
  puts("Registers: rreg <addr> wreg <addr> <val> dump [start] [len] reset boot selftest stress [n] stress_mix [n]");
}

void printHealth() {
  const uint32_t ok = device.totalSuccess();
  const uint32_t fail = device.totalFailures();
  printf("Driver: state=%s online=%s consec=%u ok=%lu fail=%lu lastOk=%lu lastErr=%lu\n",
         stateToStr(device.state()), device.isOnline() ? "yes" : "no",
         static_cast<unsigned>(device.consecutiveFailures()),
         static_cast<unsigned long>(ok), static_cast<unsigned long>(fail),
         static_cast<unsigned long>(device.lastOkMs()),
         static_cast<unsigned long>(device.lastErrorMs()));
  if (!device.lastError().ok()) printStatus(device.lastError());
}

void printMeasurement() {
  LSM6DS3TR::RawMeasurement raw;
  LSM6DS3TR::Status st = device.readAllRaw(raw);
  if (!st.ok()) {
    printStatus(st);
    return;
  }
  LSM6DS3TR::Axes accel = device.convertAccel(raw.accel);
  LSM6DS3TR::Axes gyro = device.convertGyro(raw.gyro);
  device.correctAccel(accel);
  device.correctGyro(gyro);
  printf("Raw: ax=%d ay=%d az=%d gx=%d gy=%d gz=%d t=%d\n",
         raw.accel.x, raw.accel.y, raw.accel.z, raw.gyro.x, raw.gyro.y,
         raw.gyro.z, raw.temperature);
  printf("Sample: ax=%+.3f ay=%+.3f az=%+.3f g | gx=%+.2f gy=%+.2f gz=%+.2f dps | t=%.2f C\n",
         accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z,
         device.convertTemperature(raw.temperature));
}

void scanI2c() {
  puts("I2C scan:");
  for (uint8_t addr = 0x03; addr <= 0x77; ++addr) {
    const esp_err_t err = i2c_master_probe(i2c.bus, addr, timeoutArg(I2C_TIMEOUT_MS));
    if (err == ESP_OK) printf("  0x%02X ACK\n", addr);
  }
}

void printSettings() {
  LSM6DS3TR::SettingsSnapshot s = device.settings();
  printf("Settings: init=%s state=%s addr=0x%02X timeout=%lu threshold=%u pending=%s ready=%s hasSample=%s age=%lu\n",
         s.initialized ? "yes" : "no", stateToStr(s.state), s.i2cAddress,
         static_cast<unsigned long>(s.i2cTimeoutMs), static_cast<unsigned>(s.offlineThreshold),
         s.measurementPending ? "yes" : "no", s.measurementReady ? "yes" : "no",
         s.hasSample ? "yes" : "no", static_cast<unsigned long>(device.sampleAgeMs(nowMs(nullptr))));
}

LSM6DS3TR::Odr parseOdrValue(uint32_t value) {
  if (value == 0) return LSM6DS3TR::Odr::POWER_DOWN;
  if (value == 12) return LSM6DS3TR::Odr::HZ_12_5;
  if (value == 26) return LSM6DS3TR::Odr::HZ_26;
  if (value == 52) return LSM6DS3TR::Odr::HZ_52;
  if (value == 104) return LSM6DS3TR::Odr::HZ_104;
  if (value == 208) return LSM6DS3TR::Odr::HZ_208;
  if (value == 416) return LSM6DS3TR::Odr::HZ_416;
  if (value == 833) return LSM6DS3TR::Odr::HZ_833;
  if (value == 1660) return LSM6DS3TR::Odr::HZ_1660;
  if (value == 3330) return LSM6DS3TR::Odr::HZ_3330;
  if (value == 6660) return LSM6DS3TR::Odr::HZ_6660;
  if (value == 1) return LSM6DS3TR::Odr::HZ_1_6;
  return static_cast<LSM6DS3TR::Odr>(value);
}

void updateFifoField(const char* field, char* arg) {
  LSM6DS3TR::FifoConfig cfg;
  printStatus(device.getFifoConfig(cfg));
  uint32_t value = 0;
  if (!parseU32(arg, value)) {
    puts("  Expected numeric FIFO value");
    return;
  }
  if (strcmp(field, "fifo_mode") == 0) cfg.mode = static_cast<LSM6DS3TR::FifoMode>(value);
  if (strcmp(field, "fifo_odr") == 0) cfg.odr = parseOdrValue(value);
  if (strcmp(field, "fifo_xl") == 0) cfg.accelDecimation = static_cast<LSM6DS3TR::FifoDecimation>(value);
  if (strcmp(field, "fifo_g") == 0) cfg.gyroDecimation = static_cast<LSM6DS3TR::FifoDecimation>(value);
  if (strcmp(field, "fifo_th") == 0) cfg.threshold = static_cast<uint16_t>(value);
  if (strcmp(field, "fifo_temp") == 0) cfg.storeTemperature = value != 0;
  if (strcmp(field, "fifo_step") == 0) cfg.storeTimestampStep = value != 0;
  if (strcmp(field, "fifo_stop") == 0) cfg.stopOnThreshold = value != 0;
  if (strcmp(field, "fifo_high") == 0) cfg.onlyHighData = value != 0;
  printStatus(device.configureFifo(cfg));
}

void runStress(uint32_t count) {
  uint32_t ok = 0;
  uint32_t fail = 0;
  for (uint32_t i = 0; i < count; ++i) {
    LSM6DS3TR::RawMeasurement raw;
    const LSM6DS3TR::Status st = device.readAllRaw(raw);
    st.ok() ? ++ok : ++fail;
    if (!st.ok() && verboseMode) printStatus(st);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  printf("Stress: ok=%lu fail=%lu\n", static_cast<unsigned long>(ok),
         static_cast<unsigned long>(fail));
  printHealth();
}

void processCommand(char* line) {
  char* save = nullptr;
  char* cmd = strtok_r(line, " \t", &save);
  if (cmd == nullptr) return;
  lowerInPlace(cmd);

  if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
    printHelp();
  } else if (strcmp(cmd, "version") == 0 || strcmp(cmd, "ver") == 0) {
    printf("Version: %s\n", LSM6DS3TR::VERSION_FULL);
  } else if (strcmp(cmd, "scan") == 0) {
    scanI2c();
  } else if (strcmp(cmd, "begin") == 0 || strcmp(cmd, "init") == 0) {
    device.end();
    printStatus(device.begin(makeConfig()));
    printHealth();
  } else if (strcmp(cmd, "drv") == 0 || strcmp(cmd, "drv1") == 0) {
    printHealth();
  } else if (strcmp(cmd, "cfg") == 0 || strcmp(cmd, "settings") == 0) {
    printSettings();
  } else if (strcmp(cmd, "refresh") == 0) {
    printStatus(device.refreshCachedConfig());
  } else if (strcmp(cmd, "verbose") == 0) {
    bool value = false;
    if (parseBool(nextToken(&save), value)) verboseMode = value;
    printf("Verbose: %s\n", verboseMode ? "ON" : "OFF");
  } else if (strcmp(cmd, "read") == 0 || strcmp(cmd, "raw") == 0 ||
             strcmp(cmd, "accel") == 0 || strcmp(cmd, "gyro") == 0 ||
             strcmp(cmd, "temp") == 0) {
    printMeasurement();
  } else if (strcmp(cmd, "status") == 0) {
    LSM6DS3TR::StatusReg out;
    const auto st = device.readStatus(out);
    if (st.ok()) printf("STATUS=0x%02X XLDA=%d GDA=%d TDA=%d\n", out.raw,
                        out.accelDataReady, out.gyroDataReady, out.tempDataReady);
    else printStatus(st);
  } else if (strcmp(cmd, "tsread") == 0) {
    uint32_t ts = 0;
    const auto st = device.readTimestamp(ts);
    st.ok() ? printf("Timestamp: %lu\n", static_cast<unsigned long>(ts)) : printStatus(st);
  } else if (strcmp(cmd, "steps") == 0) {
    uint16_t steps = 0;
    const auto st = device.readStepCounter(steps);
    st.ok() ? printf("Steps: %u\n", steps) : printStatus(st);
  } else if (strcmp(cmd, "fifo") == 0) {
    LSM6DS3TR::FifoStatus fs;
    const auto st = device.readFifoStatus(fs);
    if (st.ok()) printf("FIFO: words=%u pattern=%u wm=%d overrun=%d full=%d empty=%d\n",
                        fs.unreadWords, fs.pattern, fs.watermark, fs.overrun,
                        fs.fullSmart, fs.empty);
    else printStatus(st);
  } else if (strcmp(cmd, "fifo_read") == 0) {
    uint32_t count = 1;
    parseU32(nextToken(&save), count);
    for (uint32_t i = 0; i < count; ++i) {
      uint16_t word = 0;
      const auto st = device.readFifoWord(word);
      if (!st.ok()) { printStatus(st); break; }
      printf("FIFO[%lu]=0x%04X\n", static_cast<unsigned long>(i), word);
    }
  } else if (strcmp(cmd, "stream") == 0) {
    streamMode = !streamMode;
    printf("Stream: %s\n", streamMode ? "ON" : "OFF");
  } else if (strcmp(cmd, "odrxl") == 0 || strcmp(cmd, "odrg") == 0) {
    uint32_t value = 0;
    if (!parseU32(nextToken(&save), value)) { puts("  Expected ODR value"); return; }
    printStatus(strcmp(cmd, "odrxl") == 0 ? device.setAccelOdr(parseOdrValue(value))
                                          : device.setGyroOdr(parseOdrValue(value)));
  } else if (strcmp(cmd, "fsxl") == 0 || strcmp(cmd, "fsg") == 0) {
    uint32_t value = 0;
    if (!parseU32(nextToken(&save), value)) { puts("  Expected FS value"); return; }
    if (strcmp(cmd, "fsxl") == 0) {
      LSM6DS3TR::AccelFs fs = value == 16 ? LSM6DS3TR::AccelFs::G_16 :
                              value == 8 ? LSM6DS3TR::AccelFs::G_8 :
                              value == 4 ? LSM6DS3TR::AccelFs::G_4 : LSM6DS3TR::AccelFs::G_2;
      printStatus(device.setAccelFs(fs));
    } else {
      LSM6DS3TR::GyroFs fs = value == 125 ? LSM6DS3TR::GyroFs::DPS_125 :
                             value == 500 ? LSM6DS3TR::GyroFs::DPS_500 :
                             value == 1000 ? LSM6DS3TR::GyroFs::DPS_1000 :
                             value == 2000 ? LSM6DS3TR::GyroFs::DPS_2000 : LSM6DS3TR::GyroFs::DPS_250;
      printStatus(device.setGyroFs(fs));
    }
  } else if (strcmp(cmd, "apm") == 0 || strcmp(cmd, "gpm") == 0) {
    char* arg = nextToken(&save);
    const bool low = arg != nullptr && strcmp(arg, "lpn") == 0;
    printStatus(strcmp(cmd, "apm") == 0
                    ? device.setAccelPowerMode(low ? LSM6DS3TR::AccelPowerMode::LOW_POWER_NORMAL
                                                   : LSM6DS3TR::AccelPowerMode::HIGH_PERFORMANCE)
                    : device.setGyroPowerMode(low ? LSM6DS3TR::GyroPowerMode::LOW_POWER_NORMAL
                                                  : LSM6DS3TR::GyroPowerMode::HIGH_PERFORMANCE));
  } else if (strcmp(cmd, "gsleep") == 0) {
    bool value = false; if (parseBool(nextToken(&save), value)) printStatus(device.setGyroSleepEnabled(value));
  } else if (strcmp(cmd, "reset") == 0) {
    printStatus(device.softReset());
  } else if (strcmp(cmd, "boot") == 0) {
    printStatus(device.boot());
  } else if (strcmp(cmd, "alpf2") == 0 || strcmp(cmd, "aslope") == 0 || strcmp(cmd, "a6d") == 0) {
    bool value = false; if (!parseBool(nextToken(&save), value)) return;
    LSM6DS3TR::AccelFilterConfig cfg; device.getAccelFilterConfig(cfg);
    if (strcmp(cmd, "alpf2") == 0) cfg.lpf2Enabled = value;
    if (strcmp(cmd, "aslope") == 0) cfg.highPassSlopeEnabled = value;
    if (strcmp(cmd, "a6d") == 0) cfg.lowPassOn6d = value;
    printStatus(device.setAccelFilterConfig(cfg));
  } else if (strcmp(cmd, "glpf1") == 0 || strcmp(cmd, "ghpf") == 0 || strcmp(cmd, "ghpfmode") == 0) {
    uint32_t value = 0; if (!parseU32(nextToken(&save), value)) return;
    LSM6DS3TR::GyroFilterConfig cfg; device.getGyroFilterConfig(cfg);
    if (strcmp(cmd, "glpf1") == 0) cfg.lpf1Enabled = value != 0;
    if (strcmp(cmd, "ghpf") == 0) cfg.highPassEnabled = value != 0;
    if (strcmp(cmd, "ghpfmode") == 0) cfg.highPassMode = static_cast<LSM6DS3TR::GyroHpfMode>(value);
    printStatus(device.setGyroFilterConfig(cfg));
  } else if (strcmp(cmd, "ts") == 0 || strcmp(cmd, "tshr") == 0 || strcmp(cmd, "pedo") == 0 ||
             strcmp(cmd, "sigmot") == 0 || strcmp(cmd, "tilt") == 0 || strcmp(cmd, "wtilt") == 0) {
    bool value = false; if (!parseBool(nextToken(&save), value)) return;
    if (strcmp(cmd, "ts") == 0) printStatus(device.setTimestampEnabled(value));
    if (strcmp(cmd, "tshr") == 0) printStatus(device.setTimestampHighResolution(value));
    if (strcmp(cmd, "pedo") == 0) printStatus(device.setPedometerEnabled(value));
    if (strcmp(cmd, "sigmot") == 0) printStatus(device.setSignificantMotionEnabled(value));
    if (strcmp(cmd, "tilt") == 0) printStatus(device.setTiltEnabled(value));
    if (strcmp(cmd, "wtilt") == 0) printStatus(device.setWristTiltEnabled(value));
  } else if (strcmp(cmd, "tsreset") == 0) {
    printStatus(device.resetTimestamp());
  } else if (strcmp(cmd, "stepreset") == 0) {
    printStatus(device.resetStepCounter());
  } else if (strcmp(cmd, "ofswt") == 0) {
    uint32_t value = 1; parseU32(nextToken(&save), value);
    printStatus(device.setAccelOffsetWeight(value == 16 ? LSM6DS3TR::AccelOffsetWeight::MG_16
                                                        : LSM6DS3TR::AccelOffsetWeight::MG_1));
  } else if (strcmp(cmd, "offset") == 0) {
    int32_t x = 0, y = 0, z = 0;
    if (parseI32(nextToken(&save), x) && parseI32(nextToken(&save), y) && parseI32(nextToken(&save), z)) {
      printStatus(device.setAccelUserOffset({static_cast<int8_t>(x), static_cast<int8_t>(y), static_cast<int8_t>(z)}));
    } else {
      LSM6DS3TR::AccelUserOffset offset; device.getAccelUserOffset(offset);
      printf("Offset: x=%d y=%d z=%d\n", offset.x, offset.y, offset.z);
    }
  } else if (strncmp(cmd, "fifo_", 5) == 0) {
    updateFifoField(cmd, nextToken(&save));
  } else if (strcmp(cmd, "cal") == 0 || strcmp(cmd, "calxl") == 0) {
    uint32_t count = 32; parseU32(nextToken(&save), count);
    LSM6DS3TR::Axes bias; printStatus(device.captureAccelBias(static_cast<uint16_t>(count), bias));
    printf("Accel bias: %.6f %.6f %.6f\n", bias.x, bias.y, bias.z);
  } else if (strcmp(cmd, "calg") == 0) {
    uint32_t count = 32; parseU32(nextToken(&save), count);
    LSM6DS3TR::Axes bias; printStatus(device.captureGyroBias(static_cast<uint16_t>(count), bias));
    printf("Gyro bias: %.6f %.6f %.6f\n", bias.x, bias.y, bias.z);
  } else if (strcmp(cmd, "biasxl") == 0 || strcmp(cmd, "biasg") == 0) {
    char* a = nextToken(&save);
    if (a == nullptr) {
      const auto bias = strcmp(cmd, "biasxl") == 0 ? device.accelBias() : device.gyroBias();
      printf("Bias: %.6f %.6f %.6f\n", bias.x, bias.y, bias.z);
    } else {
      LSM6DS3TR::Axes bias{strtof(a, nullptr), strtof(nextToken(&save), nullptr), strtof(nextToken(&save), nullptr)};
      strcmp(cmd, "biasxl") == 0 ? device.setAccelBias(bias) : device.setGyroBias(bias);
    }
  } else if (strcmp(cmd, "biasreset") == 0) {
    device.setAccelBias({}); device.setGyroBias({}); puts("Bias reset.");
  } else if (strcmp(cmd, "probe") == 0) {
    printStatus(device.probe());
  } else if (strcmp(cmd, "recover") == 0) {
    printStatus(device.recover());
  } else if (strcmp(cmd, "whoami") == 0 || strcmp(cmd, "id") == 0) {
    uint8_t id = 0; const auto st = device.readWhoAmI(id);
    st.ok() ? printf("WHO_AM_I=0x%02X\n", id) : printStatus(st);
  } else if (strcmp(cmd, "wusrc") == 0 || strcmp(cmd, "tapsrc") == 0 || strcmp(cmd, "6dsrc") == 0 ||
             strcmp(cmd, "funcsrc1") == 0 || strcmp(cmd, "funcsrc2") == 0 || strcmp(cmd, "wtstatus") == 0) {
    uint8_t value = 0; LSM6DS3TR::Status st;
    if (strcmp(cmd, "wusrc") == 0) st = device.readWakeUpSource(value);
    if (strcmp(cmd, "tapsrc") == 0) st = device.readTapSource(value);
    if (strcmp(cmd, "6dsrc") == 0) st = device.read6dSource(value);
    if (strcmp(cmd, "funcsrc1") == 0) st = device.readFunctionSource1(value);
    if (strcmp(cmd, "funcsrc2") == 0) st = device.readFunctionSource2(value);
    if (strcmp(cmd, "wtstatus") == 0) st = device.readWristTiltStatus(value);
    st.ok() ? printf("%s=0x%02X\n", cmd, value) : printStatus(st);
  } else if (strcmp(cmd, "shub") == 0) {
    uint32_t count = 12; parseU32(nextToken(&save), count);
    LSM6DS3TR::SensorHubData out; const auto st = device.readSensorHub(out, static_cast<uint8_t>(count));
    if (!st.ok()) printStatus(st);
    else for (uint8_t i = 0; i < out.count; ++i) printf("SHUB[%u]=0x%02X\n", i, out.bytes[i]);
  } else if (strcmp(cmd, "rreg") == 0) {
    uint32_t reg = 0; if (!parseU32(nextToken(&save), reg)) return;
    uint8_t value = 0; const auto st = device.readRegisterValue(static_cast<uint8_t>(reg), value);
    st.ok() ? printf("[0x%02lX]=0x%02X\n", static_cast<unsigned long>(reg), value) : printStatus(st);
  } else if (strcmp(cmd, "wreg") == 0) {
    uint32_t reg = 0, value = 0; if (!parseU32(nextToken(&save), reg) || !parseU32(nextToken(&save), value)) return;
    printStatus(device.writeRegisterValue(static_cast<uint8_t>(reg), static_cast<uint8_t>(value)));
  } else if (strcmp(cmd, "dump") == 0) {
    uint32_t start = 0, len = 32; parseU32(nextToken(&save), start); parseU32(nextToken(&save), len);
    uint8_t data[64] = {}; if (len > sizeof(data)) len = sizeof(data);
    const auto st = device.readRegisterBlock(static_cast<uint8_t>(start), data, len);
    if (!st.ok()) printStatus(st);
    else for (uint32_t i = 0; i < len; ++i) printf("0x%02lX: 0x%02X\n", static_cast<unsigned long>(start + i), data[i]);
  } else if (strcmp(cmd, "selftest") == 0) {
    puts("Selftest:");
    printStatus(device.probe());
    printHealth();
  } else if (strcmp(cmd, "stress") == 0 || strcmp(cmd, "stress_mix") == 0) {
    uint32_t count = strcmp(cmd, "stress_mix") == 0 ? 50 : 10;
    parseU32(nextToken(&save), count);
    runStress(count);
  } else {
    printf("Unknown command: %s\n", cmd);
  }
}

void configureI2c() {
  i2c_master_bus_config_t busConfig{};
  busConfig.clk_source = I2C_CLK_SRC_DEFAULT;
  busConfig.i2c_port = I2C_NUM_0;
  busConfig.sda_io_num = I2C_SDA;
  busConfig.scl_io_num = I2C_SCL;
  busConfig.glitch_ignore_cnt = 7;
  busConfig.flags.enable_internal_pullup = true;
  ESP_ERROR_CHECK(i2c_new_master_bus(&busConfig, &i2c.bus));

  i2c_device_config_t devConfig{};
  devConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  devConfig.device_address = I2C_ADDRESS;
  devConfig.scl_speed_hz = I2C_FREQ_HZ;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c.bus, &devConfig, &i2c.dev));
}

void cliLoop() {
  static char input[INPUT_MAX];
  size_t len = 0;
  printf("> ");
  while (true) {
    device.tick(nowMs(nullptr));
    if (streamMode && device.isOnline()) {
      printMeasurement();
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    const int c = getchar();
    if (c == EOF) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (c == '\b' || c == 0x7F) {
      if (len > 0) --len;
      continue;
    }
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        input[len] = '\0';
        processCommand(input);
        len = 0;
        printf("> ");
      }
      continue;
    }
    if (len < sizeof(input) - 1) input[len++] = static_cast<char>(c);
  }
}

}  // namespace

extern "C" void app_main(void) {
  setvbuf(stdin, nullptr, _IONBF, 0);
  setvbuf(stdout, nullptr, _IONBF, 0);

  puts("=== LSM6DS3TR native ESP-IDF bringup ===");
  configureI2c();
  scanI2c();
  printStatus(device.begin(makeConfig()));
  printHealth();
  puts("Type 'help' for commands.");
  cliLoop();
}
