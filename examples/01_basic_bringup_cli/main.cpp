#include <Arduino.h>
#include <Wire.h>

#include <cinttypes>
#include <cstdlib>
#include <cstring>

#include "LSM6DS3TR/LSM6DS3TR.h"
#include "examples/common/BoardConfig.h"
#include "examples/common/I2cTransport.h"

namespace {

using namespace LSM6DS3TR;

static constexpr size_t INPUT_CAPACITY = 128;
static constexpr uint8_t POLL_TRANSACTION_BUDGET = 1;
static constexpr uint8_t INPUT_CHARS_PER_LOOP = 16;

LSM6DS3TR::LSM6DS3TR device;
DeviceProfile profile{};
OperationToken pendingToken{};
bool configureAfterProbe = false;
char input[INPUT_CAPACITY]{};
size_t inputLength = 0;
bool inputOverflow = false;
uint32_t previousMillis = 0;
uint64_t millisEpoch = 0;

uint64_t nowMs() {
  const uint32_t current = millis();
  if (current < previousMillis) {
    millisEpoch += (1ULL << 32U);
  }
  previousMillis = current;
  return millisEpoch + current;
}

OperationTiming timing(uint64_t now, uint32_t durationMs) {
  return OperationTiming{now, now + durationMs};
}

const char* jobName(JobKind kind) {
  switch (kind) {
    case JobKind::PROBE: return "probe";
    case JobKind::CONFIGURE: return "configure";
    case JobKind::SAMPLE: return "sample";
    case JobKind::RESET: return "reset";
    case JobKind::BOOT: return "boot";
    case JobKind::RECOVER: return "recover";
    case JobKind::RECONCILE: return "reconcile";
    case JobKind::POWER_DOWN: return "powerdown";
    case JobKind::SELF_TEST: return "selftest";
    case JobKind::CALIBRATION: return "calibration";
    case JobKind::FIFO_PURGE: return "purge";
    default: return "none";
  }
}

const char* operationStateName(OperationState state) {
  switch (state) {
    case OperationState::IDLE: return "idle";
    case OperationState::ACTIVE: return "active";
    case OperationState::SUCCEEDED: return "succeeded";
    case OperationState::FAILED: return "failed";
    case OperationState::CANCELLED: return "cancelled";
    case OperationState::TIMED_OUT: return "timed_out";
    case OperationState::INDETERMINATE: return "indeterminate";
    default: return "unknown";
  }
}

const char* configurationStateName(ConfigurationState state) {
  switch (state) {
    case ConfigurationState::UNCONFIGURED: return "unconfigured";
    case ConfigurationState::APPLYING: return "applying";
    case ConfigurationState::KNOWN: return "known";
    case ConfigurationState::UNKNOWN: return "unknown";
    case ConfigurationState::SETTLING: return "settling";
    default: return "invalid";
  }
}

void printStatus(const Status& status) {
  Serial.printf("status=%u detail=%ld message=%s\n",
                static_cast<unsigned>(status.code),
                static_cast<long>(status.detail), status.msg);
}

bool acceptedStart(const Status& status, OperationToken token) {
  if ((!status.ok() && !status.inProgress()) || !token.valid()) {
    printStatus(status);
    return false;
  }
  pendingToken = token;
  Serial.printf("accepted token=%" PRIu64 "\n", token.value);
  return true;
}

void printSample(const RawSampleResult& raw) {
  ConvertedSample converted{};
  const Status convertedStatus = convertSample(raw, converted);
  if (!convertedStatus.ok()) {
    printStatus(convertedStatus);
    return;
  }

  Serial.printf("sample sequence=%" PRIu64 " generation=%lu valid=0x%02X fresh=0x%02X read_ms=%" PRIu64 "\n",
                converted.sequence,
                static_cast<unsigned long>(converted.configGeneration),
                converted.validMask, converted.freshMask,
                converted.readUptimeMs);
  if ((converted.validMask & SAMPLE_ACCELERATION) != 0U) {
    Serial.printf("  accel_ug x=%" PRId64 " y=%" PRId64 " z=%" PRId64 "\n",
                  converted.accelMicroG.x, converted.accelMicroG.y,
                  converted.accelMicroG.z);
  }
  if ((converted.validMask & SAMPLE_ANGULAR_RATE) != 0U) {
    Serial.printf("  gyro_udps x=%" PRId64 " y=%" PRId64 " z=%" PRId64 "\n",
                  converted.gyroMicroDps.x, converted.gyroMicroDps.y,
                  converted.gyroMicroDps.z);
  }
  if ((converted.validMask & SAMPLE_TEMPERATURE) != 0U) {
    Serial.printf("  temperature_mC=%ld\n",
                  static_cast<long>(converted.temperatureMilliC));
  }
}

bool startDefaultConfigure(uint64_t now) {
  OperationToken token{};
  return acceptedStart(device.startConfigure(profile, timing(now, 5000), token), token);
}

void printTerminalResult(const OperationResult& result) {
  Serial.printf("result token=%" PRIu64 " kind=%s state=%s transactions=%lu/%lu changed=%s\n",
                result.token.value, jobName(result.kind),
                operationStateName(result.state),
                static_cast<unsigned long>(result.transactions),
                static_cast<unsigned long>(result.transactionLimit),
                result.hardwareStateMayHaveChanged ? "yes" : "no");
  printStatus(result.status);

  if (result.kind == JobKind::PROBE && result.status.ok()) {
    Serial.printf("  address=0x%02X who_am_i=0x%02X\n",
                  result.probe.address, result.probe.whoAmI);
  } else if ((result.kind == JobKind::CONFIGURE ||
              result.kind == JobKind::RECONCILE ||
              result.kind == JobKind::RECOVER) && result.status.ok()) {
    Serial.printf("  config=%s generation=%lu valid_after_ms=%" PRIu64 "\n",
                  configurationStateName(result.configuration.state),
                  static_cast<unsigned long>(result.configuration.generation),
                  result.configuration.validAfterUptimeMs);
  } else if (result.kind == JobKind::SAMPLE && result.status.ok()) {
    printSample(result.sample);
  } else if (result.kind == JobKind::SELF_TEST) {
    Serial.printf("  accel_pass=%s gyro_pass=%s restore=%u\n",
                  result.selfTest.accelPass ? "yes" : "no",
                  result.selfTest.gyroPass ? "yes" : "no",
                  static_cast<unsigned>(result.selfTest.restorationStatus.code));
  } else if (result.kind == JobKind::CALIBRATION && result.status.ok()) {
    Serial.printf("  bias x=%.6f y=%.6f z=%.6f peak_to_peak x=%.6f y=%.6f z=%.6f samples=%u\n",
                  result.calibration.bias.x, result.calibration.bias.y,
                  result.calibration.bias.z, result.calibration.peakToPeak.x,
                  result.calibration.peakToPeak.y,
                  result.calibration.peakToPeak.z,
                  result.calibration.samples);
  } else if (result.kind == JobKind::FIFO_PURGE) {
    Serial.printf("  discarded=%u initial=%u final=%u overrun=%s truncated=%s\n",
                  result.fifoPurge.wordsDiscarded,
                  result.fifoPurge.initialUnreadWords,
                  result.fifoPurge.finalUnreadWords,
                  result.fifoPurge.overrunObserved ? "yes" : "no",
                  result.fifoPurge.truncated ? "yes" : "no");
  }
}

void serviceOperation(uint64_t now) {
  if (device.operationActive()) {
    (void)device.poll(now, POLL_TRANSACTION_BUDGET);
  }
  if (!device.resultPending() || !pendingToken.valid()) {
    return;
  }

  OperationResult result{};
  const Status takeStatus = device.takeResult(pendingToken, result);
  if (!takeStatus.ok()) {
    printStatus(takeStatus);
    pendingToken = {};
    configureAfterProbe = false;
    return;
  }
  pendingToken = {};
  printTerminalResult(result);

  if (configureAfterProbe && result.kind == JobKind::PROBE) {
    configureAfterProbe = false;
    if (result.status.ok()) {
      (void)startDefaultConfigure(now);
    }
  }
}

bool parseUnsigned(const char* text, uint32_t maximum, uint32_t& out) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  char* end = nullptr;
  const unsigned long value = strtoul(text, &end, 0);
  if (*end != '\0' || value > maximum) {
    return false;
  }
  out = static_cast<uint32_t>(value);
  return true;
}

void printHelp() {
  Serial.println("Owner-safe: probe configure sample [all|accel|gyro|temp] [ready|direct]");
  Serial.println("Owner-safe: reset boot recover reconcile powerdown selftest [5..100]");
  Serial.println("Maintenance: calxl [1..1000] calg [1..1000] purge <1..2048>");
  Serial.println("Lifecycle: bind unbind cancel status version help");
  Serial.println("Advanced diagnostics: rreg <reg> wreg <reg> <value> dump <reg> <1..32>");
  Serial.println("All jobs use absolute deadlines and one transport callback per loop poll.");
  Serial.println("calxl is a Z-up fixture example; product mounting policy belongs above the driver.");
}

Status bindDriver() {
  DriverConfig config{};
  config.i2cWrite = transport::wireWrite;
  config.i2cWriteRead = transport::wireWriteRead;
  config.i2cUser = &Wire;
  config.address = SensorAddress::SA0_GND;
  config.i2cTimeoutMs = board::I2C_TIMEOUT_MS;
  return device.bind(config);
}

void handleCommand(char* line) {
  char* save = nullptr;
  char* command = strtok_r(line, " \t", &save);
  if (command == nullptr) {
    return;
  }
  const uint64_t now = nowMs();

  if (strcmp(command, "help") == 0) {
    printHelp();
  } else if (strcmp(command, "version") == 0) {
    Serial.printf("LSM6DS3TR %s\n", VERSION_FULL);
  } else if (strcmp(command, "status") == 0) {
    const DriverDiagnostics diag = device.diagnostics(now);
    Serial.printf("bound=%s active=%s result_pending=%s config=%s generation=%lu valid_after=%" PRIu64 "\n",
                  device.isBound() ? "yes" : "no",
                  device.operationActive() ? "yes" : "no",
                  device.resultPending() ? "yes" : "no",
                  configurationStateName(diag.configurationState),
                  static_cast<unsigned long>(diag.configGeneration),
                  diag.validAfterUptimeMs);
    Serial.printf("transport ok=%lu fail=%lu last_error=%u\n",
                  static_cast<unsigned long>(diag.transportSuccesses),
                  static_cast<unsigned long>(diag.transportFailures),
                  static_cast<unsigned>(diag.lastTransportError.code));
  } else if (strcmp(command, "bind") == 0) {
    printStatus(bindDriver());
  } else if (strcmp(command, "unbind") == 0) {
    device.unbind();
    pendingToken = {};
    configureAfterProbe = false;
    Serial.println("unbound (zero I2C)");
  } else if (strcmp(command, "cancel") == 0) {
    printStatus(device.cancelActiveJob(now));
  } else if (strcmp(command, "probe") == 0) {
    OperationToken token{};
    (void)acceptedStart(device.startProbe(timing(now, 500), token), token);
  } else if (strcmp(command, "configure") == 0) {
    (void)startDefaultConfigure(now);
  } else if (strcmp(command, "sample") == 0) {
    SampleRequest request{};
    const char* quantity = strtok_r(nullptr, " \t", &save);
    const char* mode = strtok_r(nullptr, " \t", &save);
    const char* extra = strtok_r(nullptr, " \t", &save);
    const bool validQuantity =
        quantity == nullptr || strcmp(quantity, "all") == 0 ||
        strcmp(quantity, "accel") == 0 || strcmp(quantity, "gyro") == 0 ||
        strcmp(quantity, "temp") == 0;
    const bool validMode = mode == nullptr || strcmp(mode, "ready") == 0 ||
                           strcmp(mode, "direct") == 0;
    if (!validQuantity || !validMode || extra != nullptr) {
      Serial.println("expected sample [all|accel|gyro|temp] [ready|direct]");
      return;
    }
    if (quantity != nullptr && strcmp(quantity, "accel") == 0) {
      request.quantityMask = SAMPLE_ACCELERATION;
    } else if (quantity != nullptr && strcmp(quantity, "gyro") == 0) {
      request.quantityMask = SAMPLE_ANGULAR_RATE;
    } else if (quantity != nullptr && strcmp(quantity, "temp") == 0) {
      request.quantityMask = SAMPLE_TEMPERATURE;
    }
    request.checkDataReady = mode == nullptr || strcmp(mode, "direct") != 0;
    OperationToken token{};
    (void)acceptedStart(device.startSample(request, timing(now, 1500), token), token);
  } else if (strcmp(command, "reset") == 0 || strcmp(command, "boot") == 0 ||
             strcmp(command, "recover") == 0 || strcmp(command, "reconcile") == 0 ||
             strcmp(command, "powerdown") == 0) {
    OperationToken token{};
    Status status = Status::Error(Err::INVALID_PARAM, "unknown operation");
    if (strcmp(command, "reset") == 0) status = device.startReset(timing(now, 5000), token);
    if (strcmp(command, "boot") == 0) status = device.startBoot(timing(now, 5000), token);
    if (strcmp(command, "recover") == 0) status = device.startRecover(timing(now, 5000), token);
    if (strcmp(command, "reconcile") == 0) status = device.startReconcile(timing(now, 3000), token);
    if (strcmp(command, "powerdown") == 0) status = device.startPowerDown(timing(now, 1000), token);
    (void)acceptedStart(status, token);
  } else if (strcmp(command, "selftest") == 0) {
    uint32_t samples = 5;
    const char* argument = strtok_r(nullptr, " \t", &save);
    if (argument != nullptr &&
        (!parseUnsigned(argument, 100, samples) || samples < 5U)) {
      Serial.println("expected selftest [5..100]");
      return;
    }
    SelfTestRequest request{static_cast<uint16_t>(samples)};
    OperationToken token{};
    (void)acceptedStart(device.startSelfTest(request, timing(now, 20000), token), token);
  } else if (strcmp(command, "calxl") == 0 || strcmp(command, "calg") == 0) {
    uint32_t samples = 32;
    const char* argument = strtok_r(nullptr, " \t", &save);
    const char* extra = strtok_r(nullptr, " \t", &save);
    if ((argument != nullptr &&
         (!parseUnsigned(argument, 1000, samples) || samples == 0U)) ||
        extra != nullptr) {
      Serial.println("expected calibration sample count 1..1000");
      return;
    }
    CalibrationRequest request{};
    request.samples = static_cast<uint16_t>(samples);
    request.kind = strcmp(command, "calxl") == 0
                       ? CalibrationKind::ACCELEROMETER_BIAS
                       : CalibrationKind::GYROSCOPE_BIAS;
    if (request.kind == CalibrationKind::ACCELEROMETER_BIAS) {
      request.expectedAccelerationG.z = 1.0f;
    }
    OperationToken token{};
    (void)acceptedStart(device.startCalibration(request, timing(now, 30000), token), token);
  } else if (strcmp(command, "purge") == 0) {
    uint32_t words = 0;
    if (!parseUnsigned(strtok_r(nullptr, " \t", &save), 2048, words) || words == 0U) {
      Serial.println("expected purge <1..2048>");
      return;
    }
    FifoPurgeRequest request{static_cast<uint16_t>(words)};
    OperationToken token{};
    (void)acceptedStart(device.startFifoPurge(request, timing(now, 5000), token), token);
  } else if (strcmp(command, "rreg") == 0) {
    uint32_t reg = 0;
    if (!parseUnsigned(strtok_r(nullptr, " \t", &save), 0xFF, reg)) {
      Serial.println("expected rreg <0..255>");
      return;
    }
    uint8_t value = 0;
    const Status status =
        device.diagnosticReadRegister(static_cast<uint8_t>(reg), value, now);
    printStatus(status);
    if (status.ok()) Serial.printf("0x%02lX = 0x%02X\n", static_cast<unsigned long>(reg), value);
  } else if (strcmp(command, "wreg") == 0) {
    uint32_t reg = 0;
    uint32_t value = 0;
    if (!parseUnsigned(strtok_r(nullptr, " \t", &save), 0xFF, reg) ||
        !parseUnsigned(strtok_r(nullptr, " \t", &save), 0xFF, value)) {
      Serial.println("expected wreg <0..255> <0..255>");
      return;
    }
    printStatus(device.diagnosticWriteRegister(static_cast<uint8_t>(reg),
                                               static_cast<uint8_t>(value), now));
  } else if (strcmp(command, "dump") == 0) {
    uint32_t reg = 0;
    uint32_t length = 0;
    if (!parseUnsigned(strtok_r(nullptr, " \t", &save), 0xFF, reg) ||
        !parseUnsigned(strtok_r(nullptr, " \t", &save), 32, length) || length == 0U) {
      Serial.println("expected dump <0..255> <1..32>");
      return;
    }
    uint8_t bytes[32]{};
    const Status status =
        device.diagnosticReadBlock(static_cast<uint8_t>(reg), bytes, length, now);
    printStatus(status);
    if (status.ok()) {
      for (uint32_t i = 0; i < length; ++i) Serial.printf("%02X%c", bytes[i], i + 1U == length ? '\n' : ' ');
    }
  } else {
    Serial.println("unknown command; type help");
  }
}

void serviceInput() {
  for (uint8_t serviced = 0;
       serviced < INPUT_CHARS_PER_LOOP && Serial.available() > 0; ++serviced) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      if (inputOverflow) {
        Serial.println("input line too long; discarded");
      } else {
        input[inputLength] = '\0';
        handleCommand(input);
      }
      inputLength = 0;
      inputOverflow = false;
      continue;
    }
    if (inputOverflow) continue;
    if (inputLength + 1U < INPUT_CAPACITY) {
      input[inputLength++] = c;
    } else {
      inputOverflow = true;
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(100);
  board::initI2c();
  Serial.printf("LSM6DS3TR owner-safe example %s\n", LSM6DS3TR::VERSION_FULL);
  printHelp();

  const Status bound = bindDriver();
  printStatus(bound);
  if (bound.ok()) {
    OperationToken token{};
    configureAfterProbe = true;
    if (!acceptedStart(device.startProbe(timing(nowMs(), 500), token), token)) {
      configureAfterProbe = false;
    }
  }
}

void loop() {
  const uint64_t now = nowMs();
  serviceInput();
  serviceOperation(now);
  yield();
}
