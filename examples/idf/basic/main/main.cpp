#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "LSM6DS3TR/LSM6DS3TR.h"

namespace {

using namespace LSM6DS3TR;

static constexpr gpio_num_t I2C_SDA = GPIO_NUM_8;
static constexpr gpio_num_t I2C_SCL = GPIO_NUM_9;
static constexpr uint8_t I2C_ADDRESS = 0x6A;
static constexpr uint32_t I2C_FREQUENCY_HZ = 400000;
static constexpr uint32_t I2C_TIMEOUT_MS = 20;
static constexpr size_t INPUT_CAPACITY = 128;

struct I2cContext {
  i2c_master_bus_handle_t bus = nullptr;
  i2c_master_dev_handle_t device = nullptr;
};

I2cContext i2c{};
LSM6DS3TR::LSM6DS3TR imu;
DeviceProfile profile{};
OperationToken pendingToken{};
bool configureAfterProbe = false;

uint64_t nowMs() {
  return static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;
}

OperationTiming timing(uint64_t now, uint32_t durationMs) {
  return OperationTiming{now, now + durationMs};
}

Status mapEspError(esp_err_t error, const char* message) {
  if (error == ESP_OK) return Status::Ok();
  if (error == ESP_ERR_TIMEOUT) {
    return Status::Error(Err::I2C_TIMEOUT, message, static_cast<int32_t>(error));
  }
  if (error == ESP_ERR_INVALID_ARG) {
    return Status::Error(Err::INVALID_PARAM, message, static_cast<int32_t>(error));
  }
  return Status::Error(Err::I2C_ERROR, message, static_cast<int32_t>(error));
}

Status i2cWrite(uint8_t address, const uint8_t* data, size_t length,
                uint32_t timeoutMs, void* user) {
  I2cContext* context = static_cast<I2cContext*>(user);
  if (context == nullptr || context->device == nullptr || address != I2C_ADDRESS) {
    return Status::Error(Err::INVALID_CONFIG, "I2C context/address invalid");
  }
  if (data == nullptr || length == 0U) {
    return Status::Error(Err::INVALID_PARAM, "I2C write buffer invalid");
  }
  return mapEspError(
      i2c_master_transmit(context->device, data, length,
                          static_cast<int>(timeoutMs)),
      "I2C write failed");
}

Status i2cWriteRead(uint8_t address, const uint8_t* txData, size_t txLength,
                    uint8_t* rxData, size_t rxLength, uint32_t timeoutMs,
                    void* user) {
  I2cContext* context = static_cast<I2cContext*>(user);
  if (context == nullptr || context->device == nullptr || address != I2C_ADDRESS) {
    return Status::Error(Err::INVALID_CONFIG, "I2C context/address invalid");
  }
  if (txData == nullptr || txLength == 0U || rxData == nullptr || rxLength == 0U) {
    return Status::Error(Err::INVALID_PARAM, "I2C read buffers invalid");
  }
  return mapEspError(
      i2c_master_transmit_receive(context->device, txData, txLength, rxData,
                                  rxLength, static_cast<int>(timeoutMs)),
      "I2C write-read failed");
}

Status bindDriver() {
  DriverConfig config{};
  config.i2cWrite = i2cWrite;
  config.i2cWriteRead = i2cWriteRead;
  config.i2cUser = &i2c;
  config.address = SensorAddress::SA0_GND;
  config.i2cTimeoutMs = I2C_TIMEOUT_MS;
  return imu.bind(config);
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

const char* stateName(OperationState state) {
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

const char* configStateName(ConfigurationState state) {
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
  printf("status=%u detail=%" PRId32 " message=%s\n",
         static_cast<unsigned>(status.code), status.detail, status.msg);
}

bool acceptedStart(const Status& status, OperationToken token) {
  if ((!status.ok() && !status.inProgress()) || !token.valid()) {
    printStatus(status);
    return false;
  }
  pendingToken = token;
  printf("accepted token=%" PRIu64 "\n", token.value);
  return true;
}

void printSample(const RawSampleResult& raw) {
  ConvertedSample converted{};
  const Status status = convertSample(raw, converted);
  if (!status.ok()) {
    printStatus(status);
    return;
  }
  printf("sample sequence=%" PRIu64 " generation=%" PRIu32
         " valid=0x%02X fresh=0x%02X read_ms=%" PRIu64 "\n",
         converted.sequence, converted.configGeneration, converted.validMask,
         converted.freshMask, converted.readUptimeMs);
  if ((converted.validMask & SAMPLE_ACCELERATION) != 0U) {
    printf("  accel_ug x=%" PRId64 " y=%" PRId64 " z=%" PRId64 "\n",
           converted.accelMicroG.x, converted.accelMicroG.y,
           converted.accelMicroG.z);
  }
  if ((converted.validMask & SAMPLE_ANGULAR_RATE) != 0U) {
    printf("  gyro_udps x=%" PRId64 " y=%" PRId64 " z=%" PRId64 "\n",
           converted.gyroMicroDps.x, converted.gyroMicroDps.y,
           converted.gyroMicroDps.z);
  }
  if ((converted.validMask & SAMPLE_TEMPERATURE) != 0U) {
    printf("  temperature_mC=%" PRId32 "\n", converted.temperatureMilliC);
  }
}

bool startConfigure(uint64_t now) {
  OperationToken token{};
  return acceptedStart(imu.startConfigure(profile, timing(now, 5000), token), token);
}

void printTerminal(const OperationResult& result) {
  printf("result token=%" PRIu64 " kind=%s state=%s transactions=%" PRIu32
         "/%" PRIu32 " changed=%s\n",
         result.token.value, jobName(result.kind), stateName(result.state),
         result.transactions, result.transactionLimit,
         result.hardwareStateMayHaveChanged ? "yes" : "no");
  printStatus(result.status);
  if (result.kind == JobKind::PROBE && result.status.ok()) {
    printf("  address=0x%02X who_am_i=0x%02X\n", result.probe.address,
           result.probe.whoAmI);
  } else if (result.kind == JobKind::SAMPLE && result.status.ok()) {
    printSample(result.sample);
  } else if (result.kind == JobKind::SELF_TEST) {
    printf("  accel_pass=%s gyro_pass=%s restore=%u\n",
           result.selfTest.accelPass ? "yes" : "no",
           result.selfTest.gyroPass ? "yes" : "no",
           static_cast<unsigned>(result.selfTest.restorationStatus.code));
  } else if (result.kind == JobKind::CALIBRATION && result.status.ok()) {
    printf("  bias x=%.6f y=%.6f z=%.6f peak_to_peak x=%.6f y=%.6f z=%.6f samples=%u\n",
           result.calibration.bias.x, result.calibration.bias.y,
           result.calibration.bias.z, result.calibration.peakToPeak.x,
           result.calibration.peakToPeak.y,
           result.calibration.peakToPeak.z, result.calibration.samples);
  } else if (result.kind == JobKind::FIFO_PURGE) {
    printf("  discarded=%u initial=%u final=%u overrun=%s truncated=%s\n",
           result.fifoPurge.wordsDiscarded,
           result.fifoPurge.initialUnreadWords,
           result.fifoPurge.finalUnreadWords,
           result.fifoPurge.overrunObserved ? "yes" : "no",
           result.fifoPurge.truncated ? "yes" : "no");
  }
  fflush(stdout);
}

void serviceOperation(uint64_t now) {
  if (imu.operationActive()) {
    (void)imu.poll(now, 1);
  }
  if (!imu.resultPending() || !pendingToken.valid()) return;

  OperationResult result{};
  const Status take = imu.takeResult(pendingToken, result);
  if (!take.ok()) {
    printStatus(take);
    pendingToken = {};
    configureAfterProbe = false;
    return;
  }
  pendingToken = {};
  printTerminal(result);
  if (configureAfterProbe && result.kind == JobKind::PROBE) {
    configureAfterProbe = false;
    if (result.status.ok()) (void)startConfigure(now);
  }
}

bool parseUnsigned(const char* text, uint32_t maximum, uint32_t& out) {
  if (text == nullptr || *text == '\0') return false;
  char* end = nullptr;
  const unsigned long value = strtoul(text, &end, 0);
  if (*end != '\0' || value > maximum) return false;
  out = static_cast<uint32_t>(value);
  return true;
}

void printHelp() {
  puts("Owner-safe: probe configure sample [all|accel|gyro|temp] [ready|direct]");
  puts("Owner-safe: reset boot recover reconcile powerdown selftest [5..100]");
  puts("Maintenance: calxl [1..1000] calg [1..1000] purge <1..2048>");
  puts("Lifecycle: bind unbind cancel status version help");
  puts("Advanced diagnostics: rreg <reg> wreg <reg> <value> dump <reg> <1..32>");
  puts("The owner calls poll with one transaction per pass; no driver sleep/retry/recovery.");
  puts("calxl is a Z-up fixture example; product mounting policy belongs above the driver.");
}

void processCommand(char* line) {
  char* save = nullptr;
  char* command = strtok_r(line, " \t", &save);
  if (command == nullptr) return;
  const char* argument1 = strtok_r(nullptr, " \t", &save);
  const char* argument2 = strtok_r(nullptr, " \t", &save);
  const char* argument3 = strtok_r(nullptr, " \t", &save);
  const bool zeroArgumentCommand =
      strcmp(command, "help") == 0 || strcmp(command, "version") == 0 ||
      strcmp(command, "status") == 0 || strcmp(command, "bind") == 0 ||
      strcmp(command, "unbind") == 0 || strcmp(command, "cancel") == 0 ||
      strcmp(command, "probe") == 0 || strcmp(command, "configure") == 0 ||
      strcmp(command, "reset") == 0 || strcmp(command, "boot") == 0 ||
      strcmp(command, "recover") == 0 || strcmp(command, "reconcile") == 0 ||
      strcmp(command, "powerdown") == 0;
  if (zeroArgumentCommand && argument1 != nullptr) {
    printf("expected %s\n", command);
    return;
  }
  const uint64_t now = nowMs();

  if (strcmp(command, "help") == 0) {
    printHelp();
  } else if (strcmp(command, "version") == 0) {
    printf("LSM6DS3TR %s\n", VERSION_FULL);
  } else if (strcmp(command, "status") == 0) {
    const DriverDiagnostics diag = imu.diagnostics(now);
    printf("bound=%s active=%s result_pending=%s config=%s generation=%" PRIu32
           " valid_after=%" PRIu64 "\n",
           imu.isBound() ? "yes" : "no", imu.operationActive() ? "yes" : "no",
           imu.resultPending() ? "yes" : "no",
           configStateName(diag.configurationState), diag.configGeneration,
           diag.validAfterUptimeMs);
  } else if (strcmp(command, "bind") == 0) {
    printStatus(bindDriver());
  } else if (strcmp(command, "unbind") == 0) {
    imu.unbind();
    pendingToken = {};
    configureAfterProbe = false;
    puts("unbound (zero I2C)");
  } else if (strcmp(command, "cancel") == 0) {
    printStatus(imu.cancelActiveJob(now));
  } else if (strcmp(command, "probe") == 0) {
    OperationToken token{};
    (void)acceptedStart(imu.startProbe(timing(now, 500), token), token);
  } else if (strcmp(command, "configure") == 0) {
    (void)startConfigure(now);
  } else if (strcmp(command, "sample") == 0) {
    SampleRequest request{};
    const char* quantity = argument1;
    const char* mode = argument2;
    const bool validQuantity =
        quantity == nullptr || strcmp(quantity, "all") == 0 ||
        strcmp(quantity, "accel") == 0 || strcmp(quantity, "gyro") == 0 ||
        strcmp(quantity, "temp") == 0;
    const bool validMode = mode == nullptr || strcmp(mode, "ready") == 0 ||
                           strcmp(mode, "direct") == 0;
    if (!validQuantity || !validMode || argument3 != nullptr) {
      puts("expected sample [all|accel|gyro|temp] [ready|direct]");
      return;
    }
    if (quantity != nullptr && strcmp(quantity, "accel") == 0) request.quantityMask = SAMPLE_ACCELERATION;
    if (quantity != nullptr && strcmp(quantity, "gyro") == 0) request.quantityMask = SAMPLE_ANGULAR_RATE;
    if (quantity != nullptr && strcmp(quantity, "temp") == 0) request.quantityMask = SAMPLE_TEMPERATURE;
    request.checkDataReady = mode == nullptr || strcmp(mode, "direct") != 0;
    OperationToken token{};
    (void)acceptedStart(imu.startSample(request, timing(now, 1500), token), token);
  } else if (strcmp(command, "reset") == 0 || strcmp(command, "boot") == 0 ||
             strcmp(command, "recover") == 0 || strcmp(command, "reconcile") == 0 ||
             strcmp(command, "powerdown") == 0) {
    OperationToken token{};
    Status status = Status::Error(Err::INVALID_PARAM, "unknown operation");
    if (strcmp(command, "reset") == 0) status = imu.startReset(timing(now, 5000), token);
    if (strcmp(command, "boot") == 0) status = imu.startBoot(timing(now, 5000), token);
    if (strcmp(command, "recover") == 0) status = imu.startRecover(timing(now, 5000), token);
    if (strcmp(command, "reconcile") == 0) status = imu.startReconcile(timing(now, 3000), token);
    if (strcmp(command, "powerdown") == 0) status = imu.startPowerDown(timing(now, 1000), token);
    (void)acceptedStart(status, token);
  } else if (strcmp(command, "selftest") == 0) {
    uint32_t samples = 5;
    if ((argument1 != nullptr &&
         (!parseUnsigned(argument1, 100, samples) || samples < 5U)) ||
        argument2 != nullptr) {
      puts("expected selftest [5..100]");
      return;
    }
    OperationToken token{};
    (void)acceptedStart(imu.startSelfTest(SelfTestRequest{static_cast<uint16_t>(samples)},
                                         timing(now, 20000), token), token);
  } else if (strcmp(command, "calxl") == 0 || strcmp(command, "calg") == 0) {
    uint32_t samples = 32;
    if ((argument1 != nullptr &&
         (!parseUnsigned(argument1, 1000, samples) || samples == 0U)) ||
        argument2 != nullptr) {
      puts("expected calibration sample count 1..1000");
      return;
    }
    CalibrationRequest request{};
    request.samples = static_cast<uint16_t>(samples);
    request.kind = strcmp(command, "calxl") == 0
                       ? CalibrationKind::ACCELEROMETER_BIAS
                       : CalibrationKind::GYROSCOPE_BIAS;
    if (request.kind == CalibrationKind::ACCELEROMETER_BIAS) request.expectedAccelerationG.z = 1.0f;
    OperationToken token{};
    (void)acceptedStart(imu.startCalibration(request, timing(now, 30000), token), token);
  } else if (strcmp(command, "purge") == 0) {
    uint32_t words = 0;
    if (!parseUnsigned(argument1, 2048, words) || words == 0U ||
        argument2 != nullptr) {
      puts("expected purge <1..2048>");
      return;
    }
    OperationToken token{};
    (void)acceptedStart(imu.startFifoPurge(FifoPurgeRequest{static_cast<uint16_t>(words)},
                                          timing(now, 5000), token), token);
  } else if (strcmp(command, "rreg") == 0) {
    uint32_t reg = 0;
    if (!parseUnsigned(argument1, 0xFF, reg) || argument2 != nullptr) {
      puts("expected rreg <0..255>");
      return;
    }
    uint8_t value = 0;
    const Status status =
        imu.diagnosticReadRegister(static_cast<uint8_t>(reg), value, now);
    printStatus(status);
    if (status.ok()) printf("0x%02" PRIX32 " = 0x%02X\n", reg, value);
  } else if (strcmp(command, "wreg") == 0) {
    uint32_t reg = 0;
    uint32_t value = 0;
    if (!parseUnsigned(argument1, 0xFF, reg) ||
        !parseUnsigned(argument2, 0xFF, value) || argument3 != nullptr) {
      puts("expected wreg <0..255> <0..255>");
      return;
    }
    printStatus(imu.diagnosticWriteRegister(static_cast<uint8_t>(reg),
                                            static_cast<uint8_t>(value), now));
  } else if (strcmp(command, "dump") == 0) {
    uint32_t reg = 0;
    uint32_t length = 0;
    if (!parseUnsigned(argument1, 0xFF, reg) ||
        !parseUnsigned(argument2, 32, length) || length == 0U ||
        argument3 != nullptr) {
      puts("expected dump <0..255> <1..32>");
      return;
    }
    uint8_t bytes[32]{};
    const Status status =
        imu.diagnosticReadBlock(static_cast<uint8_t>(reg), bytes, length, now);
    printStatus(status);
    if (status.ok()) {
      for (uint32_t index = 0; index < length; ++index) {
        printf("%02X%c", bytes[index], index + 1U == length ? '\n' : ' ');
      }
    }
  } else {
    printf("unknown command: %s\n", command);
  }
}

esp_err_t configureI2c() {
  i2c_master_bus_config_t busConfig{};
  busConfig.clk_source = I2C_CLK_SRC_DEFAULT;
  busConfig.i2c_port = I2C_NUM_0;
  busConfig.sda_io_num = I2C_SDA;
  busConfig.scl_io_num = I2C_SCL;
  busConfig.glitch_ignore_cnt = 7;
  busConfig.flags.enable_internal_pullup = true;
  esp_err_t error = i2c_new_master_bus(&busConfig, &i2c.bus);
  if (error != ESP_OK) return error;

  i2c_device_config_t deviceConfig{};
  deviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  deviceConfig.device_address = I2C_ADDRESS;
  deviceConfig.scl_speed_hz = I2C_FREQUENCY_HZ;
  return i2c_master_bus_add_device(i2c.bus, &deviceConfig, &i2c.device);
}

void cliLoop() {
  char input[INPUT_CAPACITY]{};
  size_t length = 0;
  bool inputOverflow = false;
  puts("type help for commands");
  while (true) {
    serviceOperation(nowMs());
    const int c = getchar();
    if (c == EOF) {
      clearerr(stdin);
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    if (c == '\b' || c == 0x7F) {
      if (!inputOverflow && length > 0U) --length;
    } else if (c == '\n' || c == '\r') {
      if (inputOverflow) {
        puts("input line too long; discarded");
      } else if (length > 0U) {
        input[length] = '\0';
        processCommand(input);
      }
      length = 0;
      inputOverflow = false;
    } else if (inputOverflow) {
      continue;
    } else if (length + 1U < INPUT_CAPACITY) {
      input[length++] = static_cast<char>(c);
    } else {
      inputOverflow = true;
    }
  }
}

}  // namespace

extern "C" void app_main(void) {
  setvbuf(stdin, nullptr, _IONBF, 0);
  setvbuf(stdout, nullptr, _IONBF, 0);
  const int stdinFlags = fcntl(STDIN_FILENO, F_GETFL, 0);
  if (stdinFlags < 0 || fcntl(STDIN_FILENO, F_SETFL, stdinFlags | O_NONBLOCK) < 0) {
    puts("failed to configure non-blocking console input");
    return;
  }
  puts("LSM6DS3TR native ESP-IDF owner-safe example");

  const esp_err_t i2cStatus = configureI2c();
  if (i2cStatus != ESP_OK) {
    printf("I2C initialization failed: %s\n", esp_err_to_name(i2cStatus));
    return;
  }
  const Status bound = bindDriver();
  printStatus(bound);
  if (bound.ok()) {
    OperationToken token{};
    configureAfterProbe = true;
    if (!acceptedStart(imu.startProbe(timing(nowMs(), 500), token), token)) {
      configureAfterProbe = false;
    }
  }
  cliLoop();
}
