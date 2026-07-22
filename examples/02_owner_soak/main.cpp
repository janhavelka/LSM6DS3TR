#include <Arduino.h>
#include <Wire.h>

#include <cinttypes>
#include <cstdint>
#include <limits>

#include "LSM6DS3TR/LSM6DS3TR.h"
#include "examples/common/BoardConfig.h"
#include "examples/common/I2cTransport.h"

namespace {

using namespace LSM6DS3TR;

static constexpr uint64_t SOAK_DURATION_MS = 3600000ULL;
static constexpr uint64_t SAMPLE_PERIOD_MS = 100ULL;
static constexpr uint64_t PROGRESS_PERIOD_MS = 30000ULL;
static constexpr uint64_t MAINTENANCE_PERIOD_MS = 300000ULL;
static constexpr uint8_t POLL_TRANSACTION_BUDGET = 1U;

enum class Phase : uint8_t {
  STARTUP_PROBE,
  STARTUP_CONFIGURE,
  SAMPLE,
  PERIODIC_PROBE,
  PERIODIC_RECONCILE,
  COMPLETE,
};

LSM6DS3TR::LSM6DS3TR device;
DeviceProfile profile{};
OperationToken pendingToken{};
Phase phase = Phase::STARTUP_PROBE;
uint32_t previousMillis = 0;
uint64_t millisEpoch = 0;
uint64_t soakStartedMs = 0;
uint64_t nextSampleMs = 0;
uint64_t nextProgressMs = 0;
uint64_t nextMaintenanceMs = 0;
uint64_t lastSequence = 0;
uint32_t expectedGeneration = 0;
uint32_t samples = 0;
uint32_t operationFailures = 0;
uint32_t contractFailures = 0;
uint32_t probeCount = 0;
uint32_t reconcileCount = 0;
int32_t minTemperatureMilliC = std::numeric_limits<int32_t>::max();
int32_t maxTemperatureMilliC = std::numeric_limits<int32_t>::min();
int64_t maxAbsAccelMicroG = 0;
int64_t maxAbsGyroMicroDps = 0;

uint64_t nowMs() {
  const uint32_t current = millis();
  if (current < previousMillis) {
    millisEpoch += (1ULL << 32U);
  }
  previousMillis = current;
  return millisEpoch + current;
}

OperationTiming timing(uint64_t now, uint32_t durationMs) {
  const uint64_t maximum = std::numeric_limits<uint64_t>::max();
  const uint64_t deadline = maximum - now < durationMs ? maximum : now + durationMs;
  return OperationTiming{now, deadline};
}

int64_t absoluteValue(int64_t value) {
  return value < 0 ? -value : value;
}

void noteContractFailure(const char* reason) {
  ++contractFailures;
  Serial.printf("HIL_CONTRACT_FAILURE reason=%s failures=%lu\n", reason,
                static_cast<unsigned long>(contractFailures));
  Serial.flush();
}

bool acceptStart(const Status& status, const OperationToken& token) {
  if ((!status.ok() && !status.inProgress()) || !token.valid()) {
    ++operationFailures;
    Serial.printf("HIL_START_FAILURE code=%u detail=%ld failures=%lu\n",
                  static_cast<unsigned>(status.code),
                  static_cast<long>(status.detail),
                  static_cast<unsigned long>(operationFailures));
    Serial.flush();
    return false;
  }
  pendingToken = token;
  return true;
}

bool startProbe(uint64_t now, Phase nextPhase) {
  OperationToken token{};
  phase = nextPhase;
  return acceptStart(device.startProbe(timing(now, 1000U), token), token);
}

bool startConfigure(uint64_t now) {
  OperationToken token{};
  phase = Phase::STARTUP_CONFIGURE;
  return acceptStart(device.startConfigure(profile, timing(now, 5000U), token), token);
}

bool startReconcile(uint64_t now) {
  OperationToken token{};
  phase = Phase::PERIODIC_RECONCILE;
  return acceptStart(device.startReconcile(timing(now, 3000U), token), token);
}

bool startSample(uint64_t now) {
  static constexpr uint8_t QUANTITIES[4] = {
      SAMPLE_ALL, SAMPLE_ACCELERATION, SAMPLE_ANGULAR_RATE, SAMPLE_TEMPERATURE};
  SampleRequest request{};
  const uint8_t pattern = static_cast<uint8_t>(samples & 0x07U);
  request.quantityMask = QUANTITIES[pattern >> 1U];
  request.checkDataReady = (pattern & 1U) == 0U;
  OperationToken token{};
  phase = Phase::SAMPLE;
  return acceptStart(device.startSample(request, timing(now, 1500U), token), token);
}

void updateRanges(const ConvertedSample& sample) {
  if ((sample.validMask & SAMPLE_ACCELERATION) != 0U) {
    const int64_t maximum =
        max(absoluteValue(sample.accelMicroG.x),
            max(absoluteValue(sample.accelMicroG.y),
                absoluteValue(sample.accelMicroG.z)));
    if (maximum > maxAbsAccelMicroG) maxAbsAccelMicroG = maximum;
    if (maximum > 2100000LL) noteContractFailure("accel_range");
  }
  if ((sample.validMask & SAMPLE_ANGULAR_RATE) != 0U) {
    const int64_t maximum =
        max(absoluteValue(sample.gyroMicroDps.x),
            max(absoluteValue(sample.gyroMicroDps.y),
                absoluteValue(sample.gyroMicroDps.z)));
    if (maximum > maxAbsGyroMicroDps) maxAbsGyroMicroDps = maximum;
    if (maximum > 251000000LL) noteContractFailure("gyro_range");
  }
  if ((sample.validMask & SAMPLE_TEMPERATURE) != 0U) {
    if (sample.temperatureMilliC < minTemperatureMilliC) {
      minTemperatureMilliC = sample.temperatureMilliC;
    }
    if (sample.temperatureMilliC > maxTemperatureMilliC) {
      maxTemperatureMilliC = sample.temperatureMilliC;
    }
    if (sample.temperatureMilliC < -40000 || sample.temperatureMilliC > 85000) {
      noteContractFailure("temperature_range");
    }
  }
}

void validateSample(const OperationResult& result) {
  const uint8_t pattern = static_cast<uint8_t>(samples & 0x07U);
  static constexpr uint8_t EXPECTED_MASKS[4] = {
      SAMPLE_ALL, SAMPLE_ACCELERATION, SAMPLE_ANGULAR_RATE, SAMPLE_TEMPERATURE};
  const uint8_t expectedMask = EXPECTED_MASKS[pattern >> 1U];
  const bool readyChecked = (pattern & 1U) == 0U;
  if (result.sample.validMask != expectedMask) noteContractFailure("valid_mask");
  if ((readyChecked && result.sample.freshMask != expectedMask) ||
      (!readyChecked && result.sample.freshMask != 0U)) {
    noteContractFailure("fresh_mask");
  }
  if (result.sample.sequence <= lastSequence) noteContractFailure("sequence");
  lastSequence = result.sample.sequence;
  if (result.sample.configGeneration != expectedGeneration) {
    noteContractFailure("generation");
  }
  ConvertedSample converted{};
  const Status convertedStatus = convertSample(result.sample, converted);
  if (!convertedStatus.ok()) {
    noteContractFailure("conversion");
  } else {
    updateRanges(converted);
  }
  ++samples;
}

bool terminalSuccess(const OperationResult& result, JobKind expectedKind,
                     uint64_t expectedToken) {
  if (result.token.value != expectedToken || result.kind != expectedKind) {
    noteContractFailure("result_identity");
    return false;
  }
  if (result.transactions > result.transactionLimit) {
    noteContractFailure("transaction_limit");
    return false;
  }
  if (result.state != OperationState::SUCCEEDED || !result.status.ok()) {
    ++operationFailures;
    Serial.printf(
        "HIL_OPERATION_FAILURE kind=%u state=%u code=%u transactions=%lu/%lu failures=%lu\n",
        static_cast<unsigned>(result.kind),
        static_cast<unsigned>(result.state),
        static_cast<unsigned>(result.status.code),
        static_cast<unsigned long>(result.transactions),
        static_cast<unsigned long>(result.transactionLimit),
        static_cast<unsigned long>(operationFailures));
    Serial.flush();
    return false;
  }
  return true;
}

void printProgress(uint64_t now, const char* marker) {
  const DriverDiagnostics diagnostics = device.diagnostics(now);
  const int32_t minimumTemperature =
      minTemperatureMilliC == std::numeric_limits<int32_t>::max()
          ? 0
          : minTemperatureMilliC;
  const int32_t maximumTemperature =
      maxTemperatureMilliC == std::numeric_limits<int32_t>::min()
          ? 0
          : maxTemperatureMilliC;
  Serial.printf(
      "%s elapsed_ms=%" PRIu64 " samples=%lu sequence=%" PRIu64
      " generation=%lu operation_failures=%lu contract_failures=%lu"
      " transport_ok=%lu transport_fail=%lu probes=%lu reconciles=%lu"
      " temp_min_mC=%ld temp_max_mC=%ld accel_abs_max_ug=%" PRId64
      " gyro_abs_max_udps=%" PRId64 "\n",
      marker, soakStartedMs == 0U ? 0U : now - soakStartedMs,
      static_cast<unsigned long>(samples), lastSequence,
      static_cast<unsigned long>(expectedGeneration),
      static_cast<unsigned long>(operationFailures),
      static_cast<unsigned long>(contractFailures),
      static_cast<unsigned long>(diagnostics.transportSuccesses),
      static_cast<unsigned long>(diagnostics.transportFailures),
      static_cast<unsigned long>(probeCount),
      static_cast<unsigned long>(reconcileCount),
      static_cast<long>(minimumTemperature),
      static_cast<long>(maximumTemperature), maxAbsAccelMicroG,
      maxAbsGyroMicroDps);
  Serial.flush();
}

void completeSoak(uint64_t now) {
  const DriverDiagnostics diagnostics = device.diagnostics(now);
  const bool passed = operationFailures == 0U && contractFailures == 0U &&
                      diagnostics.transportFailures == 0U && samples > 0U;
  printProgress(now, passed ? "HIL_SOAK_PASS" : "HIL_SOAK_FAIL");
  phase = Phase::COMPLETE;
}

void handleTerminal(const OperationResult& result, uint64_t now) {
  const Phase completedPhase = phase;
  const uint64_t completedToken = pendingToken.value;
  pendingToken = {};
  if (completedPhase == Phase::STARTUP_PROBE) {
    if (!terminalSuccess(result, JobKind::PROBE, completedToken) ||
        result.probe.address != static_cast<uint8_t>(SensorAddress::SA0_GND) ||
        result.probe.whoAmI != cmd::WHO_AM_I_VALUE) {
      Serial.println("HIL_SOAK_ABORT reason=startup_probe");
      Serial.flush();
      phase = Phase::COMPLETE;
      return;
    }
    (void)startConfigure(now);
    return;
  }
  if (completedPhase == Phase::STARTUP_CONFIGURE) {
    if (!terminalSuccess(result, JobKind::CONFIGURE, completedToken)) {
      Serial.println("HIL_SOAK_ABORT reason=startup_configure");
      Serial.flush();
      phase = Phase::COMPLETE;
      return;
    }
    expectedGeneration = result.configuration.generation;
    soakStartedMs = now;
    nextSampleMs = now;
    nextProgressMs = now + PROGRESS_PERIOD_MS;
    nextMaintenanceMs = now + MAINTENANCE_PERIOD_MS;
    Serial.printf("HIL_SOAK_START duration_ms=%" PRIu64
                  " sample_period_ms=%" PRIu64 " generation=%lu\n",
                  SOAK_DURATION_MS, SAMPLE_PERIOD_MS,
                  static_cast<unsigned long>(expectedGeneration));
    Serial.flush();
    phase = Phase::SAMPLE;
    return;
  }
  if (completedPhase == Phase::SAMPLE) {
    if (terminalSuccess(result, JobKind::SAMPLE, completedToken)) validateSample(result);
    nextSampleMs = now + SAMPLE_PERIOD_MS;
    return;
  }
  if (completedPhase == Phase::PERIODIC_PROBE) {
    if (terminalSuccess(result, JobKind::PROBE, completedToken) &&
        result.probe.whoAmI == cmd::WHO_AM_I_VALUE) {
      ++probeCount;
    } else {
      noteContractFailure("periodic_probe_identity");
    }
    (void)startReconcile(now);
    return;
  }
  if (completedPhase == Phase::PERIODIC_RECONCILE) {
    if (terminalSuccess(result, JobKind::RECONCILE, completedToken)) {
      ++reconcileCount;
      if (result.configuration.generation != expectedGeneration) {
        noteContractFailure("reconcile_generation");
      }
    }
    nextMaintenanceMs = now + MAINTENANCE_PERIOD_MS;
    nextSampleMs = now + SAMPLE_PERIOD_MS;
    phase = Phase::SAMPLE;
  }
}

void serviceOperation(uint64_t now) {
  if (device.operationActive()) {
    (void)device.poll(now, POLL_TRANSACTION_BUDGET);
  }
  if (!device.resultPending() || !pendingToken.valid()) return;
  OperationResult result{};
  const Status status = device.takeResult(pendingToken, result);
  if (!status.ok()) {
    noteContractFailure("take_result");
    pendingToken = {};
    return;
  }
  handleTerminal(result, now);
}

void scheduleWork(uint64_t now) {
  if (phase == Phase::COMPLETE || device.operationActive() ||
      device.resultPending()) {
    return;
  }
  if (soakStartedMs == 0U) return;
  if (now - soakStartedMs >= SOAK_DURATION_MS) {
    completeSoak(now);
    return;
  }
  if (now >= nextProgressMs) {
    printProgress(now, "HIL_SOAK_PROGRESS");
    nextProgressMs = now + PROGRESS_PERIOD_MS;
  }
  if (now >= nextMaintenanceMs) {
    (void)startProbe(now, Phase::PERIODIC_PROBE);
    return;
  }
  if (now >= nextSampleMs) (void)startSample(now);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(100);
  board::initI2c();

  DriverConfig config{};
  config.i2cWrite = transport::wireWrite;
  config.i2cWriteRead = transport::wireWriteRead;
  config.i2cUser = &Wire;
  config.address = SensorAddress::SA0_GND;
  config.i2cTimeoutMs = board::I2C_TIMEOUT_MS;
  const Status bound = device.bind(config);
  if (!bound.ok()) {
    Serial.printf("HIL_SOAK_ABORT reason=bind code=%u\n",
                  static_cast<unsigned>(bound.code));
    Serial.flush();
    phase = Phase::COMPLETE;
    return;
  }
  (void)startProbe(nowMs(), Phase::STARTUP_PROBE);
}

void loop() {
  const uint64_t now = nowMs();
  serviceOperation(now);
  scheduleWork(now);
  yield();
}
