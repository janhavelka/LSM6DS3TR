#include <Arduino.h>
#include <Wire.h>

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>

#include "LSM6DS3TR/LSM6DS3TR.h"
#include "examples/common/BoardConfig.h"
#include "examples/common/I2cTransport.h"
#include "examples/common/ProfileCli.h"

namespace {

using namespace LSM6DS3TR;

static constexpr size_t INPUT_CAPACITY = 128;
static constexpr uint8_t POLL_TRANSACTION_BUDGET = 1;
static constexpr uint8_t INPUT_CHARS_PER_LOOP = 16;
static constexpr size_t MAX_COMMAND_TOKENS = 8;
static constexpr uint32_t MAX_STRESS_COUNT = 10000;
static constexpr uint8_t SCAN_FIRST_ADDRESS = 0x6A;
static constexpr uint8_t SCAN_LAST_ADDRESS = 0x6B;

LSM6DS3TR::LSM6DS3TR device;
DeviceProfile stagedProfile{};
SensorAddress selectedAddress = SensorAddress::SA0_GND;
uint32_t selectedFrequencyHz = board::I2C_FREQ_HZ;
bool busReady = false;
OperationToken pendingToken{};
bool configureAfterProbe = false;
PollResult lastPoll{};
OperationResult lastResult{};
bool lastResultAvailable = false;
char input[INPUT_CAPACITY]{};
size_t inputLength = 0;
bool inputOverflow = false;
uint32_t previousMillis = 0;
uint64_t millisEpoch = 0;

enum class SessionKind : uint8_t {
  NONE,
  SCAN,
  STRESS,
  STRESS_MIX,
};

struct SessionState {
  SessionKind kind = SessionKind::NONE;
  uint32_t requested = 0;
  uint32_t completed = 0;
  uint32_t successes = 0;
  uint32_t failures = 0;
  uint32_t probeCount = 0;
  uint32_t reconcileCount = 0;
  uint32_t sampleCount = 0;
  uint32_t transportSuccessesBefore = 0;
  uint32_t transportFailuresBefore = 0;
  uint64_t startedMs = 0;
  uint64_t lastSequence = 0;
  uint32_t configGeneration = 0;
  uint8_t nextScanAddress = SCAN_FIRST_ADDRESS;
  uint8_t found = 0;
  uint8_t availableMask = 0;
  SampleRequest sampleRequest{};
  SampleRequest pendingSampleRequest{};
  JobKind pendingJob = JobKind::NONE;
  bool cancelRequested = false;
  Status firstFailure = Status::Ok();
  Status lastFailure = Status::Ok();
};

SessionState session{};

uint64_t nowMs() {
  const uint32_t current = millis();
  if (current < previousMillis) {
    millisEpoch += (1ULL << 32U);
  }
  previousMillis = current;
  return millisEpoch + current;
}

OperationTiming timing(uint64_t now, uint32_t durationMs) {
  const uint64_t duration = durationMs;
  const uint64_t deadline =
      now > std::numeric_limits<uint64_t>::max() - duration
          ? std::numeric_limits<uint64_t>::max()
          : now + duration;
  return OperationTiming{now, deadline};
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

const char* sessionName(SessionKind kind) {
  switch (kind) {
    case SessionKind::SCAN: return "scan";
    case SessionKind::STRESS: return "stress";
    case SessionKind::STRESS_MIX: return "stress_mix";
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

const char* sampleQualityName(SampleQuality quality) {
  switch (quality) {
    case SampleQuality::READY_CHECKED: return "ready_checked";
    case SampleQuality::DIRECT_UNVERIFIED: return "direct_unverified";
    case SampleQuality::CONFIG_UNKNOWN: return "config_unknown";
    case SampleQuality::SETTLING: return "settling";
    default: return "invalid";
  }
}

void printProfileRecord(const char* label, const DeviceProfile& value) {
  Serial.printf(
      "profile %s xl_odr=%s xl_fs_g=%s xl_power=%s xl_lpf2=%s "
      "xl_slope_hp=%s xl_6d_lpf=%s\n",
      label, profile_cli::odrName(value.accelOdr),
      profile_cli::accelFsName(value.accelFullScale),
      profile_cli::accelPowerName(value.accelPowerMode),
      profile_cli::boolName(value.accelFilter.lpf2Enabled),
      profile_cli::boolName(value.accelFilter.highPassSlopeEnabled),
      profile_cli::boolName(value.accelFilter.lowPassOn6d));
  Serial.printf(
      "profile %s g_odr=%s g_fs_dps=%s g_power=%s g_lpf1=%s g_hpf=%s g_hpf_mode=%s "
      "g_sleep=%s\n",
      label, profile_cli::odrName(value.gyroOdr),
      profile_cli::gyroFsName(value.gyroFullScale),
      profile_cli::gyroPowerName(value.gyroPowerMode),
      profile_cli::boolName(value.gyroFilter.lpf1Enabled),
      profile_cli::boolName(value.gyroFilter.highPassEnabled),
      profile_cli::gyroHpfModeName(value.gyroFilter.highPassMode),
      profile_cli::boolName(value.gyroSleepEnabled));
  Serial.printf(
      "profile %s bdu=%s offset_weight_mg=%s offset_x=%d offset_y=%d "
      "offset_z=%d fifo=%s interrupts=%s\n",
      label, profile_cli::boolName(value.blockDataUpdate),
      profile_cli::offsetWeightName(value.accelOffsetWeight),
      static_cast<int>(value.accelUserOffset.x),
      static_cast<int>(value.accelUserOffset.y),
      static_cast<int>(value.accelUserOffset.z),
      profile_cli::boolName(value.fifo.enabled),
      profile_cli::boolName(value.interrupts.enabled));
}

void printProfiles(uint64_t now) {
  const Status stagedStatus = validateProfile(stagedProfile);
  Serial.printf("profile staged valid=%s code=%u detail=%ld message=%s\n",
                stagedStatus.ok() ? "yes" : "no",
                static_cast<unsigned>(stagedStatus.code),
                static_cast<long>(stagedStatus.detail), stagedStatus.msg);
  printProfileRecord("staged", stagedProfile);

  DeviceProfile desired{};
  const Status desiredStatus = device.getDesiredProfile(desired);
  Serial.printf("profile desired available=%s matches_staged=%s code=%u message=%s\n",
                desiredStatus.ok() ? "yes" : "no",
                desiredStatus.ok() && profile_cli::equal(desired, stagedProfile)
                    ? "yes"
                    : "no",
                static_cast<unsigned>(desiredStatus.code), desiredStatus.msg);
  if (desiredStatus.ok()) printProfileRecord("desired", desired);

  DeviceProfile verified{};
  const Status verifiedStatus = device.getVerifiedProfile(verified, now);
  Serial.printf("profile verified available=%s matches_staged=%s code=%u message=%s\n",
                verifiedStatus.ok() ? "yes" : "no",
                verifiedStatus.ok() && profile_cli::equal(verified, stagedProfile)
                    ? "yes"
                    : "no",
                static_cast<unsigned>(verifiedStatus.code), verifiedStatus.msg);
  if (verifiedStatus.ok()) printProfileRecord("verified", verified);
  Serial.println("profile locked if_inc=on bdu=on byte_order=little fifo=bypass interrupts=off");
}

bool ownerMutationBlocked() {
  return session.kind != SessionKind::NONE || device.operationActive() ||
         device.resultPending();
}

bool requireOwnerIdle(const char* operation) {
  if (!ownerMutationBlocked()) return true;
  Serial.printf("owner_busy operation=%s\n", operation);
  printStatus(Status::Error(Err::BUSY, "Operation is active"));
  return false;
}

uint8_t availableSampleMask(const DeviceProfile& value) {
  uint8_t mask = 0;
  if (value.accelOdr != Odr::POWER_DOWN) mask |= SAMPLE_ACCELERATION;
  if (value.gyroOdr != Odr::POWER_DOWN && !value.gyroSleepEnabled)
    mask |= SAMPLE_ANGULAR_RATE;
  if (value.accelOdr != Odr::POWER_DOWN || value.gyroOdr != Odr::POWER_DOWN)
    mask |= SAMPLE_TEMPERATURE;
  return mask;
}

bool acceptedStart(const Status& status, OperationToken token) {
  if ((!status.ok() && !status.inProgress()) || !token.valid()) {
    printStatus(status);
    return false;
  }
  pendingToken = token;
  lastPoll = {};
  lastPoll.status = status;
  lastPoll.token = token;
  lastPoll.kind = device.activeJob();
  lastPoll.state = OperationState::ACTIVE;
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

  Serial.printf("sample sequence=%" PRIu64 " generation=%lu valid=0x%02X fresh=0x%02X read_ms=%" PRIu64 " quality=%s xl_fs_g=%s g_fs_dps=%s\n",
                converted.sequence,
                static_cast<unsigned long>(converted.configGeneration),
                converted.validMask, converted.freshMask,
                converted.readUptimeMs, sampleQualityName(converted.quality),
                profile_cli::accelFsName(raw.accelFullScale),
                profile_cli::gyroFsName(raw.gyroFullScale));
  Serial.printf("  raw accel=%d,%d,%d gyro=%d,%d,%d temp=%d\n",
                static_cast<int>(raw.accel.x), static_cast<int>(raw.accel.y),
                static_cast<int>(raw.accel.z), static_cast<int>(raw.gyro.x),
                static_cast<int>(raw.gyro.y), static_cast<int>(raw.gyro.z),
                static_cast<int>(raw.temperatureRaw));
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
  return acceptedStart(device.startConfigure(stagedProfile, timing(now, 5000), token), token);
}

void printAxes(const char* label, const Axes& axes) {
  Serial.printf("  %s x=%.6f y=%.6f z=%.6f\n", label, axes.x, axes.y, axes.z);
}

void printTerminalResult(const OperationResult& result) {
  Serial.printf("result token=%" PRIu64 " kind=%s state=%s transactions=%lu/%lu changed=%s started_ms=%" PRIu64 " completed_ms=%" PRIu64 "\n",
                result.token.value, jobName(result.kind),
                operationStateName(result.state),
                static_cast<unsigned long>(result.transactions),
                static_cast<unsigned long>(result.transactionLimit),
                result.hardwareStateMayHaveChanged ? "yes" : "no",
                result.startedUptimeMs, result.completedUptimeMs);
  printStatus(result.status);

  if (result.kind == JobKind::PROBE) {
    Serial.printf("  address=0x%02X who_am_i=0x%02X\n",
                  result.probe.address, result.probe.whoAmI);
  } else if (result.kind == JobKind::CONFIGURE ||
             result.kind == JobKind::RECONCILE ||
             result.kind == JobKind::RECOVER || result.kind == JobKind::RESET ||
             result.kind == JobKind::BOOT || result.kind == JobKind::POWER_DOWN) {
    Serial.printf("  config=%s generation=%lu valid_after_ms=%" PRIu64
                  " mismatch_reg=0x%02X expected=0x%02X observed=0x%02X\n",
                  configurationStateName(result.configuration.state),
                  static_cast<unsigned long>(result.configuration.generation),
                  result.configuration.validAfterUptimeMs,
                  result.configuration.mismatchRegister,
                  result.configuration.expectedValue,
                  result.configuration.observedValue);
  } else if (result.kind == JobKind::SAMPLE && result.status.ok()) {
    printSample(result.sample);
  } else if (result.kind == JobKind::SELF_TEST) {
    printAxes("accel_baseline_g", result.selfTest.accelBaselineG);
    printAxes("accel_stimulus_g", result.selfTest.accelStimulusG);
    printAxes("accel_delta_g", result.selfTest.accelDeltaG);
    printAxes("gyro_baseline_dps", result.selfTest.gyroBaselineDps);
    printAxes("gyro_stimulus_dps", result.selfTest.gyroStimulusDps);
    printAxes("gyro_delta_dps", result.selfTest.gyroDeltaDps);
    Serial.printf("  accel_pass=%s gyro_pass=%s primary_code=%u primary_detail=%ld primary_message=%s restore_code=%u restore_detail=%ld restore_message=%s\n",
                  result.selfTest.accelPass ? "yes" : "no",
                  result.selfTest.gyroPass ? "yes" : "no",
                  static_cast<unsigned>(result.selfTest.primaryStatus.code),
                  static_cast<long>(result.selfTest.primaryStatus.detail),
                  result.selfTest.primaryStatus.msg,
                  static_cast<unsigned>(result.selfTest.restorationStatus.code),
                  static_cast<long>(result.selfTest.restorationStatus.detail),
                  result.selfTest.restorationStatus.msg);
  } else if (result.kind == JobKind::CALIBRATION) {
    Serial.printf("  calibration_kind=%s bias x=%.6f y=%.6f z=%.6f peak_to_peak x=%.6f y=%.6f z=%.6f samples=%u\n",
                  result.calibration.kind == CalibrationKind::ACCELEROMETER_BIAS
                      ? "accel"
                      : "gyro",
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
  // HWCDC writes are timeout-bounded but buffered. Complete one immutable
  // terminal record before the owner accepts more console work.
  Serial.flush();
}

void recordSessionFailure(const Status& status) {
  if (session.failures == 0U) session.firstFailure = status;
  session.lastFailure = status;
  session.failures++;
}

void finishSession(uint64_t now, bool cancelled, bool aborted) {
  const SessionState finished = session;
  session = {};
  const DriverDiagnostics diag = device.diagnostics(now);
  const uint32_t okDelta = diag.transportSuccesses >= finished.transportSuccessesBefore
                               ? diag.transportSuccesses - finished.transportSuccessesBefore
                               : 0U;
  const uint32_t failDelta = diag.transportFailures >= finished.transportFailuresBefore
                                 ? diag.transportFailures - finished.transportFailuresBefore
                                 : 0U;
  if (finished.kind == SessionKind::SCAN) {
    Serial.printf("scan summary attempted=%lu found=%u failures=%lu cancelled=%s elapsed_ms=%" PRIu64 "\n",
                  static_cast<unsigned long>(finished.completed), finished.found,
                  static_cast<unsigned long>(finished.failures),
                  cancelled ? "yes" : "no", now - finished.startedMs);
  } else {
    Serial.printf("%s summary requested=%lu completed=%lu ok=%lu fail=%lu cancelled=%s aborted=%s elapsed_ms=%" PRIu64
                  " transport_ok_delta=%lu transport_fail_delta=%lu probes=%lu reconciles=%lu samples=%lu\n",
                  sessionName(finished.kind),
                  static_cast<unsigned long>(finished.requested),
                  static_cast<unsigned long>(finished.completed),
                  static_cast<unsigned long>(finished.successes),
                  static_cast<unsigned long>(finished.failures),
                  cancelled ? "yes" : "no", aborted ? "yes" : "no",
                  now - finished.startedMs, static_cast<unsigned long>(okDelta),
                  static_cast<unsigned long>(failDelta),
                  static_cast<unsigned long>(finished.probeCount),
                  static_cast<unsigned long>(finished.reconcileCount),
                  static_cast<unsigned long>(finished.sampleCount));
  }
  if (finished.failures != 0U) {
    Serial.printf("session first_error code=%u detail=%ld message=%s\n",
                  static_cast<unsigned>(finished.firstFailure.code),
                  static_cast<long>(finished.firstFailure.detail),
                  finished.firstFailure.msg);
    Serial.printf("session last_error code=%u detail=%ld message=%s\n",
                  static_cast<unsigned>(finished.lastFailure.code),
                  static_cast<long>(finished.lastFailure.detail),
                  finished.lastFailure.msg);
  }
  Serial.flush();
}

bool acceptSessionStart(const Status& status, OperationToken token,
                        JobKind expectedJob, const SampleRequest& request) {
  if ((!status.ok() && !status.inProgress()) || !token.valid()) {
    recordSessionFailure(status);
    return false;
  }
  pendingToken = token;
  lastPoll = {};
  lastPoll.status = status;
  lastPoll.token = token;
  lastPoll.kind = expectedJob;
  lastPoll.state = OperationState::ACTIVE;
  session.pendingJob = expectedJob;
  session.pendingSampleRequest = request;
  return true;
}

void startNextSessionOperation(uint64_t now) {
  if (session.kind != SessionKind::STRESS &&
      session.kind != SessionKind::STRESS_MIX) return;
  if (session.completed >= session.requested) {
    finishSession(now, false, false);
    return;
  }

  OperationToken token{};
  Status status = Status::Error(Err::INVALID_PARAM, "Invalid session operation");
  JobKind expected = JobKind::NONE;
  SampleRequest request{};
  if (session.kind == SessionKind::STRESS) {
    expected = JobKind::SAMPLE;
    request = session.sampleRequest;
    status = device.startSample(request, timing(now, 2000), token);
  } else {
    switch (session.completed % 4U) {
      case 0:
        expected = JobKind::PROBE;
        status = device.startProbe(timing(now, 500), token);
        break;
      case 1:
        expected = JobKind::RECONCILE;
        status = device.startReconcile(timing(now, 3000), token);
        break;
      case 2:
      case 3:
        expected = JobKind::SAMPLE;
        request.quantityMask = session.availableMask;
        request.checkDataReady = (session.completed % 4U) == 2U;
        status = device.startSample(request, timing(now, 2000), token);
        break;
      default:
        break;
    }
  }
  if (!acceptSessionStart(status, token, expected, request)) {
    session.completed++;
    finishSession(now, false, true);
  }
}

Status validateSessionResult(const OperationResult& result) {
  if (result.kind != session.pendingJob)
    return Status::Error(Err::STALE_RESULT, "Session job kind mismatch");
  if (result.state != OperationState::SUCCEEDED)
    return result.status.ok()
               ? Status::Error(Err::OPERATION_INDETERMINATE,
                               "Session job did not succeed")
               : result.status;
  if (result.hardwareStateMayHaveChanged)
    return Status::Error(Err::OPERATION_INDETERMINATE,
                         "Non-destructive session reported hardware change");
  if (result.transactions == 0U)
    return Status::Error(Err::OPERATION_INDETERMINATE,
                         "Session job reported zero transactions");
  if (result.transactions > result.transactionLimit)
    return Status::Error(Err::TRANSACTION_LIMIT_EXCEEDED,
                         "Session transaction ceiling exceeded");
  if (!result.status.ok()) return result.status;
  if (result.kind != JobKind::SAMPLE) return Status::Ok();
  if (result.sample.validMask != session.pendingSampleRequest.quantityMask)
    return Status::Error(Err::INVALID_PARAM, "Session sample validity mismatch");
  const uint8_t expectedFresh = session.pendingSampleRequest.checkDataReady
                                    ? session.pendingSampleRequest.quantityMask
                                    : 0U;
  if (result.sample.freshMask != expectedFresh)
    return Status::Error(Err::INVALID_PARAM, "Session sample freshness mismatch");
  if (result.sample.configGeneration != session.configGeneration)
    return Status::Error(Err::CONFIGURATION_MISMATCH,
                         "Session configuration generation changed");
  if (session.lastSequence != 0U && result.sample.sequence <= session.lastSequence)
    return Status::Error(Err::INVALID_PARAM, "Session sample sequence did not advance");
  ConvertedSample converted{};
  const Status convertedStatus = convertSample(result.sample, converted);
  if (!convertedStatus.ok()) return convertedStatus;
  session.lastSequence = result.sample.sequence;
  return Status::Ok();
}

void consumeSessionResult(const OperationResult& result, uint64_t now) {
  if (session.cancelRequested) {
    finishSession(now, true, false);
    return;
  }
  const Status status = validateSessionResult(result);
  session.completed++;
  if (result.kind == JobKind::PROBE) session.probeCount++;
  if (result.kind == JobKind::RECONCILE) session.reconcileCount++;
  if (result.kind == JobKind::SAMPLE) session.sampleCount++;
  if (status.ok()) {
    session.successes++;
  } else {
    recordSessionFailure(status);
  }
  session.pendingJob = JobKind::NONE;
  const uint32_t progressStep = session.requested < 10U ? 1U : session.requested / 10U;
  if (session.completed == session.requested ||
      (session.completed % progressStep) == 0U) {
    Serial.printf("%s progress completed=%lu/%lu ok=%lu fail=%lu\n",
                  sessionName(session.kind),
                  static_cast<unsigned long>(session.completed),
                  static_cast<unsigned long>(session.requested),
                  static_cast<unsigned long>(session.successes),
                  static_cast<unsigned long>(session.failures));
  }
  if (session.completed >= session.requested) finishSession(now, false, false);
}

void serviceScan(uint64_t now) {
  if (session.kind != SessionKind::SCAN) return;
  const uint8_t address = session.nextScanAddress;
  const Status status =
      transport::wireProbe(Wire, address, board::I2C_TIMEOUT_MS);
  const bool ack = status.ok();
  if (ack) {
    session.found++;
  } else if (!status.is(Err::I2C_NACK_ADDR)) {
    recordSessionFailure(status);
  }
  Serial.printf("scan address=0x%02X ack=%s code=%u detail=%ld message=%s\n",
                address, ack ? "yes" : "no",
                static_cast<unsigned>(status.code),
                static_cast<long>(status.detail), status.msg);
  session.completed++;
  if (address >= SCAN_LAST_ADDRESS) {
    finishSession(now, false, false);
  } else {
    session.nextScanAddress++;
  }
}

void serviceOperation(uint64_t now) {
  if (session.kind == SessionKind::SCAN) {
    serviceScan(now);
    return;
  }
  if (device.operationActive()) {
    lastPoll = device.poll(now, POLL_TRANSACTION_BUDGET);
  }
  if (device.resultPending() && pendingToken.valid()) {
    OperationResult result{};
    const Status takeStatus = device.takeResult(pendingToken, result);
    if (!takeStatus.ok()) {
      printStatus(takeStatus);
      pendingToken = {};
      configureAfterProbe = false;
      if (session.kind != SessionKind::NONE) {
        recordSessionFailure(takeStatus);
        finishSession(now, false, true);
      }
      return;
    }
    pendingToken = {};
    lastResult = result;
    lastResultAvailable = true;
    if (session.kind == SessionKind::STRESS ||
        session.kind == SessionKind::STRESS_MIX) {
      consumeSessionResult(result, now);
    } else {
      printTerminalResult(result);
      if (configureAfterProbe && result.kind == JobKind::PROBE) {
        configureAfterProbe = false;
        if (result.status.ok()) (void)startDefaultConfigure(now);
      }
    }
  }
  if (!device.operationActive() && !device.resultPending() &&
      !pendingToken.valid() &&
      (session.kind == SessionKind::STRESS ||
       session.kind == SessionKind::STRESS_MIX)) {
    startNextSessionOperation(now);
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

bool parseFloat(const char* text, float minimum, float maximum, float& out) {
  if (text == nullptr || *text == '\0') return false;
  char* end = nullptr;
  const float value = std::strtof(text, &end);
  if (*end != '\0' || !std::isfinite(value) || value < minimum ||
      value > maximum) return false;
  out = value;
  return true;
}

void printHelp() {
  Serial.println("Lifecycle: help/? version/ver bind unbind cancel status diag job [current|last] result");
  Serial.println("Bus owner: scan addr [0x6a|0x6b] freq [100000|400000]");
  Serial.println("Owner-safe: probe configure sample [all|accel|gyro|temp] [ready|direct]");
  Serial.println("Owner-safe: reset boot recover reconcile powerdown selftest [5..100]");
  Serial.println("Profile: profile [show|validate|defaults|apply] | cfg | settings");
  Serial.println("Profile: profile set <field> <value...>");
  Serial.println("Fields: xl_odr xl_fs xl_power xl_lpf2 g_odr g_fs g_power g_lpf1 g_sleep");
  Serial.println("Fields: offset_weight offset <x -127..127> <y> <z>");
  Serial.println("Fields: g_hpf_mode 0.016|0.065|0.260|1.040 (mode is staged while HPF stays locked off)");
  Serial.println("Values: xl_odr=pd|1.6|12.5|26|52|104|208|416|833|1660|3330|6660");
  Serial.println("Values: g_odr=pd|12.5|26|52|104|208|416|833|1660|3330|6660");
  Serial.println("Values: xl_fs=2|4|8|16 g_fs=125|250|500|1000|2000 xl_power/g_power=hp|lp");
  Serial.println("Values: offset_weight=1|16; offset components=-127..127");
  Serial.println("Invariant probes: xl_slope_hp xl_6d_lpf g_hpf bdu fifo interrupts");
  Serial.println("Booleans: 0/1, off/on, or false/true; unsupported production values are rejected.");
  Serial.println("Stress: stress [1..10000] [all|accel|gyro|temp] [ready|direct]");
  Serial.println("Stress: stress_mix [1..10000] (probe/reconcile/ready/direct, non-destructive)");
  Serial.println("Maintenance: calxl [samples [x y z [max_p2p_g]]] calg [samples [max_p2p_dps]]");
  Serial.println("Maintenance: purge <1..2048> rreg <reg> wreg <reg> <value> dump <reg> <1..32>");
  Serial.println("All jobs use absolute deadlines and one transport callback per loop poll.");
  Serial.println("scan checks ACK only at 0x6A/0x6B; probe validates WHO_AM_I. Profile edits are staged until apply.");
}

uint32_t calibrationTimeoutMs(const CalibrationRequest& request,
                              const DeviceProfile& verified) {
  const Odr odr = request.kind == CalibrationKind::ACCELEROMETER_BIAS
                      ? verified.accelOdr
                      : verified.gyroOdr;
  const uint64_t periodMs = (odrPeriodUs(odr) + 999U) / 1000U;
  const uint64_t worstCaseMs =
      periodMs * static_cast<uint64_t>(request.samples) * 3U + 5000U;
  return worstCaseMs > UINT32_MAX ? UINT32_MAX
                                  : static_cast<uint32_t>(worstCaseMs);
}

Status bindDriver() {
  if (!busReady)
    return Status::Error(Err::INVALID_CONFIG, "Example I2C bus is not initialized");
  DriverConfig config{};
  config.i2cWrite = transport::wireWrite;
  config.i2cWriteRead = transport::wireWriteRead;
  config.i2cUser = &Wire;
  config.address = selectedAddress;
  config.i2cTimeoutMs = board::I2C_TIMEOUT_MS;
  return device.bind(config);
}

void printJob(uint64_t now, bool includeLast) {
  Serial.printf("job session=%s requested=%lu completed=%lu ok=%lu fail=%lu cancel_requested=%s\n",
                sessionName(session.kind),
                static_cast<unsigned long>(session.requested),
                static_cast<unsigned long>(session.completed),
                static_cast<unsigned long>(session.successes),
                static_cast<unsigned long>(session.failures),
                session.cancelRequested ? "yes" : "no");
  Serial.printf("job driver bound=%s active=%s result_pending=%s token=%" PRIu64 " kind=%s\n",
                device.isBound() ? "yes" : "no",
                device.operationActive() ? "yes" : "no",
                device.resultPending() ? "yes" : "no",
                device.activeToken().value, jobName(device.activeJob()));
  Serial.printf("job poll token=%" PRIu64 " kind=%s state=%s transactions=%u/%u used=%u waiting=%s code=%u detail=%ld message=%s now_ms=%" PRIu64 "\n",
                lastPoll.token.value, jobName(lastPoll.kind),
                operationStateName(lastPoll.state), lastPoll.transactions,
                lastPoll.transactionLimit, lastPoll.transactionsUsed,
                lastPoll.waiting ? "yes" : "no",
                static_cast<unsigned>(lastPoll.status.code),
                static_cast<long>(lastPoll.status.detail), lastPoll.status.msg, now);
  if (includeLast) {
    Serial.printf("job last available=%s\n", lastResultAvailable ? "yes" : "no");
    if (lastResultAvailable) printTerminalResult(lastResult);
  }
}

void printDiagnostics(uint64_t now) {
  const DriverDiagnostics diag = device.diagnostics(now);
  const uint64_t settleRemaining =
      diag.configurationState == ConfigurationState::SETTLING &&
              diag.validAfterUptimeMs > now
          ? diag.validAfterUptimeMs - now
          : 0U;
  const bool haveTransportError = diag.transportFailures != 0U;
  const uint64_t errorAge =
      haveTransportError && now >= diag.lastTransportErrorUptimeMs
          ? now - diag.lastTransportErrorUptimeMs
          : 0U;
  Serial.printf("bus ready=%s selected_address=0x%02X bound_address=%s frequency_hz=%lu timeout_ms=%u\n",
                busReady ? "yes" : "no",
                static_cast<unsigned>(selectedAddress),
                device.isBound()
                    ? (selectedAddress == SensorAddress::SA0_GND ? "0x6A" : "0x6B")
                    : "none",
                static_cast<unsigned long>(selectedFrequencyHz),
                board::I2C_TIMEOUT_MS);
  Serial.printf("bound=%s active=%s result_pending=%s config=%s generation=%lu valid_after=%" PRIu64 " settle_remaining_ms=%" PRIu64 "\n",
                device.isBound() ? "yes" : "no",
                device.operationActive() ? "yes" : "no",
                device.resultPending() ? "yes" : "no",
                configurationStateName(diag.configurationState),
                static_cast<unsigned long>(diag.configGeneration),
                diag.validAfterUptimeMs, settleRemaining);
  Serial.printf("transport ok=%lu fail=%lu last_error=%u\n",
                static_cast<unsigned long>(diag.transportSuccesses),
                static_cast<unsigned long>(diag.transportFailures),
                static_cast<unsigned>(diag.lastTransportError.code));
  Serial.printf("last_error present=%s code=%u detail=%ld message=%s time_ms=%" PRIu64 " age_ms=%" PRIu64 "\n",
                haveTransportError ? "yes" : "no",
                static_cast<unsigned>(diag.lastTransportError.code),
                static_cast<long>(diag.lastTransportError.detail),
                diag.lastTransportError.msg, diag.lastTransportErrorUptimeMs,
                errorAge);
  Serial.printf("mismatch present=%s register=0x%02X expected=0x%02X observed=0x%02X\n",
                diag.mismatchRegister != 0U ? "yes" : "no",
                diag.mismatchRegister, diag.mismatchExpected,
                diag.mismatchObserved);
  printJob(now, false);
  printProfiles(now);
}

bool tokenize(char* line, char** tokens, size_t& count) {
  count = 0;
  char* save = nullptr;
  for (char* token = strtok_r(line, " \t", &save); token != nullptr;
       token = strtok_r(nullptr, " \t", &save)) {
    if (count >= MAX_COMMAND_TOKENS) return false;
    tokens[count++] = token;
  }
  return true;
}

bool parseSampleArguments(char* const* tokens, size_t count, size_t first,
                          SampleRequest& request) {
  if (count > first + 2U) return false;
  const char* quantity = count > first ? tokens[first] : "all";
  const char* mode = count > first + 1U ? tokens[first + 1U] : "ready";
  if (strcmp(quantity, "all") == 0) request.quantityMask = SAMPLE_ALL;
  else if (strcmp(quantity, "accel") == 0) request.quantityMask = SAMPLE_ACCELERATION;
  else if (strcmp(quantity, "gyro") == 0) request.quantityMask = SAMPLE_ANGULAR_RATE;
  else if (strcmp(quantity, "temp") == 0) request.quantityMask = SAMPLE_TEMPERATURE;
  else return false;
  if (strcmp(mode, "ready") == 0) request.checkDataReady = true;
  else if (strcmp(mode, "direct") == 0) request.checkDataReady = false;
  else return false;
  return true;
}

void startSession(SessionKind kind, uint32_t count, const SampleRequest& request,
                  const DeviceProfile& verified, uint64_t now) {
  const DriverDiagnostics diag = device.diagnostics(now);
  session = {};
  session.kind = kind;
  session.requested = count;
  session.sampleRequest = request;
  session.availableMask = availableSampleMask(verified);
  session.configGeneration = diag.configGeneration;
  session.transportSuccessesBefore = diag.transportSuccesses;
  session.transportFailuresBefore = diag.transportFailures;
  session.startedMs = now;
  Serial.printf("%s started requested=%lu mask=0x%02X mode=%s generation=%lu\n",
                sessionName(kind), static_cast<unsigned long>(count),
                kind == SessionKind::STRESS ? request.quantityMask
                                            : session.availableMask,
                kind == SessionKind::STRESS_MIX
                    ? "mixed"
                    : (request.checkDataReady ? "ready" : "direct"),
                static_cast<unsigned long>(session.configGeneration));
}

void handleCommand(char* line) {
  char* tokens[MAX_COMMAND_TOKENS]{};
  size_t count = 0;
  if (!tokenize(line, tokens, count)) {
    Serial.println("too many command tokens");
    return;
  }
  if (count == 0U) return;
  const char* command = tokens[0];
  const uint64_t now = nowMs();

  if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0) {
    if (count != 1U) { Serial.println("expected help"); return; }
    printHelp();
  } else if (strcmp(command, "version") == 0 || strcmp(command, "ver") == 0) {
    if (count != 1U) { Serial.println("expected version"); return; }
    Serial.printf("LSM6DS3TR %s\n", VERSION_FULL);
    Serial.printf("platform arduino=%s esp-idf=%s flash_bytes=%lu psram_bytes=%lu\n",
                  ESP.getCoreVersion(), ESP.getSdkVersion(),
                  static_cast<unsigned long>(ESP.getFlashChipSize()),
                  static_cast<unsigned long>(ESP.getPsramSize()));
  } else if (strcmp(command, "status") == 0 || strcmp(command, "diag") == 0) {
    if (count != 1U) { Serial.printf("expected %s\n", command); return; }
    printDiagnostics(now);
  } else if (strcmp(command, "job") == 0) {
    if (count > 2U ||
        (count == 2U && strcmp(tokens[1], "current") != 0 &&
         strcmp(tokens[1], "last") != 0)) {
      Serial.println("expected job [current|last]");
      return;
    }
    printJob(now, count == 2U && strcmp(tokens[1], "last") == 0);
  } else if (strcmp(command, "result") == 0) {
    if (count != 1U) { Serial.println("expected result"); return; }
    printJob(now, true);
  } else if (strcmp(command, "bind") == 0) {
    if (count != 1U) { Serial.println("expected bind"); return; }
    if (device.isBound()) {
      printStatus(Status::Ok());
      Serial.println("bound unchanged (zero I2C; configuration preserved)");
    } else if (requireOwnerIdle("bind")) {
      const Status status = bindDriver();
      printStatus(status);
      if (status.ok()) {
        lastResult = {};
        lastResultAvailable = false;
      }
    }
  } else if (strcmp(command, "unbind") == 0) {
    if (count != 1U) { Serial.println("expected unbind"); return; }
    if (session.kind != SessionKind::NONE)
      Serial.printf("hard teardown session=%s (zero I2C; no terminal result)\n",
                    sessionName(session.kind));
    session = {};
    device.unbind();
    pendingToken = {};
    configureAfterProbe = false;
    lastPoll = {};
    lastResult = {};
    lastResultAvailable = false;
    Serial.println("unbound (zero I2C)");
    Serial.println("hard teardown active/pending state discarded");
  } else if (strcmp(command, "cancel") == 0) {
    if (count != 1U) { Serial.println("expected cancel"); return; }
    if (session.kind == SessionKind::SCAN) {
      finishSession(now, true, false);
    } else if (session.kind != SessionKind::NONE) {
      session.cancelRequested = true;
      if (device.operationActive()) printStatus(device.cancelActiveJob(now));
      else if (!device.resultPending()) finishSession(now, true, false);
    } else {
      printStatus(device.cancelActiveJob(now));
    }
  } else if (strcmp(command, "scan") == 0) {
    if (count != 1U) { Serial.println("expected scan"); return; }
    if (!requireOwnerIdle("scan")) return;
    if (!busReady) {
      printStatus(Status::Error(Err::INVALID_CONFIG,
                                "Example I2C bus is not initialized"));
      return;
    }
    session = {};
    session.kind = SessionKind::SCAN;
    session.requested = SCAN_LAST_ADDRESS - SCAN_FIRST_ADDRESS + 1U;
    session.nextScanAddress = SCAN_FIRST_ADDRESS;
    session.startedMs = now;
    Serial.println("scan started range=0x6A..0x6B ack_only=yes");
  } else if (strcmp(command, "addr") == 0) {
    if (count == 1U) {
      Serial.printf("address selected=0x%02X bound=%s\n",
                    static_cast<unsigned>(selectedAddress),
                    device.isBound() ? "yes" : "no");
      return;
    }
    uint32_t address = 0;
    if (count != 2U || !parseUnsigned(tokens[1], 0x7F, address) ||
        (address != 0x6AU && address != 0x6BU)) {
      Serial.println("expected addr [0x6a|0x6b]");
      return;
    }
    const SensorAddress candidate = static_cast<SensorAddress>(address);
    if (candidate == selectedAddress) {
      Serial.println("address unchanged (zero I2C; configuration preserved)");
      return;
    }
    if (!requireOwnerIdle("addr")) return;
    const SensorAddress previous = selectedAddress;
    const bool wasBound = device.isBound();
    if (wasBound) {
      device.unbind();
      lastResult = {};
      lastResultAvailable = false;
    }
    selectedAddress = candidate;
    Status status = wasBound ? bindDriver() : Status::Ok();
    if (!status.ok()) {
      selectedAddress = previous;
      const Status rollback = wasBound ? bindDriver() : Status::Ok();
      Serial.printf("address change rollback_code=%u rollback_message=%s\n",
                    static_cast<unsigned>(rollback.code), rollback.msg);
    }
    printStatus(status);
    if (status.ok()) {
      lastPoll = {};
      lastResult = {};
      lastResultAvailable = false;
      Serial.printf("address selected=0x%02lX rebound=%s profile_apply_required=%s\n",
                    static_cast<unsigned long>(address), wasBound ? "yes" : "no",
                    wasBound ? "yes" : "no");
    }
  } else if (strcmp(command, "freq") == 0) {
    if (count == 1U) {
      Serial.printf("frequency_hz=%lu bus_ready=%s\n",
                    static_cast<unsigned long>(selectedFrequencyHz),
                    busReady ? "yes" : "no");
      return;
    }
    uint32_t frequency = 0;
    if (count != 2U || !parseUnsigned(tokens[1], 400000U, frequency) ||
        (frequency != 100000U && frequency != 400000U)) {
      Serial.println("expected freq [100000|400000]");
      return;
    }
    if (frequency == selectedFrequencyHz) {
      if (!busReady) {
        printStatus(Status::Error(Err::INVALID_CONFIG,
                                  "Example I2C bus is not initialized"));
        return;
      }
      Serial.println("frequency unchanged (configuration preserved)");
      return;
    }
    if (!requireOwnerIdle("freq")) return;
    if (!busReady) {
      printStatus(Status::Error(Err::INVALID_CONFIG,
                                "Example I2C bus is not initialized"));
      return;
    }
    const Status status = transport::setWireFrequency(Wire, frequency);
    printStatus(status);
    if (status.ok()) {
      selectedFrequencyHz = frequency;
      Serial.printf("frequency_hz=%lu configuration_preserved=yes\n",
                    static_cast<unsigned long>(selectedFrequencyHz));
    }
  } else if (strcmp(command, "profile") == 0 || strcmp(command, "cfg") == 0 ||
             strcmp(command, "settings") == 0) {
    if (strcmp(command, "profile") != 0) {
      if (count != 1U) { Serial.printf("expected %s\n", command); return; }
      printProfiles(now);
      return;
    }
    if (count == 1U || (count == 2U && strcmp(tokens[1], "show") == 0)) {
      printProfiles(now);
    } else if (count == 2U && strcmp(tokens[1], "validate") == 0) {
      printStatus(validateProfile(stagedProfile));
    } else if (count == 2U && strcmp(tokens[1], "defaults") == 0) {
      if (!requireOwnerIdle("profile defaults")) return;
      stagedProfile = DeviceProfile{};
      Serial.println("profile defaults staged configure_required=yes");
    } else if (count == 2U && strcmp(tokens[1], "apply") == 0) {
      if (!requireOwnerIdle("profile apply")) return;
      (void)startDefaultConfigure(now);
    } else if (count >= 4U && strcmp(tokens[1], "set") == 0) {
      if (!requireOwnerIdle("profile set")) return;
      const char* values[MAX_COMMAND_TOKENS]{};
      for (size_t index = 3U; index < count; ++index)
        values[index - 3U] = tokens[index];
      const Status status = profile_cli::setField(
          stagedProfile, tokens[2], values, count - 3U);
      printStatus(status);
      if (status.ok())
        Serial.printf("profile updated field=%s configure_required=yes\n", tokens[2]);
    } else {
      Serial.println("expected profile [show|validate|defaults|apply|set <field> <value...>]");
    }
  } else if (strcmp(command, "probe") == 0) {
    if (count != 1U) { Serial.println("expected probe"); return; }
    if (!requireOwnerIdle("probe")) return;
    OperationToken token{};
    (void)acceptedStart(device.startProbe(timing(now, 500), token), token);
  } else if (strcmp(command, "configure") == 0) {
    if (count != 1U) { Serial.println("expected configure"); return; }
    if (!requireOwnerIdle("configure")) return;
    (void)startDefaultConfigure(now);
  } else if (strcmp(command, "sample") == 0) {
    SampleRequest request{};
    if (!parseSampleArguments(tokens, count, 1U, request)) {
      Serial.println("expected sample [all|accel|gyro|temp] [ready|direct]");
      return;
    }
    if (!requireOwnerIdle("sample")) return;
    OperationToken token{};
    (void)acceptedStart(device.startSample(request, timing(now, 1500), token), token);
  } else if (strcmp(command, "reset") == 0 || strcmp(command, "boot") == 0 ||
             strcmp(command, "recover") == 0 || strcmp(command, "reconcile") == 0 ||
             strcmp(command, "powerdown") == 0) {
    if (count != 1U) { Serial.printf("expected %s\n", command); return; }
    if (!requireOwnerIdle(command)) return;
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
    if (count > 2U ||
        (count == 2U && (!parseUnsigned(tokens[1], 100, samples) || samples < 5U))) {
      Serial.println("expected selftest [5..100]");
      return;
    }
    if (!requireOwnerIdle("selftest")) return;
    Serial.println("selftest requires a stationary fixture; both sensor BIST paths will run");
    SelfTestRequest request{static_cast<uint16_t>(samples)};
    OperationToken token{};
    (void)acceptedStart(device.startSelfTest(request, timing(now, 20000), token), token);
  } else if (strcmp(command, "calxl") == 0 || strcmp(command, "calg") == 0) {
    uint32_t samples = 32;
    CalibrationRequest request{};
    request.kind = strcmp(command, "calxl") == 0
                       ? CalibrationKind::ACCELEROMETER_BIAS
                       : CalibrationKind::GYROSCOPE_BIAS;
    if (request.kind == CalibrationKind::ACCELEROMETER_BIAS) {
      request.expectedAccelerationG.z = 1.0f;
      const bool validCount = count == 1U || count == 2U || count == 5U || count == 6U;
      if (!validCount ||
          (count >= 2U && (!parseUnsigned(tokens[1], 1000, samples) || samples == 0U)) ||
          (count >= 5U &&
           (!parseFloat(tokens[2], -16.0f, 16.0f, request.expectedAccelerationG.x) ||
            !parseFloat(tokens[3], -16.0f, 16.0f, request.expectedAccelerationG.y) ||
            !parseFloat(tokens[4], -16.0f, 16.0f, request.expectedAccelerationG.z))) ||
          (count == 6U &&
           !parseFloat(tokens[5], 0.000001f, 4.0f,
                       request.limits.accelMaxPeakToPeakG))) {
        Serial.println("expected calxl [samples [x y z [max_p2p_g]]]");
        return;
      }
    } else {
      if (count > 3U ||
          (count >= 2U && (!parseUnsigned(tokens[1], 1000, samples) || samples == 0U)) ||
          (count == 3U &&
           !parseFloat(tokens[2], 0.000001f, 2000.0f,
                       request.limits.gyroMaxPeakToPeakDps))) {
        Serial.println("expected calg [samples [max_p2p_dps]]");
        return;
      }
    }
    request.samples = static_cast<uint16_t>(samples);
    if (!requireOwnerIdle(command)) return;
    DeviceProfile verified{};
    const Status verifiedStatus = device.getVerifiedProfile(verified, now);
    if (!verifiedStatus.ok()) { printStatus(verifiedStatus); return; }
    OperationToken token{};
    (void)acceptedStart(device.startCalibration(
                            request,
                            timing(now, calibrationTimeoutMs(request, verified)),
                            token),
                        token);
  } else if (strcmp(command, "purge") == 0) {
    uint32_t words = 0;
    if (count != 2U || !parseUnsigned(tokens[1], 2048, words) || words == 0U) {
      Serial.println("expected purge <1..2048>");
      return;
    }
    if (!requireOwnerIdle("purge")) return;
    FifoPurgeRequest request{static_cast<uint16_t>(words)};
    OperationToken token{};
    (void)acceptedStart(device.startFifoPurge(request, timing(now, 5000), token), token);
  } else if (strcmp(command, "rreg") == 0) {
    uint32_t reg = 0;
    if (count != 2U || !parseUnsigned(tokens[1], 0xFF, reg)) {
      Serial.println("expected rreg <0..255>");
      return;
    }
    if (!requireOwnerIdle("rreg")) return;
    uint8_t value = 0;
    const Status status =
        device.diagnosticReadRegister(static_cast<uint8_t>(reg), value, now);
    printStatus(status);
    if (status.ok()) Serial.printf("0x%02lX = 0x%02X\n", static_cast<unsigned long>(reg), value);
  } else if (strcmp(command, "wreg") == 0) {
    uint32_t reg = 0;
    uint32_t value = 0;
    if (count != 3U || !parseUnsigned(tokens[1], 0xFF, reg) ||
        !parseUnsigned(tokens[2], 0xFF, value)) {
      Serial.println("expected wreg <0..255> <0..255>");
      return;
    }
    if (!requireOwnerIdle("wreg")) return;
    printStatus(device.diagnosticWriteRegister(static_cast<uint8_t>(reg),
                                               static_cast<uint8_t>(value), now));
  } else if (strcmp(command, "dump") == 0) {
    uint32_t reg = 0;
    uint32_t length = 0;
    if (count != 3U || !parseUnsigned(tokens[1], 0xFF, reg) ||
        !parseUnsigned(tokens[2], 32, length) || length == 0U) {
      Serial.println("expected dump <0..255> <1..32>");
      return;
    }
    if (!requireOwnerIdle("dump")) return;
    uint8_t bytes[32]{};
    const Status status =
        device.diagnosticReadBlock(static_cast<uint8_t>(reg), bytes, length, now);
    printStatus(status);
    if (status.ok()) {
      for (uint32_t i = 0; i < length; ++i) Serial.printf("%02X%c", bytes[i], i + 1U == length ? '\n' : ' ');
    }
  } else if (strcmp(command, "stress") == 0 ||
             strcmp(command, "stress_mix") == 0) {
    const bool mixed = strcmp(command, "stress_mix") == 0;
    uint32_t requested = 100;
    size_t sampleStart = 1U;
    if (count >= 2U) {
      uint32_t parsedCount = 0;
      if (parseUnsigned(tokens[1], MAX_STRESS_COUNT, parsedCount)) {
        if (parsedCount == 0U) {
          Serial.printf("expected %s [1..10000]%s\n", command,
                        mixed ? "" : " [all|accel|gyro|temp] [ready|direct]");
          return;
        }
        requested = parsedCount;
        sampleStart = 2U;
      } else if (mixed) {
        Serial.printf("expected %s [1..10000]%s\n", command,
                      mixed ? "" : " [all|accel|gyro|temp] [ready|direct]");
        return;
      }
    }
    SampleRequest request{};
    if ((mixed && count > 2U) ||
        (!mixed && !parseSampleArguments(tokens, count, sampleStart, request))) {
      Serial.printf("expected %s [1..10000]%s\n", command,
                    mixed ? "" : " [all|accel|gyro|temp] [ready|direct]");
      return;
    }
    if (!requireOwnerIdle(command)) return;
    DeviceProfile verified{};
    const Status verifiedStatus = device.getVerifiedProfile(verified, now);
    if (!verifiedStatus.ok()) { printStatus(verifiedStatus); return; }
    const uint8_t available = availableSampleMask(verified);
    if ((mixed && available == 0U) ||
        (!mixed && (request.quantityMask & available) != request.quantityMask)) {
      printStatus(Status::Error(Err::INVALID_PARAM,
                                "Stress quantity is powered down or sleeping"));
      return;
    }
    startSession(mixed ? SessionKind::STRESS_MIX : SessionKind::STRESS,
                 requested, request, verified, now);
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
      return;
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
  Serial.printf("LSM6DS3TR owner-safe example %s\n", LSM6DS3TR::VERSION_FULL);
  printHelp();

  const Status busStatus = board::initI2c();
  printStatus(busStatus);
  if (!busStatus.ok()) return;
  busReady = true;
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
  serviceInput();
  serviceOperation(nowMs());
  yield();
}
