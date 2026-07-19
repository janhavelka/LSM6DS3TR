/// @file LSM6DS3TR.h
/// @brief Owner-scheduled, fixed-memory LSM6DS3TR-C driver.
#pragma once

#include <cstddef>
#include <cstdint>

#include "LSM6DS3TR/CommandTable.h"
#include "LSM6DS3TR/Config.h"
#include "LSM6DS3TR/Status.h"
#include "LSM6DS3TR/Version.h"

namespace LSM6DS3TR {

struct RawAxes {
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
};

struct Axes {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct IntegerAxes {
  int64_t x = 0;
  int64_t y = 0;
  int64_t z = 0;
};

/// @brief Identity allocated to one accepted operation. Zero is never issued.
struct OperationToken {
  uint32_t value = 0;

  constexpr bool valid() const { return value != 0; }
};

constexpr bool operator==(OperationToken lhs, OperationToken rhs) {
  return lhs.value == rhs.value;
}

constexpr bool operator!=(OperationToken lhs, OperationToken rhs) {
  return !(lhs == rhs);
}

enum class JobKind : uint8_t {
  NONE,
  PROBE,
  CONFIGURE,
  SAMPLE,
  RESET,
  BOOT,
  RECOVER,
  RECONCILE,
  POWER_DOWN,
  SELF_TEST,
  CALIBRATION,
  FIFO_PURGE
};

enum class OperationState : uint8_t {
  IDLE,
  ACTIVE,
  SUCCEEDED,
  FAILED,
  CANCELLED,
  TIMED_OUT,
  INDETERMINATE
};

enum class ConfigurationState : uint8_t {
  UNCONFIGURED,
  APPLYING,
  KNOWN,
  UNKNOWN,
  SETTLING
};

/// @brief Absolute times in the caller's single monotonic uptime domain.
struct OperationTiming {
  uint64_t nowMs = 0;
  uint64_t deadlineMs = 0;  ///< Must be strictly greater than nowMs.
};

enum class SampleQuantity : uint8_t {
  ACCELERATION = 1U << 0,
  ANGULAR_RATE = 1U << 1,
  TEMPERATURE = 1U << 2
};

constexpr uint8_t sampleMask(SampleQuantity quantity) {
  return static_cast<uint8_t>(quantity);
}

static constexpr uint8_t SAMPLE_ACCELERATION = sampleMask(SampleQuantity::ACCELERATION);
static constexpr uint8_t SAMPLE_ANGULAR_RATE = sampleMask(SampleQuantity::ANGULAR_RATE);
static constexpr uint8_t SAMPLE_TEMPERATURE = sampleMask(SampleQuantity::TEMPERATURE);
static constexpr uint8_t SAMPLE_ALL =
    SAMPLE_ACCELERATION | SAMPLE_ANGULAR_RATE | SAMPLE_TEMPERATURE;

enum class SampleQuality : uint8_t {
  READY_CHECKED,
  DIRECT_UNVERIFIED,
  CONFIG_UNKNOWN,
  SETTLING
};

struct SampleRequest {
  uint8_t quantityMask = SAMPLE_ALL;
  bool checkDataReady = true;
};

/// @brief Atomic raw sample plus the immutable interpretation provenance.
struct RawSampleResult {
  RawAxes accel = {};
  RawAxes gyro = {};
  int16_t temperatureRaw = 0;
  uint8_t validMask = 0;
  uint8_t freshMask = 0;
  SampleQuality quality = SampleQuality::DIRECT_UNVERIFIED;
  uint32_t sequence = 0;
  uint32_t configGeneration = 0;
  uint64_t readUptimeMs = 0;
  AccelFs accelFullScale = AccelFs::G_2;
  GyroFs gyroFullScale = GyroFs::DPS_250;
};

/// @brief Converted fixed-unit representation. Conversion never reads driver state.
struct ConvertedSample {
  IntegerAxes accelMicroG = {};
  IntegerAxes gyroMicroDps = {};
  int32_t temperatureMilliC = 0;
  uint8_t validMask = 0;
  uint8_t freshMask = 0;
  uint32_t sequence = 0;
  uint32_t configGeneration = 0;
  uint64_t readUptimeMs = 0;
};

struct ProbeResult {
  uint8_t address = 0;
  uint8_t whoAmI = 0;
};

struct ConfigurationResult {
  ConfigurationState state = ConfigurationState::UNCONFIGURED;
  uint32_t generation = 0;
  uint64_t validAfterUptimeMs = 0;
  uint8_t mismatchRegister = 0;
  uint8_t expectedValue = 0;
  uint8_t observedValue = 0;
};

struct SelfTestRequest {
  uint16_t samples = 5;  ///< Average count per baseline/stimulus phase, 1..100.
};

struct SelfTestResult {
  Axes accelBaselineG = {};
  Axes accelStimulusG = {};
  Axes accelDeltaG = {};
  Axes gyroBaselineDps = {};
  Axes gyroStimulusDps = {};
  Axes gyroDeltaDps = {};
  bool accelPass = false;
  bool gyroPass = false;
  Status primaryStatus = Status::Ok();
  Status restorationStatus = Status::Ok();
};

enum class CalibrationKind : uint8_t {
  ACCELEROMETER_BIAS,
  GYROSCOPE_BIAS
};

/// @brief Bounded sensor-native calibration request.
///
/// expectedAccelerationG makes mounting/orientation policy explicit; for
/// example, a Z-up fixture supplies {0, 0, 1}. The driver never assumes Z-up.
struct CalibrationRequest {
  CalibrationKind kind = CalibrationKind::GYROSCOPE_BIAS;
  uint16_t samples = 32;  ///< 1..1000.
  Axes expectedAccelerationG = {};
  CalibrationLimits limits = {};
};

struct CalibrationResult {
  CalibrationKind kind = CalibrationKind::GYROSCOPE_BIAS;
  Axes bias = {};
  Axes peakToPeak = {};
  uint16_t samples = 0;
};

struct FifoPurgeRequest {
  uint16_t maxWords = 1;  ///< Maximum destructive FIFO word reads, 1..2048.
};

struct FifoPurgeResult {
  uint16_t initialUnreadWords = 0;
  uint16_t initialPattern = 0;
  uint16_t wordsDiscarded = 0;
  uint16_t finalUnreadWords = 0;
  bool overrunObserved = false;
  bool truncated = false;
};

/// @brief One terminal result. The caller must take it exactly once.
struct OperationResult {
  OperationToken token = {};
  JobKind kind = JobKind::NONE;
  OperationState state = OperationState::IDLE;
  Status status = Status::Ok();
  bool hardwareStateMayHaveChanged = false;
  uint32_t transactions = 0;
  uint32_t transactionLimit = 0;
  uint64_t startedUptimeMs = 0;
  uint64_t completedUptimeMs = 0;
  ProbeResult probe = {};
  ConfigurationResult configuration = {};
  RawSampleResult sample = {};
  SelfTestResult selfTest = {};
  CalibrationResult calibration = {};
  FifoPurgeResult fifoPurge = {};
};

/// @brief Result of one poll call.
struct PollResult {
  OperationToken token = {};
  JobKind kind = JobKind::NONE;
  OperationState state = OperationState::IDLE;
  Status status = Status::Ok();
  uint8_t transactionsUsed = 0;
  bool waiting = false;  ///< True when time/data must advance; no hidden sleep occurs.
};

/// @brief Passive diagnostics. Counters never gate or retry I2C.
struct DriverDiagnostics {
  uint32_t transportSuccesses = 0;
  uint32_t transportFailures = 0;
  Status lastTransportError = Status::Ok();
  uint64_t lastTransportUptimeMs = 0;
  uint32_t configGeneration = 0;
  ConfigurationState configurationState = ConfigurationState::UNCONFIGURED;
  uint64_t validAfterUptimeMs = 0;
  uint8_t mismatchRegister = 0;
  uint8_t mismatchExpected = 0;
  uint8_t mismatchObserved = 0;
};

// Pure, allocation-free validation, timing, and conversion helpers.
static constexpr uint32_t MAX_PROBE_TRANSACTIONS = 1;
static constexpr uint32_t MAX_CONFIGURE_TRANSACTIONS = 67;
static constexpr uint32_t MAX_SAMPLE_TRANSACTIONS = 66;
static constexpr uint32_t MAX_RESET_TRANSACTIONS = 85;
static constexpr uint32_t MAX_RECOVER_TRANSACTIONS = 86;
static constexpr uint32_t MAX_RECONCILE_TRANSACTIONS = 34;
static constexpr uint32_t MAX_POWER_DOWN_TRANSACTIONS = 4;
Status validateDriverConfig(const DriverConfig& config);
Status validateProfile(const DeviceProfile& profile);
uint64_t odrPeriodUs(Odr odr);
uint64_t requiredSettleUs(const DeviceProfile& profile);
uint32_t maximumSelfTestTransactions(uint16_t samples);
uint32_t maximumCalibrationTransactions(uint16_t samples);
uint32_t maximumFifoPurgeTransactions(uint16_t maxWords);
Status accelSensitivityMicroGPerLsb(AccelFs fullScale, int32_t& out);
Status gyroSensitivityMicroDpsPerLsb(GyroFs fullScale, int32_t& out);
Status decodeAcceleration(const RawAxes& raw, AccelFs fullScale, IntegerAxes& outMicroG);
Status decodeAngularRate(const RawAxes& raw, GyroFs fullScale, IntegerAxes& outMicroDps);
int32_t decodeTemperatureMilliC(int16_t raw);
Status convertSample(const RawSampleResult& raw, ConvertedSample& out);
Status validateCalibrationRequest(const CalibrationRequest& request);
Status applyBias(Axes& sample, const Axes& bias);

/// @brief One non-owning driver advanced only by the external bus owner.
///
/// Concurrency contract: all non-const methods, including poll() and diagnostic
/// access, must be serialized by the application. No method is ISR-safe. A
/// normal poll uses at most maxTransactions transport callbacks. Time-only wait
/// stages use zero callbacks. The driver never sleeps, retries a transaction,
/// recovers the bus, owns a task, logs, or allocates dynamically.
class LSM6DS3TR {
public:
  LSM6DS3TR() = default;
  ~LSM6DS3TR() = default;
  LSM6DS3TR(const LSM6DS3TR&) = delete;
  LSM6DS3TR& operator=(const LSM6DS3TR&) = delete;
  LSM6DS3TR(LSM6DS3TR&&) = delete;
  LSM6DS3TR& operator=(LSM6DS3TR&&) = delete;

  /// Validate and copy a non-owning transport binding. Performs zero I2C.
  Status bind(const DriverConfig& config);

  /// Cancel local state, discard an untaken result, and unbind. Performs zero I2C.
  void unbind();

  bool isBound() const { return _bound; }
  bool operationActive() const { return _active; }
  bool resultPending() const { return _resultPending; }
  OperationToken activeToken() const { return _active ? _token : OperationToken{}; }
  JobKind activeJob() const { return _active ? _job : JobKind::NONE; }

  Status startProbe(const OperationTiming& timing, OperationToken& token);
  Status startConfigure(const DeviceProfile& profile, const OperationTiming& timing,
                        OperationToken& token);
  Status startSample(const SampleRequest& request, const OperationTiming& timing,
                     OperationToken& token);
  Status startReset(const OperationTiming& timing, OperationToken& token);
  Status startBoot(const OperationTiming& timing, OperationToken& token);
  Status startRecover(const OperationTiming& timing, OperationToken& token);
  Status startReconcile(const OperationTiming& timing, OperationToken& token);
  Status startPowerDown(const OperationTiming& timing, OperationToken& token);
  Status startSelfTest(const SelfTestRequest& request, const OperationTiming& timing,
                       OperationToken& token);
  Status startCalibration(const CalibrationRequest& request, const OperationTiming& timing,
                          OperationToken& token);
  Status startFifoPurge(const FifoPurgeRequest& request, const OperationTiming& timing,
                        OperationToken& token);

  /// Advance one operation. The budget counts transport callback invocations only.
  PollResult poll(uint64_t nowMs, uint8_t maxTransactions = 1);

  /// Publish a CANCELLED terminal result without I2C.
  Status cancelActiveJob(uint64_t nowMs);

  /// Take the matching terminal result exactly once.
  Status takeResult(OperationToken token, OperationResult& out);

  ConfigurationState configurationState(uint64_t nowMs) const;
  uint32_t configGeneration() const { return _configGeneration; }
  uint64_t validAfterUptimeMs() const { return _validAfterUptimeMs; }
  Status getDesiredProfile(DeviceProfile& out) const;
  Status getVerifiedProfile(DeviceProfile& out, uint64_t nowMs) const;
  DriverDiagnostics diagnostics(uint64_t nowMs) const;

  /// @name Advanced one-transaction diagnostic access
  /// These calls are unavailable during an operation. Reads do not update
  /// production caches, but device-defined read side effects still apply (for
  /// example FIFO consumption or clearing a latched source). Any accepted write
  /// invalidates configuration provenance and prior samples because its hardware
  /// effect may be ambiguous.
  ///@{
  Status diagnosticReadRegister(uint8_t reg, uint8_t& value, uint64_t nowMs);
  Status diagnosticReadBlock(uint8_t startReg, uint8_t* data, size_t length,
                             uint64_t nowMs);
  Status diagnosticWriteRegister(uint8_t reg, uint8_t value, uint64_t nowMs);
  ///@}

private:
  static constexpr uint8_t MANAGED_REGISTER_COUNT = 33;
  static constexpr uint8_t MAX_DIAGNOSTIC_READ = 32;
  static constexpr uint16_t MAX_SELF_TEST_SAMPLES = 100;
  static constexpr uint16_t MAX_CALIBRATION_SAMPLES = 1000;

  Status _start(JobKind kind, const OperationTiming& timing, OperationToken& token);
  PollResult _pollOne(uint64_t nowMs);
  Status _stepProbe(uint64_t nowMs, bool recovery);
  Status _stepConfigure(uint64_t nowMs, bool reconcileOnly);
  Status _stepSample(uint64_t nowMs);
  Status _stepResetBoot(uint64_t nowMs, bool boot, bool recovery);
  Status _stepPowerDown(uint64_t nowMs);
  Status _stepSelfTest(uint64_t nowMs);
  Status _stepCalibration(uint64_t nowMs);
  Status _stepFifoPurge(uint64_t nowMs);
  Status _finish(const Status& status, OperationState state);
  Status _fail(const Status& status);
  void _clearActive();
  void _invalidateConfiguration();
  void _prepareManagedImage(const DeviceProfile& profile);
  void _recordMismatch(uint8_t reg, uint8_t expected, uint8_t observed);
  Status _read(uint8_t reg, uint8_t* data, size_t length, uint64_t nowMs);
  Status _write(uint8_t reg, const uint8_t* data, size_t length, uint64_t nowMs,
                bool mayChangeConfiguration);
  Status _writeByte(uint8_t reg, uint8_t value, uint64_t nowMs,
                    bool mayChangeConfiguration = true);
  Status _checkStart(const OperationTiming& timing) const;
  Status _checkReadyForKnownConfiguration(uint64_t nowMs) const;
  static bool _validDiagnosticRange(uint8_t startReg, size_t length);
  static bool _safeDiagnosticWrite(uint8_t reg, uint8_t value);

  DriverConfig _driverConfig = {};
  bool _bound = false;

  bool _active = false;
  bool _resultPending = false;
  OperationToken _token = {};
  uint32_t _nextToken = 1;
  bool _tokenExhausted = false;
  JobKind _job = JobKind::NONE;
  uint16_t _step = 0;
  uint16_t _substep = 0;
  uint64_t _deadlineMs = 0;
  uint64_t _waitUntilMs = 0;
  uint64_t _pollNowMs = 0;
  uint32_t _operationTransactions = 0;
  uint32_t _operationTransactionLimit = 0;
  bool _transactionUsed = false;
  bool _waiting = false;
  bool _hardwareStateMayHaveChanged = false;
  bool _configurationMayBeUnknown = false;
  ConfigurationState _configurationStateBeforeOperation =
      ConfigurationState::UNCONFIGURED;
  uint64_t _validAfterBeforeOperationMs = 0;

  OperationResult _workingResult = {};
  OperationResult _terminalResult = {};

  DeviceProfile _desiredProfile = {};
  DeviceProfile _verifiedProfile = {};
  bool _hasDesiredProfile = false;
  bool _hasVerifiedProfile = false;
  ConfigurationState _configurationState = ConfigurationState::UNCONFIGURED;
  uint32_t _configGeneration = 0;
  uint64_t _validAfterUptimeMs = 0;
  uint8_t _managedRegisters[MANAGED_REGISTER_COUNT] = {};
  uint8_t _managedValues[MANAGED_REGISTER_COUNT] = {};
  uint8_t _mismatchRegister = 0;
  uint8_t _mismatchExpected = 0;
  uint8_t _mismatchObserved = 0;

  uint32_t _sampleSequence = 0;
  SampleRequest _sampleRequest = {};
  uint8_t _sampleStatus = 0;

  SelfTestRequest _selfTestRequest = {};
  CalibrationRequest _calibrationRequest = {};
  FifoPurgeRequest _fifoPurgeRequest = {};
  Status _primaryStatus = Status::Ok();
  RawAxes _phaseBaseline = {};
  RawAxes _phaseStimulus = {};
  int64_t _sumX = 0;
  int64_t _sumY = 0;
  int64_t _sumZ = 0;
  RawAxes _rawMin = {};
  RawAxes _rawMax = {};
  uint16_t _samplesDone = 0;
  uint8_t _readyPolls = 0;

  uint32_t _transportSuccesses = 0;
  uint32_t _transportFailures = 0;
  Status _lastTransportError = Status::Ok();
  uint64_t _lastTransportUptimeMs = 0;
};

}  // namespace LSM6DS3TR
