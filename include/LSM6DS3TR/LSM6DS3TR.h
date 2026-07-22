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

/// @brief Three signed raw 16-bit sensor channels.
struct RawAxes {
  int16_t x = 0;  ///< X-axis raw count.
  int16_t y = 0;  ///< Y-axis raw count.
  int16_t z = 0;  ///< Z-axis raw count.
};

/// @brief Three floating-point sensor-native axes.
struct Axes {
  float x = 0.0f;  ///< X-axis value.
  float y = 0.0f;  ///< Y-axis value.
  float z = 0.0f;  ///< Z-axis value.
};

/// @brief Three fixed-unit signed 64-bit axes.
struct IntegerAxes {
  int64_t x = 0;  ///< X-axis fixed-unit value.
  int64_t y = 0;  ///< Y-axis fixed-unit value.
  int64_t z = 0;  ///< Z-axis fixed-unit value.
};

/// @brief Identity allocated to one accepted operation. Zero is never issued.
struct OperationToken {
  uint64_t value = 0;  ///< Driver-instance-local correlation identity.

  /// @return True when the token identifies an accepted operation.
  constexpr bool valid() const { return value != 0; }
};

/// @brief Compare operation correlation identities.
/// @param lhs First token.
/// @param rhs Second token.
/// @return True when both tokens contain the same value.
constexpr bool operator==(OperationToken lhs, OperationToken rhs) {
  return lhs.value == rhs.value;
}

/// @brief Compare operation correlation identities for inequality.
/// @param lhs First token.
/// @param rhs Second token.
/// @return True when token values differ.
constexpr bool operator!=(OperationToken lhs, OperationToken rhs) {
  return !(lhs == rhs);
}

/// @brief Procedure associated with an active or terminal operation.
enum class JobKind : uint8_t {
  NONE,         ///< No operation.
  PROBE,        ///< Main-bank and WHO_AM_I identity check.
  CONFIGURE,    ///< Complete profile write and readback.
  SAMPLE,       ///< Atomic raw measurement snapshot.
  RESET,        ///< Software reset and profile restoration.
  BOOT,         ///< Device boot/reload and profile restoration.
  RECOVER,      ///< Caller-requested device recovery and profile restoration.
  RECONCILE,    ///< Read-only managed-register comparison.
  POWER_DOWN,   ///< Verified accelerometer and gyroscope power-down.
  SELF_TEST,    ///< Built-in sensor self-test and profile restoration.
  CALIBRATION,  ///< Bounded bias candidate measurement.
  FIFO_PURGE    ///< Bounded destructive FIFO data removal.
};

/// @brief Lifecycle state reported for an operation.
enum class OperationState : uint8_t {
  IDLE,           ///< No operation is represented.
  ACTIVE,         ///< Accepted operation still requires polling.
  SUCCEEDED,      ///< Terminal confirmed success.
  FAILED,         ///< Terminal failure with known effect classification.
  CANCELLED,      ///< Terminal caller cancellation.
  TIMED_OUT,      ///< Terminal absolute deadline expiry.
  INDETERMINATE   ///< Terminal outcome whose hardware effect is ambiguous.
};

/// @brief Confidence state of the managed device register image.
enum class ConfigurationState : uint8_t {
  UNCONFIGURED,  ///< No complete verified managed image is available.
  APPLYING,      ///< A procedure may be changing the managed image.
  KNOWN,         ///< Managed image is verified and output is valid.
  UNKNOWN,       ///< Desired state exists but hardware provenance is unverified.
  SETTLING       ///< Managed image is verified; sensor output is not yet valid.
};

/// @brief Absolute times in the caller's single monotonic uptime domain.
struct OperationTiming {
  uint64_t nowMs = 0;  ///< Admission time supplied by the caller.
  uint64_t deadlineMs = 0;  ///< Must be strictly greater than nowMs.
};

/// @brief Independently selectable quantities in a managed sample.
enum class SampleQuantity : uint8_t {
  ACCELERATION = 1U << 0,  ///< Three-axis accelerometer data.
  ANGULAR_RATE = 1U << 1,  ///< Three-axis gyroscope data.
  TEMPERATURE = 1U << 2    ///< Internal temperature data.
};

/// @brief Convert one SampleQuantity flag to a quantity mask.
/// @param quantity Quantity flag to encode.
/// @return Corresponding uint8_t mask bit.
constexpr uint8_t sampleMask(SampleQuantity quantity) {
  return static_cast<uint8_t>(quantity);
}

static constexpr uint8_t SAMPLE_ACCELERATION =
    sampleMask(SampleQuantity::ACCELERATION);  ///< Acceleration field mask.
static constexpr uint8_t SAMPLE_ANGULAR_RATE =
    sampleMask(SampleQuantity::ANGULAR_RATE);  ///< Angular-rate field mask.
static constexpr uint8_t SAMPLE_TEMPERATURE =
    sampleMask(SampleQuantity::TEMPERATURE);  ///< Temperature field mask.
static constexpr uint8_t SAMPLE_ALL =
    SAMPLE_ACCELERATION | SAMPLE_ANGULAR_RATE |
    SAMPLE_TEMPERATURE;  ///< Mask containing every sample quantity.

/// @brief Evidence level attached to a raw or converted sample.
enum class SampleQuality : uint8_t {
  READY_CHECKED,      ///< Requested fields had fresh data-ready evidence.
  DIRECT_UNVERIFIED,  ///< Direct burst without freshness evidence.
  CONFIG_UNKNOWN,     ///< Sample configuration provenance is unavailable.
  SETTLING            ///< Quality label for a verified but unsettled profile.
};

/// @brief Quantity and readiness policy for one atomic sample operation.
struct SampleRequest {
  uint8_t quantityMask = SAMPLE_ALL;  ///< Nonzero subset of SAMPLE_ALL.
  bool checkDataReady = true;  ///< Require fresh status evidence before the burst.
};

/// @brief Atomic raw sample plus the immutable interpretation provenance.
struct RawSampleResult {
  uint64_t readUptimeMs = 0;  ///< Caller-clock time associated with the burst.
  uint64_t sequence = 0;  ///< Saturating sequence within this driver instance.
  RawAxes accel = {};  ///< Raw acceleration; meaningful when its valid bit is set.
  RawAxes gyro = {};  ///< Raw angular rate; meaningful when its valid bit is set.
  int16_t temperatureRaw = 0;  ///< Raw temperature count when valid.
  uint8_t validMask = 0;  ///< Quantity fields that contain meaningful data.
  uint8_t freshMask = 0;  ///< Valid fields proven ready for this request.
  SampleQuality quality = SampleQuality::DIRECT_UNVERIFIED;  ///< Acquisition evidence.
  uint32_t configGeneration = 0;  ///< Verified profile generation used for the read.
  AccelFs accelFullScale = AccelFs::G_2;  ///< Immutable acceleration scale provenance.
  GyroFs gyroFullScale = GyroFs::DPS_250;  ///< Immutable angular-rate scale provenance.
};

/// @brief Converted fixed-unit representation. Conversion never reads driver state.
struct ConvertedSample {
  uint64_t readUptimeMs = 0;  ///< Copied caller-clock sample time.
  uint64_t sequence = 0;  ///< Copied driver-instance sample sequence.
  IntegerAxes accelMicroG = {};  ///< Acceleration in micro-g.
  IntegerAxes gyroMicroDps = {};  ///< Angular rate in micro-degrees per second.
  int32_t temperatureMilliC = 0;  ///< Temperature in milli-degrees Celsius.
  uint8_t validMask = 0;  ///< Copied field-validity mask.
  uint8_t freshMask = 0;  ///< Copied field-freshness mask.
  SampleQuality quality = SampleQuality::DIRECT_UNVERIFIED;  ///< Copied evidence level.
  uint32_t configGeneration = 0;  ///< Copied configuration provenance.
};

/// @brief Address and identity returned by an identity-dependent operation.
struct ProbeResult {
  uint8_t address = 0;  ///< Seven-bit address that was queried.
  uint8_t whoAmI = 0;  ///< Observed WHO_AM_I value.
};

/// @brief Configuration evidence captured in an operation result.
struct ConfigurationResult {
  ConfigurationState state = ConfigurationState::UNCONFIGURED;  ///< Terminal evidence state.
  uint32_t generation = 0;  ///< Verified configuration generation.
  uint64_t validAfterUptimeMs = 0;  ///< Earliest caller time for interpreted samples.
  uint8_t mismatchRegister = 0;  ///< First register with failed readback, or zero.
  uint8_t expectedValue = 0;  ///< Expected value for mismatchRegister.
  uint8_t observedValue = 0;  ///< Observed value for mismatchRegister.
};

/// @brief Sample count for each baseline and stimulated self-test phase.
struct SelfTestRequest {
  uint16_t samples = 5;  ///< Average count per baseline/stimulus phase, 5..100.
};

/// @brief Sensor-native self-test measurements and restoration outcome.
struct SelfTestResult {
  Axes accelBaselineG = {};  ///< Averaged unstimulated acceleration in g.
  Axes accelStimulusG = {};  ///< Averaged stimulated acceleration in g.
  Axes accelDeltaG = {};  ///< Absolute per-axis acceleration response in g.
  Axes gyroBaselineDps = {};  ///< Averaged unstimulated angular rate in dps.
  Axes gyroStimulusDps = {};  ///< Averaged stimulated angular rate in dps.
  Axes gyroDeltaDps = {};  ///< Absolute per-axis angular-rate response in dps.
  bool accelPass = false;  ///< True when all acceleration deltas meet limits.
  bool gyroPass = false;  ///< True when all angular-rate deltas meet limits.
  Status primaryStatus = Status::Ok();  ///< Measurement/self-test outcome.
  Status restorationStatus = Status::Ok();  ///< Original-profile restoration outcome.
};

/// @brief Bias-calibration procedure to execute.
enum class CalibrationKind : uint8_t {
  ACCELEROMETER_BIAS,  ///< Measure bias relative to an explicit gravity vector.
  GYROSCOPE_BIAS       ///< Measure stationary angular-rate bias.
};

/// @brief Bounded sensor-native calibration request.
///
/// expectedAccelerationG makes mounting/orientation policy explicit; for
/// example, a Z-up fixture supplies {0, 0, 1}. Accelerometer calibration
/// requires a finite vector with magnitude 0.8..1.2 g. The driver never
/// assumes Z-up. Gyroscope calibration ignores the vector.
struct CalibrationRequest {
  CalibrationKind kind = CalibrationKind::GYROSCOPE_BIAS;  ///< Sensor to calibrate.
  uint16_t samples = 32;  ///< 1..1000.
  Axes expectedAccelerationG = {};  ///< Required fixture vector for acceleration.
  CalibrationLimits limits = {};  ///< Per-axis stability limits.
};

/// @brief Candidate bias and stability evidence returned by calibration.
struct CalibrationResult {
  CalibrationKind kind = CalibrationKind::GYROSCOPE_BIAS;  ///< Completed procedure.
  Axes bias = {};  ///< Sensor-native candidate bias in g or dps.
  Axes peakToPeak = {};  ///< Per-axis sample span in g or dps.
  uint16_t samples = 0;  ///< Valid samples included in the result.
};

/// @brief Bound for one explicitly destructive FIFO purge.
struct FifoPurgeRequest {
  uint16_t maxWords = 1;  ///< Maximum destructive FIFO word reads, 1..2048.
};

/// @brief FIFO loss and progress evidence from a purge operation.
struct FifoPurgeResult {
  uint16_t initialUnreadWords = 0;  ///< Unread words before consumption.
  uint16_t initialPattern = 0;  ///< FIFO pattern value before consumption.
  uint16_t wordsDiscarded = 0;  ///< Confirmed destructive data reads.
  uint16_t finalUnreadWords = 0;  ///< Unread words reported by the final status read.
  bool overrunObserved = false;  ///< True when either status snapshot reported overrun.
  bool truncated = false;  ///< True when unread data remained or exceeded the request.
};

/// @brief One terminal result. The caller must take it exactly once.
struct OperationResult {
  OperationToken token = {};  ///< Correlation identity assigned at admission.
  JobKind kind = JobKind::NONE;  ///< Procedure that produced this result.
  OperationState state = OperationState::IDLE;  ///< Terminal lifecycle state.
  Status status = Status::Ok();  ///< Primary terminal outcome.
  bool hardwareStateMayHaveChanged = false;  ///< Possible write or consuming-read effect.
  uint32_t transactions = 0;  ///< Transport callbacks used by the operation.
  uint32_t transactionLimit = 0;  ///< Hard callback ceiling applied at admission.
  uint64_t startedUptimeMs = 0;  ///< Caller-clock admission time.
  uint64_t completedUptimeMs = 0;  ///< Caller-clock terminal observation time.
  ProbeResult probe = {};  ///< Identity evidence when the job checked WHO_AM_I.
  ConfigurationResult configuration = {};  ///< Configuration evidence at completion.
  RawSampleResult sample = {};  ///< Atomic raw sample for SAMPLE jobs.
  SelfTestResult selfTest = {};  ///< Measurements and restoration evidence for SELF_TEST.
  CalibrationResult calibration = {};  ///< Candidate bias for CALIBRATION jobs.
  FifoPurgeResult fifoPurge = {};  ///< Destructive progress for FIFO_PURGE jobs.
};

/// @brief Result of one poll call.
struct PollResult {
  Status status = Status::Ok();  ///< Current progress or terminal status.
  OperationToken token = {};  ///< Active or terminal operation identity.
  uint16_t transactions = 0;       ///< Cumulative callbacks used by this job.
  uint16_t transactionLimit = 0;   ///< Hard callback ceiling for this job.
  JobKind kind = JobKind::NONE;  ///< Active or terminal procedure.
  OperationState state = OperationState::IDLE;  ///< Lifecycle after this poll.
  uint8_t transactionsUsed = 0;  ///< Callbacks used by this poll invocation.
  bool waiting = false;  ///< True when time/data must advance; no hidden sleep occurs.
};

/// @brief Passive diagnostics. Counters never gate or retry I2C.
struct DriverDiagnostics {
  uint32_t transportSuccesses = 0;  ///< Saturating lifetime success count.
  uint32_t transportFailures = 0;  ///< Saturating lifetime failure count.
  Status lastTransportError = Status::Ok();  ///< Most recent callback failure.
  uint64_t lastTransportErrorUptimeMs = 0;  ///< Caller time of that failure.
  uint32_t configGeneration = 0;  ///< Current verified generation.
  ConfigurationState configurationState =
      ConfigurationState::UNCONFIGURED;  ///< Current configuration evidence.
  uint64_t validAfterUptimeMs = 0;  ///< Earliest valid interpreted-sample time.
  uint8_t mismatchRegister = 0;  ///< Most recently mismatched managed register.
  uint8_t mismatchExpected = 0;  ///< Expected value for mismatchRegister.
  uint8_t mismatchObserved = 0;  ///< Observed value for mismatchRegister.
};

/// @name Fixed transport callback ceilings
/// These totals include every callback from admission to terminal result.
///@{
static constexpr uint32_t MAX_PROBE_TRANSACTIONS = 2;  ///< Probe ceiling.
static constexpr uint32_t MAX_CONFIGURE_TRANSACTIONS = 68;  ///< Configure ceiling.
static constexpr uint32_t MAX_SAMPLE_TRANSACTIONS = 66;  ///< Sample ceiling.
static constexpr uint32_t MAX_RESET_TRANSACTIONS = 88;  ///< Reset/boot ceiling.
static constexpr uint32_t MAX_RECOVER_TRANSACTIONS = 87;  ///< Recovery ceiling.
static constexpr uint32_t MAX_RECONCILE_TRANSACTIONS = 35;  ///< Reconcile ceiling.
static constexpr uint32_t MAX_POWER_DOWN_TRANSACTIONS = 8;  ///< Worst power-down ceiling.
///@}

/// @name Pure validation, timing, and conversion helpers
/// These functions perform no I2C, allocate no memory, and read no driver state.
///@{

/// @brief Validate a transport binding without invoking it.
/// @param config Candidate binding.
/// @return OK when callbacks, address, and timeout are valid.
Status validateDriverConfig(const DriverConfig& config);

/// @brief Validate a complete production profile.
/// @param profile Candidate replayable profile.
/// @return OK when every value and cross-field combination is supported.
Status validateProfile(const DeviceProfile& profile);

/// @brief Return the nominal period of an output data rate.
/// @param odr Output data rate.
/// @return Period in microseconds, or zero for power-down/invalid input.
uint64_t odrPeriodUs(Odr odr);

/// @brief Calculate the conservative post-configuration settling interval.
/// @param profile Valid production profile.
/// @return Settling interval in microseconds, saturating at UINT64_MAX.
uint64_t requiredSettleUs(const DeviceProfile& profile);

/// @brief Calculate the hard callback ceiling for self-test.
/// @param samples Average count per baseline/stimulated phase.
/// @return Exact ceiling for 5..100 samples, otherwise zero.
uint32_t maximumSelfTestTransactions(uint16_t samples);

/// @brief Calculate the hard callback ceiling for calibration.
/// @param samples Required valid sample count.
/// @return Exact ceiling for 1..1000 samples, otherwise zero.
uint32_t maximumCalibrationTransactions(uint16_t samples);

/// @brief Calculate the hard callback ceiling for destructive FIFO purge.
/// @param maxWords Maximum FIFO words to consume.
/// @return Exact ceiling for 1..2048 words, otherwise zero.
uint32_t maximumFifoPurgeTransactions(uint16_t maxWords);

/// @brief Return accelerometer sensitivity for a full-scale range.
/// @param fullScale Range encoding.
/// @param out Receives sensitivity in micro-g per LSB on success.
/// @return OK or INVALID_PARAM.
Status accelSensitivityMicroGPerLsb(AccelFs fullScale, int32_t& out);

/// @brief Return gyroscope sensitivity for a full-scale range.
/// @param fullScale Range encoding.
/// @param out Receives sensitivity in micro-degrees-per-second per LSB.
/// @return OK or INVALID_PARAM.
Status gyroSensitivityMicroDpsPerLsb(GyroFs fullScale, int32_t& out);

/// @brief Decode raw acceleration using explicit scale provenance.
/// @param raw Signed sensor counts.
/// @param fullScale Scale captured with the sample.
/// @param outMicroG Receives micro-g values atomically on success.
/// @return OK or INVALID_PARAM.
Status decodeAcceleration(const RawAxes& raw, AccelFs fullScale, IntegerAxes& outMicroG);

/// @brief Decode raw angular rate using explicit scale provenance.
/// @param raw Signed sensor counts.
/// @param fullScale Scale captured with the sample.
/// @param outMicroDps Receives micro-degrees-per-second values atomically on success.
/// @return OK or INVALID_PARAM.
Status decodeAngularRate(const RawAxes& raw, GyroFs fullScale, IntegerAxes& outMicroDps);

/// @brief Decode the device temperature formula raw/256 + 25 degrees Celsius.
/// @param raw Signed temperature count.
/// @return Temperature in milli-degrees Celsius.
int32_t decodeTemperatureMilliC(int16_t raw);

/// @brief Convert every valid field using provenance carried by the raw sample.
/// @param raw Atomic raw sample and interpretation provenance.
/// @param out Receives the converted sample atomically on success.
/// @return OK or INVALID_PARAM for inconsistent masks, quality, or scale.
Status convertSample(const RawSampleResult& raw, ConvertedSample& out);

/// @brief Validate calibration bounds and fixture semantics without I2C.
/// @param request Candidate calibration request.
/// @return OK or INVALID_PARAM.
Status validateCalibrationRequest(const CalibrationRequest& request);

/// @brief Subtract a finite bias from a finite sensor-native vector.
/// @param sample In/out vector in units matching @p bias.
/// @param bias Bias to subtract.
/// @return OK, or INVALID_PARAM without modifying @p sample.
Status applyBias(Axes& sample, const Axes& bias);
///@}

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

  /// @brief Validate and copy a non-owning transport binding. Performs zero I2C.
  /// @param config Binding copied by value; i2cUser and callbacks remain non-owning.
  /// @return OK, INVALID_CONFIG, BUSY, or RESULT_PENDING.
  Status bind(const DriverConfig& config);

  /// @brief Cancel local state, discard an untaken result, and unbind.
  /// @note Performs zero I2C and publishes no cancellation result.
  void unbind();

  /// @return True when a valid DriverConfig is bound.
  bool isBound() const { return _bound; }

  /// @return True while one accepted operation is active.
  bool operationActive() const { return _active; }

  /// @return True while one terminal result awaits takeResult().
  bool resultPending() const { return _resultPending; }

  /// @return Active token, or an invalid zero token when idle.
  OperationToken activeToken() const { return _active ? _token : OperationToken{}; }

  /// @return Active job kind, or JobKind::NONE when idle.
  JobKind activeJob() const { return _active ? _job : JobKind::NONE; }

  /// @brief Start a main-bank and WHO_AM_I identity check.
  /// @param timing Admission time and absolute deadline.
  /// @param token Receives a nonzero token only when accepted.
  /// @return IN_PROGRESS when accepted, otherwise a zero-I2C admission error.
  Status startProbe(const OperationTiming& timing, OperationToken& token);

  /// @brief Start complete profile application and register readback.
  /// @param profile Valid replayable production profile retained as desired state.
  /// @param timing Admission time and absolute deadline.
  /// @param token Receives a nonzero token only when accepted.
  /// @return IN_PROGRESS when accepted, otherwise a zero-I2C admission error.
  Status startConfigure(const DeviceProfile& profile, const OperationTiming& timing,
                        OperationToken& token);

  /// @brief Start one atomic managed sample.
  /// @param request Quantity and readiness policy.
  /// @param timing Admission time and absolute deadline.
  /// @param token Receives a nonzero token only when accepted.
  /// @return IN_PROGRESS when accepted, otherwise a zero-I2C admission error.
  Status startSample(const SampleRequest& request, const OperationTiming& timing,
                     OperationToken& token);

  /// @brief Start software reset followed by desired-profile replay/readback.
  /// @param timing Admission time and absolute deadline.
  /// @param token Receives a nonzero token only when accepted.
  /// @return IN_PROGRESS when accepted, otherwise a zero-I2C admission error.
  Status startReset(const OperationTiming& timing, OperationToken& token);

  /// @brief Start device boot/reload followed by desired-profile replay/readback.
  /// @param timing Admission time and absolute deadline.
  /// @param token Receives a nonzero token only when accepted.
  /// @return IN_PROGRESS when accepted, otherwise a zero-I2C admission error.
  Status startBoot(const OperationTiming& timing, OperationToken& token);

  /// @brief Re-probe, reset, and replay the desired profile on caller request.
  /// @param timing Admission time and absolute deadline.
  /// @param token Receives a nonzero token only when accepted.
  /// @return IN_PROGRESS when accepted, otherwise a zero-I2C admission error.
  /// @note This is device recovery only; bus recovery and retries remain external.
  Status startRecover(const OperationTiming& timing, OperationToken& token);

  /// @brief Read back the entire desired managed image without writing it.
  /// @param timing Admission time and absolute deadline.
  /// @param token Receives a nonzero token only when accepted.
  /// @return IN_PROGRESS when accepted, otherwise a zero-I2C admission error.
  Status startReconcile(const OperationTiming& timing, OperationToken& token);

  /// @brief Set and verify both sensor ODRs to power-down.
  /// @param timing Admission time and absolute deadline.
  /// @param token Receives a nonzero token only when accepted.
  /// @return IN_PROGRESS when accepted, otherwise a zero-I2C admission error.
  /// @note Success leaves configuration UNCONFIGURED; no other register is claimed.
  Status startPowerDown(const OperationTiming& timing, OperationToken& token);

  /// @brief Start the bounded built-in self-test and exact profile restoration.
  /// @param request Average count for each test phase.
  /// @param timing Admission time and absolute deadline.
  /// @param token Receives a nonzero token only when accepted.
  /// @return IN_PROGRESS when accepted, otherwise a zero-I2C admission error.
  Status startSelfTest(const SelfTestRequest& request, const OperationTiming& timing,
                       OperationToken& token);

  /// @brief Start bounded sensor-native bias calibration.
  /// @param request Sensor, sample count, fixture vector, and stability limits.
  /// @param timing Admission time and absolute deadline.
  /// @param token Receives a nonzero token only when accepted.
  /// @return IN_PROGRESS when accepted, otherwise a zero-I2C admission error.
  Status startCalibration(const CalibrationRequest& request, const OperationTiming& timing,
                          OperationToken& token);

  /// @brief Start explicitly destructive, bounded FIFO data removal.
  /// @param request Maximum word reads permitted.
  /// @param timing Admission time and absolute deadline.
  /// @param token Receives a nonzero token only when accepted.
  /// @return IN_PROGRESS when accepted, otherwise a zero-I2C admission error.
  Status startFifoPurge(const FifoPurgeRequest& request, const OperationTiming& timing,
                        OperationToken& token);

  /// @brief Advance one operation without exceeding a callback budget.
  /// @param nowMs Current time in the operation's caller-owned monotonic domain.
  /// @param maxTransactions Maximum callbacks allowed in this invocation; zero
  /// advances only time/CPU state.
  /// @return Active progress, terminal result summary, or idle/binding error.
  PollResult poll(uint64_t nowMs, uint8_t maxTransactions = 1);

  /// @brief Publish a CANCELLED terminal result without I2C.
  /// @param nowMs Cancellation time in the caller-owned monotonic domain.
  /// @return OK when an active job was cancelled, otherwise a precondition error.
  Status cancelActiveJob(uint64_t nowMs);

  /// @brief Take the matching terminal result exactly once.
  /// @param token Exact token returned when the operation was accepted.
  /// @param out Receives the complete terminal result on success.
  /// @return OK, RESULT_NOT_AVAILABLE, or STALE_RESULT.
  /// @note The returned Status reports retrieval; inspect out.status for job outcome.
  Status takeResult(OperationToken token, OperationResult& out);

  /// @param nowMs Current caller-owned monotonic time.
  /// @return Current configuration state, including transition from SETTLING to KNOWN.
  ConfigurationState configurationState(uint64_t nowMs) const;

  /// @return Current verified configuration generation.
  uint32_t configGeneration() const { return _configGeneration; }

  /// @return Earliest caller-clock time at which interpreted samples are valid.
  uint64_t validAfterUptimeMs() const { return _validAfterUptimeMs; }

  /// @brief Copy the most recently accepted desired profile.
  /// @param out Receives the profile on success.
  /// @return OK or CONFIGURATION_UNKNOWN if no desired profile exists.
  Status getDesiredProfile(DeviceProfile& out) const;

  /// @brief Copy the currently verified and settled profile.
  /// @param out Receives the profile on success.
  /// @param nowMs Current caller-owned monotonic time.
  /// @return OK, SETTLING, or CONFIGURATION_UNKNOWN.
  Status getVerifiedProfile(DeviceProfile& out, uint64_t nowMs) const;

  /// @param nowMs Current caller-owned monotonic time.
  /// @return Passive transport and configuration evidence snapshot.
  DriverDiagnostics diagnostics(uint64_t nowMs) const;

  /// @name Advanced one-transaction diagnostic access
  /// These calls are unavailable during an operation. Reads do not update
  /// production caches, but device-defined read side effects still apply (for
  /// example FIFO consumption or clearing a latched source). Any accepted write
  /// invalidates configuration provenance and prior samples because its hardware
  /// effect may be ambiguous.
  /// @{

  /// @brief Read one register without populating production caches.
  /// @param reg Main-bank register address.
  /// @param value Receives the byte on confirmed success.
  /// @param nowMs Caller-owned monotonic time for diagnostics.
  /// @return Transport or precondition status.
  Status diagnosticReadRegister(uint8_t reg, uint8_t& value, uint64_t nowMs);

  /// @brief Read one auto-incremented register block without updating caches.
  /// @param startReg First main-bank register address.
  /// @param data Caller-owned output buffer.
  /// @param length Number of bytes, 1..32, remaining within the supported
  /// main-bank range through the Z user-offset register.
  /// @param nowMs Caller-owned monotonic time for diagnostics.
  /// @return Transport, range, or precondition status.
  Status diagnosticReadBlock(uint8_t startReg, uint8_t* data, size_t length,
                             uint64_t nowMs);

  /// @brief Write one safety-filtered register and invalidate configuration provenance.
  /// @param reg Main-bank register address.
  /// @param value Complete register value to write.
  /// @param nowMs Caller-owned monotonic time for diagnostics.
  /// @return Transport, safety-validation, or precondition status.
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
  uint64_t _nextToken = 1;
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
  bool _pollBoundary = false;
  bool _hardwareStateMayHaveChanged = false;
  bool _configurationMayBeUnknown = false;
  ConfigurationState _configurationStateBeforeOperation =
      ConfigurationState::UNCONFIGURED;
  uint64_t _validAfterBeforeOperationMs = 0;

  OperationResult _workingResult = {};
  OperationResult _terminalResult = {};

  DeviceProfile _desiredProfile = {};
  DeviceProfile _verifiedProfile = {};
  DeviceProfile _selfTestRestoreProfile = {};
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

  uint64_t _sampleSequence = 0;
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
  uint64_t _lastTransportErrorUptimeMs = 0;
};

}  // namespace LSM6DS3TR
