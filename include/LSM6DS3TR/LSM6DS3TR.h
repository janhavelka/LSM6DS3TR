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
  FIFO_PURGE,   ///< Bounded destructive FIFO data removal.
  FIFO_READ,    ///< Bounded typed FIFO acquisition into caller storage.
  COUNTER_RESET, ///< Explicit timestamp/step reset in a verified Bypass profile.
  EVENT_READ    ///< Bounded supported event-source observations and explicit read-clear effects.
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
  APPLYING,      ///< A procedure is writing or verifying the managed image.
  KNOWN,         ///< Managed image is verified and output is valid.
  UNKNOWN,       ///< Desired state exists but hardware provenance is unverified.
  SETTLING,      ///< Managed image is verified; sensor output is not yet valid.
  FILTER_UNQUALIFIED ///< Verified image; gyro HPF output has no source-backed settling bound.
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
  READY_CHECKED,      ///< Every valid field has fresh data-ready evidence.
  DIRECT_UNVERIFIED,  ///< Direct burst; freshMask must be zero.
  CONFIG_UNKNOWN,     ///< Configuration unknown; freshness remains field-specific.
  SETTLING,           ///< Verified profile unsettled; freshness remains field-specific.
  FILTER_UNVERIFIED   ///< Explicit direct read includes unqualified HPF; freshMask is zero.
};

/// @brief Quantity and readiness policy for one atomic sample operation.
struct SampleRequest {
  uint8_t quantityMask = SAMPLE_ALL;  ///< Nonzero subset of SAMPLE_ALL.
  bool checkDataReady = true;  ///< Require fresh status evidence before the burst.
  bool allowUnqualifiedFilter = false;  ///< Explicit opt-in, only with checkDataReady=false.
};

/// @brief Immutable filter interpretation, independent of current driver state.
/// @note Reference output is relative to a stored reference, not absolute gravity.
/// Host observation time and availability never establish filter qualification.
struct FilterProvenance {
  AccelFilterConfig accel = {};  ///< Complete accelerometer host and orientation path.
  GyroFilterConfig gyro = {};  ///< Gyroscope LPF1/HPF choices.
  Odr accelOdr = Odr::HZ_104;  ///< Accelerometer rate when captured.
  Odr gyroOdr = Odr::HZ_104;  ///< Gyroscope rate when captured.
  AccelPowerMode accelPowerMode = AccelPowerMode::HIGH_PERFORMANCE;  ///< Accel mode.
  GyroPowerMode gyroPowerMode = GyroPowerMode::HIGH_PERFORMANCE;  ///< Gyro mode.
};

/// @brief Atomic raw sample plus the immutable interpretation provenance.
struct RawSampleResult {
  uint64_t readUptimeMs = 0;  ///< Caller-clock time associated with the burst.
  uint64_t sequence = 0;  ///< Saturating sequence within this driver instance.
  RawAxes accel = {};  ///< Raw acceleration; meaningful when its valid bit is set.
  RawAxes gyro = {};  ///< Raw angular rate; meaningful when its valid bit is set.
  int16_t temperatureRaw = 0;  ///< Raw temperature count when valid.
  uint8_t validMask = 0;  ///< Quantity fields that contain meaningful data.
  uint8_t freshMask = 0;  ///< Ready evidence; all valid bits or zero for direct reads.
  SampleQuality quality = SampleQuality::DIRECT_UNVERIFIED;  ///< Acquisition evidence.
  uint32_t configGeneration = 0;  ///< Verified profile generation used for the read.
  AccelFs accelFullScale = AccelFs::G_2;  ///< Immutable acceleration scale provenance.
  GyroFs gyroFullScale = GyroFs::DPS_250;  ///< Immutable angular-rate scale provenance.
  FilterProvenance filters = {};  ///< Immutable filter interpretation.
  uint8_t qualifiedMask = SAMPLE_ALL;  ///< Qualified fields; intersect with validMask.
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
  FilterProvenance filters = {};  ///< Copied immutable filter interpretation.
  uint8_t qualifiedMask = SAMPLE_ALL;  ///< Copied qualification, separate from freshness.
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
  uint64_t validAfterUptimeMs = 0;  ///< Time gate for qualified paths only; never qualifies gyro HPF.
  uint8_t mismatchRegister = 0;  ///< First register with failed readback, or zero.
  uint8_t expectedValue = 0;  ///< Expected value for mismatchRegister.
  uint8_t observedValue = 0;  ///< Observed value for mismatchRegister.
  uint8_t qualifiedMask = 0;  ///< Qualified active quantities at completion.
  bool referenceDiscardMayHaveOccurred = false;  ///< Reference output consumption attempted.
  bool referenceDiscardConfirmed = false;  ///< Required first reference XYZ read succeeded.
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

/// @brief Bounded software bias-estimation request using sensor samples.
///
/// expectedAccelerationG makes mounting/orientation policy explicit; for
/// example, a Z-up fixture supplies {0, 0, 1}. Accelerometer calibration
/// requires a finite vector with magnitude 0.8..1.2 g. The driver never
/// assumes Z-up. Gyroscope measurement ignores the vector, but common request
/// validation still requires every vector component to be finite and bounded.
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
  uint32_t configGeneration = 0;  ///< Bias belongs to this configuration only; never auto-reused.
  FilterProvenance filters = {};  ///< Filter/ODR context of the candidate bias.
};

/// @brief Bound for one explicitly destructive FIFO purge.
struct FifoPurgeRequest {
  uint16_t maxWords = 1;  ///< Maximum destructive FIFO word reads, 1..2048.
  bool stopCollection = false;  ///< Explicitly flush residual/racing arrivals in Bypass after draining.
};

/// @brief FIFO loss and progress evidence from a purge operation.
struct FifoPurgeResult {
  uint16_t initialUnreadWords = 0;  ///< Unread words before consumption.
  uint16_t initialPattern = 0;  ///< FIFO pattern value before consumption.
  uint16_t wordsDiscarded = 0;  ///< Confirmed destructive data reads.
  uint16_t finalUnreadWords = 0;  ///< Unread words reported by the final status read.
  bool overrunObserved = false;  ///< True when either status snapshot reported overrun.
  bool truncated = false;  ///< True when unread data remained or exceeded the request.
  uint16_t wordsPossiblyDiscarded = 0;  ///< Upper bound of attempted consuming reads, including failed reads.
  bool collectionStopped = false;  ///< Bypass was written and read back successfully.
  bool residualLossPossible = false;  ///< Bypass may discard residual words or arrivals after the last observation.
};

/// @brief Interpretation of one complete six-byte FIFO record.
enum class FifoRecordKind : uint8_t {
  ANGULAR_RATE = 0,  ///< Signed gyroscope XYZ.
  ACCELERATION,     ///< Signed accelerometer XYZ.
  TEMPERATURE,      ///< Signed temperature; batching proves no new conversion.
  TIMESTAMP_STEP    ///< Raw counters, without wrap expansion or clock mapping.
};

/// @brief One complete FIFO record with immutable interpretation provenance.
/// @note No sample timestamp or freshness is inferred. The read time describes
/// host capture only. The token is published only for a successful batch.
struct FifoMotionRecord {
  OperationToken token = {};  ///< Successful acquisition identity; zero while scratch.
  uint64_t readUptimeMs = 0;  ///< Caller clock at the read completing this record.
  uint32_t configGeneration = 0;  ///< Verified generation at acquisition admission.
  RawAxes raw = {};  ///< Complete signed sensor-native XYZ vector.
  uint8_t quantityMask = 0;  ///< Motion/temperature quantity; zero for counter records.
  AccelFs accelFullScale = AccelFs::G_2;  ///< Immutable accelerometer scale.
  GyroFs gyroFullScale = GyroFs::DPS_250;  ///< Immutable gyroscope scale.
  FifoRecordKind kind = FifoRecordKind::ANGULAR_RATE;  ///< Selects the meaningful payload.
  int16_t temperatureRaw = 0;  ///< TEMPERATURE only: raw/256 + 25 degrees Celsius.
  uint32_t timestampTicks = 0;  ///< TIMESTAMP_STEP only: raw 24-bit timer, may wrap.
  uint16_t stepCount = 0;  ///< TIMESTAMP_STEP only: raw 16-bit count, may wrap.
  TimestampResolution timestampResolution = TimestampResolution::MS_6_4;  ///< Tick scale.
  uint64_t counterEpoch = 0;  ///< Driver-instance interpretation interval, never a wrap total.
  bool counterContinuityLost = true;  ///< No lossless counter observation claim in this interval.
  FilterProvenance filters = {};  ///< Interpretation established before collection activation.
  uint8_t qualifiedMask = 0;  ///< Qualified payload fields; no freshness/timestamp claim.
};

/// @brief General FIFO record name; the historical motion name remains compatible.
using FifoRecord = FifoMotionRecord;

/// @brief Explicit reset selection; at least one counter must be selected.
struct CounterResetRequest {
  bool timestamp = true;  ///< Reset timestamp, acknowledging its end-counter indication.
  bool steps = true;  ///< Assert and explicitly clear PEDO_RST_STEP.
};

/// @brief Counter lifetime and reset effects, including incomplete operations.
struct CounterResult {
  uint64_t epoch = 0;  ///< Interpretation interval; counters remain raw and wrapping.
  bool continuityLost = true;  ///< Sticky uncertainty until a successful joint explicit reset.
  bool timestampResetMayHaveOccurred = false;  ///< Timestamp command was attempted.
  bool stepResetMayHaveOccurred = false;  ///< Pedometer reset assertion was attempted.
  bool timestampResetConfirmed = false;  ///< Timestamp reset write succeeded.
  bool stepResetConfirmed = false;  ///< Pedometer reset assertion/clear/readback succeeded.
};

/// @brief Bounded caller storage and consumption limit for one FIFO acquisition.
/// @note Storage must remain alive and exclusively owned by the operation until
/// takeResult(), or unbind(). Accepted admission clears all capacity entries;
/// rejected admission leaves storage unchanged. Storage is scratch until a
/// successful matching terminal result is taken. Only its records count is valid.
/// Failure, cancellation and unbind clear the accepted buffer. No partial vector
/// is published; leading/trailing fragments and transition vectors are discarded.
struct FifoReadRequest {
  FifoMotionRecord* records = nullptr;  ///< Required buffer of capacity entries.
  uint16_t capacity = 0;  ///< Buffer entries, 1..682; short buffers stop consumption.
  uint16_t maxWords = 1;  ///< Maximum consumed words, 1..2048, including discards.
};

/// @brief One bounded status/pattern observation; not an atomic sample timestamp.
struct FifoStatusSnapshot {
  uint64_t readUptimeMs = 0;  ///< Caller time associated with the status callback.
  uint16_t unreadWords = 0;  ///< Decoded count; overrun/count-zero is 2048.
  uint16_t pattern = 0;  ///< Next word index within the verified pattern.
  bool valid = false;  ///< Status callback succeeded; consistency checked separately.
  bool empty = false;  ///< FIFO_EMPTY; asserted EMPTY always prevents data reads.
  bool full = false;  ///< FIFO_FULL_SMART predicts the next complete data set filling FIFO.
  bool overrun = false;  ///< OVER_RUN observation.
  bool watermark = false;  ///< WaterM observation, not an acquisition trigger.
};

/// @brief FIFO batch, consumption and continuity evidence, retained even on failure.
struct FifoReadResult {
  FifoStatusSnapshot before = {};  ///< Initial status before any consumption.
  FifoStatusSnapshot after = {};  ///< Latest successful status after a burst or initial status.
  uint16_t records = 0;  ///< Published complete vectors; zero on any failed/cancelled job.
  uint16_t wordsConsumed = 0;  ///< Words confirmed returned by successful callbacks.
  uint16_t wordsPossiblyConsumed = 0;  ///< Upper bound including a failed consuming callback.
  uint16_t wordsDiscarded = 0;  ///< Confirmed words not in the successful published vectors.
  bool truncated = false;  ///< Word/capacity limit, partial vector, or unread words remained.
  bool overrunObserved = false;  ///< Any observed overrun; a batch then fails.
  bool patternDiscontinuity = false;  ///< Observed index differed from predicted progress.
  bool continuityLost = false;  ///< Loss/uncertainty since activation; never a timestamp/freshness claim.
};

/// @name Event source groups (not pin routes or arbitrary register masks)
///@{
static constexpr uint8_t EVENT_DATA_READY = 1U;  ///< STATUS_REG, nonclearing.
static constexpr uint8_t EVENT_FIFO = 2U;  ///< FIFO_STATUS2, nonconsuming.
static constexpr uint8_t EVENT_EMBEDDED = 4U;  ///< FUNC_SRC1, supported step causes only.
static constexpr uint8_t EVENT_ORIENTATION = 8U;  ///< D6D_SRC, 6D/4D request and directions.
///@}

/// @brief Select supported source groups; each selected group uses one callback.
/// @note Embedded/orientation reads may acknowledge routed latched sources.
/// Timestamp end has no readable source flag; use application pin evidence and
/// the existing explicit timestamp CounterResetRequest for acknowledgement.
struct EventReadRequest {
  bool dataReady = true;  ///< Observe STATUS_REG without reading sensor outputs.
  bool fifo = false;  ///< Requires a typed FIFO profile; consumes no queued data.
  bool embedded = false;  ///< Requires the fixed timestamp/step counter profile.
  bool orientation = false;  ///< Requires an enabled typed 6D/4D profile.
};

/// @brief One independent source-register observation, never a simultaneous snapshot.
struct EventObservation {
  uint64_t readUptimeMs = 0;  ///< Owner time associated with this specific successful read.
  uint8_t raw = 0;  ///< Unmodified source byte, including uninterpreted bits.
  bool valid = false;  ///< This observation's callback succeeded.
};

/// @brief Event evidence and acknowledgement effects retained even on failure.
/// @note A token is published only for complete successful observation. Partial
/// valid observations remain evidence, not a successful event batch. Raw source
/// bits do not measure a pad level or prove that missed pulses were recovered.
struct EventReadResult {
  OperationToken token = {};  ///< Nonzero only for a successful EVENT_READ result.
  uint32_t configGeneration = 0;  ///< Verified interpretation at event-read admission.
  EventObservation ready = {};  ///< Independent data-ready status observation.
  EventObservation fifo = {};  ///< Independent FIFO status observation.
  EventObservation embedded = {};  ///< Independent step event-source observation.
  EventObservation orientation = {};  ///< Independent 6D/4D source observation.
  uint8_t requestedMask = 0;  ///< Selected EVENT_* groups.
  uint8_t readMask = 0;  ///< Groups with successful callbacks; retained on partial failure.
  uint8_t mayHaveAcknowledgedMask = 0;  ///< Attempted reads with source/output clearing semantics.
  uint8_t acknowledgedMask = 0;  ///< Successful reads with those semantics, not proof an event existed.
  uint8_t dataReadyMask = 0;  ///< Supported active sensor SAMPLE_* ready flags.
  bool fifoWatermark = false;  ///< Observed WaterM.
  bool fifoOverrun = false;  ///< Observed OVER_RUN; not persistent history.
  bool fifoFull = false;  ///< Observed FIFO_FULL_SMART.
  bool stepDetected = false;  ///< Observed supported STEP_DETECTED flag.
  bool stepOverflow = false;  ///< Observed supported STEP_OVERFLOW flag.
  bool orientationDetected = false;  ///< Observed D6D_IA request.
  bool xLow = false;  ///< Observed XL orientation bit, not a unique orientation claim.
  bool xHigh = false;  ///< Observed XH orientation bit.
  bool yLow = false;  ///< Observed YL orientation bit.
  bool yHigh = false;  ///< Observed YH orientation bit.
  bool zLow = false;  ///< Observed ZL bit; not decoded as a 4D cause.
  bool zHigh = false;  ///< Observed ZH bit; not decoded as a 4D cause.
  bool timestampRouted = false;  ///< Configuration fact only; no timestamp status bit exists.
  bool historyMayBeLost = false;  ///< Observed loss, maintenance or ambiguous acknowledgement.
  FilterProvenance filters = {};  ///< Includes orientation LPF1 versus LPF2 source selection.
  uint8_t qualifiedMask = 0;  ///< Qualified motion/temperature paths; ready flags prove availability only.
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
  FifoReadResult fifoRead = {};  ///< Typed batch and destructive-read evidence for FIFO_READ.
  CounterResult counters = {};  ///< Raw-counter epoch and explicit/required reset effects.
  bool registerBankMayBeUnknown = false;  ///< Embedded-bank restoration was not proved.
  EventReadResult events = {};  ///< Independent source observations and acknowledgement effects.
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
  uint64_t validAfterUptimeMs = 0;  ///< Time gate for documented paths; excludes gyro HPF qualification.
  uint8_t mismatchRegister = 0;  ///< Most recently mismatched managed register.
  uint8_t mismatchExpected = 0;  ///< Expected value for mismatchRegister.
  uint8_t mismatchObserved = 0;  ///< Observed value for mismatchRegister.
};

/// @name Fixed operation transaction ceilings
/// These totals include every callback from admission to terminal result.
///@{
static constexpr uint32_t MAX_PROBE_TRANSACTIONS = 2;  ///< Probe ceiling.
static constexpr uint32_t MAX_CONFIGURE_TRANSACTIONS = 111;  ///< Includes reference discard and possible held-output release.
static constexpr uint32_t MAX_SAMPLE_TRANSACTIONS = 66;  ///< Sample ceiling.
static constexpr uint32_t MAX_RESET_TRANSACTIONS = 132;  ///< Reset/boot including reference discard and held-output release.
static constexpr uint32_t MAX_RECOVER_TRANSACTIONS = 131;  ///< Recovery including reference discard and held-output release.
static constexpr uint32_t MAX_RECONCILE_TRANSACTIONS = 35;  ///< Reconcile ceiling.
static constexpr uint32_t MAX_POWER_DOWN_TRANSACTIONS = 17;  ///< Includes route masking and FIFO guard.
static constexpr uint32_t MAX_COUNTER_RESET_TRANSACTIONS = 7;  ///< Identity, Bypass, reset/clear verification.
static constexpr uint32_t MAX_EVENT_READ_TRANSACTIONS = 6;  ///< Bank, identity and four source groups.
///@}

/// @name Fixed transport buffer ceilings
/// These bounds include every byte presented to an injected callback.
///@{
static constexpr size_t MAX_TRANSPORT_WRITE_BYTES = 33;  ///< Includes register prefix.
static constexpr size_t MAX_TRANSPORT_READ_BYTES = 32;  ///< Maximum receive length.
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

/// @brief One position in the untagged internal FIFO pattern.
struct FifoPatternWord {
  uint16_t patternWords = 0;  ///< Complete repeating pattern length, at most 393.
  uint8_t quantityMask = 0;  ///< Motion/temperature quantity; zero for counter records.
  uint8_t axis = 0;  ///< Record word 0..2; X/Y/Z for motion, payload words for fourth.
  FifoRecordKind kind = FifoRecordKind::ANGULAR_RATE;  ///< Schema, not an on-wire tag.
};

static constexpr uint16_t MAX_FIFO_PATTERN_WORDS = 393;  ///< Three factors 1/3/32: 131 records.
static constexpr uint16_t MAX_FIFO_PATTERN_TICKS = 96;  ///< LCM of admitted factors.

/// @brief Resolve a word position using one bounded, allocation-free pattern walk.
/// @param profile Valid enabled FIFO profile, including prepared BYPASS.
/// @param index Word index in the repeating pattern; out-of-range is rejected.
/// @param out Receives sensor, axis and length only on success.
/// @return OK or a validation error; performs zero I2C and preserves out on error.
Status fifoPatternWord(const DeviceProfile& profile, uint16_t index,
                       FifoPatternWord& out);

/// @brief Return the nominal period of an output data rate.
/// @param odr Output data rate.
/// @return Period in microseconds, or zero for power-down/invalid input.
uint64_t odrPeriodUs(Odr odr);

/// @brief Calculate the conservative post-configuration settling interval.
/// @param profile Valid production profile.
/// @return Interval in microseconds, or UINT64_MAX for an invalid profile.
/// @note This bounds only documented paths; it never qualifies gyro HPF.
/// One further output period follows each active sensor's discard-count interval
/// so the most recent output can replace the final discarded sample.
/// Reference mode also requires staged first-output consumption and one new period.
/// @note The operation engine rounds a nonzero interval up to milliseconds and
/// adds one clock-quantization tick when arming it after full profile readback.
uint64_t requiredSettleUs(const DeviceProfile& profile);

/// @brief Capture immutable filter interpretation without I2C.
/// @param profile Valid profile to describe.
/// @return Complete filter, rate and mode value.
FilterProvenance filterProvenance(const DeviceProfile& profile);

/// @brief Identify active quantities with a source-backed filter qualification rule.
/// @param profile Valid profile; this helper does not establish elapsed settling.
/// @return SAMPLE_* mask excluding inactive sensors and unqualified gyro HPF.
uint8_t qualifiedQuantityMask(const DeviceProfile& profile);

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
/// @param stopCollection Include the final Bypass write and verification.
uint32_t maximumFifoPurgeTransactions(uint16_t maxWords, bool stopCollection = false);

/// @brief Conservative FIFO read ceiling: identity checks, initial status, then
/// at most one data/status pair per word. Each actual burst receives 1..16 words.
/// @param maxWords Maximum destructive words, 1..2048.
/// @return Hard callback ceiling 2*maxWords+3, or zero for invalid input.
uint32_t maximumFifoReadTransactions(uint16_t maxWords);

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
/// @note READY_CHECKED requires freshMask == validMask; DIRECT_UNVERIFIED
/// requires freshMask == 0. CONFIG_UNKNOWN and SETTLING retain any per-field
/// freshness subset. Failure leaves @p out unchanged.
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
  /// @note Hardware FIFO must already be in BYPASS. A collecting FIFO fails
  /// before interpretation writes with FIFO_MAINTENANCE_REQUIRED, even if empty;
  /// use explicit bounded stopCollection maintenance first. An enabled target
  /// is activated and verified only after the disabled image has settled.
  /// @param profile Valid replayable production profile retained as desired state.
  /// @param timing Admission time and absolute deadline.
  /// @param token Receives a nonzero token only when accepted.
  /// @return IN_PROGRESS when accepted, otherwise a zero-I2C admission error.
  Status startConfigure(const DeviceProfile& profile, const OperationTiming& timing,
                        OperationToken& token);

  /// @brief Start one atomic managed sample.
  /// @note Gyro HPF requires checkDataReady=false and allowUnqualifiedFilter=true;
  /// its result has FILTER_UNVERIFIED quality and no freshness claim. Other
  /// qualified requested channels retain their independent qualifiedMask bits.
  /// A failed output burst may hold a BDU byte pair; configure/recover must
  /// release it before another managed output sample or calibration.
  /// @param request Quantity and readiness policy.
  /// @param timing Admission time and absolute deadline.
  /// @param token Receives a nonzero token only when accepted.
  /// @return IN_PROGRESS when accepted, otherwise a zero-I2C admission error.
  Status startSample(const SampleRequest& request, const OperationTiming& timing,
                     OperationToken& token);

  /// @brief Start software reset followed by desired-profile replay/readback.
  /// @note Requires observed FIFO BYPASS before changing sensor configuration.
  /// @param timing Admission time and absolute deadline.
  /// @param token Receives a nonzero token only when accepted.
  /// @return IN_PROGRESS when accepted, otherwise a zero-I2C admission error.
  Status startReset(const OperationTiming& timing, OperationToken& token);

  /// @brief Start device boot/reload followed by desired-profile replay/readback.
  /// @note Requires observed FIFO BYPASS before changing sensor configuration.
  /// @param timing Admission time and absolute deadline.
  /// @param token Receives a nonzero token only when accepted.
  /// @return IN_PROGRESS when accepted, otherwise a zero-I2C admission error.
  Status startBoot(const OperationTiming& timing, OperationToken& token);

  /// @brief Re-probe, reset, and replay the desired profile on caller request.
  /// @note Stop collection explicitly first; recovery never silently flushes FIFO.
  /// @param timing Admission time and absolute deadline.
  /// @param token Receives a nonzero token only when accepted.
  /// @return IN_PROGRESS when accepted, otherwise a zero-I2C admission error.
  /// @note This is device recovery only; bus recovery and retries remain external.
  Status startRecover(const OperationTiming& timing, OperationToken& token);

  /// @brief Read back the entire desired managed image without writing it.
  /// @note Readback cannot restore lost provenance for an active FIFO profile;
  /// that case requires stopCollection maintenance and configure/recover.
  /// @param timing Admission time and absolute deadline.
  /// @param token Receives a nonzero token only when accepted.
  /// @return IN_PROGRESS when accepted, otherwise a zero-I2C admission error.
  Status startReconcile(const OperationTiming& timing, OperationToken& token);

  /// @brief Mask interrupt routes, then set and verify both sensor ODRs to power-down.
  /// @note Requires observed FIFO BYPASS before either sensor write.
  /// @param timing Admission time and absolute deadline.
  /// @param token Receives a nonzero token only when accepted.
  /// @return IN_PROGRESS when accepted, otherwise a zero-I2C admission error.
  /// @note Success leaves configuration UNCONFIGURED. Only sensor power-down
  /// and the four masked route registers are proved, not a complete profile.
  Status startPowerDown(const OperationTiming& timing, OperationToken& token);

  /// @brief Start bounded built-in self-test and full-profile restoration/readback.
  /// @note Requires a verified BYPASS profile; active FIFO returns
  /// FIFO_MAINTENANCE_REQUIRED without issuing a token or performing I2C.
  /// Before any write, the accepted operation checks the current main bank,
  /// identity and FIFO Bypass. Contradicting hardware invalidates provenance;
  /// a preflight failure performs no restoration writes.
  /// @note Keep the device stationary for the complete operation.
  /// @note SelfTestResult reports the primary and restoration outcomes separately.
  /// @param request Average count for each test phase.
  /// @param timing Admission time and absolute deadline.
  /// @param token Receives a nonzero token only when accepted.
  /// @return IN_PROGRESS when accepted, otherwise a zero-I2C admission error.
  Status startSelfTest(const SelfTestRequest& request, const OperationTiming& timing,
                       OperationToken& token);

  /// @brief Start bounded software bias estimation using sensor samples.
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

  /// @brief Read complete typed records from the settled verified FIFO profile.
  /// @param request Caller-owned scratch storage and finite consumption bound.
  /// @param timing Admission and immutable absolute deadline in the owner clock.
  /// @param token Receives a nonzero identity only on acceptance.
  /// @return IN_PROGRESS or a zero-I2C admission error.
  /// @note Only the initial observed backlog is eligible; concurrent arrivals
  /// remain queued. Every burst is followed by status/pattern validation. Overrun,
  /// discontinuity or any callback failure publishes zero records. There is no
  /// retry and possible hardware consumption cannot be rolled back. A new
  /// activation discards the first complete vector of each selected sensor.
  Status startFifoRead(const FifoReadRequest& request, const OperationTiming& timing,
                       OperationToken& token);

  /// @brief Explicitly reset selected counters through the bounded operation engine.
  /// @param request Timestamp/step selection; neither selected is invalid.
  /// @param timing One immutable absolute deadline in owner uptime.
  /// @param token Receives an accepted operation identity, otherwise zero.
  /// @return IN_PROGRESS, or a bus-silent admission error.
  /// @note Requires a known timestamp/step Bypass profile. Reads/reconcile never
  /// reset counters. Cancellation is bus-silent; an uncleared step reset leaves
  /// configuration unknown. Raw wraps are never expanded into monotonic time.
  Status startCounterReset(const CounterResetRequest& request,
                           const OperationTiming& timing, OperationToken& token);

  /// @brief Observe selected supported sources through the bounded operation engine.
  /// @param request Source groups; at least one must be selected.
  /// @param timing One immutable absolute owner-clock deadline.
  /// @param token Accepted correlation token, otherwise zero.
  /// @return IN_PROGRESS, or a bus-silent validation/admission error.
  /// @note Requires verified bytes and completed documented gates. Ready flags
  /// prove availability, never gyro HPF qualification. Routed latched sources are
  /// acknowledged by their source read, with possible effects visible on error.
  /// Timestamp end acknowledgement is the explicit timestamp counter reset.
  Status startEventRead(const EventReadRequest& request, const OperationTiming& timing,
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
  /// @return OK, NOT_BOUND, or CONFIGURATION_UNKNOWN if no desired profile exists.
  Status getDesiredProfile(DeviceProfile& out) const;

  /// @brief Copy the verified profile after its documented time gates.
  /// @note FILTER_UNQUALIFIED also exposes verified bytes; gyro HPF still has no
  /// qualified output rule. This getter does not upgrade output confidence.
  /// @param out Receives the profile on success.
  /// @param nowMs Current caller-owned monotonic time.
  /// @return OK, NOT_BOUND, SETTLING, or CONFIGURATION_UNKNOWN.
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
  /// main-bank range through the Z user-offset register. Ranges touching
  /// 0x20..0x2D must include both bytes of every touched output pair.
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
  // Native fault-injection tests lower the callback ceiling without changing
  // production admission limits or exposing a runtime test hook.
  friend struct TransactionCeilingTestAccess;

  static constexpr uint8_t MANAGED_REGISTER_COUNT = 33;
  static constexpr uint16_t CONFIGURE_SETTLE_ARM = 2U * MANAGED_REGISTER_COUNT;
  static constexpr uint16_t CONFIGURE_SETTLE_WAIT = CONFIGURE_SETTLE_ARM + 1U;
  static constexpr uint16_t CONFIGURE_FIFO_ACTIVATE = CONFIGURE_SETTLE_WAIT + 1U;
  static constexpr uint16_t CONFIGURE_FIFO_VERIFY = CONFIGURE_FIFO_ACTIVATE + 1U;
  static constexpr uint16_t CONFIGURE_COMMIT = CONFIGURE_FIFO_VERIFY + 1U;
  static constexpr uint16_t CONFIGURE_REFERENCE_READY = CONFIGURE_COMMIT + 1U;
  static constexpr uint16_t CONFIGURE_REFERENCE_READ = CONFIGURE_REFERENCE_READY + 1U;
  static constexpr uint16_t CONFIGURE_REFERENCE_ARM = CONFIGURE_REFERENCE_READ + 1U;

  Status _start(JobKind kind, const OperationTiming& timing, OperationToken& token);
  PollResult _pollOne(uint64_t nowMs);
  Status _stepProbe(uint64_t nowMs);
  Status _stepConfigure(uint64_t nowMs, bool reconcileOnly);
  Status _stepSample(uint64_t nowMs);
  Status _stepResetBoot(uint64_t nowMs, bool boot, bool recovery);
  Status _stepPowerDown(uint64_t nowMs);
  Status _stepSelfTest(uint64_t nowMs);
  Status _stepCalibration(uint64_t nowMs);
  Status _stepFifoPurge(uint64_t nowMs);
  Status _stepFifoRead(uint64_t nowMs);
  Status _stepCounterReset(uint64_t nowMs);
  Status _stepCounterSetup(const DeviceProfile& target, uint64_t nowMs);
  Status _stepEventRead(uint64_t nowMs);
  Status _stepMaskRoutes(uint64_t nowMs);
  void _breakCounterContinuity();
  void _clearFifoOutput();
  Status _finish(const Status& status, OperationState state);
  Status _fail(const Status& status);
  void _clearActive();
  void _invalidateConfiguration();
  void _prepareManagedImage(const DeviceProfile& profile);
  void _recordMismatch(uint8_t reg, uint8_t expected, uint8_t observed);
  Status _read(uint8_t reg, uint8_t* data, size_t length, uint64_t nowMs);
  Status _writeByte(uint8_t reg, uint8_t value, uint64_t nowMs);
  Status _checkStart(const OperationTiming& timing) const;
  Status _checkReadyForVerifiedConfiguration(uint64_t nowMs) const;
  Status _checkFifoBypass(uint64_t nowMs);
  static bool _validDiagnosticRange(uint8_t startReg, size_t length);
  static bool _safeDiagnosticWrite(uint8_t reg, uint8_t value);

  DriverConfig _driverConfig = {};
  bool _bound = false;
  // A fresh binding and a transport failure can leave the device holding a low
  // byte until its matching high byte is consumed. Teardown proves no release.
  bool _outputMayBeHeld = true;

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
  uint8_t _fifoDiscardMask = 0;
  bool _fifoContinuityLost = false;
  uint64_t _counterEpoch = 0;
  bool _counterContinuityLost = true;
  bool _bankMayBeUnknown = false;
  uint8_t _counterSetupStep = 0;
  bool _timestampResetNeeded = false;
  CounterResetRequest _counterResetRequest = {};
  uint8_t _routeMaskStep = 0;
  uint8_t _routeEnableStep = 0;
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
  FifoReadRequest _fifoReadRequest = {};
  RawAxes _fifoAxes = {};
  uint16_t _fifoReadTarget = 0;
  uint16_t _fifoPatternWords = 0;
  uint8_t _fifoAxesCount = 0;
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
