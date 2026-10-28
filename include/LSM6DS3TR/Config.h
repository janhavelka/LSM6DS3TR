/// @file Config.h
/// @brief Transport binding and replayable LSM6DS3TR-C device profile.
#pragma once

#include <cstddef>
#include <cstdint>

#include "LSM6DS3TR/Status.h"

namespace LSM6DS3TR {

/// @brief Execute one complete, synchronous, timeout-bounded I2C write.
/// @param address Seven-bit device address.
/// @param data Bytes to transmit, including the register address.
/// @param length Number of bytes at @p data.
/// @param timeoutMs Per-attempt timeout selected by DriverConfig.
/// @param user Opaque application context from DriverConfig::i2cUser.
/// @return Status::Ok() on confirmed success, otherwise a mapped transport error.
/// @note The callback performs exactly one physical attempt. The application
/// owns the bus, locking, retries, recovery, and scheduling.
using I2cWriteFn = Status (*)(uint8_t address, const uint8_t* data, size_t length,
                              uint32_t timeoutMs, void* user);

/// @brief Execute one complete write-then-read I2C transaction.
/// @param address Seven-bit device address.
/// @param txData Bytes to transmit, including the register address.
/// @param txLength Number of bytes at @p txData.
/// @param rxData Caller-owned receive buffer.
/// @param rxLength Number of bytes to receive.
/// @param timeoutMs Per-attempt timeout selected by DriverConfig.
/// @param user Opaque application context from DriverConfig::i2cUser.
/// @return Status::Ok() on confirmed success, otherwise a mapped transport error.
/// @note The callback performs exactly one physical attempt and must not retry.
using I2cWriteReadFn = Status (*)(uint8_t address, const uint8_t* txData, size_t txLength,
                                  uint8_t* rxData, size_t rxLength,
                                  uint32_t timeoutMs, void* user);

/// @brief Supported seven-bit addresses selected by the SA0 pin.
enum class SensorAddress : uint8_t {
  SA0_GND = 0x6A,  ///< SA0 tied low.
  SA0_VDD = 0x6B  ///< SA0 tied high.
};

/// @brief Zero-I/O transport binding. No sensor settings are applied by bind().
struct DriverConfig {
  I2cWriteFn i2cWrite = nullptr;          ///< Required single-attempt write callback.
  I2cWriteReadFn i2cWriteRead = nullptr;  ///< Required single-attempt combined callback.
  void* i2cUser = nullptr;                ///< Opaque context forwarded unchanged.
  SensorAddress address = SensorAddress::SA0_GND;  ///< Bound device address.
  uint32_t i2cTimeoutMs = 50;  ///< Nonzero timeout passed to every callback.
};

/// @brief Output data-rate encoding shared by accelerometer and gyroscope.
/// @note HZ_1_6 is valid only for the accelerometer in low-power/normal mode.
enum class Odr : uint8_t {
  POWER_DOWN = 0,  ///< Sensor disabled.
  HZ_12_5 = 1,    ///< 12.5 Hz.
  HZ_26 = 2,      ///< 26 Hz.
  HZ_52 = 3,      ///< 52 Hz.
  HZ_104 = 4,     ///< 104 Hz.
  HZ_208 = 5,     ///< 208 Hz.
  HZ_416 = 6,     ///< 416 Hz; high-performance mode only.
  HZ_833 = 7,     ///< 833 Hz; high-performance mode only.
  HZ_1660 = 8,    ///< 1.66 kHz; high-performance mode only.
  HZ_3330 = 9,    ///< 3.33 kHz; high-performance mode only.
  HZ_6660 = 10,   ///< 6.66 kHz; high-performance mode only.
  HZ_1_6 = 11     ///< 1.6 Hz accelerometer low-power/normal mode only.
};

/// @brief Accelerometer full-scale range.
enum class AccelFs : uint8_t {
  G_2 = 0,   ///< +/-2 g.
  G_16 = 1,  ///< +/-16 g.
  G_4 = 2,   ///< +/-4 g.
  G_8 = 3    ///< +/-8 g.
};

/// @brief Gyroscope full-scale range.
enum class GyroFs : uint8_t {
  DPS_125 = 0xFF,  ///< +/-125 degrees per second.
  DPS_250 = 0,     ///< +/-250 degrees per second.
  DPS_500 = 1,     ///< +/-500 degrees per second.
  DPS_1000 = 2,    ///< +/-1000 degrees per second.
  DPS_2000 = 3     ///< +/-2000 degrees per second.
};

/// @brief Accelerometer power/performance mode.
enum class AccelPowerMode : uint8_t {
  HIGH_PERFORMANCE = 0,  ///< High-performance operating mode.
  LOW_POWER_NORMAL = 1   ///< Low-power/normal mode, limited to 208 Hz.
};

/// @brief Gyroscope power/performance mode.
enum class GyroPowerMode : uint8_t {
  HIGH_PERFORMANCE = 0,  ///< High-performance operating mode.
  LOW_POWER_NORMAL = 1   ///< Low-power/normal mode, limited to 208 Hz.
};

/// @brief Register-level gyroscope high-pass cutoff encoding.
/// @note HPF configuration supports explicit unqualified direct reads only.
enum class GyroHpfMode : uint8_t {
  HZ_0_016 = 0,  ///< 0.016 Hz cutoff encoding.
  HZ_0_065 = 1,  ///< 0.065 Hz cutoff encoding.
  HZ_0_260 = 2,  ///< 0.260 Hz cutoff encoding.
  HZ_1_040 = 3   ///< 1.040 Hz cutoff encoding.
};

/// @brief Accelerometer user-offset scale.
enum class AccelOffsetWeight : uint8_t {
  MG_1 = 0,  ///< Nominal 1 mg/LSB; exactly 2^-10 g = 0.9765625 mg/LSB.
  MG_16 = 1  ///< Nominal 16 mg/LSB; exactly 2^-6 g = 15.625 mg/LSB.
};

/// @brief Signed sensor-native accelerometer user offsets.
/// @note Each value must be in -127..127; -128 is a reserved encoding.
/// @note Register values are added to X/Y output and subtracted from Z output.
struct AccelUserOffset {
  int8_t x = 0;  ///< X-axis register value.
  int8_t y = 0;  ///< Y-axis register value.
  int8_t z = 0;  ///< Z-axis register value.
};

/// @brief LPF1 host-output bandwidth selection.
enum class AccelLpf1Bandwidth : uint8_t {
  ODR_DIV_2 = 0,  ///< ODR/2 in high performance; 740 Hz in low-power/normal.
  ODR_DIV_4       ///< ODR/4 in every admitted power mode.
};

/// @brief Shared LPF2/high-pass cutoff register encoding.
enum class AccelHpcf : uint8_t {
  DIV_50 = 0,  ///< LPF2 ODR/50, or slope ODR/4 on the high-pass path.
  DIV_100,     ///< LPF2/HP ODR/100; also the canonical reference-mode code.
  DIV_9,       ///< LPF2/HP ODR/9.
  DIV_400      ///< LPF2/HP ODR/400.
};

/// @brief Input to the accelerometer composite filter.
enum class AccelCompositeInput : uint8_t {
  LOW_LATENCY = 0,  ///< LPF1 ODR/2 branch; required for slope/HP/reference output.
  LOW_NOISE        ///< LPF1 ODR/4 branch, available for the LPF2 path.
};

/// @brief Analog anti-alias bandwidth in high-performance mode.
enum class AccelAnalogBandwidth : uint8_t {
  AUTOMATIC = 0,  ///< 400 Hz below 1.66 kHz ODR; 1500 Hz at higher ODRs.
  HZ_400         ///< Force 400 Hz; admitted only at high-performance ODR >=1.66 kHz.
};

/// @brief Gyroscope LPF1 bandwidth codes from AN5130 Tables 11 and 16.
enum class GyroLpf1Bandwidth : uint8_t {
  FTYPE_00 = 0,  ///< CTRL6_C.FTYPE=00.
  FTYPE_01,      ///< CTRL6_C.FTYPE=01.
  FTYPE_10,      ///< CTRL6_C.FTYPE=10.
  FTYPE_11       ///< CTRL6_C.FTYPE=11.
};

/// @brief Coherent accelerometer host-output and orientation filter choices.
/// @note The compatible selectors are mutually exclusive: neither selects LPF1,
/// lpf2Enabled selects LPF2, highPassSlopeEnabled selects slope/HP according to
/// hpcf. Reference mode instead subtracts the captured initial vector; it is
/// not an ordinary high-pass stream. Inactive fields require canonical defaults.
struct AccelFilterConfig {
  bool lpf2Enabled = false;  ///< Host LPF2 output; excludes high-pass/reference output.
  bool highPassSlopeEnabled = false;  ///< Host slope/HP output, or reference subtraction.
  bool lowPassOn6d = false;  ///< Orientation uses the LPF2 branch; requires a routed 6D/4D profile.
  AccelHpcf hpcf = AccelHpcf::DIV_50;  ///< Shared LPF2/HP code; DIV_50 means slope on HP path.
  AccelLpf1Bandwidth lpf1Bandwidth = AccelLpf1Bandwidth::ODR_DIV_2;  ///< Direct LPF1 output only.
  AccelCompositeInput compositeInput = AccelCompositeInput::LOW_LATENCY;  ///< LPF2 input branch.
  AccelAnalogBandwidth analogBandwidth = AccelAnalogBandwidth::AUTOMATIC;  ///< Analog path policy.
  bool referenceEnabled = false;  ///< Requires HP selector, DIV_100, no LPF2/6D LPF2; discards first vector.
};

/// @brief Gyroscope filter choices represented by DeviceProfile.
struct GyroFilterConfig {
  bool lpf1Enabled = false;  ///< Enable LPF1 in high-performance mode.
  bool highPassEnabled = false;  ///< HPF: high performance only, explicitly unqualified direct output.
  GyroHpfMode highPassMode = GyroHpfMode::HZ_0_016;  ///< Register encoding when enabled.
  GyroLpf1Bandwidth lpf1Bandwidth = GyroLpf1Bandwidth::FTYPE_00;  ///< LPF1 bandwidth; default when bypassed.
};

/// @brief Supported internal-trigger, full-word motion FIFO modes.
enum class FifoMode : uint8_t {
  BYPASS = 0,      ///< Collection disabled; selecting this mode clears FIFO.
  FIFO = 1,        ///< Stop collection when the physical FIFO is full.
  CONTINUOUS = 6   ///< Replace the oldest words when the physical FIFO is full.
};

/// @brief Dataset decimation register encodings (not numeric divisors).
enum class FifoDecimation : uint8_t {
  OFF = 0,       ///< Dataset excluded from the FIFO pattern.
  EVERY_1 = 1,   ///< Store every trigger.
  EVERY_2 = 2,   ///< Store every second trigger.
  EVERY_3 = 3,   ///< Store every third trigger.
  EVERY_4 = 4,   ///< Store every fourth trigger.
  EVERY_8 = 5,   ///< Store every eighth trigger.
  EVERY_16 = 6,  ///< Store every sixteenth trigger.
  EVERY_32 = 7   ///< Store every thirty-second trigger.
};

/// @brief Mutually exclusive interpretation of the six-byte fourth dataset.
enum class FifoFourthDataset : uint8_t {
  OFF = 0,         ///< Fourth dataset excluded.
  TEMPERATURE,     ///< Signed temperature in bytes 2/3; other bytes unused.
  TIMESTAMP_STEP   ///< Captured raw 24-bit timer and 16-bit step counter.
};

/// @brief Shared trigger for every enabled FIFO dataset.
enum class FifoTrigger : uint8_t {
  INTERNAL = 0,  ///< Internal sensor/FIFO ODR trigger.
  STEP_EVENT     ///< STEP_DETECTED events, including accompanying motion records.
};

/// @brief Sensor timestamp tick resolution; not an owner-clock mapping.
enum class TimestampResolution : uint8_t {
  MS_6_4 = 0,  ///< 6400 microseconds per tick.
  US_25        ///< 25 microseconds per tick.
};

/// @brief Replayable internal motion and optional fourth-dataset FIFO policy.
/// @note Disabled profiles require the canonical default values below. Enabled
/// profiles require at least one active dataset and an active internal sensor,
/// including when preparing BYPASS. Selected sensors must supply at least the
/// decimated internal trigger cadence. Internal triggering requires a
/// 12.5..6660 Hz FIFO ODR. Step triggering requires the timestamp/step dataset
/// and accelerometer >=26 Hz; its ODR may be power-down because events supply
/// the shared trigger. Accelerometer 1.6 Hz and gyroscope sleep are not
/// admitted with FIFO, including when that sensor's dataset is excluded.
/// @note External datasets, triggered modes, high-byte packing and
/// stop-on-threshold are deliberately absent from this full-word contract.
struct FifoProfile {
  bool enabled = false;  ///< Enable the typed FIFO interpretation profile.
  FifoMode mode = FifoMode::BYPASS;  ///< Collection mode, activated after settling.
  Odr odr = Odr::POWER_DOWN;  ///< Maximum internal FIFO trigger frequency.
  FifoDecimation gyroDecimation = FifoDecimation::OFF;  ///< Dataset 1 cadence.
  FifoDecimation accelDecimation = FifoDecimation::OFF;  ///< Dataset 2 cadence.
  uint16_t watermarkWords = 0;  ///< 0..2047 words; zero is always reached, not disabled.
  FifoFourthDataset fourthDataset = FifoFourthDataset::OFF;  ///< One fourth-record schema.
  FifoDecimation fourthDecimation = FifoDecimation::OFF;  ///< Dataset 4 trigger divisor.
  FifoTrigger trigger = FifoTrigger::INTERNAL;  ///< Shared by motion and fourth records.
  TimestampResolution timestampResolution = TimestampResolution::MS_6_4;  ///< Timer policy.
};

/// @brief Shared active level of both interrupt pads.
enum class InterruptPolarity : uint8_t {
  ACTIVE_HIGH = 0,  ///< Active-high signaling.
  ACTIVE_LOW        ///< Active-low signaling.
};

/// @brief Shared electrical output driver of both interrupt pads.
enum class InterruptDrive : uint8_t {
  PUSH_PULL = 0,  ///< Push-pull pad drive.
  OPEN_DRAIN_PAD  ///< Open-drain; application supplies appropriate pull resistors.
};

/// @brief Motion data-ready pad behavior, separate from embedded event latching.
enum class DataReadyMode : uint8_t {
  LATCHED = 0,  ///< Pad remains asserted until the corresponding output is read.
  PULSED        ///< 75 us motion DRDY pad pulses; status flags remain latched.
};

/// @brief Supported accelerometer orientation engine dimensions.
enum class OrientationMode : uint8_t {
  OFF = 0,  ///< Orientation engine has no production owner.
  SIX_D,    ///< Recognize X/Y/Z positive and negative orientation conditions.
  FOUR_D    ///< Recognize X/Y conditions; Z detection is disabled.
};

/// @brief Exact silicon orientation thresholds, valid at every accel full scale.
enum class OrientationThreshold : uint8_t {
  DEG_80 = 0,  ///< 80 degrees, register code 00.
  DEG_70,      ///< 70 degrees, register code 01.
  DEG_60,      ///< 60 degrees, register code 10.
  DEG_50       ///< 50 degrees, register code 11.
};

/// @brief Typed source requests for one physical interrupt pin.
/// @note Validation rejects impossible pin/source pairs rather than translating
/// them into another route. INT2 combining does not change direct pin support.
struct InterruptRoutes {
  bool accelReady = false;  ///< Accel DRDY on either pin; requires active accel.
  bool gyroReady = false;  ///< Gyro DRDY on either pin; requires active nonsleeping gyro.
  bool temperatureReady = false;  ///< Direct INT2 only; requires an active sensor.
  bool fifoWatermark = false;  ///< Either pin; requires a collecting typed FIFO.
  bool fifoFull = false;  ///< Either pin; FIFO_FULL_SMART, not an exact occupancy claim.
  bool fifoOverrun = false;  ///< Either pin; no sticky loss-history guarantee.
  bool stepDetected = false;  ///< Direct INT1 only; requires the fixed pedometer profile.
  bool stepOverflow = false;  ///< Direct INT2 only; requires the fixed pedometer profile.
  bool timestampEndCounter = false;  ///< Direct INT1 only; no readable event-status flag.
  bool orientation = false;  ///< Either pin; requires the typed 6D/4D profile.
};

/// @brief Orientation detector with LPF1 ODR/2 or explicitly selected LPF2 input.
/// @note Detection uses two consecutive samples; no minimum 26 Hz ODR is imposed
/// on this basic engine. AccelFilterConfig::lowPassOn6d selects its LPF2 input;
/// that branch is independent of the final host-output mux.
struct OrientationProfile {
  OrientationMode mode = OrientationMode::OFF;  ///< Active accel and a route required when on.
  OrientationThreshold threshold = OrientationThreshold::DEG_80;  ///< Exact degree choice.
};

/// @brief Replayable shared electrical, source routing and 6D/4D policy.
/// @note Disabled profiles require the canonical defaults. GPIO configuration,
/// interrupts and scheduling remain application responsibilities; no API is ISR-safe.
struct InterruptProfile {
  bool enabled = false;  ///< Admit the complete typed policy instead of canonical disabled state.
  InterruptPolarity polarity = InterruptPolarity::ACTIVE_HIGH;  ///< Shared pad active level.
  InterruptDrive drive = InterruptDrive::PUSH_PULL;  ///< Shared pad electrical driver.
  DataReadyMode dataReadyMode = DataReadyMode::LATCHED;  ///< Motion DRDY pad behavior.
  bool latchEvents = false;  ///< Global LIR; source latching also requires its route.
  bool int2OnInt1 = false;  ///< Combine enabled INT2 signals onto INT1; INT2 routes remain real.
  InterruptRoutes int1 = {};  ///< Direct INT1 source choices.
  InterruptRoutes int2 = {};  ///< Direct INT2 source choices.
  OrientationProfile orientation = {};  ///< Complete supported orientation policy.
};

/// @brief Complete replayable state owned by the production driver.
struct DeviceProfile {
  Odr accelOdr = Odr::HZ_104;  ///< Accelerometer output data rate.
  AccelFs accelFullScale = AccelFs::G_2;  ///< Accelerometer measurement range.
  AccelPowerMode accelPowerMode = AccelPowerMode::HIGH_PERFORMANCE;  ///< Accelerometer mode.
  Odr gyroOdr = Odr::HZ_104;  ///< Gyroscope output data rate.
  GyroFs gyroFullScale = GyroFs::DPS_250;  ///< Gyroscope measurement range.
  GyroPowerMode gyroPowerMode = GyroPowerMode::HIGH_PERFORMANCE;  ///< Gyroscope mode.
  AccelFilterConfig accelFilter = {};  ///< Accelerometer filter policy.
  GyroFilterConfig gyroFilter = {};  ///< Gyroscope filter policy.
  bool gyroSleepEnabled = false;  ///< Put a configured gyroscope into sleep mode.
  bool blockDataUpdate = true;  ///< Must remain enabled for managed burst reads.
  AccelOffsetWeight accelOffsetWeight = AccelOffsetWeight::MG_1;  ///< Offset scale.
  AccelUserOffset accelUserOffset = {};  ///< Signed hardware offset values.
  FifoProfile fifo = {};  ///< Typed motion FIFO policy; default is disabled.
  InterruptProfile interrupts = {};  ///< Typed interrupt policy; default disables all routes.
};

/// @brief Quality limits for bounded bias calibration.
struct CalibrationLimits {
  float accelMaxPeakToPeakG = 0.08f;  ///< Per-axis acceleration stability limit.
  float gyroMaxPeakToPeakDps = 3.0f;  ///< Per-axis angular-rate stability limit.
};

}  // namespace LSM6DS3TR
