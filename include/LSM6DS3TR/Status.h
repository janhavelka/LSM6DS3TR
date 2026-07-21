/// @file Status.h
/// @brief Typed status returned by the LSM6DS3TR-C driver.
#pragma once

#include <cstdint>

/// @brief Framework-neutral public API for the LSM6DS3TR-C driver.
namespace LSM6DS3TR {

/// @brief Error and progress codes. Values are append-only within this API major version.
enum class Err : uint8_t {
  OK = 0,                    ///< Operation or helper succeeded.
  NOT_BOUND,                 ///< No transport is currently bound.
  INVALID_CONFIG,            ///< Driver or profile configuration is invalid.
  INVALID_PARAM,             ///< A call argument is outside its valid domain.
  BUSY,                      ///< Another operation or pending result owns the slot.
  IN_PROGRESS,               ///< Operation was accepted or remains active.
  RESULT_PENDING,            ///< A terminal result must be consumed first.
  RESULT_NOT_AVAILABLE,      ///< No matching active or terminal result exists.
  STALE_RESULT,              ///< The supplied token does not identify the result.
  DEADLINE_EXPIRED,          ///< Absolute operation deadline was reached.
  CANCELLED,                 ///< Caller cancelled the accepted operation.
  CONFIGURATION_UNKNOWN,     ///< Verified device configuration is unavailable.
  CONFIGURATION_MISMATCH,    ///< Register readback differed from the desired image.
  SETTLING,                  ///< Configuration is verified but output is not yet valid.
  DATA_NOT_READY,            ///< Bounded readiness checks were exhausted.
  UNSUPPORTED_PROFILE,       ///< Requested feature is outside the production profile.
  OPERATION_INDETERMINATE,   ///< Hardware effect or restoration cannot be proved.
  TRANSACTION_LIMIT_EXCEEDED, ///< Hard callback ceiling was reached.
  DEVICE_NOT_FOUND,          ///< Transport completed but no device was present.
  CHIP_ID_MISMATCH,          ///< WHO_AM_I did not identify an LSM6DS3TR-C.
  SELF_TEST_FAIL,            ///< Built-in sensor self-test limits were not met.
  CALIBRATION_UNSTABLE,      ///< Calibration samples exceeded stability limits.
  CALIBRATION_ORIENTATION,   ///< Fixture gravity did not match the requested vector.
  FIFO_EMPTY,                ///< FIFO contained no unread word.
  FIFO_OVERRUN,              ///< FIFO data loss was observed.
  I2C_ERROR,                 ///< Generic transport error.
  I2C_NACK_ADDR,             ///< Address phase was not acknowledged.
  I2C_NACK_DATA,             ///< Data phase was not acknowledged.
  I2C_TIMEOUT,               ///< Transport callback timed out.
  I2C_BUS,                   ///< Bus-level transport fault.
  I2C_BUSY                   ///< Transport owner reported the bus busy.
};

/// @brief Status structure returned by every fallible public operation.
struct Status {
  Err code = Err::OK;  ///< Stable typed outcome.
  int32_t detail = 0;  ///< Context-specific numeric detail; zero when unused.
  const char* msg = "";  ///< Static string only; never owned by the caller.

  /// @brief Construct an OK status with no detail.
  constexpr Status() = default;

  /// @brief Construct a status value without allocating memory.
  /// @param c Typed outcome.
  /// @param d Context-specific numeric detail.
  /// @param m Static-lifetime diagnostic string.
  constexpr Status(Err c, int32_t d, const char* m) : code(c), detail(d), msg(m) {}

  /// @return True only when code is Err::OK.
  constexpr bool ok() const { return code == Err::OK; }

  /// @param err Code to compare.
  /// @return True when code equals @p err.
  constexpr bool is(Err err) const { return code == err; }

  /// @return True only while an accepted operation remains active.
  constexpr bool inProgress() const { return code == Err::IN_PROGRESS; }

  /// @return Canonical successful status.
  static constexpr Status Ok() { return Status{Err::OK, 0, "OK"}; }

  /// @brief Build a failed or progress status using a static message.
  /// @param err Typed outcome.
  /// @param message Static-lifetime diagnostic string.
  /// @param detailCode Context-specific numeric detail.
  /// @return Constructed status value.
  static constexpr Status Error(Err err, const char* message, int32_t detailCode = 0) {
    return Status{err, detailCode, message};
  }
};

}  // namespace LSM6DS3TR
