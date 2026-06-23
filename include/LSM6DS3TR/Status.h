/// @file Status.h
/// @brief Error codes and status handling for LSM6DS3TR driver
#pragma once

#include <cstdint>

namespace LSM6DS3TR {

/// @brief Error codes for all LSM6DS3TR operations.
enum class Err : uint8_t {
  OK = 0,                ///< Operation successful
  NOT_INITIALIZED,       ///< begin() not called
  INVALID_CONFIG,        ///< Invalid configuration parameter
  I2C_ERROR,             ///< I2C communication failure
  TIMEOUT,               ///< Operation timed out
  INVALID_PARAM,         ///< Invalid parameter value
  DEVICE_NOT_FOUND,      ///< Device not responding on I2C bus
  CHIP_ID_MISMATCH,      ///< WHO_AM_I != 0x6A
  MEASUREMENT_NOT_READY, ///< Sample not yet available
  CONVERSION_NOT_READY = MEASUREMENT_NOT_READY, ///< Cross-library alias
  BUSY,                  ///< Device is busy
  IN_PROGRESS,           ///< Operation scheduled; call tick() to complete
  SELF_TEST_FAIL,        ///< Self-test result out of range

  // I2C transport details (append-only)
  I2C_NACK_ADDR,         ///< I2C address not acknowledged
  I2C_NACK_DATA,         ///< I2C data byte not acknowledged
  I2C_TIMEOUT,           ///< I2C transaction timeout
  I2C_BUS,               ///< I2C bus error
  FIFO_EMPTY,            ///< FIFO has no unread samples
  OFFLINE,               ///< Driver is offline; call recover()
  I2C_BUSY,              ///< Transport or shared bus temporarily busy
  FIFO_OVERRUN,          ///< FIFO overrun observed; data freshness is not guaranteed
  CALIBRATION_UNSTABLE,  ///< Calibration samples exceeded stillness limits
  CALIBRATION_ORIENTATION ///< Accelerometer calibration orientation is invalid
};

/// @brief Status structure returned by all fallible operations.
struct Status {
  Err code = Err::OK;     ///< Status code.
  int32_t detail = 0;     ///< Implementation-specific detail, such as transport error code.
  const char* msg = "";   ///< Static message string.

  constexpr Status() = default;
  constexpr Status(Err c, int32_t d, const char* m) : code(c), detail(d), msg(m) {}

  /// @brief Check whether the operation succeeded.
  /// @return true when code is OK.
  constexpr bool ok() const { return code == Err::OK; }

  /// @brief Check whether the status matches an error code.
  /// @return true when code equals @p err.
  constexpr bool is(Err err) const { return code == err; }

  /// @brief Check whether the operation is still pending.
  /// @return true when code is IN_PROGRESS.
  constexpr bool inProgress() const { return code == Err::IN_PROGRESS; }

  /// @brief Create a success status.
  static constexpr Status Ok() { return Status{Err::OK, 0, "OK"}; }

  /// @brief Create an error status.
  /// @param err Error code.
  /// @param message Static message string.
  /// @param detailCode Optional implementation-specific detail.
  /// @return Error status.
  static constexpr Status Error(Err err, const char* message, int32_t detailCode = 0) {
    return Status{err, detailCode, message};
  }
};

}  // namespace LSM6DS3TR
