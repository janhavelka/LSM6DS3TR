/// @file Status.h
/// @brief Error codes and status handling for LSM6DS3TR driver
#pragma once

#include <cstdint>

namespace LSM6DS3TR {

/// Error codes for all LSM6DS3TR operations
enum class Err : uint8_t {
  OK = 0,                    ///< Operation successful
  NOT_INITIALIZED,           ///< begin() not called
  INVALID_CONFIG,            ///< Invalid configuration parameter
  I2C_ERROR,                 ///< I2C communication failure
  TIMEOUT,                   ///< Operation timed out
  INVALID_PARAM,             ///< Invalid parameter value
  DEVICE_NOT_FOUND,          ///< Device not responding on I2C bus
  CHIP_ID_MISMATCH,          ///< WHO_AM_I != 0x6A (not an LSM6DS3TR-C)
  MEASUREMENT_NOT_READY,     ///< Sample not yet available
  BUSY,                      ///< Device is busy
  IN_PROGRESS,               ///< Operation scheduled; call tick() to complete
  SELF_TEST_FAIL,            ///< Self-test result out of range

  // I2C transport details (append-only to preserve existing values)
  I2C_NACK_ADDR,             ///< I2C address not acknowledged
  I2C_NACK_DATA,             ///< I2C data byte not acknowledged
  I2C_TIMEOUT,               ///< I2C transaction timeout
  I2C_BUS                    ///< I2C bus error (arbitration lost, etc.)
};

/// Status structure returned by all fallible operations
struct Status {
  Err code = Err::OK;
  int32_t detail = 0;        ///< Implementation-specific detail (e.g., I2C error code)
  const char* msg = "";      ///< Static string describing the error

  constexpr Status() = default;
  constexpr Status(Err c, int32_t d, const char* m) : code(c), detail(d), msg(m) {}

  /// @return true if operation succeeded
  constexpr bool ok() const { return code == Err::OK; }

  /// @return true if operation in progress (not a failure)
  constexpr bool inProgress() const { return code == Err::IN_PROGRESS; }

  /// Create a success status
  static constexpr Status Ok() { return Status{Err::OK, 0, "OK"}; }

  /// Create an error status
  static constexpr Status Error(Err err, const char* message, int32_t detailCode = 0) {
    return Status{err, detailCode, message};
  }
};

} // namespace LSM6DS3TR
