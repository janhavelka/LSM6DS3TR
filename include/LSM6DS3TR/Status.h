/// @file Status.h
/// @brief Typed status returned by the LSM6DS3TR-C driver.
#pragma once

#include <cstdint>

namespace LSM6DS3TR {

/// @brief Error and progress codes. Values are append-only within this API major version.
enum class Err : uint8_t {
  OK = 0,
  NOT_BOUND,
  INVALID_CONFIG,
  INVALID_PARAM,
  BUSY,
  IN_PROGRESS,
  RESULT_PENDING,
  RESULT_NOT_AVAILABLE,
  STALE_RESULT,
  DEADLINE_EXPIRED,
  CANCELLED,
  CONFIGURATION_UNKNOWN,
  CONFIGURATION_MISMATCH,
  SETTLING,
  DATA_NOT_READY,
  UNSUPPORTED_PROFILE,
  OPERATION_INDETERMINATE,
  TRANSACTION_LIMIT_EXCEEDED,
  DEVICE_NOT_FOUND,
  CHIP_ID_MISMATCH,
  SELF_TEST_FAIL,
  CALIBRATION_UNSTABLE,
  CALIBRATION_ORIENTATION,
  FIFO_EMPTY,
  FIFO_OVERRUN,
  I2C_ERROR,
  I2C_NACK_ADDR,
  I2C_NACK_DATA,
  I2C_TIMEOUT,
  I2C_BUS,
  I2C_BUSY
};

/// @brief Status structure returned by every fallible public operation.
struct Status {
  Err code = Err::OK;
  int32_t detail = 0;
  const char* msg = "";  ///< Static string only; never owned by the caller.

  constexpr Status() = default;
  constexpr Status(Err c, int32_t d, const char* m) : code(c), detail(d), msg(m) {}

  constexpr bool ok() const { return code == Err::OK; }
  constexpr bool is(Err err) const { return code == err; }
  constexpr bool inProgress() const { return code == Err::IN_PROGRESS; }

  static constexpr Status Ok() { return Status{Err::OK, 0, "OK"}; }
  static constexpr Status Error(Err err, const char* message, int32_t detailCode = 0) {
    return Status{err, detailCode, message};
  }
};

}  // namespace LSM6DS3TR
