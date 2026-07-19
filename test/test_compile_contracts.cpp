/// @file test_compile_contracts.cpp
/// @brief Compile-time ownership, layout, sizing, and public-header contracts.

#include <cstdint>
#include <type_traits>

#define DISABLED 0x4C534D36
#include "LSM6DS3TR/Config.h"

#ifndef DISABLED
#error "Including Config.h must not undefine an application macro"
#endif

static_assert(DISABLED == 0x4C534D36,
              "Including Config.h must preserve an application macro value");
#undef DISABLED

#include "LSM6DS3TR/LSM6DS3TR.h"

using namespace LSM6DS3TR;

static_assert(std::is_trivially_copyable<Status>::value,
              "Status must remain fixed-memory and trivially copyable");
static_assert(std::is_trivially_copyable<DriverConfig>::value,
              "The zero-I/O transport binding must be trivially copyable");
static_assert(std::is_trivially_copyable<DeviceProfile>::value,
              "A profile must remain a fixed-memory value type");
static_assert(std::is_trivially_copyable<OperationToken>::value,
              "Operation tokens must remain trivially copyable");
static_assert(std::is_trivially_copyable<PollResult>::value,
              "Poll results must remain fixed-memory values");
static_assert(std::is_trivially_copyable<OperationResult>::value,
              "Terminal results must remain fixed-memory values");
static_assert(!std::is_copy_constructible<LSM6DS3TR::LSM6DS3TR>::value,
              "A live non-owning driver must not be copy constructible");
static_assert(!std::is_copy_assignable<LSM6DS3TR::LSM6DS3TR>::value,
              "A live non-owning driver must not be copy assignable");
static_assert(!std::is_move_constructible<LSM6DS3TR::LSM6DS3TR>::value,
              "A live non-owning driver must not be move constructible");
static_assert(!std::is_move_assignable<LSM6DS3TR::LSM6DS3TR>::value,
              "A live non-owning driver must not be move assignable");
static_assert(sizeof(Status) <= 16u,
              "Status grew beyond the public fixed-memory contract");
static_assert(sizeof(DriverConfig) <= 40u,
              "DriverConfig grew beyond the public fixed-memory contract");
static_assert(sizeof(DeviceProfile) <= 32u,
              "DeviceProfile grew beyond the public fixed-memory contract");
static_assert(sizeof(RawSampleResult) <= 48u,
              "RawSampleResult grew beyond the owner payload contract");
static_assert(sizeof(PollResult) <= 32u,
              "PollResult grew beyond the owner polling contract");
static_assert(sizeof(OperationResult) <= 320u,
              "OperationResult grew beyond its fixed-memory maintenance bound");
static_assert(sizeof(LSM6DS3TR::LSM6DS3TR) <= 1024u,
              "Driver state grew beyond its fixed-memory bound");
