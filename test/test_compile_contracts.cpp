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

static_assert(cmd::REG_SENSOR_SYNC_TIME_FRAME == 0x04u,
              "SENSOR_SYNC_TIME_FRAME must match the LSM6DS3TR-C register map");
static_assert(cmd::REG_SENSOR_SYNC_RES_RATIO == 0x05u,
              "SENSOR_SYNC_RES_RATIO must match the LSM6DS3TR-C register map");
static_assert(static_cast<uint8_t>(GyroHpfMode::HZ_0_016) == 0u,
              "16 mHz gyro HPF must encode HPM_G=00");
static_assert(static_cast<uint8_t>(GyroHpfMode::HZ_0_065) == 1u,
              "65 mHz gyro HPF must encode HPM_G=01");
static_assert(static_cast<uint8_t>(GyroHpfMode::HZ_0_260) == 2u,
              "260 mHz gyro HPF must encode HPM_G=10");
static_assert(static_cast<uint8_t>(GyroHpfMode::HZ_1_040) == 3u,
              "1.040 Hz gyro HPF must encode HPM_G=11");

static_assert(std::is_trivially_copyable<Status>::value,
              "Status must remain fixed-memory and trivially copyable");
static_assert(std::is_trivially_copyable<DriverConfig>::value,
              "The zero-I/O transport binding must be trivially copyable");
static_assert(std::is_trivially_copyable<DeviceProfile>::value,
              "A profile must remain a fixed-memory value type");
static_assert(std::is_trivially_copyable<OperationToken>::value,
              "Operation tokens must remain trivially copyable");
static_assert(std::is_same<decltype(OperationToken{}.value), uint64_t>::value,
              "Operation tokens must remain 64-bit correlation identities");
static_assert(std::is_same<decltype(RawSampleResult{}.sequence), uint64_t>::value,
              "Raw sample sequences must remain 64-bit");
static_assert(std::is_same<decltype(ConvertedSample{}.sequence), uint64_t>::value,
              "Converted sample sequences must remain 64-bit");
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
