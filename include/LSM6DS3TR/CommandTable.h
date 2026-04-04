/// @file CommandTable.h
/// @brief Register addresses and bit definitions for LSM6DS3TR-C
#pragma once

#include <cstdint>

namespace LSM6DS3TR {
namespace cmd {

// ============================================================================
// Chip Identification
// ============================================================================

static constexpr uint8_t REG_WHO_AM_I       = 0x0F;
static constexpr uint8_t WHO_AM_I_VALUE     = 0x6A;

// ============================================================================
// Function Configuration Access
// ============================================================================

static constexpr uint8_t REG_FUNC_CFG_ACCESS = 0x01;

// ============================================================================
// Sensor Sync
// ============================================================================

static constexpr uint8_t REG_SENSOR_SYNC_TIME_FRAME = 0x04;
static constexpr uint8_t REG_SENSOR_SYNC_RES_RATIO  = 0x04;

// ============================================================================
// FIFO Control Registers
// ============================================================================

static constexpr uint8_t REG_FIFO_CTRL1 = 0x06;
static constexpr uint8_t REG_FIFO_CTRL2 = 0x07;
static constexpr uint8_t REG_FIFO_CTRL3 = 0x08;
static constexpr uint8_t REG_FIFO_CTRL4 = 0x09;
static constexpr uint8_t REG_FIFO_CTRL5 = 0x0A;

// ============================================================================
// DRDY Pulse Configuration
// ============================================================================

static constexpr uint8_t REG_DRDY_PULSE_CFG_G = 0x0B;

// ============================================================================
// Interrupt Control Registers
// ============================================================================

static constexpr uint8_t REG_INT1_CTRL = 0x0D;
static constexpr uint8_t REG_INT2_CTRL = 0x0E;

// ============================================================================
// Control Registers
// ============================================================================

static constexpr uint8_t REG_CTRL1_XL  = 0x10;  ///< Accel ODR + FS
static constexpr uint8_t REG_CTRL2_G   = 0x11;  ///< Gyro ODR + FS
static constexpr uint8_t REG_CTRL3_C   = 0x12;  ///< BDU, IF_INC, SW_RESET, BOOT
static constexpr uint8_t REG_CTRL4_C   = 0x13;  ///< SLEEP_G, I2C_disable, LPF1_SEL_G
static constexpr uint8_t REG_CTRL5_C   = 0x14;  ///< Self-test, rounding
static constexpr uint8_t REG_CTRL6_C   = 0x15;  ///< XL_HM_MODE, trigger/level
static constexpr uint8_t REG_CTRL7_G   = 0x16;  ///< G_HM_MODE, HP filter
static constexpr uint8_t REG_CTRL8_XL  = 0x17;  ///< Accel filter config
static constexpr uint8_t REG_CTRL9_XL  = 0x18;  ///< DEN, SOFT_EN
static constexpr uint8_t REG_CTRL10_C  = 0x19;  ///< Embedded functions enable

// ============================================================================
// Master Configuration
// ============================================================================

static constexpr uint8_t REG_MASTER_CONFIG = 0x1A;

// ============================================================================
// Status and Source Registers
// ============================================================================

static constexpr uint8_t REG_WAKE_UP_SRC = 0x1B;
static constexpr uint8_t REG_TAP_SRC     = 0x1C;
static constexpr uint8_t REG_D6D_SRC     = 0x1D;
static constexpr uint8_t REG_STATUS_REG  = 0x1E;

// ============================================================================
// Output Data Registers
// ============================================================================

// Temperature (16-bit, two's complement)
static constexpr uint8_t REG_OUT_TEMP_L  = 0x20;
static constexpr uint8_t REG_OUT_TEMP_H  = 0x21;

// Gyroscope output (3 × 16-bit, two's complement)
static constexpr uint8_t REG_OUTX_L_G   = 0x22;
static constexpr uint8_t REG_OUTX_H_G   = 0x23;
static constexpr uint8_t REG_OUTY_L_G   = 0x24;
static constexpr uint8_t REG_OUTY_H_G   = 0x25;
static constexpr uint8_t REG_OUTZ_L_G   = 0x26;
static constexpr uint8_t REG_OUTZ_H_G   = 0x27;

// Accelerometer output (3 × 16-bit, two's complement)
static constexpr uint8_t REG_OUTX_L_XL  = 0x28;
static constexpr uint8_t REG_OUTX_H_XL  = 0x29;
static constexpr uint8_t REG_OUTY_L_XL  = 0x2A;
static constexpr uint8_t REG_OUTY_H_XL  = 0x2B;
static constexpr uint8_t REG_OUTZ_L_XL  = 0x2C;
static constexpr uint8_t REG_OUTZ_H_XL  = 0x2D;

// Burst read ranges
static constexpr uint8_t REG_DATA_START_ALL = REG_OUT_TEMP_L;  ///< Temp + Gyro + Accel
static constexpr uint8_t DATA_LEN_ALL       = 14;               ///< 2 + 6 + 6 bytes

static constexpr uint8_t REG_DATA_START_GYRO = REG_OUTX_L_G;
static constexpr uint8_t DATA_LEN_GYRO       = 6;

static constexpr uint8_t REG_DATA_START_ACCEL = REG_OUTX_L_XL;
static constexpr uint8_t DATA_LEN_ACCEL       = 6;

// ============================================================================
// Sensor Hub Registers
// ============================================================================

static constexpr uint8_t REG_SENSORHUB1  = 0x2E;
static constexpr uint8_t REG_SENSORHUB12 = 0x39;

// ============================================================================
// FIFO Status Registers
// ============================================================================

static constexpr uint8_t REG_FIFO_STATUS1 = 0x3A;
static constexpr uint8_t REG_FIFO_STATUS2 = 0x3B;
static constexpr uint8_t REG_FIFO_STATUS3 = 0x3C;
static constexpr uint8_t REG_FIFO_STATUS4 = 0x3D;

static constexpr uint8_t REG_FIFO_DATA_OUT_L = 0x3E;
static constexpr uint8_t REG_FIFO_DATA_OUT_H = 0x3F;

// ============================================================================
// Timestamp Registers
// ============================================================================

static constexpr uint8_t REG_TIMESTAMP0 = 0x40;
static constexpr uint8_t REG_TIMESTAMP1 = 0x41;
static constexpr uint8_t REG_TIMESTAMP2 = 0x42;

// ============================================================================
// Step Counter / Timestamp
// ============================================================================

static constexpr uint8_t REG_STEP_TIMESTAMP_L = 0x49;
static constexpr uint8_t REG_STEP_TIMESTAMP_H = 0x4A;
static constexpr uint8_t REG_STEP_COUNTER_L   = 0x4B;
static constexpr uint8_t REG_STEP_COUNTER_H   = 0x4C;

// ============================================================================
// Sensor Hub (secondary set)
// ============================================================================

static constexpr uint8_t REG_SENSORHUB13 = 0x4D;
static constexpr uint8_t REG_SENSORHUB18 = 0x52;

// ============================================================================
// Function Source Registers
// ============================================================================

static constexpr uint8_t REG_FUNC_SRC1     = 0x53;
static constexpr uint8_t REG_FUNC_SRC2     = 0x54;
static constexpr uint8_t REG_WRIST_TILT_IA = 0x55;

// ============================================================================
// Event Configuration Registers
// ============================================================================

static constexpr uint8_t REG_TAP_CFG       = 0x58;
static constexpr uint8_t REG_TAP_THS_6D    = 0x59;
static constexpr uint8_t REG_INT_DUR2      = 0x5A;
static constexpr uint8_t REG_WAKE_UP_THS   = 0x5B;
static constexpr uint8_t REG_WAKE_UP_DUR   = 0x5C;
static constexpr uint8_t REG_FREE_FALL     = 0x5D;
static constexpr uint8_t REG_MD1_CFG       = 0x5E;
static constexpr uint8_t REG_MD2_CFG       = 0x5F;

// ============================================================================
// Master Command / Sensor Sync
// ============================================================================

static constexpr uint8_t REG_MASTER_CMD_CODE           = 0x60;
static constexpr uint8_t REG_SENS_SYNC_SPI_ERROR_CODE  = 0x61;

// ============================================================================
// Raw Magnetometer Output (Sensor Hub)
// ============================================================================

static constexpr uint8_t REG_OUT_MAG_RAW_X_L = 0x66;
static constexpr uint8_t REG_OUT_MAG_RAW_X_H = 0x67;
static constexpr uint8_t REG_OUT_MAG_RAW_Y_L = 0x68;
static constexpr uint8_t REG_OUT_MAG_RAW_Y_H = 0x69;
static constexpr uint8_t REG_OUT_MAG_RAW_Z_L = 0x6A;
static constexpr uint8_t REG_OUT_MAG_RAW_Z_H = 0x6B;

// ============================================================================
// User Offset Registers
// ============================================================================

static constexpr uint8_t REG_X_OFS_USR = 0x73;
static constexpr uint8_t REG_Y_OFS_USR = 0x74;
static constexpr uint8_t REG_Z_OFS_USR = 0x75;

// ============================================================================
// CTRL1_XL (10h) Bit Definitions — Accelerometer Control
// ============================================================================

static constexpr uint8_t BIT_ODR_XL       = 4;
static constexpr uint8_t MASK_ODR_XL      = 0xF0;
static constexpr uint8_t BIT_FS_XL        = 2;
static constexpr uint8_t MASK_FS_XL       = 0x0C;
static constexpr uint8_t BIT_LPF1_BW_SEL  = 1;
static constexpr uint8_t MASK_LPF1_BW_SEL = 0x02;
static constexpr uint8_t BIT_BW0_XL       = 0;
static constexpr uint8_t MASK_BW0_XL      = 0x01;

// ============================================================================
// CTRL2_G (11h) Bit Definitions — Gyroscope Control
// ============================================================================

static constexpr uint8_t BIT_ODR_G   = 4;
static constexpr uint8_t MASK_ODR_G  = 0xF0;
static constexpr uint8_t BIT_FS_G    = 2;
static constexpr uint8_t MASK_FS_G   = 0x0C;
static constexpr uint8_t BIT_FS_125  = 1;
static constexpr uint8_t MASK_FS_125 = 0x02;

// ============================================================================
// CTRL3_C (12h) Bit Definitions — Control 3
// ============================================================================

static constexpr uint8_t BIT_BOOT       = 7;
static constexpr uint8_t MASK_BOOT      = 0x80;
static constexpr uint8_t BIT_BDU        = 6;
static constexpr uint8_t MASK_BDU       = 0x40;
static constexpr uint8_t BIT_H_LACTIVE  = 5;
static constexpr uint8_t MASK_H_LACTIVE = 0x20;
static constexpr uint8_t BIT_PP_OD      = 4;
static constexpr uint8_t MASK_PP_OD     = 0x10;
static constexpr uint8_t BIT_SIM        = 3;
static constexpr uint8_t MASK_SIM       = 0x08;
static constexpr uint8_t BIT_IF_INC     = 2;
static constexpr uint8_t MASK_IF_INC    = 0x04;
static constexpr uint8_t BIT_BLE        = 1;
static constexpr uint8_t MASK_BLE       = 0x02;
static constexpr uint8_t BIT_SW_RESET   = 0;
static constexpr uint8_t MASK_SW_RESET  = 0x01;

/// Default value for CTRL3_C (IF_INC = 1)
static constexpr uint8_t CTRL3_C_DEFAULT = 0x04;

// ============================================================================
// CTRL4_C (13h) Bit Definitions
// ============================================================================

static constexpr uint8_t BIT_DEN_XL_EN      = 7;
static constexpr uint8_t BIT_SLEEP_G        = 6;
static constexpr uint8_t MASK_SLEEP_G       = 0x40;
static constexpr uint8_t BIT_INT2_ON_INT1   = 5;
static constexpr uint8_t BIT_DEN_DRDY_INT1  = 4;
static constexpr uint8_t BIT_DRDY_MASK      = 3;
static constexpr uint8_t BIT_I2C_DISABLE    = 2;
static constexpr uint8_t MASK_I2C_DISABLE   = 0x04;
static constexpr uint8_t BIT_LPF1_SEL_G     = 1;

// ============================================================================
// CTRL5_C (14h) Bit Definitions — Self-Test
// ============================================================================

static constexpr uint8_t BIT_ST_XL   = 0;
static constexpr uint8_t MASK_ST_XL  = 0x03;
static constexpr uint8_t BIT_ST_G    = 2;
static constexpr uint8_t MASK_ST_G   = 0x0C;

// ============================================================================
// CTRL6_C (15h) Bit Definitions
// ============================================================================

static constexpr uint8_t BIT_XL_HM_MODE   = 4;
static constexpr uint8_t MASK_XL_HM_MODE  = 0x10;
static constexpr uint8_t BIT_USR_OFF_W    = 3;
static constexpr uint8_t MASK_USR_OFF_W   = 0x08;
static constexpr uint8_t BIT_FTYPE        = 0;
static constexpr uint8_t MASK_FTYPE       = 0x03;

// ============================================================================
// CTRL7_G (16h) Bit Definitions
// ============================================================================

static constexpr uint8_t BIT_G_HM_MODE  = 7;
static constexpr uint8_t MASK_G_HM_MODE = 0x80;
static constexpr uint8_t BIT_HP_EN_G    = 6;
static constexpr uint8_t MASK_HP_EN_G   = 0x40;
static constexpr uint8_t BIT_HPM_G      = 4;
static constexpr uint8_t MASK_HPM_G     = 0x30;
static constexpr uint8_t BIT_HP_G_RST   = 3;
static constexpr uint8_t MASK_HP_G_RST  = 0x08;

// ============================================================================
// CTRL8_XL (17h) Bit Definitions
// ============================================================================

static constexpr uint8_t BIT_LPF2_XL_EN       = 7;
static constexpr uint8_t MASK_LPF2_XL_EN      = 0x80;
static constexpr uint8_t BIT_HPCF_XL          = 5;
static constexpr uint8_t MASK_HPCF_XL         = 0x60;
static constexpr uint8_t BIT_HP_SLOPE_XL_EN   = 2;
static constexpr uint8_t MASK_HP_SLOPE_XL_EN  = 0x04;
static constexpr uint8_t BIT_LOW_PASS_ON_6D   = 0;
static constexpr uint8_t MASK_LOW_PASS_ON_6D  = 0x01;

// ============================================================================
// CTRL10_C (19h) Bit Definitions — Embedded Functions
// ============================================================================

static constexpr uint8_t BIT_WRIST_TILT_EN  = 7;
static constexpr uint8_t BIT_TIMER_EN       = 6;
static constexpr uint8_t BIT_PEDO_EN        = 5;
static constexpr uint8_t BIT_TILT_EN        = 4;
static constexpr uint8_t BIT_FUNC_EN        = 3;
static constexpr uint8_t BIT_PEDO_RST_STEP  = 2;
static constexpr uint8_t BIT_SIGN_MOTION_EN = 1;
static constexpr uint8_t MASK_FUNC_EN       = 0x08;

// ============================================================================
// STATUS_REG (1Eh) Bit Definitions
// ============================================================================

static constexpr uint8_t BIT_TDA           = 2;
static constexpr uint8_t MASK_TDA          = 0x04;
static constexpr uint8_t BIT_GDA           = 1;
static constexpr uint8_t MASK_GDA          = 0x02;
static constexpr uint8_t BIT_XLDA          = 0;
static constexpr uint8_t MASK_XLDA         = 0x01;

// ============================================================================
// FUNC_CFG_ACCESS (01h) Bit Definitions
// ============================================================================

static constexpr uint8_t BIT_FUNC_CFG_EN    = 7;
static constexpr uint8_t MASK_FUNC_CFG_EN   = 0x80;
static constexpr uint8_t BIT_FUNC_CFG_EN_B  = 5;
static constexpr uint8_t MASK_FUNC_CFG_EN_B = 0x20;

// ============================================================================
// FIFO_CTRL2 (07h) Bit Definitions
// ============================================================================

static constexpr uint8_t BIT_TIMER_PEDO_FIFO_EN   = 7;
static constexpr uint8_t BIT_TIMER_PEDO_FIFO_DRDY = 6;
static constexpr uint8_t BIT_FIFO_TEMP_EN          = 3;

// ============================================================================
// FIFO_CTRL3 (08h) Bit Definitions
// ============================================================================

static constexpr uint8_t BIT_DEC_FIFO_GYRO = 3;
static constexpr uint8_t MASK_DEC_FIFO_GYRO = 0x38;
static constexpr uint8_t BIT_DEC_FIFO_XL   = 0;
static constexpr uint8_t MASK_DEC_FIFO_XL  = 0x07;

// ============================================================================
// FIFO_CTRL5 (0Ah) Bit Definitions
// ============================================================================

static constexpr uint8_t BIT_ODR_FIFO    = 3;
static constexpr uint8_t MASK_ODR_FIFO   = 0x78;
static constexpr uint8_t BIT_FIFO_MODE   = 0;
static constexpr uint8_t MASK_FIFO_MODE  = 0x07;

// ============================================================================
// FIFO_STATUS2 (3Bh) Bit Definitions
// ============================================================================

static constexpr uint8_t BIT_FIFO_WATERM      = 7;
static constexpr uint8_t MASK_FIFO_WATERM     = 0x80;
static constexpr uint8_t BIT_FIFO_OVER_RUN    = 6;
static constexpr uint8_t MASK_FIFO_OVER_RUN   = 0x40;
static constexpr uint8_t BIT_FIFO_FULL_SMART  = 5;
static constexpr uint8_t MASK_FIFO_FULL_SMART = 0x20;
static constexpr uint8_t BIT_FIFO_EMPTY       = 4;
static constexpr uint8_t MASK_FIFO_EMPTY      = 0x10;
static constexpr uint8_t MASK_DIFF_FIFO_HI    = 0x07;

// ============================================================================
// TAP_CFG (58h) Bit Definitions
// ============================================================================

static constexpr uint8_t BIT_INTERRUPTS_ENABLE = 7;
static constexpr uint8_t MASK_INTERRUPTS_ENABLE = 0x80;
static constexpr uint8_t BIT_INACT_EN          = 5;
static constexpr uint8_t MASK_INACT_EN         = 0x60;
static constexpr uint8_t BIT_SLOPE_FDS         = 4;
static constexpr uint8_t MASK_SLOPE_FDS        = 0x10;
static constexpr uint8_t BIT_TAP_X_EN          = 3;
static constexpr uint8_t BIT_TAP_Y_EN          = 2;
static constexpr uint8_t BIT_TAP_Z_EN          = 1;
static constexpr uint8_t BIT_LIR               = 0;
static constexpr uint8_t MASK_LIR              = 0x01;

// ============================================================================
// INT1_CTRL (0Dh) Bit Definitions
// ============================================================================

static constexpr uint8_t BIT_INT1_STEP_DETECTOR = 7;
static constexpr uint8_t BIT_INT1_SIGN_MOT      = 6;
static constexpr uint8_t BIT_INT1_FULL_FLAG      = 5;
static constexpr uint8_t BIT_INT1_FIFO_OVR       = 4;
static constexpr uint8_t BIT_INT1_FTH            = 3;
static constexpr uint8_t BIT_INT1_BOOT           = 2;
static constexpr uint8_t BIT_INT1_DRDY_G         = 1;
static constexpr uint8_t BIT_INT1_DRDY_XL        = 0;

// ============================================================================
// INT2_CTRL (0Eh) Bit Definitions
// ============================================================================

static constexpr uint8_t BIT_INT2_STEP_DELTA    = 7;
static constexpr uint8_t BIT_INT2_STEP_COUNT_OV = 6;
static constexpr uint8_t BIT_INT2_FULL_FLAG      = 5;
static constexpr uint8_t BIT_INT2_FIFO_OVR       = 4;
static constexpr uint8_t BIT_INT2_FTH            = 3;
static constexpr uint8_t BIT_INT2_DRDY_TEMP      = 2;
static constexpr uint8_t BIT_INT2_DRDY_G         = 1;
static constexpr uint8_t BIT_INT2_DRDY_XL        = 0;

// ============================================================================
// WAKE_UP_SRC (1Bh) Bit Definitions
// ============================================================================

static constexpr uint8_t MASK_FF_IA          = 0x20;
static constexpr uint8_t MASK_SLEEP_STATE_IA = 0x10;
static constexpr uint8_t MASK_WU_IA          = 0x08;
static constexpr uint8_t MASK_X_WU           = 0x04;
static constexpr uint8_t MASK_Y_WU           = 0x02;
static constexpr uint8_t MASK_Z_WU           = 0x01;

// ============================================================================
// Timestamp Reset Value
// ============================================================================

static constexpr uint8_t TIMESTAMP_RESET_VALUE = 0xAA;

// ============================================================================
// SW Reset / Boot Timing
// ============================================================================

static constexpr uint32_t BOOT_TIME_MS      = 15;
static constexpr uint32_t SW_RESET_TIMEOUT_MS = 10;
static constexpr uint16_t SW_RESET_MAX_POLLS  = 255;

} // namespace cmd
} // namespace LSM6DS3TR
