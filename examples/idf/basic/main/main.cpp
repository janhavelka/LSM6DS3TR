#include <cstddef>
#include <cstdint>
#include <limits>

#include "LSM6DS3TR/LSM6DS3TR.h"

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

static constexpr char TAG[] = "lsm6ds3tr_basic";
static constexpr i2c_port_num_t I2C_PORT = I2C_NUM_0;
static constexpr gpio_num_t I2C_SDA = GPIO_NUM_8;
static constexpr gpio_num_t I2C_SCL = GPIO_NUM_9;
static constexpr uint32_t I2C_FREQ_HZ = 400000;
static constexpr uint8_t IMU_ADDR = 0x6A;
static constexpr uint32_t I2C_TIMEOUT_MS = 50;
static constexpr TickType_t SAMPLE_PERIOD_TICKS = pdMS_TO_TICKS(500);

struct IdfI2cDevice {
  i2c_master_dev_handle_t handle = nullptr;
  uint8_t address = IMU_ADDR;
};

int timeoutToInt(uint32_t timeoutMs) {
  const uint32_t maxTimeout = static_cast<uint32_t>(std::numeric_limits<int>::max());
  return static_cast<int>((timeoutMs > maxTimeout) ? maxTimeout : timeoutMs);
}

LSM6DS3TR::Status mapEspError(esp_err_t err, const char* context) {
  if (err == ESP_OK) {
    return LSM6DS3TR::Status::Ok();
  }
  if (err == ESP_ERR_TIMEOUT) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_TIMEOUT, context,
                                    static_cast<int32_t>(err));
  }
  if (err == ESP_ERR_INVALID_ARG) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::INVALID_PARAM, context,
                                    static_cast<int32_t>(err));
  }
  if (err == ESP_ERR_INVALID_RESPONSE) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_ERROR, context,
                                    static_cast<int32_t>(err));
  }
  return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_ERROR, context,
                                  static_cast<int32_t>(err));
}

LSM6DS3TR::Status idfWrite(uint8_t addr, const uint8_t* data, size_t len,
                           uint32_t timeoutMs, void* user) {
  auto* dev = static_cast<IdfI2cDevice*>(user);
  if (dev == nullptr || dev->handle == nullptr || data == nullptr || len == 0 ||
      addr != dev->address) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::INVALID_PARAM,
                                    "Invalid IDF I2C write argument");
  }

  const esp_err_t err =
      i2c_master_transmit(dev->handle, data, len, timeoutToInt(timeoutMs));
  return mapEspError(err, "IDF I2C write failed");
}

LSM6DS3TR::Status idfWriteRead(uint8_t addr, const uint8_t* txData, size_t txLen,
                               uint8_t* rxData, size_t rxLen, uint32_t timeoutMs,
                               void* user) {
  auto* dev = static_cast<IdfI2cDevice*>(user);
  if (dev == nullptr || dev->handle == nullptr || addr != dev->address) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::INVALID_PARAM,
                                    "Invalid IDF I2C device");
  }
  if ((txLen > 0 && txData == nullptr) || (rxLen > 0 && rxData == nullptr)) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::INVALID_PARAM,
                                    "Invalid IDF I2C buffer");
  }

  const int timeout = timeoutToInt(timeoutMs);
  esp_err_t err = ESP_OK;
  if (rxLen == 0) {
    err = i2c_master_transmit(dev->handle, txData, txLen, timeout);
  } else if (txLen == 0) {
    err = i2c_master_receive(dev->handle, rxData, rxLen, timeout);
  } else {
    err = i2c_master_transmit_receive(dev->handle, txData, txLen, rxData, rxLen,
                                      timeout);
  }
  return mapEspError(err, "IDF I2C transfer failed");
}

uint32_t idfNowMs(void*) {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000LL);
}

LSM6DS3TR::Config makeConfig(IdfI2cDevice& i2c) {
  LSM6DS3TR::Config cfg;
  cfg.i2cWrite = idfWrite;
  cfg.i2cWriteRead = idfWriteRead;
  cfg.i2cUser = &i2c;
  cfg.nowMs = idfNowMs;
  cfg.i2cAddress = IMU_ADDR;
  cfg.i2cTimeoutMs = I2C_TIMEOUT_MS;
  cfg.odrXl = LSM6DS3TR::Odr::HZ_104;
  cfg.odrG = LSM6DS3TR::Odr::HZ_104;
  cfg.fsXl = LSM6DS3TR::AccelFs::G_2;
  cfg.fsG = LSM6DS3TR::GyroFs::DPS_250;
  return cfg;
}

}  // namespace

extern "C" void app_main(void) {
  i2c_master_bus_config_t busCfg = {};
  busCfg.clk_source = I2C_CLK_SRC_DEFAULT;
  busCfg.i2c_port = I2C_PORT;
  busCfg.scl_io_num = I2C_SCL;
  busCfg.sda_io_num = I2C_SDA;
  busCfg.glitch_ignore_cnt = 7;
  busCfg.flags.enable_internal_pullup = true;

  i2c_master_bus_handle_t busHandle = nullptr;
  ESP_ERROR_CHECK(i2c_new_master_bus(&busCfg, &busHandle));

  i2c_device_config_t devCfg = {};
  devCfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  devCfg.device_address = IMU_ADDR;
  devCfg.scl_speed_hz = I2C_FREQ_HZ;

  IdfI2cDevice i2c;
  i2c.address = IMU_ADDR;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(busHandle, &devCfg, &i2c.handle));

  LSM6DS3TR::LSM6DS3TR imu;
  LSM6DS3TR::Status st = imu.begin(makeConfig(i2c));
  if (!st.ok()) {
    ESP_LOGE(TAG, "begin failed: code=%u detail=%ld msg=%s",
             static_cast<unsigned>(st.code), static_cast<long>(st.detail), st.msg);
    return;
  }

  uint8_t whoAmI = 0;
  st = imu.readWhoAmI(whoAmI);
  if (st.ok()) {
    ESP_LOGI(TAG, "WHO_AM_I=0x%02X", static_cast<unsigned>(whoAmI));
  }

  while (true) {
    imu.tick(idfNowMs(nullptr));

    LSM6DS3TR::RawMeasurement raw;
    st = imu.readAllRaw(raw);
    if (st.ok()) {
      const LSM6DS3TR::Axes accel = imu.convertAccel(raw.accel);
      const LSM6DS3TR::Axes gyro = imu.convertGyro(raw.gyro);
      ESP_LOGI(TAG,
               "accel[g]=%.3f %.3f %.3f gyro[dps]=%.2f %.2f %.2f temp=%.2fC",
               static_cast<double>(accel.x), static_cast<double>(accel.y),
               static_cast<double>(accel.z), static_cast<double>(gyro.x),
               static_cast<double>(gyro.y), static_cast<double>(gyro.z),
               static_cast<double>(imu.convertTemperature(raw.temperature)));
    } else {
      ESP_LOGW(TAG, "read failed: code=%u detail=%ld msg=%s",
               static_cast<unsigned>(st.code), static_cast<long>(st.detail), st.msg);
    }

    vTaskDelay(SAMPLE_PERIOD_TICKS);
  }
}
