/// @file ArduinoCompat.cpp
/// @brief ESP-IDF backing for the example-local Arduino/Wire compatibility facade.

#include "Arduino.h"
#include "Wire.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <climits>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

SerialCompat Serial;
TwoWire Wire;

void String::trim() {
  size_t first = 0;
  while (first < _value.size() &&
         std::isspace(static_cast<unsigned char>(_value[first])) != 0) {
    ++first;
  }

  size_t last = _value.size();
  while (last > first &&
         std::isspace(static_cast<unsigned char>(_value[last - 1U])) != 0) {
    --last;
  }

  _value = _value.substr(first, last - first);
}

void String::remove(size_t index) {
  if (index < _value.size()) {
    _value.erase(index);
  }
}

int String::indexOf(char ch, int fromIndex) const {
  const size_t start = (fromIndex < 0) ? 0U : static_cast<size_t>(fromIndex);
  const size_t pos = _value.find(ch, start);
  if (pos == std::string::npos || pos > static_cast<size_t>(INT_MAX)) {
    return -1;
  }
  return static_cast<int>(pos);
}

String String::substring(int start) const {
  return substring(start, static_cast<int>(_value.size()));
}

String String::substring(int start, int end) const {
  const size_t len = _value.size();
  size_t first = (start < 0) ? 0U : static_cast<size_t>(start);
  size_t last = (end < 0) ? 0U : static_cast<size_t>(end);
  first = std::min(first, len);
  last = std::min(last, len);
  if (last < first) {
    last = first;
  }
  return String(_value.substr(first, last - first));
}

bool String::startsWith(const char* prefix) const {
  if (prefix == nullptr) {
    return false;
  }
  const size_t prefixLen = std::strlen(prefix);
  return _value.size() >= prefixLen &&
         _value.compare(0, prefixLen, prefix) == 0;
}

void String::toLowerCase() {
  for (char& ch : _value) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
}

void SerialCompat::begin(unsigned long baud) {
  (void)baud;
  setvbuf(stdout, nullptr, _IONBF, 0);

  const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  if (flags >= 0) {
    (void)fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
  }
}

int SerialCompat::available() {
  drainInput();
  return (_tail >= _head)
             ? static_cast<int>(_tail - _head)
             : static_cast<int>(INPUT_CAPACITY - _head + _tail);
}

int SerialCompat::read() {
  if (available() <= 0) {
    return -1;
  }
  const char ch = _input[_head];
  _head = (_head + 1U) % INPUT_CAPACITY;
  return static_cast<unsigned char>(ch);
}

void SerialCompat::print(const char* value) {
  std::fputs(value ? value : "", stdout);
}

void SerialCompat::print(char value) {
  std::fputc(value, stdout);
}

void SerialCompat::println() {
  std::fputc('\n', stdout);
}

void SerialCompat::println(const char* value) {
  print(value);
  println();
}

void SerialCompat::drainInput() {
  while (true) {
    char ch = 0;
    const ssize_t got = ::read(STDIN_FILENO, &ch, 1);
    if (got == 1) {
      const size_t nextTail = (_tail + 1U) % INPUT_CAPACITY;
      if (nextTail != _head) {
        _input[_tail] = ch;
        _tail = nextTail;
      }
      continue;
    }
    if (got < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      break;
    }
    break;
  }
}

uint32_t millis() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000LL);
}

void delay(uint32_t ms) {
  if (ms == 0U) {
    taskYIELD();
    return;
  }
  const TickType_t ticks = pdMS_TO_TICKS(ms);
  vTaskDelay(ticks == 0 ? 1 : ticks);
}

void delayMicroseconds(uint32_t us) {
  esp_rom_delay_us(us);
}

void yield() {
  taskYIELD();
}

void pinMode(int pin, int mode) {
  if (pin < 0) {
    return;
  }

  gpio_config_t cfg = {};
  cfg.pin_bit_mask = 1ULL << static_cast<uint32_t>(pin);
  cfg.intr_type = GPIO_INTR_DISABLE;

  if (mode == OUTPUT) {
    cfg.mode = GPIO_MODE_OUTPUT_OD;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  } else {
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = (mode == INPUT_PULLUP) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  }

  (void)gpio_config(&cfg);
}

void digitalWrite(int pin, int value) {
  if (pin >= 0) {
    (void)gpio_set_level(static_cast<gpio_num_t>(pin), value == LOW ? 0 : 1);
  }
}

int digitalRead(int pin) {
  if (pin < 0) {
    return LOW;
  }
  return gpio_get_level(static_cast<gpio_num_t>(pin)) == 0 ? LOW : HIGH;
}

bool TwoWire::begin(int sda, int scl) {
  end();

  _sda = sda;
  _scl = scl;

  i2c_master_bus_config_t busCfg = {};
  busCfg.clk_source = I2C_CLK_SRC_DEFAULT;
  busCfg.i2c_port = I2C_NUM_0;
  busCfg.scl_io_num = static_cast<gpio_num_t>(scl);
  busCfg.sda_io_num = static_cast<gpio_num_t>(sda);
  busCfg.glitch_ignore_cnt = 7;
  busCfg.flags.enable_internal_pullup = true;

  return i2c_new_master_bus(&busCfg, &_bus) == ESP_OK;
}

void TwoWire::end() {
  for (i2c_master_dev_handle_t& device : _devices) {
    if (device != nullptr) {
      (void)i2c_master_bus_rm_device(device);
      device = nullptr;
    }
  }

  if (_bus != nullptr) {
    (void)i2c_del_master_bus(_bus);
    _bus = nullptr;
  }

  _txLen = 0;
  _rxLen = 0;
  _rxIndex = 0;
  _hasPendingRepeatedStart = false;
}

void TwoWire::setClock(uint32_t freqHz) {
  if (freqHz > 0U) {
    _freqHz = freqHz;
  }
}

void TwoWire::setTimeOut(uint16_t timeoutMs) {
  if (timeoutMs > 0U) {
    _timeoutMs = timeoutMs;
  }
}

void TwoWire::beginTransmission(uint8_t address) {
  _txAddress = address;
  _txLen = 0;
  _hasPendingRepeatedStart = false;
}

size_t TwoWire::write(const uint8_t* data, size_t len) {
  if (data == nullptr || len == 0U || _txLen >= BUFFER_CAPACITY) {
    return 0U;
  }

  const size_t writable = std::min(len, BUFFER_CAPACITY - _txLen);
  std::memcpy(&_txBuffer[_txLen], data, writable);
  _txLen += writable;
  return writable;
}

size_t TwoWire::write(uint8_t value) {
  return write(&value, 1U);
}

uint8_t TwoWire::endTransmission(bool stop) {
  if (_bus == nullptr) {
    return 4;
  }

  if (!stop) {
    _hasPendingRepeatedStart = true;
    return 0;
  }

  if (_txLen == 0U) {
    const esp_err_t err = i2c_master_probe(_bus, _txAddress, timeoutMs());
    return mapError(err);
  }

  i2c_master_dev_handle_t device = deviceFor(_txAddress);
  if (device == nullptr) {
    return 4;
  }

  const esp_err_t err = i2c_master_transmit(device, _txBuffer, _txLen, timeoutMs());
  _txLen = 0;
  _hasPendingRepeatedStart = false;
  return mapError(err);
}

size_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
  _rxLen = 0;
  _rxIndex = 0;

  if (_bus == nullptr || quantity == 0U) {
    return 0U;
  }

  const size_t requested = std::min(static_cast<size_t>(quantity), BUFFER_CAPACITY);
  i2c_master_dev_handle_t device = deviceFor(address);
  if (device == nullptr) {
    _txLen = 0;
    _hasPendingRepeatedStart = false;
    return 0U;
  }

  esp_err_t err = ESP_OK;
  if (_hasPendingRepeatedStart && address == _txAddress && _txLen > 0U) {
    err = i2c_master_transmit_receive(device, _txBuffer, _txLen, _rxBuffer,
                                      requested, timeoutMs());
  } else {
    err = i2c_master_receive(device, _rxBuffer, requested, timeoutMs());
  }

  _txLen = 0;
  _hasPendingRepeatedStart = false;
  if (err != ESP_OK) {
    return 0U;
  }

  _rxLen = requested;
  return _rxLen;
}

int TwoWire::available() const {
  return (_rxLen > _rxIndex) ? static_cast<int>(_rxLen - _rxIndex) : 0;
}

int TwoWire::read() {
  if (_rxIndex >= _rxLen) {
    return -1;
  }
  return _rxBuffer[_rxIndex++];
}

i2c_master_dev_handle_t TwoWire::deviceFor(uint8_t address) {
  if (_bus == nullptr || address >= MAX_ADDRESS) {
    return nullptr;
  }

  if (_devices[address] != nullptr) {
    return _devices[address];
  }

  i2c_device_config_t devCfg = {};
  devCfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  devCfg.device_address = address;
  devCfg.scl_speed_hz = _freqHz;

  if (i2c_master_bus_add_device(_bus, &devCfg, &_devices[address]) != ESP_OK) {
    _devices[address] = nullptr;
  }
  return _devices[address];
}

uint8_t TwoWire::mapError(esp_err_t err) const {
  switch (err) {
    case ESP_OK:
      return 0;
    case ESP_ERR_INVALID_ARG:
      return 1;
    case ESP_ERR_NOT_FOUND:
    case ESP_ERR_INVALID_RESPONSE:
      return 2;
    case ESP_ERR_TIMEOUT:
      return 5;
    default:
      return 4;
  }
}

int TwoWire::timeoutMs() const {
  return static_cast<int>(_timeoutMs);
}
