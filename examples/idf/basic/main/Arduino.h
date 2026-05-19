/// @file Arduino.h
/// @brief Small ESP-IDF compatibility facade for the shared Arduino example CLI.
///
/// This header is example-local. It is not part of the library API and exists
/// only so the ESP-IDF example can compile the same bringup CLI source as the
/// Arduino example.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>

class String {
public:
  String() = default;
  String(const char* value) : _value(value ? value : "") {}
  String(const std::string& value) : _value(value) {}

  String& operator=(const char* value) {
    _value = value ? value : "";
    return *this;
  }

  size_t length() const {
    return _value.length();
  }

  bool reserve(size_t size) {
    _value.reserve(size);
    return true;
  }

  void trim();
  void remove(size_t index);
  int indexOf(char ch, int fromIndex = 0) const;
  String substring(int start) const;
  String substring(int start, int end) const;
  bool startsWith(const char* prefix) const;
  void toLowerCase();

  const char* c_str() const {
    return _value.c_str();
  }

  char operator[](size_t index) const {
    return _value[index];
  }

  String& operator+=(char ch) {
    _value.push_back(ch);
    return *this;
  }

  friend bool operator==(const String& lhs, const char* rhs) {
    return lhs._value == (rhs ? rhs : "");
  }

  friend bool operator!=(const String& lhs, const char* rhs) {
    return !(lhs == rhs);
  }

private:
  std::string _value;
};

class SerialCompat {
public:
  void begin(unsigned long baud);

  int available();
  int read();

  template <typename... Args>
  int printf(const char* fmt, Args... args) {
    return std::printf(fmt, args...);
  }

  void print(const char* value);
  void print(char value);
  void println();
  void println(const char* value);

private:
  void drainInput();

  static constexpr size_t INPUT_CAPACITY = 256U;
  char _input[INPUT_CAPACITY] = {};
  size_t _head = 0;
  size_t _tail = 0;
};

extern SerialCompat Serial;

uint32_t millis();
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
void yield();

enum {
  LOW = 0,
  HIGH = 1,
  INPUT_PULLUP = 2,
  OUTPUT = 3,
};

void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
int digitalRead(int pin);
