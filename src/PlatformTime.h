/// @file PlatformTime.h
/// @brief Private framework timing shim for the LSM6DS3TR core.
#pragma once

#include <cstdint>

#if defined(ARDUINO) || (!defined(ESP_PLATFORM) && __has_include(<Arduino.h>))
#include <Arduino.h>
#elif defined(ESP_PLATFORM)
#include <esp_timer.h>
#endif

namespace LSM6DS3TR {
namespace platform {

inline uint32_t nowMs() {
#if defined(ARDUINO) || (!defined(ESP_PLATFORM) && __has_include(<Arduino.h>))
  return millis();
#elif defined(ESP_PLATFORM)
  return static_cast<uint32_t>(esp_timer_get_time() / 1000LL);
#else
  return 0;
#endif
}

}  // namespace platform
}  // namespace LSM6DS3TR
