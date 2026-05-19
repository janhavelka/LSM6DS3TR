/// @file main.cpp
/// @brief ESP-IDF entrypoint for the shared LSM6DS3TR-C bringup CLI.

#include "Arduino.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern void setup();
extern void loop();

extern "C" void app_main(void) {
  setup();

  const TickType_t loopDelay = pdMS_TO_TICKS(5) == 0 ? 1 : pdMS_TO_TICKS(5);
  while (true) {
    loop();
    vTaskDelay(loopDelay);
  }
}
