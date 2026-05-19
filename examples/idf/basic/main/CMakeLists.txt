idf_component_register(
  SRCS
    "main.cpp"
    "ArduinoCompat.cpp"
    "../../../01_basic_bringup_cli/main.cpp"
  INCLUDE_DIRS "." "../../../.."
  REQUIRES LSM6DS3TR esp_driver_i2c esp_driver_gpio esp_timer freertos
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
target_compile_definitions(${COMPONENT_LIB} PRIVATE ARDUINO_ARCH_ESP32=1)
