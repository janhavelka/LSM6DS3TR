idf_component_register(
  SRCS
    "main.cpp"
  PRIV_INCLUDE_DIRS
    "."
    "../../../common"
  REQUIRES LSM6DS3TR esp_driver_i2c esp_timer freertos heap spi_flash
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
