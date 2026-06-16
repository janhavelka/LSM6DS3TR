idf_component_register(
  SRCS "src/LSM6DS3TR.cpp"
  INCLUDE_DIRS "include"
  PRIV_REQUIRES esp_timer
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
