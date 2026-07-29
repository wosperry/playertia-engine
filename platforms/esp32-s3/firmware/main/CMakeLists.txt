idf_component_register(
  SRCS
    "main.cpp"
    "board_services.cpp"
    "clock_app.cpp"
  INCLUDE_DIRS "."
  REQUIRES
    app-shell
    clock-core
    physics
    esp32_s3_touch_amoled_1_8
    esp_driver_gpio
    esp_timer
    pcf85063a
    qmi8658)

string(TIMESTAMP PLAYERTIA_BUILD_EPOCH "%s" UTC)
target_compile_definitions(${COMPONENT_LIB}
  PRIVATE PLAYERTIA_BUILD_EPOCH=${PLAYERTIA_BUILD_EPOCH})

idf_component_get_property(LVGL_LIB lvgl__lvgl COMPONENT_LIB)
if(TARGET ${LVGL_LIB})
  target_compile_options(${LVGL_LIB} PRIVATE -Wno-error=attributes)
endif()
