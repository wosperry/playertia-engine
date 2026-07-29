cmake_minimum_required(VERSION 3.24)
project(playertia_engine VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)

option(PLAYERTIA_BUILD_C_SDK "Build the versioned C SDK boundary" ON)
option(PLAYERTIA_BUILD_VALIDATION "Build the x86 validation project" OFF)
option(PLAYERTIA_BUILD_ESP32_APP_SHELL
       "Build the portable ESP32-S3 application shell" OFF)

add_subdirectory(engine/physics)
add_subdirectory(platforms/esp32-s3/clock-core)

if(PLAYERTIA_BUILD_C_SDK)
  set(PLAYERTIA_PHYSICS_ENGINE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/engine/physics")
  add_subdirectory(sdk/c)
endif()

if(PLAYERTIA_BUILD_VALIDATION)
  set(DIAL_PHYSICS_ENGINE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/engine/physics")
  add_subdirectory(validation/x86-physics-engine)
endif()

if(PLAYERTIA_BUILD_ESP32_APP_SHELL)
  add_subdirectory(platforms/esp32-s3/app-shell)
endif()
