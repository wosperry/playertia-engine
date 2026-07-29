cmake_minimum_required(VERSION 3.16)

set(PLAYERTIA_ESP32_APP_SHELL_SOURCES
  src/app_manager.cpp
  src/demo_apps.cpp)

if(COMMAND idf_component_register)
  idf_component_register(
    SRCS ${PLAYERTIA_ESP32_APP_SHELL_SOURCES}
    INCLUDE_DIRS include)
  target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
  return()
endif()

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(playertia_esp32_app_shell VERSION 0.1.0 LANGUAGES CXX)
  include(CTest)
endif()

add_library(playertia_esp32_app_shell STATIC
  ${PLAYERTIA_ESP32_APP_SHELL_SOURCES})
add_library(playertia::esp32_app_shell ALIAS playertia_esp32_app_shell)

target_include_directories(playertia_esp32_app_shell
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(playertia_esp32_app_shell PUBLIC cxx_std_17)
target_compile_options(playertia_esp32_app_shell
  PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:
      -Wall
      -Wextra
      -Wpedantic
      -Wconversion
      -Wshadow>)

if(BUILD_TESTING AND NOT ESP_PLATFORM)
  add_executable(playertia_esp32_app_shell_test
    tests/app_shell_test.cpp)
  target_link_libraries(playertia_esp32_app_shell_test
    PRIVATE playertia::esp32_app_shell)
  add_test(
    NAME playertia_esp32_app_shell_test
    COMMAND playertia_esp32_app_shell_test)
endif()
