cmake_minimum_required(VERSION 3.16)

set(PLAYERTIA_CLOCK_CORE_SOURCES
  src/clock_model.cpp)

if(COMMAND idf_component_register)
  idf_component_register(
    SRCS ${PLAYERTIA_CLOCK_CORE_SOURCES}
    INCLUDE_DIRS include)
  target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
  return()
endif()

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(playertia_clock_core VERSION 0.1.0 LANGUAGES CXX)
  include(CTest)
endif()

add_library(playertia_clock_core STATIC
  ${PLAYERTIA_CLOCK_CORE_SOURCES})
add_library(playertia::clock_core ALIAS playertia_clock_core)

target_include_directories(playertia_clock_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(playertia_clock_core PUBLIC cxx_std_17)
target_compile_options(playertia_clock_core
  PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:
      -Wall
      -Wextra
      -Wpedantic
      -Wconversion
      -Wshadow>)

if(BUILD_TESTING AND NOT ESP_PLATFORM)
  add_executable(playertia_clock_core_test
    tests/clock_model_test.cpp)
  target_link_libraries(playertia_clock_core_test
    PRIVATE playertia::clock_core)
  add_test(
    NAME playertia_clock_core_test
    COMMAND playertia_clock_core_test)
endif()
