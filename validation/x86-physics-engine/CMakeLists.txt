cmake_minimum_required(VERSION 3.24)
project(x86_physics_engine_validation VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(DIAL_BUILD_TESTS "Build validation tests" ON)
option(DIAL_BUILD_BENCHMARKS "Build performance benchmarks" ON)
option(DIAL_BUILD_FUZZERS "Build libFuzzer targets (Clang only)" OFF)
option(DIAL_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(DIAL_ENABLE_COVERAGE "Enable GCC/Clang coverage instrumentation" OFF)
set(DIAL_MUTATION_ID "0" CACHE STRING "Compile one deliberate physics mutation")
set(DIAL_PHYSICS_ENGINE_DIR
    "${CMAKE_CURRENT_LIST_DIR}/../../engine/physics"
    CACHE PATH "Path to the standalone production physics engine")

include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
  URL_HASH SHA256=42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)

if(DIAL_BUILD_BENCHMARKS)
  FetchContent_Declare(
    googlebenchmark
    URL https://github.com/google/benchmark/archive/refs/tags/v1.9.5.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
endif()

FetchContent_MakeAvailable(googletest nlohmann_json)
if(DIAL_BUILD_BENCHMARKS)
  FetchContent_MakeAvailable(googlebenchmark)
endif()

if(NOT EXISTS "${DIAL_PHYSICS_ENGINE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "Standalone physics engine not found: ${DIAL_PHYSICS_ENGINE_DIR}")
endif()
add_subdirectory(
  "${DIAL_PHYSICS_ENGINE_DIR}"
  "${CMAKE_BINARY_DIR}/dial-physics-core")

add_library(dial_reference STATIC
  src/reference.cpp)
target_include_directories(dial_reference PUBLIC include)
target_link_libraries(dial_reference PUBLIC playertia::dial_physics)
target_compile_options(dial_reference PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wshadow>)

add_library(dial_test_support INTERFACE)
target_include_directories(dial_test_support INTERFACE include)
target_link_libraries(dial_test_support INTERFACE playertia::dial_physics)

if(DIAL_ENABLE_SANITIZERS)
  foreach(target playertia_dial_physics dial_reference)
    target_compile_options(${target} PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(${target} PUBLIC -fsanitize=address,undefined)
  endforeach()
endif()

if(DIAL_ENABLE_COVERAGE)
  foreach(target playertia_dial_physics dial_reference)
    target_compile_options(${target} PUBLIC --coverage -O0 -g)
    target_link_options(${target} PUBLIC --coverage)
  endforeach()
endif()

add_executable(dial_validate tools/validation_runner.cpp)
target_link_libraries(dial_validate PRIVATE
  playertia::dial_physics
  dial_reference
  dial_test_support
  nlohmann_json::nlohmann_json)

if(DIAL_BUILD_TESTS)
  enable_testing()
  include(GoogleTest)
  add_executable(dial_tests
    tests/unit/math_tests.cpp
    tests/unit/floating_mode_tests.cpp
    tests/model/dynamics_tests.cpp
    tests/scenarios/scenario_tests.cpp
    tests/property/property_tests.cpp
    tests/fuzz/fault_tests.cpp)
  target_link_libraries(dial_tests PRIVATE
    playertia::dial_physics
    dial_reference
    dial_test_support
    GTest::gtest_main)
  # PRE_TEST avoids executing an ASan binary during the build-time discovery
  # phase. Some managed/ptraced CI environments cannot initialize LSan then.
  gtest_discover_tests(dial_tests DISCOVERY_TIMEOUT 30 DISCOVERY_MODE PRE_TEST)
endif()

if(DIAL_BUILD_BENCHMARKS)
  add_executable(dial_bench tests/performance/physics_benchmark.cpp)
  target_link_libraries(dial_bench PRIVATE
    playertia::dial_physics
    dial_test_support
    benchmark::benchmark_main)
endif()

if(DIAL_BUILD_FUZZERS AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_executable(dial_input_fuzzer tests/fuzz/input_fuzzer.cpp)
  target_link_libraries(dial_input_fuzzer PRIVATE
    playertia::dial_physics
    dial_test_support)
  target_compile_options(dial_input_fuzzer PRIVATE -fsanitize=fuzzer,address,undefined)
  target_link_options(dial_input_fuzzer PRIVATE -fsanitize=fuzzer,address,undefined)
endif()
