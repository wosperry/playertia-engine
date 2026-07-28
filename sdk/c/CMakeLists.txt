cmake_minimum_required(VERSION 3.24)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(playertia_engine_sdk VERSION 0.1.0 LANGUAGES C CXX)
endif()

option(PLAYERTIA_SDK_BUILD_SHARED "Build the C SDK as a self-contained shared library" ON)
option(PLAYERTIA_SDK_BUILD_TESTS "Build the C SDK smoke test" ON)
set(PLAYERTIA_PHYSICS_ENGINE_DIR
    "${CMAKE_CURRENT_LIST_DIR}/../../engine/physics"
    CACHE PATH "Path to the production physics engine")

if(NOT TARGET playertia::dial_physics)
  add_subdirectory(
    "${PLAYERTIA_PHYSICS_ENGINE_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/dial-physics-core")
endif()

set(PLAYERTIA_SDK_LIBRARY_TYPE STATIC)
if(PLAYERTIA_SDK_BUILD_SHARED)
  set(PLAYERTIA_SDK_LIBRARY_TYPE SHARED)
endif()

add_library(playertia_engine ${PLAYERTIA_SDK_LIBRARY_TYPE}
  src/playertia_engine.cpp)
add_library(playertia::engine ALIAS playertia_engine)

target_include_directories(playertia_engine
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(playertia_engine PRIVATE cxx_std_20)
target_link_libraries(playertia_engine PRIVATE playertia::dial_physics)
target_compile_definitions(playertia_engine
  PRIVATE PLAYERTIA_SDK_BUILDING_LIBRARY=1)
set_target_properties(playertia_engine PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
  POSITION_INDEPENDENT_CODE ON
  VERSION ${PROJECT_VERSION}
  SOVERSION 0)
target_compile_options(playertia_engine PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wshadow>)

if(PLAYERTIA_SDK_BUILD_TESTS)
  enable_testing()
  add_executable(playertia_engine_smoke tests/smoke.c)
  target_link_libraries(playertia_engine_smoke PRIVATE playertia::engine)
  add_test(NAME playertia_engine_smoke COMMAND playertia_engine_smoke)
endif()

include(GNUInstallDirs)
install(TARGETS playertia_engine
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
