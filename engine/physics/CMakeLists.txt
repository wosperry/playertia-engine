cmake_minimum_required(VERSION 3.24)

if(COMMAND idf_component_register)
  idf_component_register(
    SRCS "src/physics.cpp"
    INCLUDE_DIRS "include")
  target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_20)
  target_compile_definitions(${COMPONENT_LIB} PRIVATE DIAL_MUTATION_ID=0)
  return()
endif()

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(playertia_dial_physics_engine VERSION 1.0.0 LANGUAGES CXX)
endif()

set(DIAL_MUTATION_ID "0" CACHE STRING
    "Compile one deliberate validation mutation; production value is 0")

add_library(playertia_dial_physics STATIC
  src/physics.cpp)
add_library(playertia::dial_physics ALIAS playertia_dial_physics)
set_target_properties(playertia_dial_physics PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES)

target_include_directories(playertia_dial_physics
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(playertia_dial_physics PUBLIC cxx_std_20)
target_compile_definitions(playertia_dial_physics
  PRIVATE DIAL_MUTATION_ID=${DIAL_MUTATION_ID})
target_compile_options(playertia_dial_physics
  PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:
      -Wall
      -Wextra
      -Wpedantic
      -Wconversion
      -Wshadow>)
