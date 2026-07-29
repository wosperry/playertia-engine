cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../app-shell"
    "${CMAKE_CURRENT_LIST_DIR}/../clock-core"
    "${CMAKE_CURRENT_LIST_DIR}/../../../engine/physics")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(playertia_esp32_s3_clock)
