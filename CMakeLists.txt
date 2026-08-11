cmake_minimum_required(VERSION 3.16.0)

set(ENV{IDF_TARGET} "esp32p4")
set(IDF_TARGET "esp32p4" CACHE STRING "ESP-IDF target" FORCE)
set(PARTITION_TABLE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/partitions.csv")

add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fexceptions>)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(triathlon_timing)
