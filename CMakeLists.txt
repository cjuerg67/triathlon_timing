cmake_minimum_required(VERSION 3.16.0)

set(ENV{IDF_TARGET} "esp32p4")
set(IDF_TARGET "esp32p4" CACHE STRING "ESP-IDF target" FORCE)

add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fexceptions>)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(triathlon_timing)
