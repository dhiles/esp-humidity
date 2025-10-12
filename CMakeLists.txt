cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
list(APPEND EXTRA_COMPONENT_DIRS
    components/app_main
    components/esp32-shared
    components/bme280
)
project(suncam)
set(IDF_TARGET esp32)

# Add main component
add_subdirectory(main)
