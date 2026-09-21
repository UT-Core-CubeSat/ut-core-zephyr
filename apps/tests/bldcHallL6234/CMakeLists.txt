cmake_minimum_required(VERSION 3.20.0)

set(BOARD ut_core)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bldc_hall_l6234)

target_sources(app PRIVATE src/main.c)
target_include_directories(app PRIVATE $ENV{ROOT})

add_subdirectory($ENV{ROOT}/common ${CMAKE_BINARY_DIR}/common)
target_link_libraries(app PRIVATE common)
