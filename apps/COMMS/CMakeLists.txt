cmake_minimum_required(VERSION 3.20.0)

set(BOARD ut_core)

# The RFM26W radio driver is an out-of-tree Zephyr module that lives in this app.
list(APPEND EXTRA_ZEPHYR_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/drivers/rfm26w)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(comms_main)

# COMMS_MODULE_SOURCES: shared with the host tests (apps/tests/commsTest)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/modules.cmake)

target_sources(app PRIVATE
    src/main.c
    ${COMMS_MODULE_SOURCES}
    ${COMMS_TARGET_ONLY_SOURCES}
    $ENV{ROOT}/common/can_proto.c)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src $ENV{ROOT})

add_subdirectory($ENV{ROOT}/common ${CMAKE_BINARY_DIR}/common)
target_link_libraries(app PRIVATE common)
