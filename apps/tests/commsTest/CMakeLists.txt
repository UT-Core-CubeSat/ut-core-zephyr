cmake_minimum_required(VERSION 3.20.0)

# Host tests for the COMMS firmware: runs on this computer (native_sim), not on the board.
set(BOARD native_sim/native/64)

set(COMMS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../COMMS)

# The radio driver, and the software model of the radio that stands in for the chip
list(APPEND EXTRA_ZEPHYR_MODULES ${COMMS_DIR}/drivers/rfm26w)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(comms_test)

# COMMS_MODULE_SOURCES: the same module list the firmware builds (apps/COMMS/src/modules.cmake)
include(${COMMS_DIR}/src/modules.cmake)

set(RFM26W_EMUL_DIR ${COMMS_DIR}/drivers/rfm26w/emul)

# Every .c file in src/ is a test file (one suite per file). CONFIGURE_DEPENDS: new files are picked up on rebuild.
file(GLOB COMMS_TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)

target_sources(app PRIVATE
    ${COMMS_TEST_SOURCES}
    ${COMMS_MODULE_SOURCES}
    ${RFM26W_EMUL_DIR}/rfm26w_emul.c)
target_include_directories(app PRIVATE ${COMMS_SRC_DIR} ${RFM26W_EMUL_DIR} $ENV{ROOT})
