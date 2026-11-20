cmake_minimum_required(VERSION 3.20.0)

# Runs on the host, not on the target: the radio is a software model on an emulated SPI bus.
set(BOARD native_sim/native/64)

# The RFM26W driver is an out-of-tree Zephyr module that lives in the COMMS app.
list(APPEND EXTRA_ZEPHYR_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../../COMMS/drivers/rfm26w)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rfm26w_emul_test)

# The radio model is shared with apps/tests/commsTest
set(RFM26W_EMUL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../COMMS/drivers/rfm26w/emul)

target_sources(app PRIVATE src/main.c ${RFM26W_EMUL_DIR}/rfm26w_emul.c)
target_include_directories(app PRIVATE ${RFM26W_EMUL_DIR})
