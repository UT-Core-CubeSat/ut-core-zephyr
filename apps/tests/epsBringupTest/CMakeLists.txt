# Standalone Bringup(initial testing phase)/bench test for EPS GPIO + INA3221 wiring, no CAN.
# See apps/EPS/src/main.c for the full flight node.

cmake_minimum_required(VERSION 3.20.0)

set(BOARD ut_core)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(epsBringupTest)

target_sources(app PRIVATE src/main.c)
