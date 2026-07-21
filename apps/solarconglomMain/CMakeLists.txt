cmake_minimum_required(VERSION 3.20.0)

set(BOARD ut_core)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(solar_main)

target_sources(app PRIVATE src/demo.c)
#target_sources(app PRIVATE src/main.c)
# target_include_directories(app PRIVATE ${ROOT}/core)
target_include_directories(app PRIVATE $ENV{ROOT}/core)
