cmake_minimum_required(VERSION 3.20.0)

set(BOARD ut_core)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(can_self_test)

target_sources(app PRIVATE src/main.c)
target_include_directories(app PRIVATE $ENV{ROOT})
