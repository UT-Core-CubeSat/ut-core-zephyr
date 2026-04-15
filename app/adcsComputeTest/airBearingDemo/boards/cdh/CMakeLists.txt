cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cdh)


# C / C++ sources
target_sources(app PRIVATE
src/can_bus.c
src/soh.c
src/internal_sensors.c
src/ground_cmds.c
src/app_main.cpp
)


target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)