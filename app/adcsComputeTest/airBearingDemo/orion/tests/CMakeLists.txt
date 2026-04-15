# ──────────────────────────────────────────────────────────────────
# Host-side unit tests for the Orion B16 parser and command builder.
#
# These tests run on your DESKTOP (Windows/Mac/Linux) with no Zephyr,
# no ESP32, no hardware at all.  They verify the protocol logic is correct.
#
# HOW TO BUILD AND RUN (from the orion/ directory):
#
#   cd tests
#   mkdir build
#   cd build
#   cmake ..
#   cmake --build .
#   ctest --verbose         (or just run ./test_orion_b16 directly)
#
# On Windows with MSVC:
#   cmake .. -G "Visual Studio 17 2022"
#   cmake --build . --config Release
#   ctest -C Release --verbose
# ──────────────────────────────────────────────────────────────────

cmake_minimum_required(VERSION 3.14)
project(orion_b16_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Warnings
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# Path to the driver source (one level up from tests/)
set(DRIVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../drivers/orion_b16)

# ── Driver static library (just the portable C — no Zephyr) ─────
add_library(orion_b16 STATIC
    ${DRIVER_DIR}/orion_b16_parser.c
    ${DRIVER_DIR}/orion_b16_commands.c
)
target_include_directories(orion_b16 PUBLIC ${DRIVER_DIR})

# ── Unit test executable ─────────────────────────────────────────
add_executable(test_orion_b16
    test_orion_b16.c
)
target_link_libraries(test_orion_b16 PRIVATE orion_b16)
target_include_directories(test_orion_b16 PRIVATE ${DRIVER_DIR})

# Link math library on Unix
if(UNIX)
    target_link_libraries(test_orion_b16 PRIVATE m)
endif()

# ── Register with CTest ─────────────────────────────────────────
enable_testing()
add_test(NAME orion_b16_unit_tests COMMAND test_orion_b16)
