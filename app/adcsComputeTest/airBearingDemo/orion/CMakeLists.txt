cmake_minimum_required(VERSION 3.20.0)

# ──────────────────────────────────────────────────────────────────
# Orion B16 GNSS — Zephyr Application
#
# This is a standalone Zephyr app.  Build it from the orion/ folder:
#
#   west build -b esp32_devkitc_wroom/esp32/procpu -- -DBOARD_ROOT=.
#   west flash
#   west espressif monitor          (to see serial output)
#
# For host-side unit tests (no Zephyr, no hardware), see tests/CMakeLists.txt.
# ──────────────────────────────────────────────────────────────────

# ── Step 1: Let Zephyr set up the build system ──────────────────
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# ── Step 2: Name our project ────────────────────────────────────
project(orion_gnss C)

# ── Step 3: Tell Zephyr which .c files to compile ───────────────
#
# "app" is a CMake target that Zephyr created for us.
# We just list every .c file that should be part of our firmware image.

target_sources(app PRIVATE
    # ── The application entry point (has main()) ────────────────
    apps/gps_test_app.c

    # ── The driver library (portable C — no Zephyr deps in parser/commands) ─
    drivers/orion_b16/orion_b16_parser.c
    drivers/orion_b16/orion_b16_commands.c

    # ── The Zephyr integration layer (UART, threads, mutexes) ───
    drivers/orion_b16/orion_b16_driver.c

    # ── Configuration recipes ───────────────────────────────────
    drivers/orion_b16/orion_b16_config.c
)

# ── Step 4: Tell the compiler where to find our .h files ────────
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/orion_b16
)

# ── Optional: include the GPS simulator for loopback testing ────
# Uncomment CONFIG_ORION_B16_SIMULATOR=y in prj.conf to enable.
if(CONFIG_ORION_B16_SIMULATOR)
    target_sources(app PRIVATE
        simulators/gps_simulator.c
    )
endif()
