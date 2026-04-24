# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(adcsComputeTest)

# ============================================================================
# Build target selection:
#   -DAIR_BEARING=ON   ->  Air Bearing Demo  (airBearingDemo ADCSCore)
#   -DAIR_BEARING=OFF  ->  Orbit Flight SW   (forOrbit ADCSCore)  [default]
# ============================================================================
option(AIR_BEARING "Build for air bearing demo instead of orbit" OFF)

if(AIR_BEARING)
    message(STATUS ">>> Building: Air Bearing Demo <<<")
    set(ADCS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/airBearingDemo/ADCSCore/components)

    target_sources(app PRIVATE
        src/mainAirBearing.cpp
        src/wheel_bridge.cpp
        src/adcs_can.cpp
        ${ADCS_DIR}/ADCSCore.cpp
        ${ADCS_DIR}/core_Observer.cpp
        ${ADCS_DIR}/core_ControllerManager.cpp
        ${ADCS_DIR}/core_ControllerNDI.cpp
        ${ADCS_DIR}/core_ControllerBDot.cpp
    )
else()
    message(STATUS ">>> Building: Orbit Flight SW <<<")
    set(ADCS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/forOrbit/ADCSCore/components)

    target_sources(app PRIVATE
        src/mainOrbit.cpp
        ${ADCS_DIR}/ADCSCore.cpp
        ${ADCS_DIR}/core_Observer.cpp
        ${ADCS_DIR}/core_ControllerManager.cpp
        ${ADCS_DIR}/core_ControllerNDI.cpp
        ${ADCS_DIR}/core_ControllerBDot.cpp
        ${ADCS_DIR}/core_ReferenceGenerator.cpp
        ${ADCS_DIR}/core_HelperFunctions.cpp
    )
endif()

target_include_directories(app PRIVATE
    ${ADCS_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common
)
