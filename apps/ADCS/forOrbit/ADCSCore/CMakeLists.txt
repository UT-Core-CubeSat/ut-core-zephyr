cmake_minimum_required(VERSION 3.10)

project(ADCSCore_Sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# COMPILER OPTIONS
# ============================================================================
if(MSVC)
    add_compile_options(/W3)
else()
    add_compile_options(-Wall -Wextra)
endif()

# Default to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ============================================================================
# INCLUDE DIRECTORIES
# ============================================================================
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/components)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/plant)

# ============================================================================
# CORE LIBRARY (Flight Software - the "black box")
# ============================================================================
set(CORE_SOURCES
    components/ADCSCore.cpp
    components/core_Observer.cpp
    components/core_ControllerManager.cpp
    components/core_ControllerNDI.cpp
    components/core_ControllerBDot.cpp
    components/core_ReferenceGenerator.cpp
    components/core_HelperFunctions.cpp
)

add_library(ADCSCore STATIC ${CORE_SOURCES})
target_include_directories(ADCSCore PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/components
)

# ============================================================================
# PLANT LIBRARY (Simulation Environment)
# ============================================================================
set(PLANT_SOURCES
    plant/Dynamics.cpp
    plant/Sensors.cpp
    plant/two_body.cpp
)

add_library(Plant STATIC ${PLANT_SOURCES})
target_include_directories(Plant PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/plant
    ${CMAKE_CURRENT_SOURCE_DIR}/components
)
target_link_libraries(Plant ADCSCore)

# ============================================================================
# SIMULATION EXECUTABLE
# ============================================================================
add_executable(PlantSim plant/plant_main.cpp)
target_link_libraries(PlantSim Plant ADCSCore)

# ============================================================================
# OUTPUT DIRECTORIES
# ============================================================================
set(BUILD_OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

set_target_properties(PlantSim PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${BUILD_OUTPUT_DIR}
)
set_target_properties(ADCSCore Plant PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${BUILD_OUTPUT_DIR}
)

# ============================================================================
# SUMMARY
# ============================================================================
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Output directory: ${BUILD_OUTPUT_DIR}")