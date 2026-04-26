cmake_minimum_required(VERSION 3.10)

project(ADCSCore_Sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# BUILD OPTION
# ============================================================================
option(USE_ORBIT "Build for orbit simulation instead of air bearing demo" OFF)

if(USE_ORBIT)
    set(PROJECT_TYPE "forOrbit")
    message(STATUS "Building for orbit simulation")
else()
    set(PROJECT_TYPE "airBearingDemo")
    message(STATUS "Building for air bearing demo")
endif()

set(BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../${PROJECT_TYPE}/ADCSCore)

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
include_directories(${BASE_DIR}/components)
include_directories(${BASE_DIR}/plant)

# ============================================================================
# CORE LIBRARY (Flight Software - the "black box")
# ============================================================================
set(CORE_SOURCES
    ${BASE_DIR}/components/ADCSCore.cpp
    ${BASE_DIR}/components/core_Observer.cpp
    ${BASE_DIR}/components/core_ControllerManager.cpp
    ${BASE_DIR}/components/core_ControllerNDI.cpp
    ${BASE_DIR}/components/core_ControllerBDot.cpp
    ${BASE_DIR}/components/core_HelperFunctions.cpp
    ${BASE_DIR}/components/core_MTQAllocator.cpp
)

if(USE_ORBIT)
    list(APPEND CORE_SOURCES ${BASE_DIR}/components/core_ReferenceGenerator.cpp)
endif()

add_library(ADCSCore STATIC ${CORE_SOURCES})
target_include_directories(ADCSCore PUBLIC 
    ${BASE_DIR}/components
)

# ============================================================================
# PLANT LIBRARY (Simulation Environment)
# ============================================================================
set(PLANT_SOURCES
    ${BASE_DIR}/plant/Dynamics.cpp
    ${BASE_DIR}/plant/Sensors.cpp
)

if(USE_ORBIT)
    list(APPEND PLANT_SOURCES ${BASE_DIR}/plant/two_body.cpp)
endif()

add_library(Plant STATIC ${PLANT_SOURCES})
target_include_directories(Plant PUBLIC 
    ${BASE_DIR}/plant
    ${BASE_DIR}/components
)
target_link_libraries(Plant ADCSCore)

# ============================================================================
# SIMULATION EXECUTABLE
# ============================================================================
add_executable(PlantSim ${BASE_DIR}/plant/plant_main.cpp)
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
message(STATUS "Project type: ${PROJECT_TYPE}")
message(STATUS "Output directory: ${BUILD_OUTPUT_DIR}")