cmake_minimum_required(VERSION 3.20)

project(
        pointcloud_registration
        VERSION 0.1.0
        LANGUAGES CXX
)

# =========================
# Global configuration
# =========================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # Keep it compiler agnostic, can change later if absolutely necessary

# Export compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif ()

# =========================
# Options
# =========================
option(PCR_BUILD_TESTS "Build tests" ON)
option(PCR_BUILD_BENCHMARKS "Build benchmarks" ON)
option(PCR_BUILD_EXAMPLES "Build examples" ON)

# =========================
# CMake helpers
# =========================
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")


# Include Sanitization flags
include(Sanitizers)
include(CompilerWarnings)
include(FetchDependencies)


# =========================
# Core library
# =========================
add_library(pcr)

target_sources(pcr
        PRIVATE
        src/core/point.cpp
        src/core/point_cloud.cpp

        src/linalg/svd.cpp

        src/spatial/nearest_neighbour.cpp

        src/registration/icp.cpp

        src/io/ply.cpp
)

target_include_directories(pcr
        PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Set compilation warnings
set_project_warnings(pcr)

# =========================
# Apps / CLI
# =========================
add_subdirectory(apps/pcr_cli)

# =========================
# Examples
# =========================
if (PCR_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif ()

# =========================
# Tests
# =========================
if (PCR_BUILD_TESTS)
    # Enable CTest for this directory and below
    enable_testing()

    add_subdirectory(tests)
endif ()

# =========================
# Benchmarks
# =========================
if (PCR_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif ()
