cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

project(JoltC VERSION 1.0.0 LANGUAGES CXX)

# ============================================================================
#  Options
# ============================================================================
option(JOLTC_DOUBLE_PRECISION "Use double precision for world-space positions" OFF)

# ============================================================================
#  Build Jolt as a static library (we wrap it)
# ============================================================================
if(NOT DEFINED PHYSICS_REPO_ROOT)
    set(PHYSICS_REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../JoltPhysics)
endif()
file(TO_CMAKE_PATH "${PHYSICS_REPO_ROOT}" PHYSICS_REPO_ROOT)
include(${PHYSICS_REPO_ROOT}/Jolt/Jolt.cmake)

# Ensure Jolt static lib is built with PIC (required for linking into shared lib on Linux)
set_target_properties(Jolt PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ============================================================================
#  JoltC shared library
# ============================================================================
set(JOLTC_SRC
    src/errors.cpp
    src/common.cpp
    src/math.cpp
    src/shape.cpp
    src/physics_system.cpp
    src/body.cpp
    src/body_access.cpp
    src/constraint.cpp
    src/query.cpp
    src/character.cpp
    src/filters.cpp
    src/vehicle.cpp
    src/skeleton.cpp
)

add_library(JoltC SHARED ${JOLTC_SRC})

# Compile definitions
target_compile_definitions(JoltC PRIVATE JOLTC_EXPORTS)
if (JOLTC_DOUBLE_PRECISION)
    target_compile_definitions(JoltC PUBLIC JOLTC_DOUBLE_PRECISION)
    target_compile_definitions(JoltC PRIVATE JPH_DOUBLE_PRECISION)
endif()

# C++ standard
set_target_properties(JoltC PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

# Include directories
target_include_directories(JoltC
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${PHYSICS_REPO_ROOT}  # For Jolt headers (e.g. #include <Jolt/Jolt.h>)
)

# Link Jolt static library
target_link_libraries(JoltC PRIVATE Jolt)

# ============================================================================
#  Install
# ============================================================================
include(GNUInstallDirs)

install(TARGETS JoltC
    EXPORT JoltCExport
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/JoltC
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h"
)

install(EXPORT JoltCExport
    FILE JoltCConfig.cmake
    NAMESPACE JoltC::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/JoltC
)
