cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

project(JoltC VERSION 1.0.0 LANGUAGES CXX)

# ============================================================================
#  Options
# ============================================================================
option(JOLTC_DOUBLE_PRECISION "Use double precision for world-space positions" OFF)
option(JOLTC_STATIC "Build JoltC as a static library instead of shared" OFF)
option(JOLTC_BUILD_TESTS "Build and register the C test suite" OFF)

# ============================================================================
#  Build Jolt as a static library (we wrap it)
# ============================================================================
if(NOT DEFINED PHYSICS_REPO_ROOT)
    set(PHYSICS_REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../JoltPhysics)
endif()
file(TO_CMAKE_PATH "${PHYSICS_REPO_ROOT}" PHYSICS_REPO_ROOT)

# Jolt.cmake is included directly rather than going through JoltPhysics/Build/CMakeLists.txt,
# which is where upstream declares its options. That means none of them are defined here, and
# Jolt.cmake reads them as empty. Today that yields exactly what we want -- but by accident,
# and the set of variables it reads grew from 3 to 11 between 5.5.0 and 5.6.0. Pin the ones
# that decide what gets built, so a future release changes behaviour only when we say so.
#
# The compute backends matter most: upstream defaults all four to ON, and building them on
# macOS requires dxc and spirv-cross to be installed. We do not expose compute through the C
# API, so there is nothing to gain and a toolchain dependency to lose.
set(JPH_USE_DX12        OFF CACHE BOOL "" FORCE)
set(JPH_USE_VK          OFF CACHE BOOL "" FORCE)
set(JPH_USE_MTL         OFF CACHE BOOL "" FORCE)
set(JPH_USE_CPU_COMPUTE OFF CACHE BOOL "" FORCE)

# Jolt itself stays static and is linked into our shared library. Jolt.cmake derives
# JPH_LIB_TYPE from this; leaving it undefined happens to give STATIC too, which is how this
# built before the variable existed.
set(JPH_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

include(${PHYSICS_REPO_ROOT}/Jolt/Jolt.cmake)

# Ensure Jolt static lib is built with PIC (required for linking into shared lib on Linux)
set_target_properties(Jolt PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ============================================================================
#  JoltC library
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

if (JOLTC_STATIC)
    add_library(JoltC STATIC ${JOLTC_SRC})
else()
    add_library(JoltC SHARED ${JOLTC_SRC})
    target_compile_definitions(JoltC PRIVATE JOLTC_EXPORTS)
endif()

# Compile definitions
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
#  Tests
# ============================================================================
if (JOLTC_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

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

if (NOT JOLTC_STATIC)
    install(EXPORT JoltCExport
        FILE JoltCConfig.cmake
        NAMESPACE JoltC::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/JoltC
    )
endif()
