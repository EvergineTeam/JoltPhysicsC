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

# The debug renderer ships in every configuration since phase 6: the C API exposes it, and a
# distribution build without it would export functions that silently draw nothing. This is a
# binary size trade made on purpose -- the Draw implementations of every shape come along.
set(DEBUG_RENDERER_IN_DISTRIBUTION ON CACHE BOOL "" FORCE)

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
    src/soft_body.cpp
    src/state_recorder.cpp
    src/debug_renderer.cpp
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

# Google Play requires 64-bit native libraries to support 16 KB memory pages, and refuses
# updates that do not from 1 February 2027. The shipped android-arm64 libJoltC.so already
# satisfies it, but only because the workflow passes ANDROID_SUPPORT_FLEXIBLE_PAGE_SIZES=ON
# and the runner's NDK understands it. Neither half is guaranteed: the NDK is not pinned
# anywhere and comes from the runner image, and on an NDK older than r27 that option is an
# unknown cache variable, ignored without a word. The binary would go back to 4 KB and
# every leg would stay green.
#
# The README's manual build does not pass the option at all, so building this by hand
# already produces a library that could not be published.
#
# Only JoltC needs it: Jolt is STATIC and linked in, and an archive has no program headers
# to align. Guarded on the shared branch because JOLTC_STATIC produces an archive here too.
if(ANDROID AND NOT JOLTC_STATIC)
    target_link_options(JoltC PRIVATE "-Wl,-z,max-page-size=16384")
endif()

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
