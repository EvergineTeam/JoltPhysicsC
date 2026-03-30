cmake_minimum_required(VERSION 3.20 FATAL_ERROR)
project(JoltC_Tests VERSION 1.0.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(JoltC_Tests
    test_main.c
    test_common.c
    test_math.c
    test_physics_system.c
    test_shape.c
    test_body.c
    test_constraint.c
    test_filter.c
    test_query.c
    test_character.c
    test_skeleton.c
    test_vehicle.c
)

target_include_directories(JoltC_Tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(JoltC_Tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../build/Release/JoltC.lib
)

# Copy JoltC.dll to the output directory so the test exe can find it
add_custom_command(TARGET JoltC_Tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/../build/Release/JoltC.dll"
        "$<TARGET_FILE_DIR:JoltC_Tests>"
)
