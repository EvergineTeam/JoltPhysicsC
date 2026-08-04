# Built as part of the JoltC build, not as a standalone project. It used to be its own
# project() linking a literal path -- ../build/Release/JoltC.lib -- which pinned the suite to
# Windows, to a Release configuration, and to one particular build directory. That is why
# eleven suites of real tests existed and no CI has ever run them.

# The suite is C; the library around it is C++, and the parent project only enables CXX.
enable_language(C)

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

# The target carries its own public include directory, so nothing here needs to know where
# the headers live.
target_link_libraries(JoltC_Tests PRIVATE JoltC)

# Built alongside the library rather than copying the library next to the test. Windows
# resolves a DLL from the executable's own directory, so this satisfies that without a copy
# step -- and a copy would have put a second JoltC.dll inside the build tree, which the
# release workflow globs for. One library in one place stays true.
set_target_properties(JoltC_Tests PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "$<TARGET_FILE_DIR:JoltC>"
)

add_test(NAME JoltC_Tests COMMAND JoltC_Tests)
