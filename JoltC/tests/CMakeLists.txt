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

    # Added ahead of the JoltPhysics 5.6.0 bump. Coverage was 234 of 1,280 functions with
    # body_access at none, so a hand repair could change behaviour with nothing to notice.
    # These four target what that bump actually touches.
    test_body_access.c
    test_math_roundtrip.c
    test_character_extra.c
    test_shape_props.c
    test_skeleton_extra.c
    test_vehicle_extra.c

    # The first suite that creates, registers, steps and destroys a live vehicle:
    # the path the three incompatible-handle faults shipped through untested.
    test_vehicle_live.c

    # Live soft bodies: shared settings construction, creation-settings setters, and
    # reading simulated vertices back. Before this surface existed the creation handle
    # was a write-only cul-de-sac.
    test_soft_body.c

    # The phase 0 coverage repairs: enum values that match Jolt's, real GetSettings and the
    # articulated ragdoll they unlock, character filters, the soft body contact listener,
    # and the query settings structs connected to their queries.
    test_phase0.c
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
