/* JoltC Test Suite -- shape properties and settings round-trips
 * SPDX-License-Identifier: MIT
 *
 * test_shape.c covers construction: it creates one of each shape and checks the handle,
 * type and volume. It reads almost nothing back. This file covers the accessors, which is
 * where a hand repair during a version bump goes wrong without the compiler noticing.
 *
 * Every value here is asymmetric on purpose. A test that passes (1,1,1) cannot tell a
 * correct converter from one that swaps two components, and swapping two components is
 * precisely the mistake that survives compilation.
 *
 * Nothing here asserts a trajectory. JoltPhysics 5.6.0 replaces the friction model and
 * changes solver behaviour, so an assertion about where a body ends up after N steps would
 * break on the bump for a legitimate reason and cost somebody an afternoon proving it.
 */

#include "test_common.h"

/* Distinctive, asymmetric, and none of them a default. */
#define HALF_X 1.5f
#define HALF_Y 2.25f
#define HALF_Z 0.75f

static void test_box_dimensions_round_trip(void)
{
    TEST_BEGIN("BoxShape half extent and convex radius round-trip");
    JoltC_Vec3 half = { HALF_X, HALF_Y, HALF_Z };
    const JoltC_Shape* box = JoltC_BoxShape_Create(half, 0.03125f);
    TEST_ASSERT_NOT_NULL(box, "box created");

    if (box) {
        JoltC_Vec3 got = JoltC_BoxShape_GetHalfExtent(box);
        /* Read component by component. A single vector comparison would pass if two
         * components were swapped and happened to be compared as a whole. */
        TEST_ASSERT_FLOAT_EQ(got.x, HALF_X, 0.0001f, "half extent x survives");
        TEST_ASSERT_FLOAT_EQ(got.y, HALF_Y, 0.0001f, "half extent y survives");
        TEST_ASSERT_FLOAT_EQ(got.z, HALF_Z, 0.0001f, "half extent z survives");
        TEST_ASSERT_FLOAT_EQ(JoltC_BoxShape_GetConvexRadius(box), 0.03125f, 0.0001f,
                             "convex radius survives");
        JoltC_Shape_Release(box);
    }
    TEST_END();
}

static void test_sphere_capsule_cylinder_round_trip(void)
{
    TEST_BEGIN("Sphere, capsule and cylinder dimensions round-trip");

    const JoltC_Shape* sphere = JoltC_SphereShape_Create(0.625f);
    if (sphere) {
        TEST_ASSERT_FLOAT_EQ(JoltC_SphereShape_GetRadius(sphere), 0.625f, 0.0001f,
                             "sphere radius survives");
        JoltC_Shape_Release(sphere);
    }

    /* Half height and radius are both floats in the same signature, in that order --
     * the classic pair to get the wrong way round. Deliberately far apart in value. */
    const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(1.75f, 0.25f);
    if (capsule) {
        TEST_ASSERT_FLOAT_EQ(JoltC_CapsuleShape_GetHalfHeightOfCylinder(capsule), 1.75f,
                             0.0001f, "capsule half height is not the radius");
        TEST_ASSERT_FLOAT_EQ(JoltC_CapsuleShape_GetRadius(capsule), 0.25f, 0.0001f,
                             "capsule radius is not the half height");
        JoltC_Shape_Release(capsule);
    }

    const JoltC_Shape* cylinder = JoltC_CylinderShape_Create(2.5f, 0.375f, 0.03125f);
    if (cylinder) {
        TEST_ASSERT_FLOAT_EQ(JoltC_CylinderShape_GetHalfHeight(cylinder), 2.5f, 0.0001f,
                             "cylinder half height is not the radius");
        TEST_ASSERT_FLOAT_EQ(JoltC_CylinderShape_GetRadius(cylinder), 0.375f, 0.0001f,
                             "cylinder radius is not the half height");
        JoltC_Shape_Release(cylinder);
    }
    TEST_END();
}

static void test_user_data_round_trip(void)
{
    TEST_BEGIN("Shape user data survives a 64-bit round-trip");
    const JoltC_Shape* box = JoltC_BoxShape_Create((JoltC_Vec3){ 1.0f, 1.0f, 1.0f }, 0.05f);
    if (box) {
        /* Both halves of the 64 bits set to different values, so a truncation to 32 bits
         * or a sign-extension shows up rather than passing. */
        uint64_t sentinel = 0xDEADBEEF12345678ULL;
        JoltC_Shape_SetUserData(box, sentinel);
        TEST_ASSERT(JoltC_Shape_GetUserData(box) == sentinel,
                    "user data survives all 64 bits");
        JoltC_Shape_Release(box);
    }
    TEST_END();
}

static void test_convex_density_round_trip(void)
{
    TEST_BEGIN("ConvexShape density round-trips and changes mass");
    const JoltC_Shape* box = JoltC_BoxShape_Create((JoltC_Vec3){ 0.5f, 0.5f, 0.5f }, 0.05f);
    if (box) {
        float defaultMass = 0.0f, denseMass = 0.0f;
        JoltC_Mat44 inertia;

        JoltC_Shape_GetMassProperties(box, &defaultMass, &inertia);
        TEST_ASSERT(defaultMass > 0.0f, "a unit box has positive mass");

        JoltC_ConvexShape_SetDensity(box, 2500.0f);
        TEST_ASSERT_FLOAT_EQ(JoltC_ConvexShape_GetDensity(box), 2500.0f, 0.01f,
                             "density survives");

        JoltC_Shape_GetMassProperties(box, &denseMass, &inertia);
        /* Relational, not absolute: the point is that density reaches the mass
         * calculation, not what Jolt's default density happens to be. */
        TEST_ASSERT(denseMass > defaultMass,
                    "raising density raises mass, so the setter is connected");
        JoltC_Shape_Release(box);
    }
    TEST_END();
}

static void test_decorator_shapes_round_trip(void)
{
    TEST_BEGIN("Scaled and RotatedTranslated decorators report what they were given");
    const JoltC_Shape* inner = JoltC_BoxShape_Create((JoltC_Vec3){ 0.5f, 0.5f, 0.5f }, 0.05f);
    TEST_ASSERT_NOT_NULL(inner, "inner shape created");

    if (inner) {
        JoltC_Vec3 scale = { 2.0f, 3.0f, 4.0f };
        const JoltC_Shape* scaled = JoltC_ScaledShape_Create(inner, scale);
        if (scaled) {
            JoltC_Vec3 got = JoltC_ScaledShape_GetScale(scaled);
            TEST_ASSERT_FLOAT_EQ(got.x, 2.0f, 0.0001f, "scale x survives");
            TEST_ASSERT_FLOAT_EQ(got.y, 3.0f, 0.0001f, "scale y survives");
            TEST_ASSERT_FLOAT_EQ(got.z, 4.0f, 0.0001f, "scale z survives");
            JoltC_Shape_Release(scaled);
        }

        /* A rotation with all four components distinct and non-zero, so a reordered
         * quaternion cannot pass. Normalised by hand: (1,2,3,4)/sqrt(30). */
        JoltC_Vec3 offset = { 1.25f, -2.5f, 3.125f };
        JoltC_Quat rot = { 0.182574f, 0.365148f, 0.547723f, 0.730297f };
        const JoltC_Shape* rt = JoltC_RotatedTranslatedShape_Create(offset, rot, inner);
        if (rt) {
            JoltC_Vec3 gotPos = JoltC_RotatedTranslatedShape_GetPosition(rt);
            TEST_ASSERT_FLOAT_EQ(gotPos.x, 1.25f, 0.001f, "offset x survives");
            TEST_ASSERT_FLOAT_EQ(gotPos.y, -2.5f, 0.001f, "offset y survives, sign included");
            TEST_ASSERT_FLOAT_EQ(gotPos.z, 3.125f, 0.001f, "offset z survives");

            JoltC_Quat gotRot = JoltC_RotatedTranslatedShape_GetRotation(rt);
            TEST_ASSERT_FLOAT_EQ(gotRot.x, 0.182574f, 0.001f, "rotation x survives");
            TEST_ASSERT_FLOAT_EQ(gotRot.y, 0.365148f, 0.001f, "rotation y survives");
            TEST_ASSERT_FLOAT_EQ(gotRot.z, 0.547723f, 0.001f, "rotation z survives");
            TEST_ASSERT_FLOAT_EQ(gotRot.w, 0.730297f, 0.001f,
                                 "rotation w survives, not rotated into x");
            JoltC_Shape_Release(rt);
        }
        JoltC_Shape_Release(inner);
    }
    TEST_END();
}

static void test_shape_geometry_queries(void)
{
    TEST_BEGIN("Centre of mass, inner radius and static-only reporting");

    /* An offset decorator moves the centre of mass away from the origin, which is what
     * makes this assertable without depending on Jolt's internals. */
    const JoltC_Shape* inner = JoltC_BoxShape_Create((JoltC_Vec3){ 0.5f, 0.5f, 0.5f }, 0.05f);
    if (inner) {
        JoltC_Vec3 com = JoltC_Shape_GetCenterOfMass(inner);
        TEST_ASSERT_FLOAT_EQ(com.x, 0.0f, 0.001f, "a centred box has com at origin, x");
        TEST_ASSERT_FLOAT_EQ(com.y, 0.0f, 0.001f, "a centred box has com at origin, y");
        TEST_ASSERT_FLOAT_EQ(com.z, 0.0f, 0.001f, "a centred box has com at origin, z");

        TEST_ASSERT(JoltC_Shape_GetInnerRadius(inner) > 0.0f, "box has positive inner radius");
        TEST_ASSERT(JoltC_Shape_MustBeStatic(inner) == 0, "a box may be dynamic");
        TEST_ASSERT(JoltC_Shape_GetSubShapeIDBitsRecursive(inner) >= 0,
                    "sub shape id bits is readable");

        JoltC_Vec3 offset = { 5.0f, 0.0f, 0.0f };
        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        const JoltC_Shape* moved = JoltC_RotatedTranslatedShape_Create(offset, identity, inner);
        if (moved) {
            JoltC_Vec3 movedCom = JoltC_Shape_GetCenterOfMass(moved);
            TEST_ASSERT_FLOAT_EQ(movedCom.x, 5.0f, 0.01f,
                                 "translating the shape moves the centre of mass with it");
            JoltC_Shape_Release(moved);
        }
        JoltC_Shape_Release(inner);
    }
    TEST_END();
}

static void test_shape_ray_and_point(void)
{
    TEST_BEGIN("CastRay and CollidePoint against a unit box");
    /* Geometry, not simulation: these answers are fixed by the shape's definition and do
     * not depend on the solver, so they are safe across a version bump. */
    const JoltC_Shape* box = JoltC_BoxShape_Create((JoltC_Vec3){ 1.0f, 1.0f, 1.0f }, 0.0f);
    if (box) {
        TEST_ASSERT(JoltC_Shape_CollidePoint(box, (JoltC_Vec3){ 0.0f, 0.0f, 0.0f }) != 0,
                    "the origin is inside a box centred there");
        TEST_ASSERT(JoltC_Shape_CollidePoint(box, (JoltC_Vec3){ 9.0f, 0.0f, 0.0f }) == 0,
                    "a point well outside is outside");

        /* From x = -5 along +x, the near face of a half-extent-1 box is at x = -1, so the
         * hit is 4/5 of the way along a 5-unit direction. */
        float fraction = -1.0f;
        int hit = JoltC_Shape_CastRay(box, (JoltC_Vec3){ -5.0f, 0.0f, 0.0f },
                                      (JoltC_Vec3){ 5.0f, 0.0f, 0.0f }, &fraction);
        TEST_ASSERT(hit != 0, "a ray aimed at the box hits it");
        if (hit) {
            TEST_ASSERT_FLOAT_EQ(fraction, 0.8f, 0.01f, "hit fraction is 4/5");
        }

        float missFraction = -1.0f;
        int miss = JoltC_Shape_CastRay(box, (JoltC_Vec3){ -5.0f, 9.0f, 0.0f },
                                       (JoltC_Vec3){ 5.0f, 0.0f, 0.0f }, &missFraction);
        TEST_ASSERT(miss == 0, "a ray passing above the box misses");
        JoltC_Shape_Release(box);
    }
    TEST_END();
}

static void test_mutable_compound_modification(void)
{
    TEST_BEGIN("MutableCompoundShape add, modify, remove");
    const JoltC_Shape* child = JoltC_BoxShape_Create((JoltC_Vec3){ 0.5f, 0.5f, 0.5f }, 0.05f);
    TEST_ASSERT_NOT_NULL(child, "child shape created");

    /* Jolt requires a compound to be created with at least one sub-shape; there is no
     * empty-compound constructor. So the first child comes in through Create and the
     * second through AddShape, which is also the more interesting pair to compare. */
    JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
    JoltC_CompoundShapeSubShape initial;
    initial.shape = child;
    initial.position = (JoltC_Vec3){ 1.5f, 0.0f, 0.0f };
    initial.rotation = identity;
    initial.userData = 11u;

    const JoltC_Shape* compound = JoltC_MutableCompoundShape_Create(&initial, 1);
    TEST_ASSERT_NOT_NULL(compound, "compound created");

    if (compound && child) {
        TEST_ASSERT(JoltC_CompoundShape_GetNumSubShapes(compound) == 1,
                    "Create with one sub-shape gives one child");

        uint32_t second = JoltC_MutableCompoundShape_AddShape(
            compound, (JoltC_Vec3){ -1.5f, 0.0f, 0.0f }, identity, child, 22u);
        TEST_ASSERT(JoltC_CompoundShape_GetNumSubShapes(compound) == 2,
                    "AddShape brings it to two");

        JoltC_MutableCompoundShape_ModifyShape(
            compound, 0u, (JoltC_Vec3){ 0.0f, 2.5f, 0.0f }, identity);
        JoltC_MutableCompoundShape_AdjustCenterOfMass(compound);
        TEST_ASSERT(JoltC_CompoundShape_GetNumSubShapes(compound) == 2,
                    "modifying does not change the child count");

        JoltC_MutableCompoundShape_RemoveShape(compound, second);
        TEST_ASSERT(JoltC_CompoundShape_GetNumSubShapes(compound) == 1,
                    "removing one leaves one");
    }
    if (child) JoltC_Shape_Release(child);
    if (compound) JoltC_Shape_Release(compound);
    TEST_END();
}

static void test_height_field_settings_round_trip(void)
{
    TEST_BEGIN("HeightFieldShapeSettings properties round-trip");
    /* 4x4 is the smallest sample count Jolt accepts, and the values are deliberately
     * uneven so quantisation has something to work with. */
    float samples[16] = {
        0.0f,  0.5f,  1.0f,  1.5f,
        0.25f, 0.75f, 1.25f, 1.75f,
        0.5f,  1.0f,  1.5f,  2.0f,
        0.75f, 1.25f, 1.75f, 2.25f
    };
    JoltC_Vec3 offset = { -2.0f, 0.5f, -2.0f };
    JoltC_Vec3 scale  = { 1.25f, 2.5f, 1.25f };

    JoltC_HeightFieldShapeSettings* settings =
        JoltC_HeightFieldShapeSettings_Create(samples, offset, scale, 4u);
    TEST_ASSERT_NOT_NULL(settings, "height field settings created");

    if (settings) {
        JoltC_Vec3 gotOffset = JoltC_HeightFieldShapeSettings_GetOffset(settings);
        TEST_ASSERT_FLOAT_EQ(gotOffset.x, -2.0f, 0.0001f, "offset x survives");
        TEST_ASSERT_FLOAT_EQ(gotOffset.y, 0.5f, 0.0001f, "offset y survives");
        TEST_ASSERT_FLOAT_EQ(gotOffset.z, -2.0f, 0.0001f, "offset z survives");

        JoltC_Vec3 gotScale = JoltC_HeightFieldShapeSettings_GetScale(settings);
        TEST_ASSERT_FLOAT_EQ(gotScale.y, 2.5f, 0.0001f,
                             "scale y survives and is not confused with x");

        TEST_ASSERT(JoltC_HeightFieldShapeSettings_GetSampleCount(settings) == 4u,
                    "sample count is what was passed");

        /* 5.6.0 raises the maximum bits per sample from 8 to 16 to allow height fields
         * closer to their uncompressed values. Asserting the round-trip at 8 keeps this
         * test valid on both sides of the bump; the wrapper accepting 16 afterwards is a
         * separate question for whoever takes the release. */
        JoltC_HeightFieldShapeSettings_SetBitsPerSample(settings, 8u);
        TEST_ASSERT(JoltC_HeightFieldShapeSettings_GetBitsPerSample(settings) == 8u,
                    "bits per sample survives");

        JoltC_HeightFieldShapeSettings_SetBlockSize(settings, 2u);
        TEST_ASSERT(JoltC_HeightFieldShapeSettings_GetBlockSize(settings) == 2u,
                    "block size survives");

        JoltC_HeightFieldShapeSettings_SetMinHeightValue(settings, -3.5f);
        JoltC_HeightFieldShapeSettings_SetMaxHeightValue(settings, 7.25f);
        TEST_ASSERT_FLOAT_EQ(JoltC_HeightFieldShapeSettings_GetMinHeightValue(settings),
                             -3.5f, 0.0001f, "min height survives, sign included");
        TEST_ASSERT_FLOAT_EQ(JoltC_HeightFieldShapeSettings_GetMaxHeightValue(settings),
                             7.25f, 0.0001f, "max height survives and is not the min");

        const JoltC_Shape* shape = JoltC_HeightFieldShapeSettings_CreateShape(settings);
        TEST_ASSERT_NOT_NULL(shape, "settings produce a shape");
        if (shape) {
            TEST_ASSERT(JoltC_Shape_MustBeStatic(shape) != 0,
                        "a height field must be static");
            JoltC_Shape_Release(shape);
        }
        JoltC_HeightFieldShapeSettings_Destroy(settings);
    }
    TEST_END();
}

static void test_shape_accessor_null_safety(void)
{
    TEST_BEGIN("Shape accessors survive a null handle");
    /* The wrapper checks handles before its try block. If a repair drops one of those
     * checks the process dies here rather than in a consumer's application. */
    TEST_ASSERT(JoltC_Shape_GetUserData(NULL) == 0, "GetUserData(NULL) returns zero");
    TEST_ASSERT(JoltC_Shape_GetVolume(NULL) == 0.0f, "GetVolume(NULL) returns zero");
    TEST_ASSERT(JoltC_Shape_MustBeStatic(NULL) == 0, "MustBeStatic(NULL) returns zero");
    TEST_ASSERT(JoltC_Shape_CollidePoint(NULL, (JoltC_Vec3){ 0.0f, 0.0f, 0.0f }) == 0,
                "CollidePoint(NULL) returns zero");
    JoltC_Shape_SetUserData(NULL, 1u);   /* must not crash */
    TEST_END();
}

void run_shape_props_tests(void)
{
    printf("\n=== Shape properties and settings ===\n");
    test_box_dimensions_round_trip();
    test_sphere_capsule_cylinder_round_trip();
    test_user_data_round_trip();
    test_convex_density_round_trip();
    test_decorator_shapes_round_trip();
    test_shape_geometry_queries();
    test_shape_ray_and_point();
    test_mutable_compound_modification();
    test_height_field_settings_round_trip();
    test_shape_accessor_null_safety();
}
