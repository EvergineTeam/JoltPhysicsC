/* JoltC Test Suite -- shape.h API tests
 * SPDX-License-Identifier: MIT
 */

#include "test_common.h"

#define PI_F 3.14159265358979323846f

void run_shape_tests(void)
{
    /* test_box_shape */
    TEST_BEGIN("BoxShape create, type, volume");
    {
        JoltC_Vec3 half = { 1.0f, 1.0f, 1.0f };
        const JoltC_Shape* s = JoltC_BoxShape_Create(half, 0.0f);
        TEST_ASSERT_NOT_NULL(s, "BoxShape not null");
        TEST_ASSERT(JoltC_Shape_GetType(s) == JOLTC_SHAPE_TYPE_CONVEX, "type == CONVEX");
        TEST_ASSERT(JoltC_Shape_GetSubType(s) == JOLTC_SHAPE_SUB_TYPE_BOX, "subtype == BOX");
        float vol = JoltC_Shape_GetVolume(s);
        TEST_ASSERT_FLOAT_EQ(vol, 8.0f, 0.1f, "volume ~ 8");
        JoltC_Shape_Release(s);
    }
    TEST_END();

    /* test_sphere_shape */
    TEST_BEGIN("SphereShape create, type, volume");
    {
        const JoltC_Shape* s = JoltC_SphereShape_Create(0.5f);
        TEST_ASSERT_NOT_NULL(s, "SphereShape not null");
        TEST_ASSERT(JoltC_Shape_GetSubType(s) == JOLTC_SHAPE_SUB_TYPE_SPHERE, "subtype == SPHERE");
        float vol = JoltC_Shape_GetVolume(s);
        float expected = (4.0f / 3.0f) * PI_F * 0.5f * 0.5f * 0.5f;
        TEST_ASSERT_FLOAT_EQ(vol, expected, 0.05f, "volume ~ 4/3*pi*r^3");
        JoltC_Shape_Release(s);
    }
    TEST_END();

    /* test_capsule_shape */
    TEST_BEGIN("CapsuleShape create, type");
    {
        const JoltC_Shape* s = JoltC_CapsuleShape_Create(1.0f, 0.5f);
        TEST_ASSERT_NOT_NULL(s, "CapsuleShape not null");
        TEST_ASSERT(JoltC_Shape_GetSubType(s) == JOLTC_SHAPE_SUB_TYPE_CAPSULE, "subtype == CAPSULE");
        JoltC_Shape_Release(s);
    }
    TEST_END();

    /* test_cylinder_shape */
    TEST_BEGIN("CylinderShape create, type");
    {
        const JoltC_Shape* s = JoltC_CylinderShape_Create(1.0f, 0.5f, 0.0f);
        TEST_ASSERT_NOT_NULL(s, "CylinderShape not null");
        TEST_ASSERT(JoltC_Shape_GetSubType(s) == JOLTC_SHAPE_SUB_TYPE_CYLINDER, "subtype == CYLINDER");
        JoltC_Shape_Release(s);
    }
    TEST_END();

    /* test_tapered_capsule_shape */
    TEST_BEGIN("TaperedCapsuleShape create, type");
    {
        const JoltC_Shape* s = JoltC_TaperedCapsuleShape_Create(1.0f, 0.5f, 0.3f);
        TEST_ASSERT_NOT_NULL(s, "TaperedCapsuleShape not null");
        TEST_ASSERT(JoltC_Shape_GetSubType(s) == JOLTC_SHAPE_SUB_TYPE_TAPERED_CAPSULE, "subtype == TAPERED_CAPSULE");
        JoltC_Shape_Release(s);
    }
    TEST_END();

    /* test_convex_hull_shape */
    TEST_BEGIN("ConvexHullShape from 8 cube vertices");
    {
        JoltC_Vec3 points[8] = {
            {-1,-1,-1}, { 1,-1,-1}, { 1, 1,-1}, {-1, 1,-1},
            {-1,-1, 1}, { 1,-1, 1}, { 1, 1, 1}, {-1, 1, 1}
        };
        const JoltC_Shape* s = JoltC_ConvexHullShape_Create(points, 8, 0.0f);
        TEST_ASSERT_NOT_NULL(s, "ConvexHullShape not null");
        TEST_ASSERT(JoltC_Shape_GetSubType(s) == JOLTC_SHAPE_SUB_TYPE_CONVEX_HULL, "subtype == CONVEX_HULL");
        JoltC_Shape_Release(s);
    }
    TEST_END();

    /* test_static_compound_shape */
    TEST_BEGIN("StaticCompoundShape with 2 sub-shapes");
    {
        JoltC_Vec3 half = { 0.5f, 0.5f, 0.5f };
        const JoltC_Shape* box = JoltC_BoxShape_Create(half, 0.0f);
        const JoltC_Shape* sphere = JoltC_SphereShape_Create(0.5f);

        JoltC_CompoundShapeSubShape subs[2];
        subs[0].position = (JoltC_Vec3){ 0.0f, 0.0f, 0.0f };
        subs[0].rotation = (JoltC_Quat){ 0.0f, 0.0f, 0.0f, 1.0f };
        subs[0].shape = box;
        subs[0].userData = 0;
        subs[1].position = (JoltC_Vec3){ 2.0f, 0.0f, 0.0f };
        subs[1].rotation = (JoltC_Quat){ 0.0f, 0.0f, 0.0f, 1.0f };
        subs[1].shape = sphere;
        subs[1].userData = 0;

        const JoltC_Shape* compound = JoltC_StaticCompoundShape_Create(subs, 2);
        TEST_ASSERT_NOT_NULL(compound, "StaticCompound not null");
        TEST_ASSERT(JoltC_Shape_GetSubType(compound) == JOLTC_SHAPE_SUB_TYPE_STATIC_COMPOUND, "subtype == STATIC_COMPOUND");
        int numSubs = JoltC_CompoundShape_GetNumSubShapes(compound);
        TEST_ASSERT(numSubs == 2, "2 sub-shapes");

        JoltC_Shape_Release(compound);
        JoltC_Shape_Release(sphere);
        JoltC_Shape_Release(box);
    }
    TEST_END();

    /* test_mutable_compound_shape */
    TEST_BEGIN("MutableCompoundShape create and add");
    {
        /* MutableCompoundShape_Create requires at least 1 sub-shape */
        JoltC_Vec3 half = { 0.5f, 0.5f, 0.5f };
        const JoltC_Shape* box = JoltC_BoxShape_Create(half, 0.0f);
        JoltC_CompoundShapeSubShape initSub;
        initSub.shape = box;
        initSub.position = (JoltC_Vec3){ 0.0f, 0.0f, 0.0f };
        initSub.rotation = (JoltC_Quat){ 0.0f, 0.0f, 0.0f, 1.0f };
        initSub.userData = 0;

        const JoltC_Shape* compound = JoltC_MutableCompoundShape_Create(&initSub, 1);
        TEST_ASSERT_NOT_NULL(compound, "MutableCompound not null");
        TEST_ASSERT(JoltC_Shape_GetSubType(compound) == JOLTC_SHAPE_SUB_TYPE_MUTABLE_COMPOUND, "subtype == MUTABLE_COMPOUND");

        /* Add a second sub-shape */
        const JoltC_Shape* sphere = JoltC_SphereShape_Create(1.0f);
        JoltC_Vec3 pos = { 2.0f, 0.0f, 0.0f };
        JoltC_Quat rot = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_MutableCompoundShape_AddShape(compound, pos, rot, sphere, 0);
        int count = JoltC_CompoundShape_GetNumSubShapes(compound);
        TEST_ASSERT(count == 2, "2 sub-shapes after add");

        JoltC_Shape_Release(sphere);
        JoltC_Shape_Release(box);
        JoltC_Shape_Release(compound);
    }
    TEST_END();

    /* test_shape_get_local_bounds */
    TEST_BEGIN("Shape GetLocalBounds for box");
    {
        JoltC_Vec3 half = { 1.0f, 2.0f, 3.0f };
        const JoltC_Shape* s = JoltC_BoxShape_Create(half, 0.0f);
        JoltC_AABox bounds = JoltC_Shape_GetLocalBounds(s);
        TEST_ASSERT_FLOAT_EQ(bounds.min.x, -1.0f, 0.01f, "min.x ~ -1");
        TEST_ASSERT_FLOAT_EQ(bounds.max.x, 1.0f, 0.01f, "max.x ~ 1");
        TEST_ASSERT_FLOAT_EQ(bounds.min.y, -2.0f, 0.01f, "min.y ~ -2");
        TEST_ASSERT_FLOAT_EQ(bounds.max.y, 2.0f, 0.01f, "max.y ~ 2");
        JoltC_Shape_Release(s);
    }
    TEST_END();

    /* test_shape_ref_counting */
    TEST_BEGIN("Shape AddRef / Release no crash");
    {
        const JoltC_Shape* s = JoltC_SphereShape_Create(1.0f);
        JoltC_Shape_AddRef(s);
        JoltC_Shape_Release(s);
        JoltC_Shape_Release(s); /* final release */
        TEST_ASSERT(1, "No crash");
    }
    TEST_END();

    /* test_offset_decorated_shape */
    TEST_BEGIN("OffsetCenterOfMassShape create, type");
    {
        const JoltC_Shape* inner = JoltC_SphereShape_Create(0.5f);
        JoltC_Vec3 offset = { 1.0f, 0.0f, 0.0f };
        const JoltC_Shape* decorated = JoltC_OffsetCenterOfMassShape_Create(inner, offset);
        TEST_ASSERT_NOT_NULL(decorated, "OffsetCOM not null");
        TEST_ASSERT(JoltC_Shape_GetType(decorated) == JOLTC_SHAPE_TYPE_DECORATED, "type == DECORATED");
        TEST_ASSERT(JoltC_Shape_GetSubType(decorated) == JOLTC_SHAPE_SUB_TYPE_OFFSET_CENTER_OF_MASS, "subtype == OFFSET_COM");
        JoltC_Shape_Release(decorated);
        JoltC_Shape_Release(inner);
    }
    TEST_END();

    /* test_scaled_decorated_shape */
    TEST_BEGIN("ScaledShape create, type");
    {
        const JoltC_Shape* inner = JoltC_SphereShape_Create(0.5f);
        JoltC_Vec3 scale = { 2.0f, 2.0f, 2.0f };
        const JoltC_Shape* scaled = JoltC_ScaledShape_Create(inner, scale);
        TEST_ASSERT_NOT_NULL(scaled, "ScaledShape not null");
        TEST_ASSERT(JoltC_Shape_GetSubType(scaled) == JOLTC_SHAPE_SUB_TYPE_SCALED, "subtype == SCALED");
        JoltC_Shape_Release(scaled);
        JoltC_Shape_Release(inner);
    }
    TEST_END();

    /* test_mesh_shape */
    TEST_BEGIN("MeshShape from 2-triangle quad");
    {
        JoltC_Vec3 verts[4] = {
            {-1, 0, -1}, { 1, 0, -1}, { 1, 0, 1}, {-1, 0, 1}
        };
        JoltC_IndexedTriangle tris[2] = {
            { 0, 1, 2, 0, 0 },
            { 0, 2, 3, 0, 0 }
        };
        const JoltC_Shape* s = JoltC_MeshShape_Create(verts, 4, tris, 2);
        TEST_ASSERT_NOT_NULL(s, "MeshShape not null");
        TEST_ASSERT(JoltC_Shape_GetType(s) == JOLTC_SHAPE_TYPE_MESH, "type == MESH");
        JoltC_Shape_Release(s);
    }
    TEST_END();

    /* test_heightfield_shape */
    TEST_BEGIN("HeightFieldShape from 4x4 grid");
    {
        float samples[16] = {
            0, 0, 0, 0,
            0, 1, 1, 0,
            0, 1, 1, 0,
            0, 0, 0, 0
        };
        JoltC_Vec3 offset = { 0.0f, 0.0f, 0.0f };
        JoltC_Vec3 scale = { 1.0f, 1.0f, 1.0f };
        const JoltC_Shape* s = JoltC_HeightFieldShape_Create(samples, offset, scale, 4);
        TEST_ASSERT_NOT_NULL(s, "HeightFieldShape not null");
        TEST_ASSERT(JoltC_Shape_GetType(s) == JOLTC_SHAPE_TYPE_HEIGHT_FIELD, "type == HEIGHT_FIELD");
        JoltC_Shape_Release(s);
    }
    TEST_END();
}
