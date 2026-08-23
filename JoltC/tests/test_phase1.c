/* JoltC Test Suite — phase 1: physics materials end to end and shape introspection.
 * SPDX-License-Identifier: MIT
 *
 * Before this phase a JoltC_PhysicsMaterial could be created and connected to nothing: no shape
 * accepted a material list and nothing ever handed a material back. Every case here closes that
 * loop somewhere: a mesh built with materials returns the right one for the triangle a ray hit,
 * a height field knows the material of each cell and can be repainted and deformed at runtime,
 * a character reports what it stands on, and the triangle walk reads geometry back out of a shape.
 */

#include "test_common.h"

#include <float.h>
#include <string.h>

/* A 4 x 4 metre floor of four triangles on two materials: the two triangles at x < 0 are grass,
 * the two at x > 0 are metal. Shared by the mesh material and character tests. */
static const JoltC_Shape* create_two_material_floor(JoltC_PhysicsMaterial* grass, JoltC_PhysicsMaterial* metal)
{
    JoltC_Vec3 vertices[6] = {
        { -2.0f, 0.0f, -2.0f }, /* 0 */
        {  0.0f, 0.0f, -2.0f }, /* 1 */
        {  2.0f, 0.0f, -2.0f }, /* 2 */
        { -2.0f, 0.0f,  2.0f }, /* 3 */
        {  0.0f, 0.0f,  2.0f }, /* 4 */
        {  2.0f, 0.0f,  2.0f }, /* 5 */
    };
    JoltC_IndexedTriangle triangles[4] = {
        { 0, 3, 4, 0, 0 }, /* west, grass */
        { 0, 4, 1, 0, 0 }, /* west, grass */
        { 1, 4, 5, 1, 0 }, /* east, metal */
        { 1, 5, 2, 1, 0 }, /* east, metal */
    };
    JoltC_PhysicsMaterial* materials[2] = { grass, metal };

    return JoltC_MeshShape_Create2(vertices, 6, triangles, 4, materials, 2);
}

void run_phase1_tests(void);

void run_phase1_tests(void)
{
    /* test_mesh_materials_reach_the_query */
    TEST_BEGIN("A ray reports the material of the triangle it hit");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_PhysicsMaterial* grass = JoltC_PhysicsMaterial_Create("grass", 0xFF00FF00u);
        JoltC_PhysicsMaterial* metal = JoltC_PhysicsMaterial_Create("metal", 0xFFAAAAAAu);
        TEST_ASSERT_NOT_NULL(grass, "grass material created");
        TEST_ASSERT(strcmp(JoltC_PhysicsMaterial_GetDebugName(grass), "grass") == 0, "name survives the handle");

        const JoltC_Shape* floor = create_two_material_floor(grass, metal);
        TEST_ASSERT_NOT_NULL(floor, "mesh with a material list created");

        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_RVec3 floorPos = { 0.0f, 0.0f, 0.0f };
        JoltC_BodyCreationSettings* bodySettings = JoltC_BodyCreationSettings_Create3(
            floor, floorPos, identity, JOLTC_MOTION_TYPE_STATIC, OBJ_LAYER_STATIC);
        JoltC_BodyInterface_CreateAndAddBody(ctx.bodyInterface, bodySettings, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_BodyCreationSettings_Destroy(bodySettings);

        const JoltC_NarrowPhaseQuery* query = JoltC_PhysicsSystem_GetNarrowPhaseQuery(ctx.physicsSystem);
        JoltC_Vec3 down = { 0.0f, -4.0f, 0.0f };

        JoltC_RayCastResult west;
        JoltC_RVec3 westOrigin = { -1.0f, 2.0f, 0.0f };
        TEST_ASSERT(JoltC_NarrowPhaseQuery_CastRay(query, westOrigin, down, &west), "west ray hits the floor");
        const JoltC_PhysicsMaterial* westMaterial = JoltC_Shape_GetMaterial(floor, west.subShapeID2);
        TEST_ASSERT(westMaterial == grass, "west of the floor is the very grass handle we created");

        JoltC_RayCastResult east;
        JoltC_RVec3 eastOrigin = { 1.0f, 2.0f, 0.0f };
        TEST_ASSERT(JoltC_NarrowPhaseQuery_CastRay(query, eastOrigin, down, &east), "east ray hits the floor");
        const JoltC_PhysicsMaterial* eastMaterial = JoltC_Shape_GetMaterial(floor, east.subShapeID2);
        TEST_ASSERT(eastMaterial == metal, "east of the floor is metal");
        TEST_ASSERT(strcmp(JoltC_PhysicsMaterial_GetDebugName(eastMaterial), "metal") == 0,
                    "the material read back out of the shape still knows its name");

        /* The shape holds its own references, so the caller's handles can go first. */
        JoltC_PhysicsMaterial_Destroy(grass);
        JoltC_PhysicsMaterial_Destroy(metal);
        TEST_ASSERT(strcmp(JoltC_PhysicsMaterial_GetDebugName(JoltC_Shape_GetMaterial(floor, west.subShapeID2)), "grass") == 0,
                    "the material outlives the handle because the shape keeps it alive");

        JoltC_Shape_Release(floor);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_default_material_everywhere */
    TEST_BEGIN("A shape built without materials returns the default");
    {
        JoltC_Vec3 half = { 1.0f, 1.0f, 1.0f };
        const JoltC_Shape* box = JoltC_BoxShape_Create(half, 0.05f);
        const JoltC_PhysicsMaterial* material = JoltC_Shape_GetMaterial(box, 0);
        TEST_ASSERT_NOT_NULL(material, "every shape has a material, if only the default");
        TEST_ASSERT_NOT_NULL(JoltC_PhysicsMaterial_GetDebugName(material), "the default has a name");
        JoltC_Shape_Release(box);
    }
    TEST_END();

    /* test_height_field_materials_per_cell */
    TEST_BEGIN("A height field knows the material of each cell");
    {
        JoltC_PhysicsMaterial* sand = JoltC_PhysicsMaterial_Create("sand", 0xFFFFDD88u);
        JoltC_PhysicsMaterial* rock = JoltC_PhysicsMaterial_Create("rock", 0xFF888888u);

        /* An 8x8 sample field of 7x7 cells: the west half sand, the east half rock. */
        enum { N = 8 };
        float samples[N * N];
        uint8_t cellMaterials[(N - 1) * (N - 1)];
        for (int y = 0; y < N; y++)
            for (int x = 0; x < N; x++)
                samples[(y * N) + x] = 0.0f;
        for (int y = 0; y < N - 1; y++)
            for (int x = 0; x < N - 1; x++)
                cellMaterials[(y * (N - 1)) + x] = (x < (N - 1) / 2) ? 0 : 1;

        JoltC_PhysicsMaterial* materials[2] = { sand, rock };
        JoltC_Vec3 offset = { 0.0f, 0.0f, 0.0f };
        JoltC_Vec3 scale = { 1.0f, 1.0f, 1.0f };
        const JoltC_Shape* field = JoltC_HeightFieldShape_Create2(samples, offset, scale, N, cellMaterials, materials, 2);
        TEST_ASSERT_NOT_NULL(field, "height field with materials created");

        TEST_ASSERT(JoltC_HeightFieldShape_GetMaterial(field, 1, 3) == sand, "a west cell is sand");
        TEST_ASSERT(JoltC_HeightFieldShape_GetMaterial(field, 5, 3) == rock, "an east cell is rock");

        /* The region read back matches what the shape was built with: the sand/rock border
         * runs between cells x=2 and x=3, and the read straddles it. */
        uint8_t readBack[2 * 2];
        JoltC_HeightFieldShape_GetMaterials(field, 2, 2, 2, 2, readBack, 2);
        TEST_ASSERT(readBack[0] == 0 && readBack[1] == 1, "the read straddles the sand/rock border");

        JoltC_PhysicsMaterial_Destroy(sand);
        JoltC_PhysicsMaterial_Destroy(rock);
        JoltC_Shape_Release(field);
    }
    TEST_END();

    /* test_height_field_deforms_at_runtime */
    TEST_BEGIN("A height field deforms at runtime and collisions follow");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        /* The initial heights must span the range the deformation will use: the quantisation
         * range is fixed when the shape is built, so an all-flat field could never be raised. */
        enum { N = 8 };
        float samples[N * N];
        for (int y = 0; y < N; y++)
            for (int x = 0; x < N; x++)
                samples[(y * N) + x] = (x == 0 && y == 0) ? 4.0f : 0.0f;

        JoltC_Vec3 offset = { 0.0f, 0.0f, 0.0f };
        JoltC_Vec3 scale = { 1.0f, 1.0f, 1.0f };
        const JoltC_Shape* field = JoltC_HeightFieldShape_Create(samples, offset, scale, N);
        TEST_ASSERT_NOT_NULL(field, "height field created");

        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_RVec3 fieldPos = { 0.0f, 0.0f, 0.0f };
        JoltC_BodyCreationSettings* bodySettings = JoltC_BodyCreationSettings_Create3(
            field, fieldPos, identity, JOLTC_MOTION_TYPE_STATIC, OBJ_LAYER_STATIC);
        JoltC_BodyInterface_CreateAndAddBody(ctx.bodyInterface, bodySettings, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_BodyCreationSettings_Destroy(bodySettings);

        /* Raise a 4x4 block of samples to two metres. */
        float raised[4 * 4];
        for (int i = 0; i < 4 * 4; i++) raised[i] = 2.0f;
        JoltC_HeightFieldShape_SetHeights(field, 2, 2, 4, 4, raised, 4, ctx.tempAllocator, 0.996195f);

        float readBack[4 * 4];
        JoltC_HeightFieldShape_GetHeights(field, 2, 2, 4, 4, readBack, 4);
        TEST_ASSERT_FLOAT_EQ(readBack[5], 2.0f, 0.1f, "the raised heights read back, within quantisation");

        /* A ray dropped over the raised block now stops two metres higher than the old ground. */
        const JoltC_NarrowPhaseQuery* query = JoltC_PhysicsSystem_GetNarrowPhaseQuery(ctx.physicsSystem);
        JoltC_Vec3 down = { 0.0f, -10.0f, 0.0f };
        JoltC_RVec3 above = { 4.0f, 5.0f, 4.0f };
        JoltC_RayCastResult hit;
        TEST_ASSERT(JoltC_NarrowPhaseQuery_CastRay(query, above, down, &hit), "ray hits the deformed terrain");
        float hitHeight = 5.0f + (hit.fraction * -10.0f);
        TEST_ASSERT_FLOAT_EQ(hitHeight, 2.0f, 0.15f, "the simulation sees the new surface");

        JoltC_Shape_Release(field);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_triangles_walk_out_of_a_shape */
    TEST_BEGIN("The triangle walk reads geometry back out of a mesh");
    {
        JoltC_PhysicsMaterial* grass = JoltC_PhysicsMaterial_Create("grass", 0xFF00FF00u);
        JoltC_PhysicsMaterial* metal = JoltC_PhysicsMaterial_Create("metal", 0xFFAAAAAAu);
        const JoltC_Shape* floor = create_two_material_floor(grass, metal);

        JoltC_Vec3 boxMin = { -100.0f, -100.0f, -100.0f };
        JoltC_Vec3 boxMax = { 100.0f, 100.0f, 100.0f };
        JoltC_Vec3 zero = { 0.0f, 0.0f, 0.0f };
        JoltC_Vec3 unit = { 1.0f, 1.0f, 1.0f };
        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };

        JoltC_GetTrianglesContext* context = JoltC_Shape_GetTrianglesStart(floor, boxMin, boxMax, zero, identity, unit);
        TEST_ASSERT_NOT_NULL(context, "triangle walk started");

        float vertices[64 * 9];
        const JoltC_PhysicsMaterial* materials[64];
        int total = 0;
        int grassSeen = 0, metalSeen = 0;
        for (;;)
        {
            int count = JoltC_Shape_GetTrianglesNext(floor, context, 64, vertices, materials);
            if (count <= 0) break;
            for (int i = 0; i < count; i++)
            {
                if (materials[i] == grass) grassSeen++;
                if (materials[i] == metal) metalSeen++;

                /* Every vertex of this floor lies at y = 0 within its 4 x 4 metre footprint. */
                for (int v = 0; v < 3; v++)
                {
                    float y = vertices[(i * 9) + (v * 3) + 1];
                    TEST_ASSERT_FLOAT_EQ(y, 0.0f, 0.001f, "walked vertex sits on the floor plane");
                }
            }
            total += count;
        }
        JoltC_GetTrianglesContext_Destroy(context);

        TEST_ASSERT(total == 4, "all four triangles came back");
        TEST_ASSERT(grassSeen == 2 && metalSeen == 2, "each triangle carries its own material");

        TEST_ASSERT(JoltC_Shape_GetTrianglesNext(floor, NULL, 64, vertices, NULL) == -1, "a null context is refused");

        JoltC_PhysicsMaterial_Destroy(grass);
        JoltC_PhysicsMaterial_Destroy(metal);
        JoltC_Shape_Release(floor);
    }
    TEST_END();

    /* test_sub_shape_user_data_and_leaf */
    TEST_BEGIN("A compound hit resolves to the leaf and its user data");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_Vec3 half = { 0.5f, 0.5f, 0.5f };
        const JoltC_Shape* west = JoltC_BoxShape_Create(half, 0.05f);
        const JoltC_Shape* east = JoltC_BoxShape_Create(half, 0.05f);
        JoltC_Shape_SetUserData(west, 111);
        JoltC_Shape_SetUserData(east, 222);

        JoltC_CompoundShapeSubShape children[2];
        children[0].position = (JoltC_Vec3){ -1.0f, 0.0f, 0.0f };
        children[0].rotation = (JoltC_Quat){ 0.0f, 0.0f, 0.0f, 1.0f };
        children[0].shape = west;
        children[0].userData = 0;
        children[1].position = (JoltC_Vec3){ 1.0f, 0.0f, 0.0f };
        children[1].rotation = (JoltC_Quat){ 0.0f, 0.0f, 0.0f, 1.0f };
        children[1].shape = east;
        children[1].userData = 0;

        const JoltC_Shape* compound = JoltC_StaticCompoundShape_Create(children, 2);
        TEST_ASSERT_NOT_NULL(compound, "compound created");

        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_RVec3 origin = { 0.0f, 0.0f, 0.0f };
        JoltC_BodyCreationSettings* bodySettings = JoltC_BodyCreationSettings_Create3(
            compound, origin, identity, JOLTC_MOTION_TYPE_STATIC, OBJ_LAYER_STATIC);
        JoltC_BodyInterface_CreateAndAddBody(ctx.bodyInterface, bodySettings, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_BodyCreationSettings_Destroy(bodySettings);

        const JoltC_NarrowPhaseQuery* query = JoltC_PhysicsSystem_GetNarrowPhaseQuery(ctx.physicsSystem);
        JoltC_Vec3 down = { 0.0f, -4.0f, 0.0f };
        JoltC_RVec3 aboveWest = { -1.0f, 2.0f, 0.0f };
        JoltC_RayCastResult hit;
        TEST_ASSERT(JoltC_NarrowPhaseQuery_CastRay(query, aboveWest, down, &hit), "ray hits the west child");

        TEST_ASSERT(JoltC_Shape_GetSubShapeUserData(compound, hit.subShapeID2) == 111,
                    "the hit reports the user data of the child, not the compound");

        uint32_t remainder = 0;
        const JoltC_Shape* leaf = JoltC_Shape_GetLeafShape(compound, hit.subShapeID2, &remainder);
        TEST_ASSERT(leaf == west, "the leaf the id resolves to is the west box itself");

        JoltC_Shape_Release(compound);
        JoltC_Shape_Release(west);
        JoltC_Shape_Release(east);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_submerged_volume */
    TEST_BEGIN("Half a box under water displaces half its volume");
    {
        JoltC_Vec3 half = { 1.0f, 1.0f, 1.0f };
        const JoltC_Shape* box = JoltC_BoxShape_Create(half, 0.05f);

        JoltC_Mat44 transform;
        JoltC_Mat44_Identity(&transform);
        JoltC_Vec3 unit = { 1.0f, 1.0f, 1.0f };
        JoltC_Vec3 waterNormal = { 0.0f, 1.0f, 0.0f };

        float total = 0.0f, submerged = 0.0f;
        JoltC_Vec3 buoyancy;
        JoltC_Shape_GetSubmergedVolume(box, &transform, unit, waterNormal, 0.0f, &total, &submerged, &buoyancy);

        TEST_ASSERT_FLOAT_EQ(total, 8.0f, 0.4f, "a 2 metre box holds eight cubic metres");
        TEST_ASSERT_FLOAT_EQ(submerged, 4.0f, 0.3f, "half of it sits under a surface through its middle");
        TEST_ASSERT(buoyancy.y < 0.0f, "the centre of buoyancy is in the submerged half");

        JoltC_Shape_Release(box);
    }
    TEST_END();

    /* test_character_reports_ground_material */
    TEST_BEGIN("A character knows the material it stands on");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_PhysicsMaterial* grass = JoltC_PhysicsMaterial_Create("grass", 0xFF00FF00u);
        JoltC_PhysicsMaterial* metal = JoltC_PhysicsMaterial_Create("metal", 0xFFAAAAAAu);
        const JoltC_Shape* floor = create_two_material_floor(grass, metal);

        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_RVec3 floorPos = { 0.0f, 0.0f, 0.0f };
        JoltC_BodyCreationSettings* bodySettings = JoltC_BodyCreationSettings_Create3(
            floor, floorPos, identity, JOLTC_MOTION_TYPE_STATIC, OBJ_LAYER_STATIC);
        JoltC_BodyInterface_CreateAndAddBody(ctx.bodyInterface, bodySettings, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_BodyCreationSettings_Destroy(bodySettings);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.3f);
        JoltC_CharacterVirtualSettings settings;
        JoltC_CharacterVirtualSettings_Init(&settings);
        settings.shape = capsule;

        /* Dropped over the grass half. */
        JoltC_RVec3 spawn = { -1.0f, 2.0f, 0.0f };
        JoltC_CharacterVirtual* character = JoltC_CharacterVirtual_Create(&settings, spawn, identity, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(character, "character created");

        JoltC_Vec3 gravity = { 0.0f, -9.81f, 0.0f };
        JoltC_Vec3 fall = { 0.0f, -5.0f, 0.0f };
        for (int i = 0; i < 120; i++)
        {
            JoltC_CharacterVirtual_SetLinearVelocity(character, fall);
            JoltC_CharacterVirtual_Update(character, 1.0f / 60.0f, gravity, ctx.tempAllocator);
            JoltC_PhysicsSystem_Update(ctx.physicsSystem, 1.0f / 60.0f, 1, ctx.tempAllocator, ctx.jobSystem);
        }

        TEST_ASSERT(JoltC_CharacterVirtual_GetGroundState(character) == JOLTC_GROUND_STATE_ON_GROUND,
                    "the character landed");
        const JoltC_PhysicsMaterial* ground = JoltC_CharacterVirtual_GetGroundMaterial(character);
        TEST_ASSERT(ground == grass, "it stands on the grass triangles, and says so");

        JoltC_CharacterVirtual_Destroy(character);
        JoltC_Shape_Release(capsule);
        JoltC_PhysicsMaterial_Destroy(grass);
        JoltC_PhysicsMaterial_Destroy(metal);
        JoltC_Shape_Release(floor);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}
