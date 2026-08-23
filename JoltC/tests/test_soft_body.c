/* JoltC Test Suite -- live soft bodies: build shared settings, create, step, read, destroy.
 * SPDX-License-Identifier: MIT
 *
 * Until now JoltC_SoftBodyCreationSettings could be created but never configured, so no
 * test (and no consumer) had ever produced a living soft body. These tests cover the new
 * surface end to end: SoftBodySharedSettings construction from vertices and faces, the
 * creation-settings setters, and reading simulated vertex positions back through
 * SoftBodyMotionProperties.
 *
 * The dynamics assertions are deliberately loose (the free center sagged, the pinned
 * corners did not, the cube came to rest above the floor) so a solver change in a future
 * Jolt bump does not read as a binding regression. What must hold exactly: counts round
 * trip, faces round trip, and the bulk read agrees with the per-vertex read.
 */

#include "test_common.h"

void run_soft_body_tests(void);

#define CLOTH_N 8

void run_soft_body_tests(void)
{
    /* test_soft_body_cloth_pinned_corners */
    TEST_BEGIN("Soft body cloth sags between pinned corners");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        /* An 8x8 horizontal grid in the XZ plane, 0.2 apart, corners pinned. */
        JoltC_SoftBodySharedSettings* shared = JoltC_SoftBodySharedSettings_Create();
        TEST_ASSERT_NOT_NULL(shared, "shared settings created");

        for (int z = 0; z < CLOTH_N; z++)
        {
            for (int x = 0; x < CLOTH_N; x++)
            {
                int isCorner = (x == 0 || x == CLOTH_N - 1) && (z == 0 || z == CLOTH_N - 1);
                float invMass = isCorner ? 0.0f : 1.0f;
                uint32_t index = JoltC_SoftBodySharedSettings_AddVertex(
                    shared, 0.2f * x, 0.0f, 0.2f * z, invMass);
                TEST_ASSERT(index == (uint32_t)(z * CLOTH_N + x), "vertex index is sequential");
            }
        }

        for (int z = 0; z < CLOTH_N - 1; z++)
        {
            for (int x = 0; x < CLOTH_N - 1; x++)
            {
                uint32_t v00 = (uint32_t)(z * CLOTH_N + x);
                uint32_t v10 = v00 + 1;
                uint32_t v01 = v00 + CLOTH_N;
                uint32_t v11 = v01 + 1;
                JoltC_SoftBodySharedSettings_AddFace(shared, v00, v01, v11, 0);
                JoltC_SoftBodySharedSettings_AddFace(shared, v00, v11, v10, 0);
            }
        }

        TEST_ASSERT(JoltC_SoftBodySharedSettings_GetVertexCount(shared) == CLOTH_N * CLOTH_N,
                    "vertex count round trips");
        TEST_ASSERT(JoltC_SoftBodySharedSettings_GetFaceCount(shared) ==
                    2 * (CLOTH_N - 1) * (CLOTH_N - 1), "face count round trips");

        uint32_t f0, f1, f2;
        JoltC_SoftBodySharedSettings_GetFace(shared, 0, &f0, &f1, &f2);
        TEST_ASSERT(f0 == 0 && f1 == CLOTH_N && f2 == CLOTH_N + 1, "first face round trips");

        /* Compliance is inverse stiffness: 0 on the structural edges keeps the cloth
         * inextensible, while a large bend compliance leaves it free to drape (0 there
         * would make it an infinitely stiff plate and the sag assertion meaningless). */
        JoltC_SoftBodySharedSettings_CreateConstraints(
            shared, 0.0f, 1.0f, 1.0f, JOLTC_SOFT_BODY_BEND_TYPE_DISTANCE);
        JoltC_SoftBodySharedSettings_CalculateEdgeLengths(shared);
        JoltC_SoftBodySharedSettings_Optimize(shared);

        /* Creation settings: hang the cloth at y=2 and keep the body origin fixed so the
         * vertex positions stay comparable across steps. */
        JoltC_SoftBodyCreationSettings* creation = JoltC_SoftBodyCreationSettings_Create();
        TEST_ASSERT_NOT_NULL(creation, "creation settings created");
        JoltC_SoftBodyCreationSettings_SetSettings(creation, shared);
        JoltC_SoftBodyCreationSettings_SetPosition(creation, (JoltC_RVec3){ 0.0f, 2.0f, 0.0f });
        JoltC_SoftBodyCreationSettings_SetRotation(creation, (JoltC_Quat){ 0.0f, 0.0f, 0.0f, 1.0f });
        JoltC_SoftBodyCreationSettings_SetObjectLayer(creation, OBJ_LAYER_DYNAMIC);
        JoltC_SoftBodyCreationSettings_SetNumIterations(creation, 5);
        JoltC_SoftBodyCreationSettings_SetLinearDamping(creation, 0.1f);
        JoltC_SoftBodyCreationSettings_SetFriction(creation, 0.2f);
        JoltC_SoftBodyCreationSettings_SetRestitution(creation, 0.0f);
        JoltC_SoftBodyCreationSettings_SetGravityFactor(creation, 1.0f);
        JoltC_SoftBodyCreationSettings_SetPressure(creation, 0.0f);
        JoltC_SoftBodyCreationSettings_SetVertexRadius(creation, 0.0f);
        JoltC_SoftBodyCreationSettings_SetMaxLinearVelocity(creation, 500.0f);
        JoltC_SoftBodyCreationSettings_SetUpdatePosition(creation, 0);
        JoltC_SoftBodyCreationSettings_SetMakeRotationIdentity(creation, 1);
        JoltC_SoftBodyCreationSettings_SetAllowSleeping(creation, 0);
        JoltC_SoftBodyCreationSettings_SetUserData(creation, 42);

        JoltC_Body* body = JoltC_BodyInterface_CreateSoftBody(ctx.bodyInterface, creation);
        TEST_ASSERT_NOT_NULL(body, "soft body created");
        JoltC_SoftBodyCreationSettings_Destroy(creation);

        JoltC_BodyID bodyId = JoltC_Body_GetID(body);
        JoltC_BodyInterface_AddBody(ctx.bodyInterface, bodyId, JOLTC_ACTIVATION_ACTIVATE);
        TEST_ASSERT(JoltC_Body_IsSoftBody(body), "body reports soft");
        TEST_ASSERT(JoltC_Body_GetUserData(body) == 42, "user data survived creation");

        JoltC_SoftBodyMotionProperties* mp = JoltC_Body_GetSoftBodyMotionProperties(body);
        TEST_ASSERT_NOT_NULL(mp, "soft body motion properties reachable");
        TEST_ASSERT(JoltC_SoftBodyMotionProperties_GetVertexCount(mp) == CLOTH_N * CLOTH_N,
                    "runtime vertex count matches");

        for (int i = 0; i < 300; i++)
        {
            JoltC_PhysicsSystem_Update(ctx.physicsSystem, 1.0f / 60.0f, 1,
                                       ctx.tempAllocator, ctx.jobSystem);
        }

        /* Bulk read agrees with the per-vertex read, and the shape is a hammock: the
         * pinned corners kept their height, the free middle sagged. Positions are local
         * to the body's center of mass, so only relative heights are asserted. */
        JoltC_Vec3 positions[CLOTH_N * CLOTH_N];
        uint32_t read = JoltC_SoftBodyMotionProperties_GetVertexPositions(
            mp, positions, CLOTH_N * CLOTH_N);
        TEST_ASSERT(read == CLOTH_N * CLOTH_N, "bulk read returns every vertex");

        JoltC_Vec3 single;
        JoltC_SoftBodyMotionProperties_GetVertexPosition(mp, CLOTH_N * CLOTH_N / 2, &single);
        JoltC_Vec3 bulk = positions[CLOTH_N * CLOTH_N / 2];
        TEST_ASSERT_FLOAT_EQ(single.x, bulk.x, 1e-6f, "bulk and single reads agree (x)");
        TEST_ASSERT_FLOAT_EQ(single.y, bulk.y, 1e-6f, "bulk and single reads agree (y)");
        TEST_ASSERT_FLOAT_EQ(single.z, bulk.z, 1e-6f, "bulk and single reads agree (z)");

        float cornerY = positions[0].y;
        float centerY = positions[(CLOTH_N / 2) * CLOTH_N + CLOTH_N / 2].y;
        TEST_ASSERT(centerY < cornerY - 0.2f, "free center sagged below the pinned corners");
        TEST_ASSERT_FLOAT_EQ(positions[0].y, positions[CLOTH_N - 1].y, 1e-3f,
                             "pinned corners stayed level");

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, bodyId);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, bodyId);
        JoltC_SoftBodySharedSettings_Release(shared);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_soft_body_cube_rests_on_floor */
    TEST_BEGIN("Soft body cube falls and rests on the floor");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_Vec3 floorHalf = { 20.0f, 1.0f, 20.0f };
        const JoltC_Shape* floorShape = JoltC_BoxShape_Create(floorHalf, 0.05f);
        JoltC_RVec3 floorPos = { 0.0f, -1.0f, 0.0f };
        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_BodyCreationSettings* floorSettings = JoltC_BodyCreationSettings_Create3(
            floorShape, floorPos, identity, JOLTC_MOTION_TYPE_STATIC, OBJ_LAYER_STATIC);
        JoltC_BodyInterface_CreateAndAddBody(ctx.bodyInterface, floorSettings,
                                             JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_BodyCreationSettings_Destroy(floorSettings);
        JoltC_Shape_Release(floorShape);

        JoltC_SoftBodySharedSettings* cube = JoltC_SoftBodySharedSettings_CreateCube(4, 0.25f);
        TEST_ASSERT_NOT_NULL(cube, "cube factory returned settings");
        TEST_ASSERT(JoltC_SoftBodySharedSettings_GetVertexCount(cube) == 64,
                    "cube has 4^3 vertices");
        TEST_ASSERT(JoltC_SoftBodySharedSettings_GetFaceCount(cube) > 0,
                    "cube factory produced surface faces");
        JoltC_SoftBodySharedSettings_Optimize(cube);

        JoltC_SoftBodyCreationSettings* creation = JoltC_SoftBodyCreationSettings_Create();
        JoltC_SoftBodyCreationSettings_SetSettings(creation, cube);
        JoltC_SoftBodyCreationSettings_SetPosition(creation, (JoltC_RVec3){ 0.0f, 2.0f, 0.0f });
        JoltC_SoftBodyCreationSettings_SetObjectLayer(creation, OBJ_LAYER_DYNAMIC);

        JoltC_Body* body = JoltC_BodyInterface_CreateSoftBody(ctx.bodyInterface, creation);
        TEST_ASSERT_NOT_NULL(body, "cube soft body created");
        JoltC_SoftBodyCreationSettings_Destroy(creation);
        JoltC_BodyID bodyId = JoltC_Body_GetID(body);
        JoltC_BodyInterface_AddBody(ctx.bodyInterface, bodyId, JOLTC_ACTIVATION_ACTIVATE);

        JoltC_PhysicsSystem_OptimizeBroadPhase(ctx.physicsSystem);

        for (int i = 0; i < 180; i++)
        {
            JoltC_PhysicsSystem_Update(ctx.physicsSystem, 1.0f / 60.0f, 1,
                                       ctx.tempAllocator, ctx.jobSystem);
        }

        /* The body origin followed the fall (UpdatePosition defaults on) and stopped on
         * the floor instead of tunnelling through: below the spawn, above the slab. */
        JoltC_RVec3 position = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, bodyId);
        TEST_ASSERT(position.y < 1.5f, "cube fell from its spawn height");
        TEST_ASSERT(position.y > -0.5f, "cube did not tunnel through the floor");

        JoltC_SoftBodyMotionProperties* mp = JoltC_Body_GetSoftBodyMotionProperties(body);
        TEST_ASSERT_NOT_NULL(mp, "cube motion properties reachable");
        TEST_ASSERT(JoltC_SoftBodyMotionProperties_GetVolume(mp) > 0.0f,
                    "cube reports positive volume");

        JoltC_Vec3 boundsMin, boundsMax;
        JoltC_SoftBodyMotionProperties_GetLocalBounds(mp, &boundsMin, &boundsMax);
        TEST_ASSERT(boundsMax.x > boundsMin.x && boundsMax.y > boundsMin.y &&
                    boundsMax.z > boundsMin.z, "local bounds are non-degenerate");

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, bodyId);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, bodyId);
        JoltC_SoftBodySharedSettings_Release(cube);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_soft_body_motion_properties_null_for_rigid */
    TEST_BEGIN("Rigid body yields no soft body motion properties");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_Vec3 half = { 0.5f, 0.5f, 0.5f };
        const JoltC_Shape* shape = JoltC_BoxShape_Create(half, 0.05f);
        JoltC_RVec3 pos = { 0.0f, 0.0f, 0.0f };
        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_BodyCreationSettings* settings = JoltC_BodyCreationSettings_Create3(
            shape, pos, identity, JOLTC_MOTION_TYPE_DYNAMIC, OBJ_LAYER_DYNAMIC);
        JoltC_Body* body = JoltC_BodyInterface_CreateBodyDirect(ctx.bodyInterface, settings);
        TEST_ASSERT_NOT_NULL(body, "rigid body created");
        JoltC_BodyCreationSettings_Destroy(settings);
        JoltC_Shape_Release(shape);

        TEST_ASSERT(JoltC_Body_GetSoftBodyMotionProperties(body) == NULL,
                    "soft accessor guards against rigid bodies");

        JoltC_BodyID bodyId = JoltC_Body_GetID(body);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, bodyId);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}
