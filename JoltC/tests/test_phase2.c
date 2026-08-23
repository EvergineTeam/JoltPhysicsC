/* JoltC Test Suite -- phase 2: the complete soft body surface.
 * SPDX-License-Identifier: MIT
 *
 * Before this phase the construction API was faces-plus-one-compliance: one attribute broadcast to
 * every vertex, no long range attachments (the LRA type was fixed at None), no direct constraint
 * building, no rods, no skinning, and vertices were read-only at runtime. Each case here exercises
 * something that was impossible, in a live simulation wherever the feature has behaviour to show.
 */

#include "test_common.h"

#include <float.h>

/* A box floor for the tests that drop something. */
static void add_floor(TestPhysicsContext* ctx)
{
    JoltC_Vec3 floorHalf = { 20.0f, 1.0f, 20.0f };
    const JoltC_Shape* floorShape = JoltC_BoxShape_Create(floorHalf, 0.05f);
    JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
    JoltC_RVec3 floorPos = { 0.0f, -1.0f, 0.0f };
    JoltC_BodyCreationSettings* floorSettings = JoltC_BodyCreationSettings_Create3(
        floorShape, floorPos, identity, JOLTC_MOTION_TYPE_STATIC, OBJ_LAYER_STATIC);
    JoltC_BodyInterface_CreateAndAddBody(ctx->bodyInterface, floorSettings, JOLTC_ACTIVATION_DONT_ACTIVATE);
    JoltC_BodyCreationSettings_Destroy(floorSettings);
    JoltC_Shape_Release(floorShape);
}

/* Creates a soft body from shared settings, adds it, and hands back body and id. */
static JoltC_Body* add_soft_body(TestPhysicsContext* ctx, JoltC_SoftBodySharedSettings* shared,
                                 JoltC_RVec3 position, JoltC_BodyID* outId)
{
    JoltC_SoftBodyCreationSettings* creation = JoltC_SoftBodyCreationSettings_Create();
    JoltC_SoftBodyCreationSettings_SetSettings(creation, shared);
    JoltC_SoftBodyCreationSettings_SetPosition(creation, position);
    JoltC_SoftBodyCreationSettings_SetObjectLayer(creation, OBJ_LAYER_DYNAMIC);
    JoltC_SoftBodyCreationSettings_SetNumIterations(creation, 10);
    JoltC_SoftBodyCreationSettings_SetUpdatePosition(creation, 0);
    JoltC_SoftBodyCreationSettings_SetAllowSleeping(creation, 0);

    JoltC_Body* body = JoltC_BodyInterface_CreateSoftBody(ctx->bodyInterface, creation);
    JoltC_SoftBodyCreationSettings_Destroy(creation);
    if (!body) return NULL;

    *outId = JoltC_Body_GetID(body);
    JoltC_BodyInterface_AddBody(ctx->bodyInterface, *outId, JOLTC_ACTIVATION_ACTIVATE);

    return body;
}

static void step(TestPhysicsContext* ctx, int steps)
{
    for (int i = 0; i < steps; i++)
        JoltC_PhysicsSystem_Update(ctx->physicsSystem, 1.0f / 60.0f, 1, ctx->tempAllocator, ctx->jobSystem);
}

void run_phase2_tests(void);

void run_phase2_tests(void)
{
    /* test_lra_from_per_vertex_attributes */
    TEST_BEGIN("Per-vertex attributes create long range attachments");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        /* A hanging strip, two columns wide and eight rows tall, pinned at the top row. The
         * springs are deliberately mushy: without the LRA the strip stretches well past its rest
         * length under gravity, and the attachment is what stops it. */
        enum { COLS = 2, ROWS = 8 };
        const float spacing = 0.25f;
        JoltC_SoftBodySharedSettings* shared = JoltC_SoftBodySharedSettings_Create();

        for (int r = 0; r < ROWS; r++)
            for (int c = 0; c < COLS; c++)
                JoltC_SoftBodySharedSettings_AddVertex(
                    shared, c * spacing, -r * spacing, 0.0f, r == 0 ? 0.0f : 1.0f);

        for (int r = 0; r < ROWS - 1; r++)
        {
            uint32_t v00 = (uint32_t)(r * COLS);
            uint32_t v10 = v00 + 1;
            uint32_t v01 = v00 + COLS;
            uint32_t v11 = v01 + 1;
            JoltC_SoftBodySharedSettings_AddFace(shared, v00, v01, v11, 0);
            JoltC_SoftBodySharedSettings_AddFace(shared, v00, v11, v10, 0);
        }

        JoltC_SoftBodyVertexAttributes attributes;
        attributes.compliance = 1.0e-3f; /* mushy on purpose */
        attributes.shearCompliance = 1.0e-3f;
        attributes.bendCompliance = FLT_MAX;
        attributes.lraType = JOLTC_SOFT_BODY_LRA_TYPE_EUCLIDEAN_DISTANCE;
        attributes.lraMaxDistanceMultiplier = 1.0f;

        /* One attribute for all vertices: Jolt repeats the last entry, which is the broadcast the
         * old API hard-coded -- except this one carries an LRA type, which it could not. */
        JoltC_SoftBodySharedSettings_CreateConstraints2(
            shared, &attributes, 1, JOLTC_SOFT_BODY_BEND_TYPE_NONE, 0.14f);
        JoltC_SoftBodySharedSettings_Optimize(shared);

        uint32_t lraCount = JoltC_SoftBodySharedSettings_GetLRAConstraintCount(shared);
        TEST_ASSERT(lraCount > 0, "LRA constraints exist at last (the old path always produced zero)");

        JoltC_BodyID bodyId;
        JoltC_RVec3 spawn = { 0.0f, 4.0f, 0.0f };
        JoltC_Body* body = add_soft_body(&ctx, shared, spawn, &bodyId);
        TEST_ASSERT_NOT_NULL(body, "strip created");

        step(&ctx, 180);

        /* The bottom vertex hangs at most the strip's rest length from its pin, mushy springs or
         * not: that is precisely the constraint the attachment enforces. */
        JoltC_SoftBodyMotionProperties* mp = JoltC_Body_GetSoftBodyMotionProperties(body);
        JoltC_Vec3 top, bottom;
        JoltC_SoftBodyMotionProperties_GetVertexPosition(mp, 0, &top);
        JoltC_SoftBodyMotionProperties_GetVertexPosition(mp, (ROWS - 1) * COLS, &bottom);

        float dx = bottom.x - top.x, dy = bottom.y - top.y, dz = bottom.z - top.z;
        float distance = (float)sqrt((dx * dx) + (dy * dy) + (dz * dz));
        float restLength = (ROWS - 1) * spacing;
        TEST_ASSERT(distance < restLength * 1.10f, "the attachment holds the strip near its rest length");

        JoltC_SoftBodySharedSettings_Release(shared);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_direct_constraint_construction */
    TEST_BEGIN("A tetrahedron built constraint by constraint holds its shape");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        add_floor(&ctx);

        JoltC_SoftBodySharedSettings* shared = JoltC_SoftBodySharedSettings_Create();

        /* Four vertices of a regular-ish tetrahedron, no faces at all: everything below is added
         * by hand, which the old API had no entry point for. */
        uint32_t v0 = JoltC_SoftBodySharedSettings_AddVertex(shared, 0.0f, 0.0f, 0.0f, 1.0f);
        uint32_t v1 = JoltC_SoftBodySharedSettings_AddVertex(shared, 1.0f, 0.0f, 0.0f, 1.0f);
        uint32_t v2 = JoltC_SoftBodySharedSettings_AddVertex(shared, 0.5f, 0.0f, 0.87f, 1.0f);
        uint32_t v3 = JoltC_SoftBodySharedSettings_AddVertex(shared, 0.5f, 0.82f, 0.29f, 1.0f);

        JoltC_SoftBodySharedSettings_AddEdgeConstraint(shared, v0, v1, 0.0f);
        JoltC_SoftBodySharedSettings_AddEdgeConstraint(shared, v0, v2, 0.0f);
        JoltC_SoftBodySharedSettings_AddEdgeConstraint(shared, v0, v3, 0.0f);
        JoltC_SoftBodySharedSettings_AddEdgeConstraint(shared, v1, v2, 0.0f);
        JoltC_SoftBodySharedSettings_AddEdgeConstraint(shared, v1, v3, 0.0f);
        uint32_t lastEdge = JoltC_SoftBodySharedSettings_AddEdgeConstraint(shared, v2, v3, 0.0f);
        TEST_ASSERT(lastEdge == 5, "edge indices are sequential");

        uint32_t volume = JoltC_SoftBodySharedSettings_AddVolumeConstraint(shared, v0, v1, v2, v3, 0.0f);
        TEST_ASSERT(volume == 0, "first volume constraint has index zero");

        JoltC_SoftBodySharedSettings_CalculateEdgeLengths(shared);
        JoltC_SoftBodySharedSettings_CalculateVolumeConstraintVolumes(shared);
        JoltC_SoftBodySharedSettings_Optimize(shared);

        TEST_ASSERT(JoltC_SoftBodySharedSettings_GetEdgeConstraintCount(shared) == 6, "six edges counted");
        TEST_ASSERT(JoltC_SoftBodySharedSettings_GetVolumeConstraintCount(shared) == 1, "one volume counted");

        JoltC_BodyID bodyId;
        JoltC_RVec3 spawn = { 0.0f, 2.0f, 0.0f };
        JoltC_Body* body = add_soft_body(&ctx, shared, spawn, &bodyId);
        TEST_ASSERT_NOT_NULL(body, "tetrahedron created");

        step(&ctx, 120);

        /* Dropped on the floor, the rigid edges keep every distance at rest length. */
        JoltC_SoftBodyMotionProperties* mp = JoltC_Body_GetSoftBodyMotionProperties(body);
        JoltC_Vec3 p0, p1;
        JoltC_SoftBodyMotionProperties_GetVertexPosition(mp, v0, &p0);
        JoltC_SoftBodyMotionProperties_GetVertexPosition(mp, v1, &p1);
        float dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
        float edge01 = (float)sqrt((dx * dx) + (dy * dy) + (dz * dz));
        TEST_ASSERT_FLOAT_EQ(edge01, 1.0f, 0.1f, "an edge is still a metre after the fall");

        JoltC_SoftBodySharedSettings_Release(shared);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_cosserat_rod_chain */
    TEST_BEGIN("A chain of Cosserat rods hangs without stretching");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        enum { LINKS = 5 };
        const float spacing = 0.5f;
        JoltC_SoftBodySharedSettings* shared = JoltC_SoftBodySharedSettings_Create();

        for (int i = 0; i < LINKS; i++)
            JoltC_SoftBodySharedSettings_AddVertex(shared, 0.0f, -i * spacing, 0.0f, i == 0 ? 0.0f : 1.0f);

        uint32_t rods[LINKS - 1];
        for (int i = 0; i < LINKS - 1; i++)
            rods[i] = JoltC_SoftBodySharedSettings_AddRodStretchShearConstraint(shared, (uint32_t)i, (uint32_t)(i + 1), 0.0f);

        /* Every rod constrained rigidly to its neighbour: the chain behaves as one stiff rod, so
         * the tip-to-pin distance below stays the full length even mid-swing. (Soft bend/twist
         * would let the chain curve, and the straight-line distance would drop.) */
        for (int i = 0; i < LINKS - 2; i++)
            JoltC_SoftBodySharedSettings_AddRodBendTwistConstraint(shared, rods[i], rods[i + 1], 0.0f);

        JoltC_SoftBodySharedSettings_CalculateRodProperties(shared);
        JoltC_SoftBodySharedSettings_Optimize(shared);

        TEST_ASSERT(JoltC_SoftBodySharedSettings_GetRodStretchShearConstraintCount(shared) == LINKS - 1,
                    "one rod per segment");
        TEST_ASSERT(JoltC_SoftBodySharedSettings_GetRodBendTwistConstraintCount(shared) == LINKS - 2,
                    "one bend/twist between each pair of rods");

        JoltC_BodyID bodyId;
        JoltC_RVec3 spawn = { 0.0f, 4.0f, 0.0f };
        JoltC_Body* body = add_soft_body(&ctx, shared, spawn, &bodyId);
        TEST_ASSERT_NOT_NULL(body, "rod chain created");

        /* A sideways shove makes it swing, which is a harder test of the fixed length than
         * hanging straight down. */
        JoltC_SoftBodyMotionProperties* mp = JoltC_Body_GetSoftBodyMotionProperties(body);
        JoltC_Vec3 shove = { 3.0f, 0.0f, 0.0f };
        JoltC_SoftBodyMotionProperties_SetVertexVelocity(mp, LINKS - 1, shove);

        step(&ctx, 180);

        JoltC_Vec3 top, tip;
        JoltC_SoftBodyMotionProperties_GetVertexPosition(mp, 0, &top);
        JoltC_SoftBodyMotionProperties_GetVertexPosition(mp, LINKS - 1, &tip);
        float dx = tip.x - top.x, dy = tip.y - top.y, dz = tip.z - top.z;
        float length = (float)sqrt((dx * dx) + (dy * dy) + (dz * dz));
        TEST_ASSERT_FLOAT_EQ(length, (LINKS - 1) * spacing, 0.15f, "the swinging chain keeps its length");

        JoltC_SoftBodySharedSettings_Release(shared);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_skinning_moves_vertices */
    TEST_BEGIN("Hard-skinned vertices follow the joint they are bound to");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_SoftBodySharedSettings* shared = JoltC_SoftBodySharedSettings_Create();
        JoltC_SoftBodySharedSettings_AddVertex(shared, 0.0f, 0.0f, 0.0f, 1.0f);
        JoltC_SoftBodySharedSettings_AddVertex(shared, 1.0f, 0.0f, 0.0f, 1.0f);
        JoltC_SoftBodySharedSettings_AddEdgeConstraint(shared, 0, 1, 0.0f);
        JoltC_SoftBodySharedSettings_CalculateEdgeLengths(shared);

        JoltC_Mat44 identity;
        JoltC_Mat44_Identity(&identity);
        uint32_t invBind = JoltC_SoftBodySharedSettings_AddInvBindMatrix(shared, 0, &identity);

        float weight = 1.0f;
        for (uint32_t v = 0; v < 2; v++)
            JoltC_SoftBodySharedSettings_AddSkinnedConstraint(shared, v, 0.0f, FLT_MAX, 40.0f, &invBind, &weight, 1);

        TEST_ASSERT(JoltC_SoftBodySharedSettings_GetSkinnedConstraintCount(shared) == 2, "two skinned constraints");

        JoltC_SoftBodySharedSettings_Optimize(shared);

        JoltC_BodyID bodyId;
        JoltC_RVec3 spawn = { 0.0f, 5.0f, 0.0f };
        JoltC_Body* body = add_soft_body(&ctx, shared, spawn, &bodyId);
        TEST_ASSERT_NOT_NULL(body, "skinned pair created");

        JoltC_SoftBodyMotionProperties* mp = JoltC_Body_GetSoftBodyMotionProperties(body);

        /* Skin with the joint moved three metres up: hard skinning teleports both vertices,
         * because their max distance is zero. Everything is expressed in the space of the body's
         * own centre of mass transform, so the local positions land at bind pose plus the joint
         * offset transformed into local space. */
        JoltC_Mat44 com;
        JoltC_Body_GetCenterOfMassTransform(body, &com);

        JoltC_Mat44 joint;
        JoltC_Mat44_Identity(&joint);
        joint.m[13] = 3.0f; /* translation y */

        JoltC_SoftBodyMotionProperties_SkinVertices(mp, &com, &joint, 1, 1, ctx.tempAllocator);

        JoltC_Vec3 p0;
        JoltC_SoftBodyMotionProperties_GetVertexPosition(mp, 0, &p0);

        /* The bind pose of vertex 0 in local space was (COM-relative) somewhere below; after
         * skinning it must sit exactly three metres higher than the bind pose it had. The bind
         * pose local position is bind (world 0,0,0 relative to spawn) minus the COM offset, so
         * comparing against the skinned expectation through the COM keeps this exact. */
        float worldY = com.m[13] + p0.y; /* rotation is identity here */
        TEST_ASSERT_FLOAT_EQ(worldY, spawn.y + 3.0f, 0.01f, "the vertex sits where the joint put it");

        TEST_ASSERT(JoltC_SoftBodyMotionProperties_GetEnableSkinConstraints(mp) == JOLTC_TRUE,
                    "skin constraints report enabled");
        JoltC_SoftBodyMotionProperties_SetSkinnedMaxDistanceMultiplier(mp, 2.0f);
        TEST_ASSERT_FLOAT_EQ(JoltC_SoftBodyMotionProperties_GetSkinnedMaxDistanceMultiplier(mp), 2.0f, 1.0e-6f,
                             "max distance multiplier round trips");

        JoltC_SoftBodySharedSettings_Release(shared);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_vertex_write_access */
    TEST_BEGIN("A vertex pinned at runtime stays while the rest falls");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_SoftBodySharedSettings* shared = JoltC_SoftBodySharedSettings_CreateCube(3, 0.4f);
        TEST_ASSERT_NOT_NULL(shared, "cube settings created");

        JoltC_BodyID bodyId;
        JoltC_RVec3 spawn = { 0.0f, 6.0f, 0.0f };
        JoltC_Body* body = add_soft_body(&ctx, shared, spawn, &bodyId);
        TEST_ASSERT_NOT_NULL(body, "cube created");

        JoltC_SoftBodyMotionProperties* mp = JoltC_Body_GetSoftBodyMotionProperties(body);

        /* Pin vertex zero where it is right now and stop it dead. */
        float oldInvMass = JoltC_SoftBodyMotionProperties_GetVertexInvMass(mp, 0);
        TEST_ASSERT(oldInvMass > 0.0f, "the cube factory made a dynamic vertex");
        JoltC_Vec3 zero = { 0.0f, 0.0f, 0.0f };
        JoltC_SoftBodyMotionProperties_SetVertexVelocity(mp, 0, zero);
        JoltC_SoftBodyMotionProperties_SetVertexInvMass(mp, 0, 0.0f);
        JoltC_SoftBodyMotionProperties_CalculateMassAndInertia(mp);

        JoltC_Mat44 comBefore;
        JoltC_Body_GetCenterOfMassTransform(body, &comBefore);
        JoltC_Vec3 pinnedBefore;
        JoltC_SoftBodyMotionProperties_GetVertexPosition(mp, 0, &pinnedBefore);
        float pinnedWorldYBefore = comBefore.m[13] + pinnedBefore.y;

        step(&ctx, 120);

        JoltC_Mat44 comAfter;
        JoltC_Body_GetCenterOfMassTransform(body, &comAfter);
        JoltC_Vec3 pinnedAfter, farAfter;
        JoltC_SoftBodyMotionProperties_GetVertexPosition(mp, 0, &pinnedAfter);
        JoltC_SoftBodyMotionProperties_GetVertexPosition(mp,
            JoltC_SoftBodyMotionProperties_GetVertexCount(mp) - 1, &farAfter);
        float pinnedWorldYAfter = comAfter.m[13] + pinnedAfter.y;
        float farWorldYAfter = comAfter.m[13] + farAfter.y;

        TEST_ASSERT_FLOAT_EQ(pinnedWorldYAfter, pinnedWorldYBefore, 0.05f,
                             "the pinned vertex has not moved two seconds later");
        TEST_ASSERT(farWorldYAfter < pinnedWorldYAfter - 0.3f,
                    "the far corner hangs below the pin, so the rest of the cube kept simulating");

        /* Teleporting a vertex is a plain write now. */
        JoltC_Vec3 teleport = { 9.0f, 9.0f, 9.0f };
        JoltC_SoftBodyMotionProperties_SetVertexPosition(mp, 1, teleport);
        JoltC_Vec3 readBack;
        JoltC_SoftBodyMotionProperties_GetVertexPosition(mp, 1, &readBack);
        TEST_ASSERT_FLOAT_EQ(readBack.x, 9.0f, 1.0e-6f, "a written position reads straight back");

        /* And velocities read in bulk. */
        JoltC_Vec3 velocities[27];
        uint32_t got = JoltC_SoftBodyMotionProperties_GetVertexVelocities(mp, velocities, 27);
        TEST_ASSERT(got == 27, "bulk velocity read covers every vertex");

        JoltC_SoftBodySharedSettings_Release(shared);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_custom_update */
    TEST_BEGIN("CustomUpdate steps a soft body that is outside the system");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_SoftBodySharedSettings* shared = JoltC_SoftBodySharedSettings_CreateCube(2, 0.4f);

        JoltC_SoftBodyCreationSettings* creation = JoltC_SoftBodyCreationSettings_Create();
        JoltC_SoftBodyCreationSettings_SetSettings(creation, shared);
        JoltC_SoftBodyCreationSettings_SetPosition(creation, (JoltC_RVec3){ 0.0f, 5.0f, 0.0f });
        JoltC_SoftBodyCreationSettings_SetObjectLayer(creation, OBJ_LAYER_DYNAMIC);
        JoltC_SoftBodyCreationSettings_SetUpdatePosition(creation, 1);

        /* Created but never added: the body lives outside the broad phase and only moves when
         * stepped by hand, which is exactly what CustomUpdate is for. */
        JoltC_Body* body = JoltC_BodyInterface_CreateSoftBody(ctx.bodyInterface, creation);
        JoltC_SoftBodyCreationSettings_Destroy(creation);
        TEST_ASSERT_NOT_NULL(body, "detached soft body created");

        JoltC_SoftBodyMotionProperties* mp = JoltC_Body_GetSoftBodyMotionProperties(body);
        TEST_ASSERT_NOT_NULL(mp, "motion properties reachable");

        JoltC_Mat44 before;
        JoltC_Body_GetCenterOfMassTransform(body, &before);

        for (int i = 0; i < 60; i++)
            JoltC_SoftBodyMotionProperties_CustomUpdate(mp, 1.0f / 60.0f, body, ctx.physicsSystem);

        JoltC_Mat44 after;
        JoltC_Body_GetCenterOfMassTransform(body, &after);

        /* A second of gravity takes it about five metres down. */
        TEST_ASSERT(after.m[13] < before.m[13] - 3.0f, "the hand-stepped body fell under gravity");

        JoltC_BodyID bodyId = JoltC_Body_GetID(body);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, bodyId);
        JoltC_SoftBodySharedSettings_Release(shared);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_creation_settings_round_trip */
    TEST_BEGIN("Creation settings read back what was written");
    {
        JoltC_SoftBodyCreationSettings* creation = JoltC_SoftBodyCreationSettings_Create();

        JoltC_SoftBodyCreationSettings_SetPosition(creation, (JoltC_RVec3){ 1.0f, 2.0f, 3.0f });
        JoltC_SoftBodyCreationSettings_SetNumIterations(creation, 12);
        JoltC_SoftBodyCreationSettings_SetPressure(creation, 850.0f);
        JoltC_SoftBodyCreationSettings_SetFriction(creation, 0.4f);
        JoltC_SoftBodyCreationSettings_SetVertexRadius(creation, 0.03f);
        JoltC_SoftBodyCreationSettings_SetFacesDoubleSided(creation, 1);
        JoltC_SoftBodyCreationSettings_SetUserData(creation, 777);

        JoltC_RVec3 position = JoltC_SoftBodyCreationSettings_GetPosition(creation);
        TEST_ASSERT_FLOAT_EQ(position.y, 2.0f, 1.0e-6f, "position round trips");
        TEST_ASSERT(JoltC_SoftBodyCreationSettings_GetNumIterations(creation) == 12, "iterations round trip");
        TEST_ASSERT_FLOAT_EQ(JoltC_SoftBodyCreationSettings_GetPressure(creation), 850.0f, 1.0e-3f, "pressure round trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_SoftBodyCreationSettings_GetFriction(creation), 0.4f, 1.0e-6f, "friction round trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_SoftBodyCreationSettings_GetVertexRadius(creation), 0.03f, 1.0e-6f, "vertex radius round trips");
        TEST_ASSERT(JoltC_SoftBodyCreationSettings_GetFacesDoubleSided(creation) == JOLTC_TRUE, "double sided round trips");
        TEST_ASSERT(JoltC_SoftBodyCreationSettings_GetUserData(creation) == 777, "user data round trips");

        JoltC_SoftBodyCreationSettings_Destroy(creation);
    }
    TEST_END();

    /* test_runtime_iterations_and_double_sided */
    TEST_BEGIN("Runtime iteration count and face sidedness are visible");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_SoftBodySharedSettings* shared = JoltC_SoftBodySharedSettings_CreateCube(2, 0.4f);
        JoltC_BodyID bodyId;
        JoltC_RVec3 spawn = { 0.0f, 3.0f, 0.0f };
        JoltC_Body* body = add_soft_body(&ctx, shared, spawn, &bodyId);

        JoltC_SoftBodyMotionProperties* mp = JoltC_Body_GetSoftBodyMotionProperties(body);
        TEST_ASSERT(JoltC_SoftBodyMotionProperties_GetNumIterations(mp) == 10, "creation iterations arrived");
        JoltC_SoftBodyMotionProperties_SetNumIterations(mp, 3);
        TEST_ASSERT(JoltC_SoftBodyMotionProperties_GetNumIterations(mp) == 3, "runtime iterations round trip");
        TEST_ASSERT(JoltC_SoftBodyMotionProperties_GetFacesDoubleSided(mp) == JOLTC_FALSE, "single sided by default");

        JoltC_SoftBodySharedSettings_Release(shared);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}
