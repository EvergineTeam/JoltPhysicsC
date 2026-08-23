/* JoltC Test Suite -- phase 5: character and system to one hundred percent.
 * SPDX-License-Identifier: MIT
 *
 * The complete character contact listener with its full payload and the character-versus-character
 * family, the supporting volume, the rest of MotionProperties, closest-hit shape casts, internal
 * edge removal, broad phase box queries, the system census, process-wide combine functions, the
 * malloc allocator and single threaded job system, and collision response estimation.
 */

#include "test_common.h"

#include <math.h>
#include <string.h>

/* --- Listener v2 telemetry ------------------------------------------------- */
typedef struct ListenerTelemetry {
    int bodyContactsAdded;
    int characterContactsAdded;
    int contactSolves;
    uint32_t lastCharacterIDB;
    JoltC_BodyID lastBodyB;
    int materialWasNull;
} ListenerTelemetry;

static void on_contact_added_v2(void* userData, const JoltC_CharacterContact* contact, JoltC_Bool* ioCanPushCharacter, JoltC_Bool* ioCanReceiveImpulses)
{
    (void)ioCanPushCharacter; (void)ioCanReceiveImpulses;
    ListenerTelemetry* telemetry = (ListenerTelemetry*)userData;
    telemetry->bodyContactsAdded++;
    telemetry->lastBodyB = contact->bodyB;
    if (contact->materialB == NULL)
        telemetry->materialWasNull = 1;
}

static void on_character_contact_added_v2(void* userData, const JoltC_CharacterContact* contact, JoltC_Bool* ioCanPushCharacter, JoltC_Bool* ioCanReceiveImpulses)
{
    (void)ioCanPushCharacter; (void)ioCanReceiveImpulses;
    ListenerTelemetry* telemetry = (ListenerTelemetry*)userData;
    telemetry->characterContactsAdded++;
    telemetry->lastCharacterIDB = contact->characterIDB;
}

static void on_contact_solve_v2(void* userData, JoltC_BodyID bodyID2, uint32_t otherCharacterID, JoltC_SubShapeID subShapeID2,
                                JoltC_RVec3 contactPosition, JoltC_Vec3 contactNormal, JoltC_Vec3 contactVelocity,
                                const JoltC_PhysicsMaterial* contactMaterial, JoltC_Vec3 characterVelocity, JoltC_Vec3* ioNewCharacterVelocity)
{
    (void)bodyID2; (void)otherCharacterID; (void)subShapeID2; (void)contactPosition; (void)contactNormal;
    (void)contactVelocity; (void)contactMaterial; (void)characterVelocity; (void)ioNewCharacterVelocity;
    ((ListenerTelemetry*)userData)->contactSolves++;
}

/* --- Combine function ------------------------------------------------------ */
static float combine_full_restitution(const JoltC_Body* body1, uint32_t subShapeID1, const JoltC_Body* body2, uint32_t subShapeID2)
{
    (void)body1; (void)subShapeID1; (void)body2; (void)subShapeID2;
    return 1.0f;
}

/* --- Collectors ------------------------------------------------------------ */
static int s_closeHits;
static void count_collide_hit(void* userData, const JoltC_CollideShapeResult* result)
{
    (void)result;
    (*(int*)userData)++;
}

static int s_broadHits;
static void count_broad_cast(void* userData, const JoltC_BroadPhaseCastResult* result)
{
    (void)result;
    (*(int*)userData)++;
}

static void count_broad_body(void* userData, JoltC_BodyID bodyID)
{
    (void)bodyID;
    (*(int*)userData)++;
}

/* --- Estimation from inside the contact callback --------------------------- */
typedef struct EstimationCapture {
    int ran;
    uint32_t impulseCount;
    float firstImpulse;
} EstimationCapture;

static void on_contact_added_estimate(void* userData, const JoltC_Body* body1, const JoltC_Body* body2,
                                      const JoltC_ContactManifold* manifold, JoltC_ContactSettings* settings)
{
    (void)settings;
    EstimationCapture* capture = (EstimationCapture*)userData;
    if (capture->ran) return;

    JoltC_CollisionEstimationResult result;
    float impulses[8];
    uint32_t count = 0;
    JoltC_EstimateCollisionResponse(body1, body2, manifold, 0.2f, 0.0f, 1.0f, 10, &result, impulses, 8, &count);
    capture->ran = 1;
    capture->impulseCount = count;
    capture->firstImpulse = count > 0 ? impulses[0] : 0.0f;
}

void run_phase5_tests(void);

void run_phase5_tests(void)
{
    /* test_full_character_listener */
    TEST_BEGIN("The complete listener hears bodies, characters and solves");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_Vec3 floorHalf = { 20.0f, 1.0f, 20.0f };
        const JoltC_Shape* floorShape = JoltC_BoxShape_Create(floorHalf, 0.05f);
        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_RVec3 floorPos = { 0.0f, -1.0f, 0.0f };
        JoltC_BodyCreationSettings* floorSettings = JoltC_BodyCreationSettings_Create3(
            floorShape, floorPos, identity, JOLTC_MOTION_TYPE_STATIC, OBJ_LAYER_STATIC);
        JoltC_BodyInterface_CreateAndAddBody(ctx.bodyInterface, floorSettings, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_BodyCreationSettings_Destroy(floorSettings);
        JoltC_Shape_Release(floorShape);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.3f);
        JoltC_CharacterVirtualSettings settings;
        JoltC_CharacterVirtualSettings_Init(&settings);
        settings.shape = capsule;

        JoltC_RVec3 spawnA = { -1.5f, 1.5f, 0.0f };
        JoltC_RVec3 spawnB = { 1.5f, 1.5f, 0.0f };
        JoltC_CharacterVirtual* a = JoltC_CharacterVirtual_Create(&settings, spawnA, identity, 0, ctx.physicsSystem);
        JoltC_CharacterVirtual* b = JoltC_CharacterVirtual_Create(&settings, spawnB, identity, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(a, "character A created");
        TEST_ASSERT_NOT_NULL(b, "character B created");

        /* Characters only see each other through a character-vs-character interface. */
        JoltC_CharacterVsCharacterCollision* cvc = JoltC_CharacterVsCharacterCollision_CreateSimple();
        JoltC_CharacterVsCharacterCollisionSimple_AddCharacter(cvc, a);
        JoltC_CharacterVsCharacterCollisionSimple_AddCharacter(cvc, b);
        JoltC_CharacterVirtual_SetCharacterVsCharacterCollision(a, cvc);
        JoltC_CharacterVirtual_SetCharacterVsCharacterCollision(b, cvc);

        ListenerTelemetry telemetry;
        memset(&telemetry, 0, sizeof(telemetry));
        JoltC_CharacterContactListener_ProcsV2 procs;
        memset(&procs, 0, sizeof(procs));
        procs.onContactAdded = on_contact_added_v2;
        procs.onCharacterContactAdded = on_character_contact_added_v2;
        procs.onContactSolve = on_contact_solve_v2;

        JoltC_CharacterContactListener* listener = JoltC_CharacterContactListener_Create2(&procs, &telemetry);
        TEST_ASSERT_NOT_NULL(listener, "v2 listener created");
        JoltC_CharacterVirtual_SetListener(a, listener);

        /* March them into each other. */
        JoltC_Vec3 gravity = { 0.0f, -9.81f, 0.0f };
        for (int i = 0; i < 120; i++)
        {
            JoltC_Vec3 towardB = { 1.5f, -5.0f, 0.0f };
            JoltC_Vec3 towardA = { -1.5f, -5.0f, 0.0f };
            JoltC_CharacterVirtual_SetLinearVelocity(a, towardB);
            JoltC_CharacterVirtual_Update(a, 1.0f / 60.0f, gravity, ctx.tempAllocator);
            JoltC_CharacterVirtual_SetLinearVelocity(b, towardA);
            JoltC_CharacterVirtual_Update(b, 1.0f / 60.0f, gravity, ctx.tempAllocator);
            JoltC_PhysicsSystem_Update(ctx.physicsSystem, 1.0f / 60.0f, 1, ctx.tempAllocator, ctx.jobSystem);
        }

        TEST_ASSERT(telemetry.bodyContactsAdded > 0, "the floor contact reached the full-payload callback");
        TEST_ASSERT(telemetry.contactSolves > 0, "the solve hook fired, which is where gameplay rewrites velocity");
        TEST_ASSERT(telemetry.materialWasNull == 0, "every body contact carried a material, if only the default");
        TEST_ASSERT(telemetry.characterContactsAdded > 0, "the two characters met and the callback heard it");
        TEST_ASSERT(telemetry.lastCharacterIDB == JoltC_CharacterVirtual_GetID(b),
                    "the contact names the other character by its id");

        JoltC_CharacterVirtual_SetListener(a, NULL);
        JoltC_CharacterContactListener_Destroy(listener);
        JoltC_CharacterVirtual_Destroy(a);
        JoltC_CharacterVirtual_Destroy(b);
        JoltC_CharacterVsCharacterCollision_Destroy(cvc);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_supporting_volume */
    TEST_BEGIN("The supporting volume travels through settings and runtime");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.3f);
        JoltC_CharacterVirtualSettings settings;
        JoltC_CharacterVirtualSettings_Init(&settings);
        settings.shape = capsule;
        settings.supportingVolumeNormal = (JoltC_Vec3){ 0.0f, 1.0f, 0.0f };
        settings.supportingVolumeConstant = -0.6f;

        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_RVec3 spawn = { 0.0f, 2.0f, 0.0f };
        JoltC_CharacterVirtual* character = JoltC_CharacterVirtual_Create(&settings, spawn, identity, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(character, "character created");

        JoltC_Vec3 normal;
        float constant = 0.0f;
        JoltC_CharacterVirtual_GetSupportingVolume(character, &normal, &constant);
        TEST_ASSERT_FLOAT_EQ(constant, -0.6f, 1.0e-6f, "the settings plane arrived");

        JoltC_CharacterVirtual_SetSupportingVolume(character, (JoltC_Vec3){ 0.0f, 1.0f, 0.0f }, -0.2f);
        JoltC_CharacterVirtual_GetSupportingVolume(character, &normal, &constant);
        TEST_ASSERT_FLOAT_EQ(constant, -0.2f, 1.0e-6f, "the runtime plane round trips");

        JoltC_CharacterVirtual_Destroy(character);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_motion_properties_completion */
    TEST_BEGIN("MotionProperties answers everything now");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_RVec3 spawn = { 0.0f, 5.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, spawn, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        const JoltC_BodyLockInterface* lockInterface = JoltC_PhysicsSystem_GetBodyLockInterfaceNoLock(ctx.physicsSystem);
        JoltC_BodyLockWrite* lock = JoltC_BodyLockWrite_Create(lockInterface, id);
        JoltC_Body* body = JoltC_BodyLockWrite_GetBody(lock);
        JoltC_BodyLockWrite_Destroy(lock);
        JoltC_MotionProperties* motion = JoltC_Body_GetMotionProperties(body);
        TEST_ASSERT_NOT_NULL(motion, "motion properties reachable");

        JoltC_MotionProperties_SetGravityFactor(motion, 0.5f);
        TEST_ASSERT_FLOAT_EQ(JoltC_MotionProperties_GetGravityFactor(motion), 0.5f, 1.0e-6f, "gravity factor round trips");

        JoltC_MotionProperties_SetMaxLinearVelocity(motion, 123.0f);
        TEST_ASSERT_FLOAT_EQ(JoltC_MotionProperties_GetMaxLinearVelocity(motion), 123.0f, 1.0e-4f, "max linear velocity round trips");

        JoltC_MotionProperties_SetNumVelocityStepsOverride(motion, 20);
        TEST_ASSERT(JoltC_MotionProperties_GetNumVelocityStepsOverride(motion) == 20, "velocity steps override round trips");

        /* SetLinearVelocityClamped against the 123 cap above. */
        JoltC_MotionProperties_SetLinearVelocityClamped(motion, (JoltC_Vec3){ 500.0f, 0.0f, 0.0f });
        JoltC_Vec3 clamped = JoltC_MotionProperties_GetLinearVelocity(motion);
        TEST_ASSERT_FLOAT_EQ(clamped.x, 123.0f, 0.5f, "the clamped setter obeyed the cap");

        /* Spin it and read the velocity of an offset point: w cross r. */
        JoltC_MotionProperties_SetLinearVelocity(motion, (JoltC_Vec3){ 0.0f, 0.0f, 0.0f });
        JoltC_MotionProperties_SetAngularVelocity(motion, (JoltC_Vec3){ 0.0f, 2.0f, 0.0f });
        JoltC_Vec3 pointVelocity = JoltC_MotionProperties_GetPointVelocityCOM(motion, (JoltC_Vec3){ 1.0f, 0.0f, 0.0f });
        TEST_ASSERT_FLOAT_EQ(pointVelocity.z, -2.0f, 1.0e-4f, "point velocity is omega cross r");

        TEST_ASSERT(JoltC_MotionProperties_GetAllowSleeping(motion) == JOLTC_TRUE, "boxes may sleep by default");
        TEST_ASSERT(JoltC_MotionProperties_GetInverseMass(motion) > 0.0f, "a dynamic box has finite mass");

        /* A force accumulates until the step consumes it. */
        JoltC_BodyInterface_AddForce(ctx.bodyInterface, id, (JoltC_Vec3){ 100.0f, 0.0f, 0.0f });
        JoltC_Vec3 accumulated = JoltC_MotionProperties_GetAccumulatedForce(motion);
        TEST_ASSERT(accumulated.x > 99.0f, "the added force shows in the accumulator");
        JoltC_MotionProperties_ResetForce(motion);
        accumulated = JoltC_MotionProperties_GetAccumulatedForce(motion);
        TEST_ASSERT_FLOAT_EQ(accumulated.x, 0.0f, 1.0e-6f, "reset clears it");

        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_cast_shape_closest */
    TEST_BEGIN("The closest-hit cast returns the first thing in the way");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_RVec3 nearPos = { 0.0f, 0.0f, 3.0f };
        JoltC_RVec3 farPos = { 0.0f, 0.0f, 7.0f };
        JoltC_BodyID nearId = create_test_box_body(&ctx, nearPos, JOLTC_MOTION_TYPE_STATIC, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_BodyID farId = create_test_box_body(&ctx, farPos, JOLTC_MOTION_TYPE_STATIC, JOLTC_ACTIVATION_DONT_ACTIVATE);
        (void)farId;

        const JoltC_NarrowPhaseQuery* query = JoltC_PhysicsSystem_GetNarrowPhaseQuery(ctx.physicsSystem);
        const JoltC_Shape* probe = JoltC_SphereShape_Create(0.3f);

        JoltC_Mat44 transform;
        JoltC_Mat44_Identity(&transform);

        JoltC_ShapeCastResult hit;
        JoltC_Bool found = JoltC_NarrowPhaseQuery_CastShapeClosest(
            query, probe, (JoltC_Vec3){ 1.0f, 1.0f, 1.0f }, transform,
            (JoltC_Vec3){ 0.0f, 0.0f, 12.0f }, NULL, (JoltC_RVec3){ 0.0f, 0.0f, 0.0f },
            &hit, NULL, NULL, NULL, NULL);

        TEST_ASSERT(found == JOLTC_TRUE, "the sweep hit something");
        TEST_ASSERT(hit.bodyID2 == nearId, "and it was the near box, not the far one");
        TEST_ASSERT(hit.fraction < 0.35f, "at the fraction the near box sits at");

        JoltC_Shape_Release(probe);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_internal_edge_removal */
    TEST_BEGIN("Internal edge removal trims the seam contacts of a mesh");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        /* A flat quad of two triangles; the probe sphere sits right on the shared diagonal. */
        JoltC_Vec3 vertices[4] = {
            { -2.0f, 0.0f, -2.0f }, { 2.0f, 0.0f, -2.0f }, { 2.0f, 0.0f, 2.0f }, { -2.0f, 0.0f, 2.0f },
        };
        JoltC_IndexedTriangle triangles[2] = {
            { 0, 3, 2, 0, 0 },
            { 0, 2, 1, 0, 0 },
        };
        const JoltC_Shape* mesh = JoltC_MeshShape_Create(vertices, 4, triangles, 2);
        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_RVec3 origin = { 0.0f, 0.0f, 0.0f };
        JoltC_BodyCreationSettings* meshSettings = JoltC_BodyCreationSettings_Create3(
            mesh, origin, identity, JOLTC_MOTION_TYPE_STATIC, OBJ_LAYER_STATIC);
        JoltC_BodyInterface_CreateAndAddBody(ctx.bodyInterface, meshSettings, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_BodyCreationSettings_Destroy(meshSettings);

        const JoltC_NarrowPhaseQuery* query = JoltC_PhysicsSystem_GetNarrowPhaseQuery(ctx.physicsSystem);
        const JoltC_Shape* probe = JoltC_SphereShape_Create(0.4f);

        JoltC_Mat44 transform;
        JoltC_Mat44_Identity(&transform);
        transform.m[13] = 0.2f; /* resting on the seam */

        int plainHits = 0;
        JoltC_NarrowPhaseQuery_CollideShape2(query, probe, (JoltC_Vec3){ 1, 1, 1 }, transform, NULL,
                                             (JoltC_RVec3){ 0, 0, 0 }, count_collide_hit, &plainHits,
                                             NULL, NULL, NULL, NULL);

        int trimmedHits = 0;
        JoltC_NarrowPhaseQuery_CollideShapeWithInternalEdgeRemoval(query, probe, (JoltC_Vec3){ 1, 1, 1 }, transform, NULL,
                                                                   (JoltC_RVec3){ 0, 0, 0 }, count_collide_hit, &trimmedHits,
                                                                   NULL, NULL, NULL, NULL);

        TEST_ASSERT(plainHits >= 1, "the plain query sees the mesh");
        TEST_ASSERT(trimmedHits >= 1, "the trimmed query still sees the mesh");
        TEST_ASSERT(trimmedHits <= plainHits, "removal never invents contacts");

        JoltC_Shape_Release(probe);
        JoltC_Shape_Release(mesh);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_broad_phase_boxes */
    TEST_BEGIN("Broad phase box sweeps and oriented boxes find bodies");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_RVec3 pos = { 0.0f, 0.0f, 5.0f };
        create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_STATIC, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_PhysicsSystem_OptimizeBroadPhase(ctx.physicsSystem);

        const JoltC_BroadPhaseQuery* broad = JoltC_PhysicsSystem_GetBroadPhaseQuery(ctx.physicsSystem);

        s_broadHits = 0;
        JoltC_AABox sweep = { { -0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, 0.5f } };
        JoltC_BroadPhaseQuery_CastAABox(broad, sweep, (JoltC_Vec3){ 0.0f, 0.0f, 10.0f },
                                        count_broad_cast, &s_broadHits, NULL, NULL);
        TEST_ASSERT(s_broadHits >= 1, "the swept box found the body in its path");

        s_closeHits = 0;
        JoltC_OrientedBox oriented;
        JoltC_Mat44_Identity(&oriented.orientation);
        oriented.orientation.m[14] = 5.0f; /* centred on the body */
        oriented.halfExtents = (JoltC_Vec3){ 1.0f, 1.0f, 1.0f };
        JoltC_BroadPhaseQuery_CollideOrientedBox(broad, &oriented, count_broad_body, &s_closeHits, NULL, NULL);
        TEST_ASSERT(s_closeHits >= 1, "the oriented box found the body it covers");

        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_system_census */
    TEST_BEGIN("The system reports its census, bounds and active bodies");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_RVec3 staticPos = { 0.0f, -1.0f, 0.0f };
        create_test_box_body(&ctx, staticPos, JOLTC_MOTION_TYPE_STATIC, JOLTC_ACTIVATION_DONT_ACTIVATE);
        for (int i = 0; i < 3; i++)
        {
            JoltC_RVec3 dynamicPos = { i * 2.0f, 4.0f, 0.0f };
            create_test_box_body(&ctx, dynamicPos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);
        }

        JoltC_BodyStats stats;
        JoltC_PhysicsSystem_GetBodyStats(ctx.physicsSystem, &stats);
        TEST_ASSERT(stats.numBodies == 4, "four bodies counted");
        TEST_ASSERT(stats.numBodiesStatic == 1, "one static");
        TEST_ASSERT(stats.numBodiesDynamic == 3, "three dynamic");
        TEST_ASSERT(stats.numActiveBodiesDynamic == 3, "all three awake");

        JoltC_BodyID active[8];
        uint32_t activeCount = JoltC_PhysicsSystem_GetActiveBodies(ctx.physicsSystem, 0, active, 8);
        TEST_ASSERT(activeCount == 3, "the active list matches the census");

        JoltC_AABox bounds;
        JoltC_PhysicsSystem_GetBounds(ctx.physicsSystem, &bounds);
        TEST_ASSERT(bounds.min.y < 0.0f && bounds.max.y > 4.0f, "the bounds hold everything");

        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_combine_functions */
    TEST_BEGIN("A combine function turns a dead ball bouncy");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_Vec3 floorHalf = { 20.0f, 1.0f, 20.0f };
        const JoltC_Shape* floorShape = JoltC_BoxShape_Create(floorHalf, 0.05f);
        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_RVec3 floorPos = { 0.0f, -1.0f, 0.0f };
        JoltC_BodyCreationSettings* floorSettings = JoltC_BodyCreationSettings_Create3(
            floorShape, floorPos, identity, JOLTC_MOTION_TYPE_STATIC, OBJ_LAYER_STATIC);
        JoltC_BodyInterface_CreateAndAddBody(ctx.bodyInterface, floorSettings, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_BodyCreationSettings_Destroy(floorSettings);
        JoltC_Shape_Release(floorShape);

        /* Both restitutions default to zero, so the default combine (max) gives a dead drop. The
         * override says every impact is perfectly elastic. */
        JoltC_PhysicsSystem_SetCombineRestitution(ctx.physicsSystem, combine_full_restitution);

        JoltC_RVec3 spawn = { 0.0f, 3.0f, 0.0f };
        JoltC_BodyID ball = create_test_box_body(&ctx, spawn, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        float highestAfterImpact = -1000.0f;
        int impacted = 0;
        for (int i = 0; i < 240; i++)
        {
            JoltC_PhysicsSystem_Update(ctx.physicsSystem, 1.0f / 60.0f, 1, ctx.tempAllocator, ctx.jobSystem);
            JoltC_Vec3 velocity = JoltC_BodyInterface_GetLinearVelocity(ctx.bodyInterface, ball);
            if (velocity.y > 0.5f) impacted = 1;
            if (impacted)
            {
                JoltC_RVec3 position = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, ball);
                if (position.y > highestAfterImpact) highestAfterImpact = (float)position.y;
            }
        }

        TEST_ASSERT(impacted, "the override made the impact elastic: the box came back up");
        TEST_ASSERT(highestAfterImpact > 1.5f, "and it bounced most of the way home");

        /* Null restores Jolt's default, so the next scene is not haunted by this one. */
        JoltC_PhysicsSystem_SetCombineRestitution(ctx.physicsSystem, NULL);

        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_alternative_allocator_and_jobs */
    TEST_BEGIN("The malloc allocator and single threaded jobs run a step");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_RVec3 spawn = { 0.0f, 3.0f, 0.0f };
        create_test_box_body(&ctx, spawn, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        JoltC_TempAllocator* mallocAllocator = JoltC_TempAllocatorMalloc_Create();
        JoltC_JobSystem* singleThreaded = JoltC_JobSystemSingleThreaded_Create(1024);
        TEST_ASSERT_NOT_NULL(mallocAllocator, "malloc allocator created");
        TEST_ASSERT_NOT_NULL(singleThreaded, "single threaded job system created");

        uint32_t error = JoltC_PhysicsSystem_Update(ctx.physicsSystem, 1.0f / 60.0f, 1, mallocAllocator, singleThreaded);
        TEST_ASSERT(error == JOLTC_PHYSICS_UPDATE_ERROR_NONE, "a step through both runs clean");

        JoltC_JobSystem_Destroy(singleThreaded);
        JoltC_TempAllocator_Destroy(mallocAllocator);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_estimate_collision_response */
    TEST_BEGIN("A contact's impulses can be estimated as it is reported");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_Vec3 floorHalf = { 20.0f, 1.0f, 20.0f };
        const JoltC_Shape* floorShape = JoltC_BoxShape_Create(floorHalf, 0.05f);
        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_RVec3 floorPos = { 0.0f, -1.0f, 0.0f };
        JoltC_BodyCreationSettings* floorSettings = JoltC_BodyCreationSettings_Create3(
            floorShape, floorPos, identity, JOLTC_MOTION_TYPE_STATIC, OBJ_LAYER_STATIC);
        JoltC_BodyInterface_CreateAndAddBody(ctx.bodyInterface, floorSettings, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_BodyCreationSettings_Destroy(floorSettings);
        JoltC_Shape_Release(floorShape);

        EstimationCapture capture;
        memset(&capture, 0, sizeof(capture));
        JoltC_ContactListener* listener = JoltC_ContactListener_CreateEnhanced(
            NULL, on_contact_added_estimate, NULL, NULL, &capture);
        JoltC_PhysicsSystem_SetContactListener(ctx.physicsSystem, listener);

        JoltC_RVec3 spawn = { 0.0f, 2.0f, 0.0f };
        create_test_box_body(&ctx, spawn, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        for (int i = 0; i < 90 && !capture.ran; i++)
            JoltC_PhysicsSystem_Update(ctx.physicsSystem, 1.0f / 60.0f, 1, ctx.tempAllocator, ctx.jobSystem);

        TEST_ASSERT(capture.ran, "a contact arrived and was estimated in place");
        TEST_ASSERT(capture.impulseCount > 0, "the manifold produced contact impulses");
        TEST_ASSERT(capture.firstImpulse > 0.0f, "a falling box pushes on the floor");

        JoltC_PhysicsSystem_SetContactListener(ctx.physicsSystem, NULL);
        JoltC_ContactListener_Destroy(listener);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}
