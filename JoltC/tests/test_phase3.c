/* JoltC Test Suite -- phase 3: determinism and state.
 * SPDX-License-Identifier: MIT
 *
 * The contract under test is rollback: save, keep simulating, restore, re-simulate the same steps
 * and land on bit-identical state -- same binary, same machine. Every comparison here is exact
 * float equality on purpose: "close" is precisely what a replay or lockstep network game cannot
 * live with, and Jolt promises better.
 */

#include "test_common.h"

#include <string.h>
#include <stdlib.h>

static void step_n(TestPhysicsContext* ctx, int steps)
{
    for (int i = 0; i < steps; i++)
        JoltC_PhysicsSystem_Update(ctx->physicsSystem, 1.0f / 60.0f, 1, ctx->tempAllocator, ctx->jobSystem);
}

static void add_static_floor(TestPhysicsContext* ctx)
{
    JoltC_Vec3 floorHalf = { 20.0f, 1.0f, 20.0f };
    const JoltC_Shape* floorShape = JoltC_BoxShape_Create(floorHalf, 0.05f);
    JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
    JoltC_RVec3 floorPos = { 0.0f, -1.0f, 0.0f };
    JoltC_BodyCreationSettings* settings = JoltC_BodyCreationSettings_Create3(
        floorShape, floorPos, identity, JOLTC_MOTION_TYPE_STATIC, OBJ_LAYER_STATIC);
    JoltC_BodyInterface_CreateAndAddBody(ctx->bodyInterface, settings, JOLTC_ACTIVATION_DONT_ACTIVATE);
    JoltC_BodyCreationSettings_Destroy(settings);
    JoltC_Shape_Release(floorShape);
}

void run_phase3_tests(void);

void run_phase3_tests(void)
{
    /* test_rollback_is_bit_exact */
    TEST_BEGIN("A restored world re-simulates to bit-identical state");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        add_static_floor(&ctx);

        /* A pile worth diverging: twelve tumbling boxes and a hinged pair. */
        enum { BOXES = 12 };
        JoltC_BodyID boxes[BOXES];
        for (int i = 0; i < BOXES; i++)
        {
            JoltC_RVec3 position = { (i % 3) * 0.9f, 3.0f + ((i / 3) * 1.1f), ((i % 4) - 1.5f) * 0.8f };
            boxes[i] = create_test_box_body(&ctx, position, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);
        }

        JoltC_RVec3 posA = { 5.0f, 5.0f, 0.0f };
        JoltC_RVec3 posB = { 5.0f, 4.0f, 0.0f };
        JoltC_BodyID a = create_test_box_body(&ctx, posA, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyID b = create_test_box_body(&ctx, posB, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        JoltC_HingeConstraintSettings hinge;
        JoltC_HingeConstraintSettings_Init(&hinge);
        hinge.point1 = (JoltC_RVec3){ 5.0f, 4.5f, 0.0f };
        hinge.point2 = (JoltC_RVec3){ 5.0f, 4.5f, 0.0f };
        JoltC_Constraint* constraint = JoltC_HingeConstraint_Create(ctx.physicsSystem, a, b, &hinge);
        TEST_ASSERT_NOT_NULL(constraint, "hinge created");

        /* Registered, so the rollback below really covers a solving constraint: an unregistered
         * hinge would make this pair a determinism test of two free boxes and nothing more. */
        JoltC_PhysicsSystem_AddConstraint(ctx.physicsSystem, constraint);

        step_n(&ctx, 30);

        JoltC_StateRecorder* recorder = JoltC_StateRecorderImpl_Create();
        TEST_ASSERT_NOT_NULL(recorder, "recorder created");
        JoltC_PhysicsSystem_SaveState(ctx.physicsSystem, recorder, JOLTC_STATE_RECORDER_STATE_ALL);
        TEST_ASSERT(JoltC_StateRecorderImpl_GetDataSize(recorder) > 0, "the snapshot holds data");

        /* First timeline. */
        step_n(&ctx, 60);
        JoltC_RVec3 first[BOXES + 2];
        for (int i = 0; i < BOXES; i++)
            first[i] = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, boxes[i]);
        first[BOXES] = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, a);
        first[BOXES + 1] = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, b);

        /* Back to the snapshot, second timeline, same steps. */
        JoltC_StateRecorderImpl_Rewind(recorder);
        TEST_ASSERT(JoltC_PhysicsSystem_RestoreState(ctx.physicsSystem, recorder), "restore succeeds");
        step_n(&ctx, 60);

        int identical = 1;
        for (int i = 0; i < BOXES; i++)
        {
            JoltC_RVec3 again = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, boxes[i]);
            if (again.x != first[i].x || again.y != first[i].y || again.z != first[i].z)
                identical = 0;
        }
        JoltC_RVec3 againA = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, a);
        JoltC_RVec3 againB = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, b);
        if (againA.x != first[BOXES].x || againA.y != first[BOXES].y || againA.z != first[BOXES].z) identical = 0;
        if (againB.x != first[BOXES + 1].x || againB.y != first[BOXES + 1].y || againB.z != first[BOXES + 1].z) identical = 0;

        TEST_ASSERT(identical, "every body, hinged pair included, landed on bit-identical positions");

        JoltC_StateRecorderImpl_Destroy(recorder);
        JoltC_PhysicsSystem_RemoveConstraint(ctx.physicsSystem, constraint);
        JoltC_Constraint_Destroy(constraint);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_snapshot_survives_a_byte_copy */
    TEST_BEGIN("A snapshot shipped as bytes restores in a fresh recorder");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        add_static_floor(&ctx);

        JoltC_BodyID box[3];
        for (int i = 0; i < 3; i++)
        {
            JoltC_RVec3 position = { i * 1.2f, 2.5f + i, 0.0f };
            box[i] = create_test_box_body(&ctx, position, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);
        }

        step_n(&ctx, 20);

        JoltC_StateRecorder* original = JoltC_StateRecorderImpl_Create();
        JoltC_PhysicsSystem_SaveState(ctx.physicsSystem, original, JOLTC_STATE_RECORDER_STATE_ALL);

        /* Through bytes, the way a network or a save file would carry it. */
        uint64_t size = JoltC_StateRecorderImpl_GetDataSize(original);
        TEST_ASSERT(size > 0, "snapshot has a size");
        uint8_t* bytes = (uint8_t*)malloc((size_t)size);
        uint64_t copied = JoltC_StateRecorderImpl_CopyData(original, bytes, size);
        TEST_ASSERT(copied == size, "CopyData reports the same size it copied");

        JoltC_StateRecorder* shipped = JoltC_StateRecorderImpl_Create();
        JoltC_StateRecorderImpl_SetData(shipped, bytes, size);
        free(bytes);

        /* First timeline from the original recorder's moment. */
        step_n(&ctx, 40);
        JoltC_RVec3 first = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, box[0]);

        /* Second timeline from the byte-shipped copy. */
        JoltC_StateRecorderImpl_Rewind(shipped);
        TEST_ASSERT(JoltC_PhysicsSystem_RestoreState(ctx.physicsSystem, shipped), "restore from shipped bytes succeeds");
        step_n(&ctx, 40);
        JoltC_RVec3 again = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, box[0]);

        TEST_ASSERT(first.x == again.x && first.y == again.y && first.z == again.z,
                    "the byte-shipped snapshot replays bit-identically");

        JoltC_StateRecorderImpl_Destroy(original);
        JoltC_StateRecorderImpl_Destroy(shipped);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_single_body_state */
    TEST_BEGIN("One body's state saves and restores alone");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        add_static_floor(&ctx);

        JoltC_RVec3 spawn = { 0.0f, 3.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, spawn, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);
        step_n(&ctx, 15);

        /* The pointer is grabbed through the no-lock interface and the lock dropped at once: the
         * body pointer itself is stable for the body's lifetime, and this test is single threaded.
         * Holding a real write lock across the steps below would deadlock the step's own locking. */
        const JoltC_BodyLockInterface* lockInterface = JoltC_PhysicsSystem_GetBodyLockInterfaceNoLock(ctx.physicsSystem);
        JoltC_BodyLockWrite* lock = JoltC_BodyLockWrite_Create(lockInterface, id);
        JoltC_Body* locked = JoltC_BodyLockWrite_GetBody(lock);
        JoltC_BodyLockWrite_Destroy(lock);
        TEST_ASSERT_NOT_NULL(locked, "body pointer obtained through the lock");

        JoltC_RVec3 savedPosition = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, id);
        JoltC_Vec3 savedVelocity = JoltC_BodyInterface_GetLinearVelocity(ctx.bodyInterface, id);

        JoltC_StateRecorder* recorder = JoltC_StateRecorderImpl_Create();
        JoltC_PhysicsSystem_SaveBodyState(ctx.physicsSystem, locked, recorder);

        /* Wreck the body: shove it and let it travel. */
        JoltC_Vec3 shove = { 8.0f, 6.0f, -3.0f };
        JoltC_BodyInterface_SetLinearVelocity(ctx.bodyInterface, id, shove);
        step_n(&ctx, 30);

        JoltC_StateRecorderImpl_Rewind(recorder);
        JoltC_PhysicsSystem_RestoreBodyState(ctx.physicsSystem, locked, recorder);

        JoltC_RVec3 position = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, id);
        JoltC_Vec3 velocity = JoltC_BodyInterface_GetLinearVelocity(ctx.bodyInterface, id);
        TEST_ASSERT(position.x == savedPosition.x && position.y == savedPosition.y && position.z == savedPosition.z,
                    "the body is back exactly where it was saved");
        TEST_ASSERT(velocity.x == savedVelocity.x && velocity.y == savedVelocity.y && velocity.z == savedVelocity.z,
                    "with exactly the velocity it had");

        JoltC_StateRecorderImpl_Destroy(recorder);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_character_state_rolls_back */
    TEST_BEGIN("A virtual character rolls back with the world");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        add_static_floor(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.3f);
        JoltC_CharacterVirtualSettings settings;
        JoltC_CharacterVirtualSettings_Init(&settings);
        settings.shape = capsule;

        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_RVec3 spawn = { 0.0f, 2.0f, 0.0f };
        JoltC_CharacterVirtual* character = JoltC_CharacterVirtual_Create(&settings, spawn, identity, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(character, "character created");

        JoltC_Vec3 gravity = { 0.0f, -9.81f, 0.0f };
        JoltC_Vec3 walk = { 2.0f, -5.0f, 0.0f };

        for (int i = 0; i < 30; i++)
        {
            JoltC_CharacterVirtual_SetLinearVelocity(character, walk);
            JoltC_CharacterVirtual_Update(character, 1.0f / 60.0f, gravity, ctx.tempAllocator);
            step_n(&ctx, 1);
        }

        /* The system snapshot never includes a virtual character; it saves alongside. */
        JoltC_StateRecorder* world = JoltC_StateRecorderImpl_Create();
        JoltC_StateRecorder* jones = JoltC_StateRecorderImpl_Create();
        JoltC_PhysicsSystem_SaveState(ctx.physicsSystem, world, JOLTC_STATE_RECORDER_STATE_ALL);
        JoltC_CharacterVirtual_SaveState(character, jones);
        TEST_ASSERT(JoltC_StateRecorderImpl_GetDataSize(jones) > 0, "the character wrote state");

        for (int i = 0; i < 30; i++)
        {
            JoltC_CharacterVirtual_SetLinearVelocity(character, walk);
            JoltC_CharacterVirtual_Update(character, 1.0f / 60.0f, gravity, ctx.tempAllocator);
            step_n(&ctx, 1);
        }
        JoltC_RVec3 first = JoltC_CharacterVirtual_GetPosition(character);

        JoltC_StateRecorderImpl_Rewind(world);
        JoltC_StateRecorderImpl_Rewind(jones);
        TEST_ASSERT(JoltC_PhysicsSystem_RestoreState(ctx.physicsSystem, world), "world restored");
        JoltC_CharacterVirtual_RestoreState(character, jones);

        for (int i = 0; i < 30; i++)
        {
            JoltC_CharacterVirtual_SetLinearVelocity(character, walk);
            JoltC_CharacterVirtual_Update(character, 1.0f / 60.0f, gravity, ctx.tempAllocator);
            step_n(&ctx, 1);
        }
        JoltC_RVec3 again = JoltC_CharacterVirtual_GetPosition(character);

        TEST_ASSERT(first.x == again.x && first.y == again.y && first.z == again.z,
                    "the character walked the same walk to the bit");

        JoltC_StateRecorderImpl_Destroy(world);
        JoltC_StateRecorderImpl_Destroy(jones);
        JoltC_CharacterVirtual_Destroy(character);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_recorder_odds_and_ends */
    TEST_BEGIN("Recorder validation flag and EOF behave");
    {
        JoltC_StateRecorder* recorder = JoltC_StateRecorderImpl_Create();

        TEST_ASSERT(JoltC_StateRecorder_IsValidating(recorder) == JOLTC_FALSE, "not validating by default");
        JoltC_StateRecorder_SetValidating(recorder, 1);
        TEST_ASSERT(JoltC_StateRecorder_IsValidating(recorder) == JOLTC_TRUE, "validating flag round trips");

        TEST_ASSERT(JoltC_StateRecorderImpl_GetDataSize(recorder) == 0, "a fresh recorder is empty");

        uint8_t bytes[4] = { 1, 2, 3, 4 };
        JoltC_StateRecorderImpl_SetData(recorder, bytes, 4);
        TEST_ASSERT(JoltC_StateRecorderImpl_GetDataSize(recorder) == 4, "SetData filled the stream");

        uint8_t readBack[4] = { 0, 0, 0, 0 };
        TEST_ASSERT(JoltC_StateRecorderImpl_CopyData(recorder, readBack, 4) == 4, "CopyData returns the size");
        TEST_ASSERT(memcmp(bytes, readBack, 4) == 0, "the bytes round trip");

        JoltC_StateRecorderImpl_Clear(recorder);
        TEST_ASSERT(JoltC_StateRecorderImpl_GetDataSize(recorder) == 0, "Clear empties the stream");

        JoltC_StateRecorderImpl_Destroy(recorder);
    }
    TEST_END();
}
