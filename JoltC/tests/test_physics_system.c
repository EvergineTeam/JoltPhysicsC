/* JoltC Test Suite — physics_system.h API tests
 * SPDX-License-Identifier: MIT
 */

#include "test_common.h"
#include <string.h>

void run_physics_system_tests(void)
{
    /* test_create_temp_allocator */
    TEST_BEGIN("TempAllocator create/destroy");
    {
        JoltC_TempAllocator* alloc = JoltC_TempAllocator_Create(10 * 1024 * 1024);
        TEST_ASSERT_NOT_NULL(alloc, "TempAllocator not null");
        JoltC_TempAllocator_Destroy(alloc);
    }
    TEST_END();

    /* test_create_job_system */
    TEST_BEGIN("JobSystemThreadPool create/destroy");
    {
        JoltC_JobSystem* js = JoltC_JobSystemThreadPool_Create(2048, 8, 2);
        TEST_ASSERT_NOT_NULL(js, "JobSystem not null");
        JoltC_JobSystem_Destroy(js);
    }
    TEST_END();

    /* test_create_physics_system */
    TEST_BEGIN("PhysicsSystem full init");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        TEST_ASSERT_NOT_NULL(ctx.physicsSystem, "PhysicsSystem not null");
        TEST_ASSERT_NOT_NULL(ctx.bodyInterface, "BodyInterface not null");
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_get_body_interface */
    TEST_BEGIN("GetBodyInterface returns non-null");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_BodyInterface* bi = JoltC_PhysicsSystem_GetBodyInterface(ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(bi, "BodyInterface from GetBodyInterface");
        JoltC_BodyInterface* biNoLock = JoltC_PhysicsSystem_GetBodyInterfaceNoLock(ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(biNoLock, "BodyInterfaceNoLock");
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_gravity */
    TEST_BEGIN("Set/Get gravity");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_Vec3 g = { 0.0f, -9.81f, 0.0f };
        JoltC_PhysicsSystem_SetGravity(ctx.physicsSystem, g);
        JoltC_Vec3 got = JoltC_PhysicsSystem_GetGravity(ctx.physicsSystem);
        TEST_ASSERT_FLOAT_EQ(got.x, 0.0f, 0.001f, "gravity.x == 0");
        TEST_ASSERT_FLOAT_EQ(got.y, -9.81f, 0.01f, "gravity.y == -9.81");
        TEST_ASSERT_FLOAT_EQ(got.z, 0.0f, 0.001f, "gravity.z == 0");
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_optimize_broadphase */
    TEST_BEGIN("OptimizeBroadPhase no crash");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_PhysicsSystem_OptimizeBroadPhase(ctx.physicsSystem);
        TEST_ASSERT(1, "No crash");
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_update */
    TEST_BEGIN("Single physics Update step");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        uint32_t err = JoltC_PhysicsSystem_Update(ctx.physicsSystem, 1.0f / 60.0f, 1,
                                                   ctx.tempAllocator, ctx.jobSystem);
        TEST_ASSERT(err == 0, "Update returned 0 (no error)");
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_get_num_bodies */
    TEST_BEGIN("GetNumBodies starts at 0, increments after add");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        uint32_t initial = JoltC_PhysicsSystem_GetNumBodies(ctx.physicsSystem);
        TEST_ASSERT(initial == 0, "Initial body count == 0");

        JoltC_RVec3 pos = { 0.0f, 0.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);
        TEST_ASSERT(id != JOLTC_BODY_ID_INVALID, "Body created");

        uint32_t after = JoltC_PhysicsSystem_GetNumBodies(ctx.physicsSystem);
        TEST_ASSERT(after == 1, "Body count == 1 after add");

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, id);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_physics_settings */
    TEST_BEGIN("PhysicsSettings get/set round-trip");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_PhysicsSettings settings;
        JoltC_PhysicsSystem_GetPhysicsSettings(ctx.physicsSystem, &settings);
        uint32_t origVelSteps = settings.numVelocitySteps;
        settings.numVelocitySteps = 20;
        JoltC_PhysicsSystem_SetPhysicsSettings(ctx.physicsSystem, &settings);
        JoltC_PhysicsSettings got;
        JoltC_PhysicsSystem_GetPhysicsSettings(ctx.physicsSystem, &got);
        TEST_ASSERT(got.numVelocitySteps == 20, "velocity steps == 20 after set");
        /* restore */
        settings.numVelocitySteps = origVelSteps;
        JoltC_PhysicsSystem_SetPhysicsSettings(ctx.physicsSystem, &settings);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}
