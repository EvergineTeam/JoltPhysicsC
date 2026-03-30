/* JoltC Test Suite — query.h API tests (raycasting, broad/narrow phase)
 * SPDX-License-Identifier: MIT
 */

#include "test_common.h"

/* Broadphase callback — increments a counter */
static void bp_collide_sphere_callback(void* userData, JoltC_BodyID bodyID)
{
    (void)bodyID;
    int* count = (int*)userData;
    (*count)++;
}

void run_query_tests(void)
{
    /* test_get_narrow_phase_query */
    TEST_BEGIN("GetNarrowPhaseQuery non-null");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_NarrowPhaseQuery* npq = JoltC_PhysicsSystem_GetNarrowPhaseQuery(ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(npq, "NarrowPhaseQuery not null");

        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_get_broad_phase_query */
    TEST_BEGIN("GetBroadPhaseQuery non-null");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_BroadPhaseQuery* bpq = JoltC_PhysicsSystem_GetBroadPhaseQuery(ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(bpq, "BroadPhaseQuery not null");

        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_cast_ray_hit */
    TEST_BEGIN("CastRay hit a floor box");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        /* Create a static floor at origin */
        JoltC_RVec3 floorPos = { 0.0f, 0.0f, 0.0f };
        JoltC_BodyID floorID = create_test_box_body(&ctx, floorPos, JOLTC_MOTION_TYPE_STATIC, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_PhysicsSystem_OptimizeBroadPhase(ctx.physicsSystem);

        const JoltC_NarrowPhaseQuery* npq = JoltC_PhysicsSystem_GetNarrowPhaseQuery(ctx.physicsSystem);

        /* Cast ray from above, downward */
        JoltC_RVec3 origin = { 0.0f, 10.0f, 0.0f };
        JoltC_Vec3 direction = { 0.0f, -20.0f, 0.0f };
        JoltC_RayCastResult result;
        JoltC_Bool hit = JoltC_NarrowPhaseQuery_CastRay(npq, origin, direction, &result);
        TEST_ASSERT(hit, "Ray should hit the floor");
        TEST_ASSERT(result.fraction > 0.0f && result.fraction < 1.0f, "Fraction in (0,1)");

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, floorID);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, floorID);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_cast_ray_miss */
    TEST_BEGIN("CastRay miss (no geometry)");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_NarrowPhaseQuery* npq = JoltC_PhysicsSystem_GetNarrowPhaseQuery(ctx.physicsSystem);

        /* Cast ray into empty space */
        JoltC_RVec3 origin = { 0.0f, 10.0f, 0.0f };
        JoltC_Vec3 direction = { 0.0f, -20.0f, 0.0f };
        JoltC_RayCastResult result;
        result.fraction = -1.0f;
        JoltC_Bool hit = JoltC_NarrowPhaseQuery_CastRay(npq, origin, direction, &result);
        TEST_ASSERT(!hit, "Ray should miss in empty scene");

        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_cast_ray2_with_null_filters */
    TEST_BEGIN("CastRay2 with null filters");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_RVec3 floorPos = { 0.0f, 0.0f, 0.0f };
        JoltC_BodyID floorID = create_test_box_body(&ctx, floorPos, JOLTC_MOTION_TYPE_STATIC, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_PhysicsSystem_OptimizeBroadPhase(ctx.physicsSystem);

        const JoltC_NarrowPhaseQuery* npq = JoltC_PhysicsSystem_GetNarrowPhaseQuery(ctx.physicsSystem);

        JoltC_RVec3 origin = { 0.0f, 10.0f, 0.0f };
        JoltC_Vec3 direction = { 0.0f, -20.0f, 0.0f };
        JoltC_RayCastResult result;
        JoltC_Bool hit = JoltC_NarrowPhaseQuery_CastRay2(npq, origin, direction, &result, NULL, NULL, NULL);
        TEST_ASSERT(hit, "CastRay2 should hit floor");

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, floorID);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, floorID);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_broad_phase_collide_sphere */
    TEST_BEGIN("BroadPhaseQuery CollideSphere");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_RVec3 boxPos = { 0.0f, 0.0f, 0.0f };
        JoltC_BodyID boxID = create_test_box_body(&ctx, boxPos, JOLTC_MOTION_TYPE_STATIC, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_PhysicsSystem_OptimizeBroadPhase(ctx.physicsSystem);

        const JoltC_BroadPhaseQuery* bpq = JoltC_PhysicsSystem_GetBroadPhaseQuery(ctx.physicsSystem);

        int hitCount = 0;
        JoltC_Vec3 center = { 0.0f, 0.0f, 0.0f };
        JoltC_BroadPhaseQuery_CollideSphere(bpq, center, 5.0f, bp_collide_sphere_callback, &hitCount, NULL, NULL);
        TEST_ASSERT(hitCount >= 1, "CollideSphere should find the box");

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, boxID);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, boxID);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}
