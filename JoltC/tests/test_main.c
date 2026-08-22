/* JoltC Test Suite — entry point and test runner
 * SPDX-License-Identifier: MIT
 */

#include "test_common.h"
#include <stdio.h>
#include <string.h>

/* ========================================================================== */
/*  Global counters                                                           */
/* ========================================================================== */
int g_tests_run = 0;
int g_tests_passed = 0;
int g_tests_failed = 0;
int g_current_test_failed = 0;

/* ========================================================================== */
/*  Shared physics context helpers                                            */
/* ========================================================================== */
void setup_physics_context(TestPhysicsContext* ctx)
{
    memset(ctx, 0, sizeof(*ctx));

    ctx->tempAllocator = JoltC_TempAllocator_Create(10 * 1024 * 1024); /* 10 MB */
    ctx->jobSystem = JoltC_JobSystemThreadPool_Create(2048, 8, 2);

    /* 2-layer object filter table: static-static off, static-dynamic on, dynamic-dynamic on */
    ctx->objectLayerPairFilter = JoltC_ObjectLayerPairFilterTable_Create(NUM_OBJ_LAYERS);
    JoltC_ObjectLayerPairFilterTable_EnableCollision(ctx->objectLayerPairFilter, OBJ_LAYER_STATIC, OBJ_LAYER_DYNAMIC);
    JoltC_ObjectLayerPairFilterTable_EnableCollision(ctx->objectLayerPairFilter, OBJ_LAYER_DYNAMIC, OBJ_LAYER_DYNAMIC);

    /* Map object layers to broadphase layers */
    ctx->broadPhaseLayerInterface = JoltC_BroadPhaseLayerInterfaceTable_Create(NUM_OBJ_LAYERS, NUM_BP_LAYERS);
    JoltC_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(ctx->broadPhaseLayerInterface, OBJ_LAYER_STATIC, BP_LAYER_NON_MOVING);
    JoltC_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(ctx->broadPhaseLayerInterface, OBJ_LAYER_DYNAMIC, BP_LAYER_MOVING);

    /* Object vs broadphase filter */
    ctx->objectVsBroadPhaseLayerFilter = JoltC_ObjectVsBroadPhaseLayerFilterTable_Create(
        ctx->broadPhaseLayerInterface, NUM_BP_LAYERS,
        ctx->objectLayerPairFilter, NUM_OBJ_LAYERS);

    /* Physics system */
    ctx->physicsSystem = JoltC_PhysicsSystem_Create();
    JoltC_PhysicsSystem_Init(ctx->physicsSystem,
        1024,   /* maxBodies */
        0,      /* numBodyMutexes (0 = auto) */
        1024,   /* maxBodyPairs */
        1024,   /* maxContactConstraints */
        ctx->broadPhaseLayerInterface,
        ctx->objectVsBroadPhaseLayerFilter,
        ctx->objectLayerPairFilter);

    ctx->bodyInterface = JoltC_PhysicsSystem_GetBodyInterface(ctx->physicsSystem);
}

void teardown_physics_context(TestPhysicsContext* ctx)
{
    if (ctx->physicsSystem)  JoltC_PhysicsSystem_Destroy(ctx->physicsSystem);
    /* Filters are destroyed with physics system or manually: */
    if (ctx->objectVsBroadPhaseLayerFilter)
        JoltC_ObjectVsBroadPhaseLayerFilter_Destroy(ctx->objectVsBroadPhaseLayerFilter);
    if (ctx->broadPhaseLayerInterface)
        JoltC_BroadPhaseLayerInterface_Destroy(ctx->broadPhaseLayerInterface);
    if (ctx->objectLayerPairFilter)
        JoltC_ObjectLayerPairFilter_Destroy(ctx->objectLayerPairFilter);
    if (ctx->jobSystem)      JoltC_JobSystem_Destroy(ctx->jobSystem);
    if (ctx->tempAllocator)  JoltC_TempAllocator_Destroy(ctx->tempAllocator);
    memset(ctx, 0, sizeof(*ctx));
}

JoltC_BodyID create_test_box_body(TestPhysicsContext* ctx, JoltC_RVec3 position,
                                  JoltC_MotionType motionType, JoltC_Activation activation)
{
    JoltC_Vec3 halfExtent = { 0.5f, 0.5f, 0.5f };
    const JoltC_Shape* shape = JoltC_BoxShape_Create(halfExtent, 0.0f);

    JoltC_ObjectLayer layer = (motionType == JOLTC_MOTION_TYPE_STATIC)
        ? OBJ_LAYER_STATIC : OBJ_LAYER_DYNAMIC;
    JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };

    JoltC_BodyCreationSettings* settings = JoltC_BodyCreationSettings_Create3(
        shape, position, identity, motionType, layer);

    JoltC_BodyID bodyID = JoltC_BodyInterface_CreateAndAddBody(ctx->bodyInterface, settings, activation);

    JoltC_BodyCreationSettings_Destroy(settings);
    JoltC_Shape_Release(shape);

    return bodyID;
}

/* ========================================================================== */
/*  Main                                                                      */
/* ========================================================================== */
int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    /* Unbuffered, so a crash names the test that caused it.
     *
     * With buffering on, a segfault discards whatever had not been flushed, and the last
     * line CI shows is wherever the buffer happened to end -- which sent us looking at a
     * test that had already passed. The buffer is not worth the minutes. */
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("=== JoltC API Binding Test Suite ===\n\n");

    int init_result = JoltC_Init();
    if (!init_result) {
        printf("FATAL: JoltC_Init() failed!\n");
        return 1;
    }

    printf("[SUITE] Common\n");
    run_common_tests();

    printf("\n[SUITE] Math\n");
    run_math_tests();

    printf("\n[SUITE] PhysicsSystem\n");
    run_physics_system_tests();

    printf("\n[SUITE] Shape\n");
    run_shape_tests();

    printf("\n[SUITE] Body\n");
    run_body_tests();

    printf("\n[SUITE] Constraint\n");
    run_constraint_tests();

    printf("\n[SUITE] Filter\n");
    run_filter_tests();

    printf("\n[SUITE] Query\n");
    run_query_tests();

    printf("\n[SUITE] Character\n");
    run_character_tests();

    printf("\n[SUITE] Skeleton\n");
    run_skeleton_tests();

    printf("\n[SUITE] Vehicle\n");
    run_vehicle_tests();

    printf("\n[SUITE] BodyAccess\n");
    run_body_access_tests();

    printf("\n[SUITE] MathRoundTrip\n");
    run_math_roundtrip_tests();

    printf("\n[SUITE] CharacterExtra\n");
    run_character_extra_tests();

    printf("\n[SUITE] ShapeProperties\n");
    run_shape_props_tests();

    printf("\n[SUITE] SkeletonExtra\n");
    run_skeleton_extra_tests();

    printf("\n[SUITE] VehicleExtra\n");
    run_vehicle_extra_tests();

    run_vehicle_live_tests();

    JoltC_Shutdown();

    printf("\n=== Results: %d/%d passed, %d failed ===\n",
           g_tests_passed, g_tests_run, g_tests_failed);

    return (g_tests_failed > 0) ? 1 : 0;
}
