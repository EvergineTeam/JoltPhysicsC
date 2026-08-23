/* JoltC Test Suite -- phase 6: the debug renderer.
 * SPDX-License-Identifier: MIT
 *
 * Jolt draws what the solver actually holds and reduces it to lines, triangles and text through
 * three C callbacks. These tests count what arrives: solid shapes as triangles, wireframe
 * degradation to lines when no triangle callback exists, constraints and their limits, and the
 * internal structure of a soft body -- none of which any engine-side reimplementation can see.
 */

#include "test_common.h"

#include <string.h>

typedef struct DrawCounters {
    int lines;
    int triangles;
    int texts;
} DrawCounters;

static void count_line(void* userData, JoltC_RVec3 from, JoltC_RVec3 to, uint32_t color)
{
    (void)from; (void)to; (void)color;
    ((DrawCounters*)userData)->lines++;
}

static void count_triangle(void* userData, JoltC_RVec3 v1, JoltC_RVec3 v2, JoltC_RVec3 v3, uint32_t color, JoltC_Bool castShadow)
{
    (void)v1; (void)v2; (void)v3; (void)color; (void)castShadow;
    ((DrawCounters*)userData)->triangles++;
}

static void count_text(void* userData, JoltC_RVec3 position, const char* text, uint32_t color, float height)
{
    (void)position; (void)color; (void)height;
    if (text != NULL)
        ((DrawCounters*)userData)->texts++;
}

void run_phase6_tests(void);

void run_phase6_tests(void)
{
    /* test_bodies_arrive_as_triangles_and_lines */
    TEST_BEGIN("Bodies draw as triangles, or as lines without a triangle callback");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_RVec3 spawn = { 0.0f, 2.0f, 0.0f };
        create_test_box_body(&ctx, spawn, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        /* Full renderer: solid shapes arrive as triangles. */
        DrawCounters full;
        memset(&full, 0, sizeof(full));
        JoltC_DebugRenderer* renderer = JoltC_DebugRenderer_Create(count_line, count_triangle, count_text, &full);
        TEST_ASSERT_NOT_NULL(renderer, "renderer created");

        JoltC_PhysicsSystem_DrawBodies(ctx.physicsSystem, NULL, renderer);
        TEST_ASSERT(full.triangles > 0, "a solid box arrived as triangles");

        /* Line-only renderer: the same box degrades to its edges. */
        DrawCounters wire;
        memset(&wire, 0, sizeof(wire));
        JoltC_DebugRenderer* lineOnly = JoltC_DebugRenderer_Create(count_line, NULL, NULL, &wire);

        JoltC_PhysicsSystem_DrawBodies(ctx.physicsSystem, NULL, lineOnly);
        TEST_ASSERT(wire.lines > 0, "without a triangle callback the box arrived as lines");
        TEST_ASSERT(wire.triangles == 0, "and nothing pretended to be a triangle");

        /* The sleep statistics arrive as text. */
        DrawCounters stats;
        memset(&stats, 0, sizeof(stats));
        JoltC_DebugRenderer* textual = JoltC_DebugRenderer_Create(count_line, count_triangle, count_text, &stats);
        JoltC_BodyDrawSettings settings;
        JoltC_BodyDrawSettings_Init(&settings);
        TEST_ASSERT(settings.drawShape == JOLTC_TRUE, "Init carries Jolt's default of drawing shapes");
        settings.drawSleepStats = JOLTC_TRUE;
        JoltC_PhysicsSystem_DrawBodies(ctx.physicsSystem, &settings, textual);
        TEST_ASSERT(stats.texts > 0, "sleep statistics arrived as text");

        JoltC_DebugRenderer_Destroy(textual);
        JoltC_DebugRenderer_Destroy(lineOnly);
        JoltC_DebugRenderer_Destroy(renderer);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_constraints_and_limits_draw */
    TEST_BEGIN("Constraints and their limits reach the renderer");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_RVec3 posA = { 0.0f, 5.0f, 0.0f };
        JoltC_RVec3 posB = { 0.0f, 4.0f, 0.0f };
        JoltC_BodyID a = create_test_box_body(&ctx, posA, JOLTC_MOTION_TYPE_STATIC, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_BodyID b = create_test_box_body(&ctx, posB, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        JoltC_HingeConstraintSettings hinge;
        JoltC_HingeConstraintSettings_Init(&hinge);
        hinge.point1 = (JoltC_RVec3){ 0.0f, 4.5f, 0.0f };
        hinge.point2 = (JoltC_RVec3){ 0.0f, 4.5f, 0.0f };
        hinge.hingeAxis1 = (JoltC_Vec3){ 0.0f, 0.0f, 1.0f };
        hinge.hingeAxis2 = (JoltC_Vec3){ 0.0f, 0.0f, 1.0f };
        hinge.normalAxis1 = (JoltC_Vec3){ 1.0f, 0.0f, 0.0f };
        hinge.normalAxis2 = (JoltC_Vec3){ 1.0f, 0.0f, 0.0f };
        hinge.limitsMin = -0.5f;
        hinge.limitsMax = 0.5f;

        JoltC_Constraint* constraint = JoltC_HingeConstraint_Create(ctx.physicsSystem, a, b, &hinge);
        TEST_ASSERT_NOT_NULL(constraint, "limited hinge created");
        JoltC_PhysicsSystem_AddConstraint(ctx.physicsSystem, constraint);

        DrawCounters constraints;
        memset(&constraints, 0, sizeof(constraints));
        JoltC_DebugRenderer* renderer = JoltC_DebugRenderer_Create(count_line, count_triangle, count_text, &constraints);

        JoltC_PhysicsSystem_DrawConstraints(ctx.physicsSystem, renderer);
        TEST_ASSERT(constraints.lines + constraints.triangles > 0, "the hinge drew itself");

        DrawCounters limits;
        memset(&limits, 0, sizeof(limits));
        JoltC_DebugRenderer* limitRenderer = JoltC_DebugRenderer_Create(count_line, count_triangle, count_text, &limits);
        JoltC_PhysicsSystem_DrawConstraintLimits(ctx.physicsSystem, limitRenderer);
        TEST_ASSERT(limits.lines + limits.triangles > 0, "the limit arc drew, which no engine-side drawer shows today");

        JoltC_DebugRenderer_Destroy(limitRenderer);
        JoltC_DebugRenderer_Destroy(renderer);
        JoltC_PhysicsSystem_RemoveConstraint(ctx.physicsSystem, constraint);
        JoltC_Constraint_Destroy(constraint);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_soft_body_structure_draws */
    TEST_BEGIN("A soft body's internal structure reaches the renderer");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_SoftBodySharedSettings* shared = JoltC_SoftBodySharedSettings_CreateCube(3, 0.4f);
        JoltC_SoftBodyCreationSettings* creation = JoltC_SoftBodyCreationSettings_Create();
        JoltC_SoftBodyCreationSettings_SetSettings(creation, shared);
        JoltC_SoftBodyCreationSettings_SetPosition(creation, (JoltC_RVec3){ 0.0f, 3.0f, 0.0f });
        JoltC_SoftBodyCreationSettings_SetObjectLayer(creation, OBJ_LAYER_DYNAMIC);
        JoltC_Body* body = JoltC_BodyInterface_CreateSoftBody(ctx.bodyInterface, creation);
        JoltC_SoftBodyCreationSettings_Destroy(creation);
        TEST_ASSERT_NOT_NULL(body, "soft cube created");
        JoltC_BodyInterface_AddBody(ctx.bodyInterface, JoltC_Body_GetID(body), JOLTC_ACTIVATION_ACTIVATE);

        DrawCounters counters;
        memset(&counters, 0, sizeof(counters));
        JoltC_DebugRenderer* renderer = JoltC_DebugRenderer_Create(count_line, count_triangle, count_text, &counters);

        JoltC_BodyDrawSettings settings;
        JoltC_BodyDrawSettings_Init(&settings);
        settings.drawShape = JOLTC_FALSE;
        settings.drawSoftBodyVertices = JOLTC_TRUE;
        settings.drawSoftBodyEdgeConstraints = JOLTC_TRUE;
        settings.drawSoftBodyVolumeConstraints = JOLTC_TRUE;
        JoltC_PhysicsSystem_DrawBodies(ctx.physicsSystem, &settings, renderer);

        TEST_ASSERT(counters.lines + counters.triangles > 0,
                    "the cube's edges and volumes drew: the structure no surface mesh shows");

        JoltC_DebugRenderer_Destroy(renderer);
        JoltC_SoftBodySharedSettings_Release(shared);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_collector_hands_over_a_block */
    TEST_BEGIN("The collecting renderer hands over its lines as one block");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_RVec3 spawn = { 0.0f, 2.0f, 0.0f };
        create_test_box_body(&ctx, spawn, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        JoltC_DebugRenderer* collector = JoltC_DebugRenderer_CreateCollector();
        TEST_ASSERT_NOT_NULL(collector, "collector created");

        JoltC_PhysicsSystem_DrawBodies(ctx.physicsSystem, NULL, collector);

        const JoltC_DebugVertex* vertices = NULL;
        uint32_t count = JoltC_DebugRenderer_TakeVertices(collector, &vertices);
        TEST_ASSERT(count > 0, "the box was collected instead of called back");
        TEST_ASSERT(count % 2 == 0, "two vertices per line, so an even count");
        TEST_ASSERT_NOT_NULL(vertices, "and a block to read them from");

        const JoltC_DebugVertex* second = NULL;
        TEST_ASSERT(JoltC_DebugRenderer_TakeVertices(collector, &second) == 0,
                    "taking it a second time hands over nothing: the buffer was emptied");

        JoltC_PhysicsSystem_DrawBodies(ctx.physicsSystem, NULL, collector);
        TEST_ASSERT(JoltC_DebugRenderer_TakeVertices(collector, &second) == count,
                    "and the next frame collects the same box again");

        JoltC_DebugRenderer_Destroy(collector);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_camera_position_picks_the_level_of_detail */
    TEST_BEGIN("The camera position picks how finely a shape is drawn");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        /* A sphere, because a box carries one level of detail and a sphere carries four. */
        const JoltC_Shape* sphere = JoltC_SphereShape_Create(0.5f);
        JoltC_RVec3 away = { 0.0f, 0.0f, 100.0f };
        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_BodyCreationSettings* settings = JoltC_BodyCreationSettings_Create3(
            sphere, away, identity, JOLTC_MOTION_TYPE_STATIC, OBJ_LAYER_STATIC);
        JoltC_BodyID id = JoltC_BodyInterface_CreateAndAddBody(ctx.bodyInterface, settings, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_BodyCreationSettings_Destroy(settings);
        JoltC_Shape_Release(sphere);
        TEST_ASSERT(id != JOLTC_BODY_ID_INVALID, "distant sphere added");

        const JoltC_DebugVertex* vertices = NULL;
        JoltC_DebugRenderer* collector = JoltC_DebugRenderer_CreateCollector();

        /* Nobody has said where the camera is, so Jolt draws the finest level it holds. */
        JoltC_PhysicsSystem_DrawBodies(ctx.physicsSystem, NULL, collector);
        uint32_t blind = JoltC_DebugRenderer_TakeVertices(collector, &vertices);
        TEST_ASSERT(blind > 0, "the sphere drew");

        /* A hundred metres away, which is past every level of detail Jolt carries. */
        JoltC_RVec3 origin = { 0.0f, 0.0f, 0.0f };
        JoltC_DebugRenderer_SetCameraPos(collector, origin);
        JoltC_PhysicsSystem_DrawBodies(ctx.physicsSystem, NULL, collector);
        uint32_t distant = JoltC_DebugRenderer_TakeVertices(collector, &vertices);
        TEST_ASSERT(distant * 4 < blind, "a hundred metres away it drew a fraction of the vertices");

        /* Standing next to it, the finest level comes back. */
        JoltC_RVec3 next_to_it = { 0.0f, 0.0f, 99.0f };
        JoltC_DebugRenderer_SetCameraPos(collector, next_to_it);
        JoltC_PhysicsSystem_DrawBodies(ctx.physicsSystem, NULL, collector);
        uint32_t close = JoltC_DebugRenderer_TakeVertices(collector, &vertices);
        TEST_ASSERT(close == blind, "up close it is drawn as finely as when no camera was known");

        JoltC_DebugRenderer_Destroy(collector);
        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}
