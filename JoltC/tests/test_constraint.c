/* JoltC Test Suite — constraint.h API tests
 * SPDX-License-Identifier: MIT
 */

#include "test_common.h"

/* Helper: create two dynamic bodies for constraint testing */
static void create_two_bodies(TestPhysicsContext* ctx, JoltC_BodyID* out1, JoltC_BodyID* out2)
{
    JoltC_RVec3 pos1 = { -2.0f, 5.0f, 0.0f };
    JoltC_RVec3 pos2 = {  2.0f, 5.0f, 0.0f };
    *out1 = create_test_box_body(ctx, pos1, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);
    *out2 = create_test_box_body(ctx, pos2, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);
}

static void destroy_two_bodies(TestPhysicsContext* ctx, JoltC_BodyID b1, JoltC_BodyID b2)
{
    JoltC_BodyInterface_RemoveBody(ctx->bodyInterface, b1);
    JoltC_BodyInterface_DestroyBody(ctx->bodyInterface, b1);
    JoltC_BodyInterface_RemoveBody(ctx->bodyInterface, b2);
    JoltC_BodyInterface_DestroyBody(ctx->bodyInterface, b2);
}

void run_constraint_tests(void)
{
    /* test_point_constraint_default */
    TEST_BEGIN("PointConstraint create with defaults not null");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_BodyID b1, b2;
        create_two_bodies(&ctx, &b1, &b2);

        JoltC_PointConstraintSettings settings;
        JoltC_PointConstraintSettings_Init(&settings);

        JoltC_Constraint* c = JoltC_PointConstraint_Create(ctx.physicsSystem, b1, b2, &settings);
        TEST_ASSERT_NOT_NULL(c, "PointConstraint default not null");
        TEST_ASSERT(JoltC_Constraint_GetSubType(c) == JOLTC_CONSTRAINT_SUB_TYPE_POINT, "subtype == POINT");

        JoltC_Constraint_Release(c);
        destroy_two_bodies(&ctx, b1, b2);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_fixed_constraint_default */
    TEST_BEGIN("FixedConstraint create with defaults not null");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_BodyID b1, b2;
        create_two_bodies(&ctx, &b1, &b2);

        JoltC_FixedConstraintSettings settings;
        JoltC_FixedConstraintSettings_Init(&settings);

        JoltC_Constraint* c = JoltC_FixedConstraint_Create(ctx.physicsSystem, b1, b2, &settings);
        TEST_ASSERT_NOT_NULL(c, "FixedConstraint default not null");
        TEST_ASSERT(JoltC_Constraint_GetSubType(c) == JOLTC_CONSTRAINT_SUB_TYPE_FIXED, "subtype == FIXED");

        JoltC_Constraint_Release(c);
        destroy_two_bodies(&ctx, b1, b2);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_point_constraint */
    TEST_BEGIN("PointConstraint create, enable/disable");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_BodyID b1, b2;
        create_two_bodies(&ctx, &b1, &b2);

        JoltC_PointConstraintSettings settings;
        JoltC_PointConstraintSettings_Init(&settings);
        settings.space = JOLTC_CONSTRAINT_SPACE_WORLD_SPACE;
        settings.point1 = (JoltC_RVec3){ 0.0f, 5.0f, 0.0f };
        settings.point2 = (JoltC_RVec3){ 0.0f, 5.0f, 0.0f };

        JoltC_Constraint* c = JoltC_PointConstraint_Create(ctx.physicsSystem, b1, b2, &settings);
        TEST_ASSERT_NOT_NULL(c, "PointConstraint not null");

        JoltC_Constraint_SetEnabled(c, JOLTC_FALSE);
        TEST_ASSERT(!JoltC_Constraint_GetEnabled(c), "Disabled");
        JoltC_Constraint_SetEnabled(c, JOLTC_TRUE);
        TEST_ASSERT(JoltC_Constraint_GetEnabled(c), "Re-enabled");

        JoltC_Constraint_Release(c);
        destroy_two_bodies(&ctx, b1, b2);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_fixed_constraint */
    TEST_BEGIN("FixedConstraint create");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_BodyID b1, b2;
        create_two_bodies(&ctx, &b1, &b2);

        JoltC_FixedConstraintSettings settings;
        JoltC_FixedConstraintSettings_Init(&settings);
        settings.space = JOLTC_CONSTRAINT_SPACE_WORLD_SPACE;
        settings.autoDetectPoint = JOLTC_TRUE;

        JoltC_Constraint* c = JoltC_FixedConstraint_Create(ctx.physicsSystem, b1, b2, &settings);
        TEST_ASSERT_NOT_NULL(c, "FixedConstraint not null");
        TEST_ASSERT(JoltC_Constraint_GetSubType(c) == JOLTC_CONSTRAINT_SUB_TYPE_FIXED, "subtype == FIXED");

        JoltC_Constraint_Release(c);
        destroy_two_bodies(&ctx, b1, b2);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_distance_constraint */
    TEST_BEGIN("DistanceConstraint create, get/set distance");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_BodyID b1, b2;
        create_two_bodies(&ctx, &b1, &b2);

        JoltC_DistanceConstraintSettings settings;
        JoltC_DistanceConstraintSettings_Init(&settings);
        settings.space = JOLTC_CONSTRAINT_SPACE_WORLD_SPACE;
        settings.point1 = (JoltC_RVec3){ -2.0f, 5.0f, 0.0f };
        settings.point2 = (JoltC_RVec3){  2.0f, 5.0f, 0.0f };
        settings.minDistance = 1.0f;
        settings.maxDistance = 5.0f;

        JoltC_Constraint* c = JoltC_DistanceConstraint_Create(ctx.physicsSystem, b1, b2, &settings);
        TEST_ASSERT_NOT_NULL(c, "DistanceConstraint not null");
        TEST_ASSERT_FLOAT_EQ(JoltC_DistanceConstraint_GetMinDistance(c), 1.0f, 0.01f, "min == 1");
        TEST_ASSERT_FLOAT_EQ(JoltC_DistanceConstraint_GetMaxDistance(c), 5.0f, 0.01f, "max == 5");

        JoltC_Constraint_Release(c);
        destroy_two_bodies(&ctx, b1, b2);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_hinge_constraint */
    TEST_BEGIN("HingeConstraint create");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_BodyID b1, b2;
        create_two_bodies(&ctx, &b1, &b2);

        JoltC_HingeConstraintSettings settings;
        JoltC_HingeConstraintSettings_Init(&settings);
        settings.space = JOLTC_CONSTRAINT_SPACE_WORLD_SPACE;
        settings.point1 = (JoltC_RVec3){ 0.0f, 5.0f, 0.0f };
        settings.hingeAxis1 = (JoltC_Vec3){ 0.0f, 1.0f, 0.0f };
        settings.normalAxis1 = (JoltC_Vec3){ 1.0f, 0.0f, 0.0f };
        settings.point2 = (JoltC_RVec3){ 0.0f, 5.0f, 0.0f };
        settings.hingeAxis2 = (JoltC_Vec3){ 0.0f, 1.0f, 0.0f };
        settings.normalAxis2 = (JoltC_Vec3){ 1.0f, 0.0f, 0.0f };

        JoltC_Constraint* c = JoltC_HingeConstraint_Create(ctx.physicsSystem, b1, b2, &settings);
        TEST_ASSERT_NOT_NULL(c, "HingeConstraint not null");
        TEST_ASSERT(JoltC_Constraint_GetSubType(c) == JOLTC_CONSTRAINT_SUB_TYPE_HINGE, "subtype == HINGE");

        JoltC_Constraint_Release(c);
        destroy_two_bodies(&ctx, b1, b2);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_slider_constraint */
    TEST_BEGIN("SliderConstraint create");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_BodyID b1, b2;
        create_two_bodies(&ctx, &b1, &b2);

        JoltC_SliderConstraintSettings settings;
        JoltC_SliderConstraintSettings_Init(&settings);
        settings.space = JOLTC_CONSTRAINT_SPACE_WORLD_SPACE;
        settings.autoDetectPoint = JOLTC_TRUE;
        JoltC_Vec3 axis = { 1.0f, 0.0f, 0.0f };
        JoltC_SliderConstraintSettings_SetSliderAxis(&settings, axis);

        JoltC_Constraint* c = JoltC_SliderConstraint_Create(ctx.physicsSystem, b1, b2, &settings);
        TEST_ASSERT_NOT_NULL(c, "SliderConstraint not null");
        TEST_ASSERT(JoltC_Constraint_GetSubType(c) == JOLTC_CONSTRAINT_SUB_TYPE_SLIDER, "subtype == SLIDER");

        JoltC_Constraint_Release(c);
        destroy_two_bodies(&ctx, b1, b2);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_cone_constraint */
    TEST_BEGIN("ConeConstraint create");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_BodyID b1, b2;
        create_two_bodies(&ctx, &b1, &b2);

        JoltC_ConeConstraintSettings settings;
        JoltC_ConeConstraintSettings_Init(&settings);
        settings.space = JOLTC_CONSTRAINT_SPACE_WORLD_SPACE;
        settings.point1 = (JoltC_RVec3){ 0.0f, 5.0f, 0.0f };
        settings.twistAxis1 = (JoltC_Vec3){ 1.0f, 0.0f, 0.0f };
        settings.point2 = (JoltC_RVec3){ 0.0f, 5.0f, 0.0f };
        settings.twistAxis2 = (JoltC_Vec3){ 1.0f, 0.0f, 0.0f };
        settings.halfConeAngle = 0.5f;

        JoltC_Constraint* c = JoltC_ConeConstraint_Create(ctx.physicsSystem, b1, b2, &settings);
        TEST_ASSERT_NOT_NULL(c, "ConeConstraint not null");
        TEST_ASSERT(JoltC_Constraint_GetSubType(c) == JOLTC_CONSTRAINT_SUB_TYPE_CONE, "subtype == CONE");

        JoltC_Constraint_Release(c);
        destroy_two_bodies(&ctx, b1, b2);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_swing_twist_constraint */
    TEST_BEGIN("SwingTwistConstraint create");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_BodyID b1, b2;
        create_two_bodies(&ctx, &b1, &b2);

        JoltC_SwingTwistConstraintSettings settings;
        JoltC_SwingTwistConstraintSettings_Init(&settings);
        settings.space = JOLTC_CONSTRAINT_SPACE_WORLD_SPACE;
        settings.position1 = (JoltC_RVec3){ 0.0f, 5.0f, 0.0f };
        settings.twistAxis1 = (JoltC_Vec3){ 1.0f, 0.0f, 0.0f };
        settings.planeAxis1 = (JoltC_Vec3){ 0.0f, 1.0f, 0.0f };
        settings.position2 = (JoltC_RVec3){ 0.0f, 5.0f, 0.0f };
        settings.twistAxis2 = (JoltC_Vec3){ 1.0f, 0.0f, 0.0f };
        settings.planeAxis2 = (JoltC_Vec3){ 0.0f, 1.0f, 0.0f };

        JoltC_Constraint* c = JoltC_SwingTwistConstraint_Create(ctx.physicsSystem, b1, b2, &settings);
        TEST_ASSERT_NOT_NULL(c, "SwingTwistConstraint not null");
        TEST_ASSERT(JoltC_Constraint_GetSubType(c) == JOLTC_CONSTRAINT_SUB_TYPE_SWING_TWIST, "subtype == SWING_TWIST");

        JoltC_Constraint_Release(c);
        destroy_two_bodies(&ctx, b1, b2);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_six_dof_constraint */
    TEST_BEGIN("SixDOFConstraint create");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_BodyID b1, b2;
        create_two_bodies(&ctx, &b1, &b2);

        JoltC_SixDOFConstraintSettings settings;
        JoltC_SixDOFConstraintSettings_Init(&settings);
        settings.space = JOLTC_CONSTRAINT_SPACE_WORLD_SPACE;
        settings.position1 = (JoltC_RVec3){ 0.0f, 5.0f, 0.0f };
        settings.axisX1 = (JoltC_Vec3){ 1.0f, 0.0f, 0.0f };
        settings.axisY1 = (JoltC_Vec3){ 0.0f, 1.0f, 0.0f };
        settings.position2 = (JoltC_RVec3){ 0.0f, 5.0f, 0.0f };
        settings.axisX2 = (JoltC_Vec3){ 1.0f, 0.0f, 0.0f };
        settings.axisY2 = (JoltC_Vec3){ 0.0f, 1.0f, 0.0f };

        JoltC_Constraint* c = JoltC_SixDOFConstraint_Create(ctx.physicsSystem, b1, b2, &settings);
        TEST_ASSERT_NOT_NULL(c, "SixDOFConstraint not null");
        TEST_ASSERT(JoltC_Constraint_GetSubType(c) == JOLTC_CONSTRAINT_SUB_TYPE_SIX_DOF, "subtype == SIX_DOF");

        JoltC_Constraint_Release(c);
        destroy_two_bodies(&ctx, b1, b2);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_constraint_add_remove */
    TEST_BEGIN("Constraint add/remove from system");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_BodyID b1, b2;
        create_two_bodies(&ctx, &b1, &b2);

        JoltC_PointConstraintSettings settings;
        JoltC_PointConstraintSettings_Init(&settings);
        settings.space = JOLTC_CONSTRAINT_SPACE_WORLD_SPACE;
        settings.point1 = (JoltC_RVec3){ 0.0f, 5.0f, 0.0f };
        settings.point2 = (JoltC_RVec3){ 0.0f, 5.0f, 0.0f };

        JoltC_Constraint* c = JoltC_PointConstraint_Create(ctx.physicsSystem, b1, b2, &settings);
        JoltC_PhysicsSystem_AddConstraint(ctx.physicsSystem, c);
        TEST_ASSERT(JoltC_PhysicsSystem_GetNumConstraints(ctx.physicsSystem) == 1, "1 constraint after add");

        JoltC_PhysicsSystem_RemoveConstraint(ctx.physicsSystem, c);
        TEST_ASSERT(JoltC_PhysicsSystem_GetNumConstraints(ctx.physicsSystem) == 0, "0 constraints after remove");

        JoltC_Constraint_Release(c);
        destroy_two_bodies(&ctx, b1, b2);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_constraint_user_data */
    TEST_BEGIN("Constraint user data");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_BodyID b1, b2;
        create_two_bodies(&ctx, &b1, &b2);

        JoltC_PointConstraintSettings settings;
        JoltC_PointConstraintSettings_Init(&settings);
        settings.space = JOLTC_CONSTRAINT_SPACE_WORLD_SPACE;
        settings.point1 = (JoltC_RVec3){ 0.0f, 5.0f, 0.0f };
        settings.point2 = (JoltC_RVec3){ 0.0f, 5.0f, 0.0f };

        JoltC_Constraint* c = JoltC_PointConstraint_Create(ctx.physicsSystem, b1, b2, &settings);
        JoltC_Constraint_SetUserData(c, 42);
        TEST_ASSERT(JoltC_Constraint_GetUserData(c) == 42, "user data == 42");

        JoltC_Constraint_Release(c);
        destroy_two_bodies(&ctx, b1, b2);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}
