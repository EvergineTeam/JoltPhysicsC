/* JoltC Test Suite -- body.h API tests
 * SPDX-License-Identifier: MIT
 */

#include "test_common.h"

void run_body_tests(void)
{
    /* test_body_creation_settings */
    TEST_BEGIN("BodyCreationSettings properties round-trip");
    {
        const JoltC_Shape* shape = JoltC_SphereShape_Create(0.5f);
        JoltC_RVec3 pos = { 1.0f, 2.0f, 3.0f };
        JoltC_Quat rot = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_BodyCreationSettings* s = JoltC_BodyCreationSettings_Create3(
            shape, pos, rot, JOLTC_MOTION_TYPE_DYNAMIC, OBJ_LAYER_DYNAMIC);

        JoltC_BodyCreationSettings_SetFriction(s, 0.5f);
        TEST_ASSERT_FLOAT_EQ(JoltC_BodyCreationSettings_GetFriction(s), 0.5f, 0.001f, "friction == 0.5");

        JoltC_BodyCreationSettings_SetRestitution(s, 0.3f);
        TEST_ASSERT_FLOAT_EQ(JoltC_BodyCreationSettings_GetRestitution(s), 0.3f, 0.001f, "restitution == 0.3");

        JoltC_BodyCreationSettings_SetMotionType(s, JOLTC_MOTION_TYPE_KINEMATIC);
        TEST_ASSERT(JoltC_BodyCreationSettings_GetMotionType(s) == JOLTC_MOTION_TYPE_KINEMATIC, "motion == KINEMATIC");

        JoltC_BodyCreationSettings_Destroy(s);
        JoltC_Shape_Release(shape);
    }
    TEST_END();

    /* test_create_and_add_body */
    TEST_BEGIN("CreateAndAddBody returns valid ID");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 5.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);
        TEST_ASSERT(id != JOLTC_BODY_ID_INVALID, "BodyID != INVALID");
        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, id);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_body_position */
    TEST_BEGIN("Body set/get position");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 0.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        JoltC_RVec3 newPos = { 5.0f, 10.0f, 15.0f };
        JoltC_BodyInterface_SetPosition(ctx.bodyInterface, id, newPos, JOLTC_ACTIVATION_ACTIVATE);
        JoltC_RVec3 got = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, id);
        TEST_ASSERT_FLOAT_EQ(got.x, 5.0f, 0.01f, "pos.x == 5");
        TEST_ASSERT_FLOAT_EQ(got.y, 10.0f, 0.01f, "pos.y == 10");
        TEST_ASSERT_FLOAT_EQ(got.z, 15.0f, 0.01f, "pos.z == 15");

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, id);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_body_rotation */
    TEST_BEGIN("Body set/get rotation");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 0.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        JoltC_Quat q = { 0.0f, 0.7071f, 0.0f, 0.7071f }; /* ~45 deg Y */
        JoltC_BodyInterface_SetRotation(ctx.bodyInterface, id, q, JOLTC_ACTIVATION_ACTIVATE);
        JoltC_Quat got = JoltC_BodyInterface_GetRotation(ctx.bodyInterface, id);
        TEST_ASSERT_FLOAT_EQ(got.y, 0.7071f, 0.01f, "rot.y ~ 0.7071");
        TEST_ASSERT_FLOAT_EQ(got.w, 0.7071f, 0.01f, "rot.w ~ 0.7071");

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, id);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_body_velocity */
    TEST_BEGIN("Body set/get linear and angular velocity");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 10.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        JoltC_Vec3 lv = { 1.0f, 2.0f, 3.0f };
        JoltC_BodyInterface_SetLinearVelocity(ctx.bodyInterface, id, lv);
        JoltC_Vec3 gotLv = JoltC_BodyInterface_GetLinearVelocity(ctx.bodyInterface, id);
        TEST_ASSERT_FLOAT_EQ(gotLv.x, 1.0f, 0.01f, "lv.x == 1");

        JoltC_Vec3 av = { 0.0f, 1.0f, 0.0f };
        JoltC_BodyInterface_SetAngularVelocity(ctx.bodyInterface, id, av);
        JoltC_Vec3 gotAv = JoltC_BodyInterface_GetAngularVelocity(ctx.bodyInterface, id);
        TEST_ASSERT_FLOAT_EQ(gotAv.y, 1.0f, 0.01f, "av.y == 1");

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, id);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_body_activation */
    TEST_BEGIN("Body activate/deactivate");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 10.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        TEST_ASSERT(JoltC_BodyInterface_IsActive(ctx.bodyInterface, id), "Initially active");
        JoltC_BodyInterface_DeactivateBody(ctx.bodyInterface, id);
        TEST_ASSERT(!JoltC_BodyInterface_IsActive(ctx.bodyInterface, id), "Deactivated");
        JoltC_BodyInterface_ActivateBody(ctx.bodyInterface, id);
        TEST_ASSERT(JoltC_BodyInterface_IsActive(ctx.bodyInterface, id), "Re-activated");

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, id);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_body_motion_type */
    TEST_BEGIN("Body motion type change");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 0.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        JoltC_BodyInterface_SetMotionType(ctx.bodyInterface, id, JOLTC_MOTION_TYPE_KINEMATIC, JOLTC_ACTIVATION_ACTIVATE);
        TEST_ASSERT(JoltC_BodyInterface_GetMotionType(ctx.bodyInterface, id) == JOLTC_MOTION_TYPE_KINEMATIC, "KINEMATIC");

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, id);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_body_properties */
    TEST_BEGIN("Body friction, restitution, gravity factor");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 5.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        JoltC_BodyInterface_SetFriction(ctx.bodyInterface, id, 0.5f);
        TEST_ASSERT_FLOAT_EQ(JoltC_BodyInterface_GetFriction(ctx.bodyInterface, id), 0.5f, 0.001f, "friction == 0.5");

        JoltC_BodyInterface_SetRestitution(ctx.bodyInterface, id, 0.3f);
        TEST_ASSERT_FLOAT_EQ(JoltC_BodyInterface_GetRestitution(ctx.bodyInterface, id), 0.3f, 0.001f, "restitution == 0.3");

        JoltC_BodyInterface_SetGravityFactor(ctx.bodyInterface, id, 2.0f);
        TEST_ASSERT_FLOAT_EQ(JoltC_BodyInterface_GetGravityFactor(ctx.bodyInterface, id), 2.0f, 0.001f, "gravity factor == 2");

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, id);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_body_user_data */
    TEST_BEGIN("Body user data round-trip");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 5.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        JoltC_BodyInterface_SetUserData(ctx.bodyInterface, id, 0xDEADBEEFULL);
        uint64_t got = JoltC_BodyInterface_GetUserData(ctx.bodyInterface, id);
        TEST_ASSERT(got == 0xDEADBEEFULL, "user data == 0xDEADBEEF");

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, id);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_body_add_remove */
    TEST_BEGIN("Body add/remove lifecycle");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_Vec3 half = { 0.5f, 0.5f, 0.5f };
        const JoltC_Shape* shape = JoltC_BoxShape_Create(half, 0.0f);
        JoltC_RVec3 pos = { 0.0f, 0.0f, 0.0f };
        JoltC_Quat rot = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_BodyCreationSettings* s = JoltC_BodyCreationSettings_Create3(
            shape, pos, rot, JOLTC_MOTION_TYPE_DYNAMIC, OBJ_LAYER_DYNAMIC);

        JoltC_BodyID id = JoltC_BodyInterface_CreateBody(ctx.bodyInterface, s);
        TEST_ASSERT(id != JOLTC_BODY_ID_INVALID, "Created but not added");
        TEST_ASSERT(!JoltC_BodyInterface_IsAdded(ctx.bodyInterface, id), "Not added yet");

        JoltC_BodyInterface_AddBody(ctx.bodyInterface, id, JOLTC_ACTIVATION_ACTIVATE);
        TEST_ASSERT(JoltC_BodyInterface_IsAdded(ctx.bodyInterface, id), "Now added");

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, id);
        TEST_ASSERT(!JoltC_BodyInterface_IsAdded(ctx.bodyInterface, id), "Removed");

        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, id);
        JoltC_BodyCreationSettings_Destroy(s);
        JoltC_Shape_Release(shape);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_body_add_force */
    TEST_BEGIN("Body AddForce no crash");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 10.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        JoltC_Vec3 force = { 100.0f, 0.0f, 0.0f };
        JoltC_BodyInterface_AddForce(ctx.bodyInterface, id, force);
        TEST_ASSERT(1, "No crash");

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, id);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_body_add_impulse */
    TEST_BEGIN("Body AddImpulse changes velocity");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 10.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        JoltC_Vec3 impulse = { 10.0f, 0.0f, 0.0f };
        JoltC_BodyInterface_AddImpulse(ctx.bodyInterface, id, impulse);
        JoltC_Vec3 vel = JoltC_BodyInterface_GetLinearVelocity(ctx.bodyInterface, id);
        TEST_ASSERT(vel.x > 0.0f, "velocity.x > 0 after impulse");

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, id);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_body_sensor */
    TEST_BEGIN("Body sensor flag");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 5.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        JoltC_BodyInterface_SetIsSensor(ctx.bodyInterface, id, 1);
        TEST_ASSERT(JoltC_BodyInterface_IsSensor(ctx.bodyInterface, id), "Is sensor");

        JoltC_BodyInterface_SetIsSensor(ctx.bodyInterface, id, 0);
        TEST_ASSERT(!JoltC_BodyInterface_IsSensor(ctx.bodyInterface, id), "Not sensor");

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, id);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_body_shape_change */
    TEST_BEGIN("Body SetShape to new shape");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 5.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        const JoltC_Shape* newShape = JoltC_SphereShape_Create(1.0f);
        JoltC_BodyInterface_SetShape(ctx.bodyInterface, id, newShape, JOLTC_TRUE, JOLTC_ACTIVATION_ACTIVATE);
        const JoltC_Shape* got = JoltC_BodyInterface_GetShape(ctx.bodyInterface, id);
        TEST_ASSERT(JoltC_Shape_GetSubType(got) == JOLTC_SHAPE_SUB_TYPE_SPHERE, "Shape changed to sphere");

        JoltC_Shape_Release(newShape);
        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, id);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_body_world_transform */
    TEST_BEGIN("Body GetWorldTransform");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 0.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        JoltC_Mat44 m = JoltC_BodyInterface_GetWorldTransform(ctx.bodyInterface, id);
        /* Diagonal should be ~1 (identity rotation, no scale) */
        TEST_ASSERT_FLOAT_EQ(m.m[0], 1.0f, 0.01f, "m[0] ~ 1");
        TEST_ASSERT_FLOAT_EQ(m.m[5], 1.0f, 0.01f, "m[5] ~ 1");
        TEST_ASSERT_FLOAT_EQ(m.m[10], 1.0f, 0.01f, "m[10] ~ 1");

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, id);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_body_damping */
    TEST_BEGIN("Body linear/angular damping");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 5.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        JoltC_BodyInterface_SetLinearDamping(ctx.bodyInterface, id, 0.1f);
        TEST_ASSERT_FLOAT_EQ(JoltC_BodyInterface_GetLinearDamping(ctx.bodyInterface, id), 0.1f, 0.001f, "linear damping");
        JoltC_BodyInterface_SetAngularDamping(ctx.bodyInterface, id, 0.2f);
        TEST_ASSERT_FLOAT_EQ(JoltC_BodyInterface_GetAngularDamping(ctx.bodyInterface, id), 0.2f, 0.001f, "angular damping");

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, id);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}
