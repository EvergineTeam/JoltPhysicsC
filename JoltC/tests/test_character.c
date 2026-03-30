/* JoltC Test Suite — character.h API tests (CharacterVirtual + Character)
 * SPDX-License-Identifier: MIT
 */

#include "test_common.h"

void run_character_tests(void)
{
    /* test_character_virtual_create */
    TEST_BEGIN("CharacterVirtual create/destroy");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        /* Create a capsule shape for the character */
        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        TEST_ASSERT_NOT_NULL(capsule, "capsule shape not null");

        JoltC_CharacterVirtualSettings settings;
        JoltC_CharacterVirtualSettings_SetDefault(&settings);
        settings.shape = capsule;

        JoltC_RVec3 pos = { 0.0f, 2.0f, 0.0f };
        JoltC_Quat rot = { 0.0f, 0.0f, 0.0f, 1.0f };

        JoltC_CharacterVirtual* cv = JoltC_CharacterVirtual_Create(&settings, pos, rot, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(cv, "CharacterVirtual not null");

        JoltC_CharacterVirtual_Destroy(cv);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_character_virtual_position */
    TEST_BEGIN("CharacterVirtual set/get position");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        JoltC_CharacterVirtualSettings settings;
        JoltC_CharacterVirtualSettings_SetDefault(&settings);
        settings.shape = capsule;

        JoltC_RVec3 pos = { 1.0f, 2.0f, 3.0f };
        JoltC_Quat rot = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_CharacterVirtual* cv = JoltC_CharacterVirtual_Create(&settings, pos, rot, 0, ctx.physicsSystem);

        JoltC_RVec3 got = JoltC_CharacterVirtual_GetPosition(cv);
        TEST_ASSERT_FLOAT_EQ(got.x, 1.0f, 0.01f, "pos.x == 1");
        TEST_ASSERT_FLOAT_EQ(got.y, 2.0f, 0.01f, "pos.y == 2");
        TEST_ASSERT_FLOAT_EQ(got.z, 3.0f, 0.01f, "pos.z == 3");

        JoltC_RVec3 newPos = { 4.0f, 5.0f, 6.0f };
        JoltC_CharacterVirtual_SetPosition(cv, newPos);
        got = JoltC_CharacterVirtual_GetPosition(cv);
        TEST_ASSERT_FLOAT_EQ(got.x, 4.0f, 0.01f, "pos.x == 4 after set");

        JoltC_CharacterVirtual_Destroy(cv);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_character_virtual_velocity */
    TEST_BEGIN("CharacterVirtual set/get linear velocity");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        JoltC_CharacterVirtualSettings settings;
        JoltC_CharacterVirtualSettings_SetDefault(&settings);
        settings.shape = capsule;

        JoltC_RVec3 pos = { 0.0f, 2.0f, 0.0f };
        JoltC_Quat rot = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_CharacterVirtual* cv = JoltC_CharacterVirtual_Create(&settings, pos, rot, 0, ctx.physicsSystem);

        JoltC_Vec3 vel = { 1.0f, 0.0f, -2.0f };
        JoltC_CharacterVirtual_SetLinearVelocity(cv, vel);
        JoltC_Vec3 got = JoltC_CharacterVirtual_GetLinearVelocity(cv);
        TEST_ASSERT_FLOAT_EQ(got.x, 1.0f, 0.01f, "vel.x == 1");
        TEST_ASSERT_FLOAT_EQ(got.z, -2.0f, 0.01f, "vel.z == -2");

        JoltC_CharacterVirtual_Destroy(cv);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_character_virtual_ground_state */
    TEST_BEGIN("CharacterVirtual ground state initially InAir");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        JoltC_CharacterVirtualSettings settings;
        JoltC_CharacterVirtualSettings_SetDefault(&settings);
        settings.shape = capsule;

        JoltC_RVec3 pos = { 0.0f, 10.0f, 0.0f };
        JoltC_Quat rot = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_CharacterVirtual* cv = JoltC_CharacterVirtual_Create(&settings, pos, rot, 0, ctx.physicsSystem);

        JoltC_GroundState gs = JoltC_CharacterVirtual_GetGroundState(cv);
        TEST_ASSERT(gs == JOLTC_GROUND_STATE_NOT_SUPPORTED || gs == JOLTC_GROUND_STATE_IN_AIR,
                    "Ground state should be InAir or NotSupported initially");

        JoltC_CharacterVirtual_Destroy(cv);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_character_virtual_user_data */
    TEST_BEGIN("CharacterVirtual user data");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        JoltC_CharacterVirtualSettings settings;
        JoltC_CharacterVirtualSettings_SetDefault(&settings);
        settings.shape = capsule;

        JoltC_RVec3 pos = { 0.0f, 2.0f, 0.0f };
        JoltC_Quat rot = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_CharacterVirtual* cv = JoltC_CharacterVirtual_Create(&settings, pos, rot, 0xBEEF, ctx.physicsSystem);
        TEST_ASSERT(JoltC_CharacterVirtual_GetUserData(cv) == 0xBEEF, "user data initial");

        JoltC_CharacterVirtual_SetUserData(cv, 0xCAFE);
        TEST_ASSERT(JoltC_CharacterVirtual_GetUserData(cv) == 0xCAFE, "user data after set");

        JoltC_CharacterVirtual_Destroy(cv);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_character_virtual_extended_update */
    TEST_BEGIN("CharacterVirtual ExtendedUpdate no crash");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        JoltC_CharacterVirtualSettings settings;
        JoltC_CharacterVirtualSettings_SetDefault(&settings);
        settings.shape = capsule;

        JoltC_RVec3 pos = { 0.0f, 2.0f, 0.0f };
        JoltC_Quat rot = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_CharacterVirtual* cv = JoltC_CharacterVirtual_Create(&settings, pos, rot, 0, ctx.physicsSystem);

        JoltC_ExtendedUpdateSettings eus;
        JoltC_ExtendedUpdateSettings_SetDefault(&eus);

        JoltC_Vec3 gravity = { 0.0f, -9.81f, 0.0f };
        JoltC_CharacterVirtual_ExtendedUpdate(cv, 1.0f / 60.0f, gravity, &eus, ctx.tempAllocator);
        TEST_ASSERT(1, "ExtendedUpdate did not crash");

        JoltC_CharacterVirtual_Destroy(cv);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}
