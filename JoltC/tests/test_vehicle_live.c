/* JoltC Test Suite — a live vehicle: create, register, step, read, unregister, destroy.
 * SPDX-License-Identifier: MIT
 *
 * This is the suite test_vehicle_extra.c deliberately deferred: everything there is a
 * settings round-trip, and until now nothing in the repository had ever created a
 * VehicleConstraint bound to a body, let alone registered one. That gap is how three
 * incompatible-handle faults shipped: JoltC_VehicleConstraint* is the raw JPH object
 * while JoltC_Constraint* is a counted wrapper, so passing a vehicle to AddConstraint,
 * AddStepListener (via AsPhysicsStepListener) or Constraint_Destroy read a vtable
 * pointer as a smart pointer. The functions under test here are the repairs:
 * VehicleConstraint_AsConstraint, PhysicsSystem_Add/RemoveVehicleStepListener and
 * VehicleConstraint_Destroy.
 *
 * The trajectory assertions are deliberately loose (moved forward at all, stayed above
 * the floor) so a solver or tyre-model change in a future Jolt bump does not read as a
 * binding regression. What must hold exactly: the handle round trips, the constraint
 * count, the wheel count, and ground contact after settling.
 */

#include "test_common.h"

void run_vehicle_live_tests(void);

void run_vehicle_live_tests(void)
{
    /* test_vehicle_register_step_destroy */
    TEST_BEGIN("Vehicle registers, drives and tears down");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        /* A floor wide enough to drive on for two seconds. */
        JoltC_Vec3 floorHalf = { 50.0f, 1.0f, 50.0f };
        const JoltC_Shape* floorShape = JoltC_BoxShape_Create(floorHalf, 0.05f);
        JoltC_RVec3 floorPos = { 0.0f, -1.0f, 0.0f };
        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_BodyCreationSettings* floorSettings = JoltC_BodyCreationSettings_Create3(
            floorShape, floorPos, identity, JOLTC_MOTION_TYPE_STATIC, OBJ_LAYER_STATIC);
        JoltC_BodyID floorId = JoltC_BodyInterface_CreateAndAddBody(
            ctx.bodyInterface, floorSettings, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_BodyCreationSettings_Destroy(floorSettings);
        JoltC_Shape_Release(floorShape);

        /* The chassis: created unadded first, because VehicleConstraint_Create wants the
         * Body pointer, then added through its id like any other body. */
        JoltC_Vec3 chassisHalf = { 0.9f, 0.35f, 2.0f };
        const JoltC_Shape* chassisShape = JoltC_BoxShape_Create(chassisHalf, 0.05f);
        JoltC_RVec3 chassisPos = { 0.0f, 2.0f, 0.0f };
        JoltC_BodyCreationSettings* chassisSettings = JoltC_BodyCreationSettings_Create3(
            chassisShape, chassisPos, identity, JOLTC_MOTION_TYPE_DYNAMIC, OBJ_LAYER_DYNAMIC);
        JoltC_Body* chassisBody = JoltC_BodyInterface_CreateBodyDirect(ctx.bodyInterface, chassisSettings);
        TEST_ASSERT_NOT_NULL(chassisBody, "chassis body created");
        JoltC_BodyID chassisId = JoltC_Body_GetID(chassisBody);
        JoltC_BodyInterface_AddBody(ctx.bodyInterface, chassisId, JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyCreationSettings_Destroy(chassisSettings);
        JoltC_Shape_Release(chassisShape);

        /* Four wheels on the corners, everything else at the WheelSettingsWV defaults. */
        JoltC_WheelSettings* wheels[4];
        JoltC_Vec3 wheelPositions[4] = {
            { -0.9f, -0.3f,  1.4f },
            {  0.9f, -0.3f,  1.4f },
            { -0.9f, -0.3f, -1.4f },
            {  0.9f, -0.3f, -1.4f },
        };
        for (int i = 0; i < 4; i++)
        {
            JoltC_WheelSettingsWV* wv = JoltC_WheelSettingsWV_Create();
            wheels[i] = (JoltC_WheelSettings*)wv;
            JoltC_WheelSettings_SetPosition(wheels[i], wheelPositions[i]);
            JoltC_WheelSettings_SetRadius(wheels[i], 0.35f);
            JoltC_WheelSettings_SetWidth(wheels[i], 0.25f);
        }

        /* One differential driving the front axle. */
        JoltC_WheeledVehicleControllerSettings* controller = JoltC_WheeledVehicleControllerSettings_Create();
        JoltC_VehicleDifferentialSettings differential;
        JoltC_VehicleDifferentialSettings_Init(&differential);
        differential.leftWheel = 0;
        differential.rightWheel = 1;
        JoltC_WheeledVehicleControllerSettings_SetDifferentialsCount(controller, 1);
        JoltC_WheeledVehicleControllerSettings_SetDifferential(controller, 0, &differential);

        JoltC_VehicleConstraintSettings settings;
        JoltC_VehicleConstraintSettings_Init(&settings);
        settings.up = (JoltC_Vec3){ 0.0f, 1.0f, 0.0f };
        settings.forward = (JoltC_Vec3){ 0.0f, 0.0f, 1.0f };
        settings.wheelsCount = 4;
        settings.wheels = wheels;
        settings.controller = (JoltC_VehicleControllerSettings*)controller;

        JoltC_VehicleConstraint* vc = JoltC_VehicleConstraint_Create(chassisBody, &settings);
        TEST_ASSERT_NOT_NULL(vc, "vehicle constraint created");
        TEST_ASSERT(JoltC_VehicleConstraint_GetWheelsCount(vc) == 4, "four wheels");

        /* The constraint holds its own references; the creation ones go back. */
        JoltC_VehicleControllerSettings_Destroy((JoltC_VehicleControllerSettings*)controller);
        for (int i = 0; i < 4; i++)
        {
            JoltC_WheelSettings_Destroy(wheels[i]);
        }

        JoltC_VehicleCollisionTester* tester = JoltC_VehicleCollisionTesterRay_Create(
            OBJ_LAYER_DYNAMIC, (JoltC_Vec3){ 0.0f, 1.0f, 0.0f }, 1.396f /* ~80 degrees */);
        JoltC_VehicleConstraint_SetVehicleCollisionTester(vc, tester);

        /* The repairs under test: the constraint view registers with the system, and the
         * step listener goes in through the vehicle-aware pair. */
        JoltC_Constraint* view = JoltC_VehicleConstraint_AsConstraint(vc);
        TEST_ASSERT_NOT_NULL(view, "AsConstraint returns a view");
        JoltC_PhysicsSystem_AddConstraint(ctx.physicsSystem, view);
        TEST_ASSERT(JoltC_PhysicsSystem_GetNumConstraints(ctx.physicsSystem) == 1, "constraint registered");
        JoltC_PhysicsSystem_AddVehicleStepListener(ctx.physicsSystem, vc);

        JoltC_PhysicsSystem_OptimizeBroadPhase(ctx.physicsSystem);

        /* Two seconds at full throttle. */
        JoltC_WheeledVehicleController* live =
            (JoltC_WheeledVehicleController*)JoltC_VehicleConstraint_GetController(vc);
        TEST_ASSERT_NOT_NULL(live, "controller reachable");

        for (int step = 0; step < 120; step++)
        {
            JoltC_WheeledVehicleController_SetDriverInput(live, 1.0f, 0.0f, 0.0f, 0.0f);
            JoltC_BodyInterface_ActivateBody(ctx.bodyInterface, chassisId);
            JoltC_PhysicsSystem_Update(ctx.physicsSystem, 1.0f / 60.0f, 1, ctx.tempAllocator, ctx.jobSystem);
        }

        JoltC_Wheel* wheel0 = JoltC_VehicleConstraint_GetWheel(vc, 0);
        TEST_ASSERT_NOT_NULL(wheel0, "wheel 0 reachable");
        TEST_ASSERT(JoltC_Wheel_HasContact(wheel0), "front wheel on the ground");

        JoltC_RVec3 endPos = JoltC_BodyInterface_GetCenterOfMassPosition(ctx.bodyInterface, chassisId);
        TEST_ASSERT(endPos.z > 1.0f, "the car drove forward");
        TEST_ASSERT(endPos.y > -0.5f, "the car stayed above the floor");

        /* Teardown in dependency order: listener, registration, view, vehicle, tester. */
        JoltC_PhysicsSystem_RemoveVehicleStepListener(ctx.physicsSystem, vc);
        JoltC_PhysicsSystem_RemoveConstraint(ctx.physicsSystem, view);
        TEST_ASSERT(JoltC_PhysicsSystem_GetNumConstraints(ctx.physicsSystem) == 0, "constraint unregistered");
        JoltC_Constraint_Destroy(view);
        JoltC_VehicleConstraint_Destroy(vc);
        JoltC_VehicleCollisionTester_Destroy(tester);

        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, chassisId);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, chassisId);
        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, floorId);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, floorId);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}
