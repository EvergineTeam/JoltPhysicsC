/* JoltC Test Suite -- phase 4: constraints and vehicles to one hundred percent.
 * SPDX-License-Identifier: MIT
 *
 * Before this phase the PathConstraint existed only as an enum value, the pulley could be created
 * and never adjusted, rack/pinion and gear settings had no Init and no ratio arithmetic, and a
 * vehicle ran with no way to hear its own step or read what its tires were doing. Everything here
 * drives the new surface in a live simulation where it has behaviour to show.
 */

#include "test_common.h"

#include <math.h>

/* Slip telemetry sampled from inside the vehicle's own post-step callback, which is the moment
 * the numbers are fresh. */
typedef struct VehicleTelemetry {
    int   preSteps;
    int   postCollides;
    int   postSteps;
    float maxCombinedLongitudinalFriction;
    float maxBrakeImpulse;
} VehicleTelemetry;

static void vehicle_pre_step(void* userData, JoltC_VehicleConstraint* vehicle, float deltaTime, JoltC_Bool isFirstStep, JoltC_Bool isLastStep)
{
    (void)vehicle; (void)deltaTime; (void)isFirstStep; (void)isLastStep;
    ((VehicleTelemetry*)userData)->preSteps++;
}

static void vehicle_post_collide(void* userData, JoltC_VehicleConstraint* vehicle, float deltaTime, JoltC_Bool isFirstStep, JoltC_Bool isLastStep)
{
    (void)vehicle; (void)deltaTime; (void)isFirstStep; (void)isLastStep;
    ((VehicleTelemetry*)userData)->postCollides++;
}

static void vehicle_post_step(void* userData, JoltC_VehicleConstraint* vehicle, float deltaTime, JoltC_Bool isFirstStep, JoltC_Bool isLastStep)
{
    (void)deltaTime; (void)isFirstStep; (void)isLastStep;
    VehicleTelemetry* telemetry = (VehicleTelemetry*)userData;
    telemetry->postSteps++;

    for (uint32_t i = 0; i < JoltC_VehicleConstraint_GetWheelsCount(vehicle); i++)
    {
        const JoltC_WheelWV* wheel = (const JoltC_WheelWV*)JoltC_VehicleConstraint_GetWheel(vehicle, i);
        float friction = JoltC_WheelWV_GetCombinedLongitudinalFriction(wheel);
        float brake = JoltC_WheelWV_GetBrakeImpulse(wheel);
        if (friction > telemetry->maxCombinedLongitudinalFriction) telemetry->maxCombinedLongitudinalFriction = friction;
        if (brake > telemetry->maxBrakeImpulse) telemetry->maxBrakeImpulse = brake;
    }
}

void run_phase4_tests(void);

void run_phase4_tests(void)
{
    /* test_path_constraint_end_to_end */
    TEST_BEGIN("A body rides a Hermite path driven by its motor");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        /* A straight path is the honest first path: the geometry is exactly predictable, so the
         * assertions are about the constraint, not about spline aesthetics. */
        JoltC_PathConstraintPath* path = JoltC_PathConstraintPathHermite_Create();
        TEST_ASSERT_NOT_NULL(path, "hermite path created");
        for (int i = 0; i < 4; i++)
            JoltC_PathConstraintPathHermite_AddPoint(path,
                (JoltC_Vec3){ 2.0f * i, 0.0f, 0.0f },
                (JoltC_Vec3){ 1.0f, 0.0f, 0.0f },
                (JoltC_Vec3){ 0.0f, 1.0f, 0.0f });

        TEST_ASSERT_FLOAT_EQ(JoltC_PathConstraintPath_GetPathMaxFraction(path), 3.0f, 1.0e-3f,
                             "four points make three segments");

        JoltC_Vec3 onPath, tangent, normal, binormal;
        JoltC_PathConstraintPath_GetPointOnPath(path, 1.0f, &onPath, &tangent, &normal, &binormal);
        TEST_ASSERT_FLOAT_EQ(onPath.x, 2.0f, 0.01f, "fraction one sits on the second point");
        TEST_ASSERT_FLOAT_EQ(tangent.x, 1.0f, 0.01f, "the tangent points along the path");

        float closest = JoltC_PathConstraintPath_GetClosestPoint(path, (JoltC_Vec3){ 6.0f, 0.0f, 0.0f }, 0.0f);
        TEST_ASSERT_FLOAT_EQ(closest, 3.0f, 0.05f, "the end of the path is closest to its endpoint");

        /* Anchor and rider. */
        JoltC_RVec3 anchorPos = { 0.0f, 5.0f, 0.0f };
        JoltC_BodyID anchor = create_test_box_body(&ctx, anchorPos, JOLTC_MOTION_TYPE_STATIC, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_RVec3 riderPos = { 0.0f, 5.0f, 0.0f };
        JoltC_BodyID rider = create_test_box_body(&ctx, riderPos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);
        (void)anchor;

        JoltC_PathConstraintSettings settings;
        JoltC_PathConstraintSettings_Init(&settings);
        settings.path = path;
        settings.rotationConstraintType = JOLTC_PATH_ROTATION_CONSTRAINT_TYPE_CONSTRAIN_TO_PATH;

        JoltC_Constraint* constraint = JoltC_PathConstraint_Create(ctx.physicsSystem, anchor, rider, &settings);
        TEST_ASSERT_NOT_NULL(constraint, "path constraint created, no longer just an enum value");

        /* Creation does not register: a constraint solves nothing until it is added. */
        JoltC_PhysicsSystem_AddConstraint(ctx.physicsSystem, constraint);

        /* Drive the rider to the far end of the path. */
        JoltC_PathConstraint_SetPositionMotorState(constraint, JOLTC_MOTOR_STATE_POSITION);
        JoltC_PathConstraint_SetTargetPathFraction(constraint, 3.0f);

        for (int i = 0; i < 240; i++)
            JoltC_PhysicsSystem_Update(ctx.physicsSystem, 1.0f / 60.0f, 1, ctx.tempAllocator, ctx.jobSystem);

        JoltC_RVec3 arrived = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, rider);
        TEST_ASSERT(arrived.x > 4.5f, "the motor carried the rider down the path");
        TEST_ASSERT(fabsf(arrived.y - 5.0f) < 0.25f, "gravity could not pull it off the curve");
        TEST_ASSERT(JoltC_PathConstraint_GetPathFraction(constraint) > 2.5f, "the fraction reports the journey");

        /* Upstream honesty: Jolt 5.6 has not implemented PathConstraint::GetConstraintSettings
         * (it is an explicit "not implemented yet" returning null), so null is the correct answer
         * here, not a wrapper defect. The CreateSettings direction works and is what ragdoll-style
         * construction needs. */
        TEST_ASSERT(JoltC_PathConstraint_GetSettings(constraint) == NULL,
                    "GetSettings reports Jolt's own unimplemented state truthfully");
        JoltC_TwoBodyConstraintSettings* rebuilt = JoltC_PathConstraintSettings_CreateSettings(&settings);
        TEST_ASSERT_NOT_NULL(rebuilt, "CreateSettings produces a real settings object");
        JoltC_TwoBodyConstraintSettings_Release(rebuilt);

        JoltC_PhysicsSystem_RemoveConstraint(ctx.physicsSystem, constraint);
        JoltC_Constraint_Destroy(constraint);
        JoltC_PathConstraintPath_Release(path);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_pulley_hoists_at_runtime */
    TEST_BEGIN("Shortening a pulley's rope hoists its load");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_RVec3 leftPos = { -2.0f, 4.0f, 0.0f };
        JoltC_RVec3 rightPos = { 2.0f, 4.0f, 0.0f };
        JoltC_BodyID left = create_test_box_body(&ctx, leftPos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyID right = create_test_box_body(&ctx, rightPos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        JoltC_PulleyConstraintSettings settings;
        JoltC_PulleyConstraintSettings_Init(&settings);
        settings.bodyPoint1 = (JoltC_RVec3){ -2.0f, 4.5f, 0.0f };
        settings.fixedPoint1 = (JoltC_RVec3){ -2.0f, 8.0f, 0.0f };
        settings.bodyPoint2 = (JoltC_RVec3){ 2.0f, 4.5f, 0.0f };
        settings.fixedPoint2 = (JoltC_RVec3){ 2.0f, 8.0f, 0.0f };

        JoltC_Constraint* pulley = JoltC_PulleyConstraint_Create(ctx.physicsSystem, left, right, &settings);
        TEST_ASSERT_NOT_NULL(pulley, "pulley created");
        TEST_ASSERT_NOT_NULL(JoltC_PulleyConstraint_GetSettings(pulley), "pulley GetSettings is real");
        JoltC_PhysicsSystem_AddConstraint(ctx.physicsSystem, pulley);

        /* Let it hang taut, then wind in a metre and a half of rope. */
        for (int i = 0; i < 60; i++)
            JoltC_PhysicsSystem_Update(ctx.physicsSystem, 1.0f / 60.0f, 1, ctx.tempAllocator, ctx.jobSystem);

        float slack = JoltC_PulleyConstraint_GetMaxLength(pulley);
        TEST_ASSERT(slack > 0.0f, "creation measured the rope from the initial pose");
        JoltC_RVec3 hanging = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, left);

        JoltC_PulleyConstraint_SetLength(pulley, 0.0f, slack - 1.5f);
        TEST_ASSERT_FLOAT_EQ(JoltC_PulleyConstraint_GetMaxLength(pulley), slack - 1.5f, 1.0e-4f,
                             "the new length reads back");

        for (int i = 0; i < 120; i++)
        {
            JoltC_BodyInterface_ActivateBody(ctx.bodyInterface, left);
            JoltC_BodyInterface_ActivateBody(ctx.bodyInterface, right);
            JoltC_PhysicsSystem_Update(ctx.physicsSystem, 1.0f / 60.0f, 1, ctx.tempAllocator, ctx.jobSystem);
        }

        JoltC_RVec3 hoisted = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, left);
        TEST_ASSERT(hoisted.y > hanging.y + 0.4f, "winding in rope lifted the load");
        TEST_ASSERT(JoltC_PulleyConstraint_GetCurrentLength(pulley) < slack - 1.0f,
                    "the rope in play really is shorter");

        JoltC_PhysicsSystem_RemoveConstraint(ctx.physicsSystem, pulley);
        JoltC_Constraint_Destroy(pulley);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_ratio_arithmetic_and_inits */
    TEST_BEGIN("Gear and rack ratios come from what they are made of");
    {
        JoltC_GearConstraintSettings gear;
        JoltC_GearConstraintSettings_Init(&gear);
        JoltC_GearConstraintSettings_SetRatio(&gear, 30, 10);
        /* Jolt defines it as Gear1Rotation = -ratio * Gear2Rotation, so ratio = teeth2 / teeth1:
         * the thirty-tooth gear turns a third as fast as the ten-tooth one driving it. */
        TEST_ASSERT_FLOAT_EQ(gear.ratio, 10.0f / 30.0f, 1.0e-4f, "thirty against ten gears down to a third");

        JoltC_RackAndPinionConstraintSettings rack;
        JoltC_RackAndPinionConstraintSettings_Init(&rack);
        TEST_ASSERT(rack.ratio > 0.0f, "rack settings initialise to Jolt defaults");
        JoltC_RackAndPinionConstraintSettings_SetRatio(&rack, 20, 1.0f, 10);
        TEST_ASSERT_FLOAT_EQ(rack.ratio, 2.0f * 3.14159265f * 20.0f / (1.0f * 10.0f), 1.0e-2f,
                             "the rack ratio folds teeth and length together");
    }
    TEST_END();

    /* test_hinge_body_space_target */
    TEST_BEGIN("A hinge takes its motor target in body space");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_RVec3 topPos = { 0.0f, 5.0f, 0.0f };
        JoltC_RVec3 bottomPos = { 0.0f, 4.0f, 0.0f };
        JoltC_BodyID top = create_test_box_body(&ctx, topPos, JOLTC_MOTION_TYPE_STATIC, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_BodyID bottom = create_test_box_body(&ctx, bottomPos, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        JoltC_HingeConstraintSettings settings;
        JoltC_HingeConstraintSettings_Init(&settings);
        settings.point1 = (JoltC_RVec3){ 0.0f, 4.5f, 0.0f };
        settings.point2 = (JoltC_RVec3){ 0.0f, 4.5f, 0.0f };
        settings.hingeAxis1 = (JoltC_Vec3){ 0.0f, 0.0f, 1.0f };
        settings.hingeAxis2 = (JoltC_Vec3){ 0.0f, 0.0f, 1.0f };
        settings.normalAxis1 = (JoltC_Vec3){ 1.0f, 0.0f, 0.0f };
        settings.normalAxis2 = (JoltC_Vec3){ 1.0f, 0.0f, 0.0f };

        JoltC_Constraint* hinge = JoltC_HingeConstraint_Create(ctx.physicsSystem, top, bottom, &settings);
        TEST_ASSERT_NOT_NULL(hinge, "hinge created");

        /* Half a radian around the hinge axis, handed over as a body-space orientation: the
         * constraint does the frame math and lands it in the target angle. */
        float half = 0.25f; /* sin/cos of half the angle */
        JoltC_Quat target = { 0.0f, 0.0f, (float)sin(half), (float)cos(half) };
        JoltC_HingeConstraint_SetTargetOrientationBS(hinge, target);

        float angle = JoltC_HingeConstraint_GetTargetAngle(hinge);
        TEST_ASSERT_FLOAT_EQ(fabsf(angle), 0.5f, 0.01f, "the body space target became the hinge angle");

        JoltC_Constraint_Destroy(hinge);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_vehicle_callbacks_and_telemetry */
    TEST_BEGIN("A vehicle reports its steps and its tires");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

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

        /* Grippy tarmac: the combined tire friction folds the surface in (roughly the geometric
         * mean), and the default body friction of 0.2 would drag the telemetry down with it. */
        JoltC_BodyInterface_SetFriction(ctx.bodyInterface, floorId, 1.0f);

        JoltC_Vec3 chassisHalf = { 0.9f, 0.35f, 2.0f };
        const JoltC_Shape* chassisShape = JoltC_BoxShape_Create(chassisHalf, 0.05f);
        JoltC_RVec3 chassisPos = { 0.0f, 2.0f, 0.0f };
        JoltC_BodyCreationSettings* chassisSettings = JoltC_BodyCreationSettings_Create3(
            chassisShape, chassisPos, identity, JOLTC_MOTION_TYPE_DYNAMIC, OBJ_LAYER_DYNAMIC);
        JoltC_Body* chassisBody = JoltC_BodyInterface_CreateBodyDirect(ctx.bodyInterface, chassisSettings);
        JoltC_BodyID chassisId = JoltC_Body_GetID(chassisBody);
        JoltC_BodyInterface_AddBody(ctx.bodyInterface, chassisId, JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyCreationSettings_Destroy(chassisSettings);
        JoltC_Shape_Release(chassisShape);

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
            JoltC_WheelSettingsWV_SetMaxBrakeTorque(wv, 1500.0f);
        }

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
        TEST_ASSERT_NOT_NULL(vc, "vehicle created");
        JoltC_VehicleControllerSettings_Destroy((JoltC_VehicleControllerSettings*)controller);
        for (int i = 0; i < 4; i++)
            JoltC_WheelSettings_Destroy(wheels[i]);

        JoltC_VehicleCollisionTester* tester = JoltC_VehicleCollisionTesterRay_Create(
            OBJ_LAYER_DYNAMIC, (JoltC_Vec3){ 0.0f, 1.0f, 0.0f }, 1.396f);
        JoltC_VehicleConstraint_SetVehicleCollisionTester(vc, tester);
        TEST_ASSERT(JoltC_VehicleConstraint_GetVehicleCollisionTester(vc) == tester,
                    "the tester read back is the very handle that went in");

        JoltC_VehicleConstraint_SetMaxPitchRollAngle(vc, 1.0f);
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleConstraint_GetMaxPitchRollAngle(vc), 1.0f, 1.0e-3f,
                             "max pitch/roll round trips");

        JoltC_VehicleConstraint_SetNumStepsBetweenCollisionTestInactive(vc, 6);
        TEST_ASSERT(JoltC_VehicleConstraint_GetNumStepsBetweenCollisionTestInactive(vc) == 6,
                    "collision cadence round trips");

        VehicleTelemetry telemetry = { 0, 0, 0, 0.0f, 0.0f };
        JoltC_VehicleConstraint_SetPreStepCallback(vc, vehicle_pre_step, &telemetry);
        JoltC_VehicleConstraint_SetPostCollideCallback(vc, vehicle_post_collide, &telemetry);
        JoltC_VehicleConstraint_SetPostStepCallback(vc, vehicle_post_step, &telemetry);

        JoltC_Constraint* view = JoltC_VehicleConstraint_AsConstraint(vc);
        JoltC_PhysicsSystem_AddConstraint(ctx.physicsSystem, view);
        JoltC_PhysicsSystem_AddVehicleStepListener(ctx.physicsSystem, vc);
        JoltC_PhysicsSystem_OptimizeBroadPhase(ctx.physicsSystem);

        JoltC_WheeledVehicleController* live =
            (JoltC_WheeledVehicleController*)JoltC_VehicleConstraint_GetController(vc);

        TEST_ASSERT(JoltC_WheeledVehicleController_GetDifferentialsCount(live) == 1,
                    "the live controller reports its differential");
        JoltC_WheeledVehicleController_SetDifferentialLimitedSlipRatio(live, 1.6f);
        TEST_ASSERT_FLOAT_EQ(JoltC_WheeledVehicleController_GetDifferentialLimitedSlipRatio(live), 1.6f, 1.0e-4f,
                             "limited slip ratio round trips on the live controller");

        /* Ninety steps of throttle, thirty of hard braking: both ends of the telemetry. */
        for (int step = 0; step < 120; step++)
        {
            if (step < 90)
                JoltC_WheeledVehicleController_SetDriverInput(live, 1.0f, 0.0f, 0.0f, 0.0f);
            else
                JoltC_WheeledVehicleController_SetDriverInput(live, 0.0f, 0.0f, 1.0f, 0.0f);
            JoltC_BodyInterface_ActivateBody(ctx.bodyInterface, chassisId);
            JoltC_PhysicsSystem_Update(ctx.physicsSystem, 1.0f / 60.0f, 1, ctx.tempAllocator, ctx.jobSystem);
        }

        TEST_ASSERT(telemetry.preSteps > 0, "the pre-step callback fired");
        TEST_ASSERT(telemetry.preSteps == telemetry.postCollides && telemetry.postCollides == telemetry.postSteps,
                    "each step passes through all three moments exactly once");
        TEST_ASSERT(telemetry.maxCombinedLongitudinalFriction > 0.5f,
                    "the tires reported real friction against the floor");
        TEST_ASSERT(telemetry.maxBrakeImpulse > 0.0f,
                    "braking showed up in the brake impulse, which is what skid effects key on");

        JoltC_RVec3 endPos = JoltC_BodyInterface_GetCenterOfMassPosition(ctx.bodyInterface, chassisId);
        TEST_ASSERT(endPos.z > 1.0f, "it drove");

        /* Clearing a callback really clears it. */
        JoltC_VehicleConstraint_SetPostStepCallback(vc, NULL, NULL);
        int postStepsBefore = telemetry.postSteps;
        JoltC_BodyInterface_ActivateBody(ctx.bodyInterface, chassisId);
        JoltC_PhysicsSystem_Update(ctx.physicsSystem, 1.0f / 60.0f, 1, ctx.tempAllocator, ctx.jobSystem);
        TEST_ASSERT(telemetry.postSteps == postStepsBefore, "a cleared callback stays silent");

        JoltC_PhysicsSystem_RemoveVehicleStepListener(ctx.physicsSystem, vc);
        JoltC_PhysicsSystem_RemoveConstraint(ctx.physicsSystem, view);
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
