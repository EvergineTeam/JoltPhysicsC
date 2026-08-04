/* JoltC Test Suite — vehicle.h, second suite (settings round-trips, macro-generated
 * accessor blocks, null safety, structural checks)
 * SPDX-License-Identifier: MIT
 *
 * Why this file exists, and what it deliberately does NOT do
 * ----------------------------------------------------------
 * vehicle.h is the largest header in the wrapper (233 exported functions) and the
 * least covered in proportion: test_vehicle.c exercises 33 of them. Ahead of the
 * JoltPhysics 5.5.0 -> 5.6.0 bump the failure that costs the most is one that
 * compiles but is semantically wrong: a swapped assignment, a getter reading its
 * neighbour's member, an argument pair exchanged.
 *
 * That risk is concentrated in vehicle.cpp's token-pasting macros. WS_VEC3_PROP,
 * WS_FLOAT_PROP, WSWV_FLOAT, MCS_FLOAT and TS_FLOAT each expand a (Name, Field)
 * pair into a getter and a setter, so a single wrong member name silently breaks
 * both halves of one property while leaving the rest of the block correct. A
 * per-property round-trip cannot see that — reading back what you just wrote to the
 * wrong member still returns the right value. What sees it is writing every property
 * in a block to a DISTINCT value and only then reading them all back: if two entries
 * of a macro block name the same member, the second write destroys the first and the
 * read-back fails. Every macro block below is tested that way.
 *
 * All values are asymmetric on purpose. Same-typed neighbours are kept far apart
 * (radius 0.35 vs width 0.2; spring frequency 2.25 vs damping 0.375; engine minRPM
 * 850.25 vs maxRPM 5500.75) so an exchange cannot pass by coincidence.
 *
 * What is intentionally NOT here: anything that steps the simulation. 5.6.0 changes
 * the tyre friction model and solver behaviour, so an assertion about a vehicle
 * position, a wheel angle or a speed after stepping would fail legitimately on the
 * bump and read as a regression. The only runtime objects used are standalone Wheel
 * instances, which are pure data holders until a VehicleConstraint solves them.
 *
 * Deliberately left to a future suite (they need a live VehicleConstraint bound to a
 * Body, which is a much larger and crash-prone fixture than a settings round-trip):
 * VehicleConstraint, WheeledVehicleController, MotorcycleController,
 * TrackedVehicleController, VehicleEngine, VehicleTransmission and VehicleTrack
 * runtime accessors.
 *
 * test_vehicle.c already covers, and this file does not repeat: LinearCurve
 * create/add/sort/min/max/value/count, WheelSettings radius+width, WheelSettingsWV
 * steer angle + brake torque, WheeledVehicleControllerSettings engine get/set,
 * TrackedVehicleControllerSettings creation, transmission mode + gear count, and
 * VehicleCollisionTesterRay create + GetObjectLayer.
 */

#include "test_common.h"

void run_vehicle_extra_tests(void);

/* ========================================================================== */
/*  Local helpers                                                             */
/* ========================================================================== */

/* Upstream defaults this suite pins, from JoltPhysics 5.5.0 headers:
 *   WheelSettings      suspension spring { FrequencyAndDamping, 1.5, 0.5 },
 *                      radius 0.3, width 0.1, min/max suspension length 0.3/0.5
 *   WheelSettingsWV    inertia 0.9, angular damping 0.2, max steer 70 deg (radians),
 *                      brake 1500, hand brake 4000,
 *                      longitudinal friction curve X in [0, 0.2] (slip ratio),
 *                      lateral friction curve X in [0, 20] (slip angle in DEGREES)
 *   WheelSettingsTV    longitudinal friction 4.0, lateral friction 2.0
 *   VehicleEngineSettings        torque 500, RPM 1000..6000, inertia 0.5, damping 0.2
 *   VehicleDifferentialSettings  wheels -1/-1, ratio 3.42, split 0.5, LSR 1.4, torque 1
 *   VehicleAntiRollBar           left wheel 0, right wheel 1, stiffness 1000
 *   VehicleTrackSettings         inertia 10, damping 0.5, brake 15000, ratio 6
 *   VehicleTransmissionSettings  5 gears, 1 reverse gear, switch 0.5, clutch release
 *                      0.3, latency 0.5, shift up 4000, shift down 2000, clutch 10
 *   MotorcycleControllerSettings max lean 45 deg (radians), spring constant 5000,
 *                      damping 1000, integration coefficient 0, decay 4, smoothing 0.8
 * TrackedVehicleControllerSettings overrides the engine and transmission (see below),
 * which is what lets this suite prove the two Create functions build distinct types.
 */
#define WS_DEFAULT_RADIUS          0.3f
#define WSWV_DEFAULT_LONG_MAX_X    0.2f    /* slip ratio, dimensionless */
#define WSWV_DEFAULT_LAT_MAX_X    20.0f    /* slip angle, degrees */

/* A wheel whose settings are handed to Jolt must satisfy the asserts in
 * Wheel::Wheel: the four direction vectors normalized, min <= max suspension
 * length, preload >= 0, spring frequency > 0, radius > 0, width >= 0. These four
 * are normalized exactly in float (0.6^2 + 0.8^2 == 1) and mutually distinct, so a
 * swap between them is still visible. */
static const JoltC_Vec3 k_suspensionDirection = { 0.6f, -0.8f,  0.0f };
static const JoltC_Vec3 k_steeringAxis        = { 0.0f,  0.8f,  0.6f };
static const JoltC_Vec3 k_wheelUp             = { -0.8f, 0.6f,  0.0f };
static const JoltC_Vec3 k_wheelForward        = { 0.0f, -0.6f,  0.8f };

/* Positions are free of any normalization constraint, so they get wild values. */
static const JoltC_Vec3 k_position            = { 1.5f, -2.25f, 3.75f };
static const JoltC_Vec3 k_forcePoint          = { -4.5f, 5.25f, -6.75f };

static int vec3_equals(JoltC_Vec3 a, JoltC_Vec3 b, float eps)
{
    return fabsf(a.x - b.x) < eps && fabsf(a.y - b.y) < eps && fabsf(a.z - b.z) < eps;
}

static int vec3_is_zero(JoltC_Vec3 v)
{
    return v.x == 0.0f && v.y == 0.0f && v.z == 0.0f;
}

/* Fills a WheelSettings with the asymmetric-but-Jolt-valid configuration above.
 * Used both by the round-trip test and as the source for real Wheel instances. */
static void configure_wheel_settings(JoltC_WheelSettings* ws)
{
    JoltC_SpringSettings spring;
    spring.mode = JOLTC_SPRING_MODE_FREQUENCY_AND_DAMPING;
    spring.frequencyOrStiffness = 2.25f;
    spring.damping = 0.375f;

    JoltC_WheelSettings_SetPosition(ws, k_position);
    JoltC_WheelSettings_SetSuspensionForcePoint(ws, k_forcePoint);
    JoltC_WheelSettings_SetSuspensionDirection(ws, k_suspensionDirection);
    JoltC_WheelSettings_SetSteeringAxis(ws, k_steeringAxis);
    JoltC_WheelSettings_SetWheelUp(ws, k_wheelUp);
    JoltC_WheelSettings_SetWheelForward(ws, k_wheelForward);
    JoltC_WheelSettings_SetSuspensionMinLength(ws, 0.125f);
    JoltC_WheelSettings_SetSuspensionMaxLength(ws, 0.875f);
    JoltC_WheelSettings_SetSuspensionPreloadLength(ws, 0.0625f);
    JoltC_WheelSettings_SetSuspensionSpring(ws, spring);
    JoltC_WheelSettings_SetRadius(ws, 0.35f);
    JoltC_WheelSettings_SetWidth(ws, 0.2f);
    JoltC_WheelSettings_SetEnableSuspensionForcePoint(ws, JOLTC_TRUE);
}

/* ========================================================================== */
void run_vehicle_extra_tests(void)
{
    /* ====================================================================== */
    /*  1. Enum numbering contract                                            */
    /* ====================================================================== */

    /* vehicle.cpp converts these with a bare cast — (ESpringMode)v.mode — so the C
     * enumerators must keep the numeric values of the Jolt ones. 5.6.0 adds
     * ESpringMode::MassNormalizedStiffnessAndDamping and EMotorState::PositionAndVelocity;
     * both must be APPENDED. If either is inserted ahead of an existing enumerator
     * and common.h is renumbered to match, this test fails and says so loudly —
     * which is the intent, because every stored spring mode would silently change
     * meaning. Read through variables so the comparison is not constant-folded into
     * a /W4 warning. */
    TEST_BEGIN("SpringMode / MotorState enumerator values are append-only");
    {
        int freqAndDamping = (int)JOLTC_SPRING_MODE_FREQUENCY_AND_DAMPING;
        int stiffAndDamping = (int)JOLTC_SPRING_MODE_STIFFNESS_AND_DAMPING;
        int motorOff = (int)JOLTC_MOTOR_STATE_OFF;
        int motorVelocity = (int)JOLTC_MOTOR_STATE_VELOCITY;
        int motorPosition = (int)JOLTC_MOTOR_STATE_POSITION;

        TEST_ASSERT(freqAndDamping == 0, "SPRING_MODE_FREQUENCY_AND_DAMPING == 0");
        TEST_ASSERT(stiffAndDamping == 1, "SPRING_MODE_STIFFNESS_AND_DAMPING == 1");
        TEST_ASSERT(motorOff == 0, "MOTOR_STATE_OFF == 0");
        TEST_ASSERT(motorVelocity == 1, "MOTOR_STATE_VELOCITY == 1");
        TEST_ASSERT(motorPosition == 2, "MOTOR_STATE_POSITION == 2");
    }
    TEST_END();

    /* ====================================================================== */
    /*  2. LinearCurve — the parts test_vehicle.c does not reach              */
    /* ====================================================================== */

    TEST_BEGIN("LinearCurve Reserve / Clear / point count");
    {
        JoltC_LinearCurve* curve = JoltC_LinearCurve_Create();
        TEST_ASSERT_NOT_NULL(curve, "LinearCurve not null");

        /* Reserve is capacity, not size: it must not create points. */
        JoltC_LinearCurve_Reserve(curve, 8);
        TEST_ASSERT(JoltC_LinearCurve_GetPointCount(curve) == 0u, "Reserve adds no points");

        JoltC_LinearCurve_AddPoint(curve, 0.5f, 12.5f);
        JoltC_LinearCurve_AddPoint(curve, 2.5f, -3.75f);
        TEST_ASSERT(JoltC_LinearCurve_GetPointCount(curve) == 2u, "2 points after AddPoint");

        JoltC_LinearCurve_Clear(curve);
        TEST_ASSERT(JoltC_LinearCurve_GetPointCount(curve) == 0u, "0 points after Clear");
        TEST_ASSERT_FLOAT_EQ(JoltC_LinearCurve_GetMinX(curve), 0.0f, 1e-6f, "empty curve minX == 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_LinearCurve_GetMaxX(curve), 0.0f, 1e-6f, "empty curve maxX == 0");

        JoltC_LinearCurve_Destroy(curve);
    }
    TEST_END();

    /* GetPoint / GetPoints are the only indexed readers in the vehicle header, so
     * they are where an off-by-one or an x/y swap would live. The two points below
     * have x and y far apart and of opposite sign for exactly that reason. */
    TEST_BEGIN("LinearCurve GetPoint / GetPoints");
    {
        JoltC_LinearCurve* curve = JoltC_LinearCurve_Create();
        JoltC_Point points[4];
        JoltC_Point single;
        uint32_t count;

        JoltC_LinearCurve_AddPoint(curve, 0.25f, -8.5f);
        JoltC_LinearCurve_AddPoint(curve, 6.75f, 3.125f);
        JoltC_LinearCurve_Sort(curve);

        single.x = -999.0f;
        single.y = -999.0f;
        JoltC_LinearCurve_GetPoint(curve, 0, &single);
        TEST_ASSERT_FLOAT_EQ(single.x, 0.25f, 1e-6f, "point 0 x == 0.25");
        TEST_ASSERT_FLOAT_EQ(single.y, -8.5f, 1e-6f, "point 0 y == -8.5");

        JoltC_LinearCurve_GetPoint(curve, 1, &single);
        TEST_ASSERT_FLOAT_EQ(single.x, 6.75f, 1e-6f, "point 1 x == 6.75");
        TEST_ASSERT_FLOAT_EQ(single.y, 3.125f, 1e-6f, "point 1 y == 3.125");

        /* Out of range must leave the caller's struct alone rather than write garbage. */
        single.x = -999.0f;
        single.y = -999.0f;
        JoltC_LinearCurve_GetPoint(curve, 7, &single);
        TEST_ASSERT_FLOAT_EQ(single.x, -999.0f, 1e-6f, "GetPoint out of range leaves x");
        TEST_ASSERT_FLOAT_EQ(single.y, -999.0f, 1e-6f, "GetPoint out of range leaves y");

        /* Buffer larger than the curve: fills 2, reports 2, does not touch the rest. */
        points[2].x = -999.0f;
        count = 4;
        JoltC_LinearCurve_GetPoints(curve, points, &count);
        TEST_ASSERT(count == 2u, "GetPoints reports 2 points");
        TEST_ASSERT_FLOAT_EQ(points[0].x, 0.25f, 1e-6f, "GetPoints[0].x");
        TEST_ASSERT_FLOAT_EQ(points[1].y, 3.125f, 1e-6f, "GetPoints[1].y");
        TEST_ASSERT_FLOAT_EQ(points[2].x, -999.0f, 1e-6f, "GetPoints leaves surplus slots");

        /* Buffer smaller than the curve: copies what fits, still reports the true size. */
        points[1].x = -999.0f;
        count = 1;
        JoltC_LinearCurve_GetPoints(curve, points, &count);
        TEST_ASSERT(count == 2u, "GetPoints reports true size when buffer is short");
        TEST_ASSERT_FLOAT_EQ(points[0].x, 0.25f, 1e-6f, "short buffer got first point");
        TEST_ASSERT_FLOAT_EQ(points[1].x, -999.0f, 1e-6f, "short buffer did not overrun");

        /* Size query with no buffer. */
        count = 0;
        JoltC_LinearCurve_GetPoints(curve, NULL, &count);
        TEST_ASSERT(count == 2u, "GetPoints(NULL) is a size query");

        JoltC_LinearCurve_Destroy(curve);
    }
    TEST_END();

    TEST_BEGIN("LinearCurve null safety");
    {
        JoltC_Point p;
        uint32_t count = 4;

        p.x = -999.0f;
        p.y = -999.0f;

        /* Every one of these is guarded in the implementation. */
        JoltC_LinearCurve_Clear(NULL);
        JoltC_LinearCurve_Reserve(NULL, 4);
        JoltC_LinearCurve_AddPoint(NULL, 1.0f, 2.0f);
        JoltC_LinearCurve_Sort(NULL);
        JoltC_LinearCurve_Destroy(NULL);
        JoltC_LinearCurve_GetPoint(NULL, 0, &p);
        JoltC_LinearCurve_GetPoints(NULL, NULL, &count);

        TEST_ASSERT_FLOAT_EQ(JoltC_LinearCurve_GetMinX(NULL), 0.0f, 1e-6f, "GetMinX(NULL) == 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_LinearCurve_GetMaxX(NULL), 0.0f, 1e-6f, "GetMaxX(NULL) == 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_LinearCurve_GetValue(NULL, 0.5f), 0.0f, 1e-6f, "GetValue(NULL) == 0");
        TEST_ASSERT(JoltC_LinearCurve_GetPointCount(NULL) == 0u, "GetPointCount(NULL) == 0");
        TEST_ASSERT_FLOAT_EQ(p.x, -999.0f, 1e-6f, "GetPoint(NULL) leaves result");
        TEST_ASSERT(count == 4u, "GetPoints(NULL curve) leaves count");
    }
    TEST_END();

    /* ====================================================================== */
    /*  3. WheelSettings — WS_VEC3_PROP and WS_FLOAT_PROP macro blocks        */
    /* ====================================================================== */

    /* Six Vec3 properties from one macro and five floats from another. Everything is
     * written before anything is read, so two macro entries sharing a member cannot
     * hide. */
    TEST_BEGIN("WheelSettings full property round-trip (all set, then all read)");
    {
        JoltC_WheelSettings* ws = JoltC_WheelSettings_Create();
        TEST_ASSERT_NOT_NULL(ws, "WheelSettings not null");

        /* Default radius first: it pins that Create really default-constructs a
         * Jolt WheelSettings rather than zeroing memory. */
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettings_GetRadius(ws), WS_DEFAULT_RADIUS, 1e-6f,
                             "default radius == 0.3");

        configure_wheel_settings(ws);

        TEST_ASSERT(vec3_equals(JoltC_WheelSettings_GetPosition(ws), k_position, 1e-6f),
                    "position round-trips");
        TEST_ASSERT(vec3_equals(JoltC_WheelSettings_GetSuspensionForcePoint(ws), k_forcePoint, 1e-6f),
                    "suspensionForcePoint round-trips");
        TEST_ASSERT(vec3_equals(JoltC_WheelSettings_GetSuspensionDirection(ws), k_suspensionDirection, 1e-6f),
                    "suspensionDirection round-trips");
        TEST_ASSERT(vec3_equals(JoltC_WheelSettings_GetSteeringAxis(ws), k_steeringAxis, 1e-6f),
                    "steeringAxis round-trips");
        TEST_ASSERT(vec3_equals(JoltC_WheelSettings_GetWheelUp(ws), k_wheelUp, 1e-6f),
                    "wheelUp round-trips");
        TEST_ASSERT(vec3_equals(JoltC_WheelSettings_GetWheelForward(ws), k_wheelForward, 1e-6f),
                    "wheelForward round-trips");

        /* Min 0.125 vs max 0.875 vs preload 0.0625: three same-typed neighbours in one
         * macro block, all far apart. Note the naming is upstream's and is not what it
         * sounds like — "min" length is the fully raised pose, "max" the fully drooped
         * one, and the spring's natural length is max + preload. */
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettings_GetSuspensionMinLength(ws), 0.125f, 1e-6f,
                             "suspensionMinLength round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettings_GetSuspensionMaxLength(ws), 0.875f, 1e-6f,
                             "suspensionMaxLength round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettings_GetSuspensionPreloadLength(ws), 0.0625f, 1e-6f,
                             "suspensionPreloadLength round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettings_GetRadius(ws), 0.35f, 1e-6f, "radius round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettings_GetWidth(ws), 0.2f, 1e-6f, "width round-trips");

        TEST_ASSERT(JoltC_WheelSettings_GetEnableSuspensionForcePoint(ws) == JOLTC_TRUE,
                    "enableSuspensionForcePoint true");
        JoltC_WheelSettings_SetEnableSuspensionForcePoint(ws, JOLTC_FALSE);
        TEST_ASSERT(JoltC_WheelSettings_GetEnableSuspensionForcePoint(ws) == JOLTC_FALSE,
                    "enableSuspensionForcePoint back to false");

        JoltC_WheelSettings_Destroy(ws);
    }
    TEST_END();

    /* The suspension spring is the one place in the vehicle header where a Jolt
     * ESpringMode crosses the boundary, and 5.6.0 adds a third mode. Both existing
     * modes are round-tripped, and the payload is checked to survive a mode change:
     * upstream stores frequency and stiffness in the same union member, so a repair
     * that splits them would show up as a lost value here. */
    TEST_BEGIN("WheelSettings suspension spring, both modes");
    {
        JoltC_WheelSettings* ws = JoltC_WheelSettings_Create();
        JoltC_SpringSettings spring;
        JoltC_SpringSettings out;

        /* Frequency mode. 2.25 Hz vs damping ratio 0.375 — far apart, so an exchange
         * of the two fields cannot pass. */
        spring.mode = JOLTC_SPRING_MODE_FREQUENCY_AND_DAMPING;
        spring.frequencyOrStiffness = 2.25f;
        spring.damping = 0.375f;
        JoltC_WheelSettings_SetSuspensionSpring(ws, spring);

        out = JoltC_WheelSettings_GetSuspensionSpring(ws);
        TEST_ASSERT(out.mode == JOLTC_SPRING_MODE_FREQUENCY_AND_DAMPING, "spring mode frequency");
        TEST_ASSERT_FLOAT_EQ(out.frequencyOrStiffness, 2.25f, 1e-6f, "spring frequency == 2.25");
        TEST_ASSERT_FLOAT_EQ(out.damping, 0.375f, 1e-6f, "spring damping == 0.375");

        /* Stiffness mode. Stiffness is in N/m, so a realistic value is four orders of
         * magnitude away from a frequency in Hz — a mode that is ignored on the way in
         * or out is very visible. */
        spring.mode = JOLTC_SPRING_MODE_STIFFNESS_AND_DAMPING;
        spring.frequencyOrStiffness = 7500.5f;
        spring.damping = 1.75f;
        JoltC_WheelSettings_SetSuspensionSpring(ws, spring);

        out = JoltC_WheelSettings_GetSuspensionSpring(ws);
        TEST_ASSERT(out.mode == JOLTC_SPRING_MODE_STIFFNESS_AND_DAMPING, "spring mode stiffness");
        TEST_ASSERT_FLOAT_EQ(out.frequencyOrStiffness, 7500.5f, 1e-3f, "spring stiffness == 7500.5");
        TEST_ASSERT_FLOAT_EQ(out.damping, 1.75f, 1e-6f, "stiffness-mode damping == 1.75");

        JoltC_WheelSettings_Destroy(ws);
    }
    TEST_END();

    TEST_BEGIN("WheelSettings null safety");
    {
        JoltC_SpringSettings spring;
        JoltC_SpringSettings out;

        spring.mode = JOLTC_SPRING_MODE_STIFFNESS_AND_DAMPING;
        spring.frequencyOrStiffness = 1.0f;
        spring.damping = 1.0f;

        /* Setters on NULL are no-ops. */
        JoltC_WheelSettings_SetPosition(NULL, k_position);
        JoltC_WheelSettings_SetSuspensionForcePoint(NULL, k_position);
        JoltC_WheelSettings_SetSuspensionDirection(NULL, k_position);
        JoltC_WheelSettings_SetSteeringAxis(NULL, k_position);
        JoltC_WheelSettings_SetWheelUp(NULL, k_position);
        JoltC_WheelSettings_SetWheelForward(NULL, k_position);
        JoltC_WheelSettings_SetSuspensionMinLength(NULL, 1.0f);
        JoltC_WheelSettings_SetSuspensionMaxLength(NULL, 1.0f);
        JoltC_WheelSettings_SetSuspensionPreloadLength(NULL, 1.0f);
        JoltC_WheelSettings_SetRadius(NULL, 1.0f);
        JoltC_WheelSettings_SetWidth(NULL, 1.0f);
        JoltC_WheelSettings_SetSuspensionSpring(NULL, spring);
        JoltC_WheelSettings_SetEnableSuspensionForcePoint(NULL, JOLTC_TRUE);
        JoltC_WheelSettings_Destroy(NULL);

        TEST_ASSERT(vec3_is_zero(JoltC_WheelSettings_GetPosition(NULL)), "GetPosition(NULL) zero");
        TEST_ASSERT(vec3_is_zero(JoltC_WheelSettings_GetSuspensionDirection(NULL)),
                    "GetSuspensionDirection(NULL) zero");
        TEST_ASSERT(vec3_is_zero(JoltC_WheelSettings_GetWheelForward(NULL)),
                    "GetWheelForward(NULL) zero");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettings_GetRadius(NULL), 0.0f, 1e-6f, "GetRadius(NULL) == 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettings_GetWidth(NULL), 0.0f, 1e-6f, "GetWidth(NULL) == 0");
        TEST_ASSERT(JoltC_WheelSettings_GetEnableSuspensionForcePoint(NULL) == JOLTC_FALSE,
                    "GetEnableSuspensionForcePoint(NULL) false");

        out = JoltC_WheelSettings_GetSuspensionSpring(NULL);
        TEST_ASSERT(out.mode == JOLTC_SPRING_MODE_FREQUENCY_AND_DAMPING,
                    "GetSuspensionSpring(NULL) mode is frequency");
        TEST_ASSERT_FLOAT_EQ(out.frequencyOrStiffness, 0.0f, 1e-6f,
                             "GetSuspensionSpring(NULL) value 0");
        TEST_ASSERT_FLOAT_EQ(out.damping, 0.0f, 1e-6f, "GetSuspensionSpring(NULL) damping 0");
    }
    TEST_END();

    /* ====================================================================== */
    /*  4. WheelSettingsWV — WSWV_FLOAT block and the friction curves         */
    /* ====================================================================== */

    TEST_BEGIN("WheelSettingsWV five-float block round-trip");
    {
        JoltC_WheelSettingsWV* ws = JoltC_WheelSettingsWV_Create();
        TEST_ASSERT_NOT_NULL(ws, "WheelSettingsWV not null");

        /* Defaults first. inertia 0.9 and angularDamping 0.2 are the pair most likely
         * to be crossed, and the brake torques differ by 2.5x so their exchange shows. */
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsWV_GetInertia(ws), 0.9f, 1e-6f,
                             "default inertia == 0.9");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsWV_GetAngularDamping(ws), 0.2f, 1e-6f,
                             "default angularDamping == 0.2");
        /* Radians, not degrees: upstream default is DegreesToRadians(70). */
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsWV_GetMaxSteerAngle(ws), 1.2217305f, 1e-5f,
                             "default maxSteerAngle == 70 deg in radians");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsWV_GetMaxBrakeTorque(ws), 1500.0f, 1e-3f,
                             "default maxBrakeTorque == 1500");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsWV_GetMaxHandBrakeTorque(ws), 4000.0f, 1e-3f,
                             "default maxHandBrakeTorque == 4000");

        /* Write all five, then read all five. */
        JoltC_WheelSettingsWV_SetInertia(ws, 2.0f);
        JoltC_WheelSettingsWV_SetAngularDamping(ws, 0.375f);
        JoltC_WheelSettingsWV_SetMaxSteerAngle(ws, 1.25f);       /* radians, < pi/2 */
        JoltC_WheelSettingsWV_SetMaxBrakeTorque(ws, 1234.5f);
        JoltC_WheelSettingsWV_SetMaxHandBrakeTorque(ws, 4321.75f);

        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsWV_GetInertia(ws), 2.0f, 1e-6f, "inertia round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsWV_GetAngularDamping(ws), 0.375f, 1e-6f,
                             "angularDamping round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsWV_GetMaxSteerAngle(ws), 1.25f, 1e-6f,
                             "maxSteerAngle round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsWV_GetMaxBrakeTorque(ws), 1234.5f, 1e-3f,
                             "maxBrakeTorque round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsWV_GetMaxHandBrakeTorque(ws), 4321.75f, 1e-3f,
                             "maxHandBrakeTorque round-trips");

        /* Null guards for the same block. */
        JoltC_WheelSettingsWV_SetInertia(NULL, 1.0f);
        JoltC_WheelSettingsWV_SetAngularDamping(NULL, 1.0f);
        JoltC_WheelSettingsWV_SetMaxSteerAngle(NULL, 1.0f);
        JoltC_WheelSettingsWV_SetMaxBrakeTorque(NULL, 1.0f);
        JoltC_WheelSettingsWV_SetMaxHandBrakeTorque(NULL, 1.0f);
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsWV_GetInertia(NULL), 0.0f, 1e-6f,
                             "GetInertia(NULL) == 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsWV_GetMaxHandBrakeTorque(NULL), 0.0f, 1e-6f,
                             "GetMaxHandBrakeTorque(NULL) == 0");

        /* WheelSettingsWV has no Destroy of its own: the base one releases the same
         * refcounted object, and WheelSettings has a virtual destructor, so this is
         * the intended (undocumented) way to free it. */
        JoltC_WheelSettings_Destroy((JoltC_WheelSettings*)ws);
    }
    TEST_END();

    /* The two friction curves are the sharpest swap detector in the whole header,
     * because upstream gives them different default ranges AND different X units:
     * longitudinal X is a dimensionless slip ratio in [0, 0.2], lateral X is a slip
     * angle in DEGREES in [0, 20]. A getter or setter reading its neighbour is a
     * factor-of-100 error, not a subtle one. */
    TEST_BEGIN("WheelSettingsWV longitudinal vs lateral friction curves");
    {
        JoltC_WheelSettingsWV* ws = JoltC_WheelSettingsWV_Create();
        JoltC_LinearCurve* mine = JoltC_LinearCurve_Create();
        JoltC_LinearCurve* other = JoltC_LinearCurve_Create();
        const JoltC_LinearCurve* got;

        /* Defaults tell the two curves apart. */
        got = JoltC_WheelSettingsWV_GetLongitudinalFriction(ws);
        TEST_ASSERT_NOT_NULL(got, "default longitudinal curve not null");
        TEST_ASSERT_FLOAT_EQ(JoltC_LinearCurve_GetMaxX(got), WSWV_DEFAULT_LONG_MAX_X, 1e-6f,
                             "default longitudinal curve maxX == 0.2 (slip ratio)");

        got = JoltC_WheelSettingsWV_GetLateralFriction(ws);
        TEST_ASSERT_NOT_NULL(got, "default lateral curve not null");
        TEST_ASSERT_FLOAT_EQ(JoltC_LinearCurve_GetMaxX(got), WSWV_DEFAULT_LAT_MAX_X, 1e-6f,
                             "default lateral curve maxX == 20 (slip angle in degrees)");

        /* Set only the longitudinal curve and check the lateral one is untouched.
         * The getters hand back a pointer to a shared per-thread wrapper, so each
         * result is read out immediately rather than held across another call. */
        JoltC_LinearCurve_AddPoint(mine, 0.0f, 0.25f);
        JoltC_LinearCurve_AddPoint(mine, 1.5f, 1.75f);
        JoltC_LinearCurve_Sort(mine);
        JoltC_WheelSettingsWV_SetLongitudinalFriction(ws, mine);

        got = JoltC_WheelSettingsWV_GetLongitudinalFriction(ws);
        TEST_ASSERT(JoltC_LinearCurve_GetPointCount(got) == 2u, "longitudinal curve has 2 points");
        TEST_ASSERT_FLOAT_EQ(JoltC_LinearCurve_GetMaxX(got), 1.5f, 1e-6f, "longitudinal maxX == 1.5");
        TEST_ASSERT_FLOAT_EQ(JoltC_LinearCurve_GetValue(got, 0.0f), 0.25f, 1e-6f,
                             "longitudinal value at 0 == 0.25");

        got = JoltC_WheelSettingsWV_GetLateralFriction(ws);
        TEST_ASSERT_FLOAT_EQ(JoltC_LinearCurve_GetMaxX(got), WSWV_DEFAULT_LAT_MAX_X, 1e-6f,
                             "setting longitudinal left lateral alone");

        /* Now the other direction. */
        JoltC_LinearCurve_AddPoint(other, 0.0f, 0.5f);
        JoltC_LinearCurve_AddPoint(other, 9.0f, 1.25f);
        JoltC_LinearCurve_Sort(other);
        JoltC_WheelSettingsWV_SetLateralFriction(ws, other);

        got = JoltC_WheelSettingsWV_GetLateralFriction(ws);
        TEST_ASSERT_FLOAT_EQ(JoltC_LinearCurve_GetMaxX(got), 9.0f, 1e-6f, "lateral maxX == 9");
        TEST_ASSERT_FLOAT_EQ(JoltC_LinearCurve_GetValue(got, 9.0f), 1.25f, 1e-6f,
                             "lateral value at 9 == 1.25");

        got = JoltC_WheelSettingsWV_GetLongitudinalFriction(ws);
        TEST_ASSERT_FLOAT_EQ(JoltC_LinearCurve_GetMaxX(got), 1.5f, 1e-6f,
                             "setting lateral left longitudinal alone");

        /* The curves are copied in, not aliased: mutating the caller's curve
         * afterwards must not reach the settings. */
        JoltC_LinearCurve_Clear(mine);
        got = JoltC_WheelSettingsWV_GetLongitudinalFriction(ws);
        TEST_ASSERT(JoltC_LinearCurve_GetPointCount(got) == 2u,
                    "SetLongitudinalFriction copied the curve");

        /* Guarded: a NULL curve is ignored rather than clearing the property. */
        JoltC_WheelSettingsWV_SetLongitudinalFriction(ws, NULL);
        got = JoltC_WheelSettingsWV_GetLongitudinalFriction(ws);
        TEST_ASSERT(JoltC_LinearCurve_GetPointCount(got) == 2u,
                    "SetLongitudinalFriction(NULL) is a no-op");
        TEST_ASSERT(JoltC_WheelSettingsWV_GetLongitudinalFriction(NULL) == NULL,
                    "GetLongitudinalFriction(NULL) == NULL");
        TEST_ASSERT(JoltC_WheelSettingsWV_GetLateralFriction(NULL) == NULL,
                    "GetLateralFriction(NULL) == NULL");

        JoltC_LinearCurve_Destroy(other);
        JoltC_LinearCurve_Destroy(mine);
        JoltC_WheelSettings_Destroy((JoltC_WheelSettings*)ws);
    }
    TEST_END();

    /* ====================================================================== */
    /*  5. WheelSettingsTV                                                    */
    /* ====================================================================== */

    /* Two scalars of the same type with different defaults (4.0 and 2.0): the
     * defaults alone catch a crossed pair, and the round-trip values are 3.6x apart. */
    TEST_BEGIN("WheelSettingsTV friction round-trip");
    {
        JoltC_WheelSettingsTV* ws = JoltC_WheelSettingsTV_Create();
        TEST_ASSERT_NOT_NULL(ws, "WheelSettingsTV not null");

        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsTV_GetLongitudinalFriction(ws), 4.0f, 1e-6f,
                             "default TV longitudinal friction == 4");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsTV_GetLateralFriction(ws), 2.0f, 1e-6f,
                             "default TV lateral friction == 2");

        JoltC_WheelSettingsTV_SetLongitudinalFriction(ws, 4.5f);
        JoltC_WheelSettingsTV_SetLateralFriction(ws, 1.25f);
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsTV_GetLongitudinalFriction(ws), 4.5f, 1e-6f,
                             "TV longitudinal friction round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsTV_GetLateralFriction(ws), 1.25f, 1e-6f,
                             "TV lateral friction round-trips");

        JoltC_WheelSettingsTV_SetLongitudinalFriction(NULL, 1.0f);
        JoltC_WheelSettingsTV_SetLateralFriction(NULL, 1.0f);
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsTV_GetLongitudinalFriction(NULL), 0.0f, 1e-6f,
                             "TV longitudinal friction(NULL) == 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsTV_GetLateralFriction(NULL), 0.0f, 1e-6f,
                             "TV lateral friction(NULL) == 0");

        JoltC_WheelSettings_Destroy((JoltC_WheelSettings*)ws);
    }
    TEST_END();

    /* ====================================================================== */
    /*  6. Wheel instances (no constraint, no stepping)                       */
    /* ====================================================================== */

    /* A standalone Wheel is a data holder: nothing integrates it until a
     * VehicleConstraint solves it. So the settings link, the initial suspension
     * length and the three angle accessors are all deterministic, and none of them
     * is a trajectory assertion. */
    TEST_BEGIN("Wheel create, settings identity, initial state");
    {
        JoltC_WheelSettings* ws = JoltC_WheelSettings_Create();
        JoltC_Wheel* wheel;
        const JoltC_WheelSettings* linked;

        configure_wheel_settings(ws);
        wheel = JoltC_Wheel_Create(ws);
        TEST_ASSERT_NOT_NULL(wheel, "Wheel not null");

        /* The wheel must reference the very settings object it was built from. */
        linked = JoltC_Wheel_GetSettings(wheel);
        TEST_ASSERT(linked == (const JoltC_WheelSettings*)ws, "Wheel_GetSettings returns the settings");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettings_GetRadius(linked), 0.35f, 1e-6f,
                             "radius readable through the linked settings");

        /* Upstream documents the initial suspension length as the max length. */
        TEST_ASSERT_FLOAT_EQ(JoltC_Wheel_GetSuspensionLength(wheel), 0.875f, 1e-6f,
                             "initial suspension length == suspensionMaxLength");

        /* Fresh wheel: no contact, no accumulated impulses, no hard point. */
        TEST_ASSERT(JoltC_Wheel_HasContact(wheel) == JOLTC_FALSE, "fresh wheel has no contact");
        TEST_ASSERT(JoltC_Wheel_GetContactBodyID(wheel) == JOLTC_BODY_ID_INVALID,
                    "fresh wheel contact body invalid");
        /* Jolt's empty sub shape path is all bits set. Worth pinning because the
         * wrapper's NULL fallback for this same getter is 0, and 0 is a perfectly
         * legal sub shape ID — so "no contact" and "sub shape 0" are distinguishable
         * only as long as this sentinel holds. */
        TEST_ASSERT(JoltC_Wheel_GetContactSubShapeID(wheel) == 0xFFFFFFFFu,
                    "fresh wheel contact sub shape ID is the empty path sentinel");
        TEST_ASSERT(JoltC_Wheel_HasHitHardPoint(wheel) == JOLTC_FALSE, "fresh wheel no hard point");
        TEST_ASSERT_FLOAT_EQ(JoltC_Wheel_GetSuspensionLambda(wheel), 0.0f, 1e-6f,
                             "suspension lambda 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_Wheel_GetLongitudinalLambda(wheel), 0.0f, 1e-6f,
                             "longitudinal lambda 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_Wheel_GetLateralLambda(wheel), 0.0f, 1e-6f, "lateral lambda 0");

        /* Upstream asserts on these when there is no contact; the wrapper adds a
         * guard and returns zero instead. That divergence is deliberate and is what
         * makes the accessors safe to call from a managed caller, so it is pinned. */
        {
            JoltC_RVec3 cp = JoltC_Wheel_GetContactPosition(wheel);
            TEST_ASSERT(cp.x == 0.0 && cp.y == 0.0 && cp.z == 0.0,
                        "contact position zeroed without contact");
        }
        TEST_ASSERT(vec3_is_zero(JoltC_Wheel_GetContactPointVelocity(wheel)),
                    "contact point velocity zeroed without contact");
        TEST_ASSERT(vec3_is_zero(JoltC_Wheel_GetContactNormal(wheel)),
                    "contact normal zeroed without contact");
        TEST_ASSERT(vec3_is_zero(JoltC_Wheel_GetContactLongitudinal(wheel)),
                    "contact longitudinal zeroed without contact");
        TEST_ASSERT(vec3_is_zero(JoltC_Wheel_GetContactLateral(wheel)),
                    "contact lateral zeroed without contact");

        /* Three same-typed angle/velocity accessors, written together then read
         * together. Signs differ so a cross-assignment is visible. */
        JoltC_Wheel_SetAngularVelocity(wheel, -12.5f);   /* rad/s */
        JoltC_Wheel_SetRotationAngle(wheel, 1.75f);      /* rad, [0, 2 pi] */
        JoltC_Wheel_SetSteerAngle(wheel, -0.625f);       /* rad, positive is left */
        TEST_ASSERT_FLOAT_EQ(JoltC_Wheel_GetAngularVelocity(wheel), -12.5f, 1e-6f,
                             "angular velocity round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_Wheel_GetRotationAngle(wheel), 1.75f, 1e-6f,
                             "rotation angle round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_Wheel_GetSteerAngle(wheel), -0.625f, 1e-6f,
                             "steer angle round-trips");

        /* Null guards. */
        TEST_ASSERT(JoltC_Wheel_GetSettings(NULL) == NULL, "Wheel_GetSettings(NULL) == NULL");
        TEST_ASSERT(JoltC_Wheel_Create(NULL) == NULL, "Wheel_Create(NULL) == NULL");
        TEST_ASSERT(JoltC_Wheel_HasContact(NULL) == JOLTC_FALSE, "HasContact(NULL) false");
        TEST_ASSERT(JoltC_Wheel_GetContactBodyID(NULL) == JOLTC_BODY_ID_INVALID,
                    "GetContactBodyID(NULL) invalid");
        TEST_ASSERT_FLOAT_EQ(JoltC_Wheel_GetSteerAngle(NULL), 0.0f, 1e-6f, "GetSteerAngle(NULL) == 0");
        JoltC_Wheel_SetSteerAngle(NULL, 1.0f);
        JoltC_Wheel_Destroy(NULL);

        /* Wheel first, settings second: the wheel holds a reference to them. */
        JoltC_Wheel_Destroy(wheel);
        JoltC_WheelSettings_Destroy(ws);
    }
    TEST_END();

    /* WheelWV / WheelTV are constructed from a settings pointer that the
     * implementation static_casts to the derived settings type, so they must be fed
     * a WheelSettingsWV / WheelSettingsTV cast to the base handle — the C types give
     * no help here. ApplyTorque is included because it is the one runtime function
     * with a documented closed-form result: dw = torque * dt / inertia. That makes it
     * a genuine check that WSWV_FLOAT(Inertia, mInertia) names the right member,
     * and it is not a trajectory assertion — no solver is involved. */
    TEST_BEGIN("WheelWV / WheelTV create, settings, ApplyTorque");
    {
        JoltC_WheelSettingsWV* wsWV = JoltC_WheelSettingsWV_Create();
        JoltC_WheelSettingsTV* wsTV = JoltC_WheelSettingsTV_Create();
        JoltC_Wheel* wheelWV;
        JoltC_Wheel* wheelTV;
        const JoltC_WheelSettingsWV* linkedWV;
        const JoltC_WheelSettingsTV* linkedTV;

        /* Base properties of a WV wheel are only reachable by casting to the base
         * handle; there is no WheelSettingsWV accessor for radius. */
        JoltC_WheelSettings_SetRadius((JoltC_WheelSettings*)wsWV, 0.45f);
        JoltC_WheelSettings_SetWidth((JoltC_WheelSettings*)wsWV, 0.15f);
        JoltC_WheelSettingsWV_SetInertia(wsWV, 2.0f);

        wheelWV = JoltC_WheelWV_Create((const JoltC_WheelSettings*)wsWV);
        TEST_ASSERT_NOT_NULL(wheelWV, "WheelWV not null");

        linkedWV = JoltC_WheelWV_GetSettings((const JoltC_WheelWV*)wheelWV);
        TEST_ASSERT(linkedWV == (const JoltC_WheelSettingsWV*)wsWV,
                    "WheelWV_GetSettings returns the WV settings");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsWV_GetInertia(linkedWV), 2.0f, 1e-6f,
                             "inertia visible through WheelWV settings");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettings_GetRadius((const JoltC_WheelSettings*)linkedWV),
                             0.45f, 1e-6f, "base radius visible through WheelWV settings");

        /* dw = 3.0 * 0.5 / 2.0 = 0.75, from -4.5 to -3.75. */
        JoltC_Wheel_SetAngularVelocity(wheelWV, -4.5f);
        JoltC_WheelWV_ApplyTorque((JoltC_WheelWV*)wheelWV, 3.0f, 0.5f);
        TEST_ASSERT_FLOAT_EQ(JoltC_Wheel_GetAngularVelocity(wheelWV), -3.75f, 1e-5f,
                             "ApplyTorque adds torque * dt / inertia");

        JoltC_WheelSettingsTV_SetLongitudinalFriction(wsTV, 4.5f);
        wheelTV = JoltC_WheelTV_Create((const JoltC_WheelSettings*)wsTV);
        TEST_ASSERT_NOT_NULL(wheelTV, "WheelTV not null");

        linkedTV = JoltC_WheelTV_GetSettings((const JoltC_WheelTV*)wheelTV);
        TEST_ASSERT(linkedTV == (const JoltC_WheelSettingsTV*)wsTV,
                    "WheelTV_GetSettings returns the TV settings");
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsTV_GetLongitudinalFriction(linkedTV), 4.5f, 1e-6f,
                             "longitudinal friction visible through WheelTV settings");

        TEST_ASSERT(JoltC_WheelWV_Create(NULL) == NULL, "WheelWV_Create(NULL) == NULL");
        TEST_ASSERT(JoltC_WheelTV_Create(NULL) == NULL, "WheelTV_Create(NULL) == NULL");
        TEST_ASSERT(JoltC_WheelWV_GetSettings(NULL) == NULL, "WheelWV_GetSettings(NULL) == NULL");
        TEST_ASSERT(JoltC_WheelTV_GetSettings(NULL) == NULL, "WheelTV_GetSettings(NULL) == NULL");
        JoltC_WheelWV_ApplyTorque(NULL, 1.0f, 0.5f);

        JoltC_Wheel_Destroy(wheelTV);
        JoltC_Wheel_Destroy(wheelWV);
        JoltC_WheelSettings_Destroy((JoltC_WheelSettings*)wsTV);
        JoltC_WheelSettings_Destroy((JoltC_WheelSettings*)wsWV);
    }
    TEST_END();

    /* ====================================================================== */
    /*  7. Plain-struct initialisers                                          */
    /* ====================================================================== */

    TEST_BEGIN("VehicleConstraintSettings_Init defaults");
    {
        JoltC_VehicleConstraintSettings s;

        /* Sentinels, so "Init did nothing" cannot pass as "Init wrote the default". */
        s.up.x = -999.0f;
        s.forward.z = -999.0f;
        s.maxPitchRollAngle = -999.0f;
        s.wheelsCount = 7;
        s.antiRollBarsCount = 7;
        s.wheels = (JoltC_WheelSettings**)&s;             /* any non-NULL */
        s.antiRollBars = (const JoltC_VehicleAntiRollBar*)&s;
        s.controller = (JoltC_VehicleControllerSettings*)&s;

        JoltC_VehicleConstraintSettings_Init(&s);

        TEST_ASSERT_FLOAT_EQ(s.up.x, 0.0f, 1e-6f, "default up.x == 0");
        TEST_ASSERT_FLOAT_EQ(s.up.y, 1.0f, 1e-6f, "default up.y == 1");
        TEST_ASSERT_FLOAT_EQ(s.up.z, 0.0f, 1e-6f, "default up.z == 0");
        /* Forward is +Z, up is +Y: distinct axes, so a copy/paste between the two
         * would show here. */
        TEST_ASSERT_FLOAT_EQ(s.forward.x, 0.0f, 1e-6f, "default forward.x == 0");
        TEST_ASSERT_FLOAT_EQ(s.forward.y, 0.0f, 1e-6f, "default forward.y == 0");
        TEST_ASSERT_FLOAT_EQ(s.forward.z, 1.0f, 1e-6f, "default forward.z == 1");
        /* Radians: pi means no limit. */
        TEST_ASSERT_FLOAT_EQ(s.maxPitchRollAngle, 3.14159265f, 1e-5f,
                             "default maxPitchRollAngle == pi radians");
        TEST_ASSERT(s.wheelsCount == 0u, "default wheelsCount == 0");
        TEST_ASSERT(s.wheels == NULL, "default wheels NULL");
        TEST_ASSERT(s.antiRollBarsCount == 0u, "default antiRollBarsCount == 0");
        TEST_ASSERT(s.antiRollBars == NULL, "default antiRollBars NULL");
        TEST_ASSERT(s.controller == NULL, "default controller NULL");

        JoltC_VehicleConstraintSettings_Init(NULL);
    }
    TEST_END();

    TEST_BEGIN("VehicleEngineSettings_Init defaults");
    {
        JoltC_VehicleEngineSettings e;

        e.maxTorque = -999.0f;
        e.minRPM = -999.0f;
        e.maxRPM = -999.0f;
        e.inertia = -999.0f;
        e.angularDamping = -999.0f;
        e.normalizedTorque = (const JoltC_LinearCurve*)&e;

        JoltC_VehicleEngineSettings_Init(&e);

        TEST_ASSERT_FLOAT_EQ(e.maxTorque, 500.0f, 1e-3f, "default maxTorque == 500 Nm");
        /* 1000 vs 6000: a min/max exchange is unmissable. */
        TEST_ASSERT_FLOAT_EQ(e.minRPM, 1000.0f, 1e-3f, "default minRPM == 1000");
        TEST_ASSERT_FLOAT_EQ(e.maxRPM, 6000.0f, 1e-3f, "default maxRPM == 6000");
        TEST_ASSERT_FLOAT_EQ(e.inertia, 0.5f, 1e-6f, "default inertia == 0.5");
        TEST_ASSERT_FLOAT_EQ(e.angularDamping, 0.2f, 1e-6f, "default angularDamping == 0.2");
        /* The normalized torque curve is deliberately not surfaced: upstream embeds
         * it by value and the wrapper has nowhere to hand it out from, so the field
         * is always nulled on the way out. A caller doing read-modify-write of engine
         * settings therefore silently resets the curve to Jolt's default. */
        TEST_ASSERT(e.normalizedTorque == NULL, "normalizedTorque is not surfaced (always NULL)");

        JoltC_VehicleEngineSettings_Init(NULL);
    }
    TEST_END();

    TEST_BEGIN("VehicleDifferentialSettings_Init defaults");
    {
        JoltC_VehicleDifferentialSettings d;

        d.leftWheel = -999;
        d.rightWheel = -999;
        d.differentialRatio = -999.0f;
        d.leftRightSplit = -999.0f;
        d.limitedSlipRatio = -999.0f;
        d.engineTorqueRatio = -999.0f;

        JoltC_VehicleDifferentialSettings_Init(&d);

        TEST_ASSERT(d.leftWheel == -1, "default leftWheel == -1 (no wheel)");
        TEST_ASSERT(d.rightWheel == -1, "default rightWheel == -1 (no wheel)");
        TEST_ASSERT_FLOAT_EQ(d.differentialRatio, 3.42f, 1e-5f, "default differentialRatio == 3.42");
        /* 0 = all torque left, 0.5 = centred, 1 = all right. */
        TEST_ASSERT_FLOAT_EQ(d.leftRightSplit, 0.5f, 1e-6f, "default leftRightSplit == 0.5");
        /* A ratio of max/min wheel speed, so it must be > 1; upstream asserts that. */
        TEST_ASSERT_FLOAT_EQ(d.limitedSlipRatio, 1.4f, 1e-6f, "default limitedSlipRatio == 1.4");
        TEST_ASSERT(d.limitedSlipRatio > 1.0f, "limitedSlipRatio is a ratio > 1");
        TEST_ASSERT_FLOAT_EQ(d.engineTorqueRatio, 1.0f, 1e-6f, "default engineTorqueRatio == 1");

        JoltC_VehicleDifferentialSettings_Init(NULL);
    }
    TEST_END();

    TEST_BEGIN("VehicleAntiRollBar_Init defaults");
    {
        JoltC_VehicleAntiRollBar bar;

        bar.leftWheel = -999;
        bar.rightWheel = -999;
        bar.stiffness = -999.0f;

        JoltC_VehicleAntiRollBar_Init(&bar);

        /* Upstream defaults are 0 and 1 — different values, so a swapped assignment
         * inside the initialiser is caught. */
        TEST_ASSERT(bar.leftWheel == 0, "default anti-roll bar leftWheel == 0");
        TEST_ASSERT(bar.rightWheel == 1, "default anti-roll bar rightWheel == 1");
        TEST_ASSERT_FLOAT_EQ(bar.stiffness, 1000.0f, 1e-3f, "default stiffness == 1000 N/m");

        JoltC_VehicleAntiRollBar_Init(NULL);
    }
    TEST_END();

    TEST_BEGIN("VehicleTrackSettings_Init defaults");
    {
        JoltC_VehicleTrackSettings t;

        t.inertia = -999.0f;
        t.angularDamping = -999.0f;
        t.maxBrakeTorque = -999.0f;
        t.differentialRatio = -999.0f;
        t.wheels = (const uint32_t*)&t;
        t.wheelsCount = 7;

        JoltC_VehicleTrackSettings_Init(&t);

        TEST_ASSERT_FLOAT_EQ(t.inertia, 10.0f, 1e-4f, "default track inertia == 10");
        TEST_ASSERT_FLOAT_EQ(t.angularDamping, 0.5f, 1e-6f, "default track angularDamping == 0.5");
        TEST_ASSERT_FLOAT_EQ(t.maxBrakeTorque, 15000.0f, 1e-1f, "default track maxBrakeTorque == 15000");
        TEST_ASSERT_FLOAT_EQ(t.differentialRatio, 6.0f, 1e-5f, "default track differentialRatio == 6");
        /* The wrapper owns these two, unlike the rest. */
        TEST_ASSERT(t.wheels == NULL, "default track wheels NULL");
        TEST_ASSERT(t.wheelsCount == 0u, "default track wheelsCount == 0");
        /* drivenWheel is deliberately not asserted: upstream declares it with no
         * initialiser, so Init copies whatever the default constructor left there. */

        JoltC_VehicleTrackSettings_Init(NULL);
    }
    TEST_END();

    /* ====================================================================== */
    /*  8. WheeledVehicleControllerSettings                                   */
    /* ====================================================================== */

    TEST_BEGIN("WheeledVehicleControllerSettings engine round-trip");
    {
        JoltC_WheeledVehicleControllerSettings* s = JoltC_WheeledVehicleControllerSettings_Create();
        JoltC_LinearCurve* torque = JoltC_LinearCurve_Create();
        JoltC_VehicleEngineSettings e;

        TEST_ASSERT_NOT_NULL(s, "WheeledVehicleControllerSettings not null");

        /* Defaults, which also distinguish this type from the tracked one below. */
        JoltC_WheeledVehicleControllerSettings_GetEngine(s, &e);
        TEST_ASSERT_FLOAT_EQ(e.maxTorque, 500.0f, 1e-3f, "wheeled default maxTorque == 500");
        TEST_ASSERT_FLOAT_EQ(e.minRPM, 1000.0f, 1e-3f, "wheeled default minRPM == 1000");
        TEST_ASSERT_FLOAT_EQ(e.maxRPM, 6000.0f, 1e-3f, "wheeled default maxRPM == 6000");

        /* Write every field to something distinctive, then read them all back. */
        JoltC_LinearCurve_AddPoint(torque, 0.0f, 0.7f);
        JoltC_LinearCurve_AddPoint(torque, 1.0f, 0.9f);
        JoltC_LinearCurve_Sort(torque);

        e.maxTorque = 725.5f;
        e.minRPM = 850.25f;
        e.maxRPM = 5500.75f;
        e.inertia = 0.625f;
        e.angularDamping = 0.09375f;
        e.normalizedTorque = torque;
        JoltC_WheeledVehicleControllerSettings_SetEngine(s, &e);

        /* Read into a scrubbed struct so nothing can pass by leftover value. */
        e.maxTorque = -999.0f;
        e.minRPM = -999.0f;
        e.maxRPM = -999.0f;
        e.inertia = -999.0f;
        e.angularDamping = -999.0f;
        e.normalizedTorque = (const JoltC_LinearCurve*)torque;
        JoltC_WheeledVehicleControllerSettings_GetEngine(s, &e);

        TEST_ASSERT_FLOAT_EQ(e.maxTorque, 725.5f, 1e-3f, "engine maxTorque round-trips");
        TEST_ASSERT_FLOAT_EQ(e.minRPM, 850.25f, 1e-3f, "engine minRPM round-trips");
        TEST_ASSERT_FLOAT_EQ(e.maxRPM, 5500.75f, 1e-3f, "engine maxRPM round-trips");
        TEST_ASSERT_FLOAT_EQ(e.inertia, 0.625f, 1e-6f, "engine inertia round-trips");
        TEST_ASSERT_FLOAT_EQ(e.angularDamping, 0.09375f, 1e-6f, "engine angularDamping round-trips");
        TEST_ASSERT(e.normalizedTorque == NULL, "engine normalizedTorque comes back NULL");

        /* Guards. */
        JoltC_WheeledVehicleControllerSettings_SetEngine(s, NULL);
        JoltC_WheeledVehicleControllerSettings_SetEngine(NULL, &e);
        JoltC_WheeledVehicleControllerSettings_GetEngine(NULL, &e);
        JoltC_WheeledVehicleControllerSettings_GetEngine(s, NULL);
        TEST_ASSERT_FLOAT_EQ(e.maxTorque, 725.5f, 1e-3f, "guarded calls left the struct alone");

        JoltC_LinearCurve_Destroy(torque);
        JoltC_VehicleControllerSettings_Destroy((JoltC_VehicleControllerSettings*)s);
    }
    TEST_END();

    TEST_BEGIN("WheeledVehicleControllerSettings transmission round-trip");
    {
        JoltC_WheeledVehicleControllerSettings* s = JoltC_WheeledVehicleControllerSettings_Create();
        JoltC_VehicleTransmissionSettings* ts = JoltC_VehicleTransmissionSettings_Create();
        const JoltC_VehicleTransmissionSettings* got;
        const float gears[3] = { 3.5f, 2.25f, 1.125f };
        const float reverse[2] = { -2.5f, -1.25f };

        /* Default transmission of a wheeled controller: 5 forward gears, 1 reverse. */
        got = JoltC_WheeledVehicleControllerSettings_GetTransmission(s);
        TEST_ASSERT_NOT_NULL(got, "default transmission not null");
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetGearRatioCount(got) == 5u,
                    "wheeled default has 5 forward gears");
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetReverseGearRatioCount(got) == 1u,
                    "wheeled default has 1 reverse gear");
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetMode(got) == JOLTC_TRANSMISSION_MODE_AUTO,
                    "wheeled default transmission mode is auto");

        /* Push a distinctive transmission in and read it back out. */
        JoltC_VehicleTransmissionSettings_SetMode(ts, JOLTC_TRANSMISSION_MODE_MANUAL);
        JoltC_VehicleTransmissionSettings_SetGearRatios(ts, gears, 3);
        JoltC_VehicleTransmissionSettings_SetReverseGearRatios(ts, reverse, 2);
        JoltC_VehicleTransmissionSettings_SetShiftUpRPM(ts, 4750.5f);
        JoltC_VehicleTransmissionSettings_SetShiftDownRPM(ts, 1234.5f);
        JoltC_WheeledVehicleControllerSettings_SetTransmission(s, ts);

        got = JoltC_WheeledVehicleControllerSettings_GetTransmission(s);
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetMode(got) == JOLTC_TRANSMISSION_MODE_MANUAL,
                    "transmission mode round-trips through the controller settings");
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetGearRatioCount(got) == 3u,
                    "3 forward gears after set");
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetReverseGearRatioCount(got) == 2u,
                    "2 reverse gears after set");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetGearRatio(got, 0), 3.5f, 1e-6f,
                             "gear 0 ratio survives the copy");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetGearRatio(got, 2), 1.125f, 1e-6f,
                             "gear 2 ratio survives the copy");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetReverseGearRatio(got, 1), -1.25f,
                             1e-6f, "reverse gear 1 ratio survives the copy");
        /* Shift up 4750.5 vs shift down 1234.5: exchanging them is visible. */
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetShiftUpRPM(got), 4750.5f, 1e-3f,
                             "shiftUpRPM survives the copy");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetShiftDownRPM(got), 1234.5f, 1e-3f,
                             "shiftDownRPM survives the copy");

        /* The copy is by value: mutating the source afterwards must not be visible. */
        JoltC_VehicleTransmissionSettings_SetShiftUpRPM(ts, 9999.0f);
        got = JoltC_WheeledVehicleControllerSettings_GetTransmission(s);
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetShiftUpRPM(got), 4750.5f, 1e-3f,
                             "SetTransmission copied by value");

        JoltC_WheeledVehicleControllerSettings_SetTransmission(s, NULL);
        JoltC_WheeledVehicleControllerSettings_SetTransmission(NULL, ts);
        TEST_ASSERT(JoltC_WheeledVehicleControllerSettings_GetTransmission(NULL) == NULL,
                    "GetTransmission(NULL) == NULL");

        JoltC_VehicleTransmissionSettings_Destroy(ts);
        JoltC_VehicleControllerSettings_Destroy((JoltC_VehicleControllerSettings*)s);
    }
    TEST_END();

    TEST_BEGIN("WheeledVehicleControllerSettings differentials");
    {
        JoltC_WheeledVehicleControllerSettings* s = JoltC_WheeledVehicleControllerSettings_Create();
        JoltC_VehicleDifferentialSettings d;
        JoltC_VehicleDifferentialSettings out;
        JoltC_VehicleDifferentialSettings pair[2];

        TEST_ASSERT(JoltC_WheeledVehicleControllerSettings_GetDifferentialsCount(s) == 0u,
                    "no differentials by default");

        JoltC_WheeledVehicleControllerSettings_SetDifferentialsCount(s, 3);
        TEST_ASSERT(JoltC_WheeledVehicleControllerSettings_GetDifferentialsCount(s) == 3u,
                    "3 differentials after SetDifferentialsCount");

        /* Wheel indices 2 and 5, far apart so a left/right exchange shows; every
         * float distinct for the same reason. */
        JoltC_VehicleDifferentialSettings_Init(&d);
        d.leftWheel = 2;
        d.rightWheel = 5;
        d.differentialRatio = 3.75f;
        d.leftRightSplit = 0.25f;
        d.limitedSlipRatio = 2.5f;
        d.engineTorqueRatio = 0.625f;
        JoltC_WheeledVehicleControllerSettings_SetDifferential(s, 1, &d);

        JoltC_WheeledVehicleControllerSettings_GetDifferential(s, 1, &out);
        TEST_ASSERT(out.leftWheel == 2, "differential 1 leftWheel == 2");
        TEST_ASSERT(out.rightWheel == 5, "differential 1 rightWheel == 5");
        TEST_ASSERT_FLOAT_EQ(out.differentialRatio, 3.75f, 1e-6f, "differential 1 ratio");
        TEST_ASSERT_FLOAT_EQ(out.leftRightSplit, 0.25f, 1e-6f, "differential 1 leftRightSplit");
        TEST_ASSERT_FLOAT_EQ(out.limitedSlipRatio, 2.5f, 1e-6f, "differential 1 limitedSlipRatio");
        TEST_ASSERT_FLOAT_EQ(out.engineTorqueRatio, 0.625f, 1e-6f, "differential 1 engineTorqueRatio");

        /* Index 1 and only index 1: proves the index reaches the array. */
        JoltC_WheeledVehicleControllerSettings_GetDifferential(s, 0, &out);
        TEST_ASSERT(out.leftWheel == -1, "differential 0 untouched (still default)");
        JoltC_WheeledVehicleControllerSettings_GetDifferential(s, 2, &out);
        TEST_ASSERT(out.leftWheel == -1, "differential 2 untouched (still default)");

        /* Out of range must be a no-op in both directions. */
        out.leftWheel = -999;
        JoltC_WheeledVehicleControllerSettings_GetDifferential(s, 9, &out);
        TEST_ASSERT(out.leftWheel == -999, "GetDifferential out of range leaves the struct");
        JoltC_WheeledVehicleControllerSettings_SetDifferential(s, 9, &d);
        TEST_ASSERT(JoltC_WheeledVehicleControllerSettings_GetDifferentialsCount(s) == 3u,
                    "SetDifferential out of range did not resize");

        /* Bulk replace: the array becomes exactly the two supplied entries, each
         * still distinguishable from the other. */
        JoltC_VehicleDifferentialSettings_Init(&pair[0]);
        JoltC_VehicleDifferentialSettings_Init(&pair[1]);
        pair[0].leftWheel = 0;
        pair[0].rightWheel = 1;
        pair[0].differentialRatio = 4.25f;
        pair[1].leftWheel = 2;
        pair[1].rightWheel = 3;
        pair[1].differentialRatio = 1.75f;
        JoltC_WheeledVehicleControllerSettings_SetDifferentials(s, pair, 2);

        TEST_ASSERT(JoltC_WheeledVehicleControllerSettings_GetDifferentialsCount(s) == 2u,
                    "SetDifferentials replaced the array");
        JoltC_WheeledVehicleControllerSettings_GetDifferential(s, 0, &out);
        TEST_ASSERT(out.rightWheel == 1, "bulk differential 0 rightWheel");
        TEST_ASSERT_FLOAT_EQ(out.differentialRatio, 4.25f, 1e-6f, "bulk differential 0 ratio");
        JoltC_WheeledVehicleControllerSettings_GetDifferential(s, 1, &out);
        TEST_ASSERT(out.leftWheel == 2, "bulk differential 1 leftWheel");
        TEST_ASSERT_FLOAT_EQ(out.differentialRatio, 1.75f, 1e-6f, "bulk differential 1 ratio");

        /* Ratio between differentials, again a ratio that must exceed 1. */
        TEST_ASSERT_FLOAT_EQ(
            JoltC_WheeledVehicleControllerSettings_GetDifferentialLimitedSlipRatio(s), 1.4f, 1e-6f,
            "default differentialLimitedSlipRatio == 1.4");
        JoltC_WheeledVehicleControllerSettings_SetDifferentialLimitedSlipRatio(s, 2.75f);
        TEST_ASSERT_FLOAT_EQ(
            JoltC_WheeledVehicleControllerSettings_GetDifferentialLimitedSlipRatio(s), 2.75f, 1e-6f,
            "differentialLimitedSlipRatio round-trips");

        /* Guards. */
        JoltC_WheeledVehicleControllerSettings_SetDifferentialsCount(NULL, 2);
        JoltC_WheeledVehicleControllerSettings_SetDifferential(s, 0, NULL);
        JoltC_WheeledVehicleControllerSettings_SetDifferentials(s, NULL, 4);
        JoltC_WheeledVehicleControllerSettings_SetDifferentialLimitedSlipRatio(NULL, 2.0f);
        TEST_ASSERT(JoltC_WheeledVehicleControllerSettings_GetDifferentialsCount(NULL) == 0u,
                    "GetDifferentialsCount(NULL) == 0");
        TEST_ASSERT_FLOAT_EQ(
            JoltC_WheeledVehicleControllerSettings_GetDifferentialLimitedSlipRatio(NULL), 0.0f, 1e-6f,
            "GetDifferentialLimitedSlipRatio(NULL) == 0");
        TEST_ASSERT(JoltC_WheeledVehicleControllerSettings_GetDifferentialsCount(s) == 2u,
                    "guarded calls left the differentials alone");

        JoltC_VehicleControllerSettings_Destroy((JoltC_VehicleControllerSettings*)s);
    }
    TEST_END();

    /* ====================================================================== */
    /*  9. VehicleTransmissionSettings — TS_FLOAT block and the gear arrays   */
    /* ====================================================================== */

    TEST_BEGIN("VehicleTransmissionSettings six-float block round-trip");
    {
        JoltC_VehicleTransmissionSettings* ts = JoltC_VehicleTransmissionSettings_Create();
        TEST_ASSERT_NOT_NULL(ts, "VehicleTransmissionSettings not null");

        /* Defaults. switchTime 0.5 and switchLatency 0.5 are equal upstream, so they
         * cannot tell each other apart here — clutchReleaseTime 0.3 can, and the
         * round-trip below uses three distinct values to cover all three. */
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetSwitchTime(ts), 0.5f, 1e-6f,
                             "default switchTime == 0.5 s");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetClutchReleaseTime(ts), 0.3f, 1e-6f,
                             "default clutchReleaseTime == 0.3 s");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetSwitchLatency(ts), 0.5f, 1e-6f,
                             "default switchLatency == 0.5 s");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetShiftUpRPM(ts), 4000.0f, 1e-3f,
                             "default shiftUpRPM == 4000");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetShiftDownRPM(ts), 2000.0f, 1e-3f,
                             "default shiftDownRPM == 2000");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetClutchStrength(ts), 10.0f, 1e-5f,
                             "default clutchStrength == 10");

        /* Six writes, then six reads. */
        JoltC_VehicleTransmissionSettings_SetSwitchTime(ts, 0.375f);
        JoltC_VehicleTransmissionSettings_SetClutchReleaseTime(ts, 0.1875f);
        JoltC_VehicleTransmissionSettings_SetSwitchLatency(ts, 0.75f);
        JoltC_VehicleTransmissionSettings_SetShiftUpRPM(ts, 4750.5f);
        JoltC_VehicleTransmissionSettings_SetShiftDownRPM(ts, 1234.5f);
        JoltC_VehicleTransmissionSettings_SetClutchStrength(ts, 12.75f);

        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetSwitchTime(ts), 0.375f, 1e-6f,
                             "switchTime round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetClutchReleaseTime(ts), 0.1875f, 1e-6f,
                             "clutchReleaseTime round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetSwitchLatency(ts), 0.75f, 1e-6f,
                             "switchLatency round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetShiftUpRPM(ts), 4750.5f, 1e-3f,
                             "shiftUpRPM round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetShiftDownRPM(ts), 1234.5f, 1e-3f,
                             "shiftDownRPM round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetClutchStrength(ts), 12.75f, 1e-5f,
                             "clutchStrength round-trips");

        JoltC_VehicleTransmissionSettings_SetMode(ts, JOLTC_TRANSMISSION_MODE_MANUAL);
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetMode(ts) == JOLTC_TRANSMISSION_MODE_MANUAL,
                    "transmission mode manual round-trips");

        JoltC_VehicleTransmissionSettings_Destroy(ts);
    }
    TEST_END();

    TEST_BEGIN("VehicleTransmissionSettings gear ratio arrays");
    {
        JoltC_VehicleTransmissionSettings* ts = JoltC_VehicleTransmissionSettings_Create();
        const float gears[3] = { 3.5f, 2.25f, 1.125f };
        const float reverse[2] = { -2.5f, -1.25f };

        /* Forward ratios are positive and reverse ratios negative — a sign
         * convention the C API neither documents nor enforces, though the controller
         * asserts it. Keeping the signs right here is what makes these values usable. */
        JoltC_VehicleTransmissionSettings_SetGearRatios(ts, gears, 3);
        JoltC_VehicleTransmissionSettings_SetReverseGearRatios(ts, reverse, 2);

        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetGearRatioCount(ts) == 3u, "3 forward gears");
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetReverseGearRatioCount(ts) == 2u,
                    "2 reverse gears");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetGearRatio(ts, 0), 3.5f, 1e-6f,
                             "forward gear 0 == 3.5");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetGearRatio(ts, 1), 2.25f, 1e-6f,
                             "forward gear 1 == 2.25");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetGearRatio(ts, 2), 1.125f, 1e-6f,
                             "forward gear 2 == 1.125");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetReverseGearRatio(ts, 0), -2.5f,
                             1e-6f, "reverse gear 0 == -2.5");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetReverseGearRatio(ts, 1), -1.25f,
                             1e-6f, "reverse gear 1 == -1.25");

        /* Out of range reads are 0, not a crash and not the last element. */
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetGearRatio(ts, 7), 0.0f, 1e-6f,
                             "forward gear out of range == 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetReverseGearRatio(ts, 7), 0.0f, 1e-6f,
                             "reverse gear out of range == 0");

        /* Single-gear setters grow the array, zero-filling the gap. The zero fill is
         * the documented-by-code behaviour, so it is pinned here — but note it
         * produces ratios of 0, which WheeledVehicleController asserts against
         * (it requires every forward ratio > 0). Writing gear 4 of a 3-gear box and
         * then building a controller is therefore a debug assert waiting to happen. */
        JoltC_VehicleTransmissionSettings_SetGearRatio(ts, 4, 0.875f);
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetGearRatioCount(ts) == 5u,
                    "SetGearRatio grew the array to 5");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetGearRatio(ts, 4), 0.875f, 1e-6f,
                             "SetGearRatio wrote index 4");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetGearRatio(ts, 3), 0.0f, 1e-6f,
                             "SetGearRatio zero-filled the gap");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetGearRatio(ts, 0), 3.5f, 1e-6f,
                             "SetGearRatio kept the existing gears");

        JoltC_VehicleTransmissionSettings_SetReverseGearRatio(ts, 3, -0.75f);
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetReverseGearRatioCount(ts) == 4u,
                    "SetReverseGearRatio grew the array to 4");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetReverseGearRatio(ts, 3), -0.75f,
                             1e-6f, "SetReverseGearRatio wrote index 3");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetReverseGearRatio(ts, 0), -2.5f,
                             1e-6f, "SetReverseGearRatio kept the existing gears");

        /* Negative index is rejected without resizing. */
        JoltC_VehicleTransmissionSettings_SetGearRatio(ts, -1, 9.0f);
        JoltC_VehicleTransmissionSettings_SetReverseGearRatio(ts, -1, -9.0f);
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetGearRatioCount(ts) == 5u,
                    "negative gear index did not resize");
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetReverseGearRatioCount(ts) == 4u,
                    "negative reverse gear index did not resize");

        /* Null guards. Note GetMode(NULL) reports AUTO, which is indistinguishable
         * from a real automatic gearbox — the only accessor in this header whose
         * failure value is a legal value. */
        JoltC_VehicleTransmissionSettings_SetGearRatios(NULL, gears, 3);
        JoltC_VehicleTransmissionSettings_SetGearRatios(ts, NULL, 3);
        JoltC_VehicleTransmissionSettings_SetReverseGearRatios(NULL, reverse, 2);
        JoltC_VehicleTransmissionSettings_SetReverseGearRatios(ts, NULL, 2);
        JoltC_VehicleTransmissionSettings_SetGearRatio(NULL, 0, 1.0f);
        JoltC_VehicleTransmissionSettings_SetReverseGearRatio(NULL, 0, -1.0f);
        JoltC_VehicleTransmissionSettings_SetMode(NULL, JOLTC_TRANSMISSION_MODE_MANUAL);
        JoltC_VehicleTransmissionSettings_SetSwitchTime(NULL, 1.0f);
        JoltC_VehicleTransmissionSettings_SetClutchReleaseTime(NULL, 1.0f);
        JoltC_VehicleTransmissionSettings_SetSwitchLatency(NULL, 1.0f);
        JoltC_VehicleTransmissionSettings_SetShiftUpRPM(NULL, 1.0f);
        JoltC_VehicleTransmissionSettings_SetShiftDownRPM(NULL, 1.0f);
        JoltC_VehicleTransmissionSettings_SetClutchStrength(NULL, 1.0f);
        JoltC_VehicleTransmissionSettings_Destroy(NULL);

        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetGearRatioCount(NULL) == 0u,
                    "GetGearRatioCount(NULL) == 0");
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetReverseGearRatioCount(NULL) == 0u,
                    "GetReverseGearRatioCount(NULL) == 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetGearRatio(NULL, 0), 0.0f, 1e-6f,
                             "GetGearRatio(NULL) == 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetReverseGearRatio(NULL, 0), 0.0f,
                             1e-6f, "GetReverseGearRatio(NULL) == 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetSwitchTime(NULL), 0.0f, 1e-6f,
                             "GetSwitchTime(NULL) == 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetClutchStrength(NULL), 0.0f, 1e-6f,
                             "GetClutchStrength(NULL) == 0");
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetMode(NULL) == JOLTC_TRANSMISSION_MODE_AUTO,
                    "GetMode(NULL) falls back to AUTO");
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetGearRatioCount(ts) == 5u,
                    "guarded calls left the gear array alone");

        JoltC_VehicleTransmissionSettings_Destroy(ts);
    }
    TEST_END();

    /* ====================================================================== */
    /* 10. MotorcycleControllerSettings — MCS_FLOAT block                     */
    /* ====================================================================== */

    TEST_BEGIN("MotorcycleControllerSettings six-float block round-trip");
    {
        JoltC_MotorcycleControllerSettings* s = JoltC_MotorcycleControllerSettings_Create();
        TEST_ASSERT_NOT_NULL(s, "MotorcycleControllerSettings not null");

        /* Defaults. Spring constant 5000 vs damping 1000 is the pair a swap would
         * hit, and maxLeanAngle is radians (45 degrees), not degrees. */
        TEST_ASSERT_FLOAT_EQ(JoltC_MotorcycleControllerSettings_GetMaxLeanAngle(s), 0.78539816f,
                             1e-5f, "default maxLeanAngle == 45 deg in radians");
        TEST_ASSERT_FLOAT_EQ(JoltC_MotorcycleControllerSettings_GetLeanSpringConstant(s), 5000.0f,
                             1e-2f, "default leanSpringConstant == 5000");
        TEST_ASSERT_FLOAT_EQ(JoltC_MotorcycleControllerSettings_GetLeanSpringDamping(s), 1000.0f,
                             1e-2f, "default leanSpringDamping == 1000");
        TEST_ASSERT_FLOAT_EQ(
            JoltC_MotorcycleControllerSettings_GetLeanSpringIntegrationCoefficient(s), 0.0f, 1e-6f,
            "default leanSpringIntegrationCoefficient == 0");
        TEST_ASSERT_FLOAT_EQ(
            JoltC_MotorcycleControllerSettings_GetLeanSpringIntegrationCoefficientDecay(s), 4.0f,
            1e-5f, "default leanSpringIntegrationCoefficientDecay == 4");
        TEST_ASSERT_FLOAT_EQ(JoltC_MotorcycleControllerSettings_GetLeanSmoothingFactor(s), 0.8f,
                             1e-6f, "default leanSmoothingFactor == 0.8");

        /* Six writes, then six reads. The coefficient and its decay are the two
         * names close enough to be confused inside a token-pasting macro, so they
         * are set to values that are neither equal nor multiples of each other. */
        JoltC_MotorcycleControllerSettings_SetMaxLeanAngle(s, 0.65f);
        JoltC_MotorcycleControllerSettings_SetLeanSpringConstant(s, 6250.5f);
        JoltC_MotorcycleControllerSettings_SetLeanSpringDamping(s, 1375.25f);
        JoltC_MotorcycleControllerSettings_SetLeanSpringIntegrationCoefficient(s, 3.75f);
        JoltC_MotorcycleControllerSettings_SetLeanSpringIntegrationCoefficientDecay(s, 5.5f);
        JoltC_MotorcycleControllerSettings_SetLeanSmoothingFactor(s, 0.375f);

        TEST_ASSERT_FLOAT_EQ(JoltC_MotorcycleControllerSettings_GetMaxLeanAngle(s), 0.65f, 1e-6f,
                             "maxLeanAngle round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_MotorcycleControllerSettings_GetLeanSpringConstant(s), 6250.5f,
                             1e-2f, "leanSpringConstant round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_MotorcycleControllerSettings_GetLeanSpringDamping(s), 1375.25f,
                             1e-2f, "leanSpringDamping round-trips");
        TEST_ASSERT_FLOAT_EQ(
            JoltC_MotorcycleControllerSettings_GetLeanSpringIntegrationCoefficient(s), 3.75f, 1e-6f,
            "leanSpringIntegrationCoefficient round-trips");
        TEST_ASSERT_FLOAT_EQ(
            JoltC_MotorcycleControllerSettings_GetLeanSpringIntegrationCoefficientDecay(s), 5.5f,
            1e-6f, "leanSpringIntegrationCoefficientDecay round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_MotorcycleControllerSettings_GetLeanSmoothingFactor(s), 0.375f,
                             1e-6f, "leanSmoothingFactor round-trips");

        /* Null guards for the block. */
        JoltC_MotorcycleControllerSettings_SetMaxLeanAngle(NULL, 1.0f);
        JoltC_MotorcycleControllerSettings_SetLeanSpringConstant(NULL, 1.0f);
        JoltC_MotorcycleControllerSettings_SetLeanSpringDamping(NULL, 1.0f);
        JoltC_MotorcycleControllerSettings_SetLeanSpringIntegrationCoefficient(NULL, 1.0f);
        JoltC_MotorcycleControllerSettings_SetLeanSpringIntegrationCoefficientDecay(NULL, 1.0f);
        JoltC_MotorcycleControllerSettings_SetLeanSmoothingFactor(NULL, 1.0f);
        TEST_ASSERT_FLOAT_EQ(JoltC_MotorcycleControllerSettings_GetMaxLeanAngle(NULL), 0.0f, 1e-6f,
                             "GetMaxLeanAngle(NULL) == 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_MotorcycleControllerSettings_GetLeanSmoothingFactor(NULL), 0.0f,
                             1e-6f, "GetLeanSmoothingFactor(NULL) == 0");

        JoltC_VehicleControllerSettings_Destroy((JoltC_VehicleControllerSettings*)s);
    }
    TEST_END();

    /* A motorcycle is a wheeled vehicle upstream, so the engine, transmission and
     * differential API is reached by casting the handle. Nothing in the C headers
     * says so, and this is the only route to configure a motorcycle's engine. */
    TEST_BEGIN("MotorcycleControllerSettings reuses the wheeled controller API");
    {
        JoltC_MotorcycleControllerSettings* mcs = JoltC_MotorcycleControllerSettings_Create();
        JoltC_WheeledVehicleControllerSettings* as_wheeled =
            (JoltC_WheeledVehicleControllerSettings*)mcs;
        JoltC_VehicleEngineSettings e;

        JoltC_WheeledVehicleControllerSettings_GetEngine(as_wheeled, &e);
        TEST_ASSERT_FLOAT_EQ(e.maxRPM, 6000.0f, 1e-3f,
                             "motorcycle inherits the wheeled engine defaults");

        e.maxTorque = 312.5f;
        e.minRPM = 900.25f;
        e.maxRPM = 9500.75f;
        JoltC_WheeledVehicleControllerSettings_SetEngine(as_wheeled, &e);
        JoltC_WheeledVehicleControllerSettings_GetEngine(as_wheeled, &e);
        TEST_ASSERT_FLOAT_EQ(e.maxTorque, 312.5f, 1e-3f, "motorcycle engine maxTorque round-trips");
        TEST_ASSERT_FLOAT_EQ(e.maxRPM, 9500.75f, 1e-3f, "motorcycle engine maxRPM round-trips");

        JoltC_WheeledVehicleControllerSettings_SetDifferentialsCount(as_wheeled, 1);
        TEST_ASSERT(JoltC_WheeledVehicleControllerSettings_GetDifferentialsCount(as_wheeled) == 1u,
                    "motorcycle takes one differential");

        /* The lean properties must be untouched by all of that. */
        TEST_ASSERT_FLOAT_EQ(JoltC_MotorcycleControllerSettings_GetLeanSpringConstant(mcs), 5000.0f,
                             1e-2f, "lean spring constant survived the wheeled-API writes");

        JoltC_VehicleControllerSettings_Destroy((JoltC_VehicleControllerSettings*)mcs);
    }
    TEST_END();

    /* ====================================================================== */
    /* 11. TrackedVehicleControllerSettings                                   */
    /* ====================================================================== */

    /* Upstream gives the tracked controller its own engine and gearbox defaults
     * (tuned for a tank), which is what makes this test able to prove the two Create
     * functions really build different types. If TrackedVehicleControllerSettings_Create
     * were ever repaired into constructing a WheeledVehicleControllerSettings — a
     * plausible copy/paste during a bump, and one that compiles and runs — the RPM
     * limits and gear counts below are what would notice. */
    TEST_BEGIN("TrackedVehicleControllerSettings has tracked-specific defaults");
    {
        JoltC_TrackedVehicleControllerSettings* s = JoltC_TrackedVehicleControllerSettings_Create();
        const JoltC_VehicleTransmissionSettings* ts;
        JoltC_VehicleEngineSettings e;

        TEST_ASSERT_NOT_NULL(s, "TrackedVehicleControllerSettings not null");

        JoltC_TrackedVehicleControllerSettings_GetEngine(s, &e);
        TEST_ASSERT_FLOAT_EQ(e.minRPM, 500.0f, 1e-3f, "tracked default minRPM == 500 (wheeled: 1000)");
        TEST_ASSERT_FLOAT_EQ(e.maxRPM, 4000.0f, 1e-3f, "tracked default maxRPM == 4000 (wheeled: 6000)");
        TEST_ASSERT_FLOAT_EQ(e.maxTorque, 500.0f, 1e-3f, "tracked default maxTorque == 500");
        TEST_ASSERT(e.normalizedTorque == NULL, "tracked engine curve not surfaced");

        ts = JoltC_TrackedVehicleControllerSettings_GetTransmission(s);
        TEST_ASSERT_NOT_NULL(ts, "tracked transmission not null");
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetGearRatioCount(ts) == 4u,
                    "tracked default has 4 forward gears (wheeled: 5)");
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetReverseGearRatioCount(ts) == 2u,
                    "tracked default has 2 reverse gears (wheeled: 1)");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetShiftUpRPM(ts), 3500.0f, 1e-3f,
                             "tracked default shiftUpRPM == 3500");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetShiftDownRPM(ts), 1000.0f, 1e-3f,
                             "tracked default shiftDownRPM == 1000");

        JoltC_VehicleControllerSettings_Destroy((JoltC_VehicleControllerSettings*)s);
    }
    TEST_END();

    TEST_BEGIN("TrackedVehicleControllerSettings engine / transmission round-trip");
    {
        JoltC_TrackedVehicleControllerSettings* s = JoltC_TrackedVehicleControllerSettings_Create();
        JoltC_VehicleTransmissionSettings* mine = JoltC_VehicleTransmissionSettings_Create();
        const JoltC_VehicleTransmissionSettings* got;
        const float gears[2] = { 5.25f, 1.375f };
        JoltC_VehicleEngineSettings e;

        JoltC_VehicleEngineSettings_Init(&e);
        e.maxTorque = 480.5f;
        e.minRPM = 625.25f;
        e.maxRPM = 3750.75f;
        e.inertia = 0.8125f;
        e.angularDamping = 0.15625f;
        JoltC_TrackedVehicleControllerSettings_SetEngine(s, &e);

        e.maxTorque = -999.0f;
        e.minRPM = -999.0f;
        e.maxRPM = -999.0f;
        e.inertia = -999.0f;
        e.angularDamping = -999.0f;
        JoltC_TrackedVehicleControllerSettings_GetEngine(s, &e);

        TEST_ASSERT_FLOAT_EQ(e.maxTorque, 480.5f, 1e-3f, "tracked engine maxTorque round-trips");
        TEST_ASSERT_FLOAT_EQ(e.minRPM, 625.25f, 1e-3f, "tracked engine minRPM round-trips");
        TEST_ASSERT_FLOAT_EQ(e.maxRPM, 3750.75f, 1e-3f, "tracked engine maxRPM round-trips");
        TEST_ASSERT_FLOAT_EQ(e.inertia, 0.8125f, 1e-6f, "tracked engine inertia round-trips");
        TEST_ASSERT_FLOAT_EQ(e.angularDamping, 0.15625f, 1e-6f,
                             "tracked engine angularDamping round-trips");

        JoltC_VehicleTransmissionSettings_SetMode(mine, JOLTC_TRANSMISSION_MODE_MANUAL);
        JoltC_VehicleTransmissionSettings_SetGearRatios(mine, gears, 2);
        JoltC_VehicleTransmissionSettings_SetClutchStrength(mine, 22.5f);
        JoltC_TrackedVehicleControllerSettings_SetTransmission(s, mine);

        got = JoltC_TrackedVehicleControllerSettings_GetTransmission(s);
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetMode(got) == JOLTC_TRANSMISSION_MODE_MANUAL,
                    "tracked transmission mode round-trips");
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetGearRatioCount(got) == 2u,
                    "tracked transmission gear count round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetGearRatio(got, 0), 5.25f, 1e-6f,
                             "tracked transmission gear 0 round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_VehicleTransmissionSettings_GetClutchStrength(got), 22.5f, 1e-5f,
                             "tracked transmission clutchStrength round-trips");

        /* Guards. */
        JoltC_TrackedVehicleControllerSettings_SetEngine(s, NULL);
        JoltC_TrackedVehicleControllerSettings_SetEngine(NULL, &e);
        JoltC_TrackedVehicleControllerSettings_GetEngine(NULL, &e);
        JoltC_TrackedVehicleControllerSettings_GetEngine(s, NULL);
        JoltC_TrackedVehicleControllerSettings_SetTransmission(s, NULL);
        JoltC_TrackedVehicleControllerSettings_SetTransmission(NULL, mine);
        TEST_ASSERT(JoltC_TrackedVehicleControllerSettings_GetTransmission(NULL) == NULL,
                    "tracked GetTransmission(NULL) == NULL");
        TEST_ASSERT_FLOAT_EQ(e.maxTorque, 480.5f, 1e-3f, "guarded calls left the engine alone");

        JoltC_VehicleTransmissionSettings_Destroy(mine);
        JoltC_VehicleControllerSettings_Destroy((JoltC_VehicleControllerSettings*)s);
        JoltC_VehicleControllerSettings_Destroy(NULL);
    }
    TEST_END();

    /* ====================================================================== */
    /* 12. Collision testers                                                  */
    /* ====================================================================== */

    /* Three testers with different constructor shapes; the layer accessor is shared,
     * so it is checked on each to prove the base cast holds for all three. */
    TEST_BEGIN("VehicleCollisionTester sphere / cylinder create and layer");
    {
        JoltC_ObjectLayer staticLayer = (JoltC_ObjectLayer)OBJ_LAYER_STATIC;
        JoltC_ObjectLayer dynamicLayer = (JoltC_ObjectLayer)OBJ_LAYER_DYNAMIC;
        JoltC_Vec3 up = { 0.0f, 1.0f, 0.0f };
        JoltC_VehicleCollisionTester* sphere;
        JoltC_VehicleCollisionTester* cylinder;

        /* maxSlopeAngle is in radians; radius in metres. */
        sphere = JoltC_VehicleCollisionTesterCastSphere_Create(dynamicLayer, 0.375f, up, 0.75f);
        TEST_ASSERT_NOT_NULL(sphere, "CastSphere tester not null");
        TEST_ASSERT(JoltC_VehicleCollisionTester_GetObjectLayer(sphere) == dynamicLayer,
                    "CastSphere kept its object layer");
        JoltC_VehicleCollisionTester_SetObjectLayer(sphere, staticLayer);
        TEST_ASSERT(JoltC_VehicleCollisionTester_GetObjectLayer(sphere) == staticLayer,
                    "SetObjectLayer took effect");

        /* Cylinder takes a convex radius FRACTION, not a radius. */
        cylinder = JoltC_VehicleCollisionTesterCastCylinder_Create(dynamicLayer, 0.125f);
        TEST_ASSERT_NOT_NULL(cylinder, "CastCylinder tester not null");
        TEST_ASSERT(JoltC_VehicleCollisionTester_GetObjectLayer(cylinder) == dynamicLayer,
                    "CastCylinder kept its object layer");

        /* The two testers are independent objects. */
        TEST_ASSERT(JoltC_VehicleCollisionTester_GetObjectLayer(sphere) == staticLayer,
                    "cylinder creation did not disturb the sphere tester");

        JoltC_VehicleCollisionTester_SetObjectLayer(NULL, staticLayer);
        JoltC_VehicleCollisionTester_Destroy(NULL);
        TEST_ASSERT(JoltC_VehicleCollisionTester_GetObjectLayer(NULL) == JOLTC_OBJECT_LAYER_INVALID,
                    "GetObjectLayer(NULL) == invalid layer");

        JoltC_VehicleCollisionTester_Destroy(cylinder);
        JoltC_VehicleCollisionTester_Destroy(sphere);
    }
    TEST_END();
}
