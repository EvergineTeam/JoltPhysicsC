/* JoltC Test Suite — vehicle.h API tests (wheels, engine, transmission, constraints)
 * SPDX-License-Identifier: MIT
 */

#include "test_common.h"

void run_vehicle_tests(void)
{
    /* test_linear_curve */
    TEST_BEGIN("LinearCurve create, add points, min/max/value");
    {
        JoltC_LinearCurve* curve = JoltC_LinearCurve_Create();
        TEST_ASSERT_NOT_NULL(curve, "LinearCurve not null");

        JoltC_LinearCurve_AddPoint(curve, 0.0f, 0.0f);
        JoltC_LinearCurve_AddPoint(curve, 1.0f, 100.0f);
        JoltC_LinearCurve_Sort(curve);

        TEST_ASSERT(JoltC_LinearCurve_GetPointCount(curve) == 2, "2 points");
        TEST_ASSERT_FLOAT_EQ(JoltC_LinearCurve_GetMinX(curve), 0.0f, 0.01f, "minX == 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_LinearCurve_GetMaxX(curve), 1.0f, 0.01f, "maxX == 1");

        float v = JoltC_LinearCurve_GetValue(curve, 0.5f);
        TEST_ASSERT_FLOAT_EQ(v, 50.0f, 1.0f, "value at 0.5 ~= 50");

        JoltC_LinearCurve_Destroy(curve);
    }
    TEST_END();

    /* test_wheel_settings */
    TEST_BEGIN("WheelSettings create, set/get radius/width");
    {
        JoltC_WheelSettings* ws = JoltC_WheelSettings_Create();
        TEST_ASSERT_NOT_NULL(ws, "WheelSettings not null");

        JoltC_WheelSettings_SetRadius(ws, 0.35f);
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettings_GetRadius(ws), 0.35f, 0.01f, "radius == 0.35");

        JoltC_WheelSettings_SetWidth(ws, 0.2f);
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettings_GetWidth(ws), 0.2f, 0.01f, "width == 0.2");

        JoltC_WheelSettings_Destroy(ws);
    }
    TEST_END();

    /* test_wheel_settings_wv */
    TEST_BEGIN("WheelSettingsWV create, set/get steer angle");
    {
        JoltC_WheelSettingsWV* ws = JoltC_WheelSettingsWV_Create();
        TEST_ASSERT_NOT_NULL(ws, "WheelSettingsWV not null");

        JoltC_WheelSettingsWV_SetMaxSteerAngle(ws, 0.5f);
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsWV_GetMaxSteerAngle(ws), 0.5f, 0.01f, "steer == 0.5");

        JoltC_WheelSettingsWV_SetMaxBrakeTorque(ws, 500.0f);
        TEST_ASSERT_FLOAT_EQ(JoltC_WheelSettingsWV_GetMaxBrakeTorque(ws), 500.0f, 0.1f, "brake == 500");

        /* WheelSettingsWV is freed as part of vehicle constraint; manual destroy not exposed.
         * Just verify the API calls worked. We'll let it leak in this test. */
    }
    TEST_END();

    /* test_wheeled_vehicle_controller_settings */
    TEST_BEGIN("WheeledVehicleControllerSettings create");
    {
        JoltC_WheeledVehicleControllerSettings* s = JoltC_WheeledVehicleControllerSettings_Create();
        TEST_ASSERT_NOT_NULL(s, "WheeledVehicleControllerSettings not null");

        /* Get engine defaults */
        JoltC_VehicleEngineSettings engine;
        JoltC_WheeledVehicleControllerSettings_GetEngine(s, &engine);
        TEST_ASSERT(engine.maxRPM > 0.0f, "engine maxRPM > 0");

        /* Set engine */
        engine.maxTorque = 600.0f;
        JoltC_WheeledVehicleControllerSettings_SetEngine(s, &engine);

        JoltC_VehicleEngineSettings engine2;
        JoltC_WheeledVehicleControllerSettings_GetEngine(s, &engine2);
        TEST_ASSERT_FLOAT_EQ(engine2.maxTorque, 600.0f, 0.1f, "maxTorque == 600 after set");

        JoltC_VehicleControllerSettings_Destroy((JoltC_VehicleControllerSettings*)s);
    }
    TEST_END();

    /* test_tracked_vehicle_controller_settings */
    TEST_BEGIN("TrackedVehicleControllerSettings create");
    {
        JoltC_TrackedVehicleControllerSettings* s = JoltC_TrackedVehicleControllerSettings_Create();
        TEST_ASSERT_NOT_NULL(s, "TrackedVehicleControllerSettings not null");

        JoltC_VehicleEngineSettings engine;
        JoltC_TrackedVehicleControllerSettings_GetEngine(s, &engine);
        TEST_ASSERT(engine.maxRPM > 0.0f, "tracked engine maxRPM > 0");

        JoltC_VehicleControllerSettings_Destroy((JoltC_VehicleControllerSettings*)s);
    }
    TEST_END();

    /* test_vehicle_transmission_settings */
    TEST_BEGIN("VehicleTransmissionSettings create, set/get mode");
    {
        JoltC_VehicleTransmissionSettings* ts = JoltC_VehicleTransmissionSettings_Create();
        TEST_ASSERT_NOT_NULL(ts, "VehicleTransmissionSettings not null");

        JoltC_VehicleTransmissionSettings_SetMode(ts, JOLTC_TRANSMISSION_MODE_AUTO);
        TEST_ASSERT(JoltC_VehicleTransmissionSettings_GetMode(ts) == JOLTC_TRANSMISSION_MODE_AUTO, "mode == AUTO");

        uint32_t gears = JoltC_VehicleTransmissionSettings_GetGearRatioCount(ts);
        TEST_ASSERT(gears > 0, "default gear count > 0");

        JoltC_VehicleTransmissionSettings_Destroy(ts);
    }
    TEST_END();

    /* test_vehicle_collision_tester_ray */
    TEST_BEGIN("VehicleCollisionTesterRay create, set/get layer");
    {
        JoltC_Vec3 up = { 0.0f, 1.0f, 0.0f };
        JoltC_VehicleCollisionTester* tester = JoltC_VehicleCollisionTesterRay_Create(OBJ_LAYER_STATIC, up, 1.2f);
        TEST_ASSERT_NOT_NULL(tester, "CollisionTesterRay not null");

        JoltC_ObjectLayer ol = JoltC_VehicleCollisionTester_GetObjectLayer(tester);
        TEST_ASSERT(ol == OBJ_LAYER_STATIC, "object layer == STATIC");

        JoltC_VehicleCollisionTester_Destroy(tester);
    }
    TEST_END();
}
