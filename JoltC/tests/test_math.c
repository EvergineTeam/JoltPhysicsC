/* JoltC Test Suite -- math.h API tests (Vec3, Quat, Mat44)
 * SPDX-License-Identifier: MIT
 */

#include "test_common.h"

#define PI_F 3.14159265358979323846f

void run_math_tests(void)
{
    /* test_vec3_axis_helpers */
    TEST_BEGIN("Vec3 axis helpers");
    {
        JoltC_Vec3 x, y, z;
        JoltC_Vec3_AxisX(&x);
        JoltC_Vec3_AxisY(&y);
        JoltC_Vec3_AxisZ(&z);
        TEST_ASSERT_FLOAT_EQ(x.x, 1.0f, 0.001f, "AxisX.x == 1");
        TEST_ASSERT_FLOAT_EQ(x.y, 0.0f, 0.001f, "AxisX.y == 0");
        TEST_ASSERT_FLOAT_EQ(x.z, 0.0f, 0.001f, "AxisX.z == 0");
        TEST_ASSERT_FLOAT_EQ(y.y, 1.0f, 0.001f, "AxisY.y == 1");
        TEST_ASSERT_FLOAT_EQ(z.z, 1.0f, 0.001f, "AxisZ.z == 1");
    }
    TEST_END();

    /* test_vec3_add_subtract */
    TEST_BEGIN("Vec3 add / subtract");
    {
        JoltC_Vec3 a = { 1.0f, 2.0f, 3.0f };
        JoltC_Vec3 b = { 4.0f, 5.0f, 6.0f };
        JoltC_Vec3 sum, diff;
        JoltC_Vec3_Add(&a, &b, &sum);
        JoltC_Vec3_Subtract(&a, &b, &diff);
        TEST_ASSERT_FLOAT_EQ(sum.x, 5.0f, 0.001f, "sum.x == 5");
        TEST_ASSERT_FLOAT_EQ(sum.y, 7.0f, 0.001f, "sum.y == 7");
        TEST_ASSERT_FLOAT_EQ(sum.z, 9.0f, 0.001f, "sum.z == 9");
        TEST_ASSERT_FLOAT_EQ(diff.x, -3.0f, 0.001f, "diff.x == -3");
        TEST_ASSERT_FLOAT_EQ(diff.y, -3.0f, 0.001f, "diff.y == -3");
    }
    TEST_END();

    /* test_vec3_multiply_scalar */
    TEST_BEGIN("Vec3 multiply scalar");
    {
        JoltC_Vec3 v = { 2.0f, 3.0f, 4.0f };
        JoltC_Vec3 r;
        JoltC_Vec3_MultiplyScalar(&v, 2.0f, &r);
        TEST_ASSERT_FLOAT_EQ(r.x, 4.0f, 0.001f, "r.x == 4");
        TEST_ASSERT_FLOAT_EQ(r.y, 6.0f, 0.001f, "r.y == 6");
        TEST_ASSERT_FLOAT_EQ(r.z, 8.0f, 0.001f, "r.z == 8");
    }
    TEST_END();

    /* test_vec3_length */
    TEST_BEGIN("Vec3 length");
    {
        JoltC_Vec3 v = { 3.0f, 4.0f, 0.0f };
        float len = JoltC_Vec3_Length(&v);
        TEST_ASSERT_FLOAT_EQ(len, 5.0f, 0.001f, "length(3,4,0) == 5");
    }
    TEST_END();

    /* test_vec3_normalize */
    TEST_BEGIN("Vec3 normalize");
    {
        JoltC_Vec3 v = { 3.0f, 4.0f, 0.0f };
        JoltC_Vec3 n;
        JoltC_Vec3_Normalized(&v, &n);
        TEST_ASSERT_FLOAT_EQ(n.x, 0.6f, 0.001f, "n.x ~ 0.6");
        TEST_ASSERT_FLOAT_EQ(n.y, 0.8f, 0.001f, "n.y ~ 0.8");
        TEST_ASSERT_FLOAT_EQ(n.z, 0.0f, 0.001f, "n.z ~ 0");
    }
    TEST_END();

    /* test_vec3_dot_product */
    TEST_BEGIN("Vec3 dot product");
    {
        JoltC_Vec3 x = { 1.0f, 0.0f, 0.0f };
        JoltC_Vec3 y = { 0.0f, 1.0f, 0.0f };
        float dot_xy, dot_xx;
        JoltC_Vec3_DotProduct(&x, &y, &dot_xy);
        JoltC_Vec3_DotProduct(&x, &x, &dot_xx);
        TEST_ASSERT_FLOAT_EQ(dot_xy, 0.0f, 0.001f, "x.y == 0");
        TEST_ASSERT_FLOAT_EQ(dot_xx, 1.0f, 0.001f, "x.x == 1");
    }
    TEST_END();

    /* test_vec3_cross_product */
    TEST_BEGIN("Vec3 cross product (X x Y = Z)");
    {
        JoltC_Vec3 x = { 1.0f, 0.0f, 0.0f };
        JoltC_Vec3 y = { 0.0f, 1.0f, 0.0f };
        JoltC_Vec3 r;
        JoltC_Vec3_Cross(&x, &y, &r);
        TEST_ASSERT_FLOAT_EQ(r.x, 0.0f, 0.001f, "cross.x == 0");
        TEST_ASSERT_FLOAT_EQ(r.y, 0.0f, 0.001f, "cross.y == 0");
        TEST_ASSERT_FLOAT_EQ(r.z, 1.0f, 0.001f, "cross.z == 1");
    }
    TEST_END();

    /* test_vec3_is_close */
    TEST_BEGIN("Vec3 IsClose");
    {
        JoltC_Vec3 a = { 1.0f, 2.0f, 3.0f };
        JoltC_Vec3 b = { 1.0001f, 2.0001f, 3.0001f };
        JoltC_Vec3 c = { 10.0f, 20.0f, 30.0f };
        int close = JoltC_Vec3_IsClose(&a, &b, 0.01f);
        int far = JoltC_Vec3_IsClose(&a, &c, 0.01f);
        TEST_ASSERT(close, "a and b are close");
        TEST_ASSERT(!far, "a and c are not close");
    }
    TEST_END();

    /* test_quat_from_euler_angles */
    TEST_BEGIN("Quat from euler angles (0,0,0) = identity");
    {
        JoltC_Vec3 angles = { 0.0f, 0.0f, 0.0f };
        JoltC_Quat q;
        JoltC_Quat_FromEulerAngles(&angles, &q);
        TEST_ASSERT_FLOAT_EQ(q.x, 0.0f, 0.001f, "q.x == 0");
        TEST_ASSERT_FLOAT_EQ(q.y, 0.0f, 0.001f, "q.y == 0");
        TEST_ASSERT_FLOAT_EQ(q.z, 0.0f, 0.001f, "q.z == 0");
        TEST_ASSERT_FLOAT_EQ(q.w, 1.0f, 0.001f, "q.w == 1");
    }
    TEST_END();

    /* test_quat_rotate_vector */
    TEST_BEGIN("Quat rotate (1,0,0) by 90 deg around Y");
    {
        /* 90 degrees around Y: quat = (0, sin(45), 0, cos(45)) */
        JoltC_Vec3 angles = { 0.0f, PI_F / 2.0f, 0.0f };
        JoltC_Quat q;
        JoltC_Quat_FromEulerAngles(&angles, &q);

        JoltC_Vec3 v = { 1.0f, 0.0f, 0.0f };
        JoltC_Vec3 r;
        JoltC_Quat_Rotate(&q, &v, &r);
        TEST_ASSERT_FLOAT_EQ(r.x, 0.0f, 0.01f, "rotated.x ~ 0");
        TEST_ASSERT_FLOAT_EQ(r.y, 0.0f, 0.01f, "rotated.y ~ 0");
        TEST_ASSERT_FLOAT_EQ(r.z, -1.0f, 0.01f, "rotated.z ~ -1");
    }
    TEST_END();

    /* test_quat_slerp */
    TEST_BEGIN("Quat slerp at t=0 and t=1");
    {
        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_Vec3 angles = { 0.0f, PI_F / 2.0f, 0.0f };
        JoltC_Quat rotated;
        JoltC_Quat_FromEulerAngles(&angles, &rotated);

        JoltC_Quat r0, r1;
        JoltC_Quat_Slerp(&identity, &rotated, 0.0f, &r0);
        JoltC_Quat_Slerp(&identity, &rotated, 1.0f, &r1);
        TEST_ASSERT_FLOAT_EQ(r0.w, identity.w, 0.01f, "slerp(0) ~ identity");
        /* Compare all components with tolerance -- slerp may flip sign */
        float dot = r1.x*rotated.x + r1.y*rotated.y + r1.z*rotated.z + r1.w*rotated.w;
        TEST_ASSERT(fabsf(dot) > 0.99f, "slerp(1) ~ rotated");
    }
    TEST_END();

    /* test_quat_inverse */
    TEST_BEGIN("Quat q * inverse(q) ~ identity");
    {
        JoltC_Vec3 angles = { 0.3f, 0.5f, 0.7f };
        JoltC_Quat q, inv, result;
        JoltC_Quat_FromEulerAngles(&angles, &q);
        JoltC_Quat_Inversed(&q, &inv);
        JoltC_Quat_Multiply(&q, &inv, &result);
        TEST_ASSERT_FLOAT_EQ(result.w, 1.0f, 0.01f, "q*inv.w ~ 1");
        TEST_ASSERT_FLOAT_EQ(result.x, 0.0f, 0.01f, "q*inv.x ~ 0");
        TEST_ASSERT_FLOAT_EQ(result.y, 0.0f, 0.01f, "q*inv.y ~ 0");
        TEST_ASSERT_FLOAT_EQ(result.z, 0.0f, 0.01f, "q*inv.z ~ 0");
    }
    TEST_END();

    /* test_mat44_identity */
    TEST_BEGIN("Mat44 identity diagonal = 1");
    {
        JoltC_Mat44 m;
        JoltC_Mat44_Identity(&m);
        /* column-major: m[0]=1, m[5]=1, m[10]=1, m[15]=1 */
        TEST_ASSERT_FLOAT_EQ(m.m[0], 1.0f, 0.001f, "m[0] == 1");
        TEST_ASSERT_FLOAT_EQ(m.m[5], 1.0f, 0.001f, "m[5] == 1");
        TEST_ASSERT_FLOAT_EQ(m.m[10], 1.0f, 0.001f, "m[10] == 1");
        TEST_ASSERT_FLOAT_EQ(m.m[15], 1.0f, 0.001f, "m[15] == 1");
        TEST_ASSERT_FLOAT_EQ(m.m[1], 0.0f, 0.001f, "m[1] == 0");
        TEST_ASSERT_FLOAT_EQ(m.m[4], 0.0f, 0.001f, "m[4] == 0");
    }
    TEST_END();

    /* test_mat44_multiply_identity */
    TEST_BEGIN("Mat44 Identity * M = M");
    {
        JoltC_Mat44 identity, trans, result;
        JoltC_Mat44_Identity(&identity);
        JoltC_Vec3 t = { 5.0f, 10.0f, 15.0f };
        JoltC_Mat44_Translation(&trans, &t);
        JoltC_Mat44_Multiply(&identity, &trans, &result);
        /* Translation is in column 3 (indices 12, 13, 14) */
        TEST_ASSERT_FLOAT_EQ(result.m[12], 5.0f, 0.001f, "translation.x preserved");
        TEST_ASSERT_FLOAT_EQ(result.m[13], 10.0f, 0.001f, "translation.y preserved");
        TEST_ASSERT_FLOAT_EQ(result.m[14], 15.0f, 0.001f, "translation.z preserved");
    }
    TEST_END();

    /* test_mat44_rotation_translation_round_trip */
    TEST_BEGIN("Mat44 RotationTranslation extract");
    {
        JoltC_Quat q = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_Vec3 t = { 3.0f, 4.0f, 5.0f };
        JoltC_Mat44 m;
        JoltC_Mat44_RotationTranslation(&m, &q, &t);
        JoltC_Vec3 extracted_t;
        JoltC_Mat44_GetTranslation(&m, &extracted_t);
        TEST_ASSERT_FLOAT_EQ(extracted_t.x, 3.0f, 0.001f, "extracted t.x");
        TEST_ASSERT_FLOAT_EQ(extracted_t.y, 4.0f, 0.001f, "extracted t.y");
        TEST_ASSERT_FLOAT_EQ(extracted_t.z, 5.0f, 0.001f, "extracted t.z");
    }
    TEST_END();
}
