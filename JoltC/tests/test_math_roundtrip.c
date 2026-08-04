/* JoltC Test Suite — math.h round-trips, identities and known-answer arithmetic
 * SPDX-License-Identifier: MIT
 *
 * Companion to test_math.c. That file establishes that the math entry points are
 * callable; this one is aimed at the failure mode a by-hand upstream bump
 * produces: code that compiles and is semantically wrong. Two rules follow from
 * that, and every test below obeys them:
 *
 *   1. Every test value is asymmetric — no (1,1,1), no lone (0,0,1). A swapped
 *      component in a converter (the documented toJphVec3 x/y hazard) has to
 *      change a number this file asserts, or the test is worthless.
 *   2. Every expected value is computed by hand and the derivation is stated in
 *      a comment. Nothing here was obtained by running the library.
 *
 * The fixture quaternion is (0.2, -0.4, 0.4, 0.8): exactly unit
 * (0.04+0.16+0.16+0.64 = 1), asymmetric in all four components, and its
 * rotation matrix comes out in exact decimals (see kRotA below), which is what
 * makes hand-computed matrix expectations possible at all.
 */

#include "test_common.h"

void run_math_roundtrip_tests(void);

/* ========================================================================== */
/*  Fixtures                                                                  */
/* ========================================================================== */

/* Asymmetric vector. |a|^2 = 2.25 + 5.0625 + 14.0625 = 21.375 exactly,
 * |a| = sqrt(21.375) = 4.6233117. */
static const JoltC_Vec3  kVecA   = {  1.5f, -2.25f, 3.75f };

/* Second asymmetric vector. |b|^2 = 0.25 + 4 + 1.5625 = 5.8125,
 * |b| = 2.4109127. */
static const JoltC_Vec3  kVecB   = { -0.5f,  2.0f,  1.25f };

/* Translation used throughout. Asymmetric, mixed signs, no repeats. */
static const JoltC_Vec3  kTrans  = { -1.25f, 0.5f,  2.0f  };
static const JoltC_RVec3 kRTrans = { -1.25f, 0.5f,  2.0f  };

/* Asymmetric scale. */
static const JoltC_Vec3  kScale  = {  1.5f, -2.25f, 3.75f };

/* Unit quaternion, exactly: 0.04 + 0.16 + 0.16 + 0.64 = 1.
 * Its axis is (0.2,-0.4,0.4)/0.6 = (1/3, -2/3, 2/3) — the vector part has
 * length exactly 0.6 — and its angle is 2*acos(0.8) = 2*atan(0.75) = 1.2870022. */
static const JoltC_Quat  kQuatA  = { 0.2f, -0.4f, 0.4f, 0.8f };

/* Second unit quaternion: 0.25*4 = 1. Axis (1,1,-1)/sqrt(3), angle 2*pi/3. */
static const JoltC_Quat  kQuatB  = { 0.5f, 0.5f, -0.5f, 0.5f };

static const JoltC_Quat  kQuatId = { 0.0f, 0.0f, 0.0f, 1.0f };

/* Axis of kQuatA, exact: (1/3, -2/3, 2/3). */
#define AXIS_A_X ( 0.33333333f)
#define AXIS_A_Y (-0.66666667f)
#define AXIS_A_Z ( 0.66666667f)

/* Angle of kQuatA: 2*atan(0.6/0.8) = 1.28700221758657 rad. */
#define ANGLE_A  (1.2870022f)

/* Rotation matrix of kQuatA, from R = I + 2w[v]x + 2[v]x^2 written out as
 *   R00 = 1-2(y^2+z^2)   R01 = 2(xy-wz)      R02 = 2(xz+wy)
 *   R10 = 2(xy+wz)       R11 = 1-2(x^2+z^2)  R12 = 2(yz-wx)
 *   R20 = 2(xz-wy)       R21 = 2(yz+wx)      R22 = 1-2(x^2+y^2)
 * with x=0.2 y=-0.4 z=0.4 w=0.8:
 *   xy=-0.08 xz=0.08 yz=-0.16 wx=0.16 wy=-0.32 wz=0.32
 *   R = [ 0.36 -0.80 -0.48 ]
 *       [ 0.48  0.60 -0.64 ]
 *       [ 0.80  0.00  0.60 ]
 * Every column is unit and col0 x col1 = col2, so this is a proper rotation.
 * Stored column-major, which is what JoltC_Mat44.m is documented to be and what
 * test_math.c's translation-at-[12..14] assertion confirms. */
static const JoltC_Mat44 kRotA = { {
     0.36f,  0.48f,  0.80f, 0.0f,   /* column 0 = image of X */
    -0.80f,  0.60f,  0.00f, 0.0f,   /* column 1 = image of Y */
    -0.48f, -0.64f,  0.60f, 0.0f,   /* column 2 = image of Z */
     0.0f,   0.0f,   0.0f,  1.0f
} };

/* kRotA with kTrans in the translation column. */
static const JoltC_Mat44 kRotTransA = { {
     0.36f,  0.48f,  0.80f, 0.0f,
    -0.80f,  0.60f,  0.00f, 0.0f,
    -0.48f, -0.64f,  0.60f, 0.0f,
    -1.25f,  0.5f,   2.0f,  1.0f
} };

/* Inverse of kRotTransA = [ R^T | -R^T t ].
 *   R^T t = (col0.t, col1.t, col2.t)
 *         = (0.36*-1.25 + 0.48*0.5 + 0.80*2.0,
 *            -0.80*-1.25 + 0.60*0.5 + 0.00*2.0,
 *            -0.48*-1.25 + -0.64*0.5 + 0.60*2.0)
 *         = (-0.45+0.24+1.6, 1.0+0.3+0, 0.6-0.32+1.2)
 *         = (1.39, 1.30, 1.48)
 * so the translation column is -(1.39, 1.30, 1.48). */
static const JoltC_Mat44 kInvRotTransA = { {
     0.36f, -0.80f, -0.48f, 0.0f,
     0.48f,  0.60f, -0.64f, 0.0f,
     0.80f,  0.00f,  0.60f, 0.0f,
    -1.39f, -1.30f, -1.48f, 1.0f
} };

/* ========================================================================== */
/*  Local assertion helpers                                                   */
/* ========================================================================== */
#define MR_ASSERT_VEC3(v, ex, ey, ez, eps, label)                   \
    do {                                                            \
        TEST_ASSERT_FLOAT_EQ((v).x, (ex), (eps), label " x");       \
        TEST_ASSERT_FLOAT_EQ((v).y, (ey), (eps), label " y");       \
        TEST_ASSERT_FLOAT_EQ((v).z, (ez), (eps), label " z");       \
    } while (0)

#define MR_ASSERT_QUAT(q, ex, ey, ez, ew, eps, label)               \
    do {                                                            \
        TEST_ASSERT_FLOAT_EQ((q).x, (ex), (eps), label " x");       \
        TEST_ASSERT_FLOAT_EQ((q).y, (ey), (eps), label " y");       \
        TEST_ASSERT_FLOAT_EQ((q).z, (ez), (eps), label " z");       \
        TEST_ASSERT_FLOAT_EQ((q).w, (ew), (eps), label " w");       \
    } while (0)

/* Element-wise matrix comparison. Reports the first-failing index so a single
 * transposed or dropped element is identifiable without printing 16 lines. */
static void mr_assert_mat44(const JoltC_Mat44* actual, const JoltC_Mat44* expected,
                            float eps, const char* label)
{
    char buf[160];
    int i;
    for (i = 0; i < 16; ++i) {
        if (!(fabsf(actual->m[i] - expected->m[i]) < eps)) {
            snprintf(buf, sizeof(buf), "%s: m[%d] is %g, expected %g",
                     label, i, (double)actual->m[i], (double)expected->m[i]);
            TEST_ASSERT(0, buf);
        }
    }
}

/* Builds the unit quaternion for (axis, angle) the textbook way:
 * (axis * sin(angle/2), cos(angle/2)). Uses the wrapper's own Sin/Cos, which
 * are asserted against known values in the first test below. */
static JoltC_Quat mr_axis_angle(float ax, float ay, float az, float angle)
{
    float s = JoltC_Math_Sin(0.5f * angle);
    float c = JoltC_Math_Cos(0.5f * angle);
    JoltC_Quat q;
    q.x = ax * s;
    q.y = ay * s;
    q.z = az * s;
    q.w = c;
    return q;
}

/* ========================================================================== */
/*  Tests                                                                     */
/* ========================================================================== */
void run_math_roundtrip_tests(void)
{
    /* ---------------------------------------------------------------- */
    /*  Math helpers                                                    */
    /* ---------------------------------------------------------------- */

    /* sin(0.35) = 0.34289781, cos(0.35) = 0.93937271,
     * sin(-0.4) = -0.38941834, cos(-0.4) = 0.92106099. */
    TEST_BEGIN("Math Sin/Cos known values");
    {
        TEST_ASSERT_FLOAT_EQ(JoltC_Math_Sin(0.35f),  0.34289781f, 0.0001f, "sin(0.35)");
        TEST_ASSERT_FLOAT_EQ(JoltC_Math_Cos(0.35f),  0.93937271f, 0.0001f, "cos(0.35)");
        TEST_ASSERT_FLOAT_EQ(JoltC_Math_Sin(-0.4f), -0.38941834f, 0.0001f, "sin(-0.4)");
        TEST_ASSERT_FLOAT_EQ(JoltC_Math_Cos(-0.4f),  0.92106099f, 0.0001f, "cos(-0.4)");
        /* Cos is even, Sin is odd — catches the two being swapped. */
        TEST_ASSERT_FLOAT_EQ(JoltC_Math_Cos(0.4f), JoltC_Math_Cos(-0.4f), 0.0001f, "cos even");
        TEST_ASSERT_FLOAT_EQ(JoltC_Math_Sin(0.4f), -JoltC_Math_Sin(-0.4f), 0.0001f, "sin odd");
    }
    TEST_END();

    /* ---------------------------------------------------------------- */
    /*  Vec3 — magnitudes                                               */
    /* ---------------------------------------------------------------- */

    /* |(1.5,-2.25,3.75)|^2 = 2.25 + 5.0625 + 14.0625 = 21.375 (exact),
     * |(1.5,-2.25,3.75)|   = 4.6233117. */
    TEST_BEGIN("Vec3 Length / LengthSquared asymmetric");
    {
        TEST_ASSERT_FLOAT_EQ(JoltC_Vec3_LengthSquared(&kVecA), 21.375f,    0.001f, "lengthSq a");
        TEST_ASSERT_FLOAT_EQ(JoltC_Vec3_Length(&kVecA),        4.6233117f, 0.001f, "length a");
        /* |(-0.5,2,1.25)|^2 = 0.25 + 4 + 1.5625 = 5.8125, |b| = 2.4109127. */
        TEST_ASSERT_FLOAT_EQ(JoltC_Vec3_LengthSquared(&kVecB), 5.8125f,    0.001f, "lengthSq b");
        TEST_ASSERT_FLOAT_EQ(JoltC_Vec3_Length(&kVecB),        2.4109127f, 0.001f, "length b");
    }
    TEST_END();

    /* a / |a| = (1.5, -2.25, 3.75) / 4.6233117
     *         = (0.3244435, -0.4866652, 0.8111087) */
    TEST_BEGIN("Vec3 Normalized asymmetric known values");
    {
        JoltC_Vec3 n;
        JoltC_Vec3_Normalized(&kVecA, &n);
        MR_ASSERT_VEC3(n, 0.3244435f, -0.4866652f, 0.8111087f, 0.001f, "normalized a");
        TEST_ASSERT(JoltC_Vec3_IsNormalized(&n, 0.0001f), "result is normalized");
        TEST_ASSERT(!JoltC_Vec3_IsNormalized(&kVecA, 0.0001f), "input is not normalized");
    }
    TEST_END();

    /* (0.2,-0.4,0.4) has length exactly 0.6, so the normalized axis is
     * exactly (1/3, -2/3, 2/3) — no rounding in the expectation. */
    TEST_BEGIN("Vec3 Normalize == Normalized (exact axis)");
    {
        JoltC_Vec3 v = { 0.2f, -0.4f, 0.4f };
        JoltC_Vec3 n1, n2;
        JoltC_Vec3_Normalized(&v, &n1);
        JoltC_Vec3_Normalize(&v, &n2);
        MR_ASSERT_VEC3(n1, AXIS_A_X, AXIS_A_Y, AXIS_A_Z, 0.001f, "Normalized");
        MR_ASSERT_VEC3(n2, AXIS_A_X, AXIS_A_Y, AXIS_A_Z, 0.001f, "Normalize");
    }
    TEST_END();

    /* ---------------------------------------------------------------- */
    /*  Vec3 — component-wise arithmetic                                */
    /* ---------------------------------------------------------------- */

    TEST_BEGIN("Vec3 Negate / Abs with mixed signs");
    {
        JoltC_Vec3 neg, absv, absneg;
        JoltC_Vec3_Negate(&kVecA, &neg);
        JoltC_Vec3_Abs(&kVecA, &absv);
        JoltC_Vec3_Abs(&neg, &absneg);
        MR_ASSERT_VEC3(neg,   -1.5f, 2.25f, -3.75f, 0.001f, "negate");
        MR_ASSERT_VEC3(absv,   1.5f, 2.25f,  3.75f, 0.001f, "abs");
        MR_ASSERT_VEC3(absneg, 1.5f, 2.25f,  3.75f, 0.001f, "abs of negate");
        /* Negating twice is the identity. */
        JoltC_Vec3 back;
        JoltC_Vec3_Negate(&neg, &back);
        MR_ASSERT_VEC3(back, 1.5f, -2.25f, 3.75f, 0.001f, "double negate");
    }
    TEST_END();

    /* a*b = (1.5*-0.5, -2.25*2, 3.75*1.25) = (-0.75, -4.5, 4.6875)
     * a/b = (1.5/-0.5, -2.25/2, 3.75/1.25) = (-3.0, -1.125, 3.0) */
    TEST_BEGIN("Vec3 component-wise Multiply / Divide");
    {
        JoltC_Vec3 prod, quot;
        JoltC_Vec3_Multiply(&kVecA, &kVecB, &prod);
        JoltC_Vec3_Divide(&kVecA, &kVecB, &quot);
        MR_ASSERT_VEC3(prod, -0.75f, -4.5f,   4.6875f, 0.001f, "a*b");
        MR_ASSERT_VEC3(quot, -3.0f,  -1.125f, 3.0f,    0.001f, "a/b");
        /* (a*b)/b == a. */
        JoltC_Vec3 back;
        JoltC_Vec3_Divide(&prod, &kVecB, &back);
        MR_ASSERT_VEC3(back, 1.5f, -2.25f, 3.75f, 0.001f, "(a*b)/b");
    }
    TEST_END();

    /* a/2.5 = (0.6, -0.9, 1.5); a/-0.5 = (-3, 4.5, -7.5) */
    TEST_BEGIN("Vec3 DivideScalar incl. negative divisor");
    {
        JoltC_Vec3 r1, r2;
        JoltC_Vec3_DivideScalar(&kVecA,  2.5f, &r1);
        JoltC_Vec3_DivideScalar(&kVecA, -0.5f, &r2);
        MR_ASSERT_VEC3(r1,  0.6f, -0.9f, 1.5f,   0.001f, "a/2.5");
        MR_ASSERT_VEC3(r2, -3.0f,  4.5f, -7.5f,  0.001f, "a/-0.5");
    }
    TEST_END();

    /* ---------------------------------------------------------------- */
    /*  Vec3 — products                                                 */
    /* ---------------------------------------------------------------- */

    /* a x b with a=(1.5,-2.25,3.75), b=(-0.5,2,1.25):
     *   x = ay*bz - az*by = -2.8125 - 7.5   = -10.3125
     *   y = az*bx - ax*bz = -1.875  - 1.875 = -3.75
     *   z = ax*by - ay*bx =  3.0    - 1.125 =  1.875
     * All three components differ in magnitude, so any permutation shows up. */
    TEST_BEGIN("Vec3 Cross asymmetric known answer");
    {
        JoltC_Vec3 c;
        JoltC_Vec3_Cross(&kVecA, &kVecB, &c);
        MR_ASSERT_VEC3(c, -10.3125f, -3.75f, 1.875f, 0.001f, "a x b");
    }
    TEST_END();

    /* b x a = -(a x b), and a x b is orthogonal to both operands:
     *   (a x b).a = -15.46875 + 8.4375 + 7.03125 = 0
     *   (a x b).b = 5.15625 - 7.5 + 2.34375     = 0 */
    TEST_BEGIN("Vec3 Cross anticommutative and orthogonal");
    {
        JoltC_Vec3 ab, ba;
        float d1, d2;
        JoltC_Vec3_Cross(&kVecA, &kVecB, &ab);
        JoltC_Vec3_Cross(&kVecB, &kVecA, &ba);
        MR_ASSERT_VEC3(ba, 10.3125f, 3.75f, -1.875f, 0.001f, "b x a");
        JoltC_Vec3_DotProduct(&ab, &kVecA, &d1);
        JoltC_Vec3_DotProduct(&ab, &kVecB, &d2);
        TEST_ASSERT_FLOAT_EQ(d1, 0.0f, 0.001f, "(a x b).a == 0");
        TEST_ASSERT_FLOAT_EQ(d2, 0.0f, 0.001f, "(a x b).b == 0");
        /* A vector crossed with itself is zero. */
        JoltC_Vec3 self;
        JoltC_Vec3_Cross(&kVecA, &kVecA, &self);
        TEST_ASSERT(JoltC_Vec3_IsNearZero(&self, 1.0e-6f), "a x a == 0");
    }
    TEST_END();

    /* a.b = 1.5*-0.5 + -2.25*2 + 3.75*1.25 = -0.75 - 4.5 + 4.6875 = -0.5625
     * a.a = 21.375 (= LengthSquared) */
    TEST_BEGIN("Vec3 DotProduct asymmetric known answer");
    {
        float ab, ba, aa;
        JoltC_Vec3_DotProduct(&kVecA, &kVecB, &ab);
        JoltC_Vec3_DotProduct(&kVecB, &kVecA, &ba);
        JoltC_Vec3_DotProduct(&kVecA, &kVecA, &aa);
        TEST_ASSERT_FLOAT_EQ(ab, -0.5625f, 0.001f, "a.b");
        TEST_ASSERT_FLOAT_EQ(ba, -0.5625f, 0.001f, "b.a (symmetric)");
        TEST_ASSERT_FLOAT_EQ(aa, 21.375f,  0.001f, "a.a == |a|^2");
    }
    TEST_END();

    /* ---------------------------------------------------------------- */
    /*  Vec3 — predicates and edge cases                                */
    /* ---------------------------------------------------------------- */

    /* IsNearZero's parameter is a *squared* distance (maxDistSq).
     * (1e-7,-1e-7,1e-7) has lengthSq 3e-14 < 1e-12. */
    TEST_BEGIN("Vec3 IsNearZero edge cases");
    {
        JoltC_Vec3 zero = { 0.0f, 0.0f, 0.0f };
        JoltC_Vec3 tiny = { 1.0e-7f, -1.0e-7f, 1.0e-7f };
        TEST_ASSERT(JoltC_Vec3_IsNearZero(&zero, 1.0e-12f), "zero is near zero");
        TEST_ASSERT(JoltC_Vec3_IsNearZero(&tiny, 1.0e-12f), "1e-7 vector is near zero");
        TEST_ASSERT(!JoltC_Vec3_IsNearZero(&kVecA, 1.0e-12f), "a is not near zero");
        /* Zero-length vector: length and lengthSq are well defined even though
         * normalising it is not, so those are what we pin down here. */
        TEST_ASSERT_FLOAT_EQ(JoltC_Vec3_Length(&zero),        0.0f, 0.001f, "|zero| == 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_Vec3_LengthSquared(&zero), 0.0f, 0.001f, "|zero|^2 == 0");
        TEST_ASSERT(!JoltC_Vec3_IsNormalized(&zero, 0.0001f), "zero is not normalized");
    }
    TEST_END();

    TEST_BEGIN("Vec3 IsNaN");
    {
        JoltC_Vec3 nan_y = { 1.5f, (float)NAN, 3.75f };
        TEST_ASSERT(JoltC_Vec3_IsNaN(&nan_y), "NaN in y is detected");
        TEST_ASSERT(!JoltC_Vec3_IsNaN(&kVecA), "finite vector is not NaN");
    }
    TEST_END();

    /* ---------------------------------------------------------------- */
    /*  Quat — construction from Euler angles                           */
    /* ---------------------------------------------------------------- */

    /* A single-axis Euler rotation is convention-independent: rotating by
     * angle t about X must give (sin(t/2), 0, 0, cos(t/2)).
     *   X by  0.6: (sin 0.30, 0, 0, cos 0.30) = ( 0.29552021, 0, 0, 0.95533649)
     *   Y by -0.8: (0, sin -0.40, 0, cos -0.40) = (0, -0.38941834, 0, 0.92106099)
     *   Z by  1.1: (0, 0, sin 0.55, cos 0.55) = (0, 0, 0.52268723, 0.85252452)
     * Three separate axes, so a component swap in the angles going in or the
     * quaternion coming out cannot hide. */
    TEST_BEGIN("Quat FromEulerAngles single axis components");
    {
        JoltC_Vec3 ax = { 0.6f, 0.0f, 0.0f };
        JoltC_Vec3 ay = { 0.0f, -0.8f, 0.0f };
        JoltC_Vec3 az = { 0.0f, 0.0f, 1.1f };
        JoltC_Quat qx, qy, qz;
        JoltC_Quat_FromEulerAngles(&ax, &qx);
        JoltC_Quat_FromEulerAngles(&ay, &qy);
        JoltC_Quat_FromEulerAngles(&az, &qz);
        MR_ASSERT_QUAT(qx, 0.29552021f, 0.0f, 0.0f, 0.95533649f, 0.001f, "euler X");
        MR_ASSERT_QUAT(qy, 0.0f, -0.38941834f, 0.0f, 0.92106099f, 0.001f, "euler Y");
        MR_ASSERT_QUAT(qz, 0.0f, 0.0f, 0.52268723f, 0.85252452f, 0.001f, "euler Z");
    }
    TEST_END();

    /* Round trip: angles -> quaternion -> angles, one axis at a time so we stay
     * clear of the ordering and gimbal questions that multi-axis Euler raises. */
    TEST_BEGIN("Quat Euler angles round trip per axis");
    {
        JoltC_Vec3 in_x = { 0.6f, 0.0f, 0.0f };
        JoltC_Vec3 in_y = { 0.0f, -0.8f, 0.0f };
        JoltC_Vec3 in_z = { 0.0f, 0.0f, 1.1f };
        JoltC_Quat q;
        JoltC_Vec3 out;

        JoltC_Quat_FromEulerAngles(&in_x, &q);
        JoltC_Quat_GetEulerAngles(&q, &out);
        MR_ASSERT_VEC3(out, 0.6f, 0.0f, 0.0f, 0.001f, "round trip X");

        JoltC_Quat_FromEulerAngles(&in_y, &q);
        JoltC_Quat_GetEulerAngles(&q, &out);
        MR_ASSERT_VEC3(out, 0.0f, -0.8f, 0.0f, 0.001f, "round trip Y");

        JoltC_Quat_FromEulerAngles(&in_z, &q);
        JoltC_Quat_GetEulerAngles(&q, &out);
        MR_ASSERT_VEC3(out, 0.0f, 0.0f, 1.1f, 0.001f, "round trip Z");

        JoltC_Quat_GetEulerAngles(&kQuatId, &out);
        MR_ASSERT_VEC3(out, 0.0f, 0.0f, 0.0f, 0.001f, "identity has no rotation");
    }
    TEST_END();

    /* ---------------------------------------------------------------- */
    /*  Quat — axis/angle                                               */
    /* ---------------------------------------------------------------- */

    /* kQuatA's vector part has length exactly 0.6 and w = 0.8, so the axis is
     * exactly (1/3,-2/3,2/3) and the angle is 2*atan(0.6/0.8) = 1.2870022.
     * Second half: build a quaternion from a known axis and angle and read them
     * back out — a genuine compose/decompose round trip. */
    TEST_BEGIN("Quat GetAxisAngle known answer and round trip");
    {
        JoltC_Vec3 axis;
        float angle;
        JoltC_Quat_GetAxisAngle(&kQuatA, &axis, &angle);
        MR_ASSERT_VEC3(axis, AXIS_A_X, AXIS_A_Y, AXIS_A_Z, 0.001f, "kQuatA axis");
        TEST_ASSERT_FLOAT_EQ(angle, ANGLE_A, 0.001f, "kQuatA angle");
        TEST_ASSERT(JoltC_Vec3_IsNormalized(&axis, 0.001f), "returned axis is unit");

        JoltC_Quat q = mr_axis_angle(AXIS_A_X, AXIS_A_Y, AXIS_A_Z, 0.7f);
        JoltC_Vec3 axis2;
        float angle2;
        JoltC_Quat_GetAxisAngle(&q, &axis2, &angle2);
        MR_ASSERT_VEC3(axis2, AXIS_A_X, AXIS_A_Y, AXIS_A_Z, 0.001f, "round trip axis");
        TEST_ASSERT_FLOAT_EQ(angle2, 0.7f, 0.001f, "round trip angle");

        /* Identity has no meaningful axis; the angle must be zero. */
        JoltC_Vec3 axis3;
        float angle3;
        JoltC_Quat_GetAxisAngle(&kQuatId, &axis3, &angle3);
        TEST_ASSERT_FLOAT_EQ(angle3, 0.0f, 0.001f, "identity angle == 0");
    }
    TEST_END();

    /* For a rotation of t about unit axis a, the rotation angle measured about
     * a is t again: xyz.a / w = sin(t/2)/cos(t/2). */
    TEST_BEGIN("Quat GetRotationAngle about its own axis");
    {
        JoltC_Vec3 axis = { AXIS_A_X, AXIS_A_Y, AXIS_A_Z };
        float a1 = JoltC_Quat_GetRotationAngle(&kQuatA, &axis);
        TEST_ASSERT_FLOAT_EQ(a1, ANGLE_A, 0.001f, "kQuatA angle about its axis");

        JoltC_Quat q = mr_axis_angle(AXIS_A_X, AXIS_A_Y, AXIS_A_Z, -0.45f);
        float a2 = JoltC_Quat_GetRotationAngle(&q, &axis);
        TEST_ASSERT_FLOAT_EQ(a2, -0.45f, 0.001f, "signed angle is preserved");
    }
    TEST_END();

    /* ---------------------------------------------------------------- */
    /*  Quat — rotating vectors                                         */
    /* ---------------------------------------------------------------- */

    /* R (see kRotA) applied to a = (1.5,-2.25,3.75):
     *   x = 0.36*1.5 + -0.80*-2.25 + -0.48*3.75 = 0.54 + 1.80 - 1.80 =  0.54
     *   y = 0.48*1.5 +  0.60*-2.25 + -0.64*3.75 = 0.72 - 1.35 - 2.40 = -3.03
     *   z = 0.80*1.5 +  0.00*-2.25 +  0.60*3.75 = 1.20 + 0.00 + 2.25 =  3.45
     * Cross-check that the arithmetic is right: 0.54^2 + 3.03^2 + 3.45^2
     * = 0.2916 + 9.1809 + 11.9025 = 21.375 = |a|^2, as a rotation demands. */
    TEST_BEGIN("Quat Rotate asymmetric known answer");
    {
        JoltC_Vec3 r;
        JoltC_Quat_Rotate(&kQuatA, &kVecA, &r);
        MR_ASSERT_VEC3(r, 0.54f, -3.03f, 3.45f, 0.001f, "kQuatA * a");
        TEST_ASSERT_FLOAT_EQ(JoltC_Vec3_LengthSquared(&r), 21.375f, 0.002f, "length preserved");

        /* The identity quaternion must leave an asymmetric vector alone. */
        JoltC_Vec3 same;
        JoltC_Quat_Rotate(&kQuatId, &kVecA, &same);
        MR_ASSERT_VEC3(same, 1.5f, -2.25f, 3.75f, 0.001f, "identity rotation");
    }
    TEST_END();

    /* InverseRotate must undo Rotate, and must agree with rotating by the
     * inverse quaternion. Expected value is the hand-computed input, not output. */
    TEST_BEGIN("Quat InverseRotate undoes Rotate");
    {
        JoltC_Vec3 rotated = { 0.54f, -3.03f, 3.45f };  /* = kQuatA * kVecA */
        JoltC_Vec3 back, back2;
        JoltC_Quat inv;
        JoltC_Quat_InverseRotate(&kQuatA, &rotated, &back);
        MR_ASSERT_VEC3(back, 1.5f, -2.25f, 3.75f, 0.002f, "inverse rotate");

        JoltC_Quat_Inversed(&kQuatA, &inv);
        JoltC_Quat_Rotate(&inv, &rotated, &back2);
        MR_ASSERT_VEC3(back2, 1.5f, -2.25f, 3.75f, 0.002f, "rotate by inverse");
    }
    TEST_END();

    /* Rotating the basis vectors by kQuatA must reproduce the columns of R —
     * the same numbers JoltC_Mat44_Rotation is asserted against below. If the
     * quaternion and matrix paths ever disagree, one of these two tests moves. */
    TEST_BEGIN("Quat RotateAxisX/Y/Z are the columns of R");
    {
        JoltC_Vec3 rx, ry, rz;
        JoltC_Quat_RotateAxisX(&kQuatA, &rx);
        JoltC_Quat_RotateAxisY(&kQuatA, &ry);
        JoltC_Quat_RotateAxisZ(&kQuatA, &rz);
        MR_ASSERT_VEC3(rx,  0.36f,  0.48f, 0.80f, 0.001f, "RotateAxisX");
        MR_ASSERT_VEC3(ry, -0.80f,  0.60f, 0.00f, 0.001f, "RotateAxisY");
        MR_ASSERT_VEC3(rz, -0.48f, -0.64f, 0.60f, 0.001f, "RotateAxisZ");
    }
    TEST_END();

    /* ---------------------------------------------------------------- */
    /*  Quat — algebra                                                  */
    /* ---------------------------------------------------------------- */

    /* Conjugate negates the vector part only. For a unit quaternion the
     * conjugate is the inverse, so both must give (-0.2, 0.4, -0.4, 0.8). */
    TEST_BEGIN("Quat Conjugated and Inversed of unit quat");
    {
        JoltC_Quat conj, inv;
        JoltC_Quat_Conjugated(&kQuatA, &conj);
        JoltC_Quat_Inversed(&kQuatA, &inv);
        MR_ASSERT_QUAT(conj, -0.2f, 0.4f, -0.4f, 0.8f, 0.001f, "conjugate");
        MR_ASSERT_QUAT(inv,  -0.2f, 0.4f, -0.4f, 0.8f, 0.001f, "inverse");

        JoltC_Quat back;
        JoltC_Quat_Conjugated(&conj, &back);
        MR_ASSERT_QUAT(back, 0.2f, -0.4f, 0.4f, 0.8f, 0.001f, "double conjugate");
    }
    TEST_END();

    /* Component-wise, with kQuatA = (0.2,-0.4,0.4,0.8), kQuatB = (0.5,0.5,-0.5,0.5):
     *   A+B      = (0.7, 0.1, -0.1, 1.3)
     *   A-B      = (-0.3, -0.9, 0.9, 0.3)
     *   A*-2.5   = (-0.5, 1.0, -1.0, -2.0)
     *   A/0.4    = (0.5, -1.0, 1.0, 2.0)
     *   A.B      = 0.1 - 0.2 - 0.2 + 0.4 = 0.1
     *   A.A      = 1 (A is unit) */
    TEST_BEGIN("Quat Add / Subtract / scale / Dot");
    {
        JoltC_Quat sum, diff, scaled, divided;
        float dot_ab, dot_aa;
        JoltC_Quat_Add(&kQuatA, &kQuatB, &sum);
        JoltC_Quat_Subtract(&kQuatA, &kQuatB, &diff);
        JoltC_Quat_MultiplyScalar(&kQuatA, -2.5f, &scaled);
        JoltC_Quat_DivideScalar(&kQuatA, 0.4f, &divided);
        JoltC_Quat_Dot(&kQuatA, &kQuatB, &dot_ab);
        JoltC_Quat_Dot(&kQuatA, &kQuatA, &dot_aa);
        MR_ASSERT_QUAT(sum,      0.7f,  0.1f, -0.1f,  1.3f, 0.001f, "A+B");
        MR_ASSERT_QUAT(diff,    -0.3f, -0.9f,  0.9f,  0.3f, 0.001f, "A-B");
        MR_ASSERT_QUAT(scaled,  -0.5f,  1.0f, -1.0f, -2.0f, 0.001f, "A*-2.5");
        MR_ASSERT_QUAT(divided,  0.5f, -1.0f,  1.0f,  2.0f, 0.001f, "A/0.4");
        TEST_ASSERT_FLOAT_EQ(dot_ab, 0.1f, 0.001f, "A.B");
        TEST_ASSERT_FLOAT_EQ(dot_aa, 1.0f, 0.001f, "A.A == 1");
    }
    TEST_END();

    /* Standard Hamilton product, q1*q2 with q1 = A, q2 = B:
     *   x = w1x2 + x1w2 + y1z2 - z1y2 = 0.40 + 0.10 + 0.20 - 0.20 = 0.5
     *   y = w1y2 - x1z2 + y1w2 + z1x2 = 0.40 + 0.10 - 0.20 + 0.20 = 0.5
     *   z = w1z2 + x1y2 - y1x2 + z1w2 = -0.40 + 0.10 + 0.20 + 0.20 = 0.1
     *   w = w1w2 - x1x2 - y1y2 - z1z2 = 0.40 - 0.10 + 0.20 + 0.20 = 0.7
     * The result is unit (0.25+0.25+0.01+0.49 = 1), which is the check that the
     * hand arithmetic above is right — the product of two unit quaternions has
     * to be unit. */
    TEST_BEGIN("Quat Multiply known Hamilton product");
    {
        JoltC_Quat p;
        float len_sq;
        JoltC_Quat_Multiply(&kQuatA, &kQuatB, &p);
        MR_ASSERT_QUAT(p, 0.5f, 0.5f, 0.1f, 0.7f, 0.002f, "A*B");
        JoltC_Quat_Dot(&p, &p, &len_sq);
        TEST_ASSERT_FLOAT_EQ(len_sq, 1.0f, 0.002f, "A*B is unit");

        /* Identity on either side is a no-op. */
        JoltC_Quat li, ri;
        JoltC_Quat_Multiply(&kQuatId, &kQuatA, &li);
        JoltC_Quat_Multiply(&kQuatA, &kQuatId, &ri);
        MR_ASSERT_QUAT(li, 0.2f, -0.4f, 0.4f, 0.8f, 0.001f, "I*A");
        MR_ASSERT_QUAT(ri, 0.2f, -0.4f, 0.4f, 0.8f, 0.001f, "A*I");
    }
    TEST_END();

    /* Ordering check: (A*B) applied to a vector must equal A applied to
     * (B applied to the vector). This is what breaks if the operands of the
     * product are ever swapped during a repair — the component values above
     * would still look plausible.
     *
     * The expected vector is hand-computed rather than merely compared between
     * the two paths. A*B = (0.5,0.5,0.1,0.7); its rotation matrix is
     *   [ 0.48  0.36  0.80 ]
     *   [ 0.64  0.48 -0.60 ]
     *   [-0.60  0.80  0.00 ]
     * (columns are unit and col0 x col1 = col2), so applied to (1.5,-2.25,3.75):
     *   x = 0.72 - 0.81 + 3.00 =  2.91
     *   y = 0.96 - 1.08 - 2.25 = -2.37
     *   z = -0.90 - 1.80 + 0   = -2.70
     * and 2.91^2 + 2.37^2 + 2.70^2 = 21.375 = |a|^2, as required.
     * Reversing the operands would give B*A = (0.5,-0.1,-0.5,0.7) instead,
     * which sends a somewhere else entirely. */
    TEST_BEGIN("Quat Multiply composes in the right order");
    {
        JoltC_Quat ab;
        JoltC_Vec3 direct, stepwise, tmp;
        JoltC_Quat_Multiply(&kQuatA, &kQuatB, &ab);
        JoltC_Quat_Rotate(&ab, &kVecA, &direct);
        MR_ASSERT_VEC3(direct, 2.91f, -2.37f, -2.70f, 0.002f, "(A*B) a");
        JoltC_Quat_Rotate(&kQuatB, &kVecA, &tmp);
        JoltC_Quat_Rotate(&kQuatA, &tmp, &stepwise);
        MR_ASSERT_VEC3(stepwise, 2.91f, -2.37f, -2.70f, 0.003f, "A(B a)");
        /* And the result is still a rotation of the original. */
        TEST_ASSERT_FLOAT_EQ(JoltC_Vec3_LengthSquared(&direct), 21.375f, 0.003f,
                             "composed rotation preserves length");
    }
    TEST_END();

    /* Rotations about a common axis add their angles and commute — true under
     * any multiplication-order convention, so this pins down the product
     * formula itself rather than the operand order. */
    TEST_BEGIN("Quat Multiply about one axis adds angles");
    {
        JoltC_Quat q1 = mr_axis_angle(AXIS_A_X, AXIS_A_Y, AXIS_A_Z, 0.4f);
        JoltC_Quat q2 = mr_axis_angle(AXIS_A_X, AXIS_A_Y, AXIS_A_Z, 0.9f);
        JoltC_Quat p12, p21;
        JoltC_Vec3 axis;
        float angle;
        JoltC_Quat_Multiply(&q1, &q2, &p12);
        JoltC_Quat_Multiply(&q2, &q1, &p21);
        JoltC_Quat_GetAxisAngle(&p12, &axis, &angle);
        MR_ASSERT_VEC3(axis, AXIS_A_X, AXIS_A_Y, AXIS_A_Z, 0.002f, "combined axis");
        TEST_ASSERT_FLOAT_EQ(angle, 1.3f, 0.002f, "0.4 + 0.9 == 1.3");
        MR_ASSERT_QUAT(p21, p12.x, p12.y, p12.z, p12.w, 0.002f, "same axis commutes");
    }
    TEST_END();

    /* Lerp is a plain component-wise blend (Slerp is the normalising one).
     * At 0.25 between identity and A:
     *   x = 0.75*0 + 0.25*0.2  =  0.05
     *   y = 0.75*0 + 0.25*-0.4 = -0.10
     *   z = 0.75*0 + 0.25*0.4  =  0.10
     *   w = 0.75*1 + 0.25*0.8  =  0.95 */
    TEST_BEGIN("Quat Lerp endpoints and quarter point");
    {
        JoltC_Quat at0, at1, at25;
        JoltC_Quat_Lerp(&kQuatId, &kQuatA, 0.0f,  &at0);
        JoltC_Quat_Lerp(&kQuatId, &kQuatA, 1.0f,  &at1);
        JoltC_Quat_Lerp(&kQuatId, &kQuatA, 0.25f, &at25);
        MR_ASSERT_QUAT(at0,  0.0f,  0.0f, 0.0f, 1.0f,  0.001f, "lerp(0) == identity");
        MR_ASSERT_QUAT(at1,  0.2f, -0.4f, 0.4f, 0.8f,  0.001f, "lerp(1) == A");
        MR_ASSERT_QUAT(at25, 0.05f, -0.1f, 0.1f, 0.95f, 0.001f, "lerp(0.25)");
    }
    TEST_END();

    /* ---------------------------------------------------------------- */
    /*  Quat — swing / twist / perpendicular                            */
    /* ---------------------------------------------------------------- */

    /* A rotation entirely about some axis is its own twist about that axis. */
    TEST_BEGIN("Quat GetTwist about its own axis");
    {
        JoltC_Vec3 axis = { AXIS_A_X, AXIS_A_Y, AXIS_A_Z };
        JoltC_Quat twist;
        JoltC_Quat_GetTwist(&kQuatA, &axis, &twist);
        MR_ASSERT_QUAT(twist, 0.2f, -0.4f, 0.4f, 0.8f, 0.002f, "twist about own axis");

        /* A rotation about X has no twist about Y: the vector part is
         * perpendicular to Y, so only w survives and normalises to identity. */
        JoltC_Vec3 euler_x = { 0.6f, 0.0f, 0.0f };
        JoltC_Vec3 axis_y = { 0.0f, 1.0f, 0.0f };
        JoltC_Quat qx, twist_y;
        JoltC_Quat_FromEulerAngles(&euler_x, &qx);
        JoltC_Quat_GetTwist(&qx, &axis_y, &twist_y);
        MR_ASSERT_QUAT(twist_y, 0.0f, 0.0f, 0.0f, 1.0f, 0.002f, "no twist about Y");
    }
    TEST_END();

    /* GetSwingTwist splits around X: the twist rotates only about X and the
     * swing has no X component. For a pure X rotation the split is degenerate —
     * all twist, no swing — which is the case with a hand-known answer. */
    TEST_BEGIN("Quat GetSwingTwist structure");
    {
        JoltC_Vec3 euler_x = { 0.6f, 0.0f, 0.0f };
        JoltC_Quat qx, swing, twist;
        float len_sq;
        JoltC_Quat_FromEulerAngles(&euler_x, &qx);
        JoltC_Quat_GetSwingTwist(&qx, &swing, &twist);
        MR_ASSERT_QUAT(twist, 0.29552021f, 0.0f, 0.0f, 0.95533649f, 0.002f,
                       "pure X rotation is all twist");
        MR_ASSERT_QUAT(swing, 0.0f, 0.0f, 0.0f, 1.0f, 0.002f, "no swing");

        /* General case: only the structural guarantees, which are the ones a
         * component mix-up in the wrapper would violate. */
        JoltC_Quat swing2, twist2;
        JoltC_Quat_GetSwingTwist(&kQuatA, &swing2, &twist2);
        TEST_ASSERT_FLOAT_EQ(twist2.y, 0.0f, 0.001f, "twist has no y");
        TEST_ASSERT_FLOAT_EQ(twist2.z, 0.0f, 0.001f, "twist has no z");
        TEST_ASSERT_FLOAT_EQ(swing2.x, 0.0f, 0.001f, "swing has no x");
        JoltC_Quat_Dot(&twist2, &twist2, &len_sq);
        TEST_ASSERT_FLOAT_EQ(len_sq, 1.0f, 0.002f, "twist is unit");
        JoltC_Quat_Dot(&swing2, &swing2, &len_sq);
        TEST_ASSERT_FLOAT_EQ(len_sq, 1.0f, 0.002f, "swing is unit");
    }
    TEST_END();

    /* GetPerpendicular returns a quaternion orthogonal to the original in 4D,
     * of the same magnitude. Both facts hold whatever component shuffle is used
     * to produce it. */
    TEST_BEGIN("Quat GetPerpendicular is orthogonal, unit");
    {
        JoltC_Quat perp;
        float dot, len_sq;
        JoltC_Quat_GetPerpendicular(&kQuatA, &perp);
        JoltC_Quat_Dot(&kQuatA, &perp, &dot);
        JoltC_Quat_Dot(&perp, &perp, &len_sq);
        TEST_ASSERT_FLOAT_EQ(dot, 0.0f, 0.001f, "A . perp(A) == 0");
        TEST_ASSERT_FLOAT_EQ(len_sq, 1.0f, 0.002f, "perp of a unit quat is unit");
    }
    TEST_END();

    /* ---------------------------------------------------------------- */
    /*  Quat — FromTo                                                   */
    /* ---------------------------------------------------------------- */

    /* sFromTo(a,b) must carry a onto the direction of b. Rotation preserves
     * length, so the result is |a| * normalize(b); we compare directions using
     * normalize(b)'s hand-computed value: b / 2.4109127
     * = (-0.2073896, 0.8295585, 0.5184741). */
    TEST_BEGIN("Quat FromTo maps a onto the direction of b");
    {
        JoltC_Quat q;
        JoltC_Vec3 rotated, dir;
        JoltC_Quat_FromTo(&kVecA, &kVecB, &q);
        JoltC_Quat_Rotate(&q, &kVecA, &rotated);
        TEST_ASSERT_FLOAT_EQ(JoltC_Vec3_Length(&rotated), 4.6233117f, 0.002f,
                             "length of a is preserved");
        JoltC_Vec3_Normalized(&rotated, &dir);
        MR_ASSERT_VEC3(dir, -0.2073896f, 0.8295585f, 0.5184741f, 0.002f,
                       "direction of b");
    }
    TEST_END();

    /* The two degenerate inputs. Parallel: the rotation is the identity.
     * Anti-parallel: any 180-degree rotation about an axis perpendicular to a
     * will do, and all of them send a to -a, so that is what we assert rather
     * than a particular quaternion. */
    TEST_BEGIN("Quat FromTo parallel and anti-parallel");
    {
        JoltC_Quat q_same, q_opp;
        JoltC_Vec3 neg_a, r_same, r_opp;
        JoltC_Vec3_Negate(&kVecA, &neg_a);

        JoltC_Quat_FromTo(&kVecA, &kVecA, &q_same);
        JoltC_Quat_Rotate(&q_same, &kVecA, &r_same);
        MR_ASSERT_VEC3(r_same, 1.5f, -2.25f, 3.75f, 0.002f, "a -> a");

        JoltC_Quat_FromTo(&kVecA, &neg_a, &q_opp);
        JoltC_Quat_Rotate(&q_opp, &kVecA, &r_opp);
        MR_ASSERT_VEC3(r_opp, -1.5f, 2.25f, -3.75f, 0.01f, "a -> -a");
    }
    TEST_END();

    /* ---------------------------------------------------------------- */
    /*  Mat44 — construction                                            */
    /* ---------------------------------------------------------------- */

    TEST_BEGIN("Mat44 Zero is all sixteen zeros");
    {
        JoltC_Mat44 z;
        int i;
        JoltC_Mat44_Zero(&z);
        for (i = 0; i < 16; ++i) {
            TEST_ASSERT(z.m[i] == 0.0f, "every element is zero");
        }
    }
    TEST_END();

    /* Full element-wise check against the hand-derived R (see kRotA). This is
     * the single most swap-sensitive assertion in the file: R has nine distinct
     * entries, so any transposition, column/row confusion or component swap in
     * the quaternion-to-matrix path changes it. */
    TEST_BEGIN("Mat44 Rotation known entries");
    {
        JoltC_Mat44 m;
        JoltC_Mat44_Rotation(&m, &kQuatA);
        mr_assert_mat44(&m, &kRotA, 0.001f, "Mat44_Rotation(kQuatA)");

        JoltC_Mat44 ident_from_quat, ident;
        JoltC_Mat44_Rotation(&ident_from_quat, &kQuatId);
        JoltC_Mat44_Identity(&ident);
        mr_assert_mat44(&ident_from_quat, &ident, 0.001f, "rotation of identity quat");
    }
    TEST_END();

    /* The axis/angle constructor must land on the same matrix as the quaternion
     * constructor: kQuatA is exactly (axis (1/3,-2/3,2/3), angle 1.2870022). */
    TEST_BEGIN("Mat44 Rotation2 axis-angle == quat rotation");
    {
        JoltC_Vec3 unnormalized = { 0.2f, -0.4f, 0.4f };
        JoltC_Vec3 axis;
        JoltC_Mat44 m;
        JoltC_Vec3_Normalized(&unnormalized, &axis);
        JoltC_Mat44_Rotation2(&m, &axis, ANGLE_A);
        mr_assert_mat44(&m, &kRotA, 0.002f, "Mat44_Rotation2(axis, angle)");
    }
    TEST_END();

    /* Scale matrix: the scale factors sit on the diagonal, nothing else moves. */
    TEST_BEGIN("Mat44 Scale entries and vector transform");
    {
        const JoltC_Mat44 expected = { {
             1.5f,  0.0f,   0.0f,  0.0f,
             0.0f, -2.25f,  0.0f,  0.0f,
             0.0f,  0.0f,   3.75f, 0.0f,
             0.0f,  0.0f,   0.0f,  1.0f
        } };
        JoltC_Mat44 m;
        JoltC_Vec3 scaled;
        JoltC_Mat44_Scale(&m, &kScale);
        mr_assert_mat44(&m, &expected, 0.001f, "Mat44_Scale");
        /* diag(1.5,-2.25,3.75) * (1.5,-2.25,3.75) = (2.25, 5.0625, 14.0625) */
        JoltC_Vec3_MultiplyMatrix(&kVecA, &m, &scaled);
        MR_ASSERT_VEC3(scaled, 2.25f, 5.0625f, 14.0625f, 0.001f, "scaled vector");
    }
    TEST_END();

    /* Translation lives in column 3, and multiplying a vector by the matrix
     * treats it as a point: v + t. */
    TEST_BEGIN("Mat44 Translation transforms a point");
    {
        const JoltC_Mat44 expected = { {
             1.0f,  0.0f, 0.0f, 0.0f,
             0.0f,  1.0f, 0.0f, 0.0f,
             0.0f,  0.0f, 1.0f, 0.0f,
            -1.25f, 0.5f, 2.0f, 1.0f
        } };
        JoltC_Mat44 m;
        JoltC_Vec3 moved, t;
        JoltC_Mat44_Translation(&m, &kTrans);
        mr_assert_mat44(&m, &expected, 0.001f, "Mat44_Translation");
        /* (1.5,-2.25,3.75) + (-1.25,0.5,2) = (0.25,-1.75,5.75) */
        JoltC_Vec3_MultiplyMatrix(&kVecA, &m, &moved);
        MR_ASSERT_VEC3(moved, 0.25f, -1.75f, 5.75f, 0.001f, "translated point");
        JoltC_Mat44_GetTranslation(&m, &t);
        MR_ASSERT_VEC3(t, -1.25f, 0.5f, 2.0f, 0.001f, "GetTranslation");
    }
    TEST_END();

    /* Rotation and translation together: R in the 3x3 block, t in column 3,
     * and a point maps to R*v + t = (0.54,-3.03,3.45) + (-1.25,0.5,2)
     * = (-0.71,-2.53,5.45). The translation must survive extraction unchanged
     * even though the rotation part is non-trivial. */
    TEST_BEGIN("Mat44 RotationTranslation transform and extract");
    {
        JoltC_Mat44 m;
        JoltC_Vec3 p, t;
        JoltC_Mat44_RotationTranslation(&m, &kQuatA, &kTrans);
        mr_assert_mat44(&m, &kRotTransA, 0.001f, "Mat44_RotationTranslation");
        JoltC_Vec3_MultiplyMatrix(&kVecA, &m, &p);
        MR_ASSERT_VEC3(p, -0.71f, -2.53f, 5.45f, 0.002f, "transformed point");
        JoltC_Mat44_GetTranslation(&m, &t);
        MR_ASSERT_VEC3(t, -1.25f, 0.5f, 2.0f, 0.001f, "translation extracted");
    }
    TEST_END();

    /* Compose then decompose: the axes are the columns of R and the recovered
     * quaternion rotates a vector the same way kQuatA does (compared up to
     * sign, since q and -q are the same rotation). GetQuaternion is called on a
     * pure rotation matrix only. */
    TEST_BEGIN("Mat44 GetAxisX/Y/Z and GetQuaternion");
    {
        JoltC_Mat44 m;
        JoltC_Vec3 ax, ay, az;
        JoltC_Quat q;
        float dot;
        JoltC_Mat44_Rotation(&m, &kQuatA);
        JoltC_Mat44_GetAxisX(&m, &ax);
        JoltC_Mat44_GetAxisY(&m, &ay);
        JoltC_Mat44_GetAxisZ(&m, &az);
        MR_ASSERT_VEC3(ax,  0.36f,  0.48f, 0.80f, 0.001f, "GetAxisX");
        MR_ASSERT_VEC3(ay, -0.80f,  0.60f, 0.00f, 0.001f, "GetAxisY");
        MR_ASSERT_VEC3(az, -0.48f, -0.64f, 0.60f, 0.001f, "GetAxisZ");

        JoltC_Mat44_GetQuaternion(&m, &q);
        JoltC_Quat_Dot(&q, &kQuatA, &dot);
        TEST_ASSERT(fabsf(dot) > 0.999f, "recovered quaternion matches up to sign");
        JoltC_Vec3 r;
        JoltC_Quat_Rotate(&q, &kVecA, &r);
        MR_ASSERT_VEC3(r, 0.54f, -3.03f, 3.45f, 0.002f, "recovered quat rotates alike");
    }
    TEST_END();

    /* ---------------------------------------------------------------- */
    /*  Mat44 — multiplication                                          */
    /* ---------------------------------------------------------------- */

    /* Ordering matters and the two orders differ in the translation column:
     *   T*S: 3x3 = diag(s), column 3 = t                = (-1.25, 0.5, 2)
     *   S*T: 3x3 = diag(s), column 3 = s componentwise* t
     *        = (1.5*-1.25, -2.25*0.5, 3.75*2)           = (-1.875, -1.125, 7.5)
     * A test that only multiplied by the identity could not tell these apart. */
    TEST_BEGIN("Mat44 Multiply ordering (T*S vs S*T)");
    {
        const JoltC_Mat44 expect_ts = { {
             1.5f,  0.0f,   0.0f,  0.0f,
             0.0f, -2.25f,  0.0f,  0.0f,
             0.0f,  0.0f,   3.75f, 0.0f,
            -1.25f, 0.5f,   2.0f,  1.0f
        } };
        const JoltC_Mat44 expect_st = { {
             1.5f,   0.0f,   0.0f,  0.0f,
             0.0f,  -2.25f,  0.0f,  0.0f,
             0.0f,   0.0f,   3.75f, 0.0f,
            -1.875f,-1.125f, 7.5f,  1.0f
        } };
        JoltC_Mat44 t, s, ts, st;
        JoltC_Mat44_Translation(&t, &kTrans);
        JoltC_Mat44_Scale(&s, &kScale);
        JoltC_Mat44_Multiply(&t, &s, &ts);
        JoltC_Mat44_Multiply(&s, &t, &st);
        mr_assert_mat44(&ts, &expect_ts, 0.001f, "T*S");
        mr_assert_mat44(&st, &expect_st, 0.001f, "S*T");
    }
    TEST_END();

    /* T*R has to be exactly what RotationTranslation builds: the rotation
     * untouched in the 3x3 block, t in column 3. */
    TEST_BEGIN("Mat44 T*R == RotationTranslation");
    {
        JoltC_Mat44 t, r, tr;
        JoltC_Mat44_Translation(&t, &kTrans);
        JoltC_Mat44_Rotation(&r, &kQuatA);
        JoltC_Mat44_Multiply(&t, &r, &tr);
        mr_assert_mat44(&tr, &kRotTransA, 0.002f, "T*R");
    }
    TEST_END();

    /* The other order rotates the translation:
     *   R*t = (0.36*-1.25 + -0.80*0.5 + -0.48*2,
     *          0.48*-1.25 +  0.60*0.5 + -0.64*2,
     *          0.80*-1.25 +  0.00*0.5 +  0.60*2)
     *       = (-0.45-0.40-0.96, -0.60+0.30-1.28, -1.00+0.00+1.20)
     *       = (-1.81, -1.58, 0.20) */
    TEST_BEGIN("Mat44 R*T known translation column");
    {
        const JoltC_Mat44 expected = { {
             0.36f,  0.48f,  0.80f, 0.0f,
            -0.80f,  0.60f,  0.00f, 0.0f,
            -0.48f, -0.64f,  0.60f, 0.0f,
            -1.81f, -1.58f,  0.20f, 1.0f
        } };
        JoltC_Mat44 t, r, rt;
        JoltC_Mat44_Translation(&t, &kTrans);
        JoltC_Mat44_Rotation(&r, &kQuatA);
        JoltC_Mat44_Multiply(&r, &t, &rt);
        mr_assert_mat44(&rt, &expected, 0.002f, "R*T");
    }
    TEST_END();

    /* The quaternion and matrix layers have to compose the same way round:
     * Rotation(A*B) == Rotation(A) * Rotation(B). The expected matrix is the
     * hand-derived rotation of A*B = (0.5,0.5,0.1,0.7) written out in the
     * Quat-Multiply-ordering test above. If either layer's operand order flips,
     * this test and that one move together and the culprit is unambiguous. */
    TEST_BEGIN("Mat44 rotation composition matches Quat");
    {
        const JoltC_Mat44 expected = { {
             0.48f,  0.64f, -0.60f, 0.0f,
             0.36f,  0.48f,  0.80f, 0.0f,
             0.80f, -0.60f,  0.00f, 0.0f,
             0.0f,   0.0f,   0.0f,  1.0f
        } };
        JoltC_Quat ab;
        JoltC_Mat44 m_ab, m_a, m_b, product;
        JoltC_Quat_Multiply(&kQuatA, &kQuatB, &ab);
        JoltC_Mat44_Rotation(&m_ab, &ab);
        mr_assert_mat44(&m_ab, &expected, 0.002f, "Rotation(A*B)");
        JoltC_Mat44_Rotation(&m_a, &kQuatA);
        JoltC_Mat44_Rotation(&m_b, &kQuatB);
        JoltC_Mat44_Multiply(&m_a, &m_b, &product);
        mr_assert_mat44(&product, &expected, 0.002f, "Rotation(A) * Rotation(B)");
    }
    TEST_END();

    /* I + T doubles the diagonal and keeps t; T - I zeroes the diagonal and
     * keeps t; T * -2.5 scales everything including the translation. */
    TEST_BEGIN("Mat44 Add / Subtract / MultiplyScalar");
    {
        const JoltC_Mat44 expect_add = { {
             2.0f,  0.0f, 0.0f, 0.0f,
             0.0f,  2.0f, 0.0f, 0.0f,
             0.0f,  0.0f, 2.0f, 0.0f,
            -1.25f, 0.5f, 2.0f, 2.0f
        } };
        const JoltC_Mat44 expect_sub = { {
             0.0f,  0.0f, 0.0f, 0.0f,
             0.0f,  0.0f, 0.0f, 0.0f,
             0.0f,  0.0f, 0.0f, 0.0f,
            -1.25f, 0.5f, 2.0f, 0.0f
        } };
        const JoltC_Mat44 expect_mul = { {
            -2.5f,   0.0f,  0.0f,  0.0f,
             0.0f,  -2.5f,  0.0f,  0.0f,
             0.0f,   0.0f, -2.5f,  0.0f,
             3.125f,-1.25f,-5.0f, -2.5f
        } };
        JoltC_Mat44 i, t, sum, diff, scaled;
        JoltC_Mat44_Identity(&i);
        JoltC_Mat44_Translation(&t, &kTrans);
        JoltC_Mat44_Add(&i, &t, &sum);
        JoltC_Mat44_Subtract(&t, &i, &diff);
        JoltC_Mat44_MultiplyScalar(&t, -2.5f, &scaled);
        mr_assert_mat44(&sum,    &expect_add, 0.001f, "I+T");
        mr_assert_mat44(&diff,   &expect_sub, 0.001f, "T-I");
        mr_assert_mat44(&scaled, &expect_mul, 0.001f, "T*-2.5");
    }
    TEST_END();

    /* ---------------------------------------------------------------- */
    /*  Mat44 — transpose and inverse                                   */
    /* ---------------------------------------------------------------- */

    /* Transposing R turns its columns into rows; for a rotation that is also
     * its inverse, so applying it to (0.54,-3.03,3.45) recovers a:
     *   row0 . p = 0.36*0.54 + 0.48*-3.03 + 0.80*3.45 = 1.5
     *   row1 . p = -0.80*0.54 + 0.60*-3.03 + 0.00*3.45 = -2.25
     *   row2 . p = -0.48*0.54 + -0.64*-3.03 + 0.60*3.45 = 3.75 */
    TEST_BEGIN("Mat44 Transposed rotation == inverse rotation");
    {
        JoltC_Mat44 r, rt, inv;
        JoltC_Vec3 rotated = { 0.54f, -3.03f, 3.45f };
        JoltC_Vec3 back;
        JoltC_Mat44_Rotation(&r, &kQuatA);
        JoltC_Mat44_Transposed(&r, &rt);
        /* Transpose of kRotA is the rotation part of kInvRotTransA. */
        TEST_ASSERT_FLOAT_EQ(rt.m[0],  0.36f, 0.001f, "rt m[0]");
        TEST_ASSERT_FLOAT_EQ(rt.m[1], -0.80f, 0.001f, "rt m[1] == r m[4]");
        TEST_ASSERT_FLOAT_EQ(rt.m[2], -0.48f, 0.001f, "rt m[2] == r m[8]");
        TEST_ASSERT_FLOAT_EQ(rt.m[4],  0.48f, 0.001f, "rt m[4] == r m[1]");
        TEST_ASSERT_FLOAT_EQ(rt.m[6], -0.64f, 0.001f, "rt m[6] == r m[9]");
        TEST_ASSERT_FLOAT_EQ(rt.m[9],  0.00f, 0.001f, "rt m[9] == r m[6]");
        JoltC_Vec3_MultiplyMatrix(&rotated, &rt, &back);
        MR_ASSERT_VEC3(back, 1.5f, -2.25f, 3.75f, 0.002f, "R^T undoes R");
        /* And it agrees with the general inverse. */
        JoltC_Mat44_Inversed(&r, &inv);
        mr_assert_mat44(&rt, &inv, 0.002f, "R^T == R^-1");
    }
    TEST_END();

    /* The transpose is of the whole 4x4, so a translation column becomes the
     * bottom row: m[12..14] move to m[3], m[7], m[11]. */
    TEST_BEGIN("Mat44 Transposed moves translation to row 3");
    {
        JoltC_Mat44 t, tt, back;
        JoltC_Mat44_Translation(&t, &kTrans);
        JoltC_Mat44_Transposed(&t, &tt);
        TEST_ASSERT_FLOAT_EQ(tt.m[3],  -1.25f, 0.001f, "t.x at m[3]");
        TEST_ASSERT_FLOAT_EQ(tt.m[7],   0.5f,  0.001f, "t.y at m[7]");
        TEST_ASSERT_FLOAT_EQ(tt.m[11],  2.0f,  0.001f, "t.z at m[11]");
        TEST_ASSERT_FLOAT_EQ(tt.m[12],  0.0f,  0.001f, "column 3 cleared");
        TEST_ASSERT_FLOAT_EQ(tt.m[13],  0.0f,  0.001f, "column 3 cleared");
        TEST_ASSERT_FLOAT_EQ(tt.m[14],  0.0f,  0.001f, "column 3 cleared");
        /* Transposing twice is the identity. */
        JoltC_Mat44_Transposed(&tt, &back);
        mr_assert_mat44(&back, &t, 0.001f, "transpose twice");
    }
    TEST_END();

    /* Hand-computed inverse of the rotation+translation matrix (see
     * kInvRotTransA) plus the defining property M * M^-1 == I. */
    TEST_BEGIN("Mat44 Inversed known entries and identity");
    {
        JoltC_Mat44 m, inv, product, ident;
        JoltC_Mat44_RotationTranslation(&m, &kQuatA, &kTrans);
        JoltC_Mat44_Inversed(&m, &inv);
        mr_assert_mat44(&inv, &kInvRotTransA, 0.002f, "Inversed");
        JoltC_Mat44_Multiply(&m, &inv, &product);
        JoltC_Mat44_Identity(&ident);
        mr_assert_mat44(&product, &ident, 0.002f, "M * M^-1 == I");

        /* Round trip through the inverse recovers the original point. */
        JoltC_Vec3 transformed = { -0.71f, -2.53f, 5.45f };  /* = M * kVecA */
        JoltC_Vec3 back;
        JoltC_Vec3_MultiplyMatrix(&transformed, &inv, &back);
        MR_ASSERT_VEC3(back, 1.5f, -2.25f, 3.75f, 0.005f, "M^-1 * (M * a)");
    }
    TEST_END();

    /* Three routes to the same inverse must agree:
     *   Mat44_InverseRotationTranslation(q, t)   — builds it from q and t
     *   Mat44_Inversed(RotationTranslation(q,t)) — general inverse
     *   RMat44_InversedRotationTranslation(M)    — the R-flavoured shortcut
     * Note the two similarly named entry points do different things:
     * Mat44_InverseRotationTranslation takes a quaternion and a translation,
     * while Mat4_InverseRotationTranslation (below) inverts a whole matrix. */
    TEST_BEGIN("Mat44 InverseRotationTranslation agreement");
    {
        JoltC_Mat44 built, m, inversed, shortcut;
        JoltC_Mat44_InverseRotationTranslation(&built, &kQuatA, &kTrans);
        mr_assert_mat44(&built, &kInvRotTransA, 0.002f, "built from q and t");
        JoltC_Mat44_RotationTranslation(&m, &kQuatA, &kTrans);
        JoltC_Mat44_Inversed(&m, &inversed);
        mr_assert_mat44(&built, &inversed, 0.002f, "== general inverse");
        JoltC_RMat44_InversedRotationTranslation(&m, &shortcut);
        mr_assert_mat44(&shortcut, &kInvRotTransA, 0.002f, "== RMat44 shortcut");
    }
    TEST_END();

    /* ---------------------------------------------------------------- */
    /*  RayCast helpers — Vec3 and RVec3 paths                          */
    /* ---------------------------------------------------------------- */

    /* origin + 0.4 * direction
     *   = (1.5,-2.25,3.75) + 0.4*(-0.5,2,1.25)
     *   = (1.5-0.2, -2.25+0.8, 3.75+0.5) = (1.3,-1.45,4.25)
     * A fraction of 0 must give the origin back untouched. */
    TEST_BEGIN("RayCast GetPointOnRay known answer");
    {
        JoltC_Vec3 p, at_origin;
        JoltC_RayCast_GetPointOnRay(&kVecA, &kVecB, 0.4f, &p);
        MR_ASSERT_VEC3(p, 1.3f, -1.45f, 4.25f, 0.001f, "point at 0.4");
        JoltC_RayCast_GetPointOnRay(&kVecA, &kVecB, 0.0f, &at_origin);
        MR_ASSERT_VEC3(at_origin, 1.5f, -2.25f, 3.75f, 0.001f, "point at 0");
    }
    TEST_END();

    /* The RVec3 origin path has its own converter (toJphRVec3/fromJphRVec3,
     * deliberately not overloaded with the Vec3 pair) so it needs its own
     * assertion, not just a comparison against the Vec3 result. */
    TEST_BEGIN("RRayCast GetPointOnRay via the RVec3 path");
    {
        JoltC_RVec3 origin = { 1.5f, -2.25f, 3.75f };
        JoltC_RVec3 p;
        JoltC_Vec3 p_float;
        JoltC_RRayCast_GetPointOnRay(&origin, &kVecB, 0.4f, &p);
        TEST_ASSERT_FLOAT_EQ(p.x,  1.3f,  0.001f, "point at 0.4 x");
        TEST_ASSERT_FLOAT_EQ(p.y, -1.45f, 0.001f, "point at 0.4 y");
        TEST_ASSERT_FLOAT_EQ(p.z,  4.25f, 0.001f, "point at 0.4 z");
        /* Both paths must agree for values a float can hold exactly. */
        JoltC_RayCast_GetPointOnRay(&kVecA, &kVecB, 0.4f, &p_float);
        TEST_ASSERT_FLOAT_EQ(p.x, p_float.x, 0.001f, "RVec3 path == Vec3 path x");
        TEST_ASSERT_FLOAT_EQ(p.y, p_float.y, 0.001f, "RVec3 path == Vec3 path y");
        TEST_ASSERT_FLOAT_EQ(p.z, p_float.z, 0.001f, "RVec3 path == Vec3 path z");
    }
    TEST_END();

    /* ---------------------------------------------------------------- */
    /*  RMat44 — the RVec3-flavoured matrix constructors                */
    /* ---------------------------------------------------------------- */

    TEST_BEGIN("RMat44 Identity and Zero");
    {
        JoltC_Mat44 rident, ident, rzero;
        int i;
        JoltC_RMat44_Identity(&rident);
        JoltC_Mat44_Identity(&ident);
        mr_assert_mat44(&rident, &ident, 0.001f, "RMat44_Identity");
        JoltC_RMat44_Zero(&rzero);
        for (i = 0; i < 16; ++i) {
            TEST_ASSERT(rzero.m[i] == 0.0f, "RMat44_Zero element is zero");
        }
    }
    TEST_END();

    /* The RVec3 translation must land in column 3 in the same order as the
     * Vec3 version — this is exactly where a mixed-up converter would show. */
    TEST_BEGIN("RMat44 Translation known entries");
    {
        const JoltC_Mat44 expected = { {
             1.0f,  0.0f, 0.0f, 0.0f,
             0.0f,  1.0f, 0.0f, 0.0f,
             0.0f,  0.0f, 1.0f, 0.0f,
            -1.25f, 0.5f, 2.0f, 1.0f
        } };
        JoltC_Mat44 rm, m;
        JoltC_Vec3 t;
        JoltC_RMat44_Translation(&rm, &kRTrans);
        mr_assert_mat44(&rm, &expected, 0.001f, "RMat44_Translation");
        JoltC_Mat44_Translation(&m, &kTrans);
        mr_assert_mat44(&rm, &m, 0.001f, "== Mat44_Translation");
        JoltC_Mat44_GetTranslation(&rm, &t);
        MR_ASSERT_VEC3(t, -1.25f, 0.5f, 2.0f, 0.001f, "translation extracted");
    }
    TEST_END();

    TEST_BEGIN("RMat44 Rotation and Scale known entries");
    {
        const JoltC_Mat44 expect_scale = { {
             1.5f,  0.0f,   0.0f,  0.0f,
             0.0f, -2.25f,  0.0f,  0.0f,
             0.0f,  0.0f,   3.75f, 0.0f,
             0.0f,  0.0f,   0.0f,  1.0f
        } };
        JoltC_Mat44 rrot, rscale;
        JoltC_RMat44_Rotation(&rrot, &kQuatA);
        mr_assert_mat44(&rrot, &kRotA, 0.001f, "RMat44_Rotation");
        JoltC_RMat44_Scale(&rscale, &kScale);
        mr_assert_mat44(&rscale, &expect_scale, 0.001f, "RMat44_Scale");
    }
    TEST_END();

    /* Same hand-computed matrix as the Vec3 path, reached through the RVec3
     * translation converter. */
    TEST_BEGIN("RMat44 RotationTranslation known entries");
    {
        JoltC_Mat44 rm, m;
        JoltC_Vec3 p;
        JoltC_RMat44_RotationTranslation(&rm, &kQuatA, &kRTrans);
        mr_assert_mat44(&rm, &kRotTransA, 0.001f, "RMat44_RotationTranslation");
        JoltC_Mat44_RotationTranslation(&m, &kQuatA, &kTrans);
        mr_assert_mat44(&rm, &m, 0.001f, "== Mat44 path");
        /* R*a + t = (-0.71,-2.53,5.45), as in the Mat44 test above. */
        JoltC_Vec3_MultiplyMatrix(&kVecA, &rm, &p);
        MR_ASSERT_VEC3(p, -0.71f, -2.53f, 5.45f, 0.002f, "transformed point");
    }
    TEST_END();

    TEST_BEGIN("RMat44 Inversed known entries and identity");
    {
        JoltC_Mat44 m, inv, product, ident;
        JoltC_RMat44_RotationTranslation(&m, &kQuatA, &kRTrans);
        JoltC_RMat44_Inversed(&m, &inv);
        mr_assert_mat44(&inv, &kInvRotTransA, 0.002f, "RMat44_Inversed");
        JoltC_Mat44_Multiply(&m, &inv, &product);
        JoltC_Mat44_Identity(&ident);
        mr_assert_mat44(&product, &ident, 0.002f, "M * M^-1 == I");
    }
    TEST_END();

    /* ---------------------------------------------------------------- */
    /*  Mat4 / RMat4 — the by-value aliases                             */
    /* ---------------------------------------------------------------- */

    /* These are separate entry points that pass and return whole structs. A
     * struct returned by value is a different ABI path from an out-parameter,
     * so they are checked against the same hand-computed matrices rather than
     * assumed to be thin forwards. */
    TEST_BEGIN("Mat4 by-value constructors match Mat44");
    {
        JoltC_Mat44 ident, zero_by_ptr;
        JoltC_Mat44 by_val;
        int i;

        JoltC_Mat44_Identity(&ident);
        by_val = JoltC_Mat4_Identity();
        mr_assert_mat44(&by_val, &ident, 0.001f, "Mat4_Identity");

        JoltC_Mat44_Zero(&zero_by_ptr);
        by_val = JoltC_Mat4_Zero();
        mr_assert_mat44(&by_val, &zero_by_ptr, 0.001f, "Mat4_Zero");

        by_val = JoltC_Mat4_Rotation(kQuatA);
        mr_assert_mat44(&by_val, &kRotA, 0.001f, "Mat4_Rotation");

        JoltC_Vec3 unnormalized = { 0.2f, -0.4f, 0.4f };
        JoltC_Vec3 axis;
        JoltC_Vec3_Normalized(&unnormalized, &axis);
        by_val = JoltC_Mat4_Rotation2(axis, ANGLE_A);
        mr_assert_mat44(&by_val, &kRotA, 0.002f, "Mat4_Rotation2");

        by_val = JoltC_Mat4_RotationTranslation(kQuatA, kTrans);
        mr_assert_mat44(&by_val, &kRotTransA, 0.001f, "Mat4_RotationTranslation");

        JoltC_Mat44 trans_by_ptr, scale_by_ptr;
        JoltC_Mat44_Translation(&trans_by_ptr, &kTrans);
        by_val = JoltC_Mat4_Translation(kTrans);
        mr_assert_mat44(&by_val, &trans_by_ptr, 0.001f, "Mat4_Translation");

        JoltC_Mat44_Scale(&scale_by_ptr, &kScale);
        by_val = JoltC_Mat4_Scale(kScale);
        mr_assert_mat44(&by_val, &scale_by_ptr, 0.001f, "Mat4_Scale");

        /* Nothing above should have left a stray element behind. */
        for (i = 0; i < 16; ++i) {
            TEST_ASSERT(!isnan(by_val.m[i]), "no NaN in returned struct");
        }
    }
    TEST_END();

    TEST_BEGIN("Mat4 by-value operators match Mat44");
    {
        JoltC_Mat44 rot_trans, rot, ident, t, expected, actual;
        JoltC_Mat44_RotationTranslation(&rot_trans, &kQuatA, &kTrans);
        JoltC_Mat44_Rotation(&rot, &kQuatA);
        JoltC_Mat44_Identity(&ident);
        JoltC_Mat44_Translation(&t, &kTrans);

        actual = JoltC_Mat4_Inversed(rot_trans);
        mr_assert_mat44(&actual, &kInvRotTransA, 0.002f, "Mat4_Inversed");

        /* Careful: unlike Mat44_InverseRotationTranslation(q, t), the Mat4
         * alias of that name takes a *matrix* and inverts it. */
        actual = JoltC_Mat4_InverseRotationTranslation(rot_trans);
        mr_assert_mat44(&actual, &kInvRotTransA, 0.002f, "Mat4_InverseRotationTranslation");

        JoltC_Mat44_Transposed(&rot, &expected);
        actual = JoltC_Mat4_Transposed(rot);
        mr_assert_mat44(&actual, &expected, 0.001f, "Mat4_Transposed");

        JoltC_Mat44_Multiply(&t, &rot, &expected);
        actual = JoltC_Mat4_Multiply(t, rot);
        mr_assert_mat44(&actual, &expected, 0.002f, "Mat4_Multiply");
        mr_assert_mat44(&actual, &kRotTransA, 0.002f, "Mat4_Multiply(T,R) == RT");

        JoltC_Mat44_MultiplyScalar(&t, -2.5f, &expected);
        actual = JoltC_Mat4_MultiplyScalar(t, -2.5f);
        mr_assert_mat44(&actual, &expected, 0.001f, "Mat4_MultiplyScalar");

        JoltC_Mat44_Add(&ident, &t, &expected);
        actual = JoltC_Mat4_Add(ident, t);
        mr_assert_mat44(&actual, &expected, 0.001f, "Mat4_Add");

        JoltC_Mat44_Subtract(&t, &ident, &expected);
        actual = JoltC_Mat4_Subtract(t, ident);
        mr_assert_mat44(&actual, &expected, 0.001f, "Mat4_Subtract");
    }
    TEST_END();

    TEST_BEGIN("Mat4 by-value getters match Mat44");
    {
        JoltC_Mat44 rot, rot_trans;
        JoltC_Vec3 ax, ay, az, t;
        JoltC_Quat q;
        float dot;
        JoltC_Mat44_Rotation(&rot, &kQuatA);
        JoltC_Mat44_RotationTranslation(&rot_trans, &kQuatA, &kTrans);

        ax = JoltC_Mat4_GetAxisX(rot);
        ay = JoltC_Mat4_GetAxisY(rot);
        az = JoltC_Mat4_GetAxisZ(rot);
        MR_ASSERT_VEC3(ax,  0.36f,  0.48f, 0.80f, 0.001f, "Mat4_GetAxisX");
        MR_ASSERT_VEC3(ay, -0.80f,  0.60f, 0.00f, 0.001f, "Mat4_GetAxisY");
        MR_ASSERT_VEC3(az, -0.48f, -0.64f, 0.60f, 0.001f, "Mat4_GetAxisZ");

        t = JoltC_Mat4_GetTranslation(rot_trans);
        MR_ASSERT_VEC3(t, -1.25f, 0.5f, 2.0f, 0.001f, "Mat4_GetTranslation");

        q = JoltC_Mat4_GetQuaternion(rot);
        JoltC_Quat_Dot(&q, &kQuatA, &dot);
        TEST_ASSERT(fabsf(dot) > 0.999f, "Mat4_GetQuaternion up to sign");
    }
    TEST_END();

    TEST_BEGIN("RMat4 by-value aliases match RMat44");
    {
        JoltC_Mat44 expected, actual;
        int i;

        JoltC_RMat44_Identity(&expected);
        actual = JoltC_RMat4_Identity();
        mr_assert_mat44(&actual, &expected, 0.001f, "RMat4_Identity");

        actual = JoltC_RMat4_Zero();
        for (i = 0; i < 16; ++i) {
            TEST_ASSERT(actual.m[i] == 0.0f, "RMat4_Zero element is zero");
        }

        actual = JoltC_RMat4_Rotation(kQuatA);
        mr_assert_mat44(&actual, &kRotA, 0.001f, "RMat4_Rotation");

        actual = JoltC_RMat4_RotationTranslation(kQuatA, kRTrans);
        mr_assert_mat44(&actual, &kRotTransA, 0.001f, "RMat4_RotationTranslation");

        JoltC_RMat44_Translation(&expected, &kRTrans);
        actual = JoltC_RMat4_Translation(kRTrans);
        mr_assert_mat44(&actual, &expected, 0.001f, "RMat4_Translation");

        JoltC_RMat44_Scale(&expected, &kScale);
        actual = JoltC_RMat4_Scale(kScale);
        mr_assert_mat44(&actual, &expected, 0.001f, "RMat4_Scale");

        JoltC_Mat44 rot_trans;
        JoltC_RMat44_RotationTranslation(&rot_trans, &kQuatA, &kRTrans);
        actual = JoltC_RMat4_Inversed(rot_trans);
        mr_assert_mat44(&actual, &kInvRotTransA, 0.002f, "RMat4_Inversed");
        actual = JoltC_RMat4_InverseRotationTranslation(rot_trans);
        mr_assert_mat44(&actual, &kInvRotTransA, 0.002f, "RMat4_InverseRotationTranslation");
    }
    TEST_END();

    /* ---------------------------------------------------------------- */
    /*  MassProperties helpers                                          */
    /* ---------------------------------------------------------------- */

    /* Scaling from mass 2 to mass 6 is a factor of 3, and the inertia tensor
     * scales linearly with mass: diag(1.5, 2.25, 3.75) -> (4.5, 6.75, 11.25). */
    TEST_BEGIN("MassProperties ScaleToMass");
    {
        JoltC_MassProperties props;
        JoltC_Mat44_Identity(&props.inertia);
        props.mass = 2.0f;
        props.inertia.m[0]  = 1.5f;
        props.inertia.m[5]  = 2.25f;
        props.inertia.m[10] = 3.75f;

        JoltC_MassProperties_ScaleToMass(&props, 6.0f);
        TEST_ASSERT_FLOAT_EQ(props.mass, 6.0f, 0.001f, "mass becomes 6");
        TEST_ASSERT_FLOAT_EQ(props.inertia.m[0],  4.5f,  0.002f, "Ixx * 3");
        TEST_ASSERT_FLOAT_EQ(props.inertia.m[5],  6.75f, 0.002f, "Iyy * 3");
        TEST_ASSERT_FLOAT_EQ(props.inertia.m[10], 11.25f, 0.002f, "Izz * 3");
        /* Off-diagonal terms were zero and must stay zero. */
        TEST_ASSERT_FLOAT_EQ(props.inertia.m[1], 0.0f, 0.001f, "no off-diagonal leak");
        TEST_ASSERT_FLOAT_EQ(props.inertia.m[4], 0.0f, 0.001f, "no off-diagonal leak");
    }
    TEST_END();

    /* For a solid box of size (a,b,c) and mass m the diagonal is
     *   Ixx = m/12 (b^2+c^2), Iyy = m/12 (a^2+c^2), Izz = m/12 (a^2+b^2).
     * With m = 6 and (a,b,c) = (1.5, 2.5, 4):
     *   Ixx = 0.5*(6.25+16)  = 11.125
     *   Iyy = 0.5*(2.25+16)  =  9.125
     *   Izz = 0.5*(2.25+6.25)=  4.25
     * so recovering the box size from that diagonal must give (1.5, 2.5, 4) —
     * three different numbers, so a component swap is visible. */
    TEST_BEGIN("MassProperties equivalent solid box size");
    {
        JoltC_Vec3 diagonal = { 11.125f, 9.125f, 4.25f };
        JoltC_Vec3 size;
        JoltC_MassProperties_GetEquivalentSolidBoxSize(6.0f, &diagonal, &size);
        MR_ASSERT_VEC3(size, 1.5f, 2.5f, 4.0f, 0.01f, "recovered box size");
    }
    TEST_END();

    /* Eigen-decomposition of an already diagonal tensor. The ordering of the
     * principal moments is not part of the contract, so this asserts the two
     * things that are: the trace is invariant, and the returned rotation is
     * orthonormal. */
    TEST_BEGIN("MassProperties decompose inertia invariants");
    {
        JoltC_MassProperties props;
        JoltC_Mat44 rotation;
        JoltC_Vec3 diagonal, ax, ay, az;
        float sum;

        JoltC_Mat44_Identity(&props.inertia);
        props.mass = 2.0f;
        props.inertia.m[0]  = 1.5f;
        props.inertia.m[5]  = 2.25f;
        props.inertia.m[10] = 3.75f;

        JoltC_Mat44_Zero(&rotation);
        diagonal.x = diagonal.y = diagonal.z = 0.0f;
        JoltC_MassProperties_DecomposePrincipalMomentsOfInertia(&props, &rotation, &diagonal);

        /* 1.5 + 2.25 + 3.75 = 7.5, whatever order they come back in. */
        sum = diagonal.x + diagonal.y + diagonal.z;
        TEST_ASSERT_FLOAT_EQ(sum, 7.5f, 0.01f, "trace is preserved");

        JoltC_Mat44_GetAxisX(&rotation, &ax);
        JoltC_Mat44_GetAxisY(&rotation, &ay);
        JoltC_Mat44_GetAxisZ(&rotation, &az);
        TEST_ASSERT(JoltC_Vec3_IsNormalized(&ax, 0.01f), "rotation axis X is unit");
        TEST_ASSERT(JoltC_Vec3_IsNormalized(&ay, 0.01f), "rotation axis Y is unit");
        TEST_ASSERT(JoltC_Vec3_IsNormalized(&az, 0.01f), "rotation axis Z is unit");
    }
    TEST_END();
}
