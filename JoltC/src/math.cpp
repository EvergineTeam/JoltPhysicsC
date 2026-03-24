/* JoltC - Math utility function implementations
 * SPDX-License-Identifier: MIT
 */

#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Math/Vec4.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Mat44.h>
#include <Jolt/Math/MathTypes.h>
#include <Jolt/Physics/Body/MassProperties.h>

#include <JoltC/math.h>
#include "internal.h"
#include "errors_internal.h"

#include <cmath>

using namespace JPH;

/* -------------------------------------------------------------------------- */
/*  Math helpers                                                              */
/* -------------------------------------------------------------------------- */
extern "C" {

JOLTC_API float JoltC_Math_Sin(float value) { return std::sin(value); }
JOLTC_API float JoltC_Math_Cos(float value) { return std::cos(value); }

/* -------------------------------------------------------------------------- */
/*  Vec3                                                                      */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_Vec3_AxisX(JoltC_Vec3* result) {
    if (!result) return;
    *result = fromJphVec3(Vec3::sAxisX());
}

JOLTC_API void JoltC_Vec3_AxisY(JoltC_Vec3* result) {
    if (!result) return;
    *result = fromJphVec3(Vec3::sAxisY());
}

JOLTC_API void JoltC_Vec3_AxisZ(JoltC_Vec3* result) {
    if (!result) return;
    *result = fromJphVec3(Vec3::sAxisZ());
}

JOLTC_API int JoltC_Vec3_IsClose(const JoltC_Vec3* v1, const JoltC_Vec3* v2, float maxDistSq) {
    if (!v1 || !v2) return 0;
    return toJphVec3(*v1).IsClose(toJphVec3(*v2), maxDistSq) ? 1 : 0;
}

JOLTC_API int JoltC_Vec3_IsNearZero(const JoltC_Vec3* v, float maxDistSq) {
    if (!v) return 0;
    return toJphVec3(*v).IsNearZero(maxDistSq) ? 1 : 0;
}

JOLTC_API int JoltC_Vec3_IsNormalized(const JoltC_Vec3* v, float tolerance) {
    if (!v) return 0;
    return toJphVec3(*v).IsNormalized(tolerance) ? 1 : 0;
}

JOLTC_API int JoltC_Vec3_IsNaN(const JoltC_Vec3* v) {
    if (!v) return 0;
    return toJphVec3(*v).IsNaN() ? 1 : 0;
}

JOLTC_API void JoltC_Vec3_Negate(const JoltC_Vec3* v, JoltC_Vec3* result) {
    if (!v || !result) return;
    *result = fromJphVec3(-toJphVec3(*v));
}

JOLTC_API void JoltC_Vec3_Normalized(const JoltC_Vec3* v, JoltC_Vec3* result) {
    if (!v || !result) return;
    *result = fromJphVec3(toJphVec3(*v).Normalized());
}

JOLTC_API void JoltC_Vec3_Cross(const JoltC_Vec3* v1, const JoltC_Vec3* v2, JoltC_Vec3* result) {
    if (!v1 || !v2 || !result) return;
    *result = fromJphVec3(toJphVec3(*v1).Cross(toJphVec3(*v2)));
}

JOLTC_API void JoltC_Vec3_Abs(const JoltC_Vec3* v, JoltC_Vec3* result) {
    if (!v || !result) return;
    *result = fromJphVec3(toJphVec3(*v).Abs());
}

JOLTC_API float JoltC_Vec3_Length(const JoltC_Vec3* v) {
    if (!v) return 0.0f;
    return toJphVec3(*v).Length();
}

JOLTC_API float JoltC_Vec3_LengthSquared(const JoltC_Vec3* v) {
    if (!v) return 0.0f;
    return toJphVec3(*v).LengthSq();
}

JOLTC_API void JoltC_Vec3_DotProduct(const JoltC_Vec3* v1, const JoltC_Vec3* v2, float* result) {
    if (!v1 || !v2 || !result) return;
    *result = toJphVec3(*v1).Dot(toJphVec3(*v2));
}

JOLTC_API void JoltC_Vec3_Normalize(const JoltC_Vec3* v, JoltC_Vec3* result) {
    if (!v || !result) return;
    *result = fromJphVec3(toJphVec3(*v).Normalized());
}

JOLTC_API void JoltC_Vec3_Add(const JoltC_Vec3* v1, const JoltC_Vec3* v2, JoltC_Vec3* result) {
    if (!v1 || !v2 || !result) return;
    *result = fromJphVec3(toJphVec3(*v1) + toJphVec3(*v2));
}

JOLTC_API void JoltC_Vec3_Subtract(const JoltC_Vec3* v1, const JoltC_Vec3* v2, JoltC_Vec3* result) {
    if (!v1 || !v2 || !result) return;
    *result = fromJphVec3(toJphVec3(*v1) - toJphVec3(*v2));
}

JOLTC_API void JoltC_Vec3_Multiply(const JoltC_Vec3* v1, const JoltC_Vec3* v2, JoltC_Vec3* result) {
    if (!v1 || !v2 || !result) return;
    *result = fromJphVec3(toJphVec3(*v1) * toJphVec3(*v2));
}

JOLTC_API void JoltC_Vec3_MultiplyScalar(const JoltC_Vec3* v, float scalar, JoltC_Vec3* result) {
    if (!v || !result) return;
    *result = fromJphVec3(toJphVec3(*v) * scalar);
}

JOLTC_API void JoltC_Vec3_Divide(const JoltC_Vec3* v1, const JoltC_Vec3* v2, JoltC_Vec3* result) {
    if (!v1 || !v2 || !result) return;
    *result = fromJphVec3(toJphVec3(*v1) / toJphVec3(*v2));
}

JOLTC_API void JoltC_Vec3_DivideScalar(const JoltC_Vec3* v, float scalar, JoltC_Vec3* result) {
    if (!v || !result) return;
    *result = fromJphVec3(toJphVec3(*v) / scalar);
}

JOLTC_API void JoltC_Vec3_MultiplyMatrix(const JoltC_Vec3* v, const JoltC_Mat44* mat, JoltC_Vec3* result) {
    if (!v || !mat || !result) return;
    *result = fromJphVec3(toJphMat44(*mat) * toJphVec3(*v));
}

/* -------------------------------------------------------------------------- */
/*  Quat                                                                      */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_Quat_FromTo(const JoltC_Vec3* from, const JoltC_Vec3* to, JoltC_Quat* result) {
    if (!from || !to || !result) return;
    *result = fromJphQuat(Quat::sFromTo(toJphVec3(*from), toJphVec3(*to)));
}

JOLTC_API void JoltC_Quat_GetAxisAngle(const JoltC_Quat* quat, JoltC_Vec3* outAxis, float* outAngle) {
    if (!quat || !outAxis || !outAngle) return;
    Vec3 axis;
    float angle;
    toJphQuat(*quat).GetAxisAngle(axis, angle);
    *outAxis = fromJphVec3(axis);
    *outAngle = angle;
}

JOLTC_API void JoltC_Quat_GetEulerAngles(const JoltC_Quat* quat, JoltC_Vec3* result) {
    if (!quat || !result) return;
    *result = fromJphVec3(toJphQuat(*quat).GetEulerAngles());
}

JOLTC_API void JoltC_Quat_RotateAxisX(const JoltC_Quat* quat, JoltC_Vec3* result) {
    if (!quat || !result) return;
    *result = fromJphVec3(toJphQuat(*quat).RotateAxisX());
}

JOLTC_API void JoltC_Quat_RotateAxisY(const JoltC_Quat* quat, JoltC_Vec3* result) {
    if (!quat || !result) return;
    *result = fromJphVec3(toJphQuat(*quat).RotateAxisY());
}

JOLTC_API void JoltC_Quat_RotateAxisZ(const JoltC_Quat* quat, JoltC_Vec3* result) {
    if (!quat || !result) return;
    *result = fromJphVec3(toJphQuat(*quat).RotateAxisZ());
}

JOLTC_API void JoltC_Quat_Inversed(const JoltC_Quat* quat, JoltC_Quat* result) {
    if (!quat || !result) return;
    *result = fromJphQuat(toJphQuat(*quat).Inversed());
}

JOLTC_API float JoltC_Quat_GetRotationAngle(const JoltC_Quat* quat, const JoltC_Vec3* axis) {
    if (!quat || !axis) return 0.0f;
    return toJphQuat(*quat).GetRotationAngle(toJphVec3(*axis));
}

JOLTC_API void JoltC_Quat_FromEulerAngles(const JoltC_Vec3* angles, JoltC_Quat* result) {
    if (!angles || !result) return;
    *result = fromJphQuat(Quat::sEulerAngles(toJphVec3(*angles)));
}

JOLTC_API void JoltC_Quat_Add(const JoltC_Quat* q1, const JoltC_Quat* q2, JoltC_Quat* result) {
    if (!q1 || !q2 || !result) return;
    *result = fromJphQuat(toJphQuat(*q1) + toJphQuat(*q2));
}

JOLTC_API void JoltC_Quat_Subtract(const JoltC_Quat* q1, const JoltC_Quat* q2, JoltC_Quat* result) {
    if (!q1 || !q2 || !result) return;
    *result = fromJphQuat(toJphQuat(*q1) - toJphQuat(*q2));
}

JOLTC_API void JoltC_Quat_Multiply(const JoltC_Quat* q1, const JoltC_Quat* q2, JoltC_Quat* result) {
    if (!q1 || !q2 || !result) return;
    *result = fromJphQuat(toJphQuat(*q1) * toJphQuat(*q2));
}

JOLTC_API void JoltC_Quat_MultiplyScalar(const JoltC_Quat* q, float scalar, JoltC_Quat* result) {
    if (!q || !result) return;
    *result = fromJphQuat(toJphQuat(*q) * scalar);
}

JOLTC_API void JoltC_Quat_DivideScalar(const JoltC_Quat* q, float scalar, JoltC_Quat* result) {
    if (!q || !result) return;
    *result = fromJphQuat(toJphQuat(*q) / scalar);
}

JOLTC_API void JoltC_Quat_Dot(const JoltC_Quat* q1, const JoltC_Quat* q2, float* result) {
    if (!q1 || !q2 || !result) return;
    *result = toJphQuat(*q1).Dot(toJphQuat(*q2));
}

JOLTC_API void JoltC_Quat_Conjugated(const JoltC_Quat* quat, JoltC_Quat* result) {
    if (!quat || !result) return;
    *result = fromJphQuat(toJphQuat(*quat).Conjugated());
}

JOLTC_API void JoltC_Quat_GetTwist(const JoltC_Quat* quat, const JoltC_Vec3* axis, JoltC_Quat* result) {
    if (!quat || !axis || !result) return;
    *result = fromJphQuat(toJphQuat(*quat).GetTwist(toJphVec3(*axis)));
}

JOLTC_API void JoltC_Quat_GetSwingTwist(const JoltC_Quat* quat, JoltC_Quat* outSwing, JoltC_Quat* outTwist) {
    if (!quat || !outSwing || !outTwist) return;
    Quat swing, twist;
    toJphQuat(*quat).GetSwingTwist(swing, twist);
    *outSwing = fromJphQuat(swing);
    *outTwist = fromJphQuat(twist);
}

JOLTC_API void JoltC_Quat_Lerp(const JoltC_Quat* from, const JoltC_Quat* to, float fraction, JoltC_Quat* result) {
    if (!from || !to || !result) return;
    *result = fromJphQuat(toJphQuat(*from).LERP(toJphQuat(*to), fraction));
}

JOLTC_API void JoltC_Quat_Slerp(const JoltC_Quat* from, const JoltC_Quat* to, float fraction, JoltC_Quat* result) {
    if (!from || !to || !result) return;
    *result = fromJphQuat(toJphQuat(*from).SLERP(toJphQuat(*to), fraction));
}

JOLTC_API void JoltC_Quat_Rotate(const JoltC_Quat* quat, const JoltC_Vec3* vec, JoltC_Vec3* result) {
    if (!quat || !vec || !result) return;
    *result = fromJphVec3(toJphQuat(*quat) * toJphVec3(*vec));
}

JOLTC_API void JoltC_Quat_InverseRotate(const JoltC_Quat* quat, const JoltC_Vec3* vec, JoltC_Vec3* result) {
    if (!quat || !vec || !result) return;
    *result = fromJphVec3(toJphQuat(*quat).InverseRotate(toJphVec3(*vec)));
}

JOLTC_API void JoltC_Quat_GetPerpendicular(const JoltC_Quat* quat, JoltC_Quat* result) {
    if (!quat || !result) return;
    *result = fromJphQuat(toJphQuat(*quat).GetPerpendicular());
}

/* -------------------------------------------------------------------------- */
/*  Mat44                                                                     */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_Mat44_Zero(JoltC_Mat44* result) {
    if (!result) return;
    *result = fromJphMat44(Mat44::sZero());
}

JOLTC_API void JoltC_Mat44_Identity(JoltC_Mat44* result) {
    if (!result) return;
    *result = fromJphMat44(Mat44::sIdentity());
}

JOLTC_API void JoltC_Mat44_Rotation(JoltC_Mat44* result, const JoltC_Quat* rotation) {
    if (!result || !rotation) return;
    *result = fromJphMat44(Mat44::sRotation(toJphQuat(*rotation)));
}

JOLTC_API void JoltC_Mat44_Rotation2(JoltC_Mat44* result, const JoltC_Vec3* axis, float angle) {
    if (!result || !axis) return;
    *result = fromJphMat44(Mat44::sRotation(toJphVec3(*axis), angle));
}

JOLTC_API void JoltC_Mat44_Translation(JoltC_Mat44* result, const JoltC_Vec3* translation) {
    if (!result || !translation) return;
    *result = fromJphMat44(Mat44::sTranslation(toJphVec3(*translation)));
}

JOLTC_API void JoltC_Mat44_RotationTranslation(JoltC_Mat44* result, const JoltC_Quat* rotation, const JoltC_Vec3* translation) {
    if (!result || !rotation || !translation) return;
    *result = fromJphMat44(Mat44::sRotationTranslation(toJphQuat(*rotation), toJphVec3(*translation)));
}

JOLTC_API void JoltC_Mat44_InverseRotationTranslation(JoltC_Mat44* result, const JoltC_Quat* rotation, const JoltC_Vec3* translation) {
    if (!result || !rotation || !translation) return;
    *result = fromJphMat44(Mat44::sInverseRotationTranslation(toJphQuat(*rotation), toJphVec3(*translation)));
}

JOLTC_API void JoltC_Mat44_Scale(JoltC_Mat44* result, const JoltC_Vec3* scale) {
    if (!result || !scale) return;
    *result = fromJphMat44(Mat44::sScale(toJphVec3(*scale)));
}

JOLTC_API void JoltC_Mat44_Transposed(const JoltC_Mat44* m, JoltC_Mat44* result) {
    if (!m || !result) return;
    *result = fromJphMat44(toJphMat44(*m).Transposed());
}

JOLTC_API void JoltC_Mat44_Inversed(const JoltC_Mat44* m, JoltC_Mat44* result) {
    if (!m || !result) return;
    *result = fromJphMat44(toJphMat44(*m).Inversed());
}

JOLTC_API void JoltC_Mat44_Add(const JoltC_Mat44* m1, const JoltC_Mat44* m2, JoltC_Mat44* result) {
    if (!m1 || !m2 || !result) return;
    *result = fromJphMat44(toJphMat44(*m1) + toJphMat44(*m2));
}

JOLTC_API void JoltC_Mat44_Subtract(const JoltC_Mat44* m1, const JoltC_Mat44* m2, JoltC_Mat44* result) {
    if (!m1 || !m2 || !result) return;
    *result = fromJphMat44(toJphMat44(*m1) - toJphMat44(*m2));
}

JOLTC_API void JoltC_Mat44_Multiply(const JoltC_Mat44* m1, const JoltC_Mat44* m2, JoltC_Mat44* result) {
    if (!m1 || !m2 || !result) return;
    *result = fromJphMat44(toJphMat44(*m1) * toJphMat44(*m2));
}

JOLTC_API void JoltC_Mat44_MultiplyScalar(const JoltC_Mat44* m, float scalar, JoltC_Mat44* result) {
    if (!m || !result) return;
    *result = fromJphMat44(toJphMat44(*m) * scalar);
}

JOLTC_API void JoltC_Mat44_GetAxisX(const JoltC_Mat44* m, JoltC_Vec3* result) {
    if (!m || !result) return;
    *result = fromJphVec3(toJphMat44(*m).GetAxisX());
}

JOLTC_API void JoltC_Mat44_GetAxisY(const JoltC_Mat44* m, JoltC_Vec3* result) {
    if (!m || !result) return;
    *result = fromJphVec3(toJphMat44(*m).GetAxisY());
}

JOLTC_API void JoltC_Mat44_GetAxisZ(const JoltC_Mat44* m, JoltC_Vec3* result) {
    if (!m || !result) return;
    *result = fromJphVec3(toJphMat44(*m).GetAxisZ());
}

JOLTC_API void JoltC_Mat44_GetTranslation(const JoltC_Mat44* m, JoltC_Vec3* result) {
    if (!m || !result) return;
    *result = fromJphVec3(toJphMat44(*m).GetTranslation());
}

JOLTC_API void JoltC_Mat44_GetQuaternion(const JoltC_Mat44* m, JoltC_Quat* result) {
    if (!m || !result) return;
    *result = fromJphQuat(toJphMat44(*m).GetQuaternion());
}

/* -------------------------------------------------------------------------- */
/*  RMat44                                                                    */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_RMat44_Identity(JoltC_Mat44* result) {
    if (!result) return;
    *result = fromJphMat44(RMat44::sIdentity());
}

JOLTC_API void JoltC_RMat44_Zero(JoltC_Mat44* result) {
    if (!result) return;
    *result = fromJphMat44(RMat44::sZero());
}

JOLTC_API void JoltC_RMat44_Rotation(JoltC_Mat44* result, const JoltC_Quat* rotation) {
    if (!result || !rotation) return;
    *result = fromJphMat44(RMat44::sRotation(toJphQuat(*rotation)));
}

JOLTC_API void JoltC_RMat44_RotationTranslation(JoltC_Mat44* result, const JoltC_Quat* rotation, const JoltC_RVec3* translation) {
    if (!result || !rotation || !translation) return;
    RVec3 t = toJphRVec3(*translation);
    *result = fromJphMat44(RMat44::sRotationTranslation(toJphQuat(*rotation), t));
}

JOLTC_API void JoltC_RMat44_Translation(JoltC_Mat44* result, const JoltC_RVec3* translation) {
    if (!result || !translation) return;
    RVec3 t = toJphRVec3(*translation);
    *result = fromJphMat44(RMat44::sTranslation(t));
}

JOLTC_API void JoltC_RMat44_Scale(JoltC_Mat44* result, const JoltC_Vec3* scale) {
    if (!result || !scale) return;
    *result = fromJphMat44(RMat44::sScale(toJphVec3(*scale)));
}

JOLTC_API void JoltC_RMat44_Inversed(const JoltC_Mat44* m, JoltC_Mat44* result) {
    if (!m || !result) return;
    *result = fromJphMat44(toJphMat44(*m).Inversed());
}

JOLTC_API void JoltC_RMat44_InversedRotationTranslation(const JoltC_Mat44* m, JoltC_Mat44* result) {
    if (!m || !result) return;
    *result = fromJphMat44(toJphMat44(*m).InversedRotationTranslation());
}

/* -------------------------------------------------------------------------- */
/*  RayCast helpers                                                           */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_RayCast_GetPointOnRay(const JoltC_Vec3* origin, const JoltC_Vec3* direction, float fraction, JoltC_Vec3* result) {
    if (!origin || !direction || !result) return;
    *result = fromJphVec3(toJphVec3(*origin) + fraction * toJphVec3(*direction));
}

JOLTC_API void JoltC_RRayCast_GetPointOnRay(const JoltC_RVec3* origin, const JoltC_Vec3* direction, float fraction, JoltC_RVec3* result) {
    if (!origin || !direction || !result) return;
    *result = fromJphRVec3(toJphRVec3(*origin) + RVec3(fraction * toJphVec3(*direction)));
}

/* -------------------------------------------------------------------------- */
/*  MassProperties helpers                                                    */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_MassProperties_DecomposePrincipalMomentsOfInertia(JoltC_MassProperties* properties, JoltC_Mat44* rotation, JoltC_Vec3* diagonal) {
    if (!properties || !rotation || !diagonal) return;
    JOLTC_TRY_BEGIN
    MassProperties mp;
    mp.mMass = properties->mass;
    mp.mInertia = toJphMat44(properties->inertia);
    Mat44 rot;
    Vec3 diag;
    mp.DecomposePrincipalMomentsOfInertia(rot, diag);
    *rotation = fromJphMat44(rot);
    *diagonal = fromJphVec3(diag);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_MassProperties_ScaleToMass(JoltC_MassProperties* properties, float mass) {
    if (!properties) return;
    JOLTC_TRY_BEGIN
    MassProperties mp;
    mp.mMass = properties->mass;
    mp.mInertia = toJphMat44(properties->inertia);
    mp.ScaleToMass(mass);
    properties->mass = mp.mMass;
    properties->inertia = fromJphMat44(mp.mInertia);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_MassProperties_GetEquivalentSolidBoxSize(float mass, const JoltC_Vec3* inertiaDiagonal, JoltC_Vec3* result) {
    if (!inertiaDiagonal || !result) return;
    *result = fromJphVec3(MassProperties::sGetEquivalentSolidBoxSize(mass, toJphVec3(*inertiaDiagonal)));
}

/* -------------------------------------------------------------------------- */
/*  Mat4 aliases (by-value wrappers forwarding to the pointer-based Mat44)    */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_Mat44 JoltC_Mat4_Identity(void) {
    JoltC_Mat44 r; JoltC_Mat44_Identity(&r); return r;
}
JOLTC_API JoltC_Mat44 JoltC_Mat4_Zero(void) {
    JoltC_Mat44 r; JoltC_Mat44_Zero(&r); return r;
}
JOLTC_API JoltC_Mat44 JoltC_Mat4_Rotation(JoltC_Quat rotation) {
    JoltC_Mat44 r; JoltC_Mat44_Rotation(&r, &rotation); return r;
}
JOLTC_API JoltC_Mat44 JoltC_Mat4_Rotation2(JoltC_Vec3 axis, float angle) {
    JoltC_Mat44 r; JoltC_Mat44_Rotation2(&r, &axis, angle); return r;
}
JOLTC_API JoltC_Mat44 JoltC_Mat4_RotationTranslation(JoltC_Quat rotation, JoltC_Vec3 translation) {
    JoltC_Mat44 r; JoltC_Mat44_RotationTranslation(&r, &rotation, &translation); return r;
}
JOLTC_API JoltC_Mat44 JoltC_Mat4_Translation(JoltC_Vec3 translation) {
    JoltC_Mat44 r; JoltC_Mat44_Translation(&r, &translation); return r;
}
JOLTC_API JoltC_Mat44 JoltC_Mat4_Scale(JoltC_Vec3 scale) {
    JoltC_Mat44 r; JoltC_Mat44_Scale(&r, &scale); return r;
}
JOLTC_API JoltC_Mat44 JoltC_Mat4_Inversed(JoltC_Mat44 mat) {
    JoltC_Mat44 r; JoltC_Mat44_Inversed(&mat, &r); return r;
}
JOLTC_API JoltC_Mat44 JoltC_Mat4_InverseRotationTranslation(JoltC_Mat44 mat) {
    JOLTC_TRY_BEGIN
    return fromJphMat44(toJphMat44(mat).InversedRotationTranslation());
    JOLTC_TRY_END
    JoltC_Mat44 z = {}; return z;
}
JOLTC_API JoltC_Mat44 JoltC_Mat4_Transposed(JoltC_Mat44 mat) {
    JoltC_Mat44 r; JoltC_Mat44_Transposed(&mat, &r); return r;
}
JOLTC_API JoltC_Mat44 JoltC_Mat4_Multiply(JoltC_Mat44 a, JoltC_Mat44 b) {
    JoltC_Mat44 r; JoltC_Mat44_Multiply(&a, &b, &r); return r;
}
JOLTC_API JoltC_Mat44 JoltC_Mat4_MultiplyScalar(JoltC_Mat44 mat, float scalar) {
    JoltC_Mat44 r; JoltC_Mat44_MultiplyScalar(&mat, scalar, &r); return r;
}
JOLTC_API JoltC_Mat44 JoltC_Mat4_Add(JoltC_Mat44 a, JoltC_Mat44 b) {
    JoltC_Mat44 r; JoltC_Mat44_Add(&a, &b, &r); return r;
}
JOLTC_API JoltC_Mat44 JoltC_Mat4_Subtract(JoltC_Mat44 a, JoltC_Mat44 b) {
    JoltC_Mat44 r; JoltC_Mat44_Subtract(&a, &b, &r); return r;
}
JOLTC_API JoltC_Vec3 JoltC_Mat4_GetAxisX(JoltC_Mat44 mat) {
    JoltC_Vec3 r; JoltC_Mat44_GetAxisX(&mat, &r); return r;
}
JOLTC_API JoltC_Vec3 JoltC_Mat4_GetAxisY(JoltC_Mat44 mat) {
    JoltC_Vec3 r; JoltC_Mat44_GetAxisY(&mat, &r); return r;
}
JOLTC_API JoltC_Vec3 JoltC_Mat4_GetAxisZ(JoltC_Mat44 mat) {
    JoltC_Vec3 r; JoltC_Mat44_GetAxisZ(&mat, &r); return r;
}
JOLTC_API JoltC_Vec3 JoltC_Mat4_GetTranslation(JoltC_Mat44 mat) {
    JoltC_Vec3 r; JoltC_Mat44_GetTranslation(&mat, &r); return r;
}
JOLTC_API JoltC_Quat JoltC_Mat4_GetQuaternion(JoltC_Mat44 mat) {
    JoltC_Quat r; JoltC_Mat44_GetQuaternion(&mat, &r); return r;
}

/* -------------------------------------------------------------------------- */
/*  RMat4 aliases                                                             */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_Mat44 JoltC_RMat4_Identity(void) {
    JoltC_Mat44 r; JoltC_RMat44_Identity(&r); return r;
}
JOLTC_API JoltC_Mat44 JoltC_RMat4_Zero(void) {
    JoltC_Mat44 r; JoltC_RMat44_Zero(&r); return r;
}
JOLTC_API JoltC_Mat44 JoltC_RMat4_Rotation(JoltC_Quat rotation) {
    JoltC_Mat44 r; JoltC_RMat44_Rotation(&r, &rotation); return r;
}
JOLTC_API JoltC_Mat44 JoltC_RMat4_RotationTranslation(JoltC_Quat rotation, JoltC_RVec3 translation) {
    JoltC_Mat44 r; JoltC_RMat44_RotationTranslation(&r, &rotation, &translation); return r;
}
JOLTC_API JoltC_Mat44 JoltC_RMat4_Translation(JoltC_RVec3 translation) {
    JoltC_Mat44 r; JoltC_RMat44_Translation(&r, &translation); return r;
}
JOLTC_API JoltC_Mat44 JoltC_RMat4_Scale(JoltC_Vec3 scale) {
    JoltC_Mat44 r; JoltC_RMat44_Scale(&r, &scale); return r;
}
JOLTC_API JoltC_Mat44 JoltC_RMat4_Inversed(JoltC_Mat44 mat) {
    JoltC_Mat44 r; JoltC_RMat44_Inversed(&mat, &r); return r;
}
JOLTC_API JoltC_Mat44 JoltC_RMat4_InverseRotationTranslation(JoltC_Mat44 mat) {
    JoltC_Mat44 r; JoltC_RMat44_InversedRotationTranslation(&mat, &r); return r;
}

} /* extern "C" */
