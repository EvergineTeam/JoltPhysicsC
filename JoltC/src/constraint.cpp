/* JoltC - Constraint implementation
 * SPDX-License-Identifier: MIT
 */

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/ConeConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/PulleyConstraint.h>
#include <Jolt/Physics/Constraints/GearConstraint.h>
#include <Jolt/Physics/Constraints/RackAndPinionConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraintPath.h>
#include <Jolt/Physics/Constraints/PathConstraintPathHermite.h>

#include "errors_internal.h"
#include "internal.h"
#include "wrappers.h"

#include <JoltC/constraint.h>

using namespace JPH;

/* -------------------------------------------------------------------------- */
/*  Helpers                                                                   */
/* -------------------------------------------------------------------------- */
static Constraint* asJph(JoltC_Constraint* c) { return c->ptr.GetPtr(); }
static const Constraint* asJph(const JoltC_Constraint* c) { return c->ptr.GetPtr(); }

/* Helper to create a two-body constraint from settings + body IDs.
 * We lock both bodies briefly to get Body references needed by
 * TwoBodyConstraintSettings::Create. */
static JoltC_Constraint* createTwoBody(
    JoltC_PhysicsSystem* system,
    JoltC_BodyID b1, JoltC_BodyID b2,
    const TwoBodyConstraintSettings& settings)
{
    PhysicsSystem& ps = *system->ptr;
    JoltC_BodyID ids[2] = { b1, b2 };
    BodyLockMultiWrite lock(ps.GetBodyLockInterface(),
                            reinterpret_cast<const BodyID*>(ids), 2);
    Body* body1 = lock.GetBody(0);
    Body* body2 = lock.GetBody(1);
    if (!body1 || !body2)
        return nullptr;
    TwoBodyConstraint* c = settings.Create(*body1, *body2);
    auto* wrapper = new JoltC_Constraint();
    wrapper->ptr = c;
    return wrapper;
}

/* -------------------------------------------------------------------------- */
/*  Constraint base                                                           */
/* -------------------------------------------------------------------------- */
extern "C" {

JOLTC_API void JoltC_Constraint_AddRef(JoltC_Constraint* c) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    c->ptr->AddRef();
    JOLTC_TRY_END
}

JOLTC_API void JoltC_Constraint_Release(JoltC_Constraint* c) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    c->ptr = nullptr;
    delete c;
    JOLTC_TRY_END
}

JOLTC_API JoltC_ConstraintSubType JoltC_Constraint_GetSubType(const JoltC_Constraint* c) {
    if (!c) return JOLTC_CONSTRAINT_SUB_TYPE_FIXED;
    JOLTC_TRY_BEGIN
    return static_cast<JoltC_ConstraintSubType>(asJph(c)->GetSubType());
    JOLTC_TRY_END
    return JOLTC_CONSTRAINT_SUB_TYPE_FIXED;
}

JOLTC_API void JoltC_Constraint_SetEnabled(JoltC_Constraint* c, JoltC_Bool enabled) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    asJph(c)->SetEnabled(enabled != 0);
    JOLTC_TRY_END
}

JOLTC_API JoltC_Bool JoltC_Constraint_GetEnabled(const JoltC_Constraint* c) {
    if (!c) return JOLTC_FALSE;
    JOLTC_TRY_BEGIN
    return asJph(c)->GetEnabled() ? JOLTC_TRUE : JOLTC_FALSE;
    JOLTC_TRY_END
    return JOLTC_FALSE;
}

JOLTC_API void JoltC_Constraint_SetUserData(JoltC_Constraint* c, uint64_t userData) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    asJph(c)->SetUserData(userData);
    JOLTC_TRY_END
}

JOLTC_API uint64_t JoltC_Constraint_GetUserData(const JoltC_Constraint* c) {
    if (!c) return 0;
    JOLTC_TRY_BEGIN
    return asJph(c)->GetUserData();
    JOLTC_TRY_END
    return 0;
}

/* -------------------------------------------------------------------------- */
/*  PhysicsSystem constraint add/remove                                       */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_PhysicsSystem_AddConstraint(JoltC_PhysicsSystem* system, JoltC_Constraint* c) {
    if (!system || !c) return;
    JOLTC_TRY_BEGIN
    system->ptr->AddConstraint(c->ptr.GetPtr());
    JOLTC_TRY_END
}

JOLTC_API void JoltC_PhysicsSystem_RemoveConstraint(JoltC_PhysicsSystem* system, JoltC_Constraint* c) {
    if (!system || !c) return;
    JOLTC_TRY_BEGIN
    system->ptr->RemoveConstraint(c->ptr.GetPtr());
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  PointConstraint                                                           */
/* -------------------------------------------------------------------------- */
/* The fill helpers exist so the same conversion feeds two consumers: Create, which solves the
 * constraint between two live bodies, and CreateSettings, which hands the settings object itself
 * to whoever needs one unattached -- ragdoll parts above all, whose SetPartToParent takes settings
 * and had no way to obtain any. */
static void fillPoint(PointConstraintSettings& settings, const JoltC_PointConstraintSettings* s)
{
    settings.mSpace = toJphConstraintSpace(s->space);
    settings.mPoint1 = toJphRVec3(s->point1);
    settings.mPoint2 = toJphRVec3(s->point2);
}

/* Wraps a freshly built settings object for the C side with one reference the caller owns. */
static JoltC_TwoBodyConstraintSettings* wrapTwoBodySettings(TwoBodyConstraintSettings* settings)
{
    settings->AddRef();
    return reinterpret_cast<JoltC_TwoBodyConstraintSettings*>(settings);
}

JOLTC_API JoltC_Constraint* JoltC_PointConstraint_Create(
    JoltC_PhysicsSystem* system, JoltC_BodyID b1, JoltC_BodyID b2,
    const JoltC_PointConstraintSettings* s)
{
    if (!system || !s) return nullptr;
    JOLTC_TRY_BEGIN
    PointConstraintSettings settings;
    fillPoint(settings, s);
    return createTwoBody(system, b1, b2, settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_TwoBodyConstraintSettings* JoltC_PointConstraintSettings_CreateSettings(const JoltC_PointConstraintSettings* s)
{
    if (!s) return nullptr;
    JOLTC_TRY_BEGIN
    auto* settings = new PointConstraintSettings();
    fillPoint(*settings, s);
    return wrapTwoBodySettings(settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_PointConstraint_SetPoint1(JoltC_Constraint* c, JoltC_ConstraintSpace space, JoltC_RVec3 point) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    static_cast<PointConstraint*>(asJph(c))->SetPoint1(toJphConstraintSpace(space), toJphRVec3(point));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_PointConstraint_SetPoint2(JoltC_Constraint* c, JoltC_ConstraintSpace space, JoltC_RVec3 point) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    static_cast<PointConstraint*>(asJph(c))->SetPoint2(toJphConstraintSpace(space), toJphRVec3(point));
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  FixedConstraint                                                           */
/* -------------------------------------------------------------------------- */
static void fillFixed(FixedConstraintSettings& settings, const JoltC_FixedConstraintSettings* s)
{
    settings.mSpace = toJphConstraintSpace(s->space);
    settings.mAutoDetectPoint = s->autoDetectPoint != 0;
    settings.mPoint1 = toJphRVec3(s->point1);
    settings.mAxisX1 = toJphVec3(s->axisX1);
    settings.mAxisY1 = toJphVec3(s->axisY1);
    settings.mPoint2 = toJphRVec3(s->point2);
    settings.mAxisX2 = toJphVec3(s->axisX2);
    settings.mAxisY2 = toJphVec3(s->axisY2);
}

JOLTC_API JoltC_Constraint* JoltC_FixedConstraint_Create(
    JoltC_PhysicsSystem* system, JoltC_BodyID b1, JoltC_BodyID b2,
    const JoltC_FixedConstraintSettings* s)
{
    if (!system || !s) return nullptr;
    JOLTC_TRY_BEGIN
    FixedConstraintSettings settings;
    fillFixed(settings, s);
    return createTwoBody(system, b1, b2, settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_TwoBodyConstraintSettings* JoltC_FixedConstraintSettings_CreateSettings(const JoltC_FixedConstraintSettings* s)
{
    if (!s) return nullptr;
    JOLTC_TRY_BEGIN
    auto* settings = new FixedConstraintSettings();
    fillFixed(*settings, s);
    return wrapTwoBodySettings(settings);
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  DistanceConstraint                                                        */
/* -------------------------------------------------------------------------- */
static void fillDistance(DistanceConstraintSettings& settings, const JoltC_DistanceConstraintSettings* s)
{
    settings.mSpace = toJphConstraintSpace(s->space);
    settings.mPoint1 = toJphRVec3(s->point1);
    settings.mPoint2 = toJphRVec3(s->point2);
    settings.mMinDistance = s->minDistance;
    settings.mMaxDistance = s->maxDistance;
    settings.mLimitsSpringSettings = toJphSpringSettings(s->limitsSpringSettings);
}

JOLTC_API JoltC_Constraint* JoltC_DistanceConstraint_Create(
    JoltC_PhysicsSystem* system, JoltC_BodyID b1, JoltC_BodyID b2,
    const JoltC_DistanceConstraintSettings* s)
{
    if (!system || !s) return nullptr;
    JOLTC_TRY_BEGIN
    DistanceConstraintSettings settings;
    fillDistance(settings, s);
    return createTwoBody(system, b1, b2, settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_TwoBodyConstraintSettings* JoltC_DistanceConstraintSettings_CreateSettings(const JoltC_DistanceConstraintSettings* s)
{
    if (!s) return nullptr;
    JOLTC_TRY_BEGIN
    auto* settings = new DistanceConstraintSettings();
    fillDistance(*settings, s);
    return wrapTwoBodySettings(settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_DistanceConstraint_SetDistance(JoltC_Constraint* c, float minDist, float maxDist) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    static_cast<DistanceConstraint*>(asJph(c))->SetDistance(minDist, maxDist);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_DistanceConstraint_GetMinDistance(const JoltC_Constraint* c) {
    if (!c) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const DistanceConstraint*>(asJph(c))->GetMinDistance();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API float JoltC_DistanceConstraint_GetMaxDistance(const JoltC_Constraint* c) {
    if (!c) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const DistanceConstraint*>(asJph(c))->GetMaxDistance();
    JOLTC_TRY_END
    return 0;
}

/* -------------------------------------------------------------------------- */
/*  HingeConstraint                                                           */
/* -------------------------------------------------------------------------- */
static void fillHinge(HingeConstraintSettings& settings, const JoltC_HingeConstraintSettings* s)
{
    settings.mSpace = toJphConstraintSpace(s->space);
    settings.mPoint1 = toJphRVec3(s->point1);
    settings.mHingeAxis1 = toJphVec3(s->hingeAxis1);
    settings.mNormalAxis1 = toJphVec3(s->normalAxis1);
    settings.mPoint2 = toJphRVec3(s->point2);
    settings.mHingeAxis2 = toJphVec3(s->hingeAxis2);
    settings.mNormalAxis2 = toJphVec3(s->normalAxis2);
    settings.mLimitsMin = s->limitsMin;
    settings.mLimitsMax = s->limitsMax;
    settings.mLimitsSpringSettings = toJphSpringSettings(s->limitsSpringSettings);
    settings.mMaxFrictionTorque = s->maxFrictionTorque;
    settings.mMotorSettings = toJphMotorSettings(s->motorSettings);
}

JOLTC_API JoltC_Constraint* JoltC_HingeConstraint_Create(
    JoltC_PhysicsSystem* system, JoltC_BodyID b1, JoltC_BodyID b2,
    const JoltC_HingeConstraintSettings* s)
{
    if (!system || !s) return nullptr;
    JOLTC_TRY_BEGIN
    HingeConstraintSettings settings;
    fillHinge(settings, s);
    return createTwoBody(system, b1, b2, settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_TwoBodyConstraintSettings* JoltC_HingeConstraintSettings_CreateSettings(const JoltC_HingeConstraintSettings* s)
{
    if (!s) return nullptr;
    JOLTC_TRY_BEGIN
    auto* settings = new HingeConstraintSettings();
    fillHinge(*settings, s);
    return wrapTwoBodySettings(settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API float JoltC_HingeConstraint_GetCurrentAngle(const JoltC_Constraint* c) {
    if (!c) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const HingeConstraint*>(asJph(c))->GetCurrentAngle();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_HingeConstraint_SetMotorState(JoltC_Constraint* c, JoltC_MotorState state) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    static_cast<HingeConstraint*>(asJph(c))->SetMotorState(toJphMotorState(state));
    JOLTC_TRY_END
}

JOLTC_API JoltC_MotorState JoltC_HingeConstraint_GetMotorState(const JoltC_Constraint* c) {
    if (!c) return JOLTC_MOTOR_STATE_OFF;
    JOLTC_TRY_BEGIN
    return fromJphMotorState(static_cast<const HingeConstraint*>(asJph(c))->GetMotorState());
    JOLTC_TRY_END
    return JOLTC_MOTOR_STATE_OFF;
}

JOLTC_API void JoltC_HingeConstraint_SetTargetAngularVelocity(JoltC_Constraint* c, float v) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    static_cast<HingeConstraint*>(asJph(c))->SetTargetAngularVelocity(v);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_HingeConstraint_GetTargetAngularVelocity(const JoltC_Constraint* c) {
    if (!c) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const HingeConstraint*>(asJph(c))->GetTargetAngularVelocity();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_HingeConstraint_SetTargetAngle(JoltC_Constraint* c, float a) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    static_cast<HingeConstraint*>(asJph(c))->SetTargetAngle(a);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_HingeConstraint_SetTargetOrientationBS(JoltC_Constraint* c, JoltC_Quat orientation) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    static_cast<HingeConstraint*>(asJph(c))->SetTargetOrientationBS(toJphQuat(orientation));
    JOLTC_TRY_END
}

JOLTC_API float JoltC_HingeConstraint_GetTargetAngle(const JoltC_Constraint* c) {
    if (!c) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const HingeConstraint*>(asJph(c))->GetTargetAngle();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_HingeConstraint_SetLimits(JoltC_Constraint* c, float mn, float mx) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    static_cast<HingeConstraint*>(asJph(c))->SetLimits(mn, mx);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_HingeConstraint_GetLimitsMin(const JoltC_Constraint* c) {
    if (!c) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const HingeConstraint*>(asJph(c))->GetLimitsMin();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API float JoltC_HingeConstraint_GetLimitsMax(const JoltC_Constraint* c) {
    if (!c) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const HingeConstraint*>(asJph(c))->GetLimitsMax();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API JoltC_Bool JoltC_HingeConstraint_HasLimits(const JoltC_Constraint* c) {
    if (!c) return JOLTC_FALSE;
    JOLTC_TRY_BEGIN
    return static_cast<const HingeConstraint*>(asJph(c))->HasLimits() ? JOLTC_TRUE : JOLTC_FALSE;
    JOLTC_TRY_END
    return JOLTC_FALSE;
}

JOLTC_API void JoltC_HingeConstraint_SetMaxFrictionTorque(JoltC_Constraint* c, float torque) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    static_cast<HingeConstraint*>(asJph(c))->SetMaxFrictionTorque(torque);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_HingeConstraint_GetMaxFrictionTorque(const JoltC_Constraint* c) {
    if (!c) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const HingeConstraint*>(asJph(c))->GetMaxFrictionTorque();
    JOLTC_TRY_END
    return 0;
}

/* -------------------------------------------------------------------------- */
/*  SliderConstraint                                                          */
/* -------------------------------------------------------------------------- */
static void fillSlider(SliderConstraintSettings& settings, const JoltC_SliderConstraintSettings* s)
{
    settings.mSpace = toJphConstraintSpace(s->space);
    settings.mAutoDetectPoint = s->autoDetectPoint != 0;
    settings.mPoint1 = toJphRVec3(s->point1);
    settings.mSliderAxis1 = toJphVec3(s->sliderAxis1);
    settings.mNormalAxis1 = toJphVec3(s->normalAxis1);
    settings.mPoint2 = toJphRVec3(s->point2);
    settings.mSliderAxis2 = toJphVec3(s->sliderAxis2);
    settings.mNormalAxis2 = toJphVec3(s->normalAxis2);
    settings.mLimitsMin = s->limitsMin;
    settings.mLimitsMax = s->limitsMax;
    settings.mLimitsSpringSettings = toJphSpringSettings(s->limitsSpringSettings);
    settings.mMaxFrictionForce = s->maxFrictionForce;
    settings.mMotorSettings = toJphMotorSettings(s->motorSettings);
}

JOLTC_API JoltC_Constraint* JoltC_SliderConstraint_Create(
    JoltC_PhysicsSystem* system, JoltC_BodyID b1, JoltC_BodyID b2,
    const JoltC_SliderConstraintSettings* s)
{
    if (!system || !s) return nullptr;
    JOLTC_TRY_BEGIN
    SliderConstraintSettings settings;
    fillSlider(settings, s);
    return createTwoBody(system, b1, b2, settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_TwoBodyConstraintSettings* JoltC_SliderConstraintSettings_CreateSettings(const JoltC_SliderConstraintSettings* s)
{
    if (!s) return nullptr;
    JOLTC_TRY_BEGIN
    auto* settings = new SliderConstraintSettings();
    fillSlider(*settings, s);
    return wrapTwoBodySettings(settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API float JoltC_SliderConstraint_GetCurrentPosition(const JoltC_Constraint* c) {
    if (!c) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const SliderConstraint*>(asJph(c))->GetCurrentPosition();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_SliderConstraint_SetMotorState(JoltC_Constraint* c, JoltC_MotorState state) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    static_cast<SliderConstraint*>(asJph(c))->SetMotorState(toJphMotorState(state));
    JOLTC_TRY_END
}

JOLTC_API JoltC_MotorState JoltC_SliderConstraint_GetMotorState(const JoltC_Constraint* c) {
    if (!c) return JOLTC_MOTOR_STATE_OFF;
    JOLTC_TRY_BEGIN
    return fromJphMotorState(static_cast<const SliderConstraint*>(asJph(c))->GetMotorState());
    JOLTC_TRY_END
    return JOLTC_MOTOR_STATE_OFF;
}

JOLTC_API void JoltC_SliderConstraint_SetTargetVelocity(JoltC_Constraint* c, float v) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    static_cast<SliderConstraint*>(asJph(c))->SetTargetVelocity(v);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_SliderConstraint_GetTargetVelocity(const JoltC_Constraint* c) {
    if (!c) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const SliderConstraint*>(asJph(c))->GetTargetVelocity();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_SliderConstraint_SetTargetPosition(JoltC_Constraint* c, float p) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    static_cast<SliderConstraint*>(asJph(c))->SetTargetPosition(p);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_SliderConstraint_GetTargetPosition(const JoltC_Constraint* c) {
    if (!c) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const SliderConstraint*>(asJph(c))->GetTargetPosition();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_SliderConstraint_SetLimits(JoltC_Constraint* c, float mn, float mx) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    static_cast<SliderConstraint*>(asJph(c))->SetLimits(mn, mx);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_SliderConstraint_GetLimitsMin(const JoltC_Constraint* c) {
    if (!c) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const SliderConstraint*>(asJph(c))->GetLimitsMin();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API float JoltC_SliderConstraint_GetLimitsMax(const JoltC_Constraint* c) {
    if (!c) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const SliderConstraint*>(asJph(c))->GetLimitsMax();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API JoltC_Bool JoltC_SliderConstraint_HasLimits(const JoltC_Constraint* c) {
    if (!c) return JOLTC_FALSE;
    JOLTC_TRY_BEGIN
    return static_cast<const SliderConstraint*>(asJph(c))->HasLimits() ? JOLTC_TRUE : JOLTC_FALSE;
    JOLTC_TRY_END
    return JOLTC_FALSE;
}

JOLTC_API void JoltC_SliderConstraint_SetMaxFrictionForce(JoltC_Constraint* c, float force) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    static_cast<SliderConstraint*>(asJph(c))->SetMaxFrictionForce(force);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_SliderConstraint_GetMaxFrictionForce(const JoltC_Constraint* c) {
    if (!c) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const SliderConstraint*>(asJph(c))->GetMaxFrictionForce();
    JOLTC_TRY_END
    return 0;
}

/* -------------------------------------------------------------------------- */
/*  ConeConstraint                                                            */
/* -------------------------------------------------------------------------- */
static void fillCone(ConeConstraintSettings& settings, const JoltC_ConeConstraintSettings* s)
{
    settings.mSpace = toJphConstraintSpace(s->space);
    settings.mPoint1 = toJphRVec3(s->point1);
    settings.mTwistAxis1 = toJphVec3(s->twistAxis1);
    settings.mPoint2 = toJphRVec3(s->point2);
    settings.mTwistAxis2 = toJphVec3(s->twistAxis2);
    settings.mHalfConeAngle = s->halfConeAngle;
}

JOLTC_API JoltC_Constraint* JoltC_ConeConstraint_Create(
    JoltC_PhysicsSystem* system, JoltC_BodyID b1, JoltC_BodyID b2,
    const JoltC_ConeConstraintSettings* s)
{
    if (!system || !s) return nullptr;
    JOLTC_TRY_BEGIN
    ConeConstraintSettings settings;
    fillCone(settings, s);
    return createTwoBody(system, b1, b2, settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_TwoBodyConstraintSettings* JoltC_ConeConstraintSettings_CreateSettings(const JoltC_ConeConstraintSettings* s)
{
    if (!s) return nullptr;
    JOLTC_TRY_BEGIN
    auto* settings = new ConeConstraintSettings();
    fillCone(*settings, s);
    return wrapTwoBodySettings(settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_ConeConstraint_SetHalfConeAngle(JoltC_Constraint* c, float angle) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    static_cast<ConeConstraint*>(asJph(c))->SetHalfConeAngle(angle);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_ConeConstraint_GetCosHalfConeAngle(const JoltC_Constraint* c) {
    if (!c) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const ConeConstraint*>(asJph(c))->GetCosHalfConeAngle();
    JOLTC_TRY_END
    return 0;
}

/* -------------------------------------------------------------------------- */
/*  SwingTwistConstraint                                                      */
/* -------------------------------------------------------------------------- */
static void fillSwingTwist(SwingTwistConstraintSettings& settings, const JoltC_SwingTwistConstraintSettings* s)
{
    settings.mSpace = toJphConstraintSpace(s->space);
    settings.mPosition1 = toJphRVec3(s->position1);
    settings.mTwistAxis1 = toJphVec3(s->twistAxis1);
    settings.mPlaneAxis1 = toJphVec3(s->planeAxis1);
    settings.mPosition2 = toJphRVec3(s->position2);
    settings.mTwistAxis2 = toJphVec3(s->twistAxis2);
    settings.mPlaneAxis2 = toJphVec3(s->planeAxis2);
    settings.mSwingType = toJphSwingType(s->swingType);
    settings.mNormalHalfConeAngle = s->normalHalfConeAngle;
    settings.mPlaneHalfConeAngle = s->planeHalfConeAngle;
    settings.mTwistMinAngle = s->twistMinAngle;
    settings.mTwistMaxAngle = s->twistMaxAngle;
    settings.mMaxFrictionTorque = s->maxFrictionTorque;
    settings.mSwingMotorSettings = toJphMotorSettings(s->swingMotorSettings);
    settings.mTwistMotorSettings = toJphMotorSettings(s->twistMotorSettings);
}

JOLTC_API JoltC_Constraint* JoltC_SwingTwistConstraint_Create(
    JoltC_PhysicsSystem* system, JoltC_BodyID b1, JoltC_BodyID b2,
    const JoltC_SwingTwistConstraintSettings* s)
{
    if (!system || !s) return nullptr;
    JOLTC_TRY_BEGIN
    SwingTwistConstraintSettings settings;
    fillSwingTwist(settings, s);
    return createTwoBody(system, b1, b2, settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_TwoBodyConstraintSettings* JoltC_SwingTwistConstraintSettings_CreateSettings(const JoltC_SwingTwistConstraintSettings* s)
{
    if (!s) return nullptr;
    JOLTC_TRY_BEGIN
    auto* settings = new SwingTwistConstraintSettings();
    fillSwingTwist(*settings, s);
    return wrapTwoBodySettings(settings);
    JOLTC_TRY_END
    return nullptr;
}

#define ST_CAST(c) static_cast<SwingTwistConstraint*>(asJph(c))
#define ST_CCAST(c) static_cast<const SwingTwistConstraint*>(asJph(c))

JOLTC_API void JoltC_SwingTwistConstraint_SetNormalHalfConeAngle(JoltC_Constraint* c, float a) { if (!c) return; JOLTC_TRY_BEGIN ST_CAST(c)->SetNormalHalfConeAngle(a); JOLTC_TRY_END }
JOLTC_API float JoltC_SwingTwistConstraint_GetNormalHalfConeAngle(const JoltC_Constraint* c) { if (!c) return 0; JOLTC_TRY_BEGIN return ST_CCAST(c)->GetNormalHalfConeAngle(); JOLTC_TRY_END return 0; }
JOLTC_API void JoltC_SwingTwistConstraint_SetPlaneHalfConeAngle(JoltC_Constraint* c, float a) { if (!c) return; JOLTC_TRY_BEGIN ST_CAST(c)->SetPlaneHalfConeAngle(a); JOLTC_TRY_END }
JOLTC_API float JoltC_SwingTwistConstraint_GetPlaneHalfConeAngle(const JoltC_Constraint* c) { if (!c) return 0; JOLTC_TRY_BEGIN return ST_CCAST(c)->GetPlaneHalfConeAngle(); JOLTC_TRY_END return 0; }
JOLTC_API void JoltC_SwingTwistConstraint_SetTwistMinAngle(JoltC_Constraint* c, float a) { if (!c) return; JOLTC_TRY_BEGIN ST_CAST(c)->SetTwistMinAngle(a); JOLTC_TRY_END }
JOLTC_API float JoltC_SwingTwistConstraint_GetTwistMinAngle(const JoltC_Constraint* c) { if (!c) return 0; JOLTC_TRY_BEGIN return ST_CCAST(c)->GetTwistMinAngle(); JOLTC_TRY_END return 0; }
JOLTC_API void JoltC_SwingTwistConstraint_SetTwistMaxAngle(JoltC_Constraint* c, float a) { if (!c) return; JOLTC_TRY_BEGIN ST_CAST(c)->SetTwistMaxAngle(a); JOLTC_TRY_END }
JOLTC_API float JoltC_SwingTwistConstraint_GetTwistMaxAngle(const JoltC_Constraint* c) { if (!c) return 0; JOLTC_TRY_BEGIN return ST_CCAST(c)->GetTwistMaxAngle(); JOLTC_TRY_END return 0; }

JOLTC_API void JoltC_SwingTwistConstraint_SetSwingMotorState(JoltC_Constraint* c, JoltC_MotorState s) { if (!c) return; JOLTC_TRY_BEGIN ST_CAST(c)->SetSwingMotorState(toJphMotorState(s)); JOLTC_TRY_END }
JOLTC_API JoltC_MotorState JoltC_SwingTwistConstraint_GetSwingMotorState(const JoltC_Constraint* c) { if (!c) return JOLTC_MOTOR_STATE_OFF; JOLTC_TRY_BEGIN return fromJphMotorState(ST_CCAST(c)->GetSwingMotorState()); JOLTC_TRY_END return JOLTC_MOTOR_STATE_OFF; }
JOLTC_API void JoltC_SwingTwistConstraint_SetTwistMotorState(JoltC_Constraint* c, JoltC_MotorState s) { if (!c) return; JOLTC_TRY_BEGIN ST_CAST(c)->SetTwistMotorState(toJphMotorState(s)); JOLTC_TRY_END }
JOLTC_API JoltC_MotorState JoltC_SwingTwistConstraint_GetTwistMotorState(const JoltC_Constraint* c) { if (!c) return JOLTC_MOTOR_STATE_OFF; JOLTC_TRY_BEGIN return fromJphMotorState(ST_CCAST(c)->GetTwistMotorState()); JOLTC_TRY_END return JOLTC_MOTOR_STATE_OFF; }
JOLTC_API void JoltC_SwingTwistConstraint_SetTargetAngularVelocityCS(JoltC_Constraint* c, JoltC_Vec3 v) { if (!c) return; JOLTC_TRY_BEGIN ST_CAST(c)->SetTargetAngularVelocityCS(toJphVec3(v)); JOLTC_TRY_END }
JOLTC_API JoltC_Vec3 JoltC_SwingTwistConstraint_GetTargetAngularVelocityCS(const JoltC_Constraint* c) { JoltC_Vec3 z = {0,0,0}; if (!c) return z; JOLTC_TRY_BEGIN return fromJphVec3(ST_CCAST(c)->GetTargetAngularVelocityCS()); JOLTC_TRY_END return z; }
JOLTC_API void JoltC_SwingTwistConstraint_SetTargetOrientationCS(JoltC_Constraint* c, JoltC_Quat o) { if (!c) return; JOLTC_TRY_BEGIN ST_CAST(c)->SetTargetOrientationCS(toJphQuat(o)); JOLTC_TRY_END }
JOLTC_API void JoltC_SwingTwistConstraint_SetTargetOrientationBS(JoltC_Constraint* c, JoltC_Quat o) { if (!c) return; JOLTC_TRY_BEGIN ST_CAST(c)->SetTargetOrientationBS(toJphQuat(o)); JOLTC_TRY_END }
JOLTC_API JoltC_Quat JoltC_SwingTwistConstraint_GetTargetOrientationCS(const JoltC_Constraint* c) { JoltC_Quat z = {0,0,0,1}; if (!c) return z; JOLTC_TRY_BEGIN return fromJphQuat(ST_CCAST(c)->GetTargetOrientationCS()); JOLTC_TRY_END return z; }
JOLTC_API void JoltC_SwingTwistConstraint_SetMaxFrictionTorque(JoltC_Constraint* c, float t) { if (!c) return; JOLTC_TRY_BEGIN ST_CAST(c)->SetMaxFrictionTorque(t); JOLTC_TRY_END }
JOLTC_API float JoltC_SwingTwistConstraint_GetMaxFrictionTorque(const JoltC_Constraint* c) { if (!c) return 0; JOLTC_TRY_BEGIN return ST_CCAST(c)->GetMaxFrictionTorque(); JOLTC_TRY_END return 0; }

#undef ST_CAST
#undef ST_CCAST

/* -------------------------------------------------------------------------- */
/*  SixDOFConstraint                                                          */
/* -------------------------------------------------------------------------- */
static void fillSixDOF(SixDOFConstraintSettings& settings, const JoltC_SixDOFConstraintSettings* s)
{
    settings.mSpace = toJphConstraintSpace(s->space);
    settings.mPosition1 = toJphRVec3(s->position1);
    settings.mAxisX1 = toJphVec3(s->axisX1);
    settings.mAxisY1 = toJphVec3(s->axisY1);
    settings.mPosition2 = toJphRVec3(s->position2);
    settings.mAxisX2 = toJphVec3(s->axisX2);
    settings.mAxisY2 = toJphVec3(s->axisY2);
    settings.mSwingType = toJphSwingType(s->swingType);
    for (int i = 0; i < 6; i++) {
        settings.mMaxFriction[i] = s->maxFriction[i];
        settings.mLimitMin[i] = s->limitMin[i];
        settings.mLimitMax[i] = s->limitMax[i];
        settings.mMotorSettings[i] = toJphMotorSettings(s->motorSettings[i]);
    }
    for (int i = 0; i < 3; i++)
        settings.mLimitsSpringSettings[i] = toJphSpringSettings(s->limitsSpringSettings[i]);
}

JOLTC_API JoltC_Constraint* JoltC_SixDOFConstraint_Create(
    JoltC_PhysicsSystem* system, JoltC_BodyID b1, JoltC_BodyID b2,
    const JoltC_SixDOFConstraintSettings* s)
{
    if (!system || !s) return nullptr;
    JOLTC_TRY_BEGIN
    SixDOFConstraintSettings settings;
    fillSixDOF(settings, s);
    return createTwoBody(system, b1, b2, settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_TwoBodyConstraintSettings* JoltC_SixDOFConstraintSettings_CreateSettings(const JoltC_SixDOFConstraintSettings* s)
{
    if (!s) return nullptr;
    JOLTC_TRY_BEGIN
    auto* settings = new SixDOFConstraintSettings();
    fillSixDOF(*settings, s);
    return wrapTwoBodySettings(settings);
    JOLTC_TRY_END
    return nullptr;
}

#define SD_CAST(c) static_cast<SixDOFConstraint*>(asJph(c))
#define SD_CCAST(c) static_cast<const SixDOFConstraint*>(asJph(c))

JOLTC_API void JoltC_SixDOFConstraint_SetTranslationLimits(JoltC_Constraint* c, JoltC_Vec3 mn, JoltC_Vec3 mx) { if (!c) return; JOLTC_TRY_BEGIN SD_CAST(c)->SetTranslationLimits(toJphVec3(mn), toJphVec3(mx)); JOLTC_TRY_END }
JOLTC_API void JoltC_SixDOFConstraint_SetRotationLimits(JoltC_Constraint* c, JoltC_Vec3 mn, JoltC_Vec3 mx) { if (!c) return; JOLTC_TRY_BEGIN SD_CAST(c)->SetRotationLimits(toJphVec3(mn), toJphVec3(mx)); JOLTC_TRY_END }
JOLTC_API float JoltC_SixDOFConstraint_GetLimitsMin(const JoltC_Constraint* c, JoltC_SixDOFConstraintAxis a) { if (!c) return 0; JOLTC_TRY_BEGIN return SD_CCAST(c)->GetLimitsMin(static_cast<SixDOFConstraintSettings::EAxis>(a)); JOLTC_TRY_END return 0; }
JOLTC_API float JoltC_SixDOFConstraint_GetLimitsMax(const JoltC_Constraint* c, JoltC_SixDOFConstraintAxis a) { if (!c) return 0; JOLTC_TRY_BEGIN return SD_CCAST(c)->GetLimitsMax(static_cast<SixDOFConstraintSettings::EAxis>(a)); JOLTC_TRY_END return 0; }
JOLTC_API void JoltC_SixDOFConstraint_SetMotorState(JoltC_Constraint* c, JoltC_SixDOFConstraintAxis a, JoltC_MotorState s) { if (!c) return; JOLTC_TRY_BEGIN SD_CAST(c)->SetMotorState(static_cast<SixDOFConstraintSettings::EAxis>(a), toJphMotorState(s)); JOLTC_TRY_END }
JOLTC_API JoltC_MotorState JoltC_SixDOFConstraint_GetMotorState(const JoltC_Constraint* c, JoltC_SixDOFConstraintAxis a) { if (!c) return JOLTC_MOTOR_STATE_OFF; JOLTC_TRY_BEGIN return fromJphMotorState(SD_CCAST(c)->GetMotorState(static_cast<SixDOFConstraintSettings::EAxis>(a))); JOLTC_TRY_END return JOLTC_MOTOR_STATE_OFF; }
JOLTC_API void JoltC_SixDOFConstraint_SetTargetVelocityCS(JoltC_Constraint* c, JoltC_Vec3 v) { if (!c) return; JOLTC_TRY_BEGIN SD_CAST(c)->SetTargetVelocityCS(toJphVec3(v)); JOLTC_TRY_END }
JOLTC_API JoltC_Vec3 JoltC_SixDOFConstraint_GetTargetVelocityCS(const JoltC_Constraint* c) { JoltC_Vec3 z={0,0,0}; if (!c) return z; JOLTC_TRY_BEGIN return fromJphVec3(SD_CCAST(c)->GetTargetVelocityCS()); JOLTC_TRY_END return z; }
JOLTC_API void JoltC_SixDOFConstraint_SetTargetAngularVelocityCS(JoltC_Constraint* c, JoltC_Vec3 v) { if (!c) return; JOLTC_TRY_BEGIN SD_CAST(c)->SetTargetAngularVelocityCS(toJphVec3(v)); JOLTC_TRY_END }
JOLTC_API JoltC_Vec3 JoltC_SixDOFConstraint_GetTargetAngularVelocityCS(const JoltC_Constraint* c) { JoltC_Vec3 z={0,0,0}; if (!c) return z; JOLTC_TRY_BEGIN return fromJphVec3(SD_CCAST(c)->GetTargetAngularVelocityCS()); JOLTC_TRY_END return z; }
JOLTC_API void JoltC_SixDOFConstraint_SetTargetPositionCS(JoltC_Constraint* c, JoltC_Vec3 p) { if (!c) return; JOLTC_TRY_BEGIN SD_CAST(c)->SetTargetPositionCS(toJphVec3(p)); JOLTC_TRY_END }
JOLTC_API JoltC_Vec3 JoltC_SixDOFConstraint_GetTargetPositionCS(const JoltC_Constraint* c) { JoltC_Vec3 z={0,0,0}; if (!c) return z; JOLTC_TRY_BEGIN return fromJphVec3(SD_CCAST(c)->GetTargetPositionCS()); JOLTC_TRY_END return z; }
JOLTC_API void JoltC_SixDOFConstraint_SetTargetOrientationCS(JoltC_Constraint* c, JoltC_Quat o) { if (!c) return; JOLTC_TRY_BEGIN SD_CAST(c)->SetTargetOrientationCS(toJphQuat(o)); JOLTC_TRY_END }
JOLTC_API JoltC_Quat JoltC_SixDOFConstraint_GetTargetOrientationCS(const JoltC_Constraint* c) { JoltC_Quat z={0,0,0,1}; if (!c) return z; JOLTC_TRY_BEGIN return fromJphQuat(SD_CCAST(c)->GetTargetOrientationCS()); JOLTC_TRY_END return z; }
JOLTC_API void JoltC_SixDOFConstraint_SetMaxFriction(JoltC_Constraint* c, JoltC_SixDOFConstraintAxis a, float f) { if (!c) return; JOLTC_TRY_BEGIN SD_CAST(c)->SetMaxFriction(static_cast<SixDOFConstraintSettings::EAxis>(a), f); JOLTC_TRY_END }
JOLTC_API float JoltC_SixDOFConstraint_GetMaxFriction(const JoltC_Constraint* c, JoltC_SixDOFConstraintAxis a) { if (!c) return 0; JOLTC_TRY_BEGIN return SD_CCAST(c)->GetMaxFriction(static_cast<SixDOFConstraintSettings::EAxis>(a)); JOLTC_TRY_END return 0; }

#undef SD_CAST
#undef SD_CCAST

/* -------------------------------------------------------------------------- */
/*  PulleyConstraint                                                          */
/* -------------------------------------------------------------------------- */
static void fillPulley(PulleyConstraintSettings& settings, const JoltC_PulleyConstraintSettings* s)
{
    settings.mSpace = toJphConstraintSpace(s->space);
    settings.mBodyPoint1 = toJphRVec3(s->bodyPoint1);
    settings.mFixedPoint1 = toJphRVec3(s->fixedPoint1);
    settings.mBodyPoint2 = toJphRVec3(s->bodyPoint2);
    settings.mFixedPoint2 = toJphRVec3(s->fixedPoint2);
    settings.mRatio = s->ratio;
    settings.mMinLength = s->minLength;
    settings.mMaxLength = s->maxLength;
}

JOLTC_API JoltC_Constraint* JoltC_PulleyConstraint_Create(
    JoltC_PhysicsSystem* system, JoltC_BodyID b1, JoltC_BodyID b2,
    const JoltC_PulleyConstraintSettings* s)
{
    if (!system || !s) return nullptr;
    JOLTC_TRY_BEGIN
    PulleyConstraintSettings settings;
    fillPulley(settings, s);
    return createTwoBody(system, b1, b2, settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_PulleyConstraintSettings_Init(JoltC_PulleyConstraintSettings* s)
{
    if (!s) return;
    PulleyConstraintSettings d;
    s->space = static_cast<JoltC_ConstraintSpace>(d.mSpace);
    s->bodyPoint1 = fromJphRVec3(d.mBodyPoint1);
    s->fixedPoint1 = fromJphRVec3(d.mFixedPoint1);
    s->bodyPoint2 = fromJphRVec3(d.mBodyPoint2);
    s->fixedPoint2 = fromJphRVec3(d.mFixedPoint2);
    s->ratio = d.mRatio;
    s->minLength = d.mMinLength;
    s->maxLength = d.mMaxLength;
}

JOLTC_API JoltC_TwoBodyConstraintSettings* JoltC_PulleyConstraintSettings_CreateSettings(const JoltC_PulleyConstraintSettings* s)
{
    if (!s) return nullptr;
    JOLTC_TRY_BEGIN
    auto* settings = new PulleyConstraintSettings();
    fillPulley(*settings, s);
    return wrapTwoBodySettings(settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_PulleyConstraint_SetLength(JoltC_Constraint* c, float minLength, float maxLength)
{
    if (!c) return;
    JOLTC_TRY_BEGIN
    static_cast<PulleyConstraint*>(asJph(c))->SetLength(minLength, maxLength);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_PulleyConstraint_GetMinLength(const JoltC_Constraint* c)
{
    if (!c) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const PulleyConstraint*>(asJph(c))->GetMinLength();
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API float JoltC_PulleyConstraint_GetMaxLength(const JoltC_Constraint* c)
{
    if (!c) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const PulleyConstraint*>(asJph(c))->GetMaxLength();
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API float JoltC_PulleyConstraint_GetCurrentLength(const JoltC_Constraint* c)
{
    if (!c) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const PulleyConstraint*>(asJph(c))->GetCurrentLength();
    JOLTC_TRY_END
    return 0.0f;
}

/* -------------------------------------------------------------------------- */
/*  GearConstraint                                                            */
/* -------------------------------------------------------------------------- */
static void fillGear(GearConstraintSettings& settings, const JoltC_GearConstraintSettings* s)
{
    settings.mSpace = toJphConstraintSpace(s->space);
    settings.mHingeAxis1 = toJphVec3(s->hingeAxis1);
    settings.mHingeAxis2 = toJphVec3(s->hingeAxis2);
    settings.mRatio = s->ratio;
}

JOLTC_API JoltC_Constraint* JoltC_GearConstraint_Create(
    JoltC_PhysicsSystem* system, JoltC_BodyID b1, JoltC_BodyID b2,
    const JoltC_GearConstraintSettings* s)
{
    if (!system || !s) return nullptr;
    JOLTC_TRY_BEGIN
    GearConstraintSettings settings;
    fillGear(settings, s);
    return createTwoBody(system, b1, b2, settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_TwoBodyConstraintSettings* JoltC_GearConstraintSettings_CreateSettings(const JoltC_GearConstraintSettings* s)
{
    if (!s) return nullptr;
    JOLTC_TRY_BEGIN
    auto* settings = new GearConstraintSettings();
    fillGear(*settings, s);
    return wrapTwoBodySettings(settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_GearConstraint_SetConstraints(JoltC_Constraint* c, const JoltC_Constraint* g1, const JoltC_Constraint* g2) {
    if (!c || !g1 || !g2) return;
    JOLTC_TRY_BEGIN
    static_cast<GearConstraint*>(asJph(c))->SetConstraints(
        static_cast<const HingeConstraint*>(asJph(g1)),
        static_cast<const HingeConstraint*>(asJph(g2)));
    JOLTC_TRY_END
}

JOLTC_API float JoltC_GearConstraint_GetTotalLambda(const JoltC_Constraint* c) {
    if (!c) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const GearConstraint*>(asJph(c))->GetTotalLambda();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_GearConstraintSettings_SetRatio(JoltC_GearConstraintSettings* s, int numTeethGear1, int numTeethGear2)
{
    if (!s) return;
    /* Jolt's own arithmetic, so the two never drift apart. */
    GearConstraintSettings d;
    d.SetRatio(numTeethGear1, numTeethGear2);
    s->ratio = d.mRatio;
}

/* -------------------------------------------------------------------------- */
/*  RackAndPinionConstraint                                                   */
/* -------------------------------------------------------------------------- */
static void fillRackAndPinion(RackAndPinionConstraintSettings& settings, const JoltC_RackAndPinionConstraintSettings* s)
{
    settings.mSpace = toJphConstraintSpace(s->space);
    settings.mHingeAxis = toJphVec3(s->hingeAxis);
    settings.mSliderAxis = toJphVec3(s->sliderAxis);
    settings.mRatio = s->ratio;
}

JOLTC_API JoltC_Constraint* JoltC_RackAndPinionConstraint_Create(
    JoltC_PhysicsSystem* system, JoltC_BodyID b1, JoltC_BodyID b2,
    const JoltC_RackAndPinionConstraintSettings* s)
{
    if (!system || !s) return nullptr;
    JOLTC_TRY_BEGIN
    RackAndPinionConstraintSettings settings;
    fillRackAndPinion(settings, s);
    return createTwoBody(system, b1, b2, settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_RackAndPinionConstraintSettings_Init(JoltC_RackAndPinionConstraintSettings* s)
{
    if (!s) return;
    RackAndPinionConstraintSettings d;
    s->space = static_cast<JoltC_ConstraintSpace>(d.mSpace);
    s->hingeAxis = fromJphVec3(d.mHingeAxis);
    s->sliderAxis = fromJphVec3(d.mSliderAxis);
    s->ratio = d.mRatio;
}

JOLTC_API JoltC_TwoBodyConstraintSettings* JoltC_RackAndPinionConstraintSettings_CreateSettings(const JoltC_RackAndPinionConstraintSettings* s)
{
    if (!s) return nullptr;
    JOLTC_TRY_BEGIN
    auto* settings = new RackAndPinionConstraintSettings();
    fillRackAndPinion(*settings, s);
    return wrapTwoBodySettings(settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_RackAndPinionConstraintSettings_SetRatio(JoltC_RackAndPinionConstraintSettings* s, int numTeethRack, float rackLength, int numTeethPinion)
{
    if (!s) return;
    RackAndPinionConstraintSettings d;
    d.SetRatio(numTeethRack, rackLength, numTeethPinion);
    s->ratio = d.mRatio;
}

JOLTC_API void JoltC_RackAndPinionConstraint_SetConstraints(JoltC_Constraint* c, const JoltC_Constraint* pinion, const JoltC_Constraint* rack) {
    if (!c || !pinion || !rack) return;
    JOLTC_TRY_BEGIN
    static_cast<RackAndPinionConstraint*>(asJph(c))->SetConstraints(
        static_cast<const HingeConstraint*>(asJph(pinion)),
        static_cast<const SliderConstraint*>(asJph(rack)));
    JOLTC_TRY_END
}

JOLTC_API float JoltC_RackAndPinionConstraint_GetTotalLambda(const JoltC_Constraint* c) {
    if (!c) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const RackAndPinionConstraint*>(asJph(c))->GetTotalLambda();
    JOLTC_TRY_END
    return 0;
}

/* ========================================================================== */
/*  Constraint base - additional                                              */
/* ========================================================================== */
JOLTC_API void JoltC_Constraint_Destroy(JoltC_Constraint* c) {
    if (!c) return;
    c->ptr = nullptr;
    delete c;
}
JOLTC_API JoltC_ConstraintType JoltC_Constraint_GetType(const JoltC_Constraint* c) {
    if (!c) return JOLTC_CONSTRAINT_TYPE_CONSTRAINT;
    return static_cast<JoltC_ConstraintType>(asJph(c)->GetType());
}
JOLTC_API JoltC_Bool JoltC_Constraint_IsActive(const JoltC_Constraint* c) {
    if (!c) return JOLTC_FALSE;
    return asJph(c)->IsActive() ? JOLTC_TRUE : JOLTC_FALSE;
}
JOLTC_API uint32_t JoltC_Constraint_GetConstraintPriority(const JoltC_Constraint* c) {
    return c ? asJph(c)->GetConstraintPriority() : 0;
}
JOLTC_API void JoltC_Constraint_SetConstraintPriority(JoltC_Constraint* c, uint32_t p) {
    if (c) const_cast<Constraint*>(asJph(c))->SetConstraintPriority(p);
}
JOLTC_API uint32_t JoltC_Constraint_GetNumVelocityStepsOverride(const JoltC_Constraint* c) {
    return c ? asJph(c)->GetNumVelocityStepsOverride() : 0;
}
JOLTC_API void JoltC_Constraint_SetNumVelocityStepsOverride(JoltC_Constraint* c, uint32_t s) {
    if (c) const_cast<Constraint*>(asJph(c))->SetNumVelocityStepsOverride(s);
}
JOLTC_API uint32_t JoltC_Constraint_GetNumPositionStepsOverride(const JoltC_Constraint* c) {
    return c ? asJph(c)->GetNumPositionStepsOverride() : 0;
}
JOLTC_API void JoltC_Constraint_SetNumPositionStepsOverride(JoltC_Constraint* c, uint32_t s) {
    if (c) const_cast<Constraint*>(asJph(c))->SetNumPositionStepsOverride(s);
}
JOLTC_API void JoltC_Constraint_NotifyShapeChanged(JoltC_Constraint* c, JoltC_BodyID bodyID, JoltC_Vec3 deltaCOM) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    asJph(c)->NotifyShapeChanged(BodyID(bodyID), toJphVec3(deltaCOM));
    JOLTC_TRY_END
}
JOLTC_API void JoltC_Constraint_ResetWarmStart(JoltC_Constraint* c) {
    if (c) asJph(c)->ResetWarmStart();
}
JOLTC_API void JoltC_Constraint_SetupVelocityConstraint(JoltC_Constraint* c, float dt) {
    if (c) asJph(c)->SetupVelocityConstraint(dt);
}
JOLTC_API void JoltC_Constraint_WarmStartVelocityConstraint(JoltC_Constraint* c, float ratio) {
    if (c) asJph(c)->WarmStartVelocityConstraint(ratio);
}
JOLTC_API JoltC_Bool JoltC_Constraint_SolveVelocityConstraint(JoltC_Constraint* c, float dt) {
    if (!c) return JOLTC_FALSE;
    return asJph(c)->SolveVelocityConstraint(dt) ? JOLTC_TRUE : JOLTC_FALSE;
}
JOLTC_API JoltC_Bool JoltC_Constraint_SolvePositionConstraint(JoltC_Constraint* c, float dt, float b) {
    if (!c) return JOLTC_FALSE;
    return asJph(c)->SolvePositionConstraint(dt, b) ? JOLTC_TRUE : JOLTC_FALSE;
}

/* TwoBodyConstraint base */
JOLTC_API JoltC_Body* JoltC_TwoBodyConstraint_GetBody1(JoltC_Constraint* c) {
    if (!c) return nullptr;
    return reinterpret_cast<JoltC_Body*>(static_cast<TwoBodyConstraint*>(asJph(c))->GetBody1());
}
JOLTC_API JoltC_Body* JoltC_TwoBodyConstraint_GetBody2(JoltC_Constraint* c) {
    if (!c) return nullptr;
    return reinterpret_cast<JoltC_Body*>(static_cast<TwoBodyConstraint*>(asJph(c))->GetBody2());
}
JOLTC_API void JoltC_TwoBodyConstraint_GetConstraintToBody1Matrix(const JoltC_Constraint* c, JoltC_Mat44* r) {
    if (!c || !r) return;
    *r = fromJphMat44(static_cast<const TwoBodyConstraint*>(asJph(c))->GetConstraintToBody1Matrix());
}
JOLTC_API void JoltC_TwoBodyConstraint_GetConstraintToBody2Matrix(const JoltC_Constraint* c, JoltC_Mat44* r) {
    if (!c || !r) return;
    *r = fromJphMat44(static_cast<const TwoBodyConstraint*>(asJph(c))->GetConstraintToBody2Matrix());
}

/* ========================================================================== */
/*  Settings Init functions                                                   */
/* ========================================================================== */
JOLTC_API void JoltC_PointConstraintSettings_Init(JoltC_PointConstraintSettings* s) {
    if (!s) return;
    PointConstraintSettings d;
    s->space = static_cast<JoltC_ConstraintSpace>(d.mSpace);
    s->point1 = fromJphRVec3(d.mPoint1);
    s->point2 = fromJphRVec3(d.mPoint2);
}
JOLTC_API void JoltC_FixedConstraintSettings_Init(JoltC_FixedConstraintSettings* s) {
    if (!s) return;
    FixedConstraintSettings d;
    s->space = static_cast<JoltC_ConstraintSpace>(d.mSpace);
    s->autoDetectPoint = d.mAutoDetectPoint ? JOLTC_TRUE : JOLTC_FALSE;
    s->point1 = fromJphRVec3(d.mPoint1);
    s->axisX1 = fromJphVec3(d.mAxisX1);
    s->axisY1 = fromJphVec3(d.mAxisY1);
    s->point2 = fromJphRVec3(d.mPoint2);
    s->axisX2 = fromJphVec3(d.mAxisX2);
    s->axisY2 = fromJphVec3(d.mAxisY2);
}
JOLTC_API void JoltC_DistanceConstraintSettings_Init(JoltC_DistanceConstraintSettings* s) {
    if (!s) return;
    DistanceConstraintSettings d;
    s->space = static_cast<JoltC_ConstraintSpace>(d.mSpace);
    s->point1 = fromJphRVec3(d.mPoint1);
    s->point2 = fromJphRVec3(d.mPoint2);
    s->minDistance = d.mMinDistance;
    s->maxDistance = d.mMaxDistance;
    s->limitsSpringSettings = fromJphSpringSettings(d.mLimitsSpringSettings);
}
JOLTC_API void JoltC_HingeConstraintSettings_Init(JoltC_HingeConstraintSettings* s) {
    if (!s) return;
    HingeConstraintSettings d;
    s->space = static_cast<JoltC_ConstraintSpace>(d.mSpace);
    s->point1 = fromJphRVec3(d.mPoint1);
    s->hingeAxis1 = fromJphVec3(d.mHingeAxis1);
    s->normalAxis1 = fromJphVec3(d.mNormalAxis1);
    s->point2 = fromJphRVec3(d.mPoint2);
    s->hingeAxis2 = fromJphVec3(d.mHingeAxis2);
    s->normalAxis2 = fromJphVec3(d.mNormalAxis2);
    s->limitsMin = d.mLimitsMin;
    s->limitsMax = d.mLimitsMax;
    s->limitsSpringSettings = fromJphSpringSettings(d.mLimitsSpringSettings);
    s->maxFrictionTorque = d.mMaxFrictionTorque;
}
JOLTC_API void JoltC_SliderConstraintSettings_Init(JoltC_SliderConstraintSettings* s) {
    if (!s) return;
    SliderConstraintSettings d;
    s->space = static_cast<JoltC_ConstraintSpace>(d.mSpace);
    s->autoDetectPoint = d.mAutoDetectPoint ? JOLTC_TRUE : JOLTC_FALSE;
    s->point1 = fromJphRVec3(d.mPoint1);
    s->sliderAxis1 = fromJphVec3(d.mSliderAxis1);
    s->normalAxis1 = fromJphVec3(d.mNormalAxis1);
    s->point2 = fromJphRVec3(d.mPoint2);
    s->sliderAxis2 = fromJphVec3(d.mSliderAxis2);
    s->normalAxis2 = fromJphVec3(d.mNormalAxis2);
    s->limitsMin = d.mLimitsMin;
    s->limitsMax = d.mLimitsMax;
    s->limitsSpringSettings = fromJphSpringSettings(d.mLimitsSpringSettings);
    s->maxFrictionForce = d.mMaxFrictionForce;
}
JOLTC_API void JoltC_SliderConstraintSettings_SetSliderAxis(JoltC_SliderConstraintSettings* s, JoltC_Vec3 axis) {
    if (!s) return;
    SliderConstraintSettings d;
    d.mSpace = static_cast<EConstraintSpace>(s->space);
    d.SetSliderAxis(toJphVec3(axis));
    s->sliderAxis1 = fromJphVec3(d.mSliderAxis1);
    s->normalAxis1 = fromJphVec3(d.mNormalAxis1);
    s->sliderAxis2 = fromJphVec3(d.mSliderAxis2);
    s->normalAxis2 = fromJphVec3(d.mNormalAxis2);
}
JOLTC_API void JoltC_ConeConstraintSettings_Init(JoltC_ConeConstraintSettings* s) {
    if (!s) return;
    ConeConstraintSettings d;
    s->space = static_cast<JoltC_ConstraintSpace>(d.mSpace);
    s->point1 = fromJphRVec3(d.mPoint1);
    s->twistAxis1 = fromJphVec3(d.mTwistAxis1);
    s->point2 = fromJphRVec3(d.mPoint2);
    s->twistAxis2 = fromJphVec3(d.mTwistAxis2);
    s->halfConeAngle = d.mHalfConeAngle;
}
JOLTC_API void JoltC_SwingTwistConstraintSettings_Init(JoltC_SwingTwistConstraintSettings* s) {
    if (!s) return;
    SwingTwistConstraintSettings d;
    s->space = static_cast<JoltC_ConstraintSpace>(d.mSpace);
    s->position1 = fromJphRVec3(d.mPosition1);
    s->twistAxis1 = fromJphVec3(d.mTwistAxis1);
    s->planeAxis1 = fromJphVec3(d.mPlaneAxis1);
    s->position2 = fromJphRVec3(d.mPosition2);
    s->twistAxis2 = fromJphVec3(d.mTwistAxis2);
    s->planeAxis2 = fromJphVec3(d.mPlaneAxis2);
    s->normalHalfConeAngle = d.mNormalHalfConeAngle;
    s->planeHalfConeAngle = d.mPlaneHalfConeAngle;
    s->twistMinAngle = d.mTwistMinAngle;
    s->twistMaxAngle = d.mTwistMaxAngle;
    s->maxFrictionTorque = d.mMaxFrictionTorque;
}
JOLTC_API void JoltC_SixDOFConstraintSettings_Init(JoltC_SixDOFConstraintSettings* s) {
    if (!s) return;
    SixDOFConstraintSettings d;
    s->space = static_cast<JoltC_ConstraintSpace>(d.mSpace);
    s->position1 = fromJphRVec3(d.mPosition1);
    s->axisX1 = fromJphVec3(d.mAxisX1);
    s->axisY1 = fromJphVec3(d.mAxisY1);
    s->position2 = fromJphRVec3(d.mPosition2);
    s->axisX2 = fromJphVec3(d.mAxisX2);
    s->axisY2 = fromJphVec3(d.mAxisY2);
    for (int i = 0; i < 6; i++) s->maxFriction[i] = d.mMaxFriction[i];
    for (int i = 0; i < 6; i++) s->limitMin[i] = d.mLimitMin[i];
    for (int i = 0; i < 6; i++) s->limitMax[i] = d.mLimitMax[i];
}
JOLTC_API void JoltC_GearConstraintSettings_Init(JoltC_GearConstraintSettings* s) {
    if (!s) return;
    GearConstraintSettings d;
    s->space = static_cast<JoltC_ConstraintSpace>(d.mSpace);
    s->hingeAxis1 = fromJphVec3(d.mHingeAxis1);
    s->hingeAxis2 = fromJphVec3(d.mHingeAxis2);
    s->ratio = d.mRatio;
}

/* SixDOFConstraintSettings helpers */
JOLTC_API JoltC_Bool JoltC_SixDOFConstraintSettings_IsFreeAxis(const JoltC_SixDOFConstraintSettings* s, JoltC_SixDOFConstraintAxis axis) {
    if (!s) return JOLTC_FALSE;
    return (s->limitMin[axis] <= -FLT_MAX && s->limitMax[axis] >= FLT_MAX) ? JOLTC_TRUE : JOLTC_FALSE;
}
JOLTC_API JoltC_Bool JoltC_SixDOFConstraintSettings_IsFixedAxis(const JoltC_SixDOFConstraintSettings* s, JoltC_SixDOFConstraintAxis axis) {
    if (!s) return JOLTC_FALSE;
    return (s->limitMin[axis] == 0.0f && s->limitMax[axis] == 0.0f) ? JOLTC_TRUE : JOLTC_FALSE;
}
JOLTC_API void JoltC_SixDOFConstraintSettings_MakeFreeAxis(JoltC_SixDOFConstraintSettings* s, JoltC_SixDOFConstraintAxis axis) {
    if (!s) return;
    s->limitMin[axis] = -FLT_MAX;
    s->limitMax[axis] = FLT_MAX;
}
JOLTC_API void JoltC_SixDOFConstraintSettings_MakeFixedAxis(JoltC_SixDOFConstraintSettings* s, JoltC_SixDOFConstraintAxis axis) {
    if (!s) return;
    s->limitMin[axis] = 0.0f;
    s->limitMax[axis] = 0.0f;
}
JOLTC_API void JoltC_SixDOFConstraintSettings_SetLimitedAxis(JoltC_SixDOFConstraintSettings* s, JoltC_SixDOFConstraintAxis axis, float min, float max) {
    if (!s) return;
    s->limitMin[axis] = min;
    s->limitMax[axis] = max;
}

/* ========================================================================== */
/*  Constraint GetSettings                                                    */
/* ========================================================================== */
/* These used to be stubs returning NULL, under a comment claiming JPH constraints do not expose
 * their settings. That stopped being true: Constraint::GetConstraintSettings() is pure virtual on
 * the base and every constraint builds a fresh settings object from its live state. The stubs cost
 * more than nothing -- RagdollSettings_SetPartToParent takes a TwoBodyConstraintSettings*, and with
 * these returning NULL there was no way to produce one, so ragdolls could not be articulated from C
 * at all. The caller owns one reference and releases it with TwoBodyConstraintSettings_Release. */
static JoltC_TwoBodyConstraintSettings* getTwoBodySettings(const JoltC_Constraint* c)
{
    if (!c) return nullptr;
    JOLTC_TRY_BEGIN
    Ref<ConstraintSettings> settings = asJph(c)->GetConstraintSettings();
    if (settings == nullptr) return nullptr;

    /* Every constraint behind these entry points is a two body one, so the downcast holds. The
     * local Ref lets go on return; the AddRef inside the wrap is the caller's reference. */
    return wrapTwoBodySettings(static_cast<TwoBodyConstraintSettings*>(settings.GetPtr()));
    JOLTC_TRY_END
    return nullptr;
}

#define SETTINGS_GETTER(Name) \
    JOLTC_API JoltC_TwoBodyConstraintSettings* JoltC_ ## Name ## _GetSettings(const JoltC_Constraint* c) { return getTwoBodySettings(c); }

SETTINGS_GETTER(PointConstraint)
SETTINGS_GETTER(FixedConstraint)
SETTINGS_GETTER(DistanceConstraint)
SETTINGS_GETTER(HingeConstraint)
SETTINGS_GETTER(SliderConstraint)
SETTINGS_GETTER(ConeConstraint)
SETTINGS_GETTER(SwingTwistConstraint)
SETTINGS_GETTER(SixDOFConstraint)
SETTINGS_GETTER(GearConstraint)
SETTINGS_GETTER(PulleyConstraint)
SETTINGS_GETTER(RackAndPinionConstraint)
SETTINGS_GETTER(PathConstraint)
#undef SETTINGS_GETTER

JOLTC_API void JoltC_TwoBodyConstraintSettings_AddRef(JoltC_TwoBodyConstraintSettings* settings)
{
    if (!settings) return;
    reinterpret_cast<TwoBodyConstraintSettings*>(settings)->AddRef();
}

JOLTC_API void JoltC_TwoBodyConstraintSettings_Release(JoltC_TwoBodyConstraintSettings* settings)
{
    if (!settings) return;
    reinterpret_cast<TwoBodyConstraintSettings*>(settings)->Release();
}

/* ========================================================================== */
/*  Constraint-specific TotalLambda / Spring / Motor getters                  */
/* ========================================================================== */

/* PointConstraint */
JOLTC_API JoltC_Vec3 JoltC_PointConstraint_GetLocalSpacePoint1(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(static_cast<const PointConstraint*>(asJph(c))->GetLocalSpacePoint1());
}
JOLTC_API JoltC_Vec3 JoltC_PointConstraint_GetLocalSpacePoint2(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(static_cast<const PointConstraint*>(asJph(c))->GetLocalSpacePoint2());
}
JOLTC_API JoltC_Vec3 JoltC_PointConstraint_GetTotalLambdaPosition(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(static_cast<const PointConstraint*>(asJph(c))->GetTotalLambdaPosition());
}

/* FixedConstraint */
JOLTC_API JoltC_Vec3 JoltC_FixedConstraint_GetTotalLambdaPosition(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(static_cast<const FixedConstraint*>(asJph(c))->GetTotalLambdaPosition());
}
JOLTC_API JoltC_Vec3 JoltC_FixedConstraint_GetTotalLambdaRotation(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(static_cast<const FixedConstraint*>(asJph(c))->GetTotalLambdaRotation());
}

/* DistanceConstraint */
JOLTC_API JoltC_SpringSettings JoltC_DistanceConstraint_GetLimitsSpringSettings(const JoltC_Constraint* c) {
    if (!c) return JoltC_SpringSettings{};
    return fromJphSpringSettings(static_cast<const DistanceConstraint*>(asJph(c))->GetLimitsSpringSettings());
}
JOLTC_API void JoltC_DistanceConstraint_SetLimitsSpringSettings(JoltC_Constraint* c, JoltC_SpringSettings s) {
    if (!c) return;
    static_cast<DistanceConstraint*>(asJph(c))->SetLimitsSpringSettings(toJphSpringSettings(s));
}
JOLTC_API JoltC_Vec3 JoltC_DistanceConstraint_GetTotalLambdaPosition(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    float l = static_cast<const DistanceConstraint*>(asJph(c))->GetTotalLambdaPosition();
    return JoltC_Vec3{l,0,0};
}

/* HingeConstraint */
JOLTC_API JoltC_SpringSettings JoltC_HingeConstraint_GetLimitsSpringSettings(const JoltC_Constraint* c) {
    if (!c) return JoltC_SpringSettings{};
    return fromJphSpringSettings(static_cast<const HingeConstraint*>(asJph(c))->GetLimitsSpringSettings());
}
JOLTC_API void JoltC_HingeConstraint_SetLimitsSpringSettings(JoltC_Constraint* c, JoltC_SpringSettings s) {
    if (!c) return;
    static_cast<HingeConstraint*>(asJph(c))->SetLimitsSpringSettings(toJphSpringSettings(s));
}
JOLTC_API JoltC_MotorSettings JoltC_HingeConstraint_GetMotorSettings(const JoltC_Constraint* c) {
    if (!c) return JoltC_MotorSettings{};
    auto& ms = static_cast<const HingeConstraint*>(asJph(c))->GetMotorSettings();
    JoltC_MotorSettings r;
    r.springSettings = fromJphSpringSettings(ms.mSpringSettings);
    r.minForceLimit = ms.mMinForceLimit;
    r.maxForceLimit = ms.mMaxForceLimit;
    r.minTorqueLimit = ms.mMinTorqueLimit;
    r.maxTorqueLimit = ms.mMaxTorqueLimit;
    return r;
}
JOLTC_API void JoltC_HingeConstraint_SetMotorSettings(JoltC_Constraint* c, JoltC_MotorSettings s) {
    if (!c) return;
    MotorSettings ms;
    ms.mSpringSettings = toJphSpringSettings(s.springSettings);
    ms.mMinForceLimit = s.minForceLimit;
    ms.mMaxForceLimit = s.maxForceLimit;
    ms.mMinTorqueLimit = s.minTorqueLimit;
    ms.mMaxTorqueLimit = s.maxTorqueLimit;
    static_cast<HingeConstraint*>(asJph(c))->GetMotorSettings() = ms;
}
JOLTC_API JoltC_Vec3 JoltC_HingeConstraint_GetLocalSpacePoint1(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(static_cast<const HingeConstraint*>(asJph(c))->GetLocalSpacePoint1());
}
JOLTC_API JoltC_Vec3 JoltC_HingeConstraint_GetLocalSpacePoint2(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(static_cast<const HingeConstraint*>(asJph(c))->GetLocalSpacePoint2());
}
JOLTC_API JoltC_Vec3 JoltC_HingeConstraint_GetLocalSpaceHingeAxis1(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(static_cast<const HingeConstraint*>(asJph(c))->GetLocalSpaceHingeAxis1());
}
JOLTC_API JoltC_Vec3 JoltC_HingeConstraint_GetLocalSpaceHingeAxis2(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(static_cast<const HingeConstraint*>(asJph(c))->GetLocalSpaceHingeAxis2());
}
JOLTC_API JoltC_Vec3 JoltC_HingeConstraint_GetLocalSpaceNormalAxis1(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(static_cast<const HingeConstraint*>(asJph(c))->GetLocalSpaceNormalAxis1());
}
JOLTC_API JoltC_Vec3 JoltC_HingeConstraint_GetLocalSpaceNormalAxis2(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(static_cast<const HingeConstraint*>(asJph(c))->GetLocalSpaceNormalAxis2());
}
JOLTC_API JoltC_Vec3 JoltC_HingeConstraint_GetTotalLambdaPosition(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(static_cast<const HingeConstraint*>(asJph(c))->GetTotalLambdaPosition());
}
JOLTC_API JoltC_Vec2 JoltC_HingeConstraint_GetTotalLambdaRotation(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec2{0,0};
    auto v = static_cast<const HingeConstraint*>(asJph(c))->GetTotalLambdaRotation();
    return JoltC_Vec2{v.mF32[0],v.mF32[1]};
}
JOLTC_API float JoltC_HingeConstraint_GetTotalLambdaRotationLimits(const JoltC_Constraint* c) {
    if (!c) return 0;
    return static_cast<const HingeConstraint*>(asJph(c))->GetTotalLambdaRotationLimits();
}
JOLTC_API float JoltC_HingeConstraint_GetTotalLambdaMotor(const JoltC_Constraint* c) {
    if (!c) return 0;
    return static_cast<const HingeConstraint*>(asJph(c))->GetTotalLambdaMotor();
}

/* SliderConstraint */
JOLTC_API JoltC_SpringSettings JoltC_SliderConstraint_GetLimitsSpringSettings(const JoltC_Constraint* c) {
    if (!c) return JoltC_SpringSettings{};
    return fromJphSpringSettings(static_cast<const SliderConstraint*>(asJph(c))->GetLimitsSpringSettings());
}
JOLTC_API void JoltC_SliderConstraint_SetLimitsSpringSettings(JoltC_Constraint* c, JoltC_SpringSettings s) {
    if (!c) return;
    static_cast<SliderConstraint*>(asJph(c))->SetLimitsSpringSettings(toJphSpringSettings(s));
}
JOLTC_API JoltC_MotorSettings JoltC_SliderConstraint_GetMotorSettings(const JoltC_Constraint* c) {
    if (!c) return JoltC_MotorSettings{};
    auto& ms = static_cast<const SliderConstraint*>(asJph(c))->GetMotorSettings();
    JoltC_MotorSettings r;
    r.springSettings = fromJphSpringSettings(ms.mSpringSettings);
    r.minForceLimit = ms.mMinForceLimit;
    r.maxForceLimit = ms.mMaxForceLimit;
    r.minTorqueLimit = ms.mMinTorqueLimit;
    r.maxTorqueLimit = ms.mMaxTorqueLimit;
    return r;
}
JOLTC_API void JoltC_SliderConstraint_SetMotorSettings(JoltC_Constraint* c, JoltC_MotorSettings s) {
    if (!c) return;
    MotorSettings ms;
    ms.mSpringSettings = toJphSpringSettings(s.springSettings);
    ms.mMinForceLimit = s.minForceLimit;
    ms.mMaxForceLimit = s.maxForceLimit;
    ms.mMinTorqueLimit = s.minTorqueLimit;
    ms.mMaxTorqueLimit = s.maxTorqueLimit;
    static_cast<SliderConstraint*>(asJph(c))->GetMotorSettings() = ms;
}
JOLTC_API JoltC_Vec3 JoltC_SliderConstraint_GetTotalLambdaPosition(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    auto v = static_cast<const SliderConstraint*>(asJph(c))->GetTotalLambdaPosition();
    return JoltC_Vec3{v.mF32[0],v.mF32[1],0};
}
JOLTC_API float JoltC_SliderConstraint_GetTotalLambdaPositionLimits(const JoltC_Constraint* c) {
    if (!c) return 0;
    return static_cast<const SliderConstraint*>(asJph(c))->GetTotalLambdaPositionLimits();
}
JOLTC_API JoltC_Vec3 JoltC_SliderConstraint_GetTotalLambdaRotation(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(static_cast<const SliderConstraint*>(asJph(c))->GetTotalLambdaRotation());
}
JOLTC_API float JoltC_SliderConstraint_GetTotalLambdaMotor(const JoltC_Constraint* c) {
    if (!c) return 0;
    return static_cast<const SliderConstraint*>(asJph(c))->GetTotalLambdaMotor();
}

/* ConeConstraint */
JOLTC_API JoltC_Vec3 JoltC_ConeConstraint_GetTotalLambdaPosition(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(static_cast<const ConeConstraint*>(asJph(c))->GetTotalLambdaPosition());
}
JOLTC_API float JoltC_ConeConstraint_GetTotalLambdaRotation(const JoltC_Constraint* c) {
    if (!c) return 0;
    return static_cast<const ConeConstraint*>(asJph(c))->GetTotalLambdaRotation();
}

/* SwingTwistConstraint */
JOLTC_API JoltC_Vec3 JoltC_SwingTwistConstraint_GetTotalLambdaPosition(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(static_cast<const SwingTwistConstraint*>(asJph(c))->GetTotalLambdaPosition());
}
JOLTC_API float JoltC_SwingTwistConstraint_GetTotalLambdaTwist(const JoltC_Constraint* c) {
    if (!c) return 0;
    return static_cast<const SwingTwistConstraint*>(asJph(c))->GetTotalLambdaTwist();
}
JOLTC_API float JoltC_SwingTwistConstraint_GetTotalLambdaSwingY(const JoltC_Constraint* c) {
    if (!c) return 0;
    return static_cast<const SwingTwistConstraint*>(asJph(c))->GetTotalLambdaSwingY();
}
JOLTC_API float JoltC_SwingTwistConstraint_GetTotalLambdaSwingZ(const JoltC_Constraint* c) {
    if (!c) return 0;
    return static_cast<const SwingTwistConstraint*>(asJph(c))->GetTotalLambdaSwingZ();
}
JOLTC_API float JoltC_SwingTwistConstraint_GetTotalLambdaMotor(const JoltC_Constraint* c) {
    if (!c) return 0;
    auto v = static_cast<const SwingTwistConstraint*>(asJph(c))->GetTotalLambdaMotor();
    return v.Length();
}

/* SixDOFConstraint */
JOLTC_API JoltC_Vec3 JoltC_SixDOFConstraint_GetTranslationLimitsMin(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    auto* s = static_cast<const SixDOFConstraint*>(asJph(c));
    return JoltC_Vec3{s->GetLimitsMin((SixDOFConstraintSettings::EAxis)0),
                      s->GetLimitsMin((SixDOFConstraintSettings::EAxis)1),
                      s->GetLimitsMin((SixDOFConstraintSettings::EAxis)2)};
}
JOLTC_API JoltC_Vec3 JoltC_SixDOFConstraint_GetTranslationLimitsMax(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    auto* s = static_cast<const SixDOFConstraint*>(asJph(c));
    return JoltC_Vec3{s->GetLimitsMax((SixDOFConstraintSettings::EAxis)0),
                      s->GetLimitsMax((SixDOFConstraintSettings::EAxis)1),
                      s->GetLimitsMax((SixDOFConstraintSettings::EAxis)2)};
}
JOLTC_API JoltC_Vec3 JoltC_SixDOFConstraint_GetRotationLimitsMin(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    auto* s = static_cast<const SixDOFConstraint*>(asJph(c));
    return JoltC_Vec3{s->GetLimitsMin((SixDOFConstraintSettings::EAxis)3),
                      s->GetLimitsMin((SixDOFConstraintSettings::EAxis)4),
                      s->GetLimitsMin((SixDOFConstraintSettings::EAxis)5)};
}
JOLTC_API JoltC_Vec3 JoltC_SixDOFConstraint_GetRotationLimitsMax(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    auto* s = static_cast<const SixDOFConstraint*>(asJph(c));
    return JoltC_Vec3{s->GetLimitsMax((SixDOFConstraintSettings::EAxis)3),
                      s->GetLimitsMax((SixDOFConstraintSettings::EAxis)4),
                      s->GetLimitsMax((SixDOFConstraintSettings::EAxis)5)};
}
JOLTC_API JoltC_Bool JoltC_SixDOFConstraint_IsFreeAxis(const JoltC_Constraint* c, JoltC_SixDOFConstraintAxis axis) {
    if (!c) return JOLTC_FALSE;
    return static_cast<const SixDOFConstraint*>(asJph(c))->IsFreeAxis((SixDOFConstraintSettings::EAxis)axis) ? JOLTC_TRUE : JOLTC_FALSE;
}
JOLTC_API JoltC_Bool JoltC_SixDOFConstraint_IsFixedAxis(const JoltC_Constraint* c, JoltC_SixDOFConstraintAxis axis) {
    if (!c) return JOLTC_FALSE;
    return static_cast<const SixDOFConstraint*>(asJph(c))->IsFixedAxis((SixDOFConstraintSettings::EAxis)axis) ? JOLTC_TRUE : JOLTC_FALSE;
}
JOLTC_API JoltC_SpringSettings JoltC_SixDOFConstraint_GetLimitsSpringSettings(const JoltC_Constraint* c, JoltC_SixDOFConstraintAxis axis) {
    if (!c) return JoltC_SpringSettings{};
    return fromJphSpringSettings(static_cast<const SixDOFConstraint*>(asJph(c))->GetLimitsSpringSettings((SixDOFConstraintSettings::EAxis)axis));
}
JOLTC_API void JoltC_SixDOFConstraint_SetLimitsSpringSettings(JoltC_Constraint* c, JoltC_SixDOFConstraintAxis axis, JoltC_SpringSettings s) {
    if (!c) return;
    static_cast<SixDOFConstraint*>(asJph(c))->SetLimitsSpringSettings((SixDOFConstraintSettings::EAxis)axis, toJphSpringSettings(s));
}
JOLTC_API JoltC_MotorSettings JoltC_SixDOFConstraint_GetMotorSettings(const JoltC_Constraint* c, JoltC_SixDOFConstraintAxis axis) {
    if (!c) return JoltC_MotorSettings{};
    auto& ms = static_cast<const SixDOFConstraint*>(asJph(c))->GetMotorSettings((SixDOFConstraintSettings::EAxis)axis);
    JoltC_MotorSettings r;
    r.springSettings = fromJphSpringSettings(ms.mSpringSettings);
    r.minForceLimit = ms.mMinForceLimit;
    r.maxForceLimit = ms.mMaxForceLimit;
    r.minTorqueLimit = ms.mMinTorqueLimit;
    r.maxTorqueLimit = ms.mMaxTorqueLimit;
    return r;
}
JOLTC_API JoltC_Quat JoltC_SixDOFConstraint_GetRotationInConstraintSpace(const JoltC_Constraint* c) {
    if (!c) return JoltC_Quat{0,0,0,1};
    return fromJphQuat(static_cast<const SixDOFConstraint*>(asJph(c))->GetRotationInConstraintSpace());
}
JOLTC_API void JoltC_SixDOFConstraint_SetTargetOrientationBS(JoltC_Constraint* c, JoltC_Quat o) {
    if (!c) return;
    static_cast<SixDOFConstraint*>(asJph(c))->SetTargetOrientationBS(toJphQuat(o));
}
JOLTC_API JoltC_Vec3 JoltC_SixDOFConstraint_GetTotalLambdaPosition(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(static_cast<const SixDOFConstraint*>(asJph(c))->GetTotalLambdaPosition());
}
JOLTC_API JoltC_Vec3 JoltC_SixDOFConstraint_GetTotalLambdaRotation(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(static_cast<const SixDOFConstraint*>(asJph(c))->GetTotalLambdaRotation());
}
JOLTC_API JoltC_Vec3 JoltC_SixDOFConstraint_GetTotalLambdaMotorTranslation(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(static_cast<const SixDOFConstraint*>(asJph(c))->GetTotalLambdaMotorTranslation());
}
JOLTC_API JoltC_Vec3 JoltC_SixDOFConstraint_GetTotalLambdaMotorRotation(const JoltC_Constraint* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(static_cast<const SixDOFConstraint*>(asJph(c))->GetTotalLambdaMotorRotation());
}

/* ========================================================================== */
/*  PathConstraintPath - ref-counted raw handle, like shapes                  */
/* ========================================================================== */
/* Two names rather than an overload: this section lives inside the extern "C" block, where
 * overloading is not a thing. */
static inline PathConstraintPath* asPathM(JoltC_PathConstraintPath* p) { return reinterpret_cast<PathConstraintPath*>(p); }
static inline const PathConstraintPath* asPathC(const JoltC_PathConstraintPath* p) { return reinterpret_cast<const PathConstraintPath*>(p); }

JOLTC_API JoltC_PathConstraintPath* JoltC_PathConstraintPathHermite_Create(void)
{
    JOLTC_TRY_BEGIN
    auto* path = new PathConstraintPathHermite();
    path->AddRef();
    return reinterpret_cast<JoltC_PathConstraintPath*>(static_cast<PathConstraintPath*>(path));
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_PathConstraintPathHermite_AddPoint(JoltC_PathConstraintPath* path, JoltC_Vec3 position, JoltC_Vec3 tangent, JoltC_Vec3 normal)
{
    if (!path) return;
    JOLTC_TRY_BEGIN
    static_cast<PathConstraintPathHermite*>(asPathM(path))->AddPoint(toJphVec3(position), toJphVec3(tangent), toJphVec3(normal));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_PathConstraintPath_SetIsLooping(JoltC_PathConstraintPath* path, JoltC_Bool looping)
{
    if (!path) return;
    asPathM(path)->SetIsLooping(looping != 0);
}

JOLTC_API JoltC_Bool JoltC_PathConstraintPath_IsLooping(const JoltC_PathConstraintPath* path)
{
    if (!path) return JOLTC_FALSE;
    return asPathC(path)->IsLooping() ? JOLTC_TRUE : JOLTC_FALSE;
}

JOLTC_API float JoltC_PathConstraintPath_GetPathMaxFraction(const JoltC_PathConstraintPath* path)
{
    if (!path) return 0.0f;
    JOLTC_TRY_BEGIN
    return asPathC(path)->GetPathMaxFraction();
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API float JoltC_PathConstraintPath_GetClosestPoint(const JoltC_PathConstraintPath* path, JoltC_Vec3 position, float fractionHint)
{
    if (!path) return 0.0f;
    JOLTC_TRY_BEGIN
    return asPathC(path)->GetClosestPoint(toJphVec3(position), fractionHint);
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API void JoltC_PathConstraintPath_GetPointOnPath(const JoltC_PathConstraintPath* path, float fraction, JoltC_Vec3* outPosition, JoltC_Vec3* outTangent, JoltC_Vec3* outNormal, JoltC_Vec3* outBinormal)
{
    if (!path) return;
    JOLTC_TRY_BEGIN
    Vec3 position, tangent, normal, binormal;
    asPathC(path)->GetPointOnPath(fraction, position, tangent, normal, binormal);
    if (outPosition) *outPosition = fromJphVec3(position);
    if (outTangent) *outTangent = fromJphVec3(tangent);
    if (outNormal) *outNormal = fromJphVec3(normal);
    if (outBinormal) *outBinormal = fromJphVec3(binormal);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_PathConstraintPath_AddRef(const JoltC_PathConstraintPath* path)
{
    if (!path) return;
    asPathC(path)->AddRef();
}

JOLTC_API void JoltC_PathConstraintPath_Release(const JoltC_PathConstraintPath* path)
{
    if (!path) return;
    asPathC(path)->Release();
}

/* ========================================================================== */
/*  PathConstraint                                                            */
/* ========================================================================== */
static void fillPath(PathConstraintSettings& settings, const JoltC_PathConstraintSettings* s)
{
    settings.mPath = asPathC(s->path);
    settings.mPathPosition = toJphVec3(s->pathPosition);
    settings.mPathRotation = toJphQuat(s->pathRotation);
    settings.mPathFraction = s->pathFraction;
    settings.mMaxFrictionForce = s->maxFrictionForce;
    settings.mPositionMotorSettings = toJphMotorSettings(s->positionMotorSettings);
    settings.mRotationConstraintType = static_cast<EPathRotationConstraintType>(s->rotationConstraintType);
}

JOLTC_API void JoltC_PathConstraintSettings_Init(JoltC_PathConstraintSettings* s)
{
    if (!s) return;
    PathConstraintSettings d;
    s->path = nullptr;
    s->pathPosition = fromJphVec3(d.mPathPosition);
    s->pathRotation = fromJphQuat(d.mPathRotation);
    s->pathFraction = d.mPathFraction;
    s->maxFrictionForce = d.mMaxFrictionForce;
    s->positionMotorSettings.springSettings = fromJphSpringSettings(d.mPositionMotorSettings.mSpringSettings);
    s->positionMotorSettings.minForceLimit = d.mPositionMotorSettings.mMinForceLimit;
    s->positionMotorSettings.maxForceLimit = d.mPositionMotorSettings.mMaxForceLimit;
    s->positionMotorSettings.minTorqueLimit = d.mPositionMotorSettings.mMinTorqueLimit;
    s->positionMotorSettings.maxTorqueLimit = d.mPositionMotorSettings.mMaxTorqueLimit;
    s->rotationConstraintType = static_cast<JoltC_PathRotationConstraintType>(d.mRotationConstraintType);
}

JOLTC_API JoltC_Constraint* JoltC_PathConstraint_Create(
    JoltC_PhysicsSystem* system, JoltC_BodyID b1, JoltC_BodyID b2,
    const JoltC_PathConstraintSettings* s)
{
    if (!system || !s || !s->path) return nullptr;
    JOLTC_TRY_BEGIN
    PathConstraintSettings settings;
    fillPath(settings, s);
    return createTwoBody(system, b1, b2, settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_TwoBodyConstraintSettings* JoltC_PathConstraintSettings_CreateSettings(const JoltC_PathConstraintSettings* s)
{
    if (!s) return nullptr;
    JOLTC_TRY_BEGIN
    auto* settings = new PathConstraintSettings();
    fillPath(*settings, s);
    return wrapTwoBodySettings(settings);
    JOLTC_TRY_END
    return nullptr;
}

#define PC_CAST(c)  static_cast<PathConstraint*>(asJph(c))
#define PC_CCAST(c) static_cast<const PathConstraint*>(asJph(c))

JOLTC_API void JoltC_PathConstraint_SetPath(JoltC_Constraint* c, const JoltC_PathConstraintPath* path, float pathFraction)
{
    if (!c) return;
    JOLTC_TRY_BEGIN
    PC_CAST(c)->SetPath(asPathC(path), pathFraction);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_PathConstraint_GetPathFraction(const JoltC_Constraint* c)
{
    if (!c) return 0.0f;
    JOLTC_TRY_BEGIN
    return PC_CCAST(c)->GetPathFraction();
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API void JoltC_PathConstraint_SetMaxFrictionForce(JoltC_Constraint* c, float force)
{
    if (!c) return;
    JOLTC_TRY_BEGIN
    PC_CAST(c)->SetMaxFrictionForce(force);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_PathConstraint_GetMaxFrictionForce(const JoltC_Constraint* c)
{
    if (!c) return 0.0f;
    JOLTC_TRY_BEGIN
    return PC_CCAST(c)->GetMaxFrictionForce();
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API void JoltC_PathConstraint_SetPositionMotorState(JoltC_Constraint* c, JoltC_MotorState state)
{
    if (!c) return;
    JOLTC_TRY_BEGIN
    PC_CAST(c)->SetPositionMotorState(toJphMotorState(state));
    JOLTC_TRY_END
}

JOLTC_API JoltC_MotorState JoltC_PathConstraint_GetPositionMotorState(const JoltC_Constraint* c)
{
    if (!c) return JOLTC_MOTOR_STATE_OFF;
    JOLTC_TRY_BEGIN
    return fromJphMotorState(PC_CCAST(c)->GetPositionMotorState());
    JOLTC_TRY_END
    return JOLTC_MOTOR_STATE_OFF;
}

JOLTC_API void JoltC_PathConstraint_SetTargetVelocity(JoltC_Constraint* c, float velocity)
{
    if (!c) return;
    JOLTC_TRY_BEGIN
    PC_CAST(c)->SetTargetVelocity(velocity);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_PathConstraint_GetTargetVelocity(const JoltC_Constraint* c)
{
    if (!c) return 0.0f;
    JOLTC_TRY_BEGIN
    return PC_CCAST(c)->GetTargetVelocity();
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API void JoltC_PathConstraint_SetTargetPathFraction(JoltC_Constraint* c, float fraction)
{
    if (!c) return;
    JOLTC_TRY_BEGIN
    PC_CAST(c)->SetTargetPathFraction(fraction);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_PathConstraint_GetTargetPathFraction(const JoltC_Constraint* c)
{
    if (!c) return 0.0f;
    JOLTC_TRY_BEGIN
    return PC_CCAST(c)->GetTargetPathFraction();
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API void JoltC_PathConstraint_SetPositionMotorSettings(JoltC_Constraint* c, const JoltC_MotorSettings* settings)
{
    if (!c || !settings) return;
    JOLTC_TRY_BEGIN
    PC_CAST(c)->GetPositionMotorSettings() = toJphMotorSettings(*settings);
    JOLTC_TRY_END
}

#undef PC_CAST
#undef PC_CCAST

} /* extern "C" */
