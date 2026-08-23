/* JoltC - Vehicle system implementations
 * SPDX-License-Identifier: MIT
 */

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/Physics/Vehicle/MotorcycleController.h>
#include <Jolt/Physics/Vehicle/TrackedVehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleEngine.h>
#include <Jolt/Physics/Vehicle/VehicleTransmission.h>
#include <Jolt/Physics/Vehicle/VehicleDifferential.h>
#include <Jolt/Physics/Vehicle/VehicleAntiRollBar.h>
#include <Jolt/Physics/Vehicle/VehicleTrack.h>
#include <Jolt/Physics/Vehicle/Wheel.h>
#include <Jolt/Core/LinearCurve.h>

#include <JoltC/vehicle.h>
#include "wrappers.h"
#include "internal.h"
#include "errors_internal.h"

using namespace JPH;

/* -------------------------------------------------------------------------- */
/*  Helpers                                                                   */
/* -------------------------------------------------------------------------- */
static inline WheelSettings* asWS(JoltC_WheelSettings* h)         { return reinterpret_cast<WheelSettings*>(h); }
static inline const WheelSettings* asWS(const JoltC_WheelSettings* h) { return reinterpret_cast<const WheelSettings*>(h); }
static inline JoltC_WheelSettings* fromWS(WheelSettings* p)       { return reinterpret_cast<JoltC_WheelSettings*>(p); }

static inline WheelSettingsWV* asWSWV(JoltC_WheelSettingsWV* h)         { return reinterpret_cast<WheelSettingsWV*>(h); }
static inline const WheelSettingsWV* asWSWV(const JoltC_WheelSettingsWV* h) { return reinterpret_cast<const WheelSettingsWV*>(h); }

static inline WheelSettingsTV* asWSTV(JoltC_WheelSettingsTV* h)         { return reinterpret_cast<WheelSettingsTV*>(h); }
static inline const WheelSettingsTV* asWSTV(const JoltC_WheelSettingsTV* h) { return reinterpret_cast<const WheelSettingsTV*>(h); }

static inline Wheel* asWheel(JoltC_Wheel* h)                     { return reinterpret_cast<Wheel*>(h); }
static inline const Wheel* asWheel(const JoltC_Wheel* h)         { return reinterpret_cast<const Wheel*>(h); }
static inline JoltC_Wheel* fromWheel(Wheel* p)                   { return reinterpret_cast<JoltC_Wheel*>(p); }

static inline WheelWV* asWheelWV(JoltC_WheelWV* h)               { return reinterpret_cast<WheelWV*>(h); }
static inline const WheelWV* asWheelWV(const JoltC_WheelWV* h)   { return reinterpret_cast<const WheelWV*>(h); }

static inline WheelTV* asWheelTV(JoltC_WheelTV* h)               { return reinterpret_cast<WheelTV*>(h); }
static inline const WheelTV* asWheelTV(const JoltC_WheelTV* h)   { return reinterpret_cast<const WheelTV*>(h); }

static inline VehicleConstraint* asVC(JoltC_VehicleConstraint* h)       { return reinterpret_cast<VehicleConstraint*>(h); }
static inline const VehicleConstraint* asVC(const JoltC_VehicleConstraint* h) { return reinterpret_cast<const VehicleConstraint*>(h); }

static inline VehicleCollisionTester* asVCT(JoltC_VehicleCollisionTester* h)       { return reinterpret_cast<VehicleCollisionTester*>(h); }
static inline const VehicleCollisionTester* asVCT(const JoltC_VehicleCollisionTester* h) { return reinterpret_cast<const VehicleCollisionTester*>(h); }
static inline JoltC_VehicleCollisionTester* fromVCT(VehicleCollisionTester* p)     { return reinterpret_cast<JoltC_VehicleCollisionTester*>(p); }

static inline VehicleControllerSettings* asVCS(JoltC_VehicleControllerSettings* h) { return reinterpret_cast<VehicleControllerSettings*>(h); }
static inline WheeledVehicleControllerSettings* asWVCS(JoltC_WheeledVehicleControllerSettings* h) { return reinterpret_cast<WheeledVehicleControllerSettings*>(h); }
static inline const WheeledVehicleControllerSettings* asWVCS(const JoltC_WheeledVehicleControllerSettings* h) { return reinterpret_cast<const WheeledVehicleControllerSettings*>(h); }
static inline MotorcycleControllerSettings* asMCS(JoltC_MotorcycleControllerSettings* h) { return reinterpret_cast<MotorcycleControllerSettings*>(h); }
static inline const MotorcycleControllerSettings* asMCS(const JoltC_MotorcycleControllerSettings* h) { return reinterpret_cast<const MotorcycleControllerSettings*>(h); }
static inline TrackedVehicleControllerSettings* asTVCS(JoltC_TrackedVehicleControllerSettings* h) { return reinterpret_cast<TrackedVehicleControllerSettings*>(h); }
static inline const TrackedVehicleControllerSettings* asTVCS(const JoltC_TrackedVehicleControllerSettings* h) { return reinterpret_cast<const TrackedVehicleControllerSettings*>(h); }

static inline WheeledVehicleController* asWVC(JoltC_WheeledVehicleController* h) { return reinterpret_cast<WheeledVehicleController*>(h); }
static inline const WheeledVehicleController* asWVC(const JoltC_WheeledVehicleController* h) { return reinterpret_cast<const WheeledVehicleController*>(h); }
static inline MotorcycleController* asMC(JoltC_MotorcycleController* h) { return reinterpret_cast<MotorcycleController*>(h); }
static inline const MotorcycleController* asMC(const JoltC_MotorcycleController* h) { return reinterpret_cast<const MotorcycleController*>(h); }
static inline TrackedVehicleController* asTVC(JoltC_TrackedVehicleController* h) { return reinterpret_cast<TrackedVehicleController*>(h); }
static inline const TrackedVehicleController* asTVC(const JoltC_TrackedVehicleController* h) { return reinterpret_cast<const TrackedVehicleController*>(h); }

static inline VehicleEngine* asVE(JoltC_VehicleEngine* h)             { return reinterpret_cast<VehicleEngine*>(h); }
static inline const VehicleEngine* asVE(const JoltC_VehicleEngine* h) { return reinterpret_cast<const VehicleEngine*>(h); }
static inline VehicleTransmission* asVT(JoltC_VehicleTransmission* h)       { return reinterpret_cast<VehicleTransmission*>(h); }
static inline const VehicleTransmission* asVT(const JoltC_VehicleTransmission* h) { return reinterpret_cast<const VehicleTransmission*>(h); }
static inline const VehicleTrack* asVTr(const JoltC_VehicleTrack* h)  { return reinterpret_cast<const VehicleTrack*>(h); }
static inline VehicleTrack* asVTr(JoltC_VehicleTrack* h)              { return reinterpret_cast<VehicleTrack*>(h); }

/* Engine settings conversion helpers */
static VehicleEngineSettings toJphEngineSettings(const JoltC_VehicleEngineSettings& s) {
    VehicleEngineSettings out;
    out.mMaxTorque = s.maxTorque;
    out.mMinRPM = s.minRPM;
    out.mMaxRPM = s.maxRPM;
    if (s.normalizedTorque)
        out.mNormalizedTorque = reinterpret_cast<const JoltC_LinearCurve*>(s.normalizedTorque)->curve;
    out.mInertia = s.inertia;
    out.mAngularDamping = s.angularDamping;
    return out;
}
static void fromJphEngineSettings(const VehicleEngineSettings& src, JoltC_VehicleEngineSettings* dst) {
    dst->maxTorque = src.mMaxTorque;
    dst->minRPM = src.mMinRPM;
    dst->maxRPM = src.mMaxRPM;
    dst->normalizedTorque = nullptr; /* curve is embedded, user must manage their own LinearCurve */
    dst->inertia = src.mInertia;
    dst->angularDamping = src.mAngularDamping;
}
static VehicleDifferentialSettings toJphDiffSettings(const JoltC_VehicleDifferentialSettings& s) {
    VehicleDifferentialSettings out;
    out.mLeftWheel = s.leftWheel;
    out.mRightWheel = s.rightWheel;
    out.mDifferentialRatio = s.differentialRatio;
    out.mLeftRightSplit = s.leftRightSplit;
    out.mLimitedSlipRatio = s.limitedSlipRatio;
    out.mEngineTorqueRatio = s.engineTorqueRatio;
    return out;
}
static void fromJphDiffSettings(const VehicleDifferentialSettings& src, JoltC_VehicleDifferentialSettings* dst) {
    dst->leftWheel = src.mLeftWheel;
    dst->rightWheel = src.mRightWheel;
    dst->differentialRatio = src.mDifferentialRatio;
    dst->leftRightSplit = src.mLeftRightSplit;
    dst->limitedSlipRatio = src.mLimitedSlipRatio;
    dst->engineTorqueRatio = src.mEngineTorqueRatio;
}

extern "C" {

/* ========================================================================== */
/*  LinearCurve                                                               */
/* ========================================================================== */
JOLTC_API JoltC_LinearCurve* JoltC_LinearCurve_Create(void) {
    return new JoltC_LinearCurve;
}
JOLTC_API void JoltC_LinearCurve_Destroy(JoltC_LinearCurve* c) { delete c; }
JOLTC_API void JoltC_LinearCurve_Clear(JoltC_LinearCurve* c) { if (c) c->curve.Clear(); }
JOLTC_API void JoltC_LinearCurve_Reserve(JoltC_LinearCurve* c, uint32_t n) { if (c) c->curve.Reserve(n); }
JOLTC_API void JoltC_LinearCurve_AddPoint(JoltC_LinearCurve* c, float x, float y) { if (c) c->curve.AddPoint(x, y); }
JOLTC_API void JoltC_LinearCurve_Sort(JoltC_LinearCurve* c) { if (c) c->curve.Sort(); }
JOLTC_API float JoltC_LinearCurve_GetMinX(const JoltC_LinearCurve* c) { return c ? c->curve.GetMinX() : 0; }
JOLTC_API float JoltC_LinearCurve_GetMaxX(const JoltC_LinearCurve* c) { return c ? c->curve.GetMaxX() : 0; }
JOLTC_API float JoltC_LinearCurve_GetValue(const JoltC_LinearCurve* c, float x) { return c ? c->curve.GetValue(x) : 0; }
JOLTC_API uint32_t JoltC_LinearCurve_GetPointCount(const JoltC_LinearCurve* c) {
    return c ? (uint32_t)c->curve.mPoints.size() : 0;
}
JOLTC_API void JoltC_LinearCurve_GetPoint(const JoltC_LinearCurve* c, uint32_t index, JoltC_Point* result) {
    if (!c || !result || index >= c->curve.mPoints.size()) return;
    result->x = c->curve.mPoints[index].mX;
    result->y = c->curve.mPoints[index].mY;
}
JOLTC_API void JoltC_LinearCurve_GetPoints(const JoltC_LinearCurve* c, JoltC_Point* points, uint32_t* count) {
    if (!c || !count) return;
    uint32_t n = (uint32_t)c->curve.mPoints.size();
    if (points) {
        uint32_t toCopy = n < *count ? n : *count;
        for (uint32_t i = 0; i < toCopy; i++) {
            points[i].x = c->curve.mPoints[i].mX;
            points[i].y = c->curve.mPoints[i].mY;
        }
    }
    *count = n;
}

/* ========================================================================== */
/*  WheelSettings (base)                                                      */
/* ========================================================================== */
JOLTC_API JoltC_WheelSettings* JoltC_WheelSettings_Create(void) {
    JOLTC_TRY_BEGIN
    auto* s = new WheelSettings;
    s->AddRef();
    return fromWS(s);
    JOLTC_TRY_END
    return nullptr;
}
JOLTC_API void JoltC_WheelSettings_Destroy(JoltC_WheelSettings* s) {
    if (s) asWS(s)->Release();
}
#define WS_VEC3_PROP(Name, Field) \
JOLTC_API JoltC_Vec3 JoltC_WheelSettings_Get##Name(const JoltC_WheelSettings* s) { \
    if (!s) return JoltC_Vec3{0,0,0}; return fromJphVec3(asWS(s)->Field); } \
JOLTC_API void JoltC_WheelSettings_Set##Name(JoltC_WheelSettings* s, JoltC_Vec3 v) { \
    if (s) asWS(s)->Field = toJphVec3(v); }

WS_VEC3_PROP(Position, mPosition)
WS_VEC3_PROP(SuspensionForcePoint, mSuspensionForcePoint)
WS_VEC3_PROP(SuspensionDirection, mSuspensionDirection)
WS_VEC3_PROP(SteeringAxis, mSteeringAxis)
WS_VEC3_PROP(WheelUp, mWheelUp)
WS_VEC3_PROP(WheelForward, mWheelForward)
#undef WS_VEC3_PROP

#define WS_FLOAT_PROP(Name, Field) \
JOLTC_API float JoltC_WheelSettings_Get##Name(const JoltC_WheelSettings* s) { return s ? asWS(s)->Field : 0; } \
JOLTC_API void JoltC_WheelSettings_Set##Name(JoltC_WheelSettings* s, float v) { if (s) asWS(s)->Field = v; }

WS_FLOAT_PROP(SuspensionMinLength, mSuspensionMinLength)
WS_FLOAT_PROP(SuspensionMaxLength, mSuspensionMaxLength)
WS_FLOAT_PROP(SuspensionPreloadLength, mSuspensionPreloadLength)
WS_FLOAT_PROP(Radius, mRadius)
WS_FLOAT_PROP(Width, mWidth)
#undef WS_FLOAT_PROP

JOLTC_API JoltC_SpringSettings JoltC_WheelSettings_GetSuspensionSpring(const JoltC_WheelSettings* s) {
    if (!s) return JoltC_SpringSettings{JOLTC_SPRING_MODE_FREQUENCY_AND_DAMPING, 0, 0};
    const auto& sp = asWS(s)->mSuspensionSpring;
    return JoltC_SpringSettings{(JoltC_SpringMode)sp.mMode, sp.mFrequency, sp.mDamping};
}
JOLTC_API void JoltC_WheelSettings_SetSuspensionSpring(JoltC_WheelSettings* s, JoltC_SpringSettings v) {
    if (!s) return;
    asWS(s)->mSuspensionSpring = SpringSettings((ESpringMode)v.mode, v.frequencyOrStiffness, v.damping);
}
JOLTC_API JoltC_Bool JoltC_WheelSettings_GetEnableSuspensionForcePoint(const JoltC_WheelSettings* s) {
    return s && asWS(s)->mEnableSuspensionForcePoint ? JOLTC_TRUE : JOLTC_FALSE;
}
JOLTC_API void JoltC_WheelSettings_SetEnableSuspensionForcePoint(JoltC_WheelSettings* s, JoltC_Bool v) {
    if (s) asWS(s)->mEnableSuspensionForcePoint = (v != JOLTC_FALSE);
}

/* ========================================================================== */
/*  WheelSettingsWV                                                           */
/* ========================================================================== */
JOLTC_API JoltC_WheelSettingsWV* JoltC_WheelSettingsWV_Create(void) {
    JOLTC_TRY_BEGIN
    auto* s = new WheelSettingsWV;
    s->AddRef();
    return reinterpret_cast<JoltC_WheelSettingsWV*>(s);
    JOLTC_TRY_END
    return nullptr;
}
#define WSWV_FLOAT(Name, Field) \
JOLTC_API float JoltC_WheelSettingsWV_Get##Name(const JoltC_WheelSettingsWV* s) { return s ? asWSWV(s)->Field : 0; } \
JOLTC_API void JoltC_WheelSettingsWV_Set##Name(JoltC_WheelSettingsWV* s, float v) { if (s) asWSWV(s)->Field = v; }
WSWV_FLOAT(Inertia, mInertia)
WSWV_FLOAT(AngularDamping, mAngularDamping)
WSWV_FLOAT(MaxSteerAngle, mMaxSteerAngle)
WSWV_FLOAT(MaxBrakeTorque, mMaxBrakeTorque)
WSWV_FLOAT(MaxHandBrakeTorque, mMaxHandBrakeTorque)
#undef WSWV_FLOAT

/* Friction curves: return thread-local wrapper pointing at internal curve */
static thread_local JoltC_LinearCurve tl_longFriction, tl_latFriction;
JOLTC_API const JoltC_LinearCurve* JoltC_WheelSettingsWV_GetLongitudinalFriction(const JoltC_WheelSettingsWV* s) {
    if (!s) return nullptr;
    tl_longFriction.curve = asWSWV(s)->mLongitudinalFriction;
    return &tl_longFriction;
}
JOLTC_API void JoltC_WheelSettingsWV_SetLongitudinalFriction(JoltC_WheelSettingsWV* s, const JoltC_LinearCurve* c) {
    if (s && c) asWSWV(s)->mLongitudinalFriction = c->curve;
}
JOLTC_API const JoltC_LinearCurve* JoltC_WheelSettingsWV_GetLateralFriction(const JoltC_WheelSettingsWV* s) {
    if (!s) return nullptr;
    tl_latFriction.curve = asWSWV(s)->mLateralFriction;
    return &tl_latFriction;
}
JOLTC_API void JoltC_WheelSettingsWV_SetLateralFriction(JoltC_WheelSettingsWV* s, const JoltC_LinearCurve* c) {
    if (s && c) asWSWV(s)->mLateralFriction = c->curve;
}

/* ========================================================================== */
/*  WheelSettingsTV                                                           */
/* ========================================================================== */
JOLTC_API JoltC_WheelSettingsTV* JoltC_WheelSettingsTV_Create(void) {
    JOLTC_TRY_BEGIN
    auto* s = new WheelSettingsTV;
    s->AddRef();
    return reinterpret_cast<JoltC_WheelSettingsTV*>(s);
    JOLTC_TRY_END
    return nullptr;
}
JOLTC_API float JoltC_WheelSettingsTV_GetLongitudinalFriction(const JoltC_WheelSettingsTV* s) { return s ? asWSTV(s)->mLongitudinalFriction : 0; }
JOLTC_API void  JoltC_WheelSettingsTV_SetLongitudinalFriction(JoltC_WheelSettingsTV* s, float v) { if (s) asWSTV(s)->mLongitudinalFriction = v; }
JOLTC_API float JoltC_WheelSettingsTV_GetLateralFriction(const JoltC_WheelSettingsTV* s) { return s ? asWSTV(s)->mLateralFriction : 0; }
JOLTC_API void  JoltC_WheelSettingsTV_SetLateralFriction(JoltC_WheelSettingsTV* s, float v) { if (s) asWSTV(s)->mLateralFriction = v; }

/* ========================================================================== */
/*  Wheel (runtime)                                                           */
/* ========================================================================== */
JOLTC_API const JoltC_WheelSettings* JoltC_Wheel_GetSettings(const JoltC_Wheel* w) {
    if (!w) return nullptr;
    return reinterpret_cast<const JoltC_WheelSettings*>(asWheel(w)->GetSettings());
}
JOLTC_API float JoltC_Wheel_GetAngularVelocity(const JoltC_Wheel* w) { return w ? asWheel(w)->GetAngularVelocity() : 0; }
JOLTC_API void  JoltC_Wheel_SetAngularVelocity(JoltC_Wheel* w, float v) { if (w) asWheel(w)->SetAngularVelocity(v); }
JOLTC_API float JoltC_Wheel_GetRotationAngle(const JoltC_Wheel* w) { return w ? asWheel(w)->GetRotationAngle() : 0; }
JOLTC_API void  JoltC_Wheel_SetRotationAngle(JoltC_Wheel* w, float v) { if (w) asWheel(w)->SetRotationAngle(v); }
JOLTC_API float JoltC_Wheel_GetSteerAngle(const JoltC_Wheel* w) { return w ? asWheel(w)->GetSteerAngle() : 0; }
JOLTC_API void  JoltC_Wheel_SetSteerAngle(JoltC_Wheel* w, float v) { if (w) asWheel(w)->SetSteerAngle(v); }
JOLTC_API JoltC_Bool JoltC_Wheel_HasContact(const JoltC_Wheel* w) { return w && asWheel(w)->HasContact() ? JOLTC_TRUE : JOLTC_FALSE; }
JOLTC_API JoltC_BodyID JoltC_Wheel_GetContactBodyID(const JoltC_Wheel* w) {
    return w ? asWheel(w)->GetContactBodyID().GetIndexAndSequenceNumber() : JOLTC_BODY_ID_INVALID;
}
JOLTC_API JoltC_SubShapeID JoltC_Wheel_GetContactSubShapeID(const JoltC_Wheel* w) {
    if (!w) return 0;
    return asWheel(w)->GetContactSubShapeID().GetValue();
}
JOLTC_API JoltC_RVec3 JoltC_Wheel_GetContactPosition(const JoltC_Wheel* w) {
    if (!w || !asWheel(w)->HasContact()) return JoltC_RVec3{0,0,0};
    return fromJphRVec3(asWheel(w)->GetContactPosition());
}
JOLTC_API JoltC_Vec3 JoltC_Wheel_GetContactPointVelocity(const JoltC_Wheel* w) {
    if (!w || !asWheel(w)->HasContact()) return JoltC_Vec3{0,0,0};
    return fromJphVec3(asWheel(w)->GetContactPointVelocity());
}
JOLTC_API JoltC_Vec3 JoltC_Wheel_GetContactNormal(const JoltC_Wheel* w) {
    if (!w || !asWheel(w)->HasContact()) return JoltC_Vec3{0,0,0};
    return fromJphVec3(asWheel(w)->GetContactNormal());
}
JOLTC_API JoltC_Vec3 JoltC_Wheel_GetContactLongitudinal(const JoltC_Wheel* w) {
    if (!w || !asWheel(w)->HasContact()) return JoltC_Vec3{0,0,0};
    return fromJphVec3(asWheel(w)->GetContactLongitudinal());
}
JOLTC_API JoltC_Vec3 JoltC_Wheel_GetContactLateral(const JoltC_Wheel* w) {
    if (!w || !asWheel(w)->HasContact()) return JoltC_Vec3{0,0,0};
    return fromJphVec3(asWheel(w)->GetContactLateral());
}
JOLTC_API float JoltC_Wheel_GetSuspensionLength(const JoltC_Wheel* w) { return w ? asWheel(w)->GetSuspensionLength() : 0; }
JOLTC_API float JoltC_Wheel_GetSuspensionLambda(const JoltC_Wheel* w) { return w ? asWheel(w)->GetSuspensionLambda() : 0; }
JOLTC_API float JoltC_Wheel_GetLongitudinalLambda(const JoltC_Wheel* w) { return w ? asWheel(w)->GetLongitudinalLambda() : 0; }
JOLTC_API float JoltC_Wheel_GetLateralLambda(const JoltC_Wheel* w) { return w ? asWheel(w)->GetLateralLambda() : 0; }
JOLTC_API JoltC_Bool JoltC_Wheel_HasHitHardPoint(const JoltC_Wheel* w) { return w && asWheel(w)->HasHitHardPoint() ? JOLTC_TRUE : JOLTC_FALSE; }

/* ========================================================================== */
/*  WheelWV / WheelTV                                                         */
/* ========================================================================== */
JOLTC_API const JoltC_WheelSettingsWV* JoltC_WheelWV_GetSettings(const JoltC_WheelWV* w) {
    if (!w) return nullptr;
    return reinterpret_cast<const JoltC_WheelSettingsWV*>(asWheelWV(w)->GetSettings());
}
JOLTC_API void JoltC_WheelWV_ApplyTorque(JoltC_WheelWV* w, float torque, float dt) {
    if (w) asWheelWV(w)->ApplyTorque(torque, dt);
}
JOLTC_API const JoltC_WheelSettingsTV* JoltC_WheelTV_GetSettings(const JoltC_WheelTV* w) {
    if (!w) return nullptr;
    return reinterpret_cast<const JoltC_WheelSettingsTV*>(asWheelTV(w)->GetSettings());
}

/* ========================================================================== */
/*  VehicleConstraintSettings                                                 */
/* ========================================================================== */
JOLTC_API void JoltC_VehicleConstraintSettings_Init(JoltC_VehicleConstraintSettings* s) {
    if (!s) return;
    s->up = JoltC_Vec3{0, 1, 0};
    s->forward = JoltC_Vec3{0, 0, 1};
    s->maxPitchRollAngle = 3.14159265358979323846f; /* JPH_PI */
    s->wheelsCount = 0;
    s->wheels = nullptr;
    s->antiRollBarsCount = 0;
    s->antiRollBars = nullptr;
    s->controller = nullptr;
}

/* ========================================================================== */
/*  VehicleConstraint                                                         */
/* ========================================================================== */
JOLTC_API JoltC_VehicleConstraint* JoltC_VehicleConstraint_Create(JoltC_Body* body, const JoltC_VehicleConstraintSettings* settings) {
    if (!body || !settings) return nullptr;
    JOLTC_TRY_BEGIN
    Body* jphBody = reinterpret_cast<Body*>(body);

    VehicleConstraintSettings jphSettings;
    jphSettings.mUp = toJphVec3(settings->up);
    jphSettings.mForward = toJphVec3(settings->forward);
    jphSettings.mMaxPitchRollAngle = settings->maxPitchRollAngle;

    /* Wheels */
    for (uint32_t i = 0; i < settings->wheelsCount; i++) {
        if (settings->wheels[i])
            jphSettings.mWheels.push_back(asWS(settings->wheels[i]));
    }

    /* Anti-roll bars */
    for (uint32_t i = 0; i < settings->antiRollBarsCount; i++) {
        VehicleAntiRollBar bar;
        bar.mLeftWheel = settings->antiRollBars[i].leftWheel;
        bar.mRightWheel = settings->antiRollBars[i].rightWheel;
        bar.mStiffness = settings->antiRollBars[i].stiffness;
        jphSettings.mAntiRollBars.push_back(bar);
    }

    /* Controller */
    if (settings->controller)
        jphSettings.mController = asVCS(settings->controller);

    auto* vc = new VehicleConstraint(*jphBody, jphSettings);
    vc->AddRef();
    return reinterpret_cast<JoltC_VehicleConstraint*>(vc);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_PhysicsStepListener* JoltC_VehicleConstraint_AsPhysicsStepListener(JoltC_VehicleConstraint* vc) {
    if (!vc) return nullptr;
    return reinterpret_cast<JoltC_PhysicsStepListener*>(static_cast<PhysicsStepListener*>(asVC(vc)));
}

JOLTC_API JoltC_Constraint* JoltC_VehicleConstraint_AsConstraint(JoltC_VehicleConstraint* vc) {
    if (!vc) return nullptr;
    JOLTC_TRY_BEGIN
    /* Same shape as JoltC_Character_AsBase: a fresh wrapper holding its own Ref, freed by
     * JoltC_Constraint_Destroy. The Ref keeps the vehicle alive while the view exists. */
    auto* w = new JoltC_Constraint;
    w->ptr = static_cast<Constraint*>(asVC(vc));
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_PhysicsSystem_AddVehicleStepListener(JoltC_PhysicsSystem* system, JoltC_VehicleConstraint* vc) {
    if (!system || !vc) return;
    JOLTC_TRY_BEGIN
    system->ptr->AddStepListener(static_cast<PhysicsStepListener*>(asVC(vc)));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_PhysicsSystem_RemoveVehicleStepListener(JoltC_PhysicsSystem* system, JoltC_VehicleConstraint* vc) {
    if (!system || !vc) return;
    JOLTC_TRY_BEGIN
    system->ptr->RemoveStepListener(static_cast<PhysicsStepListener*>(asVC(vc)));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_VehicleConstraint_Destroy(JoltC_VehicleConstraint* vc) {
    if (!vc) return;
    JOLTC_TRY_BEGIN
    asVC(vc)->Release();
    JOLTC_TRY_END
}

JOLTC_API void JoltC_VehicleConstraint_SetMaxPitchRollAngle(JoltC_VehicleConstraint* vc, float angle) {
    if (vc) asVC(vc)->SetMaxPitchRollAngle(angle);
}

JOLTC_API void JoltC_VehicleConstraint_SetVehicleCollisionTester(JoltC_VehicleConstraint* vc, const JoltC_VehicleCollisionTester* tester) {
    if (vc) asVC(vc)->SetVehicleCollisionTester(asVCT(tester));
}

JOLTC_API void JoltC_VehicleConstraint_OverrideGravity(JoltC_VehicleConstraint* vc, JoltC_Vec3 gravity) {
    if (vc) asVC(vc)->OverrideGravity(toJphVec3(gravity));
}

JOLTC_API JoltC_Bool JoltC_VehicleConstraint_IsGravityOverridden(const JoltC_VehicleConstraint* vc) {
    return vc && asVC(vc)->IsGravityOverridden() ? JOLTC_TRUE : JOLTC_FALSE;
}

JOLTC_API JoltC_Vec3 JoltC_VehicleConstraint_GetGravityOverride(const JoltC_VehicleConstraint* vc) {
    if (!vc) return JoltC_Vec3{0,0,0};
    return fromJphVec3(asVC(vc)->GetGravityOverride());
}

JOLTC_API void JoltC_VehicleConstraint_ResetGravityOverride(JoltC_VehicleConstraint* vc) {
    if (vc) asVC(vc)->ResetGravityOverride();
}

JOLTC_API JoltC_Vec3 JoltC_VehicleConstraint_GetLocalForward(const JoltC_VehicleConstraint* vc) {
    if (!vc) return JoltC_Vec3{0,0,0};
    return fromJphVec3(asVC(vc)->GetLocalForward());
}

JOLTC_API JoltC_Vec3 JoltC_VehicleConstraint_GetLocalUp(const JoltC_VehicleConstraint* vc) {
    if (!vc) return JoltC_Vec3{0,0,0};
    return fromJphVec3(asVC(vc)->GetLocalUp());
}

JOLTC_API JoltC_Vec3 JoltC_VehicleConstraint_GetWorldUp(const JoltC_VehicleConstraint* vc) {
    if (!vc) return JoltC_Vec3{0,0,0};
    return fromJphVec3(asVC(vc)->GetWorldUp());
}

JOLTC_API const JoltC_Body* JoltC_VehicleConstraint_GetVehicleBody(const JoltC_VehicleConstraint* vc) {
    if (!vc) return nullptr;
    return reinterpret_cast<const JoltC_Body*>(asVC(vc)->GetVehicleBody());
}

JOLTC_API JoltC_VehicleController* JoltC_VehicleConstraint_GetController(JoltC_VehicleConstraint* vc) {
    if (!vc) return nullptr;
    return reinterpret_cast<JoltC_VehicleController*>(asVC(vc)->GetController());
}

JOLTC_API uint32_t JoltC_VehicleConstraint_GetWheelsCount(JoltC_VehicleConstraint* vc) {
    if (!vc) return 0;
    return (uint32_t)asVC(vc)->GetWheels().size();
}

JOLTC_API JoltC_Wheel* JoltC_VehicleConstraint_GetWheel(JoltC_VehicleConstraint* vc, uint32_t index) {
    if (!vc) return nullptr;
    auto& wheels = asVC(vc)->GetWheels();
    if (index >= wheels.size()) return nullptr;
    return fromWheel(wheels[index]);
}

JOLTC_API void JoltC_VehicleConstraint_GetWheelLocalTransform(JoltC_VehicleConstraint* vc, uint32_t wheelIndex, JoltC_Vec3 wheelRight, JoltC_Vec3 wheelUp, JoltC_Mat44* result) {
    if (!vc || !result) return;
    JOLTC_TRY_BEGIN
    Mat44 m = asVC(vc)->GetWheelLocalTransform(wheelIndex, toJphVec3(wheelRight), toJphVec3(wheelUp));
    *result = fromJphMat44(m);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_VehicleConstraint_GetWheelWorldTransform(JoltC_VehicleConstraint* vc, uint32_t wheelIndex, JoltC_Vec3 wheelRight, JoltC_Vec3 wheelUp, JoltC_Mat44* result) {
    if (!vc || !result) return;
    JOLTC_TRY_BEGIN
    RMat44 m = asVC(vc)->GetWheelWorldTransform(wheelIndex, toJphVec3(wheelRight), toJphVec3(wheelUp));
    *result = fromJphMat44(m);
    JOLTC_TRY_END
}

/* ========================================================================== */
/*  VehicleCollisionTester                                                    */
/* ========================================================================== */
JOLTC_API void JoltC_VehicleCollisionTester_Destroy(JoltC_VehicleCollisionTester* t) {
    if (t) asVCT(t)->Release();
}
JOLTC_API JoltC_ObjectLayer JoltC_VehicleCollisionTester_GetObjectLayer(const JoltC_VehicleCollisionTester* t) {
    return t ? (JoltC_ObjectLayer)asVCT(t)->GetObjectLayer() : JOLTC_OBJECT_LAYER_INVALID;
}
JOLTC_API void JoltC_VehicleCollisionTester_SetObjectLayer(JoltC_VehicleCollisionTester* t, JoltC_ObjectLayer layer) {
    if (t) asVCT(t)->SetObjectLayer((ObjectLayer)layer);
}

JOLTC_API JoltC_VehicleCollisionTester* JoltC_VehicleCollisionTesterRay_Create(JoltC_ObjectLayer layer, JoltC_Vec3 up, float maxSlopeAngle) {
    JOLTC_TRY_BEGIN
    auto* p = new VehicleCollisionTesterRay((ObjectLayer)layer, toJphVec3(up), maxSlopeAngle);
    p->AddRef();
    return fromVCT(p);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_VehicleCollisionTester* JoltC_VehicleCollisionTesterCastSphere_Create(JoltC_ObjectLayer layer, float radius, JoltC_Vec3 up, float maxSlopeAngle) {
    JOLTC_TRY_BEGIN
    auto* p = new VehicleCollisionTesterCastSphere((ObjectLayer)layer, radius, toJphVec3(up), maxSlopeAngle);
    p->AddRef();
    return fromVCT(p);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_VehicleCollisionTester* JoltC_VehicleCollisionTesterCastCylinder_Create(JoltC_ObjectLayer layer, float convexRadiusFraction) {
    JOLTC_TRY_BEGIN
    auto* p = new VehicleCollisionTesterCastCylinder((ObjectLayer)layer, convexRadiusFraction);
    p->AddRef();
    return fromVCT(p);
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  VehicleControllerSettings                                                 */
/* ========================================================================== */
JOLTC_API void JoltC_VehicleControllerSettings_Destroy(JoltC_VehicleControllerSettings* s) {
    if (s) asVCS(s)->Release();
}

/* ========================================================================== */
/*  WheeledVehicleControllerSettings                                          */
/* ========================================================================== */
JOLTC_API JoltC_WheeledVehicleControllerSettings* JoltC_WheeledVehicleControllerSettings_Create(void) {
    JOLTC_TRY_BEGIN
    auto* s = new WheeledVehicleControllerSettings;
    s->AddRef();
    return reinterpret_cast<JoltC_WheeledVehicleControllerSettings*>(s);
    JOLTC_TRY_END
    return nullptr;
}
JOLTC_API void JoltC_WheeledVehicleControllerSettings_GetEngine(const JoltC_WheeledVehicleControllerSettings* s, JoltC_VehicleEngineSettings* result) {
    if (!s || !result) return;
    fromJphEngineSettings(asWVCS(s)->mEngine, result);
}
JOLTC_API void JoltC_WheeledVehicleControllerSettings_SetEngine(JoltC_WheeledVehicleControllerSettings* s, const JoltC_VehicleEngineSettings* value) {
    if (!s || !value) return;
    asWVCS(s)->mEngine = toJphEngineSettings(*value);
}
JOLTC_API const JoltC_VehicleTransmissionSettings* JoltC_WheeledVehicleControllerSettings_GetTransmission(const JoltC_WheeledVehicleControllerSettings* s) {
    if (!s) return nullptr;
    /* Return a thread-local wrapper pointing at the internal settings */
    static thread_local JoltC_VehicleTransmissionSettings tl_ts;
    tl_ts.settings = asWVCS(s)->mTransmission;
    return &tl_ts;
}
JOLTC_API void JoltC_WheeledVehicleControllerSettings_SetTransmission(JoltC_WheeledVehicleControllerSettings* s, const JoltC_VehicleTransmissionSettings* value) {
    if (!s || !value) return;
    asWVCS(s)->mTransmission = value->settings;
}
JOLTC_API uint32_t JoltC_WheeledVehicleControllerSettings_GetDifferentialsCount(const JoltC_WheeledVehicleControllerSettings* s) {
    return s ? (uint32_t)asWVCS(s)->mDifferentials.size() : 0;
}
JOLTC_API void JoltC_WheeledVehicleControllerSettings_SetDifferentialsCount(JoltC_WheeledVehicleControllerSettings* s, uint32_t count) {
    if (s) asWVCS(s)->mDifferentials.resize(count);
}
JOLTC_API void JoltC_WheeledVehicleControllerSettings_GetDifferential(const JoltC_WheeledVehicleControllerSettings* s, uint32_t index, JoltC_VehicleDifferentialSettings* result) {
    if (!s || !result || index >= asWVCS(s)->mDifferentials.size()) return;
    fromJphDiffSettings(asWVCS(s)->mDifferentials[index], result);
}
JOLTC_API void JoltC_WheeledVehicleControllerSettings_SetDifferential(JoltC_WheeledVehicleControllerSettings* s, uint32_t index, const JoltC_VehicleDifferentialSettings* value) {
    if (!s || !value || index >= asWVCS(s)->mDifferentials.size()) return;
    asWVCS(s)->mDifferentials[index] = toJphDiffSettings(*value);
}
JOLTC_API void JoltC_WheeledVehicleControllerSettings_SetDifferentials(JoltC_WheeledVehicleControllerSettings* s, const JoltC_VehicleDifferentialSettings* values, uint32_t count) {
    if (!s || !values) return;
    asWVCS(s)->mDifferentials.clear();
    asWVCS(s)->mDifferentials.reserve(count);
    for (uint32_t i = 0; i < count; i++)
        asWVCS(s)->mDifferentials.push_back(toJphDiffSettings(values[i]));
}
JOLTC_API float JoltC_WheeledVehicleControllerSettings_GetDifferentialLimitedSlipRatio(const JoltC_WheeledVehicleControllerSettings* s) {
    return s ? asWVCS(s)->mDifferentialLimitedSlipRatio : 0;
}
JOLTC_API void JoltC_WheeledVehicleControllerSettings_SetDifferentialLimitedSlipRatio(JoltC_WheeledVehicleControllerSettings* s, float value) {
    if (s) asWVCS(s)->mDifferentialLimitedSlipRatio = value;
}

/* ========================================================================== */
/*  MotorcycleControllerSettings                                              */
/* ========================================================================== */
JOLTC_API JoltC_MotorcycleControllerSettings* JoltC_MotorcycleControllerSettings_Create(void) {
    JOLTC_TRY_BEGIN
    auto* s = new MotorcycleControllerSettings;
    s->AddRef();
    return reinterpret_cast<JoltC_MotorcycleControllerSettings*>(s);
    JOLTC_TRY_END
    return nullptr;
}
#define MCS_FLOAT(Name, Field) \
JOLTC_API float JoltC_MotorcycleControllerSettings_Get##Name(const JoltC_MotorcycleControllerSettings* s) { return s ? asMCS(s)->Field : 0; } \
JOLTC_API void JoltC_MotorcycleControllerSettings_Set##Name(JoltC_MotorcycleControllerSettings* s, float v) { if (s) asMCS(s)->Field = v; }
MCS_FLOAT(MaxLeanAngle, mMaxLeanAngle)
MCS_FLOAT(LeanSpringConstant, mLeanSpringConstant)
MCS_FLOAT(LeanSpringDamping, mLeanSpringDamping)
MCS_FLOAT(LeanSpringIntegrationCoefficient, mLeanSpringIntegrationCoefficient)
MCS_FLOAT(LeanSpringIntegrationCoefficientDecay, mLeanSpringIntegrationCoefficientDecay)
MCS_FLOAT(LeanSmoothingFactor, mLeanSmoothingFactor)
#undef MCS_FLOAT

/* ========================================================================== */
/*  TrackedVehicleControllerSettings                                          */
/* ========================================================================== */
JOLTC_API JoltC_TrackedVehicleControllerSettings* JoltC_TrackedVehicleControllerSettings_Create(void) {
    JOLTC_TRY_BEGIN
    auto* s = new TrackedVehicleControllerSettings;
    s->AddRef();
    return reinterpret_cast<JoltC_TrackedVehicleControllerSettings*>(s);
    JOLTC_TRY_END
    return nullptr;
}
JOLTC_API void JoltC_TrackedVehicleControllerSettings_GetEngine(const JoltC_TrackedVehicleControllerSettings* s, JoltC_VehicleEngineSettings* result) {
    if (!s || !result) return;
    fromJphEngineSettings(asTVCS(s)->mEngine, result);
}
JOLTC_API void JoltC_TrackedVehicleControllerSettings_SetEngine(JoltC_TrackedVehicleControllerSettings* s, const JoltC_VehicleEngineSettings* value) {
    if (!s || !value) return;
    asTVCS(s)->mEngine = toJphEngineSettings(*value);
}
JOLTC_API const JoltC_VehicleTransmissionSettings* JoltC_TrackedVehicleControllerSettings_GetTransmission(const JoltC_TrackedVehicleControllerSettings* s) {
    if (!s) return nullptr;
    static thread_local JoltC_VehicleTransmissionSettings tl_ts;
    tl_ts.settings = asTVCS(s)->mTransmission;
    return &tl_ts;
}
JOLTC_API void JoltC_TrackedVehicleControllerSettings_SetTransmission(JoltC_TrackedVehicleControllerSettings* s, const JoltC_VehicleTransmissionSettings* value) {
    if (!s || !value) return;
    asTVCS(s)->mTransmission = value->settings;
}

/* ========================================================================== */
/*  WheeledVehicleController (runtime)                                        */
/* ========================================================================== */
JOLTC_API void JoltC_WheeledVehicleController_SetDriverInput(JoltC_WheeledVehicleController* c, float forward, float right, float brake, float handBrake) {
    if (c) asWVC(c)->SetDriverInput(forward, right, brake, handBrake);
}
JOLTC_API void  JoltC_WheeledVehicleController_SetForwardInput(JoltC_WheeledVehicleController* c, float v) { if (c) asWVC(c)->SetForwardInput(v); }
JOLTC_API float JoltC_WheeledVehicleController_GetForwardInput(const JoltC_WheeledVehicleController* c) { return c ? asWVC(c)->GetForwardInput() : 0; }
JOLTC_API void  JoltC_WheeledVehicleController_SetRightInput(JoltC_WheeledVehicleController* c, float v) { if (c) asWVC(c)->SetRightInput(v); }
JOLTC_API float JoltC_WheeledVehicleController_GetRightInput(const JoltC_WheeledVehicleController* c) { return c ? asWVC(c)->GetRightInput() : 0; }
JOLTC_API void  JoltC_WheeledVehicleController_SetBrakeInput(JoltC_WheeledVehicleController* c, float v) { if (c) asWVC(c)->SetBrakeInput(v); }
JOLTC_API float JoltC_WheeledVehicleController_GetBrakeInput(const JoltC_WheeledVehicleController* c) { return c ? asWVC(c)->GetBrakeInput() : 0; }
JOLTC_API void  JoltC_WheeledVehicleController_SetHandBrakeInput(JoltC_WheeledVehicleController* c, float v) { if (c) asWVC(c)->SetHandBrakeInput(v); }
JOLTC_API float JoltC_WheeledVehicleController_GetHandBrakeInput(const JoltC_WheeledVehicleController* c) { return c ? asWVC(c)->GetHandBrakeInput() : 0; }
JOLTC_API float JoltC_WheeledVehicleController_GetWheelSpeedAtClutch(const JoltC_WheeledVehicleController* c) {
    return c ? asWVC(c)->GetWheelSpeedAtClutch() : 0;
}
JOLTC_API void JoltC_WheeledVehicleController_SetTireMaxImpulseCallback(JoltC_WheeledVehicleController* c, JoltC_TireMaxImpulseCallback cb, void* userData) {
    if (!c) return;
    if (!cb) {
        /* Reset to default */
        asWVC(c)->SetTireMaxImpulseCallback(
            [](uint, float &outLong, float &outLat, float suspImpulse, float longFriction, float latFriction, float, float, float) {
                outLong = longFriction * suspImpulse;
                outLat = latFriction * suspImpulse;
            });
        return;
    }
    asWVC(c)->SetTireMaxImpulseCallback(
        [cb, userData](uint inWheelIndex, float &outLong, float &outLat, float suspImpulse, float longFriction, float latFriction, float longSlip, float latSlip, float dt) {
            cb(userData, inWheelIndex, &outLong, &outLat, suspImpulse, longFriction, latFriction, longSlip, latSlip, dt);
        });
}
JOLTC_API const JoltC_VehicleEngine* JoltC_WheeledVehicleController_GetEngine(const JoltC_WheeledVehicleController* c) {
    if (!c) return nullptr;
    return reinterpret_cast<const JoltC_VehicleEngine*>(&asWVC(c)->GetEngine());
}
JOLTC_API const JoltC_VehicleTransmission* JoltC_WheeledVehicleController_GetTransmission(const JoltC_WheeledVehicleController* c) {
    if (!c) return nullptr;
    return reinterpret_cast<const JoltC_VehicleTransmission*>(&asWVC(c)->GetTransmission());
}

/* ========================================================================== */
/*  MotorcycleController (runtime)                                            */
/* ========================================================================== */
JOLTC_API float JoltC_MotorcycleController_GetWheelBase(const JoltC_MotorcycleController* c) {
    return c ? asMC(c)->GetWheelBase() : 0;
}
JOLTC_API JoltC_Bool JoltC_MotorcycleController_IsLeanControllerEnabled(const JoltC_MotorcycleController* c) {
    return c && asMC(c)->IsLeanControllerEnabled() ? JOLTC_TRUE : JOLTC_FALSE;
}
JOLTC_API void JoltC_MotorcycleController_EnableLeanController(JoltC_MotorcycleController* c, JoltC_Bool v) {
    if (c) asMC(c)->EnableLeanController(v != JOLTC_FALSE);
}
JOLTC_API JoltC_Bool JoltC_MotorcycleController_IsLeanSteeringLimitEnabled(const JoltC_MotorcycleController* c) {
    return c && asMC(c)->IsLeanSteeringLimitEnabled() ? JOLTC_TRUE : JOLTC_FALSE;
}
JOLTC_API void JoltC_MotorcycleController_EnableLeanSteeringLimit(JoltC_MotorcycleController* c, JoltC_Bool v) {
    if (c) asMC(c)->EnableLeanSteeringLimit(v != JOLTC_FALSE);
}
#define MC_FLOAT(Name, Get, Set) \
JOLTC_API float JoltC_MotorcycleController_Get##Name(const JoltC_MotorcycleController* c) { return c ? asMC(c)->Get() : 0; } \
JOLTC_API void JoltC_MotorcycleController_Set##Name(JoltC_MotorcycleController* c, float v) { if (c) asMC(c)->Set(v); }
MC_FLOAT(LeanSpringConstant, GetLeanSpringConstant, SetLeanSpringConstant)
MC_FLOAT(LeanSpringDamping, GetLeanSpringDamping, SetLeanSpringDamping)
MC_FLOAT(LeanSpringIntegrationCoefficient, GetLeanSpringIntegrationCoefficient, SetLeanSpringIntegrationCoefficient)
MC_FLOAT(LeanSpringIntegrationCoefficientDecay, GetLeanSpringIntegrationCoefficientDecay, SetLeanSpringIntegrationCoefficientDecay)
MC_FLOAT(LeanSmoothingFactor, GetLeanSmoothingFactor, SetLeanSmoothingFactor)
#undef MC_FLOAT

/* ========================================================================== */
/*  TrackedVehicleController (runtime)                                        */
/* ========================================================================== */
JOLTC_API const JoltC_VehicleTrack* JoltC_TrackedVehicleController_GetTrack(const JoltC_TrackedVehicleController* c, JoltC_TrackSide side) {
    if (!c) return nullptr;
    return reinterpret_cast<const JoltC_VehicleTrack*>(&asTVC(c)->GetTracks()[(int)side]);
}
JOLTC_API void JoltC_TrackedVehicleController_SetDriverInput(JoltC_TrackedVehicleController* c, float forward, float leftRatio, float rightRatio, float brake) {
    if (c) asTVC(c)->SetDriverInput(forward, leftRatio, rightRatio, brake);
}
JOLTC_API float JoltC_TrackedVehicleController_GetForwardInput(const JoltC_TrackedVehicleController* c) { return c ? asTVC(c)->GetForwardInput() : 0; }
JOLTC_API void  JoltC_TrackedVehicleController_SetForwardInput(JoltC_TrackedVehicleController* c, float v) { if (c) asTVC(c)->SetForwardInput(v); }
JOLTC_API float JoltC_TrackedVehicleController_GetLeftRatio(const JoltC_TrackedVehicleController* c) { return c ? asTVC(c)->GetLeftRatio() : 0; }
JOLTC_API void  JoltC_TrackedVehicleController_SetLeftRatio(JoltC_TrackedVehicleController* c, float v) { if (c) asTVC(c)->SetLeftRatio(v); }
JOLTC_API float JoltC_TrackedVehicleController_GetRightRatio(const JoltC_TrackedVehicleController* c) { return c ? asTVC(c)->GetRightRatio() : 0; }
JOLTC_API void  JoltC_TrackedVehicleController_SetRightRatio(JoltC_TrackedVehicleController* c, float v) { if (c) asTVC(c)->SetRightRatio(v); }
JOLTC_API float JoltC_TrackedVehicleController_GetBrakeInput(const JoltC_TrackedVehicleController* c) { return c ? asTVC(c)->GetBrakeInput() : 0; }
JOLTC_API void  JoltC_TrackedVehicleController_SetBrakeInput(JoltC_TrackedVehicleController* c, float v) { if (c) asTVC(c)->SetBrakeInput(v); }
JOLTC_API const JoltC_VehicleEngine* JoltC_TrackedVehicleController_GetEngine(const JoltC_TrackedVehicleController* c) {
    if (!c) return nullptr;
    return reinterpret_cast<const JoltC_VehicleEngine*>(&asTVC(c)->GetEngine());
}
JOLTC_API const JoltC_VehicleTransmission* JoltC_TrackedVehicleController_GetTransmission(const JoltC_TrackedVehicleController* c) {
    if (!c) return nullptr;
    return reinterpret_cast<const JoltC_VehicleTransmission*>(&asTVC(c)->GetTransmission());
}

/* ========================================================================== */
/*  VehicleEngine (runtime)                                                   */
/* ========================================================================== */
JOLTC_API void JoltC_VehicleEngine_ClampRPM(JoltC_VehicleEngine* e) { if (e) asVE(e)->ClampRPM(); }
JOLTC_API float JoltC_VehicleEngine_GetCurrentRPM(const JoltC_VehicleEngine* e) { return e ? asVE(e)->GetCurrentRPM() : 0; }
JOLTC_API void JoltC_VehicleEngine_SetCurrentRPM(JoltC_VehicleEngine* e, float rpm) { if (e) asVE(e)->SetCurrentRPM(rpm); }
JOLTC_API float JoltC_VehicleEngine_GetAngularVelocity(const JoltC_VehicleEngine* e) { return e ? asVE(e)->GetAngularVelocity() : 0; }
JOLTC_API float JoltC_VehicleEngine_GetTorque(const JoltC_VehicleEngine* e, float acceleration) { return e ? asVE(e)->GetTorque(acceleration) : 0; }
JOLTC_API void JoltC_VehicleEngine_ApplyTorque(JoltC_VehicleEngine* e, float torque, float dt) { if (e) asVE(e)->ApplyTorque(torque, dt); }
JOLTC_API void JoltC_VehicleEngine_ApplyDamping(JoltC_VehicleEngine* e, float dt) { if (e) asVE(e)->ApplyDamping(dt); }
JOLTC_API JoltC_Bool JoltC_VehicleEngine_AllowSleep(const JoltC_VehicleEngine* e) {
    return e && asVE(e)->AllowSleep() ? JOLTC_TRUE : JOLTC_FALSE;
}

/* ========================================================================== */
/*  VehicleTransmissionSettings (opaque)                                      */
/* ========================================================================== */
JOLTC_API JoltC_VehicleTransmissionSettings* JoltC_VehicleTransmissionSettings_Create(void) {
    return new JoltC_VehicleTransmissionSettings;
}
JOLTC_API void JoltC_VehicleTransmissionSettings_Destroy(JoltC_VehicleTransmissionSettings* s) { delete s; }
JOLTC_API JoltC_TransmissionMode JoltC_VehicleTransmissionSettings_GetMode(const JoltC_VehicleTransmissionSettings* s) {
    return s ? (JoltC_TransmissionMode)s->settings.mMode : JOLTC_TRANSMISSION_MODE_AUTO;
}
JOLTC_API void JoltC_VehicleTransmissionSettings_SetMode(JoltC_VehicleTransmissionSettings* s, JoltC_TransmissionMode mode) {
    if (s) s->settings.mMode = (ETransmissionMode)mode;
}
JOLTC_API uint32_t JoltC_VehicleTransmissionSettings_GetGearRatioCount(const JoltC_VehicleTransmissionSettings* s) {
    return s ? (uint32_t)s->settings.mGearRatios.size() : 0;
}
JOLTC_API float JoltC_VehicleTransmissionSettings_GetGearRatio(const JoltC_VehicleTransmissionSettings* s, uint32_t index) {
    if (!s || index >= s->settings.mGearRatios.size()) return 0;
    return s->settings.mGearRatios[index];
}
JOLTC_API void JoltC_VehicleTransmissionSettings_SetGearRatios(JoltC_VehicleTransmissionSettings* s, const float* values, uint32_t count) {
    if (!s || !values) return;
    s->settings.mGearRatios.assign(values, values + count);
}
JOLTC_API uint32_t JoltC_VehicleTransmissionSettings_GetReverseGearRatioCount(const JoltC_VehicleTransmissionSettings* s) {
    return s ? (uint32_t)s->settings.mReverseGearRatios.size() : 0;
}
JOLTC_API float JoltC_VehicleTransmissionSettings_GetReverseGearRatio(const JoltC_VehicleTransmissionSettings* s, uint32_t index) {
    if (!s || index >= s->settings.mReverseGearRatios.size()) return 0;
    return s->settings.mReverseGearRatios[index];
}
JOLTC_API void JoltC_VehicleTransmissionSettings_SetReverseGearRatios(JoltC_VehicleTransmissionSettings* s, const float* values, uint32_t count) {
    if (!s || !values) return;
    s->settings.mReverseGearRatios.assign(values, values + count);
}
#define TS_FLOAT(Name, Field) \
JOLTC_API float JoltC_VehicleTransmissionSettings_Get##Name(const JoltC_VehicleTransmissionSettings* s) { return s ? s->settings.Field : 0; } \
JOLTC_API void JoltC_VehicleTransmissionSettings_Set##Name(JoltC_VehicleTransmissionSettings* s, float v) { if (s) s->settings.Field = v; }
TS_FLOAT(SwitchTime, mSwitchTime)
TS_FLOAT(ClutchReleaseTime, mClutchReleaseTime)
TS_FLOAT(SwitchLatency, mSwitchLatency)
TS_FLOAT(ShiftUpRPM, mShiftUpRPM)
TS_FLOAT(ShiftDownRPM, mShiftDownRPM)
TS_FLOAT(ClutchStrength, mClutchStrength)
#undef TS_FLOAT

/* ========================================================================== */
/*  VehicleTransmission (runtime)                                             */
/* ========================================================================== */
JOLTC_API void JoltC_VehicleTransmission_Set(JoltC_VehicleTransmission* t, int currentGear, float clutchFriction) {
    if (t) asVT(t)->Set(currentGear, clutchFriction);
}
JOLTC_API void JoltC_VehicleTransmission_Update(JoltC_VehicleTransmission* t, float dt, float currentRPM, float forwardInput, JoltC_Bool canShiftUp) {
    if (t) asVT(t)->Update(dt, currentRPM, forwardInput, canShiftUp != JOLTC_FALSE);
}
JOLTC_API int JoltC_VehicleTransmission_GetCurrentGear(const JoltC_VehicleTransmission* t) { return t ? asVT(t)->GetCurrentGear() : 0; }
JOLTC_API float JoltC_VehicleTransmission_GetClutchFriction(const JoltC_VehicleTransmission* t) { return t ? asVT(t)->GetClutchFriction() : 0; }
JOLTC_API JoltC_Bool JoltC_VehicleTransmission_IsSwitchingGear(const JoltC_VehicleTransmission* t) {
    return t && asVT(t)->IsSwitchingGear() ? JOLTC_TRUE : JOLTC_FALSE;
}
JOLTC_API float JoltC_VehicleTransmission_GetCurrentRatio(const JoltC_VehicleTransmission* t) { return t ? asVT(t)->GetCurrentRatio() : 0; }
JOLTC_API JoltC_Bool JoltC_VehicleTransmission_AllowSleep(const JoltC_VehicleTransmission* t) {
    return t && asVT(t)->AllowSleep() ? JOLTC_TRUE : JOLTC_FALSE;
}

/* ========================================================================== */
/*  VehicleTrack (runtime)                                                    */
/* ========================================================================== */
JOLTC_API float JoltC_VehicleTrack_GetAngularVelocity(const JoltC_VehicleTrack* t) { return t ? asVTr(t)->mAngularVelocity : 0; }
JOLTC_API void  JoltC_VehicleTrack_SetAngularVelocity(JoltC_VehicleTrack* t, float v) { if (t) asVTr(t)->mAngularVelocity = v; }
JOLTC_API uint32_t JoltC_VehicleTrack_GetDrivenWheel(const JoltC_VehicleTrack* t) { return t ? asVTr(t)->mDrivenWheel : 0; }
JOLTC_API float JoltC_VehicleTrack_GetInertia(const JoltC_VehicleTrack* t) { return t ? asVTr(t)->mInertia : 0; }
JOLTC_API float JoltC_VehicleTrack_GetAngularDamping(const JoltC_VehicleTrack* t) { return t ? asVTr(t)->mAngularDamping : 0; }
JOLTC_API float JoltC_VehicleTrack_GetMaxBrakeTorque(const JoltC_VehicleTrack* t) { return t ? asVTr(t)->mMaxBrakeTorque : 0; }
JOLTC_API float JoltC_VehicleTrack_GetDifferentialRatio(const JoltC_VehicleTrack* t) { return t ? asVTr(t)->mDifferentialRatio : 0; }

/* ========================================================================== */
/*  Init helpers                                                              */
/* ========================================================================== */
JOLTC_API void JoltC_VehicleEngineSettings_Init(JoltC_VehicleEngineSettings* s) {
    if (!s) return;
    VehicleEngineSettings defaults;
    fromJphEngineSettings(defaults, s);
}
JOLTC_API void JoltC_VehicleDifferentialSettings_Init(JoltC_VehicleDifferentialSettings* s) {
    if (!s) return;
    VehicleDifferentialSettings defaults;
    fromJphDiffSettings(defaults, &*s);
}
JOLTC_API void JoltC_VehicleAntiRollBar_Init(JoltC_VehicleAntiRollBar* bar) {
    if (!bar) return;
    VehicleAntiRollBar defaults;
    bar->leftWheel = defaults.mLeftWheel;
    bar->rightWheel = defaults.mRightWheel;
    bar->stiffness = defaults.mStiffness;
}
JOLTC_API void JoltC_VehicleTrackSettings_Init(JoltC_VehicleTrackSettings* s) {
    if (!s) return;
    VehicleTrackSettings defaults;
    s->drivenWheel = defaults.mDrivenWheel;
    s->wheels = nullptr;
    s->wheelsCount = 0;
    s->inertia = defaults.mInertia;
    s->angularDamping = defaults.mAngularDamping;
    s->maxBrakeTorque = defaults.mMaxBrakeTorque;
    s->differentialRatio = defaults.mDifferentialRatio;
}

/* ========================================================================== */
/*  VehicleConstraint - GetWheelLocalBasis                                    */
/* ========================================================================== */
JOLTC_API void JoltC_VehicleConstraint_GetWheelLocalBasis(const JoltC_VehicleConstraint* constraint, uint32_t wheelIndex, JoltC_Vec3* outUp, JoltC_Vec3* outForward)
{
    if (!constraint || !outUp || !outForward) return;
    JOLTC_TRY_BEGIN
    const VehicleConstraint* vc = asVC(constraint);
    const Wheel* wheel = vc->GetWheel(wheelIndex);
    Vec3 forward, up, right;
    vc->GetWheelLocalBasis(wheel, forward, up, right);
    *outUp = fromJphVec3(up);
    *outForward = fromJphVec3(forward);
    JOLTC_TRY_END
}

/* ========================================================================== */
/*  VehicleTransmissionSettings - individual gear ratio setters               */
/* ========================================================================== */
JOLTC_API void JoltC_VehicleTransmissionSettings_SetGearRatio(JoltC_VehicleTransmissionSettings* s, int gearIndex, float ratio)
{
    if (!s || gearIndex < 0) return;
    JOLTC_TRY_BEGIN
    auto& ratios = s->settings.mGearRatios;
    if ((uint32_t)gearIndex >= ratios.size())
        ratios.resize(gearIndex + 1, 0.0f);
    ratios[gearIndex] = ratio;
    JOLTC_TRY_END
}

JOLTC_API void JoltC_VehicleTransmissionSettings_SetReverseGearRatio(JoltC_VehicleTransmissionSettings* s, int gearIndex, float ratio)
{
    if (!s || gearIndex < 0) return;
    JOLTC_TRY_BEGIN
    auto& ratios = s->settings.mReverseGearRatios;
    if ((uint32_t)gearIndex >= ratios.size())
        ratios.resize(gearIndex + 1, 0.0f);
    ratios[gearIndex] = ratio;
    JOLTC_TRY_END
}

/* ========================================================================== */
/*  Wheel / WheelWV / WheelTV - Create / Destroy                             */
/* ========================================================================== */
JOLTC_API JoltC_Wheel* JoltC_Wheel_Create(const JoltC_WheelSettings* settings)
{
    if (!settings) return nullptr;
    JOLTC_TRY_BEGIN
    auto* w = new Wheel(*asWS(settings));
    return fromWheel(w);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_Wheel_Destroy(JoltC_Wheel* wheel)
{
    if (!wheel) return;
    JOLTC_TRY_BEGIN
    delete asWheel(wheel);
    JOLTC_TRY_END
}

JOLTC_API JoltC_Wheel* JoltC_WheelWV_Create(const JoltC_WheelSettings* settings)
{
    if (!settings) return nullptr;
    JOLTC_TRY_BEGIN
    auto* w = new WheelWV(*static_cast<const WheelSettingsWV*>(asWS(settings)));
    return fromWheel(static_cast<Wheel*>(w));
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_Wheel* JoltC_WheelTV_Create(const JoltC_WheelSettings* settings)
{
    if (!settings) return nullptr;
    JOLTC_TRY_BEGIN
    auto* w = new WheelTV(*static_cast<const WheelSettingsTV*>(asWS(settings)));
    return fromWheel(static_cast<Wheel*>(w));
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  Phase 4: constraint getters, step callbacks, collision cadence            */
/* ========================================================================== */
JOLTC_API float JoltC_VehicleConstraint_GetMaxPitchRollAngle(const JoltC_VehicleConstraint* vc)
{
    if (!vc) return 0.0f;
    JOLTC_TRY_BEGIN
    return asVC(vc)->GetMaxPitchRollAngle();
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API const JoltC_VehicleCollisionTester* JoltC_VehicleConstraint_GetVehicleCollisionTester(const JoltC_VehicleConstraint* vc)
{
    if (!vc) return nullptr;
    return fromVCT(const_cast<VehicleCollisionTester*>(asVC(vc)->GetVehicleCollisionTester()));
}

/* The std::function captures the C pair, so the capture IS the storage and a null fn clears by
 * assigning an empty function. */
JOLTC_API void JoltC_VehicleConstraint_SetPreStepCallback(JoltC_VehicleConstraint* vc, JoltC_OnVehicleStepFn callback, void* userData)
{
    if (!vc) return;
    JOLTC_TRY_BEGIN
    if (!callback) { asVC(vc)->SetPreStepCallback(VehicleConstraint::StepCallback()); return; }
    asVC(vc)->SetPreStepCallback([callback, userData](VehicleConstraint& constraint, const PhysicsStepListenerContext& context)
    {
        callback(userData, reinterpret_cast<JoltC_VehicleConstraint*>(&constraint), context.mDeltaTime,
                 context.mIsFirstStep ? JOLTC_TRUE : JOLTC_FALSE, context.mIsLastStep ? JOLTC_TRUE : JOLTC_FALSE);
    });
    JOLTC_TRY_END
}

JOLTC_API void JoltC_VehicleConstraint_SetPostCollideCallback(JoltC_VehicleConstraint* vc, JoltC_OnVehicleStepFn callback, void* userData)
{
    if (!vc) return;
    JOLTC_TRY_BEGIN
    if (!callback) { asVC(vc)->SetPostCollideCallback(VehicleConstraint::StepCallback()); return; }
    asVC(vc)->SetPostCollideCallback([callback, userData](VehicleConstraint& constraint, const PhysicsStepListenerContext& context)
    {
        callback(userData, reinterpret_cast<JoltC_VehicleConstraint*>(&constraint), context.mDeltaTime,
                 context.mIsFirstStep ? JOLTC_TRUE : JOLTC_FALSE, context.mIsLastStep ? JOLTC_TRUE : JOLTC_FALSE);
    });
    JOLTC_TRY_END
}

JOLTC_API void JoltC_VehicleConstraint_SetPostStepCallback(JoltC_VehicleConstraint* vc, JoltC_OnVehicleStepFn callback, void* userData)
{
    if (!vc) return;
    JOLTC_TRY_BEGIN
    if (!callback) { asVC(vc)->SetPostStepCallback(VehicleConstraint::StepCallback()); return; }
    asVC(vc)->SetPostStepCallback([callback, userData](VehicleConstraint& constraint, const PhysicsStepListenerContext& context)
    {
        callback(userData, reinterpret_cast<JoltC_VehicleConstraint*>(&constraint), context.mDeltaTime,
                 context.mIsFirstStep ? JOLTC_TRUE : JOLTC_FALSE, context.mIsLastStep ? JOLTC_TRUE : JOLTC_FALSE);
    });
    JOLTC_TRY_END
}

JOLTC_API void JoltC_VehicleConstraint_SetNumStepsBetweenCollisionTestActive(JoltC_VehicleConstraint* vc, uint32_t steps)
{
    if (!vc) return;
    asVC(vc)->SetNumStepsBetweenCollisionTestActive(steps);
}

JOLTC_API uint32_t JoltC_VehicleConstraint_GetNumStepsBetweenCollisionTestActive(const JoltC_VehicleConstraint* vc)
{
    if (!vc) return 0;
    return asVC(vc)->GetNumStepsBetweenCollisionTestActive();
}

JOLTC_API void JoltC_VehicleConstraint_SetNumStepsBetweenCollisionTestInactive(JoltC_VehicleConstraint* vc, uint32_t steps)
{
    if (!vc) return;
    asVC(vc)->SetNumStepsBetweenCollisionTestInactive(steps);
}

JOLTC_API uint32_t JoltC_VehicleConstraint_GetNumStepsBetweenCollisionTestInactive(const JoltC_VehicleConstraint* vc)
{
    if (!vc) return 0;
    return asVC(vc)->GetNumStepsBetweenCollisionTestInactive();
}

/* ========================================================================== */
/*  Phase 4: what the tire is doing against the ground                        */
/* ========================================================================== */
JOLTC_API float JoltC_WheelWV_GetLongitudinalSlip(const JoltC_WheelWV* w)
{
    if (!w) return 0.0f;
    return asWheelWV(w)->mLongitudinalSlip;
}

JOLTC_API float JoltC_WheelWV_GetLateralSlip(const JoltC_WheelWV* w)
{
    if (!w) return 0.0f;
    return asWheelWV(w)->mLateralSlip;
}

JOLTC_API float JoltC_WheelWV_GetCombinedLongitudinalFriction(const JoltC_WheelWV* w)
{
    if (!w) return 0.0f;
    return asWheelWV(w)->mCombinedLongitudinalFriction;
}

JOLTC_API float JoltC_WheelWV_GetCombinedLateralFriction(const JoltC_WheelWV* w)
{
    if (!w) return 0.0f;
    return asWheelWV(w)->mCombinedLateralFriction;
}

JOLTC_API float JoltC_WheelWV_GetBrakeImpulse(const JoltC_WheelWV* w)
{
    if (!w) return 0.0f;
    return asWheelWV(w)->mBrakeImpulse;
}

JOLTC_API float JoltC_WheelTV_GetCombinedLongitudinalFriction(const JoltC_WheelTV* w)
{
    if (!w) return 0.0f;
    return asWheelTV(w)->mCombinedLongitudinalFriction;
}

JOLTC_API float JoltC_WheelTV_GetCombinedLateralFriction(const JoltC_WheelTV* w)
{
    if (!w) return 0.0f;
    return asWheelTV(w)->mCombinedLateralFriction;
}

JOLTC_API float JoltC_WheelTV_GetBrakeImpulse(const JoltC_WheelTV* w)
{
    if (!w) return 0.0f;
    return asWheelTV(w)->mBrakeImpulse;
}

/* ========================================================================== */
/*  Phase 4: the live differentials                                           */
/* ========================================================================== */
JOLTC_API uint32_t JoltC_WheeledVehicleController_GetDifferentialsCount(const JoltC_WheeledVehicleController* c)
{
    if (!c) return 0;
    return (uint32_t)asWVC(c)->GetDifferentials().size();
}

JOLTC_API void JoltC_WheeledVehicleController_GetDifferential(const JoltC_WheeledVehicleController* c, uint32_t index, JoltC_VehicleDifferentialSettings* result)
{
    if (!c || !result || index >= asWVC(c)->GetDifferentials().size()) return;
    fromJphDiffSettings(asWVC(c)->GetDifferentials()[index], result);
}

JOLTC_API void JoltC_WheeledVehicleController_SetDifferential(JoltC_WheeledVehicleController* c, uint32_t index, const JoltC_VehicleDifferentialSettings* value)
{
    if (!c || !value || index >= asWVC(c)->GetDifferentials().size()) return;
    asWVC(c)->GetDifferentials()[index] = toJphDiffSettings(*value);
}

JOLTC_API float JoltC_WheeledVehicleController_GetDifferentialLimitedSlipRatio(const JoltC_WheeledVehicleController* c)
{
    if (!c) return 0.0f;
    return asWVC(c)->GetDifferentialLimitedSlipRatio();
}

JOLTC_API void JoltC_WheeledVehicleController_SetDifferentialLimitedSlipRatio(JoltC_WheeledVehicleController* c, float value)
{
    if (!c) return;
    asWVC(c)->SetDifferentialLimitedSlipRatio(value);
}

} /* extern "C" */
