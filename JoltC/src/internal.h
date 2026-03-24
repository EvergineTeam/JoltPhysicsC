/* JoltC - Internal type conversion helpers
 * SPDX-License-Identifier: MIT
 */

#ifndef JOLTC_INTERNAL_H
#define JOLTC_INTERNAL_H

#include <JoltC/common.h>
#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Math/Vec4.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Mat44.h>
#include <Jolt/Math/Real.h>
#include <Jolt/Physics/EActivation.h>
#include <Jolt/Physics/Body/MotionType.h>
#include <Jolt/Physics/Body/MotionQuality.h>
#include <Jolt/Physics/Body/BodyType.h>
#include <Jolt/Physics/Body/AllowedDOFs.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Constraints/Constraint.h>
#include <Jolt/Physics/Constraints/MotorSettings.h>
#include <Jolt/Physics/Constraints/SpringSettings.h>
#include <Jolt/Physics/Constraints/ConstraintPart/SwingTwistConstraintPart.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/BackFaceMode.h>

using namespace JPH;

/* -------------------------------------------------------------------------- */
/*  Vec3                                                                      */
/* -------------------------------------------------------------------------- */
static inline Vec3 toJphVec3(JoltC_Vec3 v) { return Vec3(v.x, v.y, v.z); }
static inline JoltC_Vec3 fromJphVec3(Vec3 v) { return JoltC_Vec3{v.GetX(), v.GetY(), v.GetZ()}; }

/* -------------------------------------------------------------------------- */
/*  RVec3                                                                     */
/* -------------------------------------------------------------------------- */
static inline RVec3 toJphRVec3(JoltC_RVec3 v) {
    return RVec3(Real(v.x), Real(v.y), Real(v.z));
}
static inline JoltC_RVec3 fromJphRVec3(RVec3 v) {
    JoltC_RVec3 r;
#ifdef JOLTC_DOUBLE_PRECISION
    r.x = v.GetX(); r.y = v.GetY(); r.z = v.GetZ();
#else
    r.x = (float)v.GetX(); r.y = (float)v.GetY(); r.z = (float)v.GetZ();
#endif
    return r;
}

/* -------------------------------------------------------------------------- */
/*  Quat                                                                      */
/* -------------------------------------------------------------------------- */
static inline Quat toJphQuat(JoltC_Quat q) { return Quat(q.x, q.y, q.z, q.w); }
static inline JoltC_Quat fromJphQuat(Quat q) { return JoltC_Quat{q.GetX(), q.GetY(), q.GetZ(), q.GetW()}; }

/* -------------------------------------------------------------------------- */
/*  Enums                                                                     */
/* -------------------------------------------------------------------------- */
static inline EMotionType toJphMotionType(JoltC_MotionType t) { return static_cast<EMotionType>(t); }
static inline JoltC_MotionType fromJphMotionType(EMotionType t) { return static_cast<JoltC_MotionType>(t); }

static inline EMotionQuality toJphMotionQuality(JoltC_MotionQuality q) { return static_cast<EMotionQuality>(q); }
static inline EActivation toJphActivation(JoltC_Activation a) { return static_cast<EActivation>(a); }

static inline EBodyType toJphBodyType(JoltC_BodyType t) { return static_cast<EBodyType>(t); }
static inline JoltC_BodyType fromJphBodyType(EBodyType t) { return static_cast<JoltC_BodyType>(t); }

static inline EAllowedDOFs toJphAllowedDOFs(JoltC_AllowedDOFs d) { return static_cast<EAllowedDOFs>(d); }

/* -------------------------------------------------------------------------- */
/*  BodyID                                                                    */
/* -------------------------------------------------------------------------- */
static inline BodyID toJphBodyID(JoltC_BodyID id) { return BodyID(id); }
static inline JoltC_BodyID fromJphBodyID(BodyID id) { return id.GetIndexAndSequenceNumber(); }

/* -------------------------------------------------------------------------- */
/*  AABox                                                                     */
/* -------------------------------------------------------------------------- */
static inline JoltC_AABox fromJphAABox(const AABox& b) {
    return JoltC_AABox{fromJphVec3(Vec3(b.mMin)), fromJphVec3(Vec3(b.mMax))};
}

/* -------------------------------------------------------------------------- */
/*  Mat44                                                                     */
/* -------------------------------------------------------------------------- */
static inline JoltC_Mat44 fromJphMat44(Mat44 m) {
    JoltC_Mat44 r;
    m.StoreFloat4x4((Float4*)r.m);
    return r;
}
static inline Mat44 toJphMat44(const JoltC_Mat44& m) {
    return Mat44::sLoadFloat4x4(reinterpret_cast<const Float4*>(m.m));
}

/* -------------------------------------------------------------------------- */
/*  ConstraintSpace                                                           */
/* -------------------------------------------------------------------------- */
static inline EConstraintSpace toJphConstraintSpace(JoltC_ConstraintSpace s) { return static_cast<EConstraintSpace>(s); }

/* -------------------------------------------------------------------------- */
/*  MotorState                                                                */
/* -------------------------------------------------------------------------- */
static inline EMotorState toJphMotorState(JoltC_MotorState s) { return static_cast<EMotorState>(s); }
static inline JoltC_MotorState fromJphMotorState(EMotorState s) { return static_cast<JoltC_MotorState>(s); }

/* -------------------------------------------------------------------------- */
/*  SpringSettings                                                            */
/* -------------------------------------------------------------------------- */
static inline SpringSettings toJphSpringSettings(JoltC_SpringSettings s) {
    return SpringSettings(static_cast<ESpringMode>(s.mode), s.frequencyOrStiffness, s.damping);
}
static inline JoltC_SpringSettings fromJphSpringSettings(const SpringSettings& s) {
    return JoltC_SpringSettings{static_cast<JoltC_SpringMode>(s.mMode), s.mFrequency, s.mDamping};
}

/* -------------------------------------------------------------------------- */
/*  MotorSettings                                                             */
/* -------------------------------------------------------------------------- */
static inline MotorSettings toJphMotorSettings(JoltC_MotorSettings m) {
    MotorSettings r;
    r.mSpringSettings = toJphSpringSettings(m.springSettings);
    r.mMinForceLimit = m.minForceLimit;
    r.mMaxForceLimit = m.maxForceLimit;
    r.mMinTorqueLimit = m.minTorqueLimit;
    r.mMaxTorqueLimit = m.maxTorqueLimit;
    return r;
}

/* -------------------------------------------------------------------------- */
/*  SwingType                                                                 */
/* -------------------------------------------------------------------------- */
static inline ESwingType toJphSwingType(JoltC_SwingType t) { return static_cast<ESwingType>(t); }

/* -------------------------------------------------------------------------- */
/*  BackFaceMode                                                              */
/* -------------------------------------------------------------------------- */
static inline EBackFaceMode toJphBackFaceMode(JoltC_BackFaceMode m) { return static_cast<EBackFaceMode>(m); }

/* -------------------------------------------------------------------------- */
/*  RayCastResult                                                             */
/* -------------------------------------------------------------------------- */
static inline JoltC_RayCastResult fromJphRayCastResult(const RayCastResult& r) {
    JoltC_RayCastResult result;
    result.bodyID = fromJphBodyID(r.mBodyID);
    result.fraction = r.mFraction;
    result.subShapeID2 = r.mSubShapeID2.GetValue();
    return result;
}

/* -------------------------------------------------------------------------- */
/*  MotionQuality (from Jph)                                                  */
/* -------------------------------------------------------------------------- */
static inline JoltC_MotionQuality fromJphMotionQuality(EMotionQuality q) { return static_cast<JoltC_MotionQuality>(q); }

#endif /* JOLTC_INTERNAL_H */
