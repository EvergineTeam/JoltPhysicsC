/* JoltC - Body direct access, BodyLock, MotionProperties implementations
 * SPDX-License-Identifier: MIT
 */

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Body/MotionProperties.h>
#include <Jolt/Physics/Body/MassProperties.h>

#include <JoltC/body_access.h>
#include "internal.h"
#include "wrappers.h"
#include "errors_internal.h"

#include <vector>

using namespace JPH;

/* -------------------------------------------------------------------------- */
/*  Casting helpers: JoltC_Body ↔ JPH::Body                                   */
/* -------------------------------------------------------------------------- */
static inline const Body* asBody(const JoltC_Body* b) {
    return reinterpret_cast<const Body*>(b);
}
static inline Body* asBodyMut(JoltC_Body* b) {
    return reinterpret_cast<Body*>(b);
}
static inline const JoltC_Body* fromBody(const Body* b) {
    return reinterpret_cast<const JoltC_Body*>(b);
}
static inline JoltC_Body* fromBodyMut(Body* b) {
    return reinterpret_cast<JoltC_Body*>(b);
}

/* -------------------------------------------------------------------------- */
/*  MotionProperties casting                                                  */
/* -------------------------------------------------------------------------- */
static inline const MotionProperties* asMP(const JoltC_MotionProperties* p) {
    return reinterpret_cast<const MotionProperties*>(p);
}
static inline MotionProperties* asMPMut(JoltC_MotionProperties* p) {
    return reinterpret_cast<MotionProperties*>(p);
}

/* -------------------------------------------------------------------------- */
/*  RMat44 to JoltC_Mat44 helper (works both single and double precision)     */
/* -------------------------------------------------------------------------- */
static inline JoltC_Mat44 fromJphRMat44(RMat44 m) {
    return fromJphMat44(m.ToMat44());
}

/* -------------------------------------------------------------------------- */
/*  BodyLock wrappers (heap-allocated so they survive across C calls)          */
/* -------------------------------------------------------------------------- */
struct JoltC_BodyLockRead {
    BodyLockRead lock;
    JoltC_BodyLockRead(const BodyLockInterface& iface, const BodyID& id) : lock(iface, id) {}
};

struct JoltC_BodyLockWrite {
    BodyLockWrite lock;
    JoltC_BodyLockWrite(const BodyLockInterface& iface, const BodyID& id) : lock(iface, id) {}
};

extern "C" {

/* -------------------------------------------------------------------------- */
/*  BodyLockInterface access from PhysicsSystem                               */
/* -------------------------------------------------------------------------- */
JOLTC_API const JoltC_BodyLockInterface* JoltC_PhysicsSystem_GetBodyLockInterface(const JoltC_PhysicsSystem* system) {
    if (!system) return nullptr;
    return reinterpret_cast<const JoltC_BodyLockInterface*>(&system->ptr->GetBodyLockInterface());
}

JOLTC_API const JoltC_BodyLockInterface* JoltC_PhysicsSystem_GetBodyLockInterfaceNoLock(const JoltC_PhysicsSystem* system) {
    if (!system) return nullptr;
    return reinterpret_cast<const JoltC_BodyLockInterface*>(&system->ptr->GetBodyLockInterfaceNoLock());
}

/* -------------------------------------------------------------------------- */
/*  BodyLock Read                                                             */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_BodyLockRead* JoltC_BodyLockRead_Create(const JoltC_BodyLockInterface* lockInterface, JoltC_BodyID bodyID) {
    if (!lockInterface) return nullptr;
    JOLTC_TRY_BEGIN
    const auto* iface = reinterpret_cast<const BodyLockInterface*>(lockInterface);
    return new JoltC_BodyLockRead(*iface, BodyID(bodyID));
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API const JoltC_Body* JoltC_BodyLockRead_GetBody(const JoltC_BodyLockRead* lock) {
    if (!lock) return nullptr;
    if (!lock->lock.Succeeded()) return nullptr;
    return fromBody(&lock->lock.GetBody());
}

JOLTC_API void JoltC_BodyLockRead_Destroy(JoltC_BodyLockRead* lock) {
    delete lock;
}

/* -------------------------------------------------------------------------- */
/*  BodyLock Write                                                            */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_BodyLockWrite* JoltC_BodyLockWrite_Create(const JoltC_BodyLockInterface* lockInterface, JoltC_BodyID bodyID) {
    if (!lockInterface) return nullptr;
    JOLTC_TRY_BEGIN
    const auto* iface = reinterpret_cast<const BodyLockInterface*>(lockInterface);
    return new JoltC_BodyLockWrite(*iface, BodyID(bodyID));
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_Body* JoltC_BodyLockWrite_GetBody(const JoltC_BodyLockWrite* lock) {
    if (!lock) return nullptr;
    if (!lock->lock.Succeeded()) return nullptr;
    return fromBodyMut(&lock->lock.GetBody());
}

JOLTC_API void JoltC_BodyLockWrite_Destroy(JoltC_BodyLockWrite* lock) {
    delete lock;
}

/* -------------------------------------------------------------------------- */
/*  BodyLockInterface — Lock / Unlock (alternative naming)                    */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_BodyLockRead* JoltC_BodyLockInterface_LockRead(const JoltC_BodyLockInterface* lockInterface, uint32_t bodyId) {
    if (!lockInterface) return nullptr;
    JOLTC_TRY_BEGIN
    const auto* iface = reinterpret_cast<const BodyLockInterface*>(lockInterface);
    return new JoltC_BodyLockRead(*iface, BodyID(bodyId));
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_BodyLockInterface_UnlockRead(const JoltC_BodyLockInterface* lockInterface, JoltC_BodyLockRead* lock) {
    (void)lockInterface;
    delete lock;
}

JOLTC_API JoltC_BodyLockWrite* JoltC_BodyLockInterface_LockWrite(const JoltC_BodyLockInterface* lockInterface, uint32_t bodyId) {
    if (!lockInterface) return nullptr;
    JOLTC_TRY_BEGIN
    const auto* iface = reinterpret_cast<const BodyLockInterface*>(lockInterface);
    return new JoltC_BodyLockWrite(*iface, BodyID(bodyId));
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_BodyLockInterface_UnlockWrite(const JoltC_BodyLockInterface* lockInterface, JoltC_BodyLockWrite* lock) {
    (void)lockInterface;
    delete lock;
}

/* -------------------------------------------------------------------------- */
/*  BodyLockMulti Read / Write                                                */
/* -------------------------------------------------------------------------- */
struct JoltC_BodyLockMultiRead {
    std::vector<BodyID> bodyIds;
    BodyLockMultiRead* lock;
    ~JoltC_BodyLockMultiRead() { delete lock; }
};

struct JoltC_BodyLockMultiWrite {
    std::vector<BodyID> bodyIds;
    BodyLockMultiWrite* lock;
    ~JoltC_BodyLockMultiWrite() { delete lock; }
};

JOLTC_API JoltC_BodyLockMultiRead* JoltC_BodyLockInterface_LockMultiRead(const JoltC_BodyLockInterface* lockInterface, const uint32_t* bodyIds, int count) {
    if (!lockInterface || !bodyIds || count <= 0) return nullptr;
    JOLTC_TRY_BEGIN
    const auto* iface = reinterpret_cast<const BodyLockInterface*>(lockInterface);
    auto* wrapper = new JoltC_BodyLockMultiRead;
    wrapper->bodyIds.resize(count);
    for (int i = 0; i < count; i++)
        wrapper->bodyIds[i] = BodyID(bodyIds[i]);
    wrapper->lock = new BodyLockMultiRead(*iface, wrapper->bodyIds.data(), count);
    return wrapper;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_BodyLockMultiWrite* JoltC_BodyLockInterface_LockMultiWrite(const JoltC_BodyLockInterface* lockInterface, const uint32_t* bodyIds, int count) {
    if (!lockInterface || !bodyIds || count <= 0) return nullptr;
    JOLTC_TRY_BEGIN
    const auto* iface = reinterpret_cast<const BodyLockInterface*>(lockInterface);
    auto* wrapper = new JoltC_BodyLockMultiWrite;
    wrapper->bodyIds.resize(count);
    for (int i = 0; i < count; i++)
        wrapper->bodyIds[i] = BodyID(bodyIds[i]);
    wrapper->lock = new BodyLockMultiWrite(*iface, wrapper->bodyIds.data(), count);
    return wrapper;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_BodyLockMultiRead_Destroy(JoltC_BodyLockMultiRead* lock) {
    delete lock;
}

JOLTC_API void JoltC_BodyLockMultiWrite_Destroy(JoltC_BodyLockMultiWrite* lock) {
    delete lock;
}

JOLTC_API JoltC_Body* JoltC_BodyLockMultiWrite_GetBody(JoltC_BodyLockMultiWrite* lock, int index) {
    if (!lock || !lock->lock) return nullptr;
    JOLTC_TRY_BEGIN
    Body* body = lock->lock->GetBody(index);
    return reinterpret_cast<JoltC_Body*>(body);
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  Body — read-only queries                                                  */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_BodyID JoltC_Body_GetID(const JoltC_Body* body) {
    if (!body) return JOLTC_BODY_ID_INVALID;
    return fromJphBodyID(asBody(body)->GetID());
}

JOLTC_API JoltC_BodyType JoltC_Body_GetBodyType(const JoltC_Body* body) {
    if (!body) return JOLTC_BODY_TYPE_RIGID;
    return fromJphBodyType(asBody(body)->GetBodyType());
}

JOLTC_API int JoltC_Body_IsRigidBody(const JoltC_Body* body) {
    if (!body) return 0;
    return asBody(body)->IsRigidBody() ? 1 : 0;
}

JOLTC_API int JoltC_Body_IsSoftBody(const JoltC_Body* body) {
    if (!body) return 0;
    return asBody(body)->IsSoftBody() ? 1 : 0;
}

JOLTC_API int JoltC_Body_IsActive(const JoltC_Body* body) {
    if (!body) return 0;
    return asBody(body)->IsActive() ? 1 : 0;
}

JOLTC_API int JoltC_Body_IsStatic(const JoltC_Body* body) {
    if (!body) return 0;
    return asBody(body)->IsStatic() ? 1 : 0;
}

JOLTC_API int JoltC_Body_IsKinematic(const JoltC_Body* body) {
    if (!body) return 0;
    return asBody(body)->IsKinematic() ? 1 : 0;
}

JOLTC_API int JoltC_Body_IsDynamic(const JoltC_Body* body) {
    if (!body) return 0;
    return asBody(body)->IsDynamic() ? 1 : 0;
}

JOLTC_API int JoltC_Body_CanBeKinematicOrDynamic(const JoltC_Body* body) {
    if (!body) return 0;
    return asBody(body)->CanBeKinematicOrDynamic() ? 1 : 0;
}

JOLTC_API int JoltC_Body_IsSensor(const JoltC_Body* body) {
    if (!body) return 0;
    return asBody(body)->IsSensor() ? 1 : 0;
}

JOLTC_API void JoltC_Body_SetIsSensor(JoltC_Body* body, int value) {
    if (!body) return;
    asBodyMut(body)->SetIsSensor(value != 0);
}

JOLTC_API int JoltC_Body_GetCollideKinematicVsNonDynamic(const JoltC_Body* body) {
    if (!body) return 0;
    return asBody(body)->GetCollideKinematicVsNonDynamic() ? 1 : 0;
}

JOLTC_API void JoltC_Body_SetCollideKinematicVsNonDynamic(JoltC_Body* body, int value) {
    if (!body) return;
    asBodyMut(body)->SetCollideKinematicVsNonDynamic(value != 0);
}

JOLTC_API int JoltC_Body_GetUseManifoldReduction(const JoltC_Body* body) {
    if (!body) return 0;
    return asBody(body)->GetUseManifoldReduction() ? 1 : 0;
}

JOLTC_API void JoltC_Body_SetUseManifoldReduction(JoltC_Body* body, int value) {
    if (!body) return;
    asBodyMut(body)->SetUseManifoldReduction(value != 0);
}

JOLTC_API int JoltC_Body_GetUseManifoldReductionWithBody(const JoltC_Body* body, const JoltC_Body* other) {
    if (!body || !other) return 0;
    return asBody(body)->GetUseManifoldReductionWithBody(*asBody(other)) ? 1 : 0;
}

JOLTC_API int JoltC_Body_GetApplyGyroscopicForce(const JoltC_Body* body) {
    if (!body) return 0;
    return asBody(body)->GetApplyGyroscopicForce() ? 1 : 0;
}

JOLTC_API void JoltC_Body_SetApplyGyroscopicForce(JoltC_Body* body, int value) {
    if (!body) return;
    asBodyMut(body)->SetApplyGyroscopicForce(value != 0);
}

JOLTC_API int JoltC_Body_GetEnhancedInternalEdgeRemoval(const JoltC_Body* body) {
    if (!body) return 0;
    return asBody(body)->GetEnhancedInternalEdgeRemoval() ? 1 : 0;
}

JOLTC_API void JoltC_Body_SetEnhancedInternalEdgeRemoval(JoltC_Body* body, int value) {
    if (!body) return;
    asBodyMut(body)->SetEnhancedInternalEdgeRemoval(value != 0);
}

JOLTC_API int JoltC_Body_GetEnhancedInternalEdgeRemovalWithBody(const JoltC_Body* body, const JoltC_Body* other) {
    if (!body || !other) return 0;
    return asBody(body)->GetEnhancedInternalEdgeRemovalWithBody(*asBody(other)) ? 1 : 0;
}

JOLTC_API JoltC_MotionType JoltC_Body_GetMotionType(const JoltC_Body* body) {
    if (!body) return JOLTC_MOTION_TYPE_STATIC;
    return fromJphMotionType(asBody(body)->GetMotionType());
}

JOLTC_API void JoltC_Body_SetMotionType(JoltC_Body* body, JoltC_MotionType motionType) {
    if (!body) return;
    asBodyMut(body)->SetMotionType(toJphMotionType(motionType));
}

JOLTC_API JoltC_BroadPhaseLayer JoltC_Body_GetBroadPhaseLayer(const JoltC_Body* body) {
    if (!body) return 0;
    return static_cast<JoltC_BroadPhaseLayer>(asBody(body)->GetBroadPhaseLayer().GetValue());
}

JOLTC_API JoltC_ObjectLayer JoltC_Body_GetObjectLayer(const JoltC_Body* body) {
    if (!body) return JOLTC_OBJECT_LAYER_INVALID;
    return asBody(body)->GetObjectLayer();
}

JOLTC_API int JoltC_Body_GetAllowSleeping(const JoltC_Body* body) {
    if (!body) return 0;
    return asBody(body)->GetAllowSleeping() ? 1 : 0;
}

JOLTC_API void JoltC_Body_SetAllowSleeping(JoltC_Body* body, int allowSleeping) {
    if (!body) return;
    asBodyMut(body)->SetAllowSleeping(allowSleeping != 0);
}

JOLTC_API void JoltC_Body_ResetSleepTimer(JoltC_Body* body) {
    if (!body) return;
    asBodyMut(body)->ResetSleepTimer();
}

JOLTC_API float JoltC_Body_GetFriction(const JoltC_Body* body) {
    if (!body) return 0.0f;
    return asBody(body)->GetFriction();
}

JOLTC_API void JoltC_Body_SetFriction(JoltC_Body* body, float friction) {
    if (!body) return;
    asBodyMut(body)->SetFriction(friction);
}

JOLTC_API float JoltC_Body_GetRestitution(const JoltC_Body* body) {
    if (!body) return 0.0f;
    return asBody(body)->GetRestitution();
}

JOLTC_API void JoltC_Body_SetRestitution(JoltC_Body* body, float restitution) {
    if (!body) return;
    asBodyMut(body)->SetRestitution(restitution);
}

JOLTC_API void JoltC_Body_GetLinearVelocity(const JoltC_Body* body, JoltC_Vec3* result) {
    if (!body || !result) return;
    *result = fromJphVec3(asBody(body)->GetLinearVelocity());
}

JOLTC_API void JoltC_Body_SetLinearVelocity(JoltC_Body* body, const JoltC_Vec3* velocity) {
    if (!body || !velocity) return;
    asBodyMut(body)->SetLinearVelocity(toJphVec3(*velocity));
}

JOLTC_API void JoltC_Body_SetLinearVelocityClamped(JoltC_Body* body, const JoltC_Vec3* velocity) {
    if (!body || !velocity) return;
    asBodyMut(body)->SetLinearVelocityClamped(toJphVec3(*velocity));
}

JOLTC_API void JoltC_Body_GetAngularVelocity(const JoltC_Body* body, JoltC_Vec3* result) {
    if (!body || !result) return;
    *result = fromJphVec3(asBody(body)->GetAngularVelocity());
}

JOLTC_API void JoltC_Body_SetAngularVelocity(JoltC_Body* body, const JoltC_Vec3* velocity) {
    if (!body || !velocity) return;
    asBodyMut(body)->SetAngularVelocity(toJphVec3(*velocity));
}

JOLTC_API void JoltC_Body_SetAngularVelocityClamped(JoltC_Body* body, const JoltC_Vec3* velocity) {
    if (!body || !velocity) return;
    asBodyMut(body)->SetAngularVelocityClamped(toJphVec3(*velocity));
}

JOLTC_API void JoltC_Body_GetPointVelocityCOM(const JoltC_Body* body, const JoltC_Vec3* pointRelativeToCOM, JoltC_Vec3* result) {
    if (!body || !pointRelativeToCOM || !result) return;
    *result = fromJphVec3(asBody(body)->GetPointVelocityCOM(toJphVec3(*pointRelativeToCOM)));
}

JOLTC_API void JoltC_Body_GetPointVelocity(const JoltC_Body* body, const JoltC_RVec3* point, JoltC_Vec3* result) {
    if (!body || !point || !result) return;
    *result = fromJphVec3(asBody(body)->GetPointVelocity(toJphRVec3(*point)));
}

JOLTC_API void JoltC_Body_AddForce(JoltC_Body* body, const JoltC_Vec3* force) {
    if (!body || !force) return;
    asBodyMut(body)->AddForce(toJphVec3(*force));
}

JOLTC_API void JoltC_Body_AddForceAtPosition(JoltC_Body* body, const JoltC_Vec3* force, const JoltC_RVec3* position) {
    if (!body || !force || !position) return;
    asBodyMut(body)->AddForce(toJphVec3(*force), toJphRVec3(*position));
}

JOLTC_API void JoltC_Body_AddTorque(JoltC_Body* body, const JoltC_Vec3* torque) {
    if (!body || !torque) return;
    asBodyMut(body)->AddTorque(toJphVec3(*torque));
}

JOLTC_API void JoltC_Body_GetAccumulatedForce(const JoltC_Body* body, JoltC_Vec3* result) {
    if (!body || !result) return;
    *result = fromJphVec3(asBody(body)->GetAccumulatedForce());
}

JOLTC_API void JoltC_Body_GetAccumulatedTorque(const JoltC_Body* body, JoltC_Vec3* result) {
    if (!body || !result) return;
    *result = fromJphVec3(asBody(body)->GetAccumulatedTorque());
}

JOLTC_API void JoltC_Body_ResetForce(JoltC_Body* body) {
    if (!body) return;
    asBodyMut(body)->ResetForce();
}

JOLTC_API void JoltC_Body_ResetTorque(JoltC_Body* body) {
    if (!body) return;
    asBodyMut(body)->ResetTorque();
}

JOLTC_API void JoltC_Body_ResetMotion(JoltC_Body* body) {
    if (!body) return;
    asBodyMut(body)->ResetMotion();
}

JOLTC_API void JoltC_Body_GetInverseInertia(const JoltC_Body* body, JoltC_Mat44* result) {
    if (!body || !result) return;
    *result = fromJphMat44(asBody(body)->GetInverseInertia());
}

JOLTC_API void JoltC_Body_AddImpulse(JoltC_Body* body, const JoltC_Vec3* impulse) {
    if (!body || !impulse) return;
    asBodyMut(body)->AddImpulse(toJphVec3(*impulse));
}

JOLTC_API void JoltC_Body_AddImpulseAtPosition(JoltC_Body* body, const JoltC_Vec3* impulse, const JoltC_RVec3* position) {
    if (!body || !impulse || !position) return;
    asBodyMut(body)->AddImpulse(toJphVec3(*impulse), toJphRVec3(*position));
}

JOLTC_API void JoltC_Body_AddAngularImpulse(JoltC_Body* body, const JoltC_Vec3* angularImpulse) {
    if (!body || !angularImpulse) return;
    asBodyMut(body)->AddAngularImpulse(toJphVec3(*angularImpulse));
}

JOLTC_API void JoltC_Body_MoveKinematic(JoltC_Body* body, const JoltC_RVec3* targetPosition, const JoltC_Quat* targetRotation, float deltaTime) {
    if (!body || !targetPosition || !targetRotation) return;
    asBodyMut(body)->MoveKinematic(toJphRVec3(*targetPosition), toJphQuat(*targetRotation), deltaTime);
}

JOLTC_API int JoltC_Body_ApplyBuoyancyImpulse(JoltC_Body* body, const JoltC_RVec3* surfacePosition, const JoltC_Vec3* surfaceNormal, float buoyancy, float linearDrag, float angularDrag, const JoltC_Vec3* fluidVelocity, const JoltC_Vec3* gravity, float deltaTime) {
    if (!body || !surfacePosition || !surfaceNormal || !fluidVelocity || !gravity) return 0;
    return asBodyMut(body)->ApplyBuoyancyImpulse(
        toJphRVec3(*surfacePosition), toJphVec3(*surfaceNormal),
        buoyancy, linearDrag, angularDrag,
        toJphVec3(*fluidVelocity), toJphVec3(*gravity), deltaTime) ? 1 : 0;
}

JOLTC_API int JoltC_Body_IsInBroadPhase(const JoltC_Body* body) {
    if (!body) return 0;
    return asBody(body)->IsInBroadPhase() ? 1 : 0;
}

JOLTC_API int JoltC_Body_IsCollisionCacheInvalid(const JoltC_Body* body) {
    if (!body) return 0;
    return asBody(body)->IsCollisionCacheInvalid() ? 1 : 0;
}

JOLTC_API const JoltC_Shape* JoltC_Body_GetShape(const JoltC_Body* body) {
    if (!body) return nullptr;
    return reinterpret_cast<const JoltC_Shape*>(asBody(body)->GetShape());
}

JOLTC_API void JoltC_Body_GetPosition(const JoltC_Body* body, JoltC_RVec3* result) {
    if (!body || !result) return;
    *result = fromJphRVec3(asBody(body)->GetPosition());
}

JOLTC_API void JoltC_Body_GetRotation(const JoltC_Body* body, JoltC_Quat* result) {
    if (!body || !result) return;
    *result = fromJphQuat(asBody(body)->GetRotation());
}

JOLTC_API void JoltC_Body_GetWorldTransform(const JoltC_Body* body, JoltC_Mat44* result) {
    if (!body || !result) return;
    *result = fromJphRMat44(asBody(body)->GetWorldTransform());
}

JOLTC_API void JoltC_Body_GetCenterOfMassPosition(const JoltC_Body* body, JoltC_RVec3* result) {
    if (!body || !result) return;
    *result = fromJphRVec3(asBody(body)->GetCenterOfMassPosition());
}

JOLTC_API void JoltC_Body_GetCenterOfMassTransform(const JoltC_Body* body, JoltC_Mat44* result) {
    if (!body || !result) return;
    *result = fromJphRMat44(asBody(body)->GetCenterOfMassTransform());
}

JOLTC_API void JoltC_Body_GetInverseCenterOfMassTransform(const JoltC_Body* body, JoltC_Mat44* result) {
    if (!body || !result) return;
    *result = fromJphRMat44(asBody(body)->GetInverseCenterOfMassTransform());
}

JOLTC_API void JoltC_Body_GetWorldSpaceBounds(const JoltC_Body* body, JoltC_AABox* result) {
    if (!body || !result) return;
    *result = fromJphAABox(asBody(body)->GetWorldSpaceBounds());
}

JOLTC_API void JoltC_Body_GetWorldSpaceSurfaceNormal(const JoltC_Body* body, JoltC_SubShapeID subShapeID, const JoltC_RVec3* position, JoltC_Vec3* result) {
    if (!body || !position || !result) return;
    SubShapeID ssid;
    ssid.SetValue(subShapeID);
    *result = fromJphVec3(asBody(body)->GetWorldSpaceSurfaceNormal(ssid, toJphRVec3(*position)));
}

JOLTC_API JoltC_MotionProperties* JoltC_Body_GetMotionProperties(JoltC_Body* body) {
    if (!body) return nullptr;
    return reinterpret_cast<JoltC_MotionProperties*>(asBodyMut(body)->GetMotionProperties());
}

JOLTC_API JoltC_MotionProperties* JoltC_Body_GetMotionPropertiesUnchecked(JoltC_Body* body) {
    if (!body) return nullptr;
    return reinterpret_cast<JoltC_MotionProperties*>(asBodyMut(body)->GetMotionPropertiesUnchecked());
}

JOLTC_API void JoltC_Body_SetUserData(JoltC_Body* body, uint64_t userData) {
    if (!body) return;
    asBodyMut(body)->SetUserData(userData);
}

JOLTC_API uint64_t JoltC_Body_GetUserData(const JoltC_Body* body) {
    if (!body) return 0;
    return asBody(body)->GetUserData();
}

/* -------------------------------------------------------------------------- */
/*  MotionProperties                                                          */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_AllowedDOFs JoltC_MotionProperties_GetAllowedDOFs(const JoltC_MotionProperties* properties) {
    if (!properties) return JOLTC_ALLOWED_DOFS_ALL;
    return static_cast<JoltC_AllowedDOFs>(asMP(properties)->GetAllowedDOFs());
}

JOLTC_API void JoltC_MotionProperties_SetLinearDamping(JoltC_MotionProperties* properties, float damping) {
    if (!properties) return;
    asMPMut(properties)->SetLinearDamping(damping);
}

JOLTC_API float JoltC_MotionProperties_GetLinearDamping(const JoltC_MotionProperties* properties) {
    if (!properties) return 0.0f;
    return asMP(properties)->GetLinearDamping();
}

JOLTC_API void JoltC_MotionProperties_SetAngularDamping(JoltC_MotionProperties* properties, float damping) {
    if (!properties) return;
    asMPMut(properties)->SetAngularDamping(damping);
}

JOLTC_API float JoltC_MotionProperties_GetAngularDamping(const JoltC_MotionProperties* properties) {
    if (!properties) return 0.0f;
    return asMP(properties)->GetAngularDamping();
}

JOLTC_API float JoltC_MotionProperties_GetInverseMassUnchecked(const JoltC_MotionProperties* properties) {
    if (!properties) return 0.0f;
    return asMP(properties)->GetInverseMassUnchecked();
}

JOLTC_API void JoltC_MotionProperties_SetInverseMass(JoltC_MotionProperties* properties, float inverseMass) {
    if (!properties) return;
    asMPMut(properties)->SetInverseMass(inverseMass);
}

JOLTC_API void JoltC_MotionProperties_GetInverseInertiaDiagonal(const JoltC_MotionProperties* properties, JoltC_Vec3* result) {
    if (!properties || !result) return;
    *result = fromJphVec3(asMP(properties)->GetInverseInertiaDiagonal());
}

JOLTC_API void JoltC_MotionProperties_GetInertiaRotation(const JoltC_MotionProperties* properties, JoltC_Quat* result) {
    if (!properties || !result) return;
    *result = fromJphQuat(asMP(properties)->GetInertiaRotation());
}

JOLTC_API void JoltC_MotionProperties_SetInverseInertia(JoltC_MotionProperties* properties, const JoltC_Vec3* diagonal, const JoltC_Quat* rotation) {
    if (!properties || !diagonal || !rotation) return;
    asMPMut(properties)->SetInverseInertia(toJphVec3(*diagonal), toJphQuat(*rotation));
}

JOLTC_API void JoltC_MotionProperties_SetMassProperties(JoltC_MotionProperties* properties, JoltC_AllowedDOFs allowedDOFs, const JoltC_MassProperties* massProperties) {
    if (!properties || !massProperties) return;
    JOLTC_TRY_BEGIN
    MassProperties mp;
    mp.mMass = massProperties->mass;
    mp.mInertia = Mat44::sLoadFloat4x4(reinterpret_cast<const Float4*>(massProperties->inertia.m));
    asMPMut(properties)->SetMassProperties(static_cast<EAllowedDOFs>(allowedDOFs), mp);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_MotionProperties_ScaleToMass(JoltC_MotionProperties* properties, float mass) {
    if (!properties) return;
    asMPMut(properties)->ScaleToMass(mass);
}

/* ========================================================================== */
/*  Phase 5: the rest of MotionProperties                                     */
/* ========================================================================== */
JOLTC_API JoltC_MotionQuality JoltC_MotionProperties_GetMotionQuality(const JoltC_MotionProperties* properties) {
    if (!properties) return (JoltC_MotionQuality)0;
    return static_cast<JoltC_MotionQuality>(asMP(properties)->GetMotionQuality());
}

JOLTC_API JoltC_Bool JoltC_MotionProperties_GetAllowSleeping(const JoltC_MotionProperties* properties) {
    if (!properties) return JOLTC_FALSE;
    return asMP(properties)->GetAllowSleeping() ? JOLTC_TRUE : JOLTC_FALSE;
}

JOLTC_API float JoltC_MotionProperties_GetInverseMass(const JoltC_MotionProperties* properties) {
    if (!properties) return 0.0f;
    return asMP(properties)->GetInverseMass();
}

JOLTC_API JoltC_Vec3 JoltC_MotionProperties_GetLinearVelocity(const JoltC_MotionProperties* properties) {
    if (!properties) return JoltC_Vec3{0, 0, 0};
    return fromJphVec3(asMP(properties)->GetLinearVelocity());
}

JOLTC_API void JoltC_MotionProperties_SetLinearVelocity(JoltC_MotionProperties* properties, JoltC_Vec3 velocity) {
    if (!properties) return;
    asMPMut(properties)->SetLinearVelocity(toJphVec3(velocity));
}

JOLTC_API void JoltC_MotionProperties_SetLinearVelocityClamped(JoltC_MotionProperties* properties, JoltC_Vec3 velocity) {
    if (!properties) return;
    asMPMut(properties)->SetLinearVelocityClamped(toJphVec3(velocity));
}

JOLTC_API JoltC_Vec3 JoltC_MotionProperties_GetAngularVelocity(const JoltC_MotionProperties* properties) {
    if (!properties) return JoltC_Vec3{0, 0, 0};
    return fromJphVec3(asMP(properties)->GetAngularVelocity());
}

JOLTC_API void JoltC_MotionProperties_SetAngularVelocity(JoltC_MotionProperties* properties, JoltC_Vec3 velocity) {
    if (!properties) return;
    asMPMut(properties)->SetAngularVelocity(toJphVec3(velocity));
}

JOLTC_API void JoltC_MotionProperties_SetAngularVelocityClamped(JoltC_MotionProperties* properties, JoltC_Vec3 velocity) {
    if (!properties) return;
    asMPMut(properties)->SetAngularVelocityClamped(toJphVec3(velocity));
}

JOLTC_API float JoltC_MotionProperties_GetMaxLinearVelocity(const JoltC_MotionProperties* properties) {
    if (!properties) return 0.0f;
    return asMP(properties)->GetMaxLinearVelocity();
}

JOLTC_API void JoltC_MotionProperties_SetMaxLinearVelocity(JoltC_MotionProperties* properties, float velocity) {
    if (!properties) return;
    asMPMut(properties)->SetMaxLinearVelocity(velocity);
}

JOLTC_API float JoltC_MotionProperties_GetMaxAngularVelocity(const JoltC_MotionProperties* properties) {
    if (!properties) return 0.0f;
    return asMP(properties)->GetMaxAngularVelocity();
}

JOLTC_API void JoltC_MotionProperties_SetMaxAngularVelocity(JoltC_MotionProperties* properties, float velocity) {
    if (!properties) return;
    asMPMut(properties)->SetMaxAngularVelocity(velocity);
}

JOLTC_API void JoltC_MotionProperties_ClampLinearVelocity(JoltC_MotionProperties* properties) {
    if (!properties) return;
    asMPMut(properties)->ClampLinearVelocity();
}

JOLTC_API void JoltC_MotionProperties_ClampAngularVelocity(JoltC_MotionProperties* properties) {
    if (!properties) return;
    asMPMut(properties)->ClampAngularVelocity();
}

JOLTC_API float JoltC_MotionProperties_GetGravityFactor(const JoltC_MotionProperties* properties) {
    if (!properties) return 0.0f;
    return asMP(properties)->GetGravityFactor();
}

JOLTC_API void JoltC_MotionProperties_SetGravityFactor(JoltC_MotionProperties* properties, float factor) {
    if (!properties) return;
    asMPMut(properties)->SetGravityFactor(factor);
}

JOLTC_API uint32_t JoltC_MotionProperties_GetNumVelocityStepsOverride(const JoltC_MotionProperties* properties) {
    if (!properties) return 0;
    return asMP(properties)->GetNumVelocityStepsOverride();
}

JOLTC_API void JoltC_MotionProperties_SetNumVelocityStepsOverride(JoltC_MotionProperties* properties, uint32_t steps) {
    if (!properties) return;
    asMPMut(properties)->SetNumVelocityStepsOverride(steps);
}

JOLTC_API uint32_t JoltC_MotionProperties_GetNumPositionStepsOverride(const JoltC_MotionProperties* properties) {
    if (!properties) return 0;
    return asMP(properties)->GetNumPositionStepsOverride();
}

JOLTC_API void JoltC_MotionProperties_SetNumPositionStepsOverride(JoltC_MotionProperties* properties, uint32_t steps) {
    if (!properties) return;
    asMPMut(properties)->SetNumPositionStepsOverride(steps);
}

JOLTC_API JoltC_Vec3 JoltC_MotionProperties_GetAccumulatedForce(const JoltC_MotionProperties* properties) {
    if (!properties) return JoltC_Vec3{0, 0, 0};
    return fromJphVec3(asMP(properties)->GetAccumulatedForce());
}

JOLTC_API JoltC_Vec3 JoltC_MotionProperties_GetAccumulatedTorque(const JoltC_MotionProperties* properties) {
    if (!properties) return JoltC_Vec3{0, 0, 0};
    return fromJphVec3(asMP(properties)->GetAccumulatedTorque());
}

JOLTC_API void JoltC_MotionProperties_ResetForce(JoltC_MotionProperties* properties) {
    if (!properties) return;
    asMPMut(properties)->ResetForce();
}

JOLTC_API void JoltC_MotionProperties_ResetTorque(JoltC_MotionProperties* properties) {
    if (!properties) return;
    asMPMut(properties)->ResetTorque();
}

JOLTC_API JoltC_Vec3 JoltC_MotionProperties_GetPointVelocityCOM(const JoltC_MotionProperties* properties, JoltC_Vec3 pointRelativeToCOM) {
    if (!properties) return JoltC_Vec3{0, 0, 0};
    return fromJphVec3(asMP(properties)->GetPointVelocityCOM(toJphVec3(pointRelativeToCOM)));
}

JOLTC_API void JoltC_MotionProperties_GetLocalSpaceInverseInertia(const JoltC_MotionProperties* properties, JoltC_Mat44* result) {
    if (!properties || !result) return;
    *result = fromJphMat44(asMP(properties)->GetLocalSpaceInverseInertia());
}

} /* extern "C" */
