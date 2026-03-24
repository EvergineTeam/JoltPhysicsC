/* JoltC - BodyInterface implementations
 * SPDX-License-Identifier: MIT
 */

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>

#include <JoltC/body.h>
#include "internal.h"
#include "wrappers.h"
#include "errors_internal.h"

using namespace JPH;

/* -------------------------------------------------------------------------- */
/*  Helpers                                                                   */
/* -------------------------------------------------------------------------- */
static inline BodyInterface* bi(JoltC_BodyInterface* iface) {
    return iface ? iface->ptr : nullptr;
}

static inline const BodyInterface* bi(const JoltC_BodyInterface* iface) {
    return iface ? iface->ptr : nullptr;
}

static inline const Shape* asShape(const JoltC_Shape* h) {
    return reinterpret_cast<const Shape*>(h);
}

/* Convert JoltC_BodyCreationSettings -> JPH::BodyCreationSettings */
static BodyCreationSettings toJphBCS(const JoltC_BodyCreationSettings* s) {
    BodyCreationSettings bcs;

    bcs.mPosition        = toJphRVec3(s->position);
    bcs.mRotation        = toJphQuat(s->rotation);
    bcs.mLinearVelocity  = toJphVec3(s->linearVelocity);
    bcs.mAngularVelocity = toJphVec3(s->angularVelocity);
    bcs.mUserData        = s->userData;
    bcs.mObjectLayer     = s->objectLayer;
    bcs.mCollisionGroup.SetGroupID(s->collisionGroup.groupID);
    bcs.mCollisionGroup.SetSubGroupID(s->collisionGroup.subGroupID);
    if (s->collisionGroup.groupFilter)
        bcs.mCollisionGroup.SetGroupFilter(reinterpret_cast<const GroupFilter*>(s->collisionGroup.groupFilter));
    bcs.mMotionType      = toJphMotionType(s->motionType);
    bcs.mAllowedDOFs     = toJphAllowedDOFs(s->allowedDOFs);
    bcs.mAllowDynamicOrKinematic     = s->allowDynamicOrKinematic != 0;
    bcs.mIsSensor                    = s->isSensor != 0;
    bcs.mCollideKinematicVsNonDynamic = s->collideKinematicVsNonDynamic != 0;
    bcs.mUseManifoldReduction        = s->useManifoldReduction != 0;
    bcs.mApplyGyroscopicForce        = s->applyGyroscopicForce != 0;
    bcs.mMotionQuality               = toJphMotionQuality(s->motionQuality);
    bcs.mEnhancedInternalEdgeRemoval = s->enhancedInternalEdgeRemoval != 0;
    bcs.mAllowSleeping               = s->allowSleeping != 0;
    bcs.mFriction        = s->friction;
    bcs.mRestitution     = s->restitution;
    bcs.mLinearDamping   = s->linearDamping;
    bcs.mAngularDamping  = s->angularDamping;
    bcs.mMaxLinearVelocity  = s->maxLinearVelocity;
    bcs.mMaxAngularVelocity = s->maxAngularVelocity;
    bcs.mGravityFactor   = s->gravityFactor;
    bcs.mNumVelocityStepsOverride = s->numVelocityStepsOverride;
    bcs.mNumPositionStepsOverride = s->numPositionStepsOverride;
    bcs.mOverrideMassProperties = static_cast<EOverrideMassProperties>(s->overrideMassProperties);
    bcs.mInertiaMultiplier = s->inertiaMultiplier;
    bcs.mMassPropertiesOverride.mMass = s->massPropertiesOverride.mass;
    bcs.mMassPropertiesOverride.mInertia = toJphMat44(s->massPropertiesOverride.inertia);

    if (s->shape) {
        bcs.SetShape(asShape(s->shape));
    }

    return bcs;
}

/* Fill a C BCS from defaults */
static void setJphBCSDefaults(JoltC_BodyCreationSettings* s) {
    BodyCreationSettings d;
    s->position = fromJphRVec3(d.mPosition);
    s->rotation = fromJphQuat(d.mRotation);
    s->linearVelocity = fromJphVec3(d.mLinearVelocity);
    s->angularVelocity = fromJphVec3(d.mAngularVelocity);
    s->userData = d.mUserData;
    s->objectLayer = d.mObjectLayer;
    s->collisionGroup.groupFilter = nullptr;
    s->collisionGroup.groupID = d.mCollisionGroup.GetGroupID();
    s->collisionGroup.subGroupID = d.mCollisionGroup.GetSubGroupID();
    s->motionType = static_cast<JoltC_MotionType>(d.mMotionType);
    s->allowedDOFs = static_cast<JoltC_AllowedDOFs>(d.mAllowedDOFs);
    s->allowDynamicOrKinematic = d.mAllowDynamicOrKinematic ? JOLTC_TRUE : JOLTC_FALSE;
    s->isSensor = d.mIsSensor ? JOLTC_TRUE : JOLTC_FALSE;
    s->collideKinematicVsNonDynamic = d.mCollideKinematicVsNonDynamic ? JOLTC_TRUE : JOLTC_FALSE;
    s->useManifoldReduction = d.mUseManifoldReduction ? JOLTC_TRUE : JOLTC_FALSE;
    s->applyGyroscopicForce = d.mApplyGyroscopicForce ? JOLTC_TRUE : JOLTC_FALSE;
    s->motionQuality = static_cast<JoltC_MotionQuality>(d.mMotionQuality);
    s->enhancedInternalEdgeRemoval = d.mEnhancedInternalEdgeRemoval ? JOLTC_TRUE : JOLTC_FALSE;
    s->allowSleeping = d.mAllowSleeping ? JOLTC_TRUE : JOLTC_FALSE;
    s->friction = d.mFriction;
    s->restitution = d.mRestitution;
    s->linearDamping = d.mLinearDamping;
    s->angularDamping = d.mAngularDamping;
    s->maxLinearVelocity = d.mMaxLinearVelocity;
    s->maxAngularVelocity = d.mMaxAngularVelocity;
    s->gravityFactor = d.mGravityFactor;
    s->numVelocityStepsOverride = d.mNumVelocityStepsOverride;
    s->numPositionStepsOverride = d.mNumPositionStepsOverride;
    s->overrideMassProperties = static_cast<JoltC_OverrideMassProperties>(d.mOverrideMassProperties);
    s->inertiaMultiplier = d.mInertiaMultiplier;
    s->massPropertiesOverride.mass = d.mMassPropertiesOverride.mMass;
    s->massPropertiesOverride.inertia = fromJphMat44(d.mMassPropertiesOverride.mInertia);
    s->shape = nullptr;
}

extern "C" {

/* ========================================================================== */
/*  BodyCreationSettings — opaque-handle API                                  */
/* ========================================================================== */
JOLTC_API JoltC_BodyCreationSettings* JoltC_BodyCreationSettings_Create(void) {
    auto* s = new JoltC_BodyCreationSettings;
    setJphBCSDefaults(s);
    return s;
}
JOLTC_API JoltC_BodyCreationSettings* JoltC_BodyCreationSettings_Create2(const JoltC_ShapeSettings* shapeSettings, JoltC_RVec3 position, JoltC_Quat rotation, JoltC_MotionType motionType, JoltC_ObjectLayer objectLayer) {
    auto* s = new JoltC_BodyCreationSettings;
    setJphBCSDefaults(s);
    s->position = position;
    s->rotation = rotation;
    s->motionType = motionType;
    s->objectLayer = objectLayer;
    /* ShapeSettings path: caller must convert to Shape before CreateBody */
    (void)shapeSettings;
    return s;
}
JOLTC_API JoltC_BodyCreationSettings* JoltC_BodyCreationSettings_Create3(const JoltC_Shape* shape, JoltC_RVec3 position, JoltC_Quat rotation, JoltC_MotionType motionType, JoltC_ObjectLayer objectLayer) {
    auto* s = new JoltC_BodyCreationSettings;
    setJphBCSDefaults(s);
    s->position = position;
    s->rotation = rotation;
    s->motionType = motionType;
    s->objectLayer = objectLayer;
    s->shape = shape;
    return s;
}
JOLTC_API void JoltC_BodyCreationSettings_Destroy(JoltC_BodyCreationSettings* s) {
    delete s;
}

/* Position / Rotation */
JOLTC_API void JoltC_BodyCreationSettings_GetPosition(const JoltC_BodyCreationSettings* s, JoltC_RVec3* r) { if (s && r) *r = s->position; }
JOLTC_API void JoltC_BodyCreationSettings_SetPosition(JoltC_BodyCreationSettings* s, const JoltC_RVec3* v) { if (s && v) s->position = *v; }
JOLTC_API void JoltC_BodyCreationSettings_GetRotation(const JoltC_BodyCreationSettings* s, JoltC_Quat* r) { if (s && r) *r = s->rotation; }
JOLTC_API void JoltC_BodyCreationSettings_SetRotation(JoltC_BodyCreationSettings* s, const JoltC_Quat* v) { if (s && v) s->rotation = *v; }

/* Velocities */
JOLTC_API void JoltC_BodyCreationSettings_GetLinearVelocity(const JoltC_BodyCreationSettings* s, JoltC_Vec3* r) { if (s && r) *r = s->linearVelocity; }
JOLTC_API void JoltC_BodyCreationSettings_SetLinearVelocity(JoltC_BodyCreationSettings* s, const JoltC_Vec3* v) { if (s && v) s->linearVelocity = *v; }
JOLTC_API void JoltC_BodyCreationSettings_GetAngularVelocity(const JoltC_BodyCreationSettings* s, JoltC_Vec3* r) { if (s && r) *r = s->angularVelocity; }
JOLTC_API void JoltC_BodyCreationSettings_SetAngularVelocity(JoltC_BodyCreationSettings* s, const JoltC_Vec3* v) { if (s && v) s->angularVelocity = *v; }

/* User data */
JOLTC_API uint64_t JoltC_BodyCreationSettings_GetUserData(const JoltC_BodyCreationSettings* s) { return s ? s->userData : 0; }
JOLTC_API void JoltC_BodyCreationSettings_SetUserData(JoltC_BodyCreationSettings* s, uint64_t v) { if (s) s->userData = v; }

/* Layer / Group */
JOLTC_API JoltC_ObjectLayer JoltC_BodyCreationSettings_GetObjectLayer(const JoltC_BodyCreationSettings* s) { return s ? s->objectLayer : 0; }
JOLTC_API void JoltC_BodyCreationSettings_SetObjectLayer(JoltC_BodyCreationSettings* s, JoltC_ObjectLayer v) { if (s) s->objectLayer = v; }
JOLTC_API void JoltC_BodyCreationSettings_GetCollisionGroup(const JoltC_BodyCreationSettings* s, JoltC_CollisionGroup* r) { if (s && r) *r = s->collisionGroup; }
JOLTC_API void JoltC_BodyCreationSettings_SetCollisionGroup(JoltC_BodyCreationSettings* s, const JoltC_CollisionGroup* v) { if (s && v) s->collisionGroup = *v; }

/* Motion / DOFs */
JOLTC_API JoltC_MotionType JoltC_BodyCreationSettings_GetMotionType(const JoltC_BodyCreationSettings* s) { return s ? s->motionType : JOLTC_MOTION_TYPE_STATIC; }
JOLTC_API void JoltC_BodyCreationSettings_SetMotionType(JoltC_BodyCreationSettings* s, JoltC_MotionType v) { if (s) s->motionType = v; }
JOLTC_API JoltC_AllowedDOFs JoltC_BodyCreationSettings_GetAllowedDOFs(const JoltC_BodyCreationSettings* s) { return s ? s->allowedDOFs : JOLTC_ALLOWED_DOFS_ALL; }
JOLTC_API void JoltC_BodyCreationSettings_SetAllowedDOFs(JoltC_BodyCreationSettings* s, JoltC_AllowedDOFs v) { if (s) s->allowedDOFs = v; }

/* Bool flags */
JOLTC_API JoltC_Bool JoltC_BodyCreationSettings_GetAllowDynamicOrKinematic(const JoltC_BodyCreationSettings* s) { return s ? s->allowDynamicOrKinematic : JOLTC_FALSE; }
JOLTC_API void JoltC_BodyCreationSettings_SetAllowDynamicOrKinematic(JoltC_BodyCreationSettings* s, JoltC_Bool v) { if (s) s->allowDynamicOrKinematic = v; }
JOLTC_API JoltC_Bool JoltC_BodyCreationSettings_GetIsSensor(const JoltC_BodyCreationSettings* s) { return s ? s->isSensor : JOLTC_FALSE; }
JOLTC_API void JoltC_BodyCreationSettings_SetIsSensor(JoltC_BodyCreationSettings* s, JoltC_Bool v) { if (s) s->isSensor = v; }
JOLTC_API JoltC_Bool JoltC_BodyCreationSettings_GetCollideKinematicVsNonDynamic(const JoltC_BodyCreationSettings* s) { return s ? s->collideKinematicVsNonDynamic : JOLTC_FALSE; }
JOLTC_API void JoltC_BodyCreationSettings_SetCollideKinematicVsNonDynamic(JoltC_BodyCreationSettings* s, JoltC_Bool v) { if (s) s->collideKinematicVsNonDynamic = v; }
JOLTC_API JoltC_Bool JoltC_BodyCreationSettings_GetUseManifoldReduction(const JoltC_BodyCreationSettings* s) { return s ? s->useManifoldReduction : JOLTC_FALSE; }
JOLTC_API void JoltC_BodyCreationSettings_SetUseManifoldReduction(JoltC_BodyCreationSettings* s, JoltC_Bool v) { if (s) s->useManifoldReduction = v; }
JOLTC_API JoltC_Bool JoltC_BodyCreationSettings_GetApplyGyroscopicForce(const JoltC_BodyCreationSettings* s) { return s ? s->applyGyroscopicForce : JOLTC_FALSE; }
JOLTC_API void JoltC_BodyCreationSettings_SetApplyGyroscopicForce(JoltC_BodyCreationSettings* s, JoltC_Bool v) { if (s) s->applyGyroscopicForce = v; }

/* Motion quality / edge removal / sleeping */
JOLTC_API JoltC_MotionQuality JoltC_BodyCreationSettings_GetMotionQuality(const JoltC_BodyCreationSettings* s) { return s ? s->motionQuality : JOLTC_MOTION_QUALITY_DISCRETE; }
JOLTC_API void JoltC_BodyCreationSettings_SetMotionQuality(JoltC_BodyCreationSettings* s, JoltC_MotionQuality v) { if (s) s->motionQuality = v; }
JOLTC_API JoltC_Bool JoltC_BodyCreationSettings_GetEnhancedInternalEdgeRemoval(const JoltC_BodyCreationSettings* s) { return s ? s->enhancedInternalEdgeRemoval : JOLTC_FALSE; }
JOLTC_API void JoltC_BodyCreationSettings_SetEnhancedInternalEdgeRemoval(JoltC_BodyCreationSettings* s, JoltC_Bool v) { if (s) s->enhancedInternalEdgeRemoval = v; }
JOLTC_API JoltC_Bool JoltC_BodyCreationSettings_GetAllowSleeping(const JoltC_BodyCreationSettings* s) { return s ? s->allowSleeping : JOLTC_FALSE; }
JOLTC_API void JoltC_BodyCreationSettings_SetAllowSleeping(JoltC_BodyCreationSettings* s, JoltC_Bool v) { if (s) s->allowSleeping = v; }

/* Float properties */
JOLTC_API float JoltC_BodyCreationSettings_GetFriction(const JoltC_BodyCreationSettings* s) { return s ? s->friction : 0; }
JOLTC_API void JoltC_BodyCreationSettings_SetFriction(JoltC_BodyCreationSettings* s, float v) { if (s) s->friction = v; }
JOLTC_API float JoltC_BodyCreationSettings_GetRestitution(const JoltC_BodyCreationSettings* s) { return s ? s->restitution : 0; }
JOLTC_API void JoltC_BodyCreationSettings_SetRestitution(JoltC_BodyCreationSettings* s, float v) { if (s) s->restitution = v; }
JOLTC_API float JoltC_BodyCreationSettings_GetLinearDamping(const JoltC_BodyCreationSettings* s) { return s ? s->linearDamping : 0; }
JOLTC_API void JoltC_BodyCreationSettings_SetLinearDamping(JoltC_BodyCreationSettings* s, float v) { if (s) s->linearDamping = v; }
JOLTC_API float JoltC_BodyCreationSettings_GetAngularDamping(const JoltC_BodyCreationSettings* s) { return s ? s->angularDamping : 0; }
JOLTC_API void JoltC_BodyCreationSettings_SetAngularDamping(JoltC_BodyCreationSettings* s, float v) { if (s) s->angularDamping = v; }
JOLTC_API float JoltC_BodyCreationSettings_GetMaxLinearVelocity(const JoltC_BodyCreationSettings* s) { return s ? s->maxLinearVelocity : 0; }
JOLTC_API void JoltC_BodyCreationSettings_SetMaxLinearVelocity(JoltC_BodyCreationSettings* s, float v) { if (s) s->maxLinearVelocity = v; }
JOLTC_API float JoltC_BodyCreationSettings_GetMaxAngularVelocity(const JoltC_BodyCreationSettings* s) { return s ? s->maxAngularVelocity : 0; }
JOLTC_API void JoltC_BodyCreationSettings_SetMaxAngularVelocity(JoltC_BodyCreationSettings* s, float v) { if (s) s->maxAngularVelocity = v; }
JOLTC_API float JoltC_BodyCreationSettings_GetGravityFactor(const JoltC_BodyCreationSettings* s) { return s ? s->gravityFactor : 0; }
JOLTC_API void JoltC_BodyCreationSettings_SetGravityFactor(JoltC_BodyCreationSettings* s, float v) { if (s) s->gravityFactor = v; }

/* Solver overrides */
JOLTC_API uint32_t JoltC_BodyCreationSettings_GetNumVelocityStepsOverride(const JoltC_BodyCreationSettings* s) { return s ? s->numVelocityStepsOverride : 0; }
JOLTC_API void JoltC_BodyCreationSettings_SetNumVelocityStepsOverride(JoltC_BodyCreationSettings* s, uint32_t v) { if (s) s->numVelocityStepsOverride = v; }
JOLTC_API uint32_t JoltC_BodyCreationSettings_GetNumPositionStepsOverride(const JoltC_BodyCreationSettings* s) { return s ? s->numPositionStepsOverride : 0; }
JOLTC_API void JoltC_BodyCreationSettings_SetNumPositionStepsOverride(JoltC_BodyCreationSettings* s, uint32_t v) { if (s) s->numPositionStepsOverride = v; }

/* Mass properties */
JOLTC_API JoltC_OverrideMassProperties JoltC_BodyCreationSettings_GetOverrideMassProperties(const JoltC_BodyCreationSettings* s) { return s ? s->overrideMassProperties : JOLTC_OVERRIDE_MASS_CALC_MASS_AND_INERTIA; }
JOLTC_API void JoltC_BodyCreationSettings_SetOverrideMassProperties(JoltC_BodyCreationSettings* s, JoltC_OverrideMassProperties v) { if (s) s->overrideMassProperties = v; }
JOLTC_API float JoltC_BodyCreationSettings_GetInertiaMultiplier(const JoltC_BodyCreationSettings* s) { return s ? s->inertiaMultiplier : 0; }
JOLTC_API void JoltC_BodyCreationSettings_SetInertiaMultiplier(JoltC_BodyCreationSettings* s, float v) { if (s) s->inertiaMultiplier = v; }
JOLTC_API void JoltC_BodyCreationSettings_GetMassPropertiesOverride(const JoltC_BodyCreationSettings* s, JoltC_MassProperties* r) { if (s && r) *r = s->massPropertiesOverride; }
JOLTC_API void JoltC_BodyCreationSettings_SetMassPropertiesOverride(JoltC_BodyCreationSettings* s, const JoltC_MassProperties* v) { if (s && v) s->massPropertiesOverride = *v; }

/* -------------------------------------------------------------------------- */
/*  Create / Destroy                                                          */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_BodyID JoltC_BodyInterface_CreateBody(JoltC_BodyInterface* iface, const JoltC_BodyCreationSettings* settings)
{
    if (!bi(iface) || !settings) return JOLTC_BODY_ID_INVALID;
    JOLTC_TRY_BEGIN
    BodyCreationSettings bcs = toJphBCS(settings);
    Body* body = bi(iface)->CreateBody(bcs);
    if (!body) return JOLTC_BODY_ID_INVALID;
    return fromJphBodyID(body->GetID());
    JOLTC_TRY_END
    return JOLTC_BODY_ID_INVALID;
}

JOLTC_API JoltC_BodyID JoltC_BodyInterface_CreateAndAddBody(JoltC_BodyInterface* iface, const JoltC_BodyCreationSettings* settings, JoltC_Activation activation)
{
    if (!bi(iface) || !settings) return JOLTC_BODY_ID_INVALID;
    JOLTC_TRY_BEGIN
    BodyCreationSettings bcs = toJphBCS(settings);
    BodyID id = bi(iface)->CreateAndAddBody(bcs, toJphActivation(activation));
    return fromJphBodyID(id);
    JOLTC_TRY_END
    return JOLTC_BODY_ID_INVALID;
}

JOLTC_API void JoltC_BodyInterface_DestroyBody(JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->DestroyBody(toJphBodyID(bodyID));
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  Add / Remove                                                              */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_BodyInterface_AddBody(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_Activation activation)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->AddBody(toJphBodyID(bodyID), toJphActivation(activation));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_RemoveBody(JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->RemoveBody(toJphBodyID(bodyID));
    JOLTC_TRY_END
}

JOLTC_API JoltC_Bool JoltC_BodyInterface_IsAdded(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return JOLTC_FALSE;
    JOLTC_TRY_BEGIN
    return bi(iface)->IsAdded(toJphBodyID(bodyID)) ? JOLTC_TRUE : JOLTC_FALSE;
    JOLTC_TRY_END
    return JOLTC_FALSE;
}

/* -------------------------------------------------------------------------- */
/*  Activation                                                                */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_BodyInterface_ActivateBody(JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->ActivateBody(toJphBodyID(bodyID));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_DeactivateBody(JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->DeactivateBody(toJphBodyID(bodyID));
    JOLTC_TRY_END
}

JOLTC_API JoltC_Bool JoltC_BodyInterface_IsActive(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return JOLTC_FALSE;
    JOLTC_TRY_BEGIN
    return bi(iface)->IsActive(toJphBodyID(bodyID)) ? JOLTC_TRUE : JOLTC_FALSE;
    JOLTC_TRY_END
    return JOLTC_FALSE;
}

/* -------------------------------------------------------------------------- */
/*  Position / Rotation                                                       */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_BodyInterface_SetPosition(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_RVec3 position, JoltC_Activation activation)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->SetPosition(toJphBodyID(bodyID), toJphRVec3(position), toJphActivation(activation));
    JOLTC_TRY_END
}

JOLTC_API JoltC_RVec3 JoltC_BodyInterface_GetPosition(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    JoltC_RVec3 zero = {0, 0, 0};
    if (!bi(iface)) return zero;
    JOLTC_TRY_BEGIN
    return fromJphRVec3(bi(iface)->GetPosition(toJphBodyID(bodyID)));
    JOLTC_TRY_END
    return zero;
}

JOLTC_API JoltC_RVec3 JoltC_BodyInterface_GetCenterOfMassPosition(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    JoltC_RVec3 zero = {0, 0, 0};
    if (!bi(iface)) return zero;
    JOLTC_TRY_BEGIN
    return fromJphRVec3(bi(iface)->GetCenterOfMassPosition(toJphBodyID(bodyID)));
    JOLTC_TRY_END
    return zero;
}

JOLTC_API void JoltC_BodyInterface_SetRotation(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_Quat rotation, JoltC_Activation activation)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->SetRotation(toJphBodyID(bodyID), toJphQuat(rotation), toJphActivation(activation));
    JOLTC_TRY_END
}

JOLTC_API JoltC_Quat JoltC_BodyInterface_GetRotation(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    JoltC_Quat identity = {0, 0, 0, 1};
    if (!bi(iface)) return identity;
    JOLTC_TRY_BEGIN
    return fromJphQuat(bi(iface)->GetRotation(toJphBodyID(bodyID)));
    JOLTC_TRY_END
    return identity;
}

JOLTC_API void JoltC_BodyInterface_SetPositionAndRotation(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_RVec3 position, JoltC_Quat rotation, JoltC_Activation activation)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->SetPositionAndRotation(toJphBodyID(bodyID), toJphRVec3(position), toJphQuat(rotation), toJphActivation(activation));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_GetPositionAndRotation(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_RVec3* outPosition, JoltC_Quat* outRotation)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    RVec3 pos;
    Quat rot;
    bi(iface)->GetPositionAndRotation(toJphBodyID(bodyID), pos, rot);
    if (outPosition) *outPosition = fromJphRVec3(pos);
    if (outRotation) *outRotation = fromJphQuat(rot);
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  Velocity                                                                  */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_BodyInterface_SetLinearVelocity(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_Vec3 velocity)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->SetLinearVelocity(toJphBodyID(bodyID), toJphVec3(velocity));
    JOLTC_TRY_END
}

JOLTC_API JoltC_Vec3 JoltC_BodyInterface_GetLinearVelocity(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return JoltC_Vec3{0, 0, 0};
    JOLTC_TRY_BEGIN
    return fromJphVec3(bi(iface)->GetLinearVelocity(toJphBodyID(bodyID)));
    JOLTC_TRY_END
    return JoltC_Vec3{0, 0, 0};
}

JOLTC_API void JoltC_BodyInterface_AddLinearVelocity(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_Vec3 velocity)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->AddLinearVelocity(toJphBodyID(bodyID), toJphVec3(velocity));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_SetAngularVelocity(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_Vec3 angularVelocity)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->SetAngularVelocity(toJphBodyID(bodyID), toJphVec3(angularVelocity));
    JOLTC_TRY_END
}

JOLTC_API JoltC_Vec3 JoltC_BodyInterface_GetAngularVelocity(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return JoltC_Vec3{0, 0, 0};
    JOLTC_TRY_BEGIN
    return fromJphVec3(bi(iface)->GetAngularVelocity(toJphBodyID(bodyID)));
    JOLTC_TRY_END
    return JoltC_Vec3{0, 0, 0};
}

JOLTC_API void JoltC_BodyInterface_SetLinearAndAngularVelocity(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_Vec3 linearVelocity, JoltC_Vec3 angularVelocity)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->SetLinearAndAngularVelocity(toJphBodyID(bodyID), toJphVec3(linearVelocity), toJphVec3(angularVelocity));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_MoveKinematic(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_RVec3 targetPosition, JoltC_Quat targetRotation, float deltaTime)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->MoveKinematic(toJphBodyID(bodyID), toJphRVec3(targetPosition), toJphQuat(targetRotation), deltaTime);
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  Force / Torque / Impulse                                                  */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_BodyInterface_AddForce(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_Vec3 force)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->AddForce(toJphBodyID(bodyID), toJphVec3(force));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_AddForceAtPosition(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_Vec3 force, JoltC_RVec3 point)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->AddForce(toJphBodyID(bodyID), toJphVec3(force), toJphRVec3(point));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_AddTorque(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_Vec3 torque)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->AddTorque(toJphBodyID(bodyID), toJphVec3(torque));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_AddImpulse(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_Vec3 impulse)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->AddImpulse(toJphBodyID(bodyID), toJphVec3(impulse));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_AddImpulseAtPosition(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_Vec3 impulse, JoltC_RVec3 point)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->AddImpulse(toJphBodyID(bodyID), toJphVec3(impulse), toJphRVec3(point));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_AddAngularImpulse(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_Vec3 angularImpulse)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->AddAngularImpulse(toJphBodyID(bodyID), toJphVec3(angularImpulse));
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  Properties                                                                */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_BodyInterface_SetMotionType(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_MotionType motionType, JoltC_Activation activation)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->SetMotionType(toJphBodyID(bodyID), toJphMotionType(motionType), toJphActivation(activation));
    JOLTC_TRY_END
}

JOLTC_API JoltC_MotionType JoltC_BodyInterface_GetMotionType(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return JOLTC_MOTION_TYPE_STATIC;
    JOLTC_TRY_BEGIN
    return fromJphMotionType(bi(iface)->GetMotionType(toJphBodyID(bodyID)));
    JOLTC_TRY_END
    return JOLTC_MOTION_TYPE_STATIC;
}

JOLTC_API JoltC_BodyType JoltC_BodyInterface_GetBodyType(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return JOLTC_BODY_TYPE_RIGID;
    JOLTC_TRY_BEGIN
    return fromJphBodyType(bi(iface)->GetBodyType(toJphBodyID(bodyID)));
    JOLTC_TRY_END
    return JOLTC_BODY_TYPE_RIGID;
}

JOLTC_API void JoltC_BodyInterface_SetFriction(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, float friction)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->SetFriction(toJphBodyID(bodyID), friction);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_BodyInterface_GetFriction(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return 0.0f;
    JOLTC_TRY_BEGIN
    return bi(iface)->GetFriction(toJphBodyID(bodyID));
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API void JoltC_BodyInterface_SetRestitution(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, float restitution)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->SetRestitution(toJphBodyID(bodyID), restitution);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_BodyInterface_GetRestitution(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return 0.0f;
    JOLTC_TRY_BEGIN
    return bi(iface)->GetRestitution(toJphBodyID(bodyID));
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API void JoltC_BodyInterface_SetGravityFactor(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, float gravityFactor)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->SetGravityFactor(toJphBodyID(bodyID), gravityFactor);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_BodyInterface_GetGravityFactor(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return 0.0f;
    JOLTC_TRY_BEGIN
    return bi(iface)->GetGravityFactor(toJphBodyID(bodyID));
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API void JoltC_BodyInterface_SetObjectLayer(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_ObjectLayer layer)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->SetObjectLayer(toJphBodyID(bodyID), layer);
    JOLTC_TRY_END
}

JOLTC_API JoltC_ObjectLayer JoltC_BodyInterface_GetObjectLayer(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return JOLTC_OBJECT_LAYER_INVALID;
    JOLTC_TRY_BEGIN
    return bi(iface)->GetObjectLayer(toJphBodyID(bodyID));
    JOLTC_TRY_END
    return JOLTC_OBJECT_LAYER_INVALID;
}

JOLTC_API void JoltC_BodyInterface_SetUserData(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, uint64_t userData)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->SetUserData(toJphBodyID(bodyID), userData);
    JOLTC_TRY_END
}

JOLTC_API uint64_t JoltC_BodyInterface_GetUserData(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return 0;
    JOLTC_TRY_BEGIN
    return bi(iface)->GetUserData(toJphBodyID(bodyID));
    JOLTC_TRY_END
    return 0;
}

/* -------------------------------------------------------------------------- */
/*  Shape                                                                     */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_BodyInterface_SetShape(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, const JoltC_Shape* shape, JoltC_Bool updateMassProperties, JoltC_Activation activation)
{
    if (!bi(iface) || !shape) return;
    JOLTC_TRY_BEGIN
    bi(iface)->SetShape(toJphBodyID(bodyID), asShape(shape), updateMassProperties != 0, toJphActivation(activation));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_NotifyShapeChanged(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_Vec3 previousCenterOfMass, JoltC_Bool updateMassProperties, JoltC_Activation activation)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->NotifyShapeChanged(toJphBodyID(bodyID), toJphVec3(previousCenterOfMass), updateMassProperties != 0, toJphActivation(activation));
    JOLTC_TRY_END
}

JOLTC_API const JoltC_Shape* JoltC_BodyInterface_GetShape(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return nullptr;
    JOLTC_TRY_BEGIN
    RefConst<Shape> shape = bi(iface)->GetShape(toJphBodyID(bodyID));
    if (!shape) return nullptr;
    shape->AddRef();
    return reinterpret_cast<const JoltC_Shape*>(shape.GetPtr());
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  Point velocity / Transform                                                */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_Vec3 JoltC_BodyInterface_GetPointVelocity(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_RVec3 point)
{
    JoltC_Vec3 z = {0,0,0}; if (!bi(iface)) return z;
    JOLTC_TRY_BEGIN
    return fromJphVec3(bi(iface)->GetPointVelocity(toJphBodyID(bodyID), toJphRVec3(point)));
    JOLTC_TRY_END
    return z;
}

JOLTC_API JoltC_Mat44 JoltC_BodyInterface_GetWorldTransform(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    JoltC_Mat44 z = {}; if (!bi(iface)) return z;
    JOLTC_TRY_BEGIN
    return fromJphMat44(bi(iface)->GetWorldTransform(toJphBodyID(bodyID)));
    JOLTC_TRY_END
    return z;
}

JOLTC_API JoltC_Mat44 JoltC_BodyInterface_GetCenterOfMassTransform(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    JoltC_Mat44 z = {}; if (!bi(iface)) return z;
    JOLTC_TRY_BEGIN
    return fromJphMat44(bi(iface)->GetCenterOfMassTransform(toJphBodyID(bodyID)));
    JOLTC_TRY_END
    return z;
}

/* -------------------------------------------------------------------------- */
/*  Inverse mass                                                              */
/* -------------------------------------------------------------------------- */
JOLTC_API float JoltC_BodyInterface_GetInverseMass(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return 0;
    JOLTC_TRY_BEGIN
    BodyLockRead lock(iface->system->GetBodyLockInterface(), toJphBodyID(bodyID));
    if (lock.Succeeded())
        return lock.GetBody().GetMotionProperties()->GetInverseMass();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_BodyInterface_SetInverseMass(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, float inverseMass)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    BodyLockWrite lock(iface->system->GetBodyLockInterface(), toJphBodyID(bodyID));
    if (lock.Succeeded())
        lock.GetBody().GetMotionProperties()->SetInverseMass(inverseMass);
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  Additional body interface methods                                         */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_BodyInterface_SetMotionQuality(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_MotionQuality quality)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->SetMotionQuality(toJphBodyID(bodyID), toJphMotionQuality(quality));
    JOLTC_TRY_END
}

JOLTC_API JoltC_MotionQuality JoltC_BodyInterface_GetMotionQuality(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return JOLTC_MOTION_QUALITY_DISCRETE;
    JOLTC_TRY_BEGIN
    return fromJphMotionQuality(bi(iface)->GetMotionQuality(toJphBodyID(bodyID)));
    JOLTC_TRY_END
    return JOLTC_MOTION_QUALITY_DISCRETE;
}

JOLTC_API void JoltC_BodyInterface_SetLinearDamping(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, float damping)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    BodyLockWrite lock(iface->system->GetBodyLockInterface(), toJphBodyID(bodyID));
    if (lock.Succeeded())
        lock.GetBody().GetMotionProperties()->SetLinearDamping(damping);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_BodyInterface_GetLinearDamping(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return 0;
    JOLTC_TRY_BEGIN
    BodyLockRead lock(iface->system->GetBodyLockInterface(), toJphBodyID(bodyID));
    if (lock.Succeeded())
        return lock.GetBody().GetMotionProperties()->GetLinearDamping();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_BodyInterface_SetAngularDamping(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, float damping)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    BodyLockWrite lock(iface->system->GetBodyLockInterface(), toJphBodyID(bodyID));
    if (lock.Succeeded())
        lock.GetBody().GetMotionProperties()->SetAngularDamping(damping);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_BodyInterface_GetAngularDamping(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return 0;
    JOLTC_TRY_BEGIN
    BodyLockRead lock(iface->system->GetBodyLockInterface(), toJphBodyID(bodyID));
    if (lock.Succeeded())
        return lock.GetBody().GetMotionProperties()->GetAngularDamping();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_BodyInterface_SetMaxLinearVelocity(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, float maxVelocity)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->SetMaxLinearVelocity(toJphBodyID(bodyID), maxVelocity);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_BodyInterface_GetMaxLinearVelocity(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return 0;
    JOLTC_TRY_BEGIN
    return bi(iface)->GetMaxLinearVelocity(toJphBodyID(bodyID));
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_BodyInterface_SetMaxAngularVelocity(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, float maxVelocity)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->SetMaxAngularVelocity(toJphBodyID(bodyID), maxVelocity);
    JOLTC_TRY_END
}

JOLTC_API float JoltC_BodyInterface_GetMaxAngularVelocity(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return 0;
    JOLTC_TRY_BEGIN
    return bi(iface)->GetMaxAngularVelocity(toJphBodyID(bodyID));
    JOLTC_TRY_END
    return 0;
}

JOLTC_API JoltC_Vec3 JoltC_BodyInterface_GetWorldSpaceSurfaceNormal(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID, uint32_t subShapeID, JoltC_RVec3 position)
{
    JoltC_Vec3 z = {0,0,0}; if (!bi(iface)) return z;
    JOLTC_TRY_BEGIN
    BodyLockRead lock(iface->system->GetBodyLockInterface(), toJphBodyID(bodyID));
    if (lock.Succeeded()) {
        SubShapeID ssid;
        ssid.SetValue(subShapeID);
        return fromJphVec3(lock.GetBody().GetWorldSpaceSurfaceNormal(ssid, toJphRVec3(position)));
    }
    JOLTC_TRY_END
    return z;
}

/* -------------------------------------------------------------------------- */
/*  Extended BodyInterface methods                                            */
/* -------------------------------------------------------------------------- */

static inline const Body* asBody(const JoltC_Body* b) {
    return reinterpret_cast<const Body*>(b);
}
static inline Body* asBodyMut(JoltC_Body* b) {
    return reinterpret_cast<Body*>(b);
}
static inline JoltC_Body* fromBody(Body* b) {
    return reinterpret_cast<JoltC_Body*>(b);
}

JOLTC_API JoltC_Body* JoltC_BodyInterface_CreateBodyDirect(JoltC_BodyInterface* iface, const JoltC_BodyCreationSettings* settings)
{
    if (!bi(iface) || !settings) return nullptr;
    JOLTC_TRY_BEGIN
    return fromBody(bi(iface)->CreateBody(toJphBCS(settings)));
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_Body* JoltC_BodyInterface_CreateBodyWithID(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, const JoltC_BodyCreationSettings* settings)
{
    if (!bi(iface) || !settings) return nullptr;
    JOLTC_TRY_BEGIN
    return fromBody(bi(iface)->CreateBodyWithID(toJphBodyID(bodyID), toJphBCS(settings)));
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_Body* JoltC_BodyInterface_CreateBodyWithoutID(JoltC_BodyInterface* iface, const JoltC_BodyCreationSettings* settings)
{
    if (!bi(iface) || !settings) return nullptr;
    JOLTC_TRY_BEGIN
    return fromBody(bi(iface)->CreateBodyWithoutID(toJphBCS(settings)));
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_BodyInterface_DestroyBodyWithoutID(JoltC_BodyInterface* iface, JoltC_Body* body)
{
    if (!bi(iface) || !body) return;
    JOLTC_TRY_BEGIN
    bi(iface)->DestroyBodyWithoutID(asBodyMut(body));
    JOLTC_TRY_END
}

JOLTC_API int JoltC_BodyInterface_AssignBodyID(JoltC_BodyInterface* iface, JoltC_Body* body)
{
    if (!bi(iface) || !body) return 0;
    JOLTC_TRY_BEGIN
    return bi(iface)->AssignBodyID(asBodyMut(body)) ? 1 : 0;
    JOLTC_TRY_END
    return 0;
}

JOLTC_API int JoltC_BodyInterface_AssignBodyIDWithID(JoltC_BodyInterface* iface, JoltC_Body* body, JoltC_BodyID desiredID)
{
    if (!bi(iface) || !body) return 0;
    JOLTC_TRY_BEGIN
    return bi(iface)->AssignBodyID(asBodyMut(body), toJphBodyID(desiredID)) ? 1 : 0;
    JOLTC_TRY_END
    return 0;
}

JOLTC_API JoltC_Body* JoltC_BodyInterface_UnassignBodyID(JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return nullptr;
    JOLTC_TRY_BEGIN
    return fromBody(bi(iface)->UnassignBodyID(toJphBodyID(bodyID)));
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_BodyInterface_RemoveAndDestroyBody(JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->RemoveBody(toJphBodyID(bodyID));
    bi(iface)->DestroyBody(toJphBodyID(bodyID));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_SetPositionAndRotationWhenChanged(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_RVec3 position, JoltC_Quat rotation, JoltC_Activation activation)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->SetPositionAndRotationWhenChanged(toJphBodyID(bodyID), toJphRVec3(position), toJphQuat(rotation), toJphActivation(activation));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_SetPositionRotationAndVelocity(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_RVec3 position, JoltC_Quat rotation, JoltC_Vec3 linearVelocity, JoltC_Vec3 angularVelocity)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->SetPositionRotationAndVelocity(toJphBodyID(bodyID), toJphRVec3(position), toJphQuat(rotation), toJphVec3(linearVelocity), toJphVec3(angularVelocity));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_GetLinearAndAngularVelocity(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_Vec3* outLinear, JoltC_Vec3* outAngular)
{
    if (!bi(iface) || !outLinear || !outAngular) return;
    JOLTC_TRY_BEGIN
    Vec3 lin, ang;
    bi(iface)->GetLinearAndAngularVelocity(toJphBodyID(bodyID), lin, ang);
    *outLinear = fromJphVec3(lin);
    *outAngular = fromJphVec3(ang);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_AddLinearAndAngularVelocity(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_Vec3 linearVelocity, JoltC_Vec3 angularVelocity)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->AddLinearAndAngularVelocity(toJphBodyID(bodyID), toJphVec3(linearVelocity), toJphVec3(angularVelocity));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_AddForceAndTorque(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_Vec3 force, JoltC_Vec3 torque)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->AddForceAndTorque(toJphBodyID(bodyID), toJphVec3(force), toJphVec3(torque));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_GetInverseInertia(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_Mat44* result)
{
    if (!bi(iface) || !result) return;
    JOLTC_TRY_BEGIN
    *result = fromJphMat44(bi(iface)->GetInverseInertia(toJphBodyID(bodyID)));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_ActivateBodies(JoltC_BodyInterface* iface, const JoltC_BodyID* bodyIDs, uint32_t count)
{
    if (!bi(iface) || !bodyIDs || count == 0) return;
    JOLTC_TRY_BEGIN
    bi(iface)->ActivateBodies(reinterpret_cast<const BodyID*>(bodyIDs), static_cast<int>(count));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_DeactivateBodies(JoltC_BodyInterface* iface, const JoltC_BodyID* bodyIDs, uint32_t count)
{
    if (!bi(iface) || !bodyIDs || count == 0) return;
    JOLTC_TRY_BEGIN
    bi(iface)->DeactivateBodies(reinterpret_cast<const BodyID*>(bodyIDs), static_cast<int>(count));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_ResetSleepTimer(JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->ResetSleepTimer(toJphBodyID(bodyID));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_SetUseManifoldReduction(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, int value)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->SetUseManifoldReduction(toJphBodyID(bodyID), value != 0);
    JOLTC_TRY_END
}

JOLTC_API int JoltC_BodyInterface_GetUseManifoldReduction(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return 0;
    JOLTC_TRY_BEGIN
    return bi(iface)->GetUseManifoldReduction(toJphBodyID(bodyID)) ? 1 : 0;
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_BodyInterface_SetIsSensor(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, int value)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    BodyLockWrite lock(iface->system->GetBodyLockInterface(), toJphBodyID(bodyID));
    if (lock.Succeeded())
        lock.GetBody().SetIsSensor(value != 0);
    JOLTC_TRY_END
}

JOLTC_API int JoltC_BodyInterface_IsSensor(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return 0;
    JOLTC_TRY_BEGIN
    BodyLockRead lock(iface->system->GetBodyLockInterface(), toJphBodyID(bodyID));
    if (lock.Succeeded())
        return lock.GetBody().IsSensor() ? 1 : 0;
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_BodyInterface_InvalidateContactCache(JoltC_BodyInterface* iface, JoltC_BodyID bodyID)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->InvalidateContactCache(toJphBodyID(bodyID));
    JOLTC_TRY_END
}

JOLTC_API int JoltC_BodyInterface_ApplyBuoyancyImpulse(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_RVec3 surfacePosition, JoltC_Vec3 surfaceNormal, float buoyancy, float linearDrag, float angularDrag, JoltC_Vec3 fluidVelocity, JoltC_Vec3 gravity, float deltaTime)
{
    if (!bi(iface)) return 0;
    JOLTC_TRY_BEGIN
    return bi(iface)->ApplyBuoyancyImpulse(toJphBodyID(bodyID), toJphRVec3(surfacePosition), toJphVec3(surfaceNormal), buoyancy, linearDrag, angularDrag, toJphVec3(fluidVelocity), toJphVec3(gravity), deltaTime) ? 1 : 0;
    JOLTC_TRY_END
    return 0;
}

/* -------------------------------------------------------------------------- */
/*  BodyInterface — collision group                                           */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_BodyInterface_GetCollisionGroup(const JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_CollisionGroup* outGroup)
{
    if (!bi(iface) || !outGroup) return;
    JOLTC_TRY_BEGIN
    BodyLockRead lock(iface->system->GetBodyLockInterface(), toJphBodyID(bodyID));
    if (lock.Succeeded()) {
        const CollisionGroup& cg = lock.GetBody().GetCollisionGroup();
        outGroup->groupFilter = reinterpret_cast<const JoltC_GroupFilter*>(cg.GetGroupFilter());
        outGroup->groupID     = cg.GetGroupID();
        outGroup->subGroupID  = cg.GetSubGroupID();
    }
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_SetCollisionGroup(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, const JoltC_CollisionGroup* group)
{
    if (!bi(iface) || !group) return;
    JOLTC_TRY_BEGIN
    CollisionGroup cg;
    if (group->groupFilter)
        cg.SetGroupFilter(reinterpret_cast<const GroupFilter*>(group->groupFilter));
    cg.SetGroupID(group->groupID);
    cg.SetSubGroupID(group->subGroupID);
    BodyLockWrite lock(iface->system->GetBodyLockInterface(), toJphBodyID(bodyID));
    if (lock.Succeeded())
        lock.GetBody().SetCollisionGroup(cg);
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  BodyInterface — additional methods (batch 2)                              */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_BodyInterface_ActivateBodiesInAABox(JoltC_BodyInterface* iface, JoltC_Vec3 min, JoltC_Vec3 max)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    AABox box(toJphVec3(min), toJphVec3(max));
    bi(iface)->ActivateBodiesInAABox(box, BroadPhaseLayerFilter(), ObjectLayerFilter());
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_AddForce2(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_Vec3 force, JoltC_RVec3 point)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->AddForce(toJphBodyID(bodyID), toJphVec3(force), toJphRVec3(point));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyInterface_AddImpulse2(JoltC_BodyInterface* iface, JoltC_BodyID bodyID, JoltC_Vec3 impulse, JoltC_RVec3 point)
{
    if (!bi(iface)) return;
    JOLTC_TRY_BEGIN
    bi(iface)->AddImpulse(toJphBodyID(bodyID), toJphVec3(impulse), toJphRVec3(point));
    JOLTC_TRY_END
}

JOLTC_API int JoltC_BodyInterface_AssignBodyID2(JoltC_BodyInterface* iface, JoltC_Body* body, JoltC_BodyID desiredID)
{
    if (!bi(iface) || !body) return 0;
    JOLTC_TRY_BEGIN
    return bi(iface)->AssignBodyID(asBodyMut(body), toJphBodyID(desiredID)) ? 1 : 0;
    JOLTC_TRY_END
    return 0;
}

/* -------------------------------------------------------------------------- */
/*  Body — direct access                                                      */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_Body_GetCollisionGroup(const JoltC_Body* body, JoltC_CollisionGroup* outGroup)
{
    if (!body || !outGroup) return;
    JOLTC_TRY_BEGIN
    const Body* b = asBody(body);
    const CollisionGroup& cg = b->GetCollisionGroup();
    outGroup->groupFilter = reinterpret_cast<const JoltC_GroupFilter*>(cg.GetGroupFilter());
    outGroup->groupID     = cg.GetGroupID();
    outGroup->subGroupID  = cg.GetSubGroupID();
    JOLTC_TRY_END
}

JOLTC_API void JoltC_Body_SetCollisionGroup(JoltC_Body* body, const JoltC_CollisionGroup* group)
{
    if (!body || !group) return;
    JOLTC_TRY_BEGIN
    Body* b = asBodyMut(body);
    CollisionGroup cg;
    if (group->groupFilter)
        cg.SetGroupFilter(reinterpret_cast<const GroupFilter*>(group->groupFilter));
    cg.SetGroupID(group->groupID);
    cg.SetSubGroupID(group->subGroupID);
    b->SetCollisionGroup(cg);
    JOLTC_TRY_END
}

JOLTC_API const JoltC_Body* JoltC_Body_GetFixedToWorldBody(void)
{
    return reinterpret_cast<const JoltC_Body*>(&Body::sFixedToWorld);
}

/* -------------------------------------------------------------------------- */
/*  BodyInterface — SoftBody creation                                         */
/* -------------------------------------------------------------------------- */
static inline JoltC_SoftBodyCreationSettings_Impl* asSBCS(const JoltC_SoftBodyCreationSettings* s) {
    return reinterpret_cast<JoltC_SoftBodyCreationSettings_Impl*>(const_cast<JoltC_SoftBodyCreationSettings*>(s));
}

JOLTC_API JoltC_Body* JoltC_BodyInterface_CreateSoftBody(JoltC_BodyInterface* iface, const JoltC_SoftBodyCreationSettings* settings)
{
    if (!bi(iface) || !settings) return nullptr;
    JOLTC_TRY_BEGIN
    return fromBody(bi(iface)->CreateSoftBody(asSBCS(settings)->settings));
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_Body* JoltC_BodyInterface_CreateSoftBodyWithID(JoltC_BodyInterface* iface, const JoltC_SoftBodyCreationSettings* settings, uint32_t bodyId)
{
    if (!bi(iface) || !settings) return nullptr;
    JOLTC_TRY_BEGIN
    return fromBody(bi(iface)->CreateSoftBodyWithID(toJphBodyID(bodyId), asSBCS(settings)->settings));
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_Body* JoltC_BodyInterface_CreateSoftBodyWithoutID(JoltC_BodyInterface* iface, const JoltC_SoftBodyCreationSettings* settings)
{
    if (!bi(iface) || !settings) return nullptr;
    JOLTC_TRY_BEGIN
    return fromBody(bi(iface)->CreateSoftBodyWithoutID(asSBCS(settings)->settings));
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_BodyID JoltC_BodyInterface_CreateAndAddSoftBody(JoltC_BodyInterface* iface, const JoltC_SoftBodyCreationSettings* settings, JoltC_Activation activation)
{
    if (!bi(iface) || !settings) return JOLTC_BODY_ID_INVALID;
    JOLTC_TRY_BEGIN
    BodyID id = bi(iface)->CreateAndAddSoftBody(asSBCS(settings)->settings, toJphActivation(activation));
    return fromJphBodyID(id);
    JOLTC_TRY_END
    return JOLTC_BODY_ID_INVALID;
}

} /* extern "C" */
