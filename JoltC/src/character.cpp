/* JoltC - CharacterVirtual implementation
 * SPDX-License-Identifier: MIT
 */

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Character/Character.h>

#include "errors_internal.h"
#include "internal.h"
#include "wrappers.h"

#include <JoltC/character.h>

using namespace JPH;

/* -------------------------------------------------------------------------- */
/*  Helper casts                                                              */
/* -------------------------------------------------------------------------- */
static CharacterVirtual* cv(JoltC_CharacterVirtual* c) { return c->ptr.GetPtr(); }
static const CharacterVirtual* cv(const JoltC_CharacterVirtual* c) { return c->ptr.GetPtr(); }

static Character* ch(JoltC_Character* c) { return c->ptr.GetPtr(); }
static const Character* ch(const JoltC_Character* c) { return c->ptr.GetPtr(); }

static CharacterBase* cb(JoltC_CharacterBase* c) { return c->ptr.GetPtr(); }
static const CharacterBase* cb(const JoltC_CharacterBase* c) { return c->ptr.GetPtr(); }

extern "C" {

/* -------------------------------------------------------------------------- */
/*  Settings defaults                                                         */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_CharacterVirtualSettings_SetDefault(JoltC_CharacterVirtualSettings* s) {
    if (!s) return;
    s->up = JoltC_Vec3{0, 1, 0};
    s->maxSlopeAngle = 50.0f * (3.14159265358979323846f / 180.0f);
    s->enhancedInternalEdgeRemoval = JOLTC_FALSE;
    s->shape = nullptr;
    s->mass = 70.0f;
    s->maxStrength = 100.0f;
    s->shapeOffset = JoltC_Vec3{0, 0, 0};
    s->backFaceMode = JOLTC_BACK_FACE_COLLIDE;
    s->predictiveContactDistance = 0.1f;
    s->maxCollisionIterations = 5;
    s->maxConstraintIterations = 15;
    s->minTimeRemaining = 1.0e-4f;
    s->collisionTolerance = 1.0e-3f;
    s->characterPadding = 0.02f;
    s->maxNumHits = 256;
    s->hitReductionCosMaxAngle = 0.999f;
    s->penetrationRecoverySpeed = 1.0f;
    s->innerBodyShape = nullptr;
    s->innerBodyLayer = 0;
}

JOLTC_API void JoltC_ExtendedUpdateSettings_SetDefault(JoltC_ExtendedUpdateSettings* s) {
    if (!s) return;
    s->stickToFloorStepDown = JoltC_Vec3{0, -0.5f, 0};
    s->walkStairsStepUp = JoltC_Vec3{0, 0.4f, 0};
    s->walkStairsMinStepForward = 0.02f;
    s->walkStairsStepForwardTest = 0.15f;
    s->walkStairsCosAngleForwardContact = 0.258819f; /* cos(75 degrees) */
    s->walkStairsStepDownExtra = JoltC_Vec3{0, 0, 0};
}

/* -------------------------------------------------------------------------- */
/*  CharacterContactListener                                                  */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_CharacterContactListener* JoltC_CharacterContactListener_Create(
    JoltC_OnCharacterContactValidateFn onValidate,
    JoltC_OnCharacterContactAddedFn onAdded,
    JoltC_OnCharacterContactPersistedFn onPersisted,
    void* userData)
{
    JOLTC_TRY_BEGIN
    auto* wrapper = new JoltC_CharacterContactListener();
    wrapper->ptr = std::make_unique<CharacterContactListenerCallback>();
    wrapper->ptr->fnValidate = onValidate;
    wrapper->ptr->fnAdded = onAdded;
    wrapper->ptr->fnPersisted = onPersisted;
    wrapper->ptr->userData = userData;
    return wrapper;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_CharacterContactListener_Destroy(JoltC_CharacterContactListener* listener) {
    delete listener;
}

/* -------------------------------------------------------------------------- */
/*  CharacterVirtual create / destroy                                         */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_CharacterVirtual* JoltC_CharacterVirtual_Create(
    const JoltC_CharacterVirtualSettings* s,
    JoltC_RVec3 position, JoltC_Quat rotation,
    uint64_t userData,
    JoltC_PhysicsSystem* system)
{
    if (!s || !system || !s->shape) return nullptr;
    JOLTC_TRY_BEGIN
    CharacterVirtualSettings settings;
    settings.mUp = toJphVec3(s->up);
    settings.mMaxSlopeAngle = s->maxSlopeAngle;
    settings.mEnhancedInternalEdgeRemoval = s->enhancedInternalEdgeRemoval != 0;
    settings.mShape = reinterpret_cast<const Shape*>(s->shape);
    settings.mMass = s->mass;
    settings.mMaxStrength = s->maxStrength;
    settings.mShapeOffset = toJphVec3(s->shapeOffset);
    settings.mBackFaceMode = toJphBackFaceMode(s->backFaceMode);
    settings.mPredictiveContactDistance = s->predictiveContactDistance;
    settings.mMaxCollisionIterations = s->maxCollisionIterations;
    settings.mMaxConstraintIterations = s->maxConstraintIterations;
    settings.mMinTimeRemaining = s->minTimeRemaining;
    settings.mCollisionTolerance = s->collisionTolerance;
    settings.mCharacterPadding = s->characterPadding;
    settings.mMaxNumHits = s->maxNumHits;
    settings.mHitReductionCosMaxAngle = s->hitReductionCosMaxAngle;
    settings.mPenetrationRecoverySpeed = s->penetrationRecoverySpeed;
    if (s->innerBodyShape)
        settings.mInnerBodyShape = reinterpret_cast<const Shape*>(s->innerBodyShape);
    settings.mInnerBodyLayer = s->innerBodyLayer;

    auto* ch = new CharacterVirtual(&settings, toJphRVec3(position), toJphQuat(rotation), userData, system->ptr.get());
    auto* wrapper = new JoltC_CharacterVirtual();
    wrapper->ptr = ch;
    return wrapper;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_CharacterVirtual_Destroy(JoltC_CharacterVirtual* c) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    c->ptr = nullptr;
    delete c;
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  CharacterVirtual methods                                                  */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_CharacterVirtual_SetListener(JoltC_CharacterVirtual* c, JoltC_CharacterContactListener* listener) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    c->listener = listener;
    cv(c)->SetListener(listener ? listener->ptr.get() : nullptr);
    JOLTC_TRY_END
}

JOLTC_API JoltC_Vec3 JoltC_CharacterVirtual_GetLinearVelocity(const JoltC_CharacterVirtual* c) {
    JoltC_Vec3 z = {0,0,0}; if (!c) return z;
    JOLTC_TRY_BEGIN return fromJphVec3(cv(c)->GetLinearVelocity()); JOLTC_TRY_END return z;
}
JOLTC_API void JoltC_CharacterVirtual_SetLinearVelocity(JoltC_CharacterVirtual* c, JoltC_Vec3 v) {
    if (!c) return; JOLTC_TRY_BEGIN cv(c)->SetLinearVelocity(toJphVec3(v)); JOLTC_TRY_END
}
JOLTC_API JoltC_RVec3 JoltC_CharacterVirtual_GetPosition(const JoltC_CharacterVirtual* c) {
    JoltC_RVec3 z = {0,0,0}; if (!c) return z;
    JOLTC_TRY_BEGIN return fromJphRVec3(cv(c)->GetPosition()); JOLTC_TRY_END return z;
}
JOLTC_API void JoltC_CharacterVirtual_SetPosition(JoltC_CharacterVirtual* c, JoltC_RVec3 p) {
    if (!c) return; JOLTC_TRY_BEGIN cv(c)->SetPosition(toJphRVec3(p)); JOLTC_TRY_END
}
JOLTC_API JoltC_Quat JoltC_CharacterVirtual_GetRotation(const JoltC_CharacterVirtual* c) {
    JoltC_Quat z = {0,0,0,1}; if (!c) return z;
    JOLTC_TRY_BEGIN return fromJphQuat(cv(c)->GetRotation()); JOLTC_TRY_END return z;
}
JOLTC_API void JoltC_CharacterVirtual_SetRotation(JoltC_CharacterVirtual* c, JoltC_Quat q) {
    if (!c) return; JOLTC_TRY_BEGIN cv(c)->SetRotation(toJphQuat(q)); JOLTC_TRY_END
}
JOLTC_API JoltC_RVec3 JoltC_CharacterVirtual_GetCenterOfMassPosition(const JoltC_CharacterVirtual* c) {
    JoltC_RVec3 z = {0,0,0}; if (!c) return z;
    JOLTC_TRY_BEGIN return fromJphRVec3(cv(c)->GetCenterOfMassPosition()); JOLTC_TRY_END return z;
}

JOLTC_API float JoltC_CharacterVirtual_GetMass(const JoltC_CharacterVirtual* c) { if (!c) return 0; JOLTC_TRY_BEGIN return cv(c)->GetMass(); JOLTC_TRY_END return 0; }
JOLTC_API void JoltC_CharacterVirtual_SetMass(JoltC_CharacterVirtual* c, float m) { if (!c) return; JOLTC_TRY_BEGIN cv(c)->SetMass(m); JOLTC_TRY_END }
JOLTC_API float JoltC_CharacterVirtual_GetMaxStrength(const JoltC_CharacterVirtual* c) { if (!c) return 0; JOLTC_TRY_BEGIN return cv(c)->GetMaxStrength(); JOLTC_TRY_END return 0; }
JOLTC_API void JoltC_CharacterVirtual_SetMaxStrength(JoltC_CharacterVirtual* c, float s) { if (!c) return; JOLTC_TRY_BEGIN cv(c)->SetMaxStrength(s); JOLTC_TRY_END }
JOLTC_API float JoltC_CharacterVirtual_GetPenetrationRecoverySpeed(const JoltC_CharacterVirtual* c) { if (!c) return 0; JOLTC_TRY_BEGIN return cv(c)->GetPenetrationRecoverySpeed(); JOLTC_TRY_END return 0; }
JOLTC_API void JoltC_CharacterVirtual_SetPenetrationRecoverySpeed(JoltC_CharacterVirtual* c, float s) { if (!c) return; JOLTC_TRY_BEGIN cv(c)->SetPenetrationRecoverySpeed(s); JOLTC_TRY_END }
JOLTC_API float JoltC_CharacterVirtual_GetCharacterPadding(const JoltC_CharacterVirtual* c) { if (!c) return 0; JOLTC_TRY_BEGIN return cv(c)->GetCharacterPadding(); JOLTC_TRY_END return 0; }
JOLTC_API uint32_t JoltC_CharacterVirtual_GetMaxNumHits(const JoltC_CharacterVirtual* c) { if (!c) return 0; JOLTC_TRY_BEGIN return cv(c)->GetMaxNumHits(); JOLTC_TRY_END return 0; }
JOLTC_API void JoltC_CharacterVirtual_SetMaxNumHits(JoltC_CharacterVirtual* c, uint32_t n) { if (!c) return; JOLTC_TRY_BEGIN cv(c)->SetMaxNumHits(n); JOLTC_TRY_END }

JOLTC_API uint64_t JoltC_CharacterVirtual_GetUserData(const JoltC_CharacterVirtual* c) { if (!c) return 0; JOLTC_TRY_BEGIN return cv(c)->GetUserData(); JOLTC_TRY_END return 0; }
JOLTC_API void JoltC_CharacterVirtual_SetUserData(JoltC_CharacterVirtual* c, uint64_t d) { if (!c) return; JOLTC_TRY_BEGIN cv(c)->SetUserData(d); JOLTC_TRY_END }
JOLTC_API JoltC_BodyID JoltC_CharacterVirtual_GetInnerBodyID(const JoltC_CharacterVirtual* c) { if (!c) return JOLTC_BODY_ID_INVALID; JOLTC_TRY_BEGIN return fromJphBodyID(cv(c)->GetInnerBodyID()); JOLTC_TRY_END return JOLTC_BODY_ID_INVALID; }

/* -------------------------------------------------------------------------- */
/*  Ground state                                                              */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_GroundState JoltC_CharacterVirtual_GetGroundState(const JoltC_CharacterVirtual* c) { if (!c) return JOLTC_GROUND_STATE_IN_AIR; JOLTC_TRY_BEGIN return static_cast<JoltC_GroundState>(cv(c)->GetGroundState()); JOLTC_TRY_END return JOLTC_GROUND_STATE_IN_AIR; }
JOLTC_API JoltC_Bool JoltC_CharacterVirtual_IsSupported(const JoltC_CharacterVirtual* c) { if (!c) return JOLTC_FALSE; JOLTC_TRY_BEGIN return cv(c)->IsSupported() ? JOLTC_TRUE : JOLTC_FALSE; JOLTC_TRY_END return JOLTC_FALSE; }
JOLTC_API JoltC_RVec3 JoltC_CharacterVirtual_GetGroundPosition(const JoltC_CharacterVirtual* c) { JoltC_RVec3 z={0,0,0}; if (!c) return z; JOLTC_TRY_BEGIN return fromJphRVec3(cv(c)->GetGroundPosition()); JOLTC_TRY_END return z; }
JOLTC_API JoltC_Vec3 JoltC_CharacterVirtual_GetGroundNormal(const JoltC_CharacterVirtual* c) { JoltC_Vec3 z={0,0,0}; if (!c) return z; JOLTC_TRY_BEGIN return fromJphVec3(cv(c)->GetGroundNormal()); JOLTC_TRY_END return z; }
JOLTC_API JoltC_Vec3 JoltC_CharacterVirtual_GetGroundVelocity(const JoltC_CharacterVirtual* c) { JoltC_Vec3 z={0,0,0}; if (!c) return z; JOLTC_TRY_BEGIN return fromJphVec3(cv(c)->GetGroundVelocity()); JOLTC_TRY_END return z; }
JOLTC_API JoltC_BodyID JoltC_CharacterVirtual_GetGroundBodyID(const JoltC_CharacterVirtual* c) { if (!c) return JOLTC_BODY_ID_INVALID; JOLTC_TRY_BEGIN return fromJphBodyID(cv(c)->GetGroundBodyID()); JOLTC_TRY_END return JOLTC_BODY_ID_INVALID; }
JOLTC_API const JoltC_PhysicsMaterial* JoltC_CharacterVirtual_GetGroundMaterial(const JoltC_CharacterVirtual* c) { if (!c) return nullptr; JOLTC_TRY_BEGIN return fromPhysicsMaterial(cv(c)->GetGroundMaterial()); JOLTC_TRY_END return nullptr; }

JOLTC_API JoltC_Vec3 JoltC_CharacterVirtual_GetUp(const JoltC_CharacterVirtual* c) { JoltC_Vec3 z={0,0,0}; if (!c) return z; JOLTC_TRY_BEGIN return fromJphVec3(cv(c)->GetUp()); JOLTC_TRY_END return z; }
JOLTC_API void JoltC_CharacterVirtual_SetUp(JoltC_CharacterVirtual* c, JoltC_Vec3 up) { if (!c) return; JOLTC_TRY_BEGIN cv(c)->SetUp(toJphVec3(up)); JOLTC_TRY_END }
JOLTC_API void JoltC_CharacterVirtual_SetMaxSlopeAngle(JoltC_CharacterVirtual* c, float a) { if (!c) return; JOLTC_TRY_BEGIN cv(c)->SetMaxSlopeAngle(a); JOLTC_TRY_END }

JOLTC_API const JoltC_Shape* JoltC_CharacterVirtual_GetShape(const JoltC_CharacterVirtual* c) {
    if (!c) return nullptr;
    JOLTC_TRY_BEGIN
    return reinterpret_cast<const JoltC_Shape*>(cv(c)->GetShape());
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_Bool JoltC_CharacterVirtual_SetShape(JoltC_CharacterVirtual* c, const JoltC_Shape* shape, float maxPenetrationDepth, JoltC_TempAllocator* allocator) {
    if (!c || !shape || !allocator) return JOLTC_FALSE;
    JOLTC_TRY_BEGIN
    bool result = cv(c)->SetShape(
        reinterpret_cast<const Shape*>(shape),
        maxPenetrationDepth,
        {}, {}, {}, {},
        *allocator->ptr);
    return result ? JOLTC_TRUE : JOLTC_FALSE;
    JOLTC_TRY_END
    return JOLTC_FALSE;
}

/* -------------------------------------------------------------------------- */
/*  Simulation                                                                */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_CharacterVirtual_Update(
    JoltC_CharacterVirtual* c, float dt, JoltC_Vec3 gravity,
    JoltC_TempAllocator* allocator)
{
    if (!c || !allocator) return;
    JOLTC_TRY_BEGIN
    cv(c)->Update(dt, toJphVec3(gravity), {}, {}, {}, {}, *allocator->ptr);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_CharacterVirtual_ExtendedUpdate(
    JoltC_CharacterVirtual* c, float dt, JoltC_Vec3 gravity,
    const JoltC_ExtendedUpdateSettings* s,
    JoltC_TempAllocator* allocator)
{
    if (!c || !s || !allocator) return;
    JOLTC_TRY_BEGIN
    CharacterVirtual::ExtendedUpdateSettings eus;
    eus.mStickToFloorStepDown = toJphVec3(s->stickToFloorStepDown);
    eus.mWalkStairsStepUp = toJphVec3(s->walkStairsStepUp);
    eus.mWalkStairsMinStepForward = s->walkStairsMinStepForward;
    eus.mWalkStairsStepForwardTest = s->walkStairsStepForwardTest;
    eus.mWalkStairsCosAngleForwardContact = s->walkStairsCosAngleForwardContact;
    eus.mWalkStairsStepDownExtra = toJphVec3(s->walkStairsStepDownExtra);
    cv(c)->ExtendedUpdate(dt, toJphVec3(gravity), eus, {}, {}, {}, {}, *allocator->ptr);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_CharacterVirtual_RefreshContacts(
    JoltC_CharacterVirtual* c, JoltC_TempAllocator* allocator)
{
    if (!c || !allocator) return;
    JOLTC_TRY_BEGIN
    cv(c)->RefreshContacts({}, {}, {}, {}, *allocator->ptr);
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  Additional CharacterBase/CharacterVirtual accessors                       */
/* -------------------------------------------------------------------------- */
JOLTC_API float JoltC_CharacterVirtual_GetCosMaxSlopeAngle(const JoltC_CharacterVirtual* c) {
    if (!c) return 0;
    return cv(c)->GetCosMaxSlopeAngle();
}

JOLTC_API JoltC_Bool JoltC_CharacterVirtual_IsSlopeTooSteep(const JoltC_CharacterVirtual* c, JoltC_Vec3 normal) {
    if (!c) return JOLTC_FALSE;
    return cv(c)->IsSlopeTooSteep(toJphVec3(normal)) ? JOLTC_TRUE : JOLTC_FALSE;
}

JOLTC_API JoltC_SubShapeID JoltC_CharacterVirtual_GetGroundSubShapeID(const JoltC_CharacterVirtual* c) {
    if (!c) return 0;
    return cv(c)->GetGroundSubShapeID().GetValue();
}

JOLTC_API uint64_t JoltC_CharacterVirtual_GetGroundUserData(const JoltC_CharacterVirtual* c) {
    if (!c) return 0;
    return cv(c)->GetGroundUserData();
}

JOLTC_API JoltC_Mat44 JoltC_CharacterVirtual_GetWorldTransform(const JoltC_CharacterVirtual* c) {
    if (!c) return JoltC_Mat44{};
    return fromJphMat44(cv(c)->GetWorldTransform());
}

JOLTC_API JoltC_Mat44 JoltC_CharacterVirtual_GetCenterOfMassTransform(const JoltC_CharacterVirtual* c) {
    if (!c) return JoltC_Mat44{};
    return fromJphMat44(cv(c)->GetCenterOfMassTransform());
}

JOLTC_API JoltC_Bool JoltC_CharacterVirtual_GetEnhancedInternalEdgeRemoval(const JoltC_CharacterVirtual* c) {
    if (!c) return JOLTC_FALSE;
    return cv(c)->GetEnhancedInternalEdgeRemoval() ? JOLTC_TRUE : JOLTC_FALSE;
}

JOLTC_API void JoltC_CharacterVirtual_SetEnhancedInternalEdgeRemoval(JoltC_CharacterVirtual* c, JoltC_Bool value) {
    if (!c) return;
    cv(c)->SetEnhancedInternalEdgeRemoval(value != 0);
}

JOLTC_API float JoltC_CharacterVirtual_GetHitReductionCosMaxAngle(const JoltC_CharacterVirtual* c) {
    if (!c) return 0;
    return cv(c)->GetHitReductionCosMaxAngle();
}

JOLTC_API void JoltC_CharacterVirtual_SetHitReductionCosMaxAngle(JoltC_CharacterVirtual* c, float value) {
    if (!c) return;
    cv(c)->SetHitReductionCosMaxAngle(value);
}

JOLTC_API JoltC_Bool JoltC_CharacterVirtual_GetMaxHitsExceeded(const JoltC_CharacterVirtual* c) {
    if (!c) return JOLTC_FALSE;
    return cv(c)->GetMaxHitsExceeded() ? JOLTC_TRUE : JOLTC_FALSE;
}

JOLTC_API JoltC_Vec3 JoltC_CharacterVirtual_GetShapeOffset(const JoltC_CharacterVirtual* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(cv(c)->GetShapeOffset());
}

JOLTC_API void JoltC_CharacterVirtual_SetShapeOffset(JoltC_CharacterVirtual* c, JoltC_Vec3 value) {
    if (!c) return;
    cv(c)->SetShapeOffset(toJphVec3(value));
}

JOLTC_API JoltC_Vec3 JoltC_CharacterVirtual_CancelVelocityTowardsSteepSlopes(const JoltC_CharacterVirtual* c, JoltC_Vec3 desiredVelocity) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(cv(c)->CancelVelocityTowardsSteepSlopes(toJphVec3(desiredVelocity)));
}

JOLTC_API void JoltC_CharacterVirtual_UpdateGroundVelocity(JoltC_CharacterVirtual* c) {
    if (!c) return;
    cv(c)->UpdateGroundVelocity();
}

JOLTC_API JoltC_Bool JoltC_CharacterVirtual_HasCollidedWithBody(const JoltC_CharacterVirtual* c, JoltC_BodyID bodyID) {
    if (!c) return JOLTC_FALSE;
    return cv(c)->HasCollidedWith(toJphBodyID(bodyID)) ? JOLTC_TRUE : JOLTC_FALSE;
}

JOLTC_API uint32_t JoltC_CharacterVirtual_GetNumActiveContacts(const JoltC_CharacterVirtual* c) {
    if (!c) return 0;
    return static_cast<uint32_t>(cv(c)->GetActiveContacts().size());
}

JOLTC_API void JoltC_CharacterVirtual_SetInnerBodyShape(JoltC_CharacterVirtual* c, const JoltC_Shape* shape) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    cv(c)->SetInnerBodyShape(reinterpret_cast<const Shape*>(shape));
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */

/*  CharacterSettings defaults                                                */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_CharacterSettings_SetDefault(JoltC_CharacterSettings* s) {
    if (!s) return;
    s->up = JoltC_Vec3{0, 1, 0};
    s->maxSlopeAngle = 50.0f * (3.14159265358979323846f / 180.0f);
    s->enhancedInternalEdgeRemoval = JOLTC_FALSE;
    s->shape = nullptr;
    s->layer = 0;
    s->mass = 80.0f;
    s->friction = 0.2f;
    s->gravityFactor = 1.0f;
}

/* -------------------------------------------------------------------------- */
/*  Character create / destroy                                                */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_Character* JoltC_Character_Create(
    const JoltC_CharacterSettings* settings,
    JoltC_RVec3 position, JoltC_Quat rotation,
    uint64_t userData, JoltC_PhysicsSystem* system)
{
    if (!settings || !system) return nullptr;
    JOLTC_TRY_BEGIN
    CharacterSettings jphSettings;
    jphSettings.mUp = toJphVec3(settings->up);
    jphSettings.mMaxSlopeAngle = settings->maxSlopeAngle;
    jphSettings.mEnhancedInternalEdgeRemoval = settings->enhancedInternalEdgeRemoval != 0;
    if (settings->shape)
        jphSettings.mShape = reinterpret_cast<const Shape*>(settings->shape);
    jphSettings.mLayer = settings->layer;
    jphSettings.mMass = settings->mass;
    jphSettings.mFriction = settings->friction;
    jphSettings.mGravityFactor = settings->gravityFactor;

    auto* w = new JoltC_Character;
    w->ptr = new Character(&jphSettings, toJphRVec3(position), toJphQuat(rotation), userData, system->ptr.get());
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_Character_Destroy(JoltC_Character* character) {
    delete character;
}

JOLTC_API void JoltC_Character_AddToPhysicsSystem(JoltC_Character* c, JoltC_Activation activation, JoltC_Bool lockBodies) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    ch(c)->AddToPhysicsSystem(toJphActivation(activation), lockBodies != 0);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_Character_RemoveFromPhysicsSystem(JoltC_Character* c, JoltC_Bool lockBodies) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    ch(c)->RemoveFromPhysicsSystem(lockBodies != 0);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_Character_Activate(JoltC_Character* c, JoltC_Bool lockBodies) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    ch(c)->Activate(lockBodies != 0);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_Character_PostSimulation(JoltC_Character* c, float maxSeparationDistance, JoltC_Bool lockBodies) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    ch(c)->PostSimulation(maxSeparationDistance, lockBodies != 0);
    JOLTC_TRY_END
}

JOLTC_API JoltC_Vec3 JoltC_Character_GetLinearVelocity(const JoltC_Character* c, JoltC_Bool lockBodies) {
    if (!c) return JoltC_Vec3{0,0,0};
    JOLTC_TRY_BEGIN
    return fromJphVec3(ch(c)->GetLinearVelocity(lockBodies != 0));
    JOLTC_TRY_END
    return JoltC_Vec3{0,0,0};
}

JOLTC_API void JoltC_Character_SetLinearVelocity(JoltC_Character* c, JoltC_Vec3 velocity, JoltC_Bool lockBodies) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    ch(c)->SetLinearVelocity(toJphVec3(velocity), lockBodies != 0);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_Character_AddLinearVelocity(JoltC_Character* c, JoltC_Vec3 velocity, JoltC_Bool lockBodies) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    ch(c)->AddLinearVelocity(toJphVec3(velocity), lockBodies != 0);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_Character_AddImpulse(JoltC_Character* c, JoltC_Vec3 impulse, JoltC_Bool lockBodies) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    ch(c)->AddImpulse(toJphVec3(impulse), lockBodies != 0);
    JOLTC_TRY_END
}

JOLTC_API JoltC_BodyID JoltC_Character_GetBodyID(const JoltC_Character* c) {
    if (!c) return JOLTC_BODY_ID_INVALID;
    return fromJphBodyID(ch(c)->GetBodyID());
}

JOLTC_API JoltC_RVec3 JoltC_Character_GetPosition(const JoltC_Character* c, JoltC_Bool lockBodies) {
    if (!c) { JoltC_RVec3 r = {0,0,0}; return r; }
    JOLTC_TRY_BEGIN
    return fromJphRVec3(ch(c)->GetPosition(lockBodies != 0));
    JOLTC_TRY_END
    JoltC_RVec3 r = {0,0,0}; return r;
}

JOLTC_API void JoltC_Character_SetPosition(JoltC_Character* c, JoltC_RVec3 position, JoltC_Activation activation, JoltC_Bool lockBodies) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    ch(c)->SetPosition(toJphRVec3(position), toJphActivation(activation), lockBodies != 0);
    JOLTC_TRY_END
}

JOLTC_API JoltC_Quat JoltC_Character_GetRotation(const JoltC_Character* c, JoltC_Bool lockBodies) {
    if (!c) return JoltC_Quat{0,0,0,1};
    JOLTC_TRY_BEGIN
    return fromJphQuat(ch(c)->GetRotation(lockBodies != 0));
    JOLTC_TRY_END
    return JoltC_Quat{0,0,0,1};
}

JOLTC_API void JoltC_Character_SetRotation(JoltC_Character* c, JoltC_Quat rotation, JoltC_Activation activation, JoltC_Bool lockBodies) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    ch(c)->SetRotation(toJphQuat(rotation), toJphActivation(activation), lockBodies != 0);
    JOLTC_TRY_END
}

JOLTC_API JoltC_RVec3 JoltC_Character_GetCenterOfMassPosition(const JoltC_Character* c, JoltC_Bool lockBodies) {
    if (!c) { JoltC_RVec3 r = {0,0,0}; return r; }
    JOLTC_TRY_BEGIN
    return fromJphRVec3(ch(c)->GetCenterOfMassPosition(lockBodies != 0));
    JOLTC_TRY_END
    JoltC_RVec3 r = {0,0,0}; return r;
}

JOLTC_API JoltC_ObjectLayer JoltC_Character_GetLayer(const JoltC_Character* c) {
    if (!c) return JOLTC_OBJECT_LAYER_INVALID;
    return ch(c)->GetLayer();
}

JOLTC_API void JoltC_Character_SetLayer(JoltC_Character* c, JoltC_ObjectLayer layer, JoltC_Bool lockBodies) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    ch(c)->SetLayer(layer, lockBodies != 0);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_Character_SetShape(JoltC_Character* c, const JoltC_Shape* shape, float maxPenetrationDepth, JoltC_Bool lockBodies) {
    if (!c || !shape) return;
    JOLTC_TRY_BEGIN
    ch(c)->SetShape(reinterpret_cast<const Shape*>(shape), maxPenetrationDepth, lockBodies != 0);
    JOLTC_TRY_END
}

/* CharacterBase accessors on Character */
JOLTC_API JoltC_GroundState JoltC_Character_GetGroundState(const JoltC_Character* c) {
    if (!c) return JOLTC_GROUND_STATE_IN_AIR;
    return static_cast<JoltC_GroundState>(ch(c)->GetGroundState());
}

JOLTC_API JoltC_Bool JoltC_Character_IsSupported(const JoltC_Character* c) {
    if (!c) return JOLTC_FALSE;
    return ch(c)->IsSupported() ? JOLTC_TRUE : JOLTC_FALSE;
}

JOLTC_API JoltC_RVec3 JoltC_Character_GetGroundPosition(const JoltC_Character* c) {
    if (!c) { JoltC_RVec3 r = {0,0,0}; return r; }
    return fromJphRVec3(ch(c)->GetGroundPosition());
}

JOLTC_API JoltC_Vec3 JoltC_Character_GetGroundNormal(const JoltC_Character* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(ch(c)->GetGroundNormal());
}

JOLTC_API JoltC_Vec3 JoltC_Character_GetGroundVelocity(const JoltC_Character* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(ch(c)->GetGroundVelocity());
}

JOLTC_API JoltC_BodyID JoltC_Character_GetGroundBodyID(const JoltC_Character* c) {
    if (!c) return JOLTC_BODY_ID_INVALID;
    return fromJphBodyID(ch(c)->GetGroundBodyID());
}

JOLTC_API const JoltC_PhysicsMaterial* JoltC_Character_GetGroundMaterial(const JoltC_Character* c) {
    if (!c) return nullptr;
    JOLTC_TRY_BEGIN
    return fromPhysicsMaterial(ch(c)->GetGroundMaterial());
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_Vec3 JoltC_Character_GetUp(const JoltC_Character* c) {
    if (!c) return JoltC_Vec3{0,1,0};
    return fromJphVec3(ch(c)->GetUp());
}

JOLTC_API void JoltC_Character_SetUp(JoltC_Character* c, JoltC_Vec3 up) {
    if (!c) return;
    ch(c)->SetUp(toJphVec3(up));
}

JOLTC_API void JoltC_Character_SetMaxSlopeAngle(JoltC_Character* c, float maxSlopeAngle) {
    if (!c) return;
    ch(c)->SetMaxSlopeAngle(maxSlopeAngle);
}

/* -------------------------------------------------------------------------- */
/*  Additional Character methods                                              */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_Character_GetPositionAndRotation(const JoltC_Character* c, JoltC_RVec3* outPosition, JoltC_Quat* outRotation) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    RVec3 pos;
    Quat rot;
    ch(c)->GetPositionAndRotation(pos, rot, true);
    if (outPosition) *outPosition = fromJphRVec3(pos);
    if (outRotation) *outRotation = fromJphQuat(rot);
    JOLTC_TRY_END
}

JOLTC_API JoltC_Mat44 JoltC_Character_GetWorldTransform(const JoltC_Character* c) {
    if (!c) return JoltC_Mat44{};
    JOLTC_TRY_BEGIN
    return fromJphMat44(ch(c)->GetWorldTransform(true));
    JOLTC_TRY_END
    return JoltC_Mat44{};
}

JOLTC_API void JoltC_Character_SetLinearAndAngularVelocity(JoltC_Character* c, JoltC_Vec3 linearVelocity, JoltC_Vec3 angularVelocity) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    ch(c)->SetLinearAndAngularVelocity(toJphVec3(linearVelocity), toJphVec3(angularVelocity), true);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_Character_SetPositionAndRotation(JoltC_Character* c, JoltC_RVec3 position, JoltC_Quat rotation, int activation) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    ch(c)->SetPositionAndRotation(toJphRVec3(position), toJphQuat(rotation), static_cast<EActivation>(activation), true);
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  CharacterBase (polymorphic base)                                          */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_CharacterBase* JoltC_Character_AsBase(JoltC_Character* c) {
    if (!c) return nullptr;
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_CharacterBase;
    w->ptr = c->ptr.GetPtr();
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_CharacterBase* JoltC_CharacterVirtual_AsBase(JoltC_CharacterVirtual* c) {
    if (!c) return nullptr;
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_CharacterBase;
    w->ptr = c->ptr.GetPtr();
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_CharacterBase_Destroy(JoltC_CharacterBase* character) {
    delete character;
}

JOLTC_API float JoltC_CharacterBase_GetCosMaxSlopeAngle(const JoltC_CharacterBase* c) {
    if (!c) return 0;
    return cb(c)->GetCosMaxSlopeAngle();
}

JOLTC_API uint32_t JoltC_CharacterBase_GetGroundBodyId(const JoltC_CharacterBase* c) {
    if (!c) return JOLTC_BODY_ID_INVALID;
    return fromJphBodyID(cb(c)->GetGroundBodyID());
}

JOLTC_API JoltC_Vec3 JoltC_CharacterBase_GetGroundNormal(const JoltC_CharacterBase* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(cb(c)->GetGroundNormal());
}

JOLTC_API JoltC_RVec3 JoltC_CharacterBase_GetGroundPosition(const JoltC_CharacterBase* c) {
    if (!c) { JoltC_RVec3 r = {0,0,0}; return r; }
    return fromJphRVec3(cb(c)->GetGroundPosition());
}

JOLTC_API int JoltC_CharacterBase_GetGroundState(const JoltC_CharacterBase* c) {
    if (!c) return JOLTC_GROUND_STATE_IN_AIR;
    return static_cast<int>(cb(c)->GetGroundState());
}

JOLTC_API uint32_t JoltC_CharacterBase_GetGroundSubShapeId(const JoltC_CharacterBase* c) {
    if (!c) return 0;
    return cb(c)->GetGroundSubShapeID().GetValue();
}

JOLTC_API uint64_t JoltC_CharacterBase_GetGroundUserData(const JoltC_CharacterBase* c) {
    if (!c) return 0;
    return cb(c)->GetGroundUserData();
}

JOLTC_API JoltC_Vec3 JoltC_CharacterBase_GetGroundVelocity(const JoltC_CharacterBase* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(cb(c)->GetGroundVelocity());
}

JOLTC_API JoltC_Vec3 JoltC_CharacterBase_GetUp(const JoltC_CharacterBase* c) {
    if (!c) return JoltC_Vec3{0,0,0};
    return fromJphVec3(cb(c)->GetUp());
}

JOLTC_API int JoltC_CharacterBase_IsSlopeTooSteep(const JoltC_CharacterBase* c, JoltC_Vec3 normal) {
    if (!c) return 0;
    return cb(c)->IsSlopeTooSteep(toJphVec3(normal)) ? 1 : 0;
}

JOLTC_API int JoltC_CharacterBase_IsSupported(const JoltC_CharacterBase* c) {
    if (!c) return 0;
    return cb(c)->IsSupported() ? 1 : 0;
}

JOLTC_API void JoltC_CharacterBase_SetMaxSlopeAngle(JoltC_CharacterBase* c, float maxSlopeAngle) {
    if (!c) return;
    cb(c)->SetMaxSlopeAngle(maxSlopeAngle);
}

JOLTC_API void JoltC_CharacterBase_SetUp(JoltC_CharacterBase* c, JoltC_Vec3 up) {
    if (!c) return;
    cb(c)->SetUp(toJphVec3(up));
}

/* -------------------------------------------------------------------------- */
/*  CharacterSettings / CharacterVirtualSettings Init                         */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_CharacterSettings_Init(JoltC_CharacterSettings* s) {
    JoltC_CharacterSettings_SetDefault(s);
}

JOLTC_API void JoltC_CharacterVirtualSettings_Init(JoltC_CharacterVirtualSettings* s) {
    JoltC_CharacterVirtualSettings_SetDefault(s);
}

/* -------------------------------------------------------------------------- */
/*  Additional CharacterVirtual methods                                       */
/* -------------------------------------------------------------------------- */
JOLTC_API int JoltC_CharacterVirtual_CanWalkStairs(const JoltC_CharacterVirtual* c, JoltC_Vec3 linearVelocity) {
    if (!c) return 0;
    JOLTC_TRY_BEGIN
    return cv(c)->CanWalkStairs(toJphVec3(linearVelocity)) ? 1 : 0;
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_CharacterVirtual_FinishTrackingContactChanges(JoltC_CharacterVirtual* c) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    cv(c)->FinishTrackingContactChanges();
    JOLTC_TRY_END
}

JOLTC_API void JoltC_CharacterVirtual_GetActiveContact(const JoltC_CharacterVirtual* c, uint32_t index,
    JoltC_Vec3* outContactNormal, JoltC_Vec3* outContactVelocity, uint32_t* outBodyId2, uint32_t* outSubShapeId2)
{
    if (!c) return;
    JOLTC_TRY_BEGIN
    const auto& contacts = cv(c)->GetActiveContacts();
    if (index >= contacts.size()) return;
    const auto& contact = contacts[index];
    if (outContactNormal)   *outContactNormal   = fromJphVec3(contact.mContactNormal);
    if (outContactVelocity) *outContactVelocity = fromJphVec3(contact.mLinearVelocity);
    if (outBodyId2)         *outBodyId2         = contact.mBodyB.GetIndexAndSequenceNumber();
    if (outSubShapeId2)     *outSubShapeId2     = contact.mSubShapeIDB.GetValue();
    JOLTC_TRY_END
}

JOLTC_API uint32_t JoltC_CharacterVirtual_GetID(const JoltC_CharacterVirtual* c) {
    if (!c) return 0xFFFFFFFFU;
    return cv(c)->GetID().GetValue();
}

JOLTC_API int JoltC_CharacterVirtual_HasCollidedWith(const JoltC_CharacterVirtual* c, uint32_t bodyId) {
    if (!c) return 0;
    return cv(c)->HasCollidedWith(BodyID(bodyId)) ? 1 : 0;
}

JOLTC_API int JoltC_CharacterVirtual_HasCollidedWithCharacter(const JoltC_CharacterVirtual* c, uint32_t otherCharacterId) {
    if (!c) return 0;
    return cv(c)->HasCollidedWith(CharacterID(otherCharacterId)) ? 1 : 0;
}

JOLTC_API void JoltC_CharacterVirtual_SetCharacterVsCharacterCollision(JoltC_CharacterVirtual* c, JoltC_CharacterVsCharacterCollision* collision) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    cv(c)->SetCharacterVsCharacterCollision(collision ? collision->ptr.get() : nullptr);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_CharacterVirtual_StartTrackingContactChanges(JoltC_CharacterVirtual* c) {
    if (!c) return;
    JOLTC_TRY_BEGIN
    cv(c)->StartTrackingContactChanges();
    JOLTC_TRY_END
}

JOLTC_API int JoltC_CharacterVirtual_StickToFloor(JoltC_CharacterVirtual* c, JoltC_Vec3 stepDown, JoltC_TempAllocator* allocator) {
    if (!c || !allocator) return 0;
    JOLTC_TRY_BEGIN
    return cv(c)->StickToFloor(toJphVec3(stepDown), {}, {}, {}, {}, *allocator->ptr) ? 1 : 0;
    JOLTC_TRY_END
    return 0;
}

JOLTC_API int JoltC_CharacterVirtual_WalkStairs(JoltC_CharacterVirtual* c, float deltaTime,
    JoltC_Vec3 stepUp, JoltC_Vec3 stepForward, JoltC_Vec3 stepForwardTest, JoltC_Vec3 stepDownExtra,
    JoltC_TempAllocator* allocator)
{
    if (!c || !allocator) return 0;
    JOLTC_TRY_BEGIN
    return cv(c)->WalkStairs(deltaTime, toJphVec3(stepUp), toJphVec3(stepForward),
        toJphVec3(stepForwardTest), toJphVec3(stepDownExtra),
        {}, {}, {}, {}, *allocator->ptr) ? 1 : 0;
    JOLTC_TRY_END
    return 0;
}

/* -------------------------------------------------------------------------- */
/*  Simulation with collision filters                                         */
/* -------------------------------------------------------------------------- */
/* The plain entry points above pass default constructed filters, and a default filter says yes to
 * everything: a character simulated through them collides with every layer there is, sensors
 * included, and no work on the caller's side can narrow that. These variants take the same four
 * filters every C++ signature has taken all along. Null means the permissive default, so a caller
 * can pass only the ones it cares about. */
static const BroadPhaseLayerFilter& characterBpFilter(const JoltC_BroadPhaseLayerFilter* f) {
    static BroadPhaseLayerFilter sDefault;
    return f ? *f->ptr : sDefault;
}
static const ObjectLayerFilter& characterOlFilter(const JoltC_ObjectLayerFilter* f) {
    static ObjectLayerFilter sDefault;
    return f ? *f->ptr : sDefault;
}
static const BodyFilter& characterBodyFilter(const JoltC_BodyFilter* f) {
    static BodyFilter sDefault;
    return f ? *f->ptr : sDefault;
}
static const ShapeFilter& characterShapeFilter(const JoltC_ShapeFilter* f) {
    static ShapeFilter sDefault;
    return f ? *f->ptr : sDefault;
}

JOLTC_API void JoltC_CharacterVirtual_Update_WithFilters(
    JoltC_CharacterVirtual* c, float dt, JoltC_Vec3 gravity,
    const JoltC_BroadPhaseLayerFilter* bpFilter,
    const JoltC_ObjectLayerFilter* olFilter,
    const JoltC_BodyFilter* bodyFilter,
    const JoltC_ShapeFilter* shapeFilter,
    JoltC_TempAllocator* allocator)
{
    if (!c || !allocator) return;
    JOLTC_TRY_BEGIN
    cv(c)->Update(dt, toJphVec3(gravity),
        characterBpFilter(bpFilter), characterOlFilter(olFilter),
        characterBodyFilter(bodyFilter), characterShapeFilter(shapeFilter),
        *allocator->ptr);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_CharacterVirtual_ExtendedUpdate_WithFilters(
    JoltC_CharacterVirtual* c, float dt, JoltC_Vec3 gravity,
    const JoltC_ExtendedUpdateSettings* s,
    const JoltC_BroadPhaseLayerFilter* bpFilter,
    const JoltC_ObjectLayerFilter* olFilter,
    const JoltC_BodyFilter* bodyFilter,
    const JoltC_ShapeFilter* shapeFilter,
    JoltC_TempAllocator* allocator)
{
    if (!c || !s || !allocator) return;
    JOLTC_TRY_BEGIN
    CharacterVirtual::ExtendedUpdateSettings eus;
    eus.mStickToFloorStepDown = toJphVec3(s->stickToFloorStepDown);
    eus.mWalkStairsStepUp = toJphVec3(s->walkStairsStepUp);
    eus.mWalkStairsMinStepForward = s->walkStairsMinStepForward;
    eus.mWalkStairsStepForwardTest = s->walkStairsStepForwardTest;
    eus.mWalkStairsCosAngleForwardContact = s->walkStairsCosAngleForwardContact;
    eus.mWalkStairsStepDownExtra = toJphVec3(s->walkStairsStepDownExtra);
    cv(c)->ExtendedUpdate(dt, toJphVec3(gravity), eus,
        characterBpFilter(bpFilter), characterOlFilter(olFilter),
        characterBodyFilter(bodyFilter), characterShapeFilter(shapeFilter),
        *allocator->ptr);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_CharacterVirtual_RefreshContacts_WithFilters(
    JoltC_CharacterVirtual* c,
    const JoltC_BroadPhaseLayerFilter* bpFilter,
    const JoltC_ObjectLayerFilter* olFilter,
    const JoltC_BodyFilter* bodyFilter,
    const JoltC_ShapeFilter* shapeFilter,
    JoltC_TempAllocator* allocator)
{
    if (!c || !allocator) return;
    JOLTC_TRY_BEGIN
    cv(c)->RefreshContacts(
        characterBpFilter(bpFilter), characterOlFilter(olFilter),
        characterBodyFilter(bodyFilter), characterShapeFilter(shapeFilter),
        *allocator->ptr);
    JOLTC_TRY_END
}

JOLTC_API JoltC_Bool JoltC_CharacterVirtual_SetShape_WithFilters(
    JoltC_CharacterVirtual* c, const JoltC_Shape* shape, float maxPenetrationDepth,
    const JoltC_BroadPhaseLayerFilter* bpFilter,
    const JoltC_ObjectLayerFilter* olFilter,
    const JoltC_BodyFilter* bodyFilter,
    const JoltC_ShapeFilter* shapeFilter,
    JoltC_TempAllocator* allocator)
{
    if (!c || !shape || !allocator) return JOLTC_FALSE;
    JOLTC_TRY_BEGIN
    bool result = cv(c)->SetShape(
        reinterpret_cast<const Shape*>(shape), maxPenetrationDepth,
        characterBpFilter(bpFilter), characterOlFilter(olFilter),
        characterBodyFilter(bodyFilter), characterShapeFilter(shapeFilter),
        *allocator->ptr);
    return result ? JOLTC_TRUE : JOLTC_FALSE;
    JOLTC_TRY_END
    return JOLTC_FALSE;
}

JOLTC_API int JoltC_CharacterVirtual_StickToFloor_WithFilters(
    JoltC_CharacterVirtual* c, JoltC_Vec3 stepDown,
    const JoltC_BroadPhaseLayerFilter* bpFilter,
    const JoltC_ObjectLayerFilter* olFilter,
    const JoltC_BodyFilter* bodyFilter,
    const JoltC_ShapeFilter* shapeFilter,
    JoltC_TempAllocator* allocator)
{
    if (!c || !allocator) return 0;
    JOLTC_TRY_BEGIN
    return cv(c)->StickToFloor(toJphVec3(stepDown),
        characterBpFilter(bpFilter), characterOlFilter(olFilter),
        characterBodyFilter(bodyFilter), characterShapeFilter(shapeFilter),
        *allocator->ptr) ? 1 : 0;
    JOLTC_TRY_END
    return 0;
}

JOLTC_API int JoltC_CharacterVirtual_WalkStairs_WithFilters(
    JoltC_CharacterVirtual* c, float deltaTime,
    JoltC_Vec3 stepUp, JoltC_Vec3 stepForward, JoltC_Vec3 stepForwardTest, JoltC_Vec3 stepDownExtra,
    const JoltC_BroadPhaseLayerFilter* bpFilter,
    const JoltC_ObjectLayerFilter* olFilter,
    const JoltC_BodyFilter* bodyFilter,
    const JoltC_ShapeFilter* shapeFilter,
    JoltC_TempAllocator* allocator)
{
    if (!c || !allocator) return 0;
    JOLTC_TRY_BEGIN
    return cv(c)->WalkStairs(deltaTime, toJphVec3(stepUp), toJphVec3(stepForward),
        toJphVec3(stepForwardTest), toJphVec3(stepDownExtra),
        characterBpFilter(bpFilter), characterOlFilter(olFilter),
        characterBodyFilter(bodyFilter), characterShapeFilter(shapeFilter),
        *allocator->ptr) ? 1 : 0;
    JOLTC_TRY_END
    return 0;
}

/* -------------------------------------------------------------------------- */
/*  CharacterContactListener SetProcs                                         */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_CharacterContactListener_SetProcs(JoltC_CharacterContactListener* listener,
    JoltC_CharacterContactListener_Procs procs, void* userData)
{
    if (!listener || !listener->ptr) return;
    listener->ptr->fnValidate  = procs.onValidate;
    listener->ptr->fnAdded     = procs.onAdded;
    listener->ptr->fnPersisted = procs.onPersisted;
    listener->ptr->userData    = userData;
}

/* -------------------------------------------------------------------------- */
/*  CharacterVsCharacterCollision                                             */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_CharacterVsCharacterCollision* JoltC_CharacterVsCharacterCollision_Create(void) {
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_CharacterVsCharacterCollision;
    w->ptr = std::make_unique<CharacterVsCharacterCollisionSimple>();
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_CharacterVsCharacterCollision* JoltC_CharacterVsCharacterCollision_CreateSimple(void) {
    return JoltC_CharacterVsCharacterCollision_Create();
}

JOLTC_API void JoltC_CharacterVsCharacterCollision_Destroy(JoltC_CharacterVsCharacterCollision* collision) {
    delete collision;
}

JOLTC_API void JoltC_CharacterVsCharacterCollisionSimple_AddCharacter(JoltC_CharacterVsCharacterCollision* collision, JoltC_CharacterVirtual* character) {
    if (!collision || !character) return;
    JOLTC_TRY_BEGIN
    collision->ptr->Add(cv(character));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_CharacterVsCharacterCollisionSimple_RemoveCharacter(JoltC_CharacterVsCharacterCollision* collision, JoltC_CharacterVirtual* character) {
    if (!collision || !character) return;
    JOLTC_TRY_BEGIN
    collision->ptr->Remove(cv(character));
    JOLTC_TRY_END
}

} /* extern "C" */
