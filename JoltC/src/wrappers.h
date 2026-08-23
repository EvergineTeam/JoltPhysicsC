/* JoltC - ALL internal wrapper structs (single definition for ODR safety)
 * SPDX-License-Identifier: MIT
 */

#ifndef JOLTC_WRAPPERS_H
#define JOLTC_WRAPPERS_H

#include <Jolt/Jolt.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Constraints/Constraint.h>
#include <Jolt/Physics/Constraints/TwoBodyConstraint.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Character/Character.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Collision/ShapeFilter.h>
#include <Jolt/Physics/Collision/SimShapeFilter.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Physics/PhysicsStepListener.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/Physics/Vehicle/MotorcycleController.h>
#include <Jolt/Physics/Vehicle/TrackedVehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Core/LinearCurve.h>
#include <JoltC/common.h>
#include <JoltC/character.h>
#include "internal.h"

#include <memory>

using namespace JPH;

/* -------------------------------------------------------------------------- */
/*  TempAllocator wrapper                                                     */
/* -------------------------------------------------------------------------- */
struct JoltC_TempAllocator {
    std::unique_ptr<TempAllocatorImpl> ptr;
};

/* -------------------------------------------------------------------------- */
/*  JobSystem wrapper                                                         */
/* -------------------------------------------------------------------------- */
struct JoltC_JobSystem {
    std::unique_ptr<JobSystemThreadPool> ptr;
};

/* -------------------------------------------------------------------------- */
/*  BroadPhaseQuery — non-owning, obtained from PhysicsSystem                */
/* -------------------------------------------------------------------------- */
struct JoltC_BroadPhaseQuery {
    const BroadPhaseQuery* ptr;
};

/* -------------------------------------------------------------------------- */
/*  PhysicsSystem wrapper                                                     */
/* -------------------------------------------------------------------------- */
struct JoltC_PhysicsSystem {
    std::unique_ptr<PhysicsSystem> ptr;

    /* Handed out by GetBroadPhaseQuery. Owned per system, because the thread_local this replaces
     * meant a second system's Get invalidated the first system's pointer on the same thread. */
    JoltC_BroadPhaseQuery broadPhaseQuery { nullptr };
};

/* -------------------------------------------------------------------------- */
/*  BodyInterface — thin wrapper, non-owning pointer                          */
/* -------------------------------------------------------------------------- */
struct JoltC_BodyInterface {
    BodyInterface* ptr;
    PhysicsSystem* system;
};

/* -------------------------------------------------------------------------- */
/*  BroadPhaseLayerInterface — callback-based implementation                  */
/* -------------------------------------------------------------------------- */
class BroadPhaseLayerInterfaceCallback final : public BroadPhaseLayerInterface {
public:
    JoltC_GetNumBroadPhaseLayersFn fnGetNumLayers;
    JoltC_GetBroadPhaseLayerFn     fnGetBroadPhaseLayer;
    void*                          userData;

    uint GetNumBroadPhaseLayers() const override {
        return fnGetNumLayers(userData);
    }

    BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer inLayer) const override {
        return BroadPhaseLayer(fnGetBroadPhaseLayer(userData, inLayer));
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(BroadPhaseLayer) const override {
        return "JoltC_Layer";
    }
#endif
};

struct JoltC_BroadPhaseLayerInterface {
    std::unique_ptr<BroadPhaseLayerInterface> ptr;
};

/* -------------------------------------------------------------------------- */
/*  ObjectVsBroadPhaseLayerFilter — callback-based                            */
/* -------------------------------------------------------------------------- */
class ObjectVsBroadPhaseLayerFilterCallback final : public ObjectVsBroadPhaseLayerFilter {
public:
    JoltC_ObjectVsBroadPhaseLayerFilterFn fnFilter;
    void* userData;

    bool ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const override {
        return fnFilter(userData, inLayer1, static_cast<JoltC_BroadPhaseLayer>(inLayer2.GetValue())) != 0;
    }
};

struct JoltC_ObjectVsBroadPhaseLayerFilter {
    std::unique_ptr<ObjectVsBroadPhaseLayerFilter> ptr;
};

/* -------------------------------------------------------------------------- */
/*  ObjectLayerPairFilter — callback-based                                    */
/* -------------------------------------------------------------------------- */
class ObjectLayerPairFilterCallback final : public ObjectLayerPairFilter {
public:
    JoltC_ObjectLayerPairFilterFn fnFilter;
    void* userData;

    bool ShouldCollide(ObjectLayer inLayer1, ObjectLayer inLayer2) const override {
        return fnFilter(userData, inLayer1, inLayer2) != 0;
    }
};

struct JoltC_ObjectLayerPairFilter {
    std::unique_ptr<ObjectLayerPairFilter> ptr;
};

/* -------------------------------------------------------------------------- */
/*  ContactListener — callback-based                                          */
/* -------------------------------------------------------------------------- */
class ContactListenerCallback final : public ContactListener {
public:
    JoltC_OnContactValidateFn  fnValidate;
    JoltC_OnContactAddedFn     fnAdded;
    JoltC_OnContactPersistedFn fnPersisted;
    JoltC_OnContactRemovedFn   fnRemoved;
    void*                      userData;

    ValidateResult OnContactValidate(const Body& inBody1, const Body& inBody2, RVec3Arg, const CollideShapeResult&) override {
        if (fnValidate)
            return static_cast<ValidateResult>(fnValidate(userData, inBody1.GetID().GetIndexAndSequenceNumber(), inBody2.GetID().GetIndexAndSequenceNumber()));
        return ValidateResult::AcceptAllContactsForThisBodyPair;
    }

    void OnContactAdded(const Body& inBody1, const Body& inBody2, const ContactManifold&, ContactSettings&) override {
        if (fnAdded)
            fnAdded(userData, inBody1.GetID().GetIndexAndSequenceNumber(), inBody2.GetID().GetIndexAndSequenceNumber());
    }

    void OnContactPersisted(const Body& inBody1, const Body& inBody2, const ContactManifold&, ContactSettings&) override {
        if (fnPersisted)
            fnPersisted(userData, inBody1.GetID().GetIndexAndSequenceNumber(), inBody2.GetID().GetIndexAndSequenceNumber());
    }

    void OnContactRemoved(const SubShapeIDPair& inSubShapePair) override {
        if (fnRemoved)
            fnRemoved(userData, inSubShapePair.GetBody1ID().GetIndexAndSequenceNumber(), inSubShapePair.GetBody2ID().GetIndexAndSequenceNumber());
    }
};

struct JoltC_ContactListener {
    std::unique_ptr<ContactListener> ptr;
};

/* -------------------------------------------------------------------------- */
/*  BodyActivationListener — callback-based                                   */
/* -------------------------------------------------------------------------- */
class BodyActivationListenerCallback final : public BodyActivationListener {
public:
    JoltC_OnBodyActivatedFn   fnActivated;
    JoltC_OnBodyDeactivatedFn fnDeactivated;
    void*                     userData;

    void OnBodyActivated(const BodyID& inBodyID, uint64 inBodyUserData) override {
        if (fnActivated)
            fnActivated(userData, inBodyID.GetIndexAndSequenceNumber(), inBodyUserData);
    }

    void OnBodyDeactivated(const BodyID& inBodyID, uint64 inBodyUserData) override {
        if (fnDeactivated)
            fnDeactivated(userData, inBodyID.GetIndexAndSequenceNumber(), inBodyUserData);
    }
};

struct JoltC_BodyActivationListener {
    std::unique_ptr<BodyActivationListenerCallback> ptr;
};

/* -------------------------------------------------------------------------- */
/*  Constraint — wraps ref-counted Constraint*                                */
/* -------------------------------------------------------------------------- */
struct JoltC_Constraint {
    Ref<Constraint> ptr;
};

/* -------------------------------------------------------------------------- */
/*  NarrowPhaseQuery — non-owning, obtained from PhysicsSystem                */
/* -------------------------------------------------------------------------- */
struct JoltC_NarrowPhaseQuery {
    const NarrowPhaseQuery* ptr;
};

/* -------------------------------------------------------------------------- */
/*  CharacterContactListener — callback-based                                 */
/* -------------------------------------------------------------------------- */
class CharacterContactListenerCallback final : public CharacterContactListener {
public:
    JoltC_OnCharacterContactValidateFn  fnValidate;
    JoltC_OnCharacterContactAddedFn     fnAdded;
    JoltC_OnCharacterContactPersistedFn fnPersisted;
    void*                               userData;

    /* JoltPhysics 5.6.0 collapsed the loose parameters of these three virtuals into one
     * CharacterContact struct, which carries a good deal more than the wrapper needs:
     *
     *   OnContactValidate (const CharacterVirtual*, const CharacterContact&)
     *   OnContactAdded    (const CharacterVirtual*, const CharacterContact&, CharacterContactSettings&)
     *   OnContactPersisted(const CharacterVirtual*, const CharacterContact&, CharacterContactSettings&)
     *
     * The body id moved to the CharacterContactKey base as mBodyB; position and normal are
     * mPosition and mContactNormal. The C ABI is untouched -- the JoltC_OnCharacterContact*Fn
     * signatures and everything a downstream binding sees are exactly as they were.
     *
     * Note for whoever exposes more of this later: the struct also carries mSurfaceNormal,
     * mDistance, mFraction, mMotionTypeB, mIsSensorB, mCharacterB, mMaterial and
     * mIsBackFacingContact. None of them reach C today. mSurfaceNormal in particular is not
     * mContactNormal -- it is flipped for back-facing contacts -- so a future addition should
     * not treat them as interchangeable. */
    bool OnContactValidate(const CharacterVirtual*, const CharacterContact &inContact) override {
        if (fnValidate) {
            JoltC_Bool accept = JOLTC_TRUE;
            fnValidate(userData, inContact.mBodyB.GetIndexAndSequenceNumber(), &accept);
            return accept != 0;
        }
        return true;
    }

    void OnContactAdded(const CharacterVirtual*, const CharacterContact &inContact,
                         CharacterContactSettings &ioSettings) override {
        if (fnAdded) {
            JoltC_RVec3 pos; pos.x = (float)inContact.mPosition.GetX(); pos.y = (float)inContact.mPosition.GetY(); pos.z = (float)inContact.mPosition.GetZ();
            JoltC_Vec3 norm = {inContact.mContactNormal.GetX(), inContact.mContactNormal.GetY(), inContact.mContactNormal.GetZ()};
            JoltC_Bool canPush = ioSettings.mCanPushCharacter ? JOLTC_TRUE : JOLTC_FALSE;
            JoltC_Bool canImpulse = ioSettings.mCanReceiveImpulses ? JOLTC_TRUE : JOLTC_FALSE;
            fnAdded(userData, inContact.mBodyB.GetIndexAndSequenceNumber(), pos, norm, &canPush, &canImpulse);
            ioSettings.mCanPushCharacter = canPush != 0;
            ioSettings.mCanReceiveImpulses = canImpulse != 0;
        }
    }

    void OnContactPersisted(const CharacterVirtual*, const CharacterContact &inContact,
                             CharacterContactSettings &ioSettings) override {
        if (fnPersisted) {
            JoltC_RVec3 pos; pos.x = (float)inContact.mPosition.GetX(); pos.y = (float)inContact.mPosition.GetY(); pos.z = (float)inContact.mPosition.GetZ();
            JoltC_Vec3 norm = {inContact.mContactNormal.GetX(), inContact.mContactNormal.GetY(), inContact.mContactNormal.GetZ()};
            JoltC_Bool canPush = ioSettings.mCanPushCharacter ? JOLTC_TRUE : JOLTC_FALSE;
            JoltC_Bool canImpulse = ioSettings.mCanReceiveImpulses ? JOLTC_TRUE : JOLTC_FALSE;
            fnPersisted(userData, inContact.mBodyB.GetIndexAndSequenceNumber(), pos, norm, &canPush, &canImpulse);
            ioSettings.mCanPushCharacter = canPush != 0;
            ioSettings.mCanReceiveImpulses = canImpulse != 0;
        }
    }
};

struct JoltC_CharacterContactListener {
    std::unique_ptr<CharacterContactListenerCallback> ptr;
};

/* -------------------------------------------------------------------------- */
/*  CharacterVirtual wrapper                                                  */
/* -------------------------------------------------------------------------- */
struct JoltC_CharacterVirtual {
    Ref<CharacterVirtual> ptr;
    JoltC_CharacterContactListener* listener = nullptr;
};

/* -------------------------------------------------------------------------- */
/*  BroadPhaseLayerFilter — callback-based query filter                       */
/* -------------------------------------------------------------------------- */
class BroadPhaseLayerFilterCallback final : public BroadPhaseLayerFilter {
public:
    JoltC_BroadPhaseLayerFilterFn fn;
    void* userData;
    bool ShouldCollide(BroadPhaseLayer inLayer) const override {
        return fn(userData, static_cast<JoltC_BroadPhaseLayer>(inLayer.GetValue())) != 0;
    }
};
struct JoltC_BroadPhaseLayerFilter {
    /* Base typed, because a filter handle can now also carry one of the system's own default
     * filters, not only the callback kind. */
    std::unique_ptr<BroadPhaseLayerFilter> ptr;

    /* Set only when ptr is the callback kind; SetProcs has nothing to write to otherwise. */
    BroadPhaseLayerFilterCallback* callback = nullptr;
};

/* -------------------------------------------------------------------------- */
/*  ObjectLayerFilter — callback-based query filter                           */
/* -------------------------------------------------------------------------- */
class ObjectLayerFilterCallback final : public ObjectLayerFilter {
public:
    JoltC_ObjectLayerFilterFn fn;
    void* userData;
    bool ShouldCollide(ObjectLayer inLayer) const override {
        return fn(userData, inLayer) != 0;
    }
};
struct JoltC_ObjectLayerFilter {
    std::unique_ptr<ObjectLayerFilter> ptr;
    ObjectLayerFilterCallback* callback = nullptr;
};

/* -------------------------------------------------------------------------- */
/*  BodyFilter — callback-based query filter                                  */
/* -------------------------------------------------------------------------- */
class BodyFilterCallback final : public BodyFilter {
public:
    JoltC_BodyFilterFn fn;
    JoltC_BodyFilterLockedFn fnLocked;
    void* userData;
    bool ShouldCollide(const BodyID &inBodyID) const override {
        if (fn) return fn(userData, inBodyID.GetIndexAndSequenceNumber()) != 0;
        return true;
    }
    bool ShouldCollideLocked(const Body &inBody) const override {
        if (fnLocked) return fnLocked(userData, reinterpret_cast<const JoltC_Body*>(&inBody)) != 0;
        return true;
    }
};
struct JoltC_BodyFilter {
    std::unique_ptr<BodyFilterCallback> ptr;
};

/* -------------------------------------------------------------------------- */
/*  ShapeFilter — callback-based query filter                                 */
/* -------------------------------------------------------------------------- */
class ShapeFilterCallback final : public ShapeFilter {
public:
    JoltC_ShapeFilterFn fn1;
    JoltC_ShapeFilter2Fn fn2;
    void* userData;
    bool ShouldCollide(const Shape *inShape2, const SubShapeID &inSubShapeIDOfShape2) const override {
        if (fn1) return fn1(userData,
            reinterpret_cast<const JoltC_Shape*>(inShape2),
            inSubShapeIDOfShape2.GetValue()) != 0;
        return true;
    }
    bool ShouldCollide(const Shape *inShape1, const SubShapeID &inSubShapeIDOfShape1,
                       const Shape *inShape2, const SubShapeID &inSubShapeIDOfShape2) const override {
        if (fn2) return fn2(userData,
            reinterpret_cast<const JoltC_Shape*>(inShape1), inSubShapeIDOfShape1.GetValue(),
            reinterpret_cast<const JoltC_Shape*>(inShape2), inSubShapeIDOfShape2.GetValue()) != 0;
        return true;
    }
};
struct JoltC_ShapeFilter {
    std::unique_ptr<ShapeFilterCallback> ptr;
};

/* -------------------------------------------------------------------------- */
/*  SimShapeFilter — callback-based simulation filter                         */
/* -------------------------------------------------------------------------- */
class SimShapeFilterCallback final : public SimShapeFilter {
public:
    JoltC_SimShapeFilterFn fn;
    void* userData;
    bool ShouldCollide(const Body &inBody1, const Shape *inShape1, const SubShapeID &inSubShapeIDOfShape1,
                       const Body &inBody2, const Shape *inShape2, const SubShapeID &inSubShapeIDOfShape2) const override {
        if (fn) return fn(userData,
            reinterpret_cast<const JoltC_Body*>(&inBody1),
            reinterpret_cast<const JoltC_Shape*>(inShape1), inSubShapeIDOfShape1.GetValue(),
            reinterpret_cast<const JoltC_Body*>(&inBody2),
            reinterpret_cast<const JoltC_Shape*>(inShape2), inSubShapeIDOfShape2.GetValue()) != 0;
        return true;
    }
};
struct JoltC_SimShapeFilter {
    std::unique_ptr<SimShapeFilterCallback> ptr;
};

/* -------------------------------------------------------------------------- */
/*  PhysicsStepListener — callback-based                                      */
/* -------------------------------------------------------------------------- */
class PhysicsStepListenerCallback final : public PhysicsStepListener {
public:
    JoltC_OnPhysicsStepFn fn;
    void* userData;
    void OnStep(const PhysicsStepListenerContext &ctx) override {
        if (fn) fn(userData, ctx.mDeltaTime,
                   ctx.mIsFirstStep ? JOLTC_TRUE : JOLTC_FALSE,
                   ctx.mIsLastStep  ? JOLTC_TRUE : JOLTC_FALSE);
    }
};
struct JoltC_PhysicsStepListener {
    std::unique_ptr<PhysicsStepListenerCallback> ptr;
};

/* -------------------------------------------------------------------------- */
/*  PhysicsMaterial — custom subclass that stores name + color                */
/* -------------------------------------------------------------------------- */
class PhysicsMaterialImpl final : public PhysicsMaterial {
public:
    std::string name;
    Color color;
    const char* GetDebugName() const override { return name.c_str(); }
    Color GetDebugColor() const override { return color; }
};
struct JoltC_PhysicsMaterial {
    Ref<PhysicsMaterialImpl> ptr;
};

/* -------------------------------------------------------------------------- */
/*  GroupFilter — ref-counted, wraps GroupFilter*                             */
/* -------------------------------------------------------------------------- */
struct JoltC_GroupFilter {
    Ref<GroupFilter> ptr;
};

/* -------------------------------------------------------------------------- */
/*  Shape settings wrappers                                                   */
/* -------------------------------------------------------------------------- */
struct JoltC_HeightFieldShapeSettings {
    Ref<HeightFieldShapeSettings> ptr;
};
struct JoltC_MeshShapeSettings {
    Ref<MeshShapeSettings> ptr;
};
struct JoltC_ConvexHullShapeSettings {
    Ref<ConvexHullShapeSettings> ptr;
};

/* -------------------------------------------------------------------------- */
/*  Vehicle system wrappers                                                   */
/* -------------------------------------------------------------------------- */
struct JoltC_LinearCurve {
    LinearCurve curve;
};
struct JoltC_VehicleTransmissionSettings {
    VehicleTransmissionSettings settings;
};

/* -------------------------------------------------------------------------- */
/*  Character (non-virtual, body-based) wrapper                               */
/* -------------------------------------------------------------------------- */
struct JoltC_Character {
    Ref<Character> ptr;
};

/* -------------------------------------------------------------------------- */
/*  Enhanced ContactListener — passes Body/ContactManifold/ContactSettings    */
/* -------------------------------------------------------------------------- */
class ContactListenerEnhancedCallback final : public ContactListener {
public:
    JoltC_OnContactValidateEnhancedFn  fnValidate;
    JoltC_OnContactAddedEnhancedFn     fnAdded;
    JoltC_OnContactPersistedEnhancedFn fnPersisted;
    JoltC_OnContactRemovedEnhancedFn   fnRemoved;
    void*                              userData;

    ValidateResult OnContactValidate(const Body& inBody1, const Body& inBody2, RVec3Arg inBaseOffset, const CollideShapeResult& inResult) override {
        if (fnValidate) {
            JoltC_RVec3 baseOffset = fromJphRVec3(inBaseOffset);
            JoltC_CollideShapeResult cr;
            cr.contactPointOn1 = fromJphVec3(inResult.mContactPointOn1);
            cr.contactPointOn2 = fromJphVec3(inResult.mContactPointOn2);
            cr.penetrationAxis = fromJphVec3(inResult.mPenetrationAxis);
            cr.penetrationDepth = inResult.mPenetrationDepth;
            cr.subShapeID1 = inResult.mSubShapeID1.GetValue();
            cr.subShapeID2 = inResult.mSubShapeID2.GetValue();
            cr.bodyID2 = inResult.mBodyID2.GetIndexAndSequenceNumber();
            return static_cast<ValidateResult>(fnValidate(userData,
                reinterpret_cast<const JoltC_Body*>(&inBody1),
                reinterpret_cast<const JoltC_Body*>(&inBody2),
                baseOffset, &cr));
        }
        return ValidateResult::AcceptAllContactsForThisBodyPair;
    }

    void OnContactAdded(const Body& inBody1, const Body& inBody2, const ContactManifold& inManifold, ContactSettings& ioSettings) override {
        if (fnAdded) {
            JoltC_ContactSettings cs;
            cs.combinedFriction = ioSettings.mCombinedFriction;
            cs.combinedRestitution = ioSettings.mCombinedRestitution;
            cs.invMassScale1 = ioSettings.mInvMassScale1;
            cs.invInertiaScale1 = ioSettings.mInvInertiaScale1;
            cs.invMassScale2 = ioSettings.mInvMassScale2;
            cs.invInertiaScale2 = ioSettings.mInvInertiaScale2;
            cs.isSensor = ioSettings.mIsSensor ? JOLTC_TRUE : JOLTC_FALSE;
            cs.relativeLinearSurfaceVelocity = fromJphVec3(ioSettings.mRelativeLinearSurfaceVelocity);
            cs.relativeAngularSurfaceVelocity = fromJphVec3(ioSettings.mRelativeAngularSurfaceVelocity);
            fnAdded(userData,
                reinterpret_cast<const JoltC_Body*>(&inBody1),
                reinterpret_cast<const JoltC_Body*>(&inBody2),
                reinterpret_cast<const JoltC_ContactManifold*>(&inManifold), &cs);
            ioSettings.mCombinedFriction = cs.combinedFriction;
            ioSettings.mCombinedRestitution = cs.combinedRestitution;
            ioSettings.mInvMassScale1 = cs.invMassScale1;
            ioSettings.mInvInertiaScale1 = cs.invInertiaScale1;
            ioSettings.mInvMassScale2 = cs.invMassScale2;
            ioSettings.mInvInertiaScale2 = cs.invInertiaScale2;
            ioSettings.mIsSensor = cs.isSensor != 0;
            ioSettings.mRelativeLinearSurfaceVelocity = toJphVec3(cs.relativeLinearSurfaceVelocity);
            ioSettings.mRelativeAngularSurfaceVelocity = toJphVec3(cs.relativeAngularSurfaceVelocity);
        }
    }

    void OnContactPersisted(const Body& inBody1, const Body& inBody2, const ContactManifold& inManifold, ContactSettings& ioSettings) override {
        if (fnPersisted) {
            JoltC_ContactSettings cs;
            cs.combinedFriction = ioSettings.mCombinedFriction;
            cs.combinedRestitution = ioSettings.mCombinedRestitution;
            cs.invMassScale1 = ioSettings.mInvMassScale1;
            cs.invInertiaScale1 = ioSettings.mInvInertiaScale1;
            cs.invMassScale2 = ioSettings.mInvMassScale2;
            cs.invInertiaScale2 = ioSettings.mInvInertiaScale2;
            cs.isSensor = ioSettings.mIsSensor ? JOLTC_TRUE : JOLTC_FALSE;
            cs.relativeLinearSurfaceVelocity = fromJphVec3(ioSettings.mRelativeLinearSurfaceVelocity);
            cs.relativeAngularSurfaceVelocity = fromJphVec3(ioSettings.mRelativeAngularSurfaceVelocity);
            fnPersisted(userData,
                reinterpret_cast<const JoltC_Body*>(&inBody1),
                reinterpret_cast<const JoltC_Body*>(&inBody2),
                reinterpret_cast<const JoltC_ContactManifold*>(&inManifold), &cs);
            ioSettings.mCombinedFriction = cs.combinedFriction;
            ioSettings.mCombinedRestitution = cs.combinedRestitution;
            ioSettings.mInvMassScale1 = cs.invMassScale1;
            ioSettings.mInvInertiaScale1 = cs.invInertiaScale1;
            ioSettings.mInvMassScale2 = cs.invMassScale2;
            ioSettings.mInvInertiaScale2 = cs.invInertiaScale2;
            ioSettings.mIsSensor = cs.isSensor != 0;
            ioSettings.mRelativeLinearSurfaceVelocity = toJphVec3(cs.relativeLinearSurfaceVelocity);
            ioSettings.mRelativeAngularSurfaceVelocity = toJphVec3(cs.relativeAngularSurfaceVelocity);
        }
    }

    void OnContactRemoved(const SubShapeIDPair& inSubShapePair) override {
        if (fnRemoved) {
            JoltC_SubShapeIDPair sp;
            sp.body1ID = inSubShapePair.GetBody1ID().GetIndexAndSequenceNumber();
            sp.subShapeID1 = inSubShapePair.GetSubShapeID1().GetValue();
            sp.body2ID = inSubShapePair.GetBody2ID().GetIndexAndSequenceNumber();
            sp.subShapeID2 = inSubShapePair.GetSubShapeID2().GetValue();
            fnRemoved(userData, &sp);
        }
    }
};

/* -------------------------------------------------------------------------- */
/*  CharacterBase (polymorphic base — ref-counted, non-owning view)           */
/* -------------------------------------------------------------------------- */
struct JoltC_CharacterBase {
    Ref<CharacterBase> ptr;
};

/* -------------------------------------------------------------------------- */
/*  CharacterVsCharacterCollision — wraps Simple variant                      */
/* -------------------------------------------------------------------------- */
struct JoltC_CharacterVsCharacterCollision {
    std::unique_ptr<CharacterVsCharacterCollisionSimple> ptr;
};

/* -------------------------------------------------------------------------- */
/*  SoftBodyCreationSettings wrapper                                          */
/* -------------------------------------------------------------------------- */
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>

struct JoltC_SoftBodyCreationSettings_Impl {
    SoftBodyCreationSettings settings;
};

#endif /* JOLTC_WRAPPERS_H */
