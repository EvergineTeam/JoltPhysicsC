/* JoltC - Soft body implementations
 * SPDX-License-Identifier: MIT
 */

#include <Jolt/Jolt.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/SoftBody/SoftBodyContactListener.h>
#include <Jolt/Physics/SoftBody/SoftBodyManifold.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/Body.h>

#include <JoltC/soft_body.h>
#include "internal.h"
#include "errors_internal.h"

using namespace JPH;

/* wrappers.h cannot live inside extern "C"; include it before opening the block
 * (pattern of the SoftBodyCreationSettings section in physics_system.cpp). */
#include "wrappers.h"

/* -------------------------------------------------------------------------- */
/*  Helpers                                                                   */
/* -------------------------------------------------------------------------- */
static inline SoftBodySharedSettings* asSBSS(JoltC_SoftBodySharedSettings* h) { return reinterpret_cast<SoftBodySharedSettings*>(h); }
static inline const SoftBodySharedSettings* asSBSS(const JoltC_SoftBodySharedSettings* h) { return reinterpret_cast<const SoftBodySharedSettings*>(h); }
static inline JoltC_SoftBodySharedSettings* fromSBSS(SoftBodySharedSettings* p) { return reinterpret_cast<JoltC_SoftBodySharedSettings*>(p); }

static inline JoltC_SoftBodyCreationSettings_Impl* asSBCS(const JoltC_SoftBodyCreationSettings* s) {
    return reinterpret_cast<JoltC_SoftBodyCreationSettings_Impl*>(const_cast<JoltC_SoftBodyCreationSettings*>(s));
}

static inline SoftBodyMotionProperties* asSBMP(JoltC_SoftBodyMotionProperties* h) { return reinterpret_cast<SoftBodyMotionProperties*>(h); }
static inline const SoftBodyMotionProperties* asSBMP(const JoltC_SoftBodyMotionProperties* h) { return reinterpret_cast<const SoftBodyMotionProperties*>(h); }

static inline const SoftBodyManifold* asManifold(const JoltC_SoftBodyManifold* h) { return reinterpret_cast<const SoftBodyManifold*>(h); }

/* The listener that forwards into C. Callbacks run while every body is locked, so the C side must
 * not call back into the body interface -- the same contract Jolt states for the C++ listener. */
struct JoltC_SoftBodyContactListener_Impl final : public SoftBodyContactListener
{
    JoltC_OnSoftBodyContactValidateFn onValidate = nullptr;
    JoltC_OnSoftBodyContactAddedFn onAdded = nullptr;
    void* userData = nullptr;

    SoftBodyValidateResult OnSoftBodyContactValidate(const Body& softBody, const Body& otherBody, SoftBodyContactSettings& ioSettings) override
    {
        if (!onValidate)
            return SoftBodyValidateResult::AcceptContact;

        JoltC_SoftBodyContactSettings settings;
        settings.invMassScale1 = ioSettings.mInvMassScale1;
        settings.invMassScale2 = ioSettings.mInvMassScale2;
        settings.invInertiaScale2 = ioSettings.mInvInertiaScale2;
        settings.isSensor = ioSettings.mIsSensor ? 1 : 0;

        JoltC_SoftBodyValidateResult result = onValidate(
            userData,
            reinterpret_cast<const JoltC_Body*>(&softBody),
            reinterpret_cast<const JoltC_Body*>(&otherBody),
            &settings);

        ioSettings.mInvMassScale1 = settings.invMassScale1;
        ioSettings.mInvMassScale2 = settings.invMassScale2;
        ioSettings.mInvInertiaScale2 = settings.invInertiaScale2;
        ioSettings.mIsSensor = settings.isSensor != 0;

        return result == JOLTC_SOFT_BODY_VALIDATE_RESULT_REJECT_CONTACT
            ? SoftBodyValidateResult::RejectContact
            : SoftBodyValidateResult::AcceptContact;
    }

    void OnSoftBodyContactAdded(const Body& softBody, const SoftBodyManifold& manifold) override
    {
        if (onAdded)
            onAdded(userData,
                reinterpret_cast<const JoltC_Body*>(&softBody),
                reinterpret_cast<const JoltC_SoftBodyManifold*>(&manifold));
    }
};

extern "C" {

/* ========================================================================== */
/*  SoftBodySharedSettings                                                    */
/* ========================================================================== */
JOLTC_API JoltC_SoftBodySharedSettings* JoltC_SoftBodySharedSettings_Create(void)
{
    JOLTC_TRY_BEGIN
    SoftBodySharedSettings* settings = new SoftBodySharedSettings();
    settings->AddRef();
    return fromSBSS(settings);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_SoftBodySharedSettings_AddRef(const JoltC_SoftBodySharedSettings* settings)
{
    if (!settings) return;
    asSBSS(settings)->AddRef();
}

JOLTC_API void JoltC_SoftBodySharedSettings_Release(const JoltC_SoftBodySharedSettings* settings)
{
    if (!settings) return;
    asSBSS(settings)->Release();
}

JOLTC_API uint32_t JoltC_SoftBodySharedSettings_AddVertex(JoltC_SoftBodySharedSettings* settings, float x, float y, float z, float invMass)
{
    if (!settings) return 0;
    JOLTC_TRY_BEGIN
    SoftBodySharedSettings* s = asSBSS(settings);
    SoftBodySharedSettings::Vertex v;
    v.mPosition = Float3(x, y, z);
    v.mInvMass = invMass;
    s->mVertices.push_back(v);
    return (uint32_t)(s->mVertices.size() - 1);
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_SoftBodySharedSettings_AddFace(JoltC_SoftBodySharedSettings* settings, uint32_t vertex0, uint32_t vertex1, uint32_t vertex2, uint32_t materialIndex)
{
    if (!settings) return;
    JOLTC_TRY_BEGIN
    SoftBodySharedSettings::Face f;
    f.mVertex[0] = vertex0;
    f.mVertex[1] = vertex1;
    f.mVertex[2] = vertex2;
    f.mMaterialIndex = materialIndex;
    asSBSS(settings)->AddFace(f);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_SoftBodySharedSettings_CreateConstraints(JoltC_SoftBodySharedSettings* settings, float compliance, float shearCompliance, float bendCompliance, JoltC_SoftBodyBendType bendType)
{
    if (!settings) return;
    JOLTC_TRY_BEGIN
    SoftBodySharedSettings::VertexAttributes attributes;
    attributes.mCompliance = compliance;
    attributes.mShearCompliance = shearCompliance;
    attributes.mBendCompliance = bendCompliance;
    asSBSS(settings)->CreateConstraints(&attributes, 1, static_cast<SoftBodySharedSettings::EBendType>(bendType));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_SoftBodySharedSettings_CalculateEdgeLengths(JoltC_SoftBodySharedSettings* settings)
{
    if (!settings) return;
    JOLTC_TRY_BEGIN
    asSBSS(settings)->CalculateEdgeLengths();
    JOLTC_TRY_END
}

JOLTC_API void JoltC_SoftBodySharedSettings_CalculateVolumeConstraintVolumes(JoltC_SoftBodySharedSettings* settings)
{
    if (!settings) return;
    JOLTC_TRY_BEGIN
    asSBSS(settings)->CalculateVolumeConstraintVolumes();
    JOLTC_TRY_END
}

JOLTC_API void JoltC_SoftBodySharedSettings_Optimize(JoltC_SoftBodySharedSettings* settings)
{
    if (!settings) return;
    JOLTC_TRY_BEGIN
    asSBSS(settings)->Optimize();
    JOLTC_TRY_END
}

JOLTC_API JoltC_SoftBodySharedSettings* JoltC_SoftBodySharedSettings_CreateCube(uint32_t gridSize, float gridSpacing)
{
    if (gridSize < 2) return nullptr;
    JOLTC_TRY_BEGIN
    Ref<SoftBodySharedSettings> settings = SoftBodySharedSettings::sCreateCube(gridSize, gridSpacing);
    settings->AddRef(); /* hand one reference to the caller; Ref releases its own on scope exit */
    return fromSBSS(settings.GetPtr());
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API uint32_t JoltC_SoftBodySharedSettings_GetVertexCount(const JoltC_SoftBodySharedSettings* settings)
{
    if (!settings) return 0;
    return (uint32_t)asSBSS(settings)->mVertices.size();
}

JOLTC_API uint32_t JoltC_SoftBodySharedSettings_GetFaceCount(const JoltC_SoftBodySharedSettings* settings)
{
    if (!settings) return 0;
    return (uint32_t)asSBSS(settings)->mFaces.size();
}

JOLTC_API void JoltC_SoftBodySharedSettings_GetFace(const JoltC_SoftBodySharedSettings* settings, uint32_t index, uint32_t* outVertex0, uint32_t* outVertex1, uint32_t* outVertex2)
{
    if (!settings || !outVertex0 || !outVertex1 || !outVertex2) return;
    const SoftBodySharedSettings* s = asSBSS(settings);
    if (index >= s->mFaces.size()) return;
    const SoftBodySharedSettings::Face& f = s->mFaces[index];
    *outVertex0 = f.mVertex[0];
    *outVertex1 = f.mVertex[1];
    *outVertex2 = f.mVertex[2];
}

/* ========================================================================== */
/*  SoftBodyCreationSettings setters                                          */
/* ========================================================================== */
JOLTC_API void JoltC_SoftBodyCreationSettings_SetSettings(JoltC_SoftBodyCreationSettings* settings, const JoltC_SoftBodySharedSettings* sharedSettings)
{
    if (!settings) return;
    asSBCS(settings)->settings.mSettings = asSBSS(sharedSettings);
}

JOLTC_API void JoltC_SoftBodyCreationSettings_SetPosition(JoltC_SoftBodyCreationSettings* settings, JoltC_RVec3 position)
{
    if (!settings) return;
    asSBCS(settings)->settings.mPosition = toJphRVec3(position);
}

JOLTC_API void JoltC_SoftBodyCreationSettings_SetRotation(JoltC_SoftBodyCreationSettings* settings, JoltC_Quat rotation)
{
    if (!settings) return;
    asSBCS(settings)->settings.mRotation = toJphQuat(rotation);
}

JOLTC_API void JoltC_SoftBodyCreationSettings_SetObjectLayer(JoltC_SoftBodyCreationSettings* settings, JoltC_ObjectLayer objectLayer)
{
    if (!settings) return;
    asSBCS(settings)->settings.mObjectLayer = objectLayer;
}

JOLTC_API void JoltC_SoftBodyCreationSettings_SetNumIterations(JoltC_SoftBodyCreationSettings* settings, uint32_t numIterations)
{
    if (!settings) return;
    asSBCS(settings)->settings.mNumIterations = numIterations;
}

JOLTC_API void JoltC_SoftBodyCreationSettings_SetLinearDamping(JoltC_SoftBodyCreationSettings* settings, float linearDamping)
{
    if (!settings) return;
    asSBCS(settings)->settings.mLinearDamping = linearDamping;
}

JOLTC_API void JoltC_SoftBodyCreationSettings_SetMaxLinearVelocity(JoltC_SoftBodyCreationSettings* settings, float maxLinearVelocity)
{
    if (!settings) return;
    asSBCS(settings)->settings.mMaxLinearVelocity = maxLinearVelocity;
}

JOLTC_API void JoltC_SoftBodyCreationSettings_SetRestitution(JoltC_SoftBodyCreationSettings* settings, float restitution)
{
    if (!settings) return;
    asSBCS(settings)->settings.mRestitution = restitution;
}

JOLTC_API void JoltC_SoftBodyCreationSettings_SetFriction(JoltC_SoftBodyCreationSettings* settings, float friction)
{
    if (!settings) return;
    asSBCS(settings)->settings.mFriction = friction;
}

JOLTC_API void JoltC_SoftBodyCreationSettings_SetPressure(JoltC_SoftBodyCreationSettings* settings, float pressure)
{
    if (!settings) return;
    asSBCS(settings)->settings.mPressure = pressure;
}

JOLTC_API void JoltC_SoftBodyCreationSettings_SetGravityFactor(JoltC_SoftBodyCreationSettings* settings, float gravityFactor)
{
    if (!settings) return;
    asSBCS(settings)->settings.mGravityFactor = gravityFactor;
}

JOLTC_API void JoltC_SoftBodyCreationSettings_SetVertexRadius(JoltC_SoftBodyCreationSettings* settings, float vertexRadius)
{
    if (!settings) return;
    asSBCS(settings)->settings.mVertexRadius = vertexRadius;
}

JOLTC_API void JoltC_SoftBodyCreationSettings_SetUpdatePosition(JoltC_SoftBodyCreationSettings* settings, JoltC_Bool updatePosition)
{
    if (!settings) return;
    asSBCS(settings)->settings.mUpdatePosition = updatePosition != 0;
}

JOLTC_API void JoltC_SoftBodyCreationSettings_SetMakeRotationIdentity(JoltC_SoftBodyCreationSettings* settings, JoltC_Bool makeRotationIdentity)
{
    if (!settings) return;
    asSBCS(settings)->settings.mMakeRotationIdentity = makeRotationIdentity != 0;
}

JOLTC_API void JoltC_SoftBodyCreationSettings_SetAllowSleeping(JoltC_SoftBodyCreationSettings* settings, JoltC_Bool allowSleeping)
{
    if (!settings) return;
    asSBCS(settings)->settings.mAllowSleeping = allowSleeping != 0;
}

JOLTC_API void JoltC_SoftBodyCreationSettings_SetUserData(JoltC_SoftBodyCreationSettings* settings, uint64_t userData)
{
    if (!settings) return;
    asSBCS(settings)->settings.mUserData = userData;
}

/* ========================================================================== */
/*  SoftBodyMotionProperties                                                  */
/* ========================================================================== */
JOLTC_API JoltC_SoftBodyMotionProperties* JoltC_Body_GetSoftBodyMotionProperties(JoltC_Body* body)
{
    if (!body) return nullptr;
    Body* b = reinterpret_cast<Body*>(body);
    if (!b->IsSoftBody()) return nullptr;
    return reinterpret_cast<JoltC_SoftBodyMotionProperties*>(static_cast<SoftBodyMotionProperties*>(b->GetMotionProperties()));
}

JOLTC_API uint32_t JoltC_SoftBodyMotionProperties_GetVertexCount(const JoltC_SoftBodyMotionProperties* motionProperties)
{
    if (!motionProperties) return 0;
    return (uint32_t)asSBMP(motionProperties)->GetVertices().size();
}

JOLTC_API uint32_t JoltC_SoftBodyMotionProperties_GetVertexPositions(const JoltC_SoftBodyMotionProperties* motionProperties, JoltC_Vec3* outPositions, uint32_t capacity)
{
    if (!motionProperties || !outPositions) return 0;
    const Array<SoftBodyVertex>& vertices = asSBMP(motionProperties)->GetVertices();
    uint32_t count = (uint32_t)vertices.size();
    if (count > capacity) count = capacity;
    for (uint32_t i = 0; i < count; ++i)
        outPositions[i] = fromJphVec3(vertices[i].mPosition);
    return count;
}

JOLTC_API void JoltC_SoftBodyMotionProperties_GetVertexPosition(const JoltC_SoftBodyMotionProperties* motionProperties, uint32_t index, JoltC_Vec3* outPosition)
{
    if (!motionProperties || !outPosition) return;
    const Array<SoftBodyVertex>& vertices = asSBMP(motionProperties)->GetVertices();
    if (index >= vertices.size()) return;
    *outPosition = fromJphVec3(vertices[index].mPosition);
}

JOLTC_API float JoltC_SoftBodyMotionProperties_GetVolume(const JoltC_SoftBodyMotionProperties* motionProperties)
{
    if (!motionProperties) return 0.0f;
    return asSBMP(motionProperties)->GetVolume();
}

JOLTC_API void JoltC_SoftBodyMotionProperties_GetLocalBounds(const JoltC_SoftBodyMotionProperties* motionProperties, JoltC_Vec3* outMin, JoltC_Vec3* outMax)
{
    if (!motionProperties || !outMin || !outMax) return;
    const AABox& bounds = asSBMP(motionProperties)->GetLocalBounds();
    *outMin = fromJphVec3(bounds.mMin);
    *outMax = fromJphVec3(bounds.mMax);
}

JOLTC_API float JoltC_SoftBodyMotionProperties_GetPressure(const JoltC_SoftBodyMotionProperties* motionProperties)
{
    if (!motionProperties) return 0.0f;
    return asSBMP(motionProperties)->GetPressure();
}

JOLTC_API void JoltC_SoftBodyMotionProperties_SetPressure(JoltC_SoftBodyMotionProperties* motionProperties, float pressure)
{
    if (!motionProperties) return;
    asSBMP(motionProperties)->SetPressure(pressure);
}

/* ========================================================================== */
/*  SoftBodyContactListener                                                   */
/* ========================================================================== */
JOLTC_API JoltC_SoftBodyContactListener* JoltC_SoftBodyContactListener_Create(
    JoltC_OnSoftBodyContactValidateFn onValidate,
    JoltC_OnSoftBodyContactAddedFn onAdded,
    void* userData)
{
    JOLTC_TRY_BEGIN
    auto* listener = new JoltC_SoftBodyContactListener_Impl();
    listener->onValidate = onValidate;
    listener->onAdded = onAdded;
    listener->userData = userData;
    return reinterpret_cast<JoltC_SoftBodyContactListener*>(listener);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_SoftBodyContactListener_Destroy(JoltC_SoftBodyContactListener* listener)
{
    delete reinterpret_cast<JoltC_SoftBodyContactListener_Impl*>(listener);
}

JOLTC_API void JoltC_PhysicsSystem_SetSoftBodyContactListener(JoltC_PhysicsSystem* system, JoltC_SoftBodyContactListener* listener)
{
    if (!system) return;
    system->ptr->SetSoftBodyContactListener(reinterpret_cast<JoltC_SoftBodyContactListener_Impl*>(listener));
}

/* ========================================================================== */
/*  SoftBodyManifold                                                          */
/* ========================================================================== */
JOLTC_API uint32_t JoltC_SoftBodyManifold_GetVertexCount(const JoltC_SoftBodyManifold* manifold)
{
    if (!manifold) return 0;
    return (uint32_t)asManifold(manifold)->GetVertices().size();
}

JOLTC_API JoltC_Bool JoltC_SoftBodyManifold_HasContact(const JoltC_SoftBodyManifold* manifold, uint32_t vertexIndex)
{
    if (!manifold) return 0;
    const Array<SoftBodyVertex>& vertices = asManifold(manifold)->GetVertices();
    if (vertexIndex >= vertices.size()) return 0;
    return asManifold(manifold)->HasContact(vertices[vertexIndex]) ? 1 : 0;
}

JOLTC_API void JoltC_SoftBodyManifold_GetLocalContactPoint(const JoltC_SoftBodyManifold* manifold, uint32_t vertexIndex, JoltC_Vec3* outPoint)
{
    if (!manifold || !outPoint) return;
    const Array<SoftBodyVertex>& vertices = asManifold(manifold)->GetVertices();
    if (vertexIndex >= vertices.size()) return;
    *outPoint = fromJphVec3(asManifold(manifold)->GetLocalContactPoint(vertices[vertexIndex]));
}

JOLTC_API void JoltC_SoftBodyManifold_GetContactNormal(const JoltC_SoftBodyManifold* manifold, uint32_t vertexIndex, JoltC_Vec3* outNormal)
{
    if (!manifold || !outNormal) return;
    const Array<SoftBodyVertex>& vertices = asManifold(manifold)->GetVertices();
    if (vertexIndex >= vertices.size()) return;
    *outNormal = fromJphVec3(asManifold(manifold)->GetContactNormal(vertices[vertexIndex]));
}

JOLTC_API JoltC_BodyID JoltC_SoftBodyManifold_GetContactBodyID(const JoltC_SoftBodyManifold* manifold, uint32_t vertexIndex)
{
    if (!manifold) return JOLTC_BODY_ID_INVALID;
    const Array<SoftBodyVertex>& vertices = asManifold(manifold)->GetVertices();
    if (vertexIndex >= vertices.size()) return JOLTC_BODY_ID_INVALID;
    return fromJphBodyID(asManifold(manifold)->GetContactBodyID(vertices[vertexIndex]));
}

JOLTC_API uint32_t JoltC_SoftBodyManifold_GetNumSensorContacts(const JoltC_SoftBodyManifold* manifold)
{
    if (!manifold) return 0;
    return asManifold(manifold)->GetNumSensorContacts();
}

JOLTC_API JoltC_BodyID JoltC_SoftBodyManifold_GetSensorContactBodyID(const JoltC_SoftBodyManifold* manifold, uint32_t index)
{
    if (!manifold) return JOLTC_BODY_ID_INVALID;
    if (index >= asManifold(manifold)->GetNumSensorContacts()) return JOLTC_BODY_ID_INVALID;
    return fromJphBodyID(asManifold(manifold)->GetSensorContactBodyID(index));
}

} /* extern "C" */
