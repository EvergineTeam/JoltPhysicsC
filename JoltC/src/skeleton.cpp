/* JoltC - Skeleton / Ragdoll implementations
 * SPDX-License-Identifier: MIT
 */

#include <Jolt/Jolt.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Skeleton/SkeletonPose.h>
#include <Jolt/Skeleton/SkeletalAnimation.h>
#include <Jolt/Skeleton/SkeletonMapper.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Constraints/TwoBodyConstraint.h>

#include <JoltC/skeleton.h>
#include "wrappers.h"
#include "internal.h"
#include "errors_internal.h"

using namespace JPH;

/* -------------------------------------------------------------------------- */
/*  Helpers                                                                   */
/* -------------------------------------------------------------------------- */
static inline Skeleton* asSkel(JoltC_Skeleton* h)             { return reinterpret_cast<Skeleton*>(h); }
static inline const Skeleton* asSkel(const JoltC_Skeleton* h) { return reinterpret_cast<const Skeleton*>(h); }

/* SkeletonPose, SkeletalAnimation, SkeletonMapper, RagdollSettings, Ragdoll
   are all ref-counted, so we use Ref<> wrappers. */
struct JoltC_SkeletonPose_Impl {
    SkeletonPose pose;
};
struct JoltC_SkeletalAnimation_Impl {
    Ref<SkeletalAnimation> ptr;
};
struct JoltC_SkeletonMapper_Impl {
    Ref<SkeletonMapper> ptr;
};
struct JoltC_RagdollSettings_Impl {
    Ref<RagdollSettings> ptr;
};
struct JoltC_Ragdoll_Impl {
    Ref<Ragdoll> ptr;
};

static inline JoltC_SkeletonPose_Impl* asSP(JoltC_SkeletonPose* h) { return reinterpret_cast<JoltC_SkeletonPose_Impl*>(h); }
static inline const JoltC_SkeletonPose_Impl* asSP(const JoltC_SkeletonPose* h) { return reinterpret_cast<const JoltC_SkeletonPose_Impl*>(h); }
static inline JoltC_SkeletalAnimation_Impl* asSA(JoltC_SkeletalAnimation* h) { return reinterpret_cast<JoltC_SkeletalAnimation_Impl*>(h); }
static inline const JoltC_SkeletalAnimation_Impl* asSA(const JoltC_SkeletalAnimation* h) { return reinterpret_cast<const JoltC_SkeletalAnimation_Impl*>(h); }
static inline JoltC_SkeletonMapper_Impl* asSM(JoltC_SkeletonMapper* h) { return reinterpret_cast<JoltC_SkeletonMapper_Impl*>(h); }
static inline const JoltC_SkeletonMapper_Impl* asSM(const JoltC_SkeletonMapper* h) { return reinterpret_cast<const JoltC_SkeletonMapper_Impl*>(h); }
static inline JoltC_RagdollSettings_Impl* asRS(JoltC_RagdollSettings* h) { return reinterpret_cast<JoltC_RagdollSettings_Impl*>(h); }
static inline const JoltC_RagdollSettings_Impl* asRS(const JoltC_RagdollSettings* h) { return reinterpret_cast<const JoltC_RagdollSettings_Impl*>(h); }
static inline JoltC_Ragdoll_Impl* asRD(JoltC_Ragdoll* h) { return reinterpret_cast<JoltC_Ragdoll_Impl*>(h); }
static inline const JoltC_Ragdoll_Impl* asRD(const JoltC_Ragdoll* h) { return reinterpret_cast<const JoltC_Ragdoll_Impl*>(h); }

/* Thread-local string buffers for returning joint names */
static thread_local std::string tl_jointName, tl_parentName;

extern "C" {

/* ========================================================================== */
/*  Skeleton                                                                  */
/* ========================================================================== */
JOLTC_API JoltC_Skeleton* JoltC_Skeleton_Create(void) {
    JOLTC_TRY_BEGIN
    auto* s = new Skeleton;
    s->AddRef();
    return reinterpret_cast<JoltC_Skeleton*>(s);
    JOLTC_TRY_END
    return nullptr;
}
JOLTC_API void JoltC_Skeleton_Destroy(JoltC_Skeleton* skeleton) {
    if (skeleton) asSkel(skeleton)->Release();
}
JOLTC_API uint32_t JoltC_Skeleton_AddJoint(JoltC_Skeleton* skeleton, const char* name) {
    if (!skeleton || !name) return 0;
    JOLTC_TRY_BEGIN
    return asSkel(skeleton)->AddJoint(name);
    JOLTC_TRY_END
    return 0;
}
JOLTC_API uint32_t JoltC_Skeleton_AddJoint2(JoltC_Skeleton* skeleton, const char* name, int parentIndex) {
    if (!skeleton || !name) return 0;
    JOLTC_TRY_BEGIN
    return asSkel(skeleton)->AddJoint(name, parentIndex);
    JOLTC_TRY_END
    return 0;
}
JOLTC_API uint32_t JoltC_Skeleton_AddJoint3(JoltC_Skeleton* skeleton, const char* name, const char* parentName) {
    if (!skeleton || !name || !parentName) return 0;
    JOLTC_TRY_BEGIN
    return asSkel(skeleton)->AddJoint(name, parentName);
    JOLTC_TRY_END
    return 0;
}
JOLTC_API int JoltC_Skeleton_GetJointCount(const JoltC_Skeleton* skeleton) {
    return skeleton ? (int)asSkel(skeleton)->GetJointCount() : 0;
}
JOLTC_API void JoltC_Skeleton_GetJoint(const JoltC_Skeleton* skeleton, int index, JoltC_SkeletonJoint* joint) {
    if (!skeleton || !joint) return;
    const auto& joints = asSkel(skeleton)->GetJoints();
    if (index < 0 || (uint)index >= joints.size()) return;
    const auto& j = joints[index];
    tl_jointName = j.mName;
    tl_parentName = j.mParentName;
    joint->name = tl_jointName.c_str();
    joint->parentName = tl_parentName.c_str();
    joint->parentJointIndex = j.mParentJointIndex;
}
JOLTC_API int JoltC_Skeleton_GetJointIndex(const JoltC_Skeleton* skeleton, const char* name) {
    if (!skeleton || !name) return -1;
    return asSkel(skeleton)->GetJointIndex(name);
}
JOLTC_API void JoltC_Skeleton_CalculateParentJointIndices(JoltC_Skeleton* skeleton) {
    if (skeleton) asSkel(skeleton)->CalculateParentJointIndices();
}
JOLTC_API JoltC_Bool JoltC_Skeleton_AreJointsCorrectlyOrdered(const JoltC_Skeleton* skeleton) {
    return skeleton && asSkel(skeleton)->AreJointsCorrectlyOrdered() ? JOLTC_TRUE : JOLTC_FALSE;
}

/* ========================================================================== */
/*  SkeletonPose                                                              */
/* ========================================================================== */
JOLTC_API JoltC_SkeletonPose* JoltC_SkeletonPose_Create(void) {
    return reinterpret_cast<JoltC_SkeletonPose*>(new JoltC_SkeletonPose_Impl);
}
JOLTC_API void JoltC_SkeletonPose_Destroy(JoltC_SkeletonPose* pose) {
    delete asSP(pose);
}
JOLTC_API void JoltC_SkeletonPose_SetSkeleton(JoltC_SkeletonPose* pose, const JoltC_Skeleton* skeleton) {
    if (!pose) return;
    asSP(pose)->pose.SetSkeleton(asSkel(skeleton));
}
JOLTC_API const JoltC_Skeleton* JoltC_SkeletonPose_GetSkeleton(const JoltC_SkeletonPose* pose) {
    if (!pose) return nullptr;
    return reinterpret_cast<const JoltC_Skeleton*>(asSP(pose)->pose.GetSkeleton());
}
JOLTC_API void JoltC_SkeletonPose_SetRootOffset(JoltC_SkeletonPose* pose, JoltC_RVec3 offset) {
    if (pose) asSP(pose)->pose.SetRootOffset(toJphRVec3(offset));
}
JOLTC_API JoltC_RVec3 JoltC_SkeletonPose_GetRootOffset(const JoltC_SkeletonPose* pose) {
    if (!pose) return JoltC_RVec3{0,0,0};
    return fromJphRVec3(asSP(pose)->pose.GetRootOffset());
}
JOLTC_API int JoltC_SkeletonPose_GetJointCount(const JoltC_SkeletonPose* pose) {
    return pose ? (int)asSP(pose)->pose.GetJointCount() : 0;
}
JOLTC_API void JoltC_SkeletonPose_GetJointState(const JoltC_SkeletonPose* pose, int index, JoltC_Vec3* outTranslation, JoltC_Quat* outRotation) {
    if (!pose) return;
    const auto& joints = asSP(pose)->pose.GetJoints();
    if (index < 0 || (uint)index >= joints.size()) return;
    if (outTranslation) *outTranslation = fromJphVec3(joints[index].mTranslation);
    if (outRotation)    *outRotation = fromJphQuat(joints[index].mRotation);
}
JOLTC_API void JoltC_SkeletonPose_SetJointState(JoltC_SkeletonPose* pose, int index, JoltC_Vec3 translation, JoltC_Quat rotation) {
    if (!pose) return;
    auto& joints = asSP(pose)->pose.GetJoints();
    if (index < 0 || (uint)index >= joints.size()) return;
    joints[index].mTranslation = toJphVec3(translation);
    joints[index].mRotation = toJphQuat(rotation);
}
JOLTC_API void JoltC_SkeletonPose_GetJointMatrix(const JoltC_SkeletonPose* pose, int index, JoltC_Mat44* result) {
    if (!pose || !result) return;
    const auto& mats = asSP(pose)->pose.GetJointMatrices();
    if (index < 0 || (uint)index >= mats.size()) return;
    *result = fromJphMat44(mats[index]);
}
JOLTC_API void JoltC_SkeletonPose_SetJointMatrix(JoltC_SkeletonPose* pose, int index, const JoltC_Mat44* matrix) {
    if (!pose || !matrix) return;
    auto& mats = asSP(pose)->pose.GetJointMatrices();
    if (index < 0 || (uint)index >= mats.size()) return;
    mats[index] = toJphMat44(*matrix);
}
JOLTC_API void JoltC_SkeletonPose_GetJointMatrices(const JoltC_SkeletonPose* pose, JoltC_Mat44* outMatrices, int count) {
    if (!pose || !outMatrices) return;
    const auto& mats = asSP(pose)->pose.GetJointMatrices();
    int n = count < (int)mats.size() ? count : (int)mats.size();
    for (int i = 0; i < n; i++)
        outMatrices[i] = fromJphMat44(mats[i]);
}
JOLTC_API void JoltC_SkeletonPose_SetJointMatrices(JoltC_SkeletonPose* pose, const JoltC_Mat44* matrices, int count) {
    if (!pose || !matrices) return;
    auto& mats = asSP(pose)->pose.GetJointMatrices();
    int n = count < (int)mats.size() ? count : (int)mats.size();
    for (int i = 0; i < n; i++)
        mats[i] = toJphMat44(matrices[i]);
}
JOLTC_API void JoltC_SkeletonPose_CalculateJointMatrices(JoltC_SkeletonPose* pose) {
    if (pose) asSP(pose)->pose.CalculateJointMatrices();
}
JOLTC_API void JoltC_SkeletonPose_CalculateJointStates(JoltC_SkeletonPose* pose) {
    if (pose) asSP(pose)->pose.CalculateJointStates();
}
JOLTC_API void JoltC_SkeletonPose_CalculateLocalSpaceJointMatrices(const JoltC_SkeletonPose* pose, JoltC_Mat44* outMatrices) {
    if (!pose || !outMatrices) return;
    JOLTC_TRY_BEGIN
    const auto& mats = asSP(pose)->pose.GetJointMatrices();
    Array<Mat44> localMats(mats.size());
    asSP(pose)->pose.CalculateLocalSpaceJointMatrices(localMats.data());
    for (size_t i = 0; i < localMats.size(); i++)
        outMatrices[i] = fromJphMat44(localMats[i]);
    JOLTC_TRY_END
}

/* ========================================================================== */
/*  SkeletalAnimation                                                         */
/* ========================================================================== */
JOLTC_API JoltC_SkeletalAnimation* JoltC_SkeletalAnimation_Create(void) {
    auto* w = new JoltC_SkeletalAnimation_Impl;
    w->ptr = new SkeletalAnimation;
    return reinterpret_cast<JoltC_SkeletalAnimation*>(w);
}
JOLTC_API void JoltC_SkeletalAnimation_Destroy(JoltC_SkeletalAnimation* anim) {
    delete asSA(anim);
}
JOLTC_API float JoltC_SkeletalAnimation_GetDuration(const JoltC_SkeletalAnimation* anim) {
    return anim ? asSA(anim)->ptr->GetDuration() : 0;
}
JOLTC_API JoltC_Bool JoltC_SkeletalAnimation_IsLooping(const JoltC_SkeletalAnimation* anim) {
    return anim && asSA(anim)->ptr->IsLooping() ? JOLTC_TRUE : JOLTC_FALSE;
}
JOLTC_API void JoltC_SkeletalAnimation_SetIsLooping(JoltC_SkeletalAnimation* anim, JoltC_Bool looping) {
    if (anim) asSA(anim)->ptr->SetIsLooping(looping != JOLTC_FALSE);
}
JOLTC_API void JoltC_SkeletalAnimation_ScaleJoints(JoltC_SkeletalAnimation* anim, float scale) {
    if (anim) asSA(anim)->ptr->ScaleJoints(scale);
}
JOLTC_API void JoltC_SkeletalAnimation_Sample(const JoltC_SkeletalAnimation* anim, float time, JoltC_SkeletonPose* pose) {
    if (!anim || !pose) return;
    JOLTC_TRY_BEGIN
    asSA(anim)->ptr->Sample(time, asSP(pose)->pose);
    JOLTC_TRY_END
}
JOLTC_API int JoltC_SkeletalAnimation_GetAnimatedJointCount(const JoltC_SkeletalAnimation* anim) {
    return anim ? (int)asSA(anim)->ptr->GetAnimatedJoints().size() : 0;
}
JOLTC_API void JoltC_SkeletalAnimation_AddAnimatedJoint(JoltC_SkeletalAnimation* anim, const char* jointName) {
    if (!anim || !jointName) return;
    JOLTC_TRY_BEGIN
    SkeletalAnimation::AnimatedJoint aj;
    aj.mJointName = String(jointName);
    asSA(anim)->ptr->GetAnimatedJoints().push_back(std::move(aj));
    JOLTC_TRY_END
}
JOLTC_API void JoltC_SkeletalAnimation_AddKeyframe(JoltC_SkeletalAnimation* anim, int jointIndex, float time, JoltC_Vec3 translation, JoltC_Quat rotation) {
    if (!anim) return;
    JOLTC_TRY_BEGIN
    auto& joints = asSA(anim)->ptr->GetAnimatedJoints();
    if (jointIndex < 0 || (uint)jointIndex >= joints.size()) return;
    SkeletalAnimation::Keyframe kf;
    kf.mTime = time;
    kf.mTranslation = toJphVec3(translation);
    kf.mRotation = toJphQuat(rotation);
    joints[jointIndex].mKeyframes.push_back(kf);
    JOLTC_TRY_END
}

/* ========================================================================== */
/*  SkeletonMapper                                                            */
/* ========================================================================== */
JOLTC_API JoltC_SkeletonMapper* JoltC_SkeletonMapper_Create(void) {
    auto* w = new JoltC_SkeletonMapper_Impl;
    w->ptr = new SkeletonMapper;
    return reinterpret_cast<JoltC_SkeletonMapper*>(w);
}
JOLTC_API void JoltC_SkeletonMapper_Destroy(JoltC_SkeletonMapper* mapper) {
    delete asSM(mapper);
}
JOLTC_API void JoltC_SkeletonMapper_Initialize(JoltC_SkeletonMapper* mapper, const JoltC_Skeleton* sk1, const JoltC_Mat44* neutralPose1, const JoltC_Skeleton* sk2, const JoltC_Mat44* neutralPose2) {
    if (!mapper || !sk1 || !sk2 || !neutralPose1 || !neutralPose2) return;
    JOLTC_TRY_BEGIN
    int n1 = asSkel(sk1)->GetJointCount();
    int n2 = asSkel(sk2)->GetJointCount();
    Array<Mat44> np1(n1), np2(n2);
    for (int i = 0; i < n1; i++) np1[i] = toJphMat44(neutralPose1[i]);
    for (int i = 0; i < n2; i++) np2[i] = toJphMat44(neutralPose2[i]);
    asSM(mapper)->ptr->Initialize(asSkel(sk1), np1.data(), asSkel(sk2), np2.data());
    JOLTC_TRY_END
}
JOLTC_API void JoltC_SkeletonMapper_LockAllTranslations(JoltC_SkeletonMapper* mapper, const JoltC_Skeleton* sk2, const JoltC_Mat44* neutralPose2) {
    if (!mapper || !sk2 || !neutralPose2) return;
    JOLTC_TRY_BEGIN
    int n2 = asSkel(sk2)->GetJointCount();
    Array<Mat44> np2(n2);
    for (int i = 0; i < n2; i++) np2[i] = toJphMat44(neutralPose2[i]);
    asSM(mapper)->ptr->LockAllTranslations(asSkel(sk2), np2.data());
    JOLTC_TRY_END
}
JOLTC_API void JoltC_SkeletonMapper_LockTranslations(JoltC_SkeletonMapper* mapper, const JoltC_Skeleton* sk2, const JoltC_Bool* lockedTranslations, const JoltC_Mat44* neutralPose2) {
    if (!mapper || !sk2 || !lockedTranslations || !neutralPose2) return;
    JOLTC_TRY_BEGIN
    int n2 = asSkel(sk2)->GetJointCount();
    Array<bool> locked(n2);
    Array<Mat44> np2(n2);
    for (int i = 0; i < n2; i++) {
        locked[i] = (lockedTranslations[i] != JOLTC_FALSE);
        np2[i] = toJphMat44(neutralPose2[i]);
    }
    asSM(mapper)->ptr->LockTranslations(asSkel(sk2), locked.data(), np2.data());
    JOLTC_TRY_END
}
JOLTC_API void JoltC_SkeletonMapper_Map(const JoltC_SkeletonMapper* mapper, const JoltC_Mat44* pose1ModelSpace, const JoltC_Mat44* pose2LocalSpace, JoltC_Mat44* outPose2ModelSpace) {
    if (!mapper || !pose1ModelSpace || !pose2LocalSpace || !outPose2ModelSpace) return;
    JOLTC_TRY_BEGIN
    /* Determine sizes from mapper's internal mappings */
    const auto& m = asSM(mapper)->ptr->GetMappings();
    /* We need to know sizes of pose1 and pose2. Use the mappings to find max indices. */
    /* For a correct implementation, sizes should be known a priori. Use the skeleton joint counts. */
    /* Since we don't have direct skeleton access here, use the mapper's internal data. */
    /* The simplest approach: the caller provides correctly-sized arrays. Cast directly. */
    asSM(mapper)->ptr->Map(reinterpret_cast<const Mat44*>(pose1ModelSpace),
                           reinterpret_cast<const Mat44*>(pose2LocalSpace),
                           reinterpret_cast<Mat44*>(outPose2ModelSpace));
    JOLTC_TRY_END
}
JOLTC_API void JoltC_SkeletonMapper_MapReverse(const JoltC_SkeletonMapper* mapper, const JoltC_Mat44* pose2ModelSpace, JoltC_Mat44* outPose1ModelSpace) {
    if (!mapper || !pose2ModelSpace || !outPose1ModelSpace) return;
    JOLTC_TRY_BEGIN
    asSM(mapper)->ptr->MapReverse(reinterpret_cast<const Mat44*>(pose2ModelSpace),
                                  reinterpret_cast<Mat44*>(outPose1ModelSpace));
    JOLTC_TRY_END
}
JOLTC_API int JoltC_SkeletonMapper_GetMappedJointIndex(const JoltC_SkeletonMapper* mapper, int joint1Index) {
    if (!mapper) return -1;
    const auto& mappings = asSM(mapper)->ptr->GetMappings();
    for (size_t i = 0; i < mappings.size(); i++)
        if (mappings[i].mJointIdx1 == joint1Index)
            return mappings[i].mJointIdx2;
    return -1;
}
JOLTC_API JoltC_Bool JoltC_SkeletonMapper_IsJointTranslationLocked(const JoltC_SkeletonMapper* mapper, int joint2Index) {
    if (!mapper) return JOLTC_FALSE;
    const auto& locked = asSM(mapper)->ptr->GetLockedTranslations();
    for (size_t i = 0; i < locked.size(); i++)
        if (locked[i].mJointIdx == joint2Index)
            return JOLTC_TRUE;
    return JOLTC_FALSE;
}

/* ========================================================================== */
/*  RagdollSettings                                                           */
/* ========================================================================== */
JOLTC_API JoltC_RagdollSettings* JoltC_RagdollSettings_Create(void) {
    auto* w = new JoltC_RagdollSettings_Impl;
    w->ptr = new RagdollSettings;
    return reinterpret_cast<JoltC_RagdollSettings*>(w);
}
JOLTC_API void JoltC_RagdollSettings_Destroy(JoltC_RagdollSettings* s) {
    delete asRS(s);
}
JOLTC_API const JoltC_Skeleton* JoltC_RagdollSettings_GetSkeleton(const JoltC_RagdollSettings* s) {
    if (!s) return nullptr;
    return reinterpret_cast<const JoltC_Skeleton*>(asRS(s)->ptr->GetSkeleton());
}
JOLTC_API void JoltC_RagdollSettings_SetSkeleton(JoltC_RagdollSettings* s, JoltC_Skeleton* skeleton) {
    if (!s) return;
    asRS(s)->ptr->mSkeleton = asSkel(skeleton);
}
JOLTC_API JoltC_Bool JoltC_RagdollSettings_Stabilize(JoltC_RagdollSettings* s) {
    if (!s) return JOLTC_FALSE;
    JOLTC_TRY_BEGIN
    return asRS(s)->ptr->Stabilize() ? JOLTC_TRUE : JOLTC_FALSE;
    JOLTC_TRY_END
    return JOLTC_FALSE;
}
JOLTC_API void JoltC_RagdollSettings_DisableParentChildCollisions(JoltC_RagdollSettings* s, const JoltC_Mat44* jointMatrices, float minSeparationDistance) {
    if (!s) return;
    JOLTC_TRY_BEGIN
    if (jointMatrices) {
        int count = (int)asRS(s)->ptr->mParts.size();
        Array<Mat44> mats(count);
        for (int i = 0; i < count; i++) mats[i] = toJphMat44(jointMatrices[i]);
        asRS(s)->ptr->DisableParentChildCollisions(mats.data(), minSeparationDistance);
    } else {
        asRS(s)->ptr->DisableParentChildCollisions(nullptr, minSeparationDistance);
    }
    JOLTC_TRY_END
}
JOLTC_API void JoltC_RagdollSettings_CalculateBodyIndexToConstraintIndex(JoltC_RagdollSettings* s) {
    if (s) asRS(s)->ptr->CalculateBodyIndexToConstraintIndex();
}
JOLTC_API int JoltC_RagdollSettings_GetConstraintIndexForBodyIndex(JoltC_RagdollSettings* s, int bodyIndex) {
    if (!s) return -1;
    return asRS(s)->ptr->GetConstraintIndexForBodyIndex(bodyIndex);
}
JOLTC_API void JoltC_RagdollSettings_CalculateConstraintIndexToBodyIdxPair(JoltC_RagdollSettings* s) {
    if (s) asRS(s)->ptr->CalculateConstraintIndexToBodyIdxPair();
}
JOLTC_API void JoltC_RagdollSettings_ResizeParts(JoltC_RagdollSettings* s, int count) {
    if (!s || count < 0) return;
    asRS(s)->ptr->mParts.resize(count);
}
JOLTC_API int JoltC_RagdollSettings_GetPartCount(const JoltC_RagdollSettings* s) {
    return s ? (int)asRS(s)->ptr->mParts.size() : 0;
}
JOLTC_API void JoltC_RagdollSettings_SetPartShape(JoltC_RagdollSettings* s, int partIndex, const JoltC_Shape* shape) {
    if (!s || partIndex < 0 || (uint)partIndex >= asRS(s)->ptr->mParts.size()) return;
    asRS(s)->ptr->mParts[partIndex].SetShape(reinterpret_cast<const Shape*>(shape));
}
JOLTC_API void JoltC_RagdollSettings_SetPartPosition(JoltC_RagdollSettings* s, int partIndex, JoltC_RVec3 position) {
    if (!s || partIndex < 0 || (uint)partIndex >= asRS(s)->ptr->mParts.size()) return;
    asRS(s)->ptr->mParts[partIndex].mPosition = toJphRVec3(position);
}
JOLTC_API void JoltC_RagdollSettings_SetPartRotation(JoltC_RagdollSettings* s, int partIndex, JoltC_Quat rotation) {
    if (!s || partIndex < 0 || (uint)partIndex >= asRS(s)->ptr->mParts.size()) return;
    asRS(s)->ptr->mParts[partIndex].mRotation = toJphQuat(rotation);
}
JOLTC_API void JoltC_RagdollSettings_SetPartMotionType(JoltC_RagdollSettings* s, int partIndex, JoltC_MotionType motionType) {
    if (!s || partIndex < 0 || (uint)partIndex >= asRS(s)->ptr->mParts.size()) return;
    asRS(s)->ptr->mParts[partIndex].mMotionType = (EMotionType)motionType;
}
JOLTC_API void JoltC_RagdollSettings_SetPartObjectLayer(JoltC_RagdollSettings* s, int partIndex, JoltC_ObjectLayer layer) {
    if (!s || partIndex < 0 || (uint)partIndex >= asRS(s)->ptr->mParts.size()) return;
    asRS(s)->ptr->mParts[partIndex].mObjectLayer = (ObjectLayer)layer;
}
JOLTC_API void JoltC_RagdollSettings_SetPartMassProperties(JoltC_RagdollSettings* s, int partIndex, float mass) {
    if (!s || partIndex < 0 || (uint)partIndex >= asRS(s)->ptr->mParts.size()) return;
    asRS(s)->ptr->mParts[partIndex].mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
    asRS(s)->ptr->mParts[partIndex].mMassPropertiesOverride.mMass = mass;
}
JOLTC_API void JoltC_RagdollSettings_SetPartToParent(JoltC_RagdollSettings* settings, int partIndex, int parentIndex, const JoltC_TwoBodyConstraintSettings* constraintSettings) {
    if (!settings || partIndex < 0 || (uint)partIndex >= asRS(settings)->ptr->mParts.size()) return;
    JOLTC_TRY_BEGIN
    (void)parentIndex; /* parent is determined by the skeleton */
    if (constraintSettings) {
        auto* cs = reinterpret_cast<const TwoBodyConstraintSettings*>(constraintSettings);
        asRS(settings)->ptr->mParts[partIndex].mToParent = const_cast<TwoBodyConstraintSettings*>(cs);
    } else {
        asRS(settings)->ptr->mParts[partIndex].mToParent = nullptr;
    }
    JOLTC_TRY_END
}
JOLTC_API JoltC_Ragdoll* JoltC_RagdollSettings_CreateRagdoll(JoltC_RagdollSettings* s, JoltC_PhysicsSystem* system, JoltC_CollisionGroupID collisionGroup, uint64_t userData) {
    if (!s || !system) return nullptr;
    JOLTC_TRY_BEGIN
    PhysicsSystem* ps = reinterpret_cast<PhysicsSystem*>(system);
    Ragdoll* rd = asRS(s)->ptr->CreateRagdoll(collisionGroup, userData, ps);
    if (!rd) return nullptr;
    auto* w = new JoltC_Ragdoll_Impl;
    w->ptr = rd;
    return reinterpret_cast<JoltC_Ragdoll*>(w);
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  Ragdoll                                                                   */
/* ========================================================================== */
JOLTC_API void JoltC_Ragdoll_Destroy(JoltC_Ragdoll* ragdoll) {
    delete asRD(ragdoll);
}
JOLTC_API void JoltC_Ragdoll_AddToPhysicsSystem(JoltC_Ragdoll* ragdoll, JoltC_Activation activation, JoltC_Bool lockBodies) {
    if (!ragdoll) return;
    JOLTC_TRY_BEGIN
    asRD(ragdoll)->ptr->AddToPhysicsSystem((EActivation)activation, lockBodies != JOLTC_FALSE);
    JOLTC_TRY_END
}
JOLTC_API void JoltC_Ragdoll_RemoveFromPhysicsSystem(JoltC_Ragdoll* ragdoll, JoltC_Bool lockBodies) {
    if (!ragdoll) return;
    JOLTC_TRY_BEGIN
    asRD(ragdoll)->ptr->RemoveFromPhysicsSystem(lockBodies != JOLTC_FALSE);
    JOLTC_TRY_END
}
JOLTC_API void JoltC_Ragdoll_Activate(JoltC_Ragdoll* ragdoll, JoltC_Bool lockBodies) {
    if (!ragdoll) return;
    asRD(ragdoll)->ptr->Activate(lockBodies != JOLTC_FALSE);
}
JOLTC_API JoltC_Bool JoltC_Ragdoll_IsActive(const JoltC_Ragdoll* ragdoll, JoltC_Bool lockBodies) {
    if (!ragdoll) return JOLTC_FALSE;
    return asRD(ragdoll)->ptr->IsActive(lockBodies != JOLTC_FALSE) ? JOLTC_TRUE : JOLTC_FALSE;
}
JOLTC_API void JoltC_Ragdoll_ResetWarmStart(JoltC_Ragdoll* ragdoll) {
    if (ragdoll) asRD(ragdoll)->ptr->ResetWarmStart();
}
JOLTC_API void JoltC_Ragdoll_SetPose(JoltC_Ragdoll* ragdoll, const JoltC_SkeletonPose* pose, JoltC_Bool lockBodies) {
    if (!ragdoll || !pose) return;
    JOLTC_TRY_BEGIN
    asRD(ragdoll)->ptr->SetPose(asSP(pose)->pose, lockBodies != JOLTC_FALSE);
    JOLTC_TRY_END
}
JOLTC_API void JoltC_Ragdoll_SetPose2(JoltC_Ragdoll* ragdoll, JoltC_RVec3 rootOffset, const JoltC_Mat44* jointMatrices, JoltC_Bool lockBodies) {
    if (!ragdoll || !jointMatrices) return;
    JOLTC_TRY_BEGIN
    asRD(ragdoll)->ptr->SetPose(toJphRVec3(rootOffset), reinterpret_cast<const Mat44*>(jointMatrices), lockBodies != JOLTC_FALSE);
    JOLTC_TRY_END
}
JOLTC_API void JoltC_Ragdoll_GetPose(const JoltC_Ragdoll* ragdoll, JoltC_SkeletonPose* outPose, JoltC_Bool lockBodies) {
    if (!ragdoll || !outPose) return;
    JOLTC_TRY_BEGIN
    asRD(ragdoll)->ptr->GetPose(asSP(outPose)->pose, lockBodies != JOLTC_FALSE);
    JOLTC_TRY_END
}
JOLTC_API void JoltC_Ragdoll_GetPose2(const JoltC_Ragdoll* ragdoll, JoltC_RVec3* outRootOffset, JoltC_Mat44* outJointMatrices, JoltC_Bool lockBodies) {
    if (!ragdoll || !outRootOffset || !outJointMatrices) return;
    JOLTC_TRY_BEGIN
    RVec3 offset;
    asRD(ragdoll)->ptr->GetPose(offset, reinterpret_cast<Mat44*>(outJointMatrices), lockBodies != JOLTC_FALSE);
    *outRootOffset = fromJphRVec3(offset);
    JOLTC_TRY_END
}
JOLTC_API void JoltC_Ragdoll_DriveToPoseUsingMotors(JoltC_Ragdoll* ragdoll, const JoltC_SkeletonPose* pose) {
    if (!ragdoll || !pose) return;
    JOLTC_TRY_BEGIN
    asRD(ragdoll)->ptr->DriveToPoseUsingMotors(asSP(pose)->pose);
    JOLTC_TRY_END
}
JOLTC_API void JoltC_Ragdoll_DriveToPoseUsingKinematics(JoltC_Ragdoll* ragdoll, const JoltC_SkeletonPose* pose, float deltaTime, JoltC_Bool lockBodies) {
    if (!ragdoll || !pose) return;
    JOLTC_TRY_BEGIN
    asRD(ragdoll)->ptr->DriveToPoseUsingKinematics(asSP(pose)->pose, deltaTime, lockBodies != JOLTC_FALSE);
    JOLTC_TRY_END
}
JOLTC_API void JoltC_Ragdoll_GetRootTransform(const JoltC_Ragdoll* ragdoll, JoltC_RVec3* outPosition, JoltC_Quat* outRotation, JoltC_Bool lockBodies) {
    if (!ragdoll) return;
    JOLTC_TRY_BEGIN
    RVec3 pos;
    Quat rot;
    asRD(ragdoll)->ptr->GetRootTransform(pos, rot, lockBodies != JOLTC_FALSE);
    if (outPosition) *outPosition = fromJphRVec3(pos);
    if (outRotation) *outRotation = fromJphQuat(rot);
    JOLTC_TRY_END
}
JOLTC_API int JoltC_Ragdoll_GetBodyCount(const JoltC_Ragdoll* ragdoll) {
    return ragdoll ? (int)asRD(ragdoll)->ptr->GetBodyCount() : 0;
}
JOLTC_API JoltC_BodyID JoltC_Ragdoll_GetBodyID(const JoltC_Ragdoll* ragdoll, int bodyIndex) {
    if (!ragdoll) return JOLTC_BODY_ID_INVALID;
    return asRD(ragdoll)->ptr->GetBodyID(bodyIndex).GetIndexAndSequenceNumber();
}
JOLTC_API int JoltC_Ragdoll_GetConstraintCount(const JoltC_Ragdoll* ragdoll) {
    return ragdoll ? (int)asRD(ragdoll)->ptr->GetConstraintCount() : 0;
}
JOLTC_API JoltC_Constraint* JoltC_Ragdoll_GetConstraint(JoltC_Ragdoll* ragdoll, int constraintIndex) {
    if (!ragdoll) return nullptr;
    TwoBodyConstraint* c = asRD(ragdoll)->ptr->GetConstraint(constraintIndex);
    return reinterpret_cast<JoltC_Constraint*>(c);
}
JOLTC_API const JoltC_RagdollSettings* JoltC_Ragdoll_GetRagdollSettings(const JoltC_Ragdoll* ragdoll) {
    /* This returns a borrowed reference; the caller should not destroy it. */
    /* We return a thread-local wrapper pointing to the internal settings. */
    if (!ragdoll) return nullptr;
    static thread_local JoltC_RagdollSettings_Impl tl_rs;
    tl_rs.ptr = const_cast<RagdollSettings*>(asRD(ragdoll)->ptr->GetRagdollSettings());
    return reinterpret_cast<const JoltC_RagdollSettings*>(&tl_rs);
}

} /* extern "C" */
