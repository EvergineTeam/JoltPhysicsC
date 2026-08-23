/* JoltC - Query implementation (narrow & broad phase)
 * SPDX-License-Identifier: MIT
 */

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h>

#include "errors_internal.h"
#include "internal.h"
#include "wrappers.h"

#include <JoltC/query.h>

using namespace JPH;

/* -------------------------------------------------------------------------- */
/*  Helpers: get filter references or defaults for nullable params            */
/* -------------------------------------------------------------------------- */
static const BroadPhaseLayerFilter& bpf(const JoltC_BroadPhaseLayerFilter* f) {
    static BroadPhaseLayerFilter sDefault;
    return f ? *f->ptr : sDefault;
}
static const ObjectLayerFilter& olf(const JoltC_ObjectLayerFilter* f) {
    static ObjectLayerFilter sDefault;
    return f ? *f->ptr : sDefault;
}
static const BodyFilter& bf(const JoltC_BodyFilter* f) {
    static BodyFilter sDefault;
    return f ? *f->ptr : sDefault;
}
static const ShapeFilter& sf(const JoltC_ShapeFilter* f) {
    static ShapeFilter sDefault;
    return f ? *f->ptr : sDefault;
}

/* -------------------------------------------------------------------------- */
/*  Helpers: load Mat44 from JoltC_Mat44                                      */
/* -------------------------------------------------------------------------- */
static inline Mat44 toJphMat44FromBlittable(JoltC_Mat44 m) {
    return Mat44::sLoadFloat4x4(reinterpret_cast<const Float4*>(m.m));
}

/* -------------------------------------------------------------------------- */
/*  Helpers: convert CollideShapeResult / ShapeCastResult                     */
/* -------------------------------------------------------------------------- */
static inline JoltC_CollideShapeResult fromJphCollideShapeResult(const CollideShapeResult& r) {
    JoltC_CollideShapeResult cr;
    cr.contactPointOn1 = fromJphVec3(r.mContactPointOn1);
    cr.contactPointOn2 = fromJphVec3(r.mContactPointOn2);
    cr.penetrationAxis = fromJphVec3(r.mPenetrationAxis);
    cr.penetrationDepth = r.mPenetrationDepth;
    cr.subShapeID1 = r.mSubShapeID1.GetValue();
    cr.subShapeID2 = r.mSubShapeID2.GetValue();
    cr.bodyID2 = r.mBodyID2.GetIndexAndSequenceNumber();
    return cr;
}

static inline JoltC_ShapeCastResult fromJphShapeCastResult(const ShapeCastResult& r) {
    JoltC_ShapeCastResult cr;
    cr.contactPointOn1 = fromJphVec3(r.mContactPointOn1);
    cr.contactPointOn2 = fromJphVec3(r.mContactPointOn2);
    cr.penetrationAxis = fromJphVec3(r.mPenetrationAxis);
    cr.penetrationDepth = r.mPenetrationDepth;
    cr.subShapeID1 = r.mSubShapeID1.GetValue();
    cr.subShapeID2 = r.mSubShapeID2.GetValue();
    cr.bodyID2 = r.mBodyID2.GetIndexAndSequenceNumber();
    cr.fraction = r.mFraction;
    cr.isBackFaceHit = r.mIsBackFaceHit ? JOLTC_TRUE : JOLTC_FALSE;
    return cr;
}

/* -------------------------------------------------------------------------- */
/*  ContactManifold helpers                                                   */
/* -------------------------------------------------------------------------- */
static inline const ContactManifold* asManifold(const JoltC_ContactManifold* m) {
    return reinterpret_cast<const ContactManifold*>(m);
}

extern "C" {

/* -------------------------------------------------------------------------- */
/*  NarrowPhaseQuery access                                                   */
/* -------------------------------------------------------------------------- */
static thread_local JoltC_NarrowPhaseQuery s_npqLock;
static thread_local JoltC_NarrowPhaseQuery s_npqNoLock;

JOLTC_API const JoltC_NarrowPhaseQuery* JoltC_PhysicsSystem_GetNarrowPhaseQuery(const JoltC_PhysicsSystem* system) {
    if (!system) return nullptr;
    JOLTC_TRY_BEGIN
    s_npqLock.ptr = &system->ptr->GetNarrowPhaseQuery();
    return &s_npqLock;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API const JoltC_NarrowPhaseQuery* JoltC_PhysicsSystem_GetNarrowPhaseQueryNoLock(const JoltC_PhysicsSystem* system) {
    if (!system) return nullptr;
    JOLTC_TRY_BEGIN
    s_npqNoLock.ptr = &system->ptr->GetNarrowPhaseQueryNoLock();
    return &s_npqNoLock;
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  CastRay — single closest hit (no filters)                                 */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_Bool JoltC_NarrowPhaseQuery_CastRay(
    const JoltC_NarrowPhaseQuery* query,
    JoltC_RVec3 origin, JoltC_Vec3 direction,
    JoltC_RayCastResult* outResult)
{
    if (!query || !outResult) return JOLTC_FALSE;
    JOLTC_TRY_BEGIN
    RRayCast ray(toJphRVec3(origin), toJphVec3(direction));
    RayCastResult hit;
    bool found = query->ptr->CastRay(ray, hit);
    if (found) {
        *outResult = fromJphRayCastResult(hit);
    }
    return found ? JOLTC_TRUE : JOLTC_FALSE;
    JOLTC_TRY_END
    return JOLTC_FALSE;
}

/* -------------------------------------------------------------------------- */
/*  CastRay — multiple hits via callback (no filters)                         */
/* -------------------------------------------------------------------------- */
class CastRayCallbackCollector final : public CastRayCollector {
public:
    JoltC_CastRayCollectorFn fn;
    void* userData;
    void AddHit(const RayCastResult& hit) override {
        JoltC_RayCastResult r = fromJphRayCastResult(hit);
        fn(userData, &r);
    }
};

JOLTC_API void JoltC_NarrowPhaseQuery_CastRayAll(
    const JoltC_NarrowPhaseQuery* query,
    JoltC_RVec3 origin, JoltC_Vec3 direction,
    const JoltC_RayCastSettings* rayCastSettings,
    JoltC_CastRayCollectorFn callback,
    void* userData)
{
    if (!query || !callback) return;
    JOLTC_TRY_BEGIN
    RRayCast ray(toJphRVec3(origin), toJphVec3(direction));
    RayCastSettings settings;
    if (rayCastSettings) {
        settings.mBackFaceModeTriangles = toJphBackFaceMode(rayCastSettings->backFaceModeTriangles);
        settings.mBackFaceModeConvex = toJphBackFaceMode(rayCastSettings->backFaceModeConvex);
        settings.mTreatConvexAsSolid = rayCastSettings->treatConvexAsSolid != 0;
    }
    CastRayCallbackCollector collector;
    collector.fn = callback;
    collector.userData = userData;
    query->ptr->CastRay(ray, settings, collector);
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  CollidePoint (no filters)                                                 */
/* -------------------------------------------------------------------------- */
class CollidePointCallbackCollector final : public CollidePointCollector {
public:
    JoltC_CollidePointCollectorFn fn;
    void* userData;
    void AddHit(const CollidePointResult& hit) override {
        fn(userData, hit.mBodyID.GetIndexAndSequenceNumber(), hit.mSubShapeID2.GetValue());
    }
};

JOLTC_API void JoltC_NarrowPhaseQuery_CollidePoint(
    const JoltC_NarrowPhaseQuery* query,
    JoltC_RVec3 point,
    JoltC_CollidePointCollectorFn callback,
    void* userData)
{
    if (!query || !callback) return;
    JOLTC_TRY_BEGIN
    CollidePointCallbackCollector collector;
    collector.fn = callback;
    collector.userData = userData;
    query->ptr->CollidePoint(toJphRVec3(point), collector);
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  CastRay2 — single closest hit with filters                                */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_Bool JoltC_NarrowPhaseQuery_CastRay2(
    const JoltC_NarrowPhaseQuery* query,
    JoltC_RVec3 origin, JoltC_Vec3 direction,
    JoltC_RayCastResult* outResult,
    const JoltC_BroadPhaseLayerFilter* bpFilter,
    const JoltC_ObjectLayerFilter* olFilter,
    const JoltC_BodyFilter* bodyFilter)
{
    if (!query || !outResult) return JOLTC_FALSE;
    JOLTC_TRY_BEGIN
    RRayCast ray(toJphRVec3(origin), toJphVec3(direction));
    RayCastResult hit;
    bool found = query->ptr->CastRay(ray, hit, bpf(bpFilter), olf(olFilter), bf(bodyFilter));
    if (found) {
        *outResult = fromJphRayCastResult(hit);
    }
    return found ? JOLTC_TRUE : JOLTC_FALSE;
    JOLTC_TRY_END
    return JOLTC_FALSE;
}

/* -------------------------------------------------------------------------- */
/*  CastRay3 — multiple hits with all filters                                 */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_NarrowPhaseQuery_CastRay3(
    const JoltC_NarrowPhaseQuery* query,
    JoltC_RVec3 origin, JoltC_Vec3 direction,
    const JoltC_RayCastSettings* rayCastSettings,
    JoltC_CastRayCollectorFn callback, void* userData,
    const JoltC_BroadPhaseLayerFilter* bpFilter,
    const JoltC_ObjectLayerFilter* olFilter,
    const JoltC_BodyFilter* bodyFilter,
    const JoltC_ShapeFilter* shapeFilter)
{
    if (!query || !callback) return;
    JOLTC_TRY_BEGIN
    RRayCast ray(toJphRVec3(origin), toJphVec3(direction));
    RayCastSettings settings;
    if (rayCastSettings) {
        settings.mBackFaceModeTriangles = toJphBackFaceMode(rayCastSettings->backFaceModeTriangles);
        settings.mBackFaceModeConvex = toJphBackFaceMode(rayCastSettings->backFaceModeConvex);
        settings.mTreatConvexAsSolid = rayCastSettings->treatConvexAsSolid != 0;
    }
    CastRayCallbackCollector collector;
    collector.fn = callback;
    collector.userData = userData;
    query->ptr->CastRay(ray, settings, collector, bpf(bpFilter), olf(olFilter), bf(bodyFilter), sf(shapeFilter));
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  CollidePoint2 — with filters                                              */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_NarrowPhaseQuery_CollidePoint2(
    const JoltC_NarrowPhaseQuery* query,
    JoltC_RVec3 point,
    JoltC_CollidePointCollectorFn callback, void* userData,
    const JoltC_BroadPhaseLayerFilter* bpFilter,
    const JoltC_ObjectLayerFilter* olFilter,
    const JoltC_BodyFilter* bodyFilter,
    const JoltC_ShapeFilter* shapeFilter)
{
    if (!query || !callback) return;
    JOLTC_TRY_BEGIN
    CollidePointCallbackCollector collector;
    collector.fn = callback;
    collector.userData = userData;
    query->ptr->CollidePoint(toJphRVec3(point), collector, bpf(bpFilter), olf(olFilter), bf(bodyFilter), sf(shapeFilter));
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  CollideShape                                                              */
/* -------------------------------------------------------------------------- */
class CollideShapeCallbackCollector final : public CollideShapeCollector {
public:
    JoltC_CollideShapeResultFn fn;
    void* userData;
    void AddHit(const CollideShapeResult& hit) override {
        JoltC_CollideShapeResult r = fromJphCollideShapeResult(hit);
        fn(userData, &r);
    }
};

JOLTC_API void JoltC_NarrowPhaseQuery_CollideShape(
    const JoltC_NarrowPhaseQuery* query,
    const JoltC_Shape* shape,
    JoltC_Vec3 scale,
    JoltC_Mat44 centerOfMassTransform,
    JoltC_RVec3 baseOffset,
    JoltC_CollideShapeResultFn callback, void* userData,
    const JoltC_BroadPhaseLayerFilter* bpFilter,
    const JoltC_ObjectLayerFilter* olFilter,
    const JoltC_BodyFilter* bodyFilter,
    const JoltC_ShapeFilter* shapeFilter)
{
    if (!query || !shape || !callback) return;
    JOLTC_TRY_BEGIN
    const auto* jphShape = reinterpret_cast<const Shape*>(shape);
    RMat44 com = toJphMat44FromBlittable(centerOfMassTransform);
    CollideShapeSettings settings;
    CollideShapeCallbackCollector collector;
    collector.fn = callback;
    collector.userData = userData;
    query->ptr->CollideShape(jphShape, toJphVec3(scale), com, settings, toJphRVec3(baseOffset), collector,
                             bpf(bpFilter), olf(olFilter), bf(bodyFilter), sf(shapeFilter));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_NarrowPhaseQuery_CollideShape2(
    const JoltC_NarrowPhaseQuery* query,
    const JoltC_Shape* shape,
    JoltC_Vec3 scale,
    JoltC_Mat44 centerOfMassTransform,
    const JoltC_CollideShapeSettings* collideSettings,
    JoltC_RVec3 baseOffset,
    JoltC_CollideShapeResultFn callback, void* userData,
    const JoltC_BroadPhaseLayerFilter* bpFilter,
    const JoltC_ObjectLayerFilter* olFilter,
    const JoltC_BodyFilter* bodyFilter,
    const JoltC_ShapeFilter* shapeFilter)
{
    if (!query || !shape || !callback) return;
    JOLTC_TRY_BEGIN
    const auto* jphShape = reinterpret_cast<const Shape*>(shape);
    RMat44 com = toJphMat44FromBlittable(centerOfMassTransform);

    /* The settings struct existed for a long time with an Init helper and nothing that accepted
     * it. This is its consumer. Null keeps the defaults, same as the older entry point. */
    CollideShapeSettings settings;
    if (collideSettings)
    {
        settings.mBackFaceMode = toJphBackFaceMode(collideSettings->backFaceMode);
        settings.mMaxSeparationDistance = collideSettings->maxSeparationDistance;
        settings.mCollisionTolerance = collideSettings->collisionTolerance;
        settings.mPenetrationTolerance = collideSettings->penetrationTolerance;
        settings.mInternalEdgeRemovalVertexToleranceSq = collideSettings->internalEdgeRemovalVertexToleranceSq;
    }

    CollideShapeCallbackCollector collector;
    collector.fn = callback;
    collector.userData = userData;
    query->ptr->CollideShape(jphShape, toJphVec3(scale), com, settings, toJphRVec3(baseOffset), collector,
                             bpf(bpFilter), olf(olFilter), bf(bodyFilter), sf(shapeFilter));
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  CastShape                                                                 */
/* -------------------------------------------------------------------------- */
class CastShapeCallbackCollector final : public CastShapeCollector {
public:
    JoltC_CastShapeResultFn fn;
    void* userData;
    void AddHit(const ShapeCastResult& hit) override {
        JoltC_ShapeCastResult r = fromJphShapeCastResult(hit);
        fn(userData, &r);
    }
};

JOLTC_API void JoltC_NarrowPhaseQuery_CastShape(
    const JoltC_NarrowPhaseQuery* query,
    const JoltC_Shape* shape,
    JoltC_Vec3 scale,
    JoltC_Mat44 centerOfMassTransform,
    JoltC_Vec3 direction,
    JoltC_RVec3 baseOffset,
    JoltC_CastShapeResultFn callback, void* userData,
    const JoltC_BroadPhaseLayerFilter* bpFilter,
    const JoltC_ObjectLayerFilter* olFilter,
    const JoltC_BodyFilter* bodyFilter,
    const JoltC_ShapeFilter* shapeFilter)
{
    if (!query || !shape || !callback) return;
    JOLTC_TRY_BEGIN
    const auto* jphShape = reinterpret_cast<const Shape*>(shape);
    RMat44 com = toJphMat44FromBlittable(centerOfMassTransform);
    RShapeCast shapeCast(jphShape, toJphVec3(scale), com, toJphVec3(direction));
    ShapeCastSettings settings;
    CastShapeCallbackCollector collector;
    collector.fn = callback;
    collector.userData = userData;
    query->ptr->CastShape(shapeCast, settings, toJphRVec3(baseOffset), collector,
                          bpf(bpFilter), olf(olFilter), bf(bodyFilter), sf(shapeFilter));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_NarrowPhaseQuery_CastShape2(
    const JoltC_NarrowPhaseQuery* query,
    const JoltC_Shape* shape,
    JoltC_Vec3 scale,
    JoltC_Mat44 centerOfMassTransform,
    JoltC_Vec3 direction,
    const JoltC_ShapeCastSettings* castSettings,
    JoltC_RVec3 baseOffset,
    JoltC_CastShapeResultFn callback, void* userData,
    const JoltC_BroadPhaseLayerFilter* bpFilter,
    const JoltC_ObjectLayerFilter* olFilter,
    const JoltC_BodyFilter* bodyFilter,
    const JoltC_ShapeFilter* shapeFilter)
{
    if (!query || !shape || !callback) return;
    JOLTC_TRY_BEGIN
    const auto* jphShape = reinterpret_cast<const Shape*>(shape);
    RMat44 com = toJphMat44FromBlittable(centerOfMassTransform);
    RShapeCast shapeCast(jphShape, toJphVec3(scale), com, toJphVec3(direction));

    ShapeCastSettings settings;
    if (castSettings)
    {
        settings.mBackFaceModeTriangles = toJphBackFaceMode(castSettings->backFaceModeTriangles);
        settings.mBackFaceModeConvex = toJphBackFaceMode(castSettings->backFaceModeConvex);
        settings.mUseShrunkenShapeAndConvexRadius = castSettings->useShrunkenShapeAndConvexRadius != 0;
        settings.mReturnDeepestPoint = castSettings->returnDeepestPoint != 0;
        settings.mCollisionTolerance = castSettings->collisionTolerance;
        settings.mPenetrationTolerance = castSettings->penetrationTolerance;
        settings.mExtraConvexRadius = castSettings->extraConvexRadius;
    }

    CastShapeCallbackCollector collector;
    collector.fn = callback;
    collector.userData = userData;
    query->ptr->CastShape(shapeCast, settings, toJphRVec3(baseOffset), collector,
                          bpf(bpFilter), olf(olFilter), bf(bodyFilter), sf(shapeFilter));
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  Default layer filters                                                     */
/* -------------------------------------------------------------------------- */
/* The system's own layer logic packaged as filters, so a caller querying "what layer X can hit"
 * passes these instead of rebuilding the layer matrix on its side. They reference the system's
 * interfaces, so they are valid only while the system is. */
JOLTC_API JoltC_BroadPhaseLayerFilter* JoltC_PhysicsSystem_GetDefaultBroadPhaseLayerFilter(const JoltC_PhysicsSystem* system, JoltC_ObjectLayer layer)
{
    if (!system) return nullptr;
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_BroadPhaseLayerFilter;

    /* Direct initialization from the prvalue, which guaranteed copy elision builds in place:
     * the filter type is non copyable, so make_unique's forwarding cannot produce one. */
    w->ptr.reset(new DefaultBroadPhaseLayerFilter(system->ptr->GetDefaultBroadPhaseLayerFilter(layer)));
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_ObjectLayerFilter* JoltC_PhysicsSystem_GetDefaultLayerFilter(const JoltC_PhysicsSystem* system, JoltC_ObjectLayer layer)
{
    if (!system) return nullptr;
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_ObjectLayerFilter;
    w->ptr.reset(new DefaultObjectLayerFilter(system->ptr->GetDefaultLayerFilter(layer)));
    return w;
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  BroadPhaseQuery access                                                    */
/* -------------------------------------------------------------------------- */
JOLTC_API const JoltC_BroadPhaseQuery* JoltC_PhysicsSystem_GetBroadPhaseQuery(const JoltC_PhysicsSystem* system) {
    if (!system) return nullptr;
    JOLTC_TRY_BEGIN
    /* Stored on the system wrapper rather than in a thread_local: with two systems on one thread,
     * the shared slot meant the second Get quietly invalidated the first system's pointer. */
    auto* mutableSystem = const_cast<JoltC_PhysicsSystem*>(system);
    mutableSystem->broadPhaseQuery.ptr = &system->ptr->GetBroadPhaseQuery();
    return &mutableSystem->broadPhaseQuery;
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  BroadPhaseQuery — CastRay                                                 */
/* -------------------------------------------------------------------------- */
class BroadPhaseCastRayCollector final : public RayCastBodyCollector {
public:
    JoltC_BroadPhaseCastResultFn fn;
    void* userData;
    void AddHit(const BroadPhaseCastResult& hit) override {
        JoltC_BroadPhaseCastResult r;
        r.bodyID = hit.mBodyID.GetIndexAndSequenceNumber();
        r.fraction = hit.mFraction;
        fn(userData, &r);
    }
};

JOLTC_API void JoltC_BroadPhaseQuery_CastRay(
    const JoltC_BroadPhaseQuery* query,
    JoltC_Vec3 origin, JoltC_Vec3 direction,
    JoltC_BroadPhaseCastResultFn callback, void* userData,
    const JoltC_BroadPhaseLayerFilter* bpFilter,
    const JoltC_ObjectLayerFilter* olFilter)
{
    if (!query || !callback) return;
    JOLTC_TRY_BEGIN
    RayCast ray(toJphVec3(origin), toJphVec3(direction));
    BroadPhaseCastRayCollector collector;
    collector.fn = callback;
    collector.userData = userData;
    query->ptr->CastRay(ray, collector, bpf(bpFilter), olf(olFilter));
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  BroadPhaseQuery — CollideAABox                                            */
/* -------------------------------------------------------------------------- */
class BroadPhaseCollideBodyCollector final : public CollideShapeBodyCollector {
public:
    JoltC_CollideShapeBodyResultFn fn;
    void* userData;
    void AddHit(const BodyID& hit) override {
        fn(userData, hit.GetIndexAndSequenceNumber());
    }
};

JOLTC_API void JoltC_BroadPhaseQuery_CollideAABox(
    const JoltC_BroadPhaseQuery* query,
    JoltC_AABox box,
    JoltC_CollideShapeBodyResultFn callback, void* userData,
    const JoltC_BroadPhaseLayerFilter* bpFilter,
    const JoltC_ObjectLayerFilter* olFilter)
{
    if (!query || !callback) return;
    JOLTC_TRY_BEGIN
    AABox aabb(toJphVec3(box.min), toJphVec3(box.max));
    BroadPhaseCollideBodyCollector collector;
    collector.fn = callback;
    collector.userData = userData;
    query->ptr->CollideAABox(aabb, collector, bpf(bpFilter), olf(olFilter));
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  BroadPhaseQuery — CollideSphere                                           */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_BroadPhaseQuery_CollideSphere(
    const JoltC_BroadPhaseQuery* query,
    JoltC_Vec3 center, float radius,
    JoltC_CollideShapeBodyResultFn callback, void* userData,
    const JoltC_BroadPhaseLayerFilter* bpFilter,
    const JoltC_ObjectLayerFilter* olFilter)
{
    if (!query || !callback) return;
    JOLTC_TRY_BEGIN
    BroadPhaseCollideBodyCollector collector;
    collector.fn = callback;
    collector.userData = userData;
    query->ptr->CollideSphere(toJphVec3(center), radius, collector, bpf(bpFilter), olf(olFilter));
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  BroadPhaseQuery — CollidePoint                                            */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_BroadPhaseQuery_CollidePoint(
    const JoltC_BroadPhaseQuery* query,
    JoltC_Vec3 point,
    JoltC_CollideShapeBodyResultFn callback, void* userData,
    const JoltC_BroadPhaseLayerFilter* bpFilter,
    const JoltC_ObjectLayerFilter* olFilter)
{
    if (!query || !callback) return;
    JOLTC_TRY_BEGIN
    BroadPhaseCollideBodyCollector collector;
    collector.fn = callback;
    collector.userData = userData;
    query->ptr->CollidePoint(toJphVec3(point), collector, bpf(bpFilter), olf(olFilter));
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  ContactManifold reader functions                                          */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_Vec3 JoltC_ContactManifold_GetWorldSpaceNormal(const JoltC_ContactManifold* manifold) {
    if (!manifold) return JoltC_Vec3{0,0,0};
    return fromJphVec3(asManifold(manifold)->mWorldSpaceNormal);
}

JOLTC_API float JoltC_ContactManifold_GetPenetrationDepth(const JoltC_ContactManifold* manifold) {
    if (!manifold) return 0.0f;
    return asManifold(manifold)->mPenetrationDepth;
}

JOLTC_API JoltC_SubShapeID JoltC_ContactManifold_GetSubShapeID1(const JoltC_ContactManifold* manifold) {
    if (!manifold) return 0;
    return asManifold(manifold)->mSubShapeID1.GetValue();
}

JOLTC_API JoltC_SubShapeID JoltC_ContactManifold_GetSubShapeID2(const JoltC_ContactManifold* manifold) {
    if (!manifold) return 0;
    return asManifold(manifold)->mSubShapeID2.GetValue();
}

JOLTC_API uint32_t JoltC_ContactManifold_GetPointCount(const JoltC_ContactManifold* manifold) {
    if (!manifold) return 0;
    return (uint32_t)asManifold(manifold)->mRelativeContactPointsOn1.size();
}

JOLTC_API JoltC_RVec3 JoltC_ContactManifold_GetWorldSpaceContactPointOn1(const JoltC_ContactManifold* manifold, uint32_t index) {
    if (!manifold) { JoltC_RVec3 r = {0,0,0}; return r; }
    return fromJphRVec3(asManifold(manifold)->GetWorldSpaceContactPointOn1(index));
}

JOLTC_API JoltC_RVec3 JoltC_ContactManifold_GetWorldSpaceContactPointOn2(const JoltC_ContactManifold* manifold, uint32_t index) {
    if (!manifold) { JoltC_RVec3 r = {0,0,0}; return r; }
    return fromJphRVec3(asManifold(manifold)->GetWorldSpaceContactPointOn2(index));
}

} /* extern "C" */
