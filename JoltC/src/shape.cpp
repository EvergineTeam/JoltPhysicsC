/* JoltC - Shape implementations
 * SPDX-License-Identifier: MIT
 */

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCylinderShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/DecoratedShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexShape.h>
#include <Jolt/Physics/Collision/Shape/TriangleShape.h>
#include <Jolt/Physics/Collision/Shape/PlaneShape.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Geometry/Plane.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/Shape/SubShapeID.h>
#include <Jolt/Physics/Body/MassProperties.h>
#include <Jolt/Core/Reference.h>

#include <JoltC/shape.h>
#include "wrappers.h"
#include "internal.h"
#include "errors_internal.h"

using namespace JPH;

/* -------------------------------------------------------------------------- */
/*  Helpers — Shape* ↔ JoltC_Shape* reinterpret casts                         */
/* -------------------------------------------------------------------------- */
static inline const Shape* asShape(const JoltC_Shape* h) {
    return reinterpret_cast<const Shape*>(h);
}

static inline const JoltC_Shape* fromShape(const Shape* s) {
    return reinterpret_cast<const JoltC_Shape*>(s);
}

static inline ShapeSettings* asShapeSettings(JoltC_ShapeSettings* h) {
    return reinterpret_cast<ShapeSettings*>(h);
}

static inline const ShapeSettings* asShapeSettings(const JoltC_ShapeSettings* h) {
    return reinterpret_cast<const ShapeSettings*>(h);
}

static inline JoltC_ShapeSettings* fromShapeSettings(ShapeSettings* s) {
    return reinterpret_cast<JoltC_ShapeSettings*>(s);
}

extern "C" {

/* -------------------------------------------------------------------------- */
/*  Ref-counting                                                              */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_Shape_AddRef(const JoltC_Shape* shape)
{
    if (!shape) return;
    JOLTC_TRY_BEGIN
    asShape(shape)->AddRef();
    JOLTC_TRY_END
}

JOLTC_API void JoltC_Shape_Release(const JoltC_Shape* shape)
{
    if (!shape) return;
    JOLTC_TRY_BEGIN
    asShape(shape)->Release();
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  Queries                                                                   */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_ShapeType JoltC_Shape_GetType(const JoltC_Shape* shape)
{
    if (!shape) return JOLTC_SHAPE_TYPE_EMPTY;
    JOLTC_TRY_BEGIN
    return static_cast<JoltC_ShapeType>(asShape(shape)->GetType());
    JOLTC_TRY_END
    return JOLTC_SHAPE_TYPE_EMPTY;
}

JOLTC_API JoltC_ShapeSubType JoltC_Shape_GetSubType(const JoltC_Shape* shape)
{
    if (!shape) return JOLTC_SHAPE_SUB_TYPE_SPHERE;
    JOLTC_TRY_BEGIN
    return static_cast<JoltC_ShapeSubType>(asShape(shape)->GetSubType());
    JOLTC_TRY_END
    return JOLTC_SHAPE_SUB_TYPE_SPHERE;
}

JOLTC_API JoltC_Vec3 JoltC_Shape_GetCenterOfMass(const JoltC_Shape* shape)
{
    if (!shape) return JoltC_Vec3{0, 0, 0};
    JOLTC_TRY_BEGIN
    return fromJphVec3(asShape(shape)->GetCenterOfMass());
    JOLTC_TRY_END
    return JoltC_Vec3{0, 0, 0};
}

JOLTC_API JoltC_AABox JoltC_Shape_GetLocalBounds(const JoltC_Shape* shape)
{
    JoltC_AABox zero = {{0,0,0},{0,0,0}};
    if (!shape) return zero;
    JOLTC_TRY_BEGIN
    return fromJphAABox(asShape(shape)->GetLocalBounds());
    JOLTC_TRY_END
    return zero;
}

JOLTC_API float JoltC_Shape_GetInnerRadius(const JoltC_Shape* shape)
{
    if (!shape) return 0.0f;
    JOLTC_TRY_BEGIN
    return asShape(shape)->GetInnerRadius();
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API float JoltC_Shape_GetVolume(const JoltC_Shape* shape)
{
    if (!shape) return 0.0f;
    JOLTC_TRY_BEGIN
    return asShape(shape)->GetVolume();
    JOLTC_TRY_END
    return 0.0f;
}

/* -------------------------------------------------------------------------- */
/*  Box shape                                                                 */
/* -------------------------------------------------------------------------- */
JOLTC_API const JoltC_Shape* JoltC_BoxShape_Create(JoltC_Vec3 halfExtent, float convexRadius)
{
    JOLTC_TRY_BEGIN
    BoxShape* s = new BoxShape(toJphVec3(halfExtent), convexRadius);
    s->AddRef();
    return fromShape(s);
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  Sphere shape                                                              */
/* -------------------------------------------------------------------------- */
JOLTC_API const JoltC_Shape* JoltC_SphereShape_Create(float radius)
{
    JOLTC_TRY_BEGIN
    SphereShape* s = new SphereShape(radius);
    s->AddRef();
    return fromShape(s);
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  Capsule shape                                                             */
/* -------------------------------------------------------------------------- */
JOLTC_API const JoltC_Shape* JoltC_CapsuleShape_Create(float halfHeightOfCylinder, float radius)
{
    JOLTC_TRY_BEGIN
    CapsuleShape* s = new CapsuleShape(halfHeightOfCylinder, radius);
    s->AddRef();
    return fromShape(s);
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  Cylinder shape                                                            */
/* -------------------------------------------------------------------------- */
JOLTC_API const JoltC_Shape* JoltC_CylinderShape_Create(float halfHeight, float radius, float convexRadius)
{
    JOLTC_TRY_BEGIN
    CylinderShape* s = new CylinderShape(halfHeight, radius, convexRadius);
    s->AddRef();
    return fromShape(s);
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  Tapered capsule shape (Settings-based creation)                           */
/* -------------------------------------------------------------------------- */
JOLTC_API const JoltC_Shape* JoltC_TaperedCapsuleShape_Create(float halfHeight, float topRadius, float bottomRadius)
{
    JOLTC_TRY_BEGIN
    TaperedCapsuleShapeSettings settings(halfHeight, topRadius, bottomRadius);
    auto result = settings.Create();
    if (result.HasError()) {
        joltc_set_last_error(result.GetError().c_str());
        return nullptr;
    }
    const Shape* s = result.Get().GetPtr();
    s->AddRef();
    return fromShape(s);
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  ConvexHull shape                                                          */
/* -------------------------------------------------------------------------- */
JOLTC_API const JoltC_Shape* JoltC_ConvexHullShape_Create(const JoltC_Vec3* points, int numPoints, float maxConvexRadius)
{
    if (!points || numPoints <= 0) return nullptr;
    JOLTC_TRY_BEGIN
    Array<Vec3> pts;
    pts.reserve(numPoints);
    for (int i = 0; i < numPoints; i++)
        pts.push_back(toJphVec3(points[i]));
    ConvexHullShapeSettings settings(pts, maxConvexRadius);
    auto result = settings.Create();
    if (result.HasError()) {
        joltc_set_last_error(result.GetError().c_str());
        return nullptr;
    }
    const Shape* s = result.Get().GetPtr();
    s->AddRef();
    return fromShape(s);
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  Mesh shape                                                                */
/* -------------------------------------------------------------------------- */
JOLTC_API const JoltC_Shape* JoltC_MeshShape_Create(
    const JoltC_Vec3* vertices, int numVertices,
    const JoltC_IndexedTriangle* triangles, int numTriangles)
{
    if (!vertices || !triangles || numVertices <= 0 || numTriangles <= 0) return nullptr;
    JOLTC_TRY_BEGIN
    VertexList verts;
    verts.reserve(numVertices);
    for (int i = 0; i < numVertices; i++) {
        Float3 f;
        f.x = vertices[i].x; f.y = vertices[i].y; f.z = vertices[i].z;
        verts.push_back(f);
    }
    IndexedTriangleList tris;
    tris.reserve(numTriangles);
    for (int i = 0; i < numTriangles; i++) {
        IndexedTriangle t(triangles[i].i1, triangles[i].i2, triangles[i].i3, triangles[i].materialIndex);
        tris.push_back(t);
    }
    MeshShapeSettings settings(std::move(verts), std::move(tris));
    auto result = settings.Create();
    if (result.HasError()) {
        joltc_set_last_error(result.GetError().c_str());
        return nullptr;
    }
    const Shape* s = result.Get().GetPtr();
    s->AddRef();
    return fromShape(s);
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  HeightField shape                                                         */
/* -------------------------------------------------------------------------- */
JOLTC_API const JoltC_Shape* JoltC_HeightFieldShape_Create(
    const float* samples, JoltC_Vec3 offset, JoltC_Vec3 scale, uint32_t sampleCount)
{
    if (!samples || sampleCount < 2) return nullptr;
    JOLTC_TRY_BEGIN
    HeightFieldShapeSettings settings(samples, toJphVec3(offset), toJphVec3(scale), sampleCount);
    auto result = settings.Create();
    if (result.HasError()) {
        joltc_set_last_error(result.GetError().c_str());
        return nullptr;
    }
    const Shape* s = result.Get().GetPtr();
    s->AddRef();
    return fromShape(s);
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  StaticCompoundShape                                                       */
/* -------------------------------------------------------------------------- */
JOLTC_API const JoltC_Shape* JoltC_StaticCompoundShape_Create(
    const JoltC_CompoundShapeSubShape* subShapes, int numSubShapes)
{
    if (!subShapes || numSubShapes <= 0) return nullptr;
    JOLTC_TRY_BEGIN
    StaticCompoundShapeSettings settings;
    for (int i = 0; i < numSubShapes; i++) {
        settings.AddShape(
            toJphVec3(subShapes[i].position),
            toJphQuat(subShapes[i].rotation),
            asShape(subShapes[i].shape),
            subShapes[i].userData);
    }
    auto result = settings.Create();
    if (result.HasError()) {
        joltc_set_last_error(result.GetError().c_str());
        return nullptr;
    }
    const Shape* s = result.Get().GetPtr();
    s->AddRef();
    return fromShape(s);
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  MutableCompoundShape                                                      */
/* -------------------------------------------------------------------------- */
JOLTC_API const JoltC_Shape* JoltC_MutableCompoundShape_Create(
    const JoltC_CompoundShapeSubShape* subShapes, int numSubShapes)
{
    if (!subShapes || numSubShapes <= 0) return nullptr;
    JOLTC_TRY_BEGIN
    MutableCompoundShapeSettings settings;
    for (int i = 0; i < numSubShapes; i++) {
        settings.AddShape(
            toJphVec3(subShapes[i].position),
            toJphQuat(subShapes[i].rotation),
            asShape(subShapes[i].shape),
            subShapes[i].userData);
    }
    auto result = settings.Create();
    if (result.HasError()) {
        joltc_set_last_error(result.GetError().c_str());
        return nullptr;
    }
    const Shape* s = result.Get().GetPtr();
    s->AddRef();
    return fromShape(s);
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  RotatedTranslatedShape                                                    */
/* -------------------------------------------------------------------------- */
JOLTC_API const JoltC_Shape* JoltC_RotatedTranslatedShape_Create(
    JoltC_Vec3 position, JoltC_Quat rotation, const JoltC_Shape* innerShape)
{
    if (!innerShape) return nullptr;
    JOLTC_TRY_BEGIN
    auto* s = new RotatedTranslatedShape(toJphVec3(position), toJphQuat(rotation), asShape(innerShape));
    s->AddRef();
    return fromShape(s);
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  ScaledShape                                                               */
/* -------------------------------------------------------------------------- */
JOLTC_API const JoltC_Shape* JoltC_ScaledShape_Create(
    const JoltC_Shape* innerShape, JoltC_Vec3 scale)
{
    if (!innerShape) return nullptr;
    JOLTC_TRY_BEGIN
    auto* s = new ScaledShape(asShape(innerShape), toJphVec3(scale));
    s->AddRef();
    return fromShape(s);
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  OffsetCenterOfMassShape                                                   */
/* -------------------------------------------------------------------------- */
JOLTC_API const JoltC_Shape* JoltC_OffsetCenterOfMassShape_Create(
    const JoltC_Shape* innerShape, JoltC_Vec3 offset)
{
    if (!innerShape) return nullptr;
    JOLTC_TRY_BEGIN
    auto* s = new OffsetCenterOfMassShape(asShape(innerShape), toJphVec3(offset));
    s->AddRef();
    return fromShape(s);
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  CompoundShape accessors                                                   */
/* -------------------------------------------------------------------------- */
JOLTC_API int JoltC_CompoundShape_GetNumSubShapes(const JoltC_Shape* shape)
{
    if (!shape) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<int>(static_cast<const CompoundShape*>(asShape(shape))->GetNumSubShapes());
    JOLTC_TRY_END
    return 0;
}

JOLTC_API const JoltC_Shape* JoltC_CompoundShape_GetSubShape(const JoltC_Shape* shape, int index)
{
    if (!shape) return nullptr;
    JOLTC_TRY_BEGIN
    auto& sub = static_cast<const CompoundShape*>(asShape(shape))->GetSubShape(index);
    return fromShape(sub.mShape);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API uint32_t JoltC_CompoundShape_GetSubShapeUserData(const JoltC_Shape* shape, int index)
{
    if (!shape) return 0;
    JOLTC_TRY_BEGIN
    auto& sub = static_cast<const CompoundShape*>(asShape(shape))->GetSubShape(index);
    return sub.mUserData;
    JOLTC_TRY_END
    return 0;
}

JOLTC_API JoltC_Vec3 JoltC_CompoundShape_GetSubShapePosition(const JoltC_Shape* shape, int index)
{
    JoltC_Vec3 z = {0,0,0}; if (!shape) return z;
    JOLTC_TRY_BEGIN
    auto& sub = static_cast<const CompoundShape*>(asShape(shape))->GetSubShape(index);
    return fromJphVec3(sub.GetPositionCOM());
    JOLTC_TRY_END
    return z;
}

JOLTC_API JoltC_Quat JoltC_CompoundShape_GetSubShapeRotation(const JoltC_Shape* shape, int index)
{
    JoltC_Quat z = {0,0,0,1}; if (!shape) return z;
    JOLTC_TRY_BEGIN
    auto& sub = static_cast<const CompoundShape*>(asShape(shape))->GetSubShape(index);
    return fromJphQuat(sub.GetRotation());
    JOLTC_TRY_END
    return z;
}

/* -------------------------------------------------------------------------- */
/*  DecoratedShape accessor                                                   */
/* -------------------------------------------------------------------------- */
JOLTC_API const JoltC_Shape* JoltC_DecoratedShape_GetInnerShape(const JoltC_Shape* shape)
{
    if (!shape) return nullptr;
    JOLTC_TRY_BEGIN
    return fromShape(static_cast<const DecoratedShape*>(asShape(shape))->GetInnerShape());
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  Shape user data                                                           */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_Shape_SetUserData(const JoltC_Shape* shape, uint64_t userData)
{
    if (!shape) return;
    JOLTC_TRY_BEGIN
    const_cast<Shape*>(asShape(shape))->SetUserData(userData);
    JOLTC_TRY_END
}

JOLTC_API uint64_t JoltC_Shape_GetUserData(const JoltC_Shape* shape)
{
    if (!shape) return 0;
    JOLTC_TRY_BEGIN
    return asShape(shape)->GetUserData();
    JOLTC_TRY_END
    return 0;
}

/* -------------------------------------------------------------------------- */
/*  Extended Shape queries                                                    */
/* -------------------------------------------------------------------------- */
JOLTC_API int JoltC_Shape_MustBeStatic(const JoltC_Shape* shape)
{
    if (!shape) return 0;
    return asShape(shape)->MustBeStatic() ? 1 : 0;
}

JOLTC_API uint32_t JoltC_Shape_GetSubShapeIDBitsRecursive(const JoltC_Shape* shape)
{
    if (!shape) return 0;
    return asShape(shape)->GetSubShapeIDBitsRecursive();
}

JOLTC_API void JoltC_Shape_GetMassProperties(const JoltC_Shape* shape, float* outMass, JoltC_Mat44* outInertia)
{
    if (!shape || !outMass || !outInertia) return;
    JOLTC_TRY_BEGIN
    MassProperties mp = asShape(shape)->GetMassProperties();
    *outMass = mp.mMass;
    *outInertia = fromJphMat44(mp.mInertia);
    JOLTC_TRY_END
}

JOLTC_API int JoltC_Shape_CastRay(const JoltC_Shape* shape, JoltC_Vec3 origin, JoltC_Vec3 direction, float* outFraction)
{
    if (!shape || !outFraction) return 0;
    JOLTC_TRY_BEGIN
    RayCast ray{toJphVec3(origin), toJphVec3(direction)};
    SubShapeIDCreator creator;
    RayCastResult hit;
    if (asShape(shape)->CastRay(ray, creator, hit)) {
        *outFraction = hit.mFraction;
        return 1;
    }
    JOLTC_TRY_END
    return 0;
}

JOLTC_API int JoltC_Shape_CollidePoint(const JoltC_Shape* shape, JoltC_Vec3 point)
{
    if (!shape) return 0;
    JOLTC_TRY_BEGIN
    SubShapeIDCreator creator;
    // If it generates any hit, it collides
    struct Collector : public CollidePointCollector {
        bool mHasHit = false;
        void AddHit(const CollidePointResult&) override { mHasHit = true; ForceEarlyOut(); }
    } collector;
    asShape(shape)->CollidePoint(toJphVec3(point), creator, collector);
    return collector.mHasHit ? 1 : 0;
    JOLTC_TRY_END
    return 0;
}

/* -------------------------------------------------------------------------- */
/*  ConvexShape density                                                       */
/* -------------------------------------------------------------------------- */
JOLTC_API float JoltC_ConvexShape_GetDensity(const JoltC_Shape* shape)
{
    if (!shape) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const ConvexShape*>(asShape(shape))->GetDensity();
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API void JoltC_ConvexShape_SetDensity(const JoltC_Shape* shape, float density)
{
    if (!shape) return;
    JOLTC_TRY_BEGIN
    const_cast<ConvexShape*>(static_cast<const ConvexShape*>(asShape(shape)))->SetDensity(density);
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  Box shape getters                                                         */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_Vec3 JoltC_BoxShape_GetHalfExtent(const JoltC_Shape* shape)
{
    JoltC_Vec3 z = {0,0,0}; if (!shape) return z;
    JOLTC_TRY_BEGIN
    return fromJphVec3(static_cast<const BoxShape*>(asShape(shape))->GetHalfExtent());
    JOLTC_TRY_END
    return z;
}

JOLTC_API float JoltC_BoxShape_GetConvexRadius(const JoltC_Shape* shape)
{
    if (!shape) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const BoxShape*>(asShape(shape))->GetConvexRadius();
    JOLTC_TRY_END
    return 0.0f;
}

/* -------------------------------------------------------------------------- */
/*  Sphere shape getters                                                      */
/* -------------------------------------------------------------------------- */
JOLTC_API float JoltC_SphereShape_GetRadius(const JoltC_Shape* shape)
{
    if (!shape) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const SphereShape*>(asShape(shape))->GetRadius();
    JOLTC_TRY_END
    return 0.0f;
}

/* -------------------------------------------------------------------------- */
/*  Capsule shape getters                                                     */
/* -------------------------------------------------------------------------- */
JOLTC_API float JoltC_CapsuleShape_GetRadius(const JoltC_Shape* shape)
{
    if (!shape) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const CapsuleShape*>(asShape(shape))->GetRadius();
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API float JoltC_CapsuleShape_GetHalfHeightOfCylinder(const JoltC_Shape* shape)
{
    if (!shape) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const CapsuleShape*>(asShape(shape))->GetHalfHeightOfCylinder();
    JOLTC_TRY_END
    return 0.0f;
}

/* -------------------------------------------------------------------------- */
/*  Cylinder shape getters                                                    */
/* -------------------------------------------------------------------------- */
JOLTC_API float JoltC_CylinderShape_GetRadius(const JoltC_Shape* shape)
{
    if (!shape) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const CylinderShape*>(asShape(shape))->GetRadius();
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API float JoltC_CylinderShape_GetHalfHeight(const JoltC_Shape* shape)
{
    if (!shape) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const CylinderShape*>(asShape(shape))->GetHalfHeight();
    JOLTC_TRY_END
    return 0.0f;
}

/* -------------------------------------------------------------------------- */
/*  TaperedCapsule shape getters                                              */
/* -------------------------------------------------------------------------- */
JOLTC_API float JoltC_TaperedCapsuleShape_GetTopRadius(const JoltC_Shape* shape)
{
    if (!shape) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const TaperedCapsuleShape*>(asShape(shape))->GetTopRadius();
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API float JoltC_TaperedCapsuleShape_GetBottomRadius(const JoltC_Shape* shape)
{
    if (!shape) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const TaperedCapsuleShape*>(asShape(shape))->GetBottomRadius();
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API float JoltC_TaperedCapsuleShape_GetHalfHeight(const JoltC_Shape* shape)
{
    if (!shape) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const TaperedCapsuleShape*>(asShape(shape))->GetHalfHeight();
    JOLTC_TRY_END
    return 0.0f;
}

/* -------------------------------------------------------------------------- */
/*  ConvexHull shape getters                                                  */
/* -------------------------------------------------------------------------- */
JOLTC_API uint32_t JoltC_ConvexHullShape_GetNumPoints(const JoltC_Shape* shape)
{
    if (!shape) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const ConvexHullShape*>(asShape(shape))->GetNumPoints();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API JoltC_Vec3 JoltC_ConvexHullShape_GetPoint(const JoltC_Shape* shape, uint32_t index)
{
    JoltC_Vec3 z = {0,0,0}; if (!shape) return z;
    JOLTC_TRY_BEGIN
    return fromJphVec3(static_cast<const ConvexHullShape*>(asShape(shape))->GetPoint(index));
    JOLTC_TRY_END
    return z;
}

JOLTC_API uint32_t JoltC_ConvexHullShape_GetNumFaces(const JoltC_Shape* shape)
{
    if (!shape) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const ConvexHullShape*>(asShape(shape))->GetNumFaces();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API uint32_t JoltC_ConvexHullShape_GetNumVerticesInFace(const JoltC_Shape* shape, uint32_t faceIndex)
{
    if (!shape) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const ConvexHullShape*>(asShape(shape))->GetNumVerticesInFace(faceIndex);
    JOLTC_TRY_END
    return 0;
}

JOLTC_API uint32_t JoltC_ConvexHullShape_GetFaceVertices(const JoltC_Shape* shape, uint32_t faceIndex, uint32_t maxVertices, uint32_t* vertices)
{
    if (!shape || !vertices) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const ConvexHullShape*>(asShape(shape))->GetFaceVertices(faceIndex, maxVertices, vertices);
    JOLTC_TRY_END
    return 0;
}

/* -------------------------------------------------------------------------- */
/*  RotatedTranslatedShape getters                                            */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_Vec3 JoltC_RotatedTranslatedShape_GetPosition(const JoltC_Shape* shape)
{
    JoltC_Vec3 z = {0,0,0}; if (!shape) return z;
    JOLTC_TRY_BEGIN
    return fromJphVec3(static_cast<const RotatedTranslatedShape*>(asShape(shape))->GetPosition());
    JOLTC_TRY_END
    return z;
}

JOLTC_API JoltC_Quat JoltC_RotatedTranslatedShape_GetRotation(const JoltC_Shape* shape)
{
    JoltC_Quat z = {0,0,0,1}; if (!shape) return z;
    JOLTC_TRY_BEGIN
    return fromJphQuat(static_cast<const RotatedTranslatedShape*>(asShape(shape))->GetRotation());
    JOLTC_TRY_END
    return z;
}

/* -------------------------------------------------------------------------- */
/*  ScaledShape getters                                                       */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_Vec3 JoltC_ScaledShape_GetScale(const JoltC_Shape* shape)
{
    JoltC_Vec3 z = {0,0,0}; if (!shape) return z;
    JOLTC_TRY_BEGIN
    return fromJphVec3(static_cast<const ScaledShape*>(asShape(shape))->GetScale());
    JOLTC_TRY_END
    return z;
}

/* -------------------------------------------------------------------------- */
/*  OffsetCenterOfMassShape getters                                           */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_Vec3 JoltC_OffsetCenterOfMassShape_GetOffset(const JoltC_Shape* shape)
{
    JoltC_Vec3 z = {0,0,0}; if (!shape) return z;
    JOLTC_TRY_BEGIN
    return fromJphVec3(static_cast<const OffsetCenterOfMassShape*>(asShape(shape))->GetOffset());
    JOLTC_TRY_END
    return z;
}

/* -------------------------------------------------------------------------- */
/*  MutableCompoundShape modification                                         */
/* -------------------------------------------------------------------------- */
JOLTC_API uint32_t JoltC_MutableCompoundShape_AddShape(const JoltC_Shape* compound, JoltC_Vec3 position, JoltC_Quat rotation, const JoltC_Shape* child, uint32_t userData)
{
    if (!compound || !child) return UINT32_MAX;
    JOLTC_TRY_BEGIN
    return const_cast<MutableCompoundShape*>(static_cast<const MutableCompoundShape*>(asShape(compound)))->AddShape(toJphVec3(position), toJphQuat(rotation), asShape(child), userData);
    JOLTC_TRY_END
    return UINT32_MAX;
}

JOLTC_API void JoltC_MutableCompoundShape_RemoveShape(const JoltC_Shape* compound, uint32_t index)
{
    if (!compound) return;
    JOLTC_TRY_BEGIN
    const_cast<MutableCompoundShape*>(static_cast<const MutableCompoundShape*>(asShape(compound)))->RemoveShape(index);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_MutableCompoundShape_ModifyShape(const JoltC_Shape* compound, uint32_t index, JoltC_Vec3 position, JoltC_Quat rotation)
{
    if (!compound) return;
    JOLTC_TRY_BEGIN
    const_cast<MutableCompoundShape*>(static_cast<const MutableCompoundShape*>(asShape(compound)))->ModifyShape(index, toJphVec3(position), toJphQuat(rotation));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_MutableCompoundShape_ModifyShapeWithShape(const JoltC_Shape* compound, uint32_t index, JoltC_Vec3 position, JoltC_Quat rotation, const JoltC_Shape* newShape)
{
    if (!compound || !newShape) return;
    JOLTC_TRY_BEGIN
    const_cast<MutableCompoundShape*>(static_cast<const MutableCompoundShape*>(asShape(compound)))->ModifyShape(index, toJphVec3(position), toJphQuat(rotation), asShape(newShape));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_MutableCompoundShape_AdjustCenterOfMass(const JoltC_Shape* compound)
{
    if (!compound) return;
    JOLTC_TRY_BEGIN
    const_cast<MutableCompoundShape*>(static_cast<const MutableCompoundShape*>(asShape(compound)))->AdjustCenterOfMass();
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  Triangle shape                                                            */
/* -------------------------------------------------------------------------- */
JOLTC_API const JoltC_Shape* JoltC_TriangleShape_Create(JoltC_Vec3 v1, JoltC_Vec3 v2, JoltC_Vec3 v3, float convexRadius)
{
    JOLTC_TRY_BEGIN
    auto* s = new TriangleShape(toJphVec3(v1), toJphVec3(v2), toJphVec3(v3), convexRadius);
    s->AddRef();
    return fromShape(s);
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  Plane shape                                                               */
/* -------------------------------------------------------------------------- */
JOLTC_API const JoltC_Shape* JoltC_PlaneShape_Create(JoltC_Vec3 normal, float distance, float halfExtent)
{
    JOLTC_TRY_BEGIN
    PlaneShapeSettings settings(Plane(toJphVec3(normal), distance), nullptr, halfExtent);
    auto result = settings.Create();
    if (result.HasError()) {
        joltc_set_last_error(result.GetError().c_str());
        return nullptr;
    }
    const Shape* s = result.Get().GetPtr();
    s->AddRef();
    return fromShape(s);
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  Empty shape                                                               */
/* -------------------------------------------------------------------------- */
JOLTC_API const JoltC_Shape* JoltC_EmptyShape_Create(JoltC_Vec3 centerOfMass)
{
    JOLTC_TRY_BEGIN
    EmptyShapeSettings settings(toJphVec3(centerOfMass));
    auto result = settings.Create();
    if (result.HasError()) {
        joltc_set_last_error(result.GetError().c_str());
        return nullptr;
    }
    const Shape* s = result.Get().GetPtr();
    s->AddRef();
    return fromShape(s);
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  HeightFieldShapeSettings                                                  */
/* ========================================================================== */
JOLTC_API JoltC_HeightFieldShapeSettings* JoltC_HeightFieldShapeSettings_Create(
    const float* samples, JoltC_Vec3 offset, JoltC_Vec3 scale, uint32_t sampleCount)
{
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_HeightFieldShapeSettings;
    w->ptr = new HeightFieldShapeSettings(samples, toJphVec3(offset), toJphVec3(scale), sampleCount);
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_HeightFieldShapeSettings_Destroy(JoltC_HeightFieldShapeSettings* settings) {
    delete settings;
}

JOLTC_API JoltC_Vec3 JoltC_HeightFieldShapeSettings_GetOffset(const JoltC_HeightFieldShapeSettings* s) {
    if (!s) return JoltC_Vec3{0,0,0};
    return fromJphVec3(Vec3(s->ptr->mOffset));
}
JOLTC_API void JoltC_HeightFieldShapeSettings_SetOffset(JoltC_HeightFieldShapeSettings* s, JoltC_Vec3 value) {
    if (!s) return;
    s->ptr->mOffset = toJphVec3(value);
}

JOLTC_API JoltC_Vec3 JoltC_HeightFieldShapeSettings_GetScale(const JoltC_HeightFieldShapeSettings* s) {
    if (!s) return JoltC_Vec3{0,0,0};
    return fromJphVec3(Vec3(s->ptr->mScale));
}
JOLTC_API void JoltC_HeightFieldShapeSettings_SetScale(JoltC_HeightFieldShapeSettings* s, JoltC_Vec3 value) {
    if (!s) return;
    s->ptr->mScale = toJphVec3(value);
}

JOLTC_API uint32_t JoltC_HeightFieldShapeSettings_GetSampleCount(const JoltC_HeightFieldShapeSettings* s) {
    if (!s) return 0;
    return s->ptr->mSampleCount;
}

JOLTC_API float JoltC_HeightFieldShapeSettings_GetMinHeightValue(const JoltC_HeightFieldShapeSettings* s) {
    if (!s) return 0;
    return s->ptr->mMinHeightValue;
}
JOLTC_API void JoltC_HeightFieldShapeSettings_SetMinHeightValue(JoltC_HeightFieldShapeSettings* s, float value) {
    if (!s) return;
    s->ptr->mMinHeightValue = value;
}

JOLTC_API float JoltC_HeightFieldShapeSettings_GetMaxHeightValue(const JoltC_HeightFieldShapeSettings* s) {
    if (!s) return 0;
    return s->ptr->mMaxHeightValue;
}
JOLTC_API void JoltC_HeightFieldShapeSettings_SetMaxHeightValue(JoltC_HeightFieldShapeSettings* s, float value) {
    if (!s) return;
    s->ptr->mMaxHeightValue = value;
}

JOLTC_API uint32_t JoltC_HeightFieldShapeSettings_GetBlockSize(const JoltC_HeightFieldShapeSettings* s) {
    if (!s) return 0;
    return s->ptr->mBlockSize;
}
JOLTC_API void JoltC_HeightFieldShapeSettings_SetBlockSize(JoltC_HeightFieldShapeSettings* s, uint32_t value) {
    if (!s) return;
    s->ptr->mBlockSize = value;
}

JOLTC_API uint32_t JoltC_HeightFieldShapeSettings_GetBitsPerSample(const JoltC_HeightFieldShapeSettings* s) {
    if (!s) return 0;
    return s->ptr->mBitsPerSample;
}
JOLTC_API void JoltC_HeightFieldShapeSettings_SetBitsPerSample(JoltC_HeightFieldShapeSettings* s, uint32_t value) {
    if (!s) return;
    s->ptr->mBitsPerSample = value;
}

JOLTC_API const JoltC_Shape* JoltC_HeightFieldShapeSettings_CreateShape(JoltC_HeightFieldShapeSettings* s) {
    if (!s) return nullptr;
    JOLTC_TRY_BEGIN
    auto result = s->ptr->Create();
    if (result.HasError()) {
        joltc_set_last_error(result.GetError().c_str());
        return nullptr;
    }
    const Shape* shape = result.Get().GetPtr();
    shape->AddRef();
    return fromShape(shape);
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  HeightFieldShape instance queries                                         */
/* ========================================================================== */
JOLTC_API uint32_t JoltC_HeightFieldShape_GetSampleCount(const JoltC_Shape* shape) {
    if (!shape) return 0;
    auto* hf = static_cast<const HeightFieldShape*>(asShape(shape));
    return hf->GetSampleCount();
}

JOLTC_API uint32_t JoltC_HeightFieldShape_GetBlockSize(const JoltC_Shape* shape) {
    if (!shape) return 0;
    auto* hf = static_cast<const HeightFieldShape*>(asShape(shape));
    return hf->GetBlockSize();
}

JOLTC_API JoltC_Vec3 JoltC_HeightFieldShape_GetPosition(const JoltC_Shape* shape, uint32_t x, uint32_t y) {
    if (!shape) return JoltC_Vec3{0,0,0};
    auto* hf = static_cast<const HeightFieldShape*>(asShape(shape));
    return fromJphVec3(hf->GetPosition(x, y));
}

JOLTC_API JoltC_Bool JoltC_HeightFieldShape_IsNoCollision(const JoltC_Shape* shape, uint32_t x, uint32_t y) {
    if (!shape) return JOLTC_TRUE;
    auto* hf = static_cast<const HeightFieldShape*>(asShape(shape));
    return hf->IsNoCollision(x, y) ? JOLTC_TRUE : JOLTC_FALSE;
}

JOLTC_API float JoltC_HeightFieldShape_GetMinHeightValue(const JoltC_Shape* shape) {
    if (!shape) return 0;
    auto* hf = static_cast<const HeightFieldShape*>(asShape(shape));
    return hf->GetMinHeightValue();
}

JOLTC_API float JoltC_HeightFieldShape_GetMaxHeightValue(const JoltC_Shape* shape) {
    if (!shape) return 0;
    auto* hf = static_cast<const HeightFieldShape*>(asShape(shape));
    return hf->GetMaxHeightValue();
}

/* ========================================================================== */
/*  MeshShapeSettings                                                         */
/* ========================================================================== */
JOLTC_API JoltC_MeshShapeSettings* JoltC_MeshShapeSettings_Create(const JoltC_Triangle* triangles, uint32_t triangleCount) {
    if (!triangles || triangleCount == 0) return nullptr;
    JOLTC_TRY_BEGIN
    TriangleList tris;
    tris.reserve(triangleCount);
    for (uint32_t i = 0; i < triangleCount; i++) {
        const auto& t = triangles[i];
        tris.push_back(Triangle(
            Float3(t.v1.x, t.v1.y, t.v1.z),
            Float3(t.v2.x, t.v2.y, t.v2.z),
            Float3(t.v3.x, t.v3.y, t.v3.z),
            t.materialIndex));
    }
    auto* w = new JoltC_MeshShapeSettings;
    w->ptr = new MeshShapeSettings(tris);
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_MeshShapeSettings* JoltC_MeshShapeSettings_Create2(
    const JoltC_Vec3* vertices, uint32_t vertexCount,
    const JoltC_IndexedTriangle* triangles, uint32_t triangleCount)
{
    if (!vertices || !triangles || vertexCount == 0 || triangleCount == 0) return nullptr;
    JOLTC_TRY_BEGIN
    VertexList verts;
    verts.reserve(vertexCount);
    for (uint32_t i = 0; i < vertexCount; i++)
        verts.push_back(Float3(vertices[i].x, vertices[i].y, vertices[i].z));
    IndexedTriangleList idxTris;
    idxTris.reserve(triangleCount);
    for (uint32_t i = 0; i < triangleCount; i++) {
        const auto& t = triangles[i];
        idxTris.push_back(IndexedTriangle(t.i1, t.i2, t.i3, t.materialIndex, t.userData));
    }
    auto* w = new JoltC_MeshShapeSettings;
    w->ptr = new MeshShapeSettings(verts, idxTris);
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_MeshShapeSettings_Destroy(JoltC_MeshShapeSettings* settings) {
    delete settings;
}

JOLTC_API uint32_t JoltC_MeshShapeSettings_GetMaxTrianglesPerLeaf(const JoltC_MeshShapeSettings* s) {
    if (!s) return 0;
    return s->ptr->mMaxTrianglesPerLeaf;
}
JOLTC_API void JoltC_MeshShapeSettings_SetMaxTrianglesPerLeaf(JoltC_MeshShapeSettings* s, uint32_t value) {
    if (!s) return;
    s->ptr->mMaxTrianglesPerLeaf = value;
}

JOLTC_API void JoltC_MeshShapeSettings_Sanitize(JoltC_MeshShapeSettings* s) {
    if (!s) return;
    JOLTC_TRY_BEGIN
    s->ptr->Sanitize();
    JOLTC_TRY_END
}

JOLTC_API const JoltC_Shape* JoltC_MeshShapeSettings_CreateShape(const JoltC_MeshShapeSettings* s) {
    if (!s) return nullptr;
    JOLTC_TRY_BEGIN
    auto result = s->ptr->Create();
    if (result.HasError()) {
        joltc_set_last_error(result.GetError().c_str());
        return nullptr;
    }
    const Shape* shape = result.Get().GetPtr();
    shape->AddRef();
    return fromShape(shape);
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  ConvexHullShapeSettings                                                   */
/* ========================================================================== */
JOLTC_API JoltC_ConvexHullShapeSettings* JoltC_ConvexHullShapeSettings_Create(
    const JoltC_Vec3* points, uint32_t pointCount, float maxConvexRadius)
{
    if (!points || pointCount == 0) return nullptr;
    JOLTC_TRY_BEGIN
    Array<Vec3> pts;
    pts.reserve(pointCount);
    for (uint32_t i = 0; i < pointCount; i++)
        pts.push_back(toJphVec3(points[i]));
    auto* w = new JoltC_ConvexHullShapeSettings;
    w->ptr = new ConvexHullShapeSettings(pts, maxConvexRadius);
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_ConvexHullShapeSettings_Destroy(JoltC_ConvexHullShapeSettings* settings) {
    delete settings;
}

JOLTC_API const JoltC_Shape* JoltC_ConvexHullShapeSettings_CreateShape(const JoltC_ConvexHullShapeSettings* s) {
    if (!s) return nullptr;
    JOLTC_TRY_BEGIN
    auto result = s->ptr->Create();
    if (result.HasError()) {
        joltc_set_last_error(result.GetError().c_str());
        return nullptr;
    }
    const Shape* shape = result.Get().GetPtr();
    shape->AddRef();
    return fromShape(shape);
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  ShapeSettings base                                                        */
/* ========================================================================== */
JOLTC_API void JoltC_ShapeSettings_Destroy(JoltC_ShapeSettings* settings)
{
    if (!settings) return;
    JOLTC_TRY_BEGIN
    asShapeSettings(settings)->Release();
    JOLTC_TRY_END
}

JOLTC_API uint64_t JoltC_ShapeSettings_GetUserData(const JoltC_ShapeSettings* settings)
{
    if (!settings) return 0;
    JOLTC_TRY_BEGIN
    return asShapeSettings(settings)->mUserData;
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_ShapeSettings_SetUserData(JoltC_ShapeSettings* settings, uint64_t userData)
{
    if (!settings) return;
    JOLTC_TRY_BEGIN
    asShapeSettings(settings)->mUserData = userData;
    JOLTC_TRY_END
}

/* ========================================================================== */
/*  Shape_Destroy                                                             */
/* ========================================================================== */
JOLTC_API void JoltC_Shape_Destroy(const JoltC_Shape* shape)
{
    if (!shape) return;
    JOLTC_TRY_BEGIN
    asShape(shape)->Release();
    JOLTC_TRY_END
}

/* ========================================================================== */
/*  BoxShapeSettings                                                          */
/* ========================================================================== */
JOLTC_API JoltC_ShapeSettings* JoltC_BoxShapeSettings_Create(JoltC_Vec3 halfExtent, float convexRadius)
{
    JOLTC_TRY_BEGIN
    auto* s = new BoxShapeSettings(toJphVec3(halfExtent), convexRadius);
    s->AddRef();
    return fromShapeSettings(s);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API const JoltC_Shape* JoltC_BoxShapeSettings_CreateShape(JoltC_Vec3 halfExtent, float convexRadius)
{
    JOLTC_TRY_BEGIN
    BoxShapeSettings settings(toJphVec3(halfExtent), convexRadius);
    auto result = settings.Create();
    if (result.HasError()) { joltc_set_last_error(result.GetError().c_str()); return nullptr; }
    const Shape* shape = result.Get().GetPtr();
    shape->AddRef();
    return fromShape(shape);
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  SphereShapeSettings                                                       */
/* ========================================================================== */
JOLTC_API JoltC_ShapeSettings* JoltC_SphereShapeSettings_Create(float radius)
{
    JOLTC_TRY_BEGIN
    auto* s = new SphereShapeSettings(radius);
    s->AddRef();
    return fromShapeSettings(s);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API const JoltC_Shape* JoltC_SphereShapeSettings_CreateShape(float radius)
{
    JOLTC_TRY_BEGIN
    SphereShapeSettings settings(radius);
    auto result = settings.Create();
    if (result.HasError()) { joltc_set_last_error(result.GetError().c_str()); return nullptr; }
    const Shape* shape = result.Get().GetPtr();
    shape->AddRef();
    return fromShape(shape);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API float JoltC_SphereShapeSettings_GetRadius(const JoltC_ShapeSettings* settings)
{
    if (!settings) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const SphereShapeSettings*>(asShapeSettings(settings))->mRadius;
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API void JoltC_SphereShapeSettings_SetRadius(JoltC_ShapeSettings* settings, float radius)
{
    if (!settings) return;
    JOLTC_TRY_BEGIN
    static_cast<SphereShapeSettings*>(asShapeSettings(settings))->mRadius = radius;
    JOLTC_TRY_END
}

/* ========================================================================== */
/*  CapsuleShapeSettings                                                      */
/* ========================================================================== */
JOLTC_API JoltC_ShapeSettings* JoltC_CapsuleShapeSettings_Create(float halfHeight, float radius)
{
    JOLTC_TRY_BEGIN
    auto* s = new CapsuleShapeSettings(halfHeight, radius);
    s->AddRef();
    return fromShapeSettings(s);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API const JoltC_Shape* JoltC_CapsuleShapeSettings_CreateShape(float halfHeight, float radius)
{
    JOLTC_TRY_BEGIN
    CapsuleShapeSettings settings(halfHeight, radius);
    auto result = settings.Create();
    if (result.HasError()) { joltc_set_last_error(result.GetError().c_str()); return nullptr; }
    const Shape* shape = result.Get().GetPtr();
    shape->AddRef();
    return fromShape(shape);
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  CylinderShapeSettings                                                     */
/* ========================================================================== */
JOLTC_API JoltC_ShapeSettings* JoltC_CylinderShapeSettings_Create(float halfHeight, float radius, float convexRadius)
{
    JOLTC_TRY_BEGIN
    auto* s = new CylinderShapeSettings(halfHeight, radius, convexRadius);
    s->AddRef();
    return fromShapeSettings(s);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API const JoltC_Shape* JoltC_CylinderShapeSettings_CreateShape(float halfHeight, float radius, float convexRadius)
{
    JOLTC_TRY_BEGIN
    CylinderShapeSettings settings(halfHeight, radius, convexRadius);
    auto result = settings.Create();
    if (result.HasError()) { joltc_set_last_error(result.GetError().c_str()); return nullptr; }
    const Shape* shape = result.Get().GetPtr();
    shape->AddRef();
    return fromShape(shape);
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  TaperedCapsuleShapeSettings                                               */
/* ========================================================================== */
JOLTC_API JoltC_ShapeSettings* JoltC_TaperedCapsuleShapeSettings_Create(float halfHeight, float topRadius, float bottomRadius)
{
    JOLTC_TRY_BEGIN
    auto* s = new TaperedCapsuleShapeSettings(halfHeight, topRadius, bottomRadius);
    s->AddRef();
    return fromShapeSettings(s);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API const JoltC_Shape* JoltC_TaperedCapsuleShapeSettings_CreateShape(float halfHeight, float topRadius, float bottomRadius)
{
    JOLTC_TRY_BEGIN
    TaperedCapsuleShapeSettings settings(halfHeight, topRadius, bottomRadius);
    auto result = settings.Create();
    if (result.HasError()) { joltc_set_last_error(result.GetError().c_str()); return nullptr; }
    const Shape* shape = result.Get().GetPtr();
    shape->AddRef();
    return fromShape(shape);
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  TaperedCylinderShapeSettings                                              */
/* ========================================================================== */
JOLTC_API JoltC_ShapeSettings* JoltC_TaperedCylinderShapeSettings_Create(float halfHeight, float topRadius, float bottomRadius, float convexRadius)
{
    JOLTC_TRY_BEGIN
    auto* s = new TaperedCylinderShapeSettings(halfHeight, topRadius, bottomRadius, convexRadius);
    s->AddRef();
    return fromShapeSettings(s);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API const JoltC_Shape* JoltC_TaperedCylinderShapeSettings_CreateShape(float halfHeight, float topRadius, float bottomRadius, float convexRadius)
{
    JOLTC_TRY_BEGIN
    TaperedCylinderShapeSettings settings(halfHeight, topRadius, bottomRadius, convexRadius);
    auto result = settings.Create();
    if (result.HasError()) { joltc_set_last_error(result.GetError().c_str()); return nullptr; }
    const Shape* shape = result.Get().GetPtr();
    shape->AddRef();
    return fromShape(shape);
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  TriangleShapeSettings                                                     */
/* ========================================================================== */
JOLTC_API JoltC_ShapeSettings* JoltC_TriangleShapeSettings_Create(JoltC_Vec3 v1, JoltC_Vec3 v2, JoltC_Vec3 v3, float convexRadius)
{
    JOLTC_TRY_BEGIN
    auto* s = new TriangleShapeSettings(toJphVec3(v1), toJphVec3(v2), toJphVec3(v3), convexRadius);
    s->AddRef();
    return fromShapeSettings(s);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API const JoltC_Shape* JoltC_TriangleShapeSettings_CreateShape(JoltC_Vec3 v1, JoltC_Vec3 v2, JoltC_Vec3 v3, float convexRadius)
{
    JOLTC_TRY_BEGIN
    TriangleShapeSettings settings(toJphVec3(v1), toJphVec3(v2), toJphVec3(v3), convexRadius);
    auto result = settings.Create();
    if (result.HasError()) { joltc_set_last_error(result.GetError().c_str()); return nullptr; }
    const Shape* shape = result.Get().GetPtr();
    shape->AddRef();
    return fromShape(shape);
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  PlaneShapeSettings                                                        */
/* ========================================================================== */
JOLTC_API JoltC_ShapeSettings* JoltC_PlaneShapeSettings_Create(JoltC_Vec3 normal, float distance, float halfExtent)
{
    JOLTC_TRY_BEGIN
    auto* s = new PlaneShapeSettings(Plane(toJphVec3(normal), distance), nullptr, halfExtent);
    s->AddRef();
    return fromShapeSettings(s);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API const JoltC_Shape* JoltC_PlaneShapeSettings_CreateShape(JoltC_Vec3 normal, float distance, float halfExtent)
{
    JOLTC_TRY_BEGIN
    PlaneShapeSettings settings(Plane(toJphVec3(normal), distance), nullptr, halfExtent);
    auto result = settings.Create();
    if (result.HasError()) { joltc_set_last_error(result.GetError().c_str()); return nullptr; }
    const Shape* shape = result.Get().GetPtr();
    shape->AddRef();
    return fromShape(shape);
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  EmptyShapeSettings                                                        */
/* ========================================================================== */
JOLTC_API JoltC_ShapeSettings* JoltC_EmptyShapeSettings_Create(JoltC_Vec3 centerOfMass)
{
    JOLTC_TRY_BEGIN
    auto* s = new EmptyShapeSettings(toJphVec3(centerOfMass));
    s->AddRef();
    return fromShapeSettings(s);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API const JoltC_Shape* JoltC_EmptyShapeSettings_CreateShape(JoltC_Vec3 centerOfMass)
{
    JOLTC_TRY_BEGIN
    EmptyShapeSettings settings(toJphVec3(centerOfMass));
    auto result = settings.Create();
    if (result.HasError()) { joltc_set_last_error(result.GetError().c_str()); return nullptr; }
    const Shape* shape = result.Get().GetPtr();
    shape->AddRef();
    return fromShape(shape);
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  RotatedTranslatedShapeSettings                                            */
/* ========================================================================== */
JOLTC_API JoltC_ShapeSettings* JoltC_RotatedTranslatedShapeSettings_Create(
    JoltC_Vec3 position, JoltC_Quat rotation, const JoltC_ShapeSettings* innerSettings)
{
    if (!innerSettings) return nullptr;
    JOLTC_TRY_BEGIN
    auto* s = new RotatedTranslatedShapeSettings(toJphVec3(position), toJphQuat(rotation), asShapeSettings(innerSettings));
    s->AddRef();
    return fromShapeSettings(s);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_ShapeSettings* JoltC_RotatedTranslatedShapeSettings_Create2(
    JoltC_Vec3 position, JoltC_Quat rotation, const JoltC_Shape* innerShape)
{
    if (!innerShape) return nullptr;
    JOLTC_TRY_BEGIN
    auto* s = new RotatedTranslatedShapeSettings(toJphVec3(position), toJphQuat(rotation), asShape(innerShape));
    s->AddRef();
    return fromShapeSettings(s);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API const JoltC_Shape* JoltC_RotatedTranslatedShapeSettings_CreateShape(
    JoltC_Vec3 position, JoltC_Quat rotation, const JoltC_Shape* innerShape)
{
    if (!innerShape) return nullptr;
    JOLTC_TRY_BEGIN
    RotatedTranslatedShapeSettings settings(toJphVec3(position), toJphQuat(rotation), asShape(innerShape));
    auto result = settings.Create();
    if (result.HasError()) { joltc_set_last_error(result.GetError().c_str()); return nullptr; }
    const Shape* shape = result.Get().GetPtr();
    shape->AddRef();
    return fromShape(shape);
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  ScaledShapeSettings                                                       */
/* ========================================================================== */
JOLTC_API JoltC_ShapeSettings* JoltC_ScaledShapeSettings_Create(
    const JoltC_ShapeSettings* innerSettings, JoltC_Vec3 scale)
{
    if (!innerSettings) return nullptr;
    JOLTC_TRY_BEGIN
    auto* s = new ScaledShapeSettings(asShapeSettings(innerSettings), toJphVec3(scale));
    s->AddRef();
    return fromShapeSettings(s);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_ShapeSettings* JoltC_ScaledShapeSettings_Create2(
    const JoltC_Shape* innerShape, JoltC_Vec3 scale)
{
    if (!innerShape) return nullptr;
    JOLTC_TRY_BEGIN
    auto* s = new ScaledShapeSettings(asShape(innerShape), toJphVec3(scale));
    s->AddRef();
    return fromShapeSettings(s);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API const JoltC_Shape* JoltC_ScaledShapeSettings_CreateShape(
    const JoltC_Shape* innerShape, JoltC_Vec3 scale)
{
    if (!innerShape) return nullptr;
    JOLTC_TRY_BEGIN
    ScaledShapeSettings settings(asShape(innerShape), toJphVec3(scale));
    auto result = settings.Create();
    if (result.HasError()) { joltc_set_last_error(result.GetError().c_str()); return nullptr; }
    const Shape* shape = result.Get().GetPtr();
    shape->AddRef();
    return fromShape(shape);
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  OffsetCenterOfMassShapeSettings                                           */
/* ========================================================================== */
JOLTC_API JoltC_ShapeSettings* JoltC_OffsetCenterOfMassShapeSettings_Create(
    JoltC_Vec3 offset, const JoltC_ShapeSettings* innerSettings)
{
    if (!innerSettings) return nullptr;
    JOLTC_TRY_BEGIN
    auto* s = new OffsetCenterOfMassShapeSettings(toJphVec3(offset), asShapeSettings(innerSettings));
    s->AddRef();
    return fromShapeSettings(s);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_ShapeSettings* JoltC_OffsetCenterOfMassShapeSettings_Create2(
    JoltC_Vec3 offset, const JoltC_Shape* innerShape)
{
    if (!innerShape) return nullptr;
    JOLTC_TRY_BEGIN
    auto* s = new OffsetCenterOfMassShapeSettings(toJphVec3(offset), asShape(innerShape));
    s->AddRef();
    return fromShapeSettings(s);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API const JoltC_Shape* JoltC_OffsetCenterOfMassShapeSettings_CreateShape(
    JoltC_Vec3 offset, const JoltC_Shape* innerShape)
{
    if (!innerShape) return nullptr;
    JOLTC_TRY_BEGIN
    OffsetCenterOfMassShapeSettings settings(toJphVec3(offset), asShape(innerShape));
    auto result = settings.Create();
    if (result.HasError()) { joltc_set_last_error(result.GetError().c_str()); return nullptr; }
    const Shape* shape = result.Get().GetPtr();
    shape->AddRef();
    return fromShape(shape);
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  StaticCompoundShapeSettings / MutableCompoundShapeSettings                */
/* ========================================================================== */
JOLTC_API JoltC_ShapeSettings* JoltC_StaticCompoundShapeSettings_Create(void)
{
    JOLTC_TRY_BEGIN
    auto* s = new StaticCompoundShapeSettings();
    s->AddRef();
    return fromShapeSettings(s);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_ShapeSettings* JoltC_MutableCompoundShapeSettings_Create(void)
{
    JOLTC_TRY_BEGIN
    auto* s = new MutableCompoundShapeSettings();
    s->AddRef();
    return fromShapeSettings(s);
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  CompoundShapeSettings_AddShape                                            */
/* ========================================================================== */
JOLTC_API void JoltC_CompoundShapeSettings_AddShape(
    JoltC_ShapeSettings* settings, JoltC_Vec3 position, JoltC_Quat rotation,
    const JoltC_ShapeSettings* shape, uint32_t userData)
{
    if (!settings || !shape) return;
    JOLTC_TRY_BEGIN
    static_cast<CompoundShapeSettings*>(asShapeSettings(settings))->AddShape(
        toJphVec3(position), toJphQuat(rotation), asShapeSettings(shape), userData);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_CompoundShapeSettings_AddShape2(
    JoltC_ShapeSettings* settings, JoltC_Vec3 position, JoltC_Quat rotation,
    const JoltC_Shape* shape, uint32_t userData)
{
    if (!settings || !shape) return;
    JOLTC_TRY_BEGIN
    static_cast<CompoundShapeSettings*>(asShapeSettings(settings))->AddShape(
        toJphVec3(position), toJphQuat(rotation), asShape(shape), userData);
    JOLTC_TRY_END
}

/* ========================================================================== */
/*  ConvexShapeSettings density                                               */
/* ========================================================================== */
JOLTC_API float JoltC_ConvexShapeSettings_GetDensity(const JoltC_ShapeSettings* settings)
{
    if (!settings) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const ConvexShapeSettings*>(asShapeSettings(settings))->mDensity;
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API void JoltC_ConvexShapeSettings_SetDensity(JoltC_ShapeSettings* settings, float density)
{
    if (!settings) return;
    JOLTC_TRY_BEGIN
    static_cast<ConvexShapeSettings*>(asShapeSettings(settings))->SetDensity(density);
    JOLTC_TRY_END
}

/* ========================================================================== */
/*  TriangleShape getters                                                     */
/* ========================================================================== */
JOLTC_API float JoltC_TriangleShape_GetConvexRadius(const JoltC_Shape* shape)
{
    if (!shape) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const TriangleShape*>(asShape(shape))->GetConvexRadius();
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API JoltC_Vec3 JoltC_TriangleShape_GetVertex1(const JoltC_Shape* shape)
{
    JoltC_Vec3 z = {0,0,0}; if (!shape) return z;
    JOLTC_TRY_BEGIN
    return fromJphVec3(static_cast<const TriangleShape*>(asShape(shape))->GetVertex1());
    JOLTC_TRY_END
    return z;
}

JOLTC_API JoltC_Vec3 JoltC_TriangleShape_GetVertex2(const JoltC_Shape* shape)
{
    JoltC_Vec3 z = {0,0,0}; if (!shape) return z;
    JOLTC_TRY_BEGIN
    return fromJphVec3(static_cast<const TriangleShape*>(asShape(shape))->GetVertex2());
    JOLTC_TRY_END
    return z;
}

JOLTC_API JoltC_Vec3 JoltC_TriangleShape_GetVertex3(const JoltC_Shape* shape)
{
    JoltC_Vec3 z = {0,0,0}; if (!shape) return z;
    JOLTC_TRY_BEGIN
    return fromJphVec3(static_cast<const TriangleShape*>(asShape(shape))->GetVertex3());
    JOLTC_TRY_END
    return z;
}

/* ========================================================================== */
/*  TaperedCylinderShape getters                                              */
/* ========================================================================== */
JOLTC_API float JoltC_TaperedCylinderShape_GetHalfHeight(const JoltC_Shape* shape)
{
    if (!shape) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const TaperedCylinderShape*>(asShape(shape))->GetHalfHeight();
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API float JoltC_TaperedCylinderShape_GetTopRadius(const JoltC_Shape* shape)
{
    if (!shape) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const TaperedCylinderShape*>(asShape(shape))->GetTopRadius();
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API float JoltC_TaperedCylinderShape_GetBottomRadius(const JoltC_Shape* shape)
{
    if (!shape) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const TaperedCylinderShape*>(asShape(shape))->GetBottomRadius();
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API float JoltC_TaperedCylinderShape_GetConvexRadius(const JoltC_Shape* shape)
{
    if (!shape) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const TaperedCylinderShape*>(asShape(shape))->GetConvexRadius();
    JOLTC_TRY_END
    return 0.0f;
}

/* ========================================================================== */
/*  PlaneShape getters                                                        */
/* ========================================================================== */
JOLTC_API float JoltC_PlaneShape_GetHalfExtent(const JoltC_Shape* shape)
{
    if (!shape) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const PlaneShape*>(asShape(shape))->GetHalfExtent();
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API JoltC_Vec3 JoltC_PlaneShape_GetPlane(const JoltC_Shape* shape, float* outDistance)
{
    JoltC_Vec3 z = {0,0,0};
    if (!shape) return z;
    JOLTC_TRY_BEGIN
    const Plane& p = static_cast<const PlaneShape*>(asShape(shape))->GetPlane();
    if (outDistance) *outDistance = p.GetConstant();
    return fromJphVec3(p.GetNormal());
    JOLTC_TRY_END
    return z;
}

/* ========================================================================== */
/*  Shape base — additional functions                                         */
/* ========================================================================== */
JOLTC_API JoltC_Vec3 JoltC_Shape_GetSurfaceNormal(const JoltC_Shape* shape, uint32_t subShapeId, JoltC_Vec3 localSurfacePosition)
{
    JoltC_Vec3 z = {0,0,0};
    if (!shape) return z;
    JOLTC_TRY_BEGIN
    SubShapeID sid;
    sid.SetValue(subShapeId);
    return fromJphVec3(asShape(shape)->GetSurfaceNormal(sid, toJphVec3(localSurfacePosition)));
    JOLTC_TRY_END
    return z;
}

JOLTC_API void JoltC_Shape_GetWorldSpaceBounds(const JoltC_Shape* shape, JoltC_Mat44 centerOfMassTransform, JoltC_Vec3 scale, JoltC_Vec3* outMin, JoltC_Vec3* outMax)
{
    if (!shape || !outMin || !outMax) return;
    JOLTC_TRY_BEGIN
    AABox box = asShape(shape)->GetWorldSpaceBounds(toJphMat44(centerOfMassTransform), toJphVec3(scale));
    *outMin = fromJphVec3(Vec3(box.mMin));
    *outMax = fromJphVec3(Vec3(box.mMax));
    JOLTC_TRY_END
}

JOLTC_API int JoltC_Shape_IsValidScale(const JoltC_Shape* shape, JoltC_Vec3 scale)
{
    if (!shape) return 0;
    JOLTC_TRY_BEGIN
    return asShape(shape)->IsValidScale(toJphVec3(scale)) ? 1 : 0;
    JOLTC_TRY_END
    return 0;
}

JOLTC_API JoltC_Vec3 JoltC_Shape_MakeScaleValid(const JoltC_Shape* shape, JoltC_Vec3 scale)
{
    JoltC_Vec3 z = {0,0,0};
    if (!shape) return z;
    JOLTC_TRY_BEGIN
    return fromJphVec3(asShape(shape)->MakeScaleValid(toJphVec3(scale)));
    JOLTC_TRY_END
    return z;
}

JOLTC_API const JoltC_Shape* JoltC_Shape_ScaleShape(const JoltC_Shape* shape, JoltC_Vec3 scale)
{
    if (!shape) return nullptr;
    JOLTC_TRY_BEGIN
    Shape::ShapeResult result = asShape(shape)->ScaleShape(toJphVec3(scale));
    if (result.HasError()) {
        joltc_set_last_error(result.GetError().c_str());
        return nullptr;
    }
    const Shape* s = result.Get().GetPtr();
    s->AddRef();
    return fromShape(s);
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  HeightFieldShape — ProjectOntoSurface                                     */
/* ========================================================================== */
JOLTC_API int JoltC_HeightFieldShape_ProjectOntoSurface(const JoltC_Shape* shape, JoltC_Vec3 localPosition, JoltC_Vec3* outSurfacePosition, uint32_t* outSubShapeId)
{
    if (!shape || !outSurfacePosition || !outSubShapeId) return 0;
    JOLTC_TRY_BEGIN
    auto* hf = static_cast<const HeightFieldShape*>(asShape(shape));
    Vec3 surfPos;
    SubShapeID sid;
    bool ok = hf->ProjectOntoSurface(toJphVec3(localPosition), surfPos, sid);
    if (ok) {
        *outSurfacePosition = fromJphVec3(surfPos);
        *outSubShapeId = sid.GetValue();
    }
    return ok ? 1 : 0;
    JOLTC_TRY_END
    return 0;
}

/* ========================================================================== */
/*  HeightFieldShapeSettings — generic (via JoltC_ShapeSettings*)             */
/* ========================================================================== */
JOLTC_API float JoltC_HeightFieldShapeSettings_GetActiveEdgeCosThresholdAngle(const JoltC_ShapeSettings* settings)
{
    if (!settings) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const HeightFieldShapeSettings*>(asShapeSettings(settings))->mActiveEdgeCosThresholdAngle;
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API void JoltC_HeightFieldShapeSettings_SetActiveEdgeCosThresholdAngle(JoltC_ShapeSettings* settings, float angle)
{
    if (!settings) return;
    JOLTC_TRY_BEGIN
    static_cast<HeightFieldShapeSettings*>(asShapeSettings(settings))->mActiveEdgeCosThresholdAngle = angle;
    JOLTC_TRY_END
}

JOLTC_API void JoltC_HeightFieldShapeSettings_SetSampleCount(JoltC_ShapeSettings* settings, uint32_t count)
{
    if (!settings) return;
    JOLTC_TRY_BEGIN
    static_cast<HeightFieldShapeSettings*>(asShapeSettings(settings))->mSampleCount = count;
    JOLTC_TRY_END
}

JOLTC_API uint8_t JoltC_HeightFieldShapeSettings_CalculateBitsPerSampleForError(JoltC_ShapeSettings* settings, float maxError)
{
    if (!settings) return 1;
    JOLTC_TRY_BEGIN
    return (uint8_t)static_cast<HeightFieldShapeSettings*>(asShapeSettings(settings))->CalculateBitsPerSampleForError(maxError);
    JOLTC_TRY_END
    return 1;
}

JOLTC_API void JoltC_HeightFieldShapeSettings_DetermineMinAndMaxSample(JoltC_ShapeSettings* settings, float* outMin, float* outMax, float* outQuantizationScale)
{
    if (!settings || !outMin || !outMax || !outQuantizationScale) return;
    JOLTC_TRY_BEGIN
    static_cast<const HeightFieldShapeSettings*>(asShapeSettings(settings))->DetermineMinAndMaxSample(*outMin, *outMax, *outQuantizationScale);
    JOLTC_TRY_END
}

/* ========================================================================== */
/*  MeshShapeSettings — generic (via JoltC_ShapeSettings*)                    */
/* ========================================================================== */
JOLTC_API float JoltC_MeshShapeSettings_GetActiveEdgeCosThresholdAngle(const JoltC_ShapeSettings* settings)
{
    if (!settings) return 0.0f;
    JOLTC_TRY_BEGIN
    return static_cast<const MeshShapeSettings*>(asShapeSettings(settings))->mActiveEdgeCosThresholdAngle;
    JOLTC_TRY_END
    return 0.0f;
}

JOLTC_API void JoltC_MeshShapeSettings_SetActiveEdgeCosThresholdAngle(JoltC_ShapeSettings* settings, float angle)
{
    if (!settings) return;
    JOLTC_TRY_BEGIN
    static_cast<MeshShapeSettings*>(asShapeSettings(settings))->mActiveEdgeCosThresholdAngle = angle;
    JOLTC_TRY_END
}

JOLTC_API int JoltC_MeshShapeSettings_GetPerTriangleUserData(const JoltC_ShapeSettings* settings)
{
    if (!settings) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<const MeshShapeSettings*>(asShapeSettings(settings))->mPerTriangleUserData ? 1 : 0;
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_MeshShapeSettings_SetPerTriangleUserData(JoltC_ShapeSettings* settings, int perTriangleUserData)
{
    if (!settings) return;
    JOLTC_TRY_BEGIN
    static_cast<MeshShapeSettings*>(asShapeSettings(settings))->mPerTriangleUserData = (perTriangleUserData != 0);
    JOLTC_TRY_END
}

JOLTC_API int JoltC_MeshShapeSettings_GetBuildQuality(const JoltC_ShapeSettings* settings)
{
    if (!settings) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<int>(static_cast<const MeshShapeSettings*>(asShapeSettings(settings))->mBuildQuality);
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_MeshShapeSettings_SetBuildQuality(JoltC_ShapeSettings* settings, int quality)
{
    if (!settings) return;
    JOLTC_TRY_BEGIN
    static_cast<MeshShapeSettings*>(asShapeSettings(settings))->mBuildQuality = static_cast<MeshShapeSettings::EBuildQuality>(quality);
    JOLTC_TRY_END
}

/* ========================================================================== */
/*  MeshShape — instance queries                                              */
/* ========================================================================== */
JOLTC_API uint32_t JoltC_MeshShape_GetTriangleUserData(const JoltC_Shape* shape, uint32_t subShapeId)
{
    if (!shape) return 0;
    JOLTC_TRY_BEGIN
    SubShapeID sid;
    sid.SetValue(subShapeId);
    return static_cast<const MeshShape*>(asShape(shape))->GetTriangleUserData(sid);
    JOLTC_TRY_END
    return 0;
}

/* ========================================================================== */
/*  CompoundShape — GetSubShapeIndexFromID                                    */
/* ========================================================================== */
JOLTC_API uint32_t JoltC_CompoundShape_GetSubShapeIndexFromID(const JoltC_Shape* shape, uint32_t subShapeId, uint32_t* outRemainder)
{
    if (!shape || !outRemainder) return 0;
    JOLTC_TRY_BEGIN
    SubShapeID sid;
    sid.SetValue(subShapeId);
    SubShapeID remainder;
    uint32_t idx = static_cast<const CompoundShape*>(asShape(shape))->GetSubShapeIndexFromID(sid, remainder);
    *outRemainder = remainder.GetValue();
    return idx;
    JOLTC_TRY_END
    return 0;
}

/* ========================================================================== */
/*  MutableCompoundShape — ModifyShape2                                       */
/* ========================================================================== */
JOLTC_API void JoltC_MutableCompoundShape_ModifyShape2(JoltC_Shape* shape, uint32_t subShapeIndex, JoltC_Vec3 position, JoltC_Quat rotation, const JoltC_Shape* newShape)
{
    if (!shape || !newShape) return;
    JOLTC_TRY_BEGIN
    static_cast<MutableCompoundShape*>(const_cast<Shape*>(asShape(shape)))->ModifyShape(subShapeIndex, toJphVec3(position), toJphQuat(rotation), asShape(newShape));
    JOLTC_TRY_END
}

/* ========================================================================== */
/*  CollideShapeSettings / ShapeCastSettings init                             */
/* ========================================================================== */
JOLTC_API void JoltC_CollideShapeSettings_Init(JoltC_CollideShapeSettings* settings)
{
    if (!settings) return;
    CollideShapeSettings defaults;
    settings->backFaceMode          = static_cast<JoltC_BackFaceMode>(defaults.mBackFaceMode);
    settings->maxSeparationDistance = defaults.mMaxSeparationDistance;
    settings->collisionTolerance    = defaults.mCollisionTolerance;
    settings->penetrationTolerance  = defaults.mPenetrationTolerance;
    settings->internalEdgeRemovalVertexToleranceSq = defaults.mInternalEdgeRemovalVertexToleranceSq;
}

JOLTC_API void JoltC_ShapeCastSettings_Init(JoltC_ShapeCastSettings* settings)
{
    if (!settings) return;
    ShapeCastSettings defaults;
    settings->backFaceModeTriangles       = static_cast<JoltC_BackFaceMode>(defaults.mBackFaceModeTriangles);
    settings->backFaceModeConvex          = static_cast<JoltC_BackFaceMode>(defaults.mBackFaceModeConvex);
    settings->useShrunkenShapeAndConvexRadius = defaults.mUseShrunkenShapeAndConvexRadius ? JOLTC_TRUE : JOLTC_FALSE;
    settings->returnDeepestPoint          = defaults.mReturnDeepestPoint ? JOLTC_TRUE : JOLTC_FALSE;
    settings->collisionTolerance          = defaults.mCollisionTolerance;
    settings->penetrationTolerance        = defaults.mPenetrationTolerance;
    settings->extraConvexRadius           = defaults.mExtraConvexRadius;
}

/* ========================================================================== */
/*  CollideShapeResult / CollisionEstimationResult free helpers               */
/* ========================================================================== */
JOLTC_API void JoltC_CollideShapeResult_FreeMembers(JoltC_CollideShapeResult* result)
{
    /* The C struct is blittable and doesn't own any heap memory — this is a no-op
       provided for API completeness / FFI parity. */
    (void)result;
}

JOLTC_API void JoltC_CollisionEstimationResult_FreeMembers(JoltC_CollisionEstimationResult* result)
{
    /* The C wrapper does not include the dynamic impulse array — no-op. */
    (void)result;
}

} /* extern "C" */
