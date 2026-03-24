/* JoltC - PhysicsSystem, JobSystem, TempAllocator, and callback wrappers
 * SPDX-License-Identifier: MIT
 */

#include <Jolt/Jolt.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/Color.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterMask.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceMask.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterMask.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>

#include <JoltC/physics_system.h>
#include "internal.h"
#include "wrappers.h"
#include "errors_internal.h"

using namespace JPH;

extern "C" {

/* -------------------------------------------------------------------------- */
/*  TempAllocator                                                             */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_TempAllocator* JoltC_TempAllocator_Create(uint32_t size)
{
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_TempAllocator();
    w->ptr = std::make_unique<TempAllocatorImpl>(static_cast<size_t>(size));
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_TempAllocator_Destroy(JoltC_TempAllocator* allocator)
{
    if (!allocator) return;
    JOLTC_TRY_BEGIN
    delete allocator;
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  JobSystem                                                                 */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_JobSystem* JoltC_JobSystemThreadPool_Create(uint32_t maxJobs, uint32_t maxBarriers, int numThreads)
{
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_JobSystem();
    w->ptr = std::make_unique<JobSystemThreadPool>(maxJobs, maxBarriers, numThreads);
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_JobSystem_Destroy(JoltC_JobSystem* jobSystem)
{
    if (!jobSystem) return;
    JOLTC_TRY_BEGIN
    delete jobSystem;
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  BroadPhaseLayerInterface                                                  */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_BroadPhaseLayerInterface* JoltC_BroadPhaseLayerInterface_Create(
    JoltC_GetNumBroadPhaseLayersFn getNumLayers,
    JoltC_GetBroadPhaseLayerFn     getBroadPhaseLayer,
    void*                          userData)
{
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_BroadPhaseLayerInterface();
    auto cb = std::make_unique<BroadPhaseLayerInterfaceCallback>();
    cb->fnGetNumLayers      = getNumLayers;
    cb->fnGetBroadPhaseLayer = getBroadPhaseLayer;
    cb->userData             = userData;
    w->ptr = std::move(cb);
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_BroadPhaseLayerInterface_Destroy(JoltC_BroadPhaseLayerInterface* iface)
{
    if (!iface) return;
    JOLTC_TRY_BEGIN
    delete iface;
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  ObjectVsBroadPhaseLayerFilter                                             */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_ObjectVsBroadPhaseLayerFilter* JoltC_ObjectVsBroadPhaseLayerFilter_Create(
    JoltC_ObjectVsBroadPhaseLayerFilterFn filterFn,
    void*                                 userData)
{
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_ObjectVsBroadPhaseLayerFilter();
    auto cb = std::make_unique<ObjectVsBroadPhaseLayerFilterCallback>();
    cb->fnFilter  = filterFn;
    cb->userData  = userData;
    w->ptr = std::move(cb);
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_ObjectVsBroadPhaseLayerFilter_Destroy(JoltC_ObjectVsBroadPhaseLayerFilter* filter)
{
    if (!filter) return;
    JOLTC_TRY_BEGIN
    delete filter;
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  ObjectLayerPairFilter                                                     */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_ObjectLayerPairFilter* JoltC_ObjectLayerPairFilter_Create(
    JoltC_ObjectLayerPairFilterFn filterFn,
    void*                         userData)
{
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_ObjectLayerPairFilter();
    auto cb = std::make_unique<ObjectLayerPairFilterCallback>();
    cb->fnFilter = filterFn;
    cb->userData = userData;
    w->ptr = std::move(cb);
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_ObjectLayerPairFilter_Destroy(JoltC_ObjectLayerPairFilter* filter)
{
    if (!filter) return;
    JOLTC_TRY_BEGIN
    delete filter;
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  ContactListener                                                           */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_ContactListener* JoltC_ContactListener_Create(
    JoltC_OnContactValidateFn  onValidate,
    JoltC_OnContactAddedFn     onAdded,
    JoltC_OnContactPersistedFn onPersisted,
    JoltC_OnContactRemovedFn   onRemoved,
    void*                      userData)
{
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_ContactListener();
    auto cb = std::make_unique<ContactListenerCallback>();
    cb->fnValidate  = onValidate;
    cb->fnAdded     = onAdded;
    cb->fnPersisted = onPersisted;
    cb->fnRemoved   = onRemoved;
    cb->userData    = userData;
    w->ptr = std::move(cb);
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_ContactListener_Destroy(JoltC_ContactListener* listener)
{
    if (!listener) return;
    JOLTC_TRY_BEGIN
    delete listener;
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  BodyActivationListener                                                    */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_BodyActivationListener* JoltC_BodyActivationListener_Create(
    JoltC_OnBodyActivatedFn   onActivated,
    JoltC_OnBodyDeactivatedFn onDeactivated,
    void*                     userData)
{
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_BodyActivationListener();
    w->ptr = std::make_unique<BodyActivationListenerCallback>();
    w->ptr->fnActivated   = onActivated;
    w->ptr->fnDeactivated = onDeactivated;
    w->ptr->userData      = userData;
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_BodyActivationListener_Destroy(JoltC_BodyActivationListener* listener)
{
    if (!listener) return;
    JOLTC_TRY_BEGIN
    delete listener;
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  PhysicsSystem                                                             */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_PhysicsSystem* JoltC_PhysicsSystem_Create(void)
{
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_PhysicsSystem();
    w->ptr = std::make_unique<PhysicsSystem>();
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_PhysicsSystem_Destroy(JoltC_PhysicsSystem* system)
{
    if (!system) return;
    JOLTC_TRY_BEGIN
    delete system;
    JOLTC_TRY_END
}

JOLTC_API void JoltC_PhysicsSystem_Init(
    JoltC_PhysicsSystem*                   system,
    uint32_t                               maxBodies,
    uint32_t                               numBodyMutexes,
    uint32_t                               maxBodyPairs,
    uint32_t                               maxContactConstraints,
    JoltC_BroadPhaseLayerInterface*        broadPhaseLayerInterface,
    JoltC_ObjectVsBroadPhaseLayerFilter*   objectVsBroadPhaseLayerFilter,
    JoltC_ObjectLayerPairFilter*           objectLayerPairFilter)
{
    if (!system || !broadPhaseLayerInterface || !objectVsBroadPhaseLayerFilter || !objectLayerPairFilter) return;
    JOLTC_TRY_BEGIN
    system->ptr->Init(
        maxBodies,
        numBodyMutexes,
        maxBodyPairs,
        maxContactConstraints,
        *broadPhaseLayerInterface->ptr,
        *objectVsBroadPhaseLayerFilter->ptr,
        *objectLayerPairFilter->ptr);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_PhysicsSystem_OptimizeBroadPhase(JoltC_PhysicsSystem* system)
{
    if (!system) return;
    JOLTC_TRY_BEGIN
    system->ptr->OptimizeBroadPhase();
    JOLTC_TRY_END
}

JOLTC_API uint32_t JoltC_PhysicsSystem_Update(
    JoltC_PhysicsSystem* system,
    float                deltaTime,
    int                  collisionSteps,
    JoltC_TempAllocator* tempAllocator,
    JoltC_JobSystem*     jobSystem)
{
    if (!system || !tempAllocator || !jobSystem) return 0;
    JOLTC_TRY_BEGIN
    EPhysicsUpdateError err = system->ptr->Update(deltaTime, collisionSteps, tempAllocator->ptr.get(), jobSystem->ptr.get());
    return static_cast<uint32_t>(err);
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_PhysicsSystem_SetGravity(JoltC_PhysicsSystem* system, JoltC_Vec3 gravity)
{
    if (!system) return;
    JOLTC_TRY_BEGIN
    system->ptr->SetGravity(toJphVec3(gravity));
    JOLTC_TRY_END
}

JOLTC_API JoltC_Vec3 JoltC_PhysicsSystem_GetGravity(const JoltC_PhysicsSystem* system)
{
    if (!system) return JoltC_Vec3{0, 0, 0};
    JOLTC_TRY_BEGIN
    return fromJphVec3(system->ptr->GetGravity());
    JOLTC_TRY_END
    return JoltC_Vec3{0, 0, 0};
}

JOLTC_API void JoltC_PhysicsSystem_SetContactListener(JoltC_PhysicsSystem* system, JoltC_ContactListener* listener)
{
    if (!system) return;
    JOLTC_TRY_BEGIN
    system->ptr->SetContactListener(listener ? listener->ptr.get() : nullptr);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_PhysicsSystem_SetBodyActivationListener(JoltC_PhysicsSystem* system, JoltC_BodyActivationListener* listener)
{
    if (!system) return;
    JOLTC_TRY_BEGIN
    system->ptr->SetBodyActivationListener(listener ? listener->ptr.get() : nullptr);
    JOLTC_TRY_END
}

JOLTC_API uint32_t JoltC_PhysicsSystem_GetNumBodies(const JoltC_PhysicsSystem* system)
{
    if (!system) return 0;
    JOLTC_TRY_BEGIN
    return system->ptr->GetNumBodies();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API uint32_t JoltC_PhysicsSystem_GetNumActiveBodies(const JoltC_PhysicsSystem* system, JoltC_BodyType bodyType)
{
    if (!system) return 0;
    JOLTC_TRY_BEGIN
    return system->ptr->GetNumActiveBodies(toJphBodyType(bodyType));
    JOLTC_TRY_END
    return 0;
}

JOLTC_API uint32_t JoltC_PhysicsSystem_GetMaxBodies(const JoltC_PhysicsSystem* system)
{
    if (!system) return 0;
    JOLTC_TRY_BEGIN
    return system->ptr->GetMaxBodies();
    JOLTC_TRY_END
    return 0;
}

/* -------------------------------------------------------------------------- */
/*  BodyInterface accessors                                                   */
/* -------------------------------------------------------------------------- */

/* We keep two static BodyInterface wrappers per PhysicsSystem to avoid allocation.
   Since they're non-owning, we store them as statics in thread-safe manner —
   actually we just allocate them on the heap per call since they cost very little. */

JOLTC_API JoltC_BodyInterface* JoltC_PhysicsSystem_GetBodyInterface(JoltC_PhysicsSystem* system)
{
    if (!system) return nullptr;
    JOLTC_TRY_BEGIN
    /* Return a thin wrapper. Caller must NOT destroy this — it lives as long as system. */
    static thread_local JoltC_BodyInterface wrapper;
    wrapper.ptr = &system->ptr->GetBodyInterface();
    wrapper.system = system->ptr.get();
    return &wrapper;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_BodyInterface* JoltC_PhysicsSystem_GetBodyInterfaceNoLock(JoltC_PhysicsSystem* system)
{
    if (!system) return nullptr;
    JOLTC_TRY_BEGIN
    static thread_local JoltC_BodyInterface wrapper;
    wrapper.ptr = &system->ptr->GetBodyInterfaceNoLock();
    wrapper.system = system->ptr.get();
    return &wrapper;
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  Additional PhysicsSystem queries                                          */
/* -------------------------------------------------------------------------- */
JOLTC_API int JoltC_PhysicsSystem_WereBodiesInContact(const JoltC_PhysicsSystem* system, JoltC_BodyID body1, JoltC_BodyID body2)
{
    if (!system) return 0;
    JOLTC_TRY_BEGIN
    return system->ptr->WereBodiesInContact(toJphBodyID(body1), toJphBodyID(body2)) ? 1 : 0;
    JOLTC_TRY_END
    return 0;
}

JOLTC_API uint32_t JoltC_PhysicsSystem_GetNumConstraints(const JoltC_PhysicsSystem* system)
{
    if (!system) return 0;
    JOLTC_TRY_BEGIN
    return static_cast<uint32_t>(system->ptr->GetConstraints().size());
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_PhysicsSystem_GetBodies(const JoltC_PhysicsSystem* system, JoltC_BodyID* outIDs, uint32_t maxCount)
{
    if (!system || !outIDs || maxCount == 0) return;
    JOLTC_TRY_BEGIN
    BodyIDVector ids;
    system->ptr->GetBodies(ids);
    uint32_t count = static_cast<uint32_t>(ids.size());
    if (count > maxCount) count = maxCount;
    for (uint32_t i = 0; i < count; i++)
        outIDs[i] = fromJphBodyID(ids[i]);
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  ObjectLayerPairFilterTable                                                */
/* -------------------------------------------------------------------------- */

/* The callback-based ObjectLayerPairFilter stores a unique_ptr<ObjectLayerPairFilterCallback>.
   For built-in table/mask implementations, we store a different subclass in the same slot.
   We use a tagged-union approach: the wrapper struct's `ptr` field is a unique_ptr<ObjectLayerPairFilter>.
   But our current wrapper uses ObjectLayerPairFilterCallback which is our custom class...
   Instead, let's create a new wrapper that stores the base class and reuse the opaque handle. */

/* Helper: cast to table (dynamic_cast) */
static ObjectLayerPairFilterTable* asOLPFTable(JoltC_ObjectLayerPairFilter* f) {
    return dynamic_cast<ObjectLayerPairFilterTable*>(f->ptr.get());
}

JOLTC_API JoltC_ObjectLayerPairFilter* JoltC_ObjectLayerPairFilterTable_Create(uint32_t numObjectLayers)
{
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_ObjectLayerPairFilter;
    w->ptr = std::make_unique<ObjectLayerPairFilterTable>(numObjectLayers);
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_ObjectLayerPairFilterTable_DisableCollision(JoltC_ObjectLayerPairFilter* filter, JoltC_ObjectLayer layer1, JoltC_ObjectLayer layer2)
{
    if (!filter) return;
    JOLTC_TRY_BEGIN
    auto* table = asOLPFTable(filter);
    if (table) table->DisableCollision(layer1, layer2);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_ObjectLayerPairFilterTable_EnableCollision(JoltC_ObjectLayerPairFilter* filter, JoltC_ObjectLayer layer1, JoltC_ObjectLayer layer2)
{
    if (!filter) return;
    JOLTC_TRY_BEGIN
    auto* table = asOLPFTable(filter);
    if (table) table->EnableCollision(layer1, layer2);
    JOLTC_TRY_END
}

JOLTC_API JoltC_Bool JoltC_ObjectLayerPairFilterTable_ShouldCollide(const JoltC_ObjectLayerPairFilter* filter, JoltC_ObjectLayer layer1, JoltC_ObjectLayer layer2)
{
    if (!filter) return JOLTC_FALSE;
    JOLTC_TRY_BEGIN
    return filter->ptr->ShouldCollide(layer1, layer2) ? JOLTC_TRUE : JOLTC_FALSE;
    JOLTC_TRY_END
    return JOLTC_FALSE;
}

/* -------------------------------------------------------------------------- */
/*  ObjectLayerPairFilterMask                                                 */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_ObjectLayerPairFilter* JoltC_ObjectLayerPairFilterMask_Create(void)
{
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_ObjectLayerPairFilter;
    w->ptr = std::make_unique<ObjectLayerPairFilterMask>();
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API JoltC_ObjectLayer JoltC_ObjectLayerPairFilterMask_GetObjectLayer(uint32_t group, uint32_t mask)
{
    return ObjectLayerPairFilterMask::sGetObjectLayer(group, mask);
}

JOLTC_API uint32_t JoltC_ObjectLayerPairFilterMask_GetGroup(JoltC_ObjectLayer layer)
{
    return ObjectLayerPairFilterMask::sGetGroup(layer);
}

JOLTC_API uint32_t JoltC_ObjectLayerPairFilterMask_GetMask(JoltC_ObjectLayer layer)
{
    return ObjectLayerPairFilterMask::sGetMask(layer);
}

/* -------------------------------------------------------------------------- */
/*  BroadPhaseLayerInterfaceTable                                             */
/* -------------------------------------------------------------------------- */

static BroadPhaseLayerInterfaceTable* asBPLITable(JoltC_BroadPhaseLayerInterface* i) {
    return dynamic_cast<BroadPhaseLayerInterfaceTable*>(i->ptr.get());
}

JOLTC_API JoltC_BroadPhaseLayerInterface* JoltC_BroadPhaseLayerInterfaceTable_Create(uint32_t numObjectLayers, uint32_t numBroadPhaseLayers)
{
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_BroadPhaseLayerInterface;
    w->ptr = std::make_unique<BroadPhaseLayerInterfaceTable>(numObjectLayers, numBroadPhaseLayers);
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(JoltC_BroadPhaseLayerInterface* iface, JoltC_ObjectLayer objectLayer, JoltC_BroadPhaseLayer broadPhaseLayer)
{
    if (!iface) return;
    JOLTC_TRY_BEGIN
    auto* table = asBPLITable(iface);
    if (table) table->MapObjectToBroadPhaseLayer(objectLayer, BroadPhaseLayer(broadPhaseLayer));
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  BroadPhaseLayerInterfaceMask                                              */
/* -------------------------------------------------------------------------- */

static BroadPhaseLayerInterfaceMask* asBPLIMask(JoltC_BroadPhaseLayerInterface* i) {
    return dynamic_cast<BroadPhaseLayerInterfaceMask*>(i->ptr.get());
}

JOLTC_API JoltC_BroadPhaseLayerInterface* JoltC_BroadPhaseLayerInterfaceMask_Create(uint32_t numBroadPhaseLayers)
{
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_BroadPhaseLayerInterface;
    w->ptr = std::make_unique<BroadPhaseLayerInterfaceMask>(numBroadPhaseLayers);
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_BroadPhaseLayerInterfaceMask_ConfigureLayer(JoltC_BroadPhaseLayerInterface* iface, JoltC_BroadPhaseLayer broadPhaseLayer, uint32_t groupsToInclude, uint32_t groupsToExclude)
{
    if (!iface) return;
    JOLTC_TRY_BEGIN
    auto* mask = asBPLIMask(iface);
    if (mask) mask->ConfigureLayer(BroadPhaseLayer(broadPhaseLayer), groupsToInclude, groupsToExclude);
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  GroupFilter / GroupFilterTable                                            */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_GroupFilter_Destroy(JoltC_GroupFilter* filter)
{
    delete filter;
}

JOLTC_API JoltC_Bool JoltC_GroupFilter_CanCollide(const JoltC_GroupFilter* filter, const JoltC_CollisionGroup* group1, const JoltC_CollisionGroup* group2)
{
    if (!filter || !group1 || !group2) return JOLTC_FALSE;
    JOLTC_TRY_BEGIN
    CollisionGroup g1, g2;
    g1.SetGroupFilter(filter->ptr.GetPtr());
    g1.SetGroupID(group1->groupID);
    g1.SetSubGroupID(group1->subGroupID);
    g2.SetGroupFilter(filter->ptr.GetPtr());
    g2.SetGroupID(group2->groupID);
    g2.SetSubGroupID(group2->subGroupID);
    return filter->ptr->CanCollide(g1, g2) ? JOLTC_TRUE : JOLTC_FALSE;
    JOLTC_TRY_END
    return JOLTC_FALSE;
}

static GroupFilterTable* asGFTable(JoltC_GroupFilter* f) {
    return dynamic_cast<GroupFilterTable*>(f->ptr.GetPtr());
}
static const GroupFilterTable* asGFTableConst(const JoltC_GroupFilter* f) {
    return dynamic_cast<const GroupFilterTable*>(f->ptr.GetPtr());
}

JOLTC_API JoltC_GroupFilter* JoltC_GroupFilterTable_Create(uint32_t numSubGroups)
{
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_GroupFilter;
    auto* table = new GroupFilterTable(numSubGroups);
    table->AddRef();
    w->ptr = table;
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_GroupFilterTable_DisableCollision(JoltC_GroupFilter* filter, JoltC_CollisionSubGroupID subGroup1, JoltC_CollisionSubGroupID subGroup2)
{
    if (!filter) return;
    JOLTC_TRY_BEGIN
    auto* table = asGFTable(filter);
    if (table) table->DisableCollision(subGroup1, subGroup2);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_GroupFilterTable_EnableCollision(JoltC_GroupFilter* filter, JoltC_CollisionSubGroupID subGroup1, JoltC_CollisionSubGroupID subGroup2)
{
    if (!filter) return;
    JOLTC_TRY_BEGIN
    auto* table = asGFTable(filter);
    if (table) table->EnableCollision(subGroup1, subGroup2);
    JOLTC_TRY_END
}

JOLTC_API JoltC_Bool JoltC_GroupFilterTable_IsCollisionEnabled(const JoltC_GroupFilter* filter, JoltC_CollisionSubGroupID subGroup1, JoltC_CollisionSubGroupID subGroup2)
{
    if (!filter) return JOLTC_FALSE;
    JOLTC_TRY_BEGIN
    auto* table = asGFTableConst(filter);
    if (table) return table->IsCollisionEnabled(subGroup1, subGroup2) ? JOLTC_TRUE : JOLTC_FALSE;
    JOLTC_TRY_END
    return JOLTC_FALSE;
}

/* -------------------------------------------------------------------------- */
/*  PhysicsMaterial                                                           */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_PhysicsMaterial* JoltC_PhysicsMaterial_Create(const char* name, uint32_t color)
{
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_PhysicsMaterial;
    auto* m = new PhysicsMaterialImpl;
    m->name = name ? name : "";
    m->color = Color(color);
    m->AddRef();
    w->ptr = m;
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_PhysicsMaterial_Destroy(JoltC_PhysicsMaterial* material)
{
    delete material;
}

JOLTC_API const char* JoltC_PhysicsMaterial_GetDebugName(const JoltC_PhysicsMaterial* material)
{
    if (!material) return "";
    return material->ptr->GetDebugName();
}

JOLTC_API uint32_t JoltC_PhysicsMaterial_GetDebugColor(const JoltC_PhysicsMaterial* material)
{
    if (!material) return 0;
    return material->ptr->GetDebugColor().GetUInt32();
}

/* -------------------------------------------------------------------------- */
/*  PhysicsStepListener                                                       */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_PhysicsStepListener* JoltC_PhysicsStepListener_Create(JoltC_OnPhysicsStepFn fn, void* userData)
{
    if (!fn) return nullptr;
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_PhysicsStepListener;
    w->ptr = std::make_unique<PhysicsStepListenerCallback>();
    w->ptr->fn = fn;
    w->ptr->userData = userData;
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_PhysicsStepListener_Destroy(JoltC_PhysicsStepListener* listener)
{
    delete listener;
}

JOLTC_API void JoltC_PhysicsSystem_AddStepListener(JoltC_PhysicsSystem* system, JoltC_PhysicsStepListener* listener)
{
    if (!system || !listener) return;
    JOLTC_TRY_BEGIN
    system->ptr->AddStepListener(listener->ptr.get());
    JOLTC_TRY_END
}

JOLTC_API void JoltC_PhysicsSystem_RemoveStepListener(JoltC_PhysicsSystem* system, JoltC_PhysicsStepListener* listener)
{
    if (!system || !listener) return;
    JOLTC_TRY_BEGIN
    system->ptr->RemoveStepListener(listener->ptr.get());
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  SimShapeFilter on PhysicsSystem                                           */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_PhysicsSystem_SetSimShapeFilter(JoltC_PhysicsSystem* system, const JoltC_SimShapeFilter* filter)
{
    if (!system) return;
    JOLTC_TRY_BEGIN
    system->ptr->SetSimShapeFilter(filter ? filter->ptr.get() : nullptr);
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  Enhanced ContactListener                                                  */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_ContactListener* JoltC_ContactListener_CreateEnhanced(
    JoltC_OnContactValidateEnhancedFn  onValidate,
    JoltC_OnContactAddedEnhancedFn     onAdded,
    JoltC_OnContactPersistedEnhancedFn onPersisted,
    JoltC_OnContactRemovedEnhancedFn   onRemoved,
    void*                              userData)
{
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_ContactListener;
    auto* cb = new ContactListenerEnhancedCallback;
    cb->fnValidate = onValidate;
    cb->fnAdded = onAdded;
    cb->fnPersisted = onPersisted;
    cb->fnRemoved = onRemoved;
    cb->userData = userData;
    w->ptr.reset(cb);
    return w;
    JOLTC_TRY_END
    return nullptr;
}

/* -------------------------------------------------------------------------- */
/*  PhysicsSystem — constraints                                               */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_PhysicsSystem_AddConstraints(JoltC_PhysicsSystem* system, JoltC_Constraint** constraints, int count)
{
    if (!system || !constraints || count <= 0) return;
    JOLTC_TRY_BEGIN
    auto arr = std::make_unique<Constraint*[]>(count);
    for (int i = 0; i < count; i++)
        arr[i] = constraints[i] ? constraints[i]->ptr.GetPtr() : nullptr;
    system->ptr->AddConstraints(arr.get(), count);
    JOLTC_TRY_END
}

JOLTC_API void JoltC_PhysicsSystem_RemoveConstraints(JoltC_PhysicsSystem* system, JoltC_Constraint** constraints, int count)
{
    if (!system || !constraints || count <= 0) return;
    JOLTC_TRY_BEGIN
    auto arr = std::make_unique<Constraint*[]>(count);
    for (int i = 0; i < count; i++)
        arr[i] = constraints[i] ? constraints[i]->ptr.GetPtr() : nullptr;
    system->ptr->RemoveConstraints(arr.get(), count);
    JOLTC_TRY_END
}

JOLTC_API int JoltC_PhysicsSystem_GetConstraints(const JoltC_PhysicsSystem* system, JoltC_Constraint** outConstraints, int maxCount)
{
    if (!system || !outConstraints || maxCount <= 0) return 0;
    JOLTC_TRY_BEGIN
    Constraints cs = system->ptr->GetConstraints();
    int count = static_cast<int>(cs.size());
    if (count > maxCount) count = maxCount;
    for (int i = 0; i < count; i++) {
        auto* w = new JoltC_Constraint;
        w->ptr = cs[i];
        outConstraints[i] = w;
    }
    return count;
    JOLTC_TRY_END
    return 0;
}

/* -------------------------------------------------------------------------- */
/*  PhysicsSystem — settings                                                  */
/* -------------------------------------------------------------------------- */
static JoltC_PhysicsSettings fromJphPhysicsSettings(const PhysicsSettings& s) {
    JoltC_PhysicsSettings r;
    r.maxInFlightBodyPairs                 = s.mMaxInFlightBodyPairs;
    r.stepListenersBatchSize               = s.mStepListenersBatchSize;
    r.stepListenerBatchesPerJob            = s.mStepListenerBatchesPerJob;
    r.baumgarte                            = s.mBaumgarte;
    r.speculativeContactDistance           = s.mSpeculativeContactDistance;
    r.penetrationSlop                      = s.mPenetrationSlop;
    r.linearCastThreshold                  = s.mLinearCastThreshold;
    r.linearCastMaxPenetration             = s.mLinearCastMaxPenetration;
    r.manifoldTolerance                    = s.mManifoldTolerance;
    r.maxPenetrationDistance               = s.mMaxPenetrationDistance;
    r.bodyPairCacheMaxDeltaPositionSq      = s.mBodyPairCacheMaxDeltaPositionSq;
    r.bodyPairCacheCosMaxDeltaRotationDiv2 = s.mBodyPairCacheCosMaxDeltaRotationDiv2;
    r.contactNormalCosMaxDeltaRotation     = s.mContactNormalCosMaxDeltaRotation;
    r.contactPointPreserveLambdaMaxDistSq  = s.mContactPointPreserveLambdaMaxDistSq;
    r.internalEdgeRemovalVertexToleranceSq = s.mInternalEdgeRemovalVertexToleranceSq;
    r.numVelocitySteps                     = s.mNumVelocitySteps;
    r.numPositionSteps                     = s.mNumPositionSteps;
    r.minVelocityForRestitution            = s.mMinVelocityForRestitution;
    r.timeBeforeSleep                      = s.mTimeBeforeSleep;
    r.pointVelocitySleepThreshold          = s.mPointVelocitySleepThreshold;
    r.deterministicSimulation              = s.mDeterministicSimulation ? JOLTC_TRUE : JOLTC_FALSE;
    r.constraintWarmStart                  = s.mConstraintWarmStart     ? JOLTC_TRUE : JOLTC_FALSE;
    r.useBodyPairContactCache              = s.mUseBodyPairContactCache ? JOLTC_TRUE : JOLTC_FALSE;
    r.useManifoldReduction                 = s.mUseManifoldReduction    ? JOLTC_TRUE : JOLTC_FALSE;
    r.useLargeIslandSplitter               = s.mUseLargeIslandSplitter  ? JOLTC_TRUE : JOLTC_FALSE;
    r.allowSleeping                        = s.mAllowSleeping           ? JOLTC_TRUE : JOLTC_FALSE;
    r.checkActiveEdges                     = s.mCheckActiveEdges        ? JOLTC_TRUE : JOLTC_FALSE;
    return r;
}

static PhysicsSettings toJphPhysicsSettings(const JoltC_PhysicsSettings& s) {
    PhysicsSettings r;
    r.mMaxInFlightBodyPairs                 = s.maxInFlightBodyPairs;
    r.mStepListenersBatchSize               = s.stepListenersBatchSize;
    r.mStepListenerBatchesPerJob            = s.stepListenerBatchesPerJob;
    r.mBaumgarte                            = s.baumgarte;
    r.mSpeculativeContactDistance           = s.speculativeContactDistance;
    r.mPenetrationSlop                      = s.penetrationSlop;
    r.mLinearCastThreshold                  = s.linearCastThreshold;
    r.mLinearCastMaxPenetration             = s.linearCastMaxPenetration;
    r.mManifoldTolerance                    = s.manifoldTolerance;
    r.mMaxPenetrationDistance               = s.maxPenetrationDistance;
    r.mBodyPairCacheMaxDeltaPositionSq      = s.bodyPairCacheMaxDeltaPositionSq;
    r.mBodyPairCacheCosMaxDeltaRotationDiv2 = s.bodyPairCacheCosMaxDeltaRotationDiv2;
    r.mContactNormalCosMaxDeltaRotation     = s.contactNormalCosMaxDeltaRotation;
    r.mContactPointPreserveLambdaMaxDistSq  = s.contactPointPreserveLambdaMaxDistSq;
    r.mInternalEdgeRemovalVertexToleranceSq = s.internalEdgeRemovalVertexToleranceSq;
    r.mNumVelocitySteps                     = s.numVelocitySteps;
    r.mNumPositionSteps                     = s.numPositionSteps;
    r.mMinVelocityForRestitution            = s.minVelocityForRestitution;
    r.mTimeBeforeSleep                      = s.timeBeforeSleep;
    r.mPointVelocitySleepThreshold          = s.pointVelocitySleepThreshold;
    r.mDeterministicSimulation              = s.deterministicSimulation != 0;
    r.mConstraintWarmStart                  = s.constraintWarmStart != 0;
    r.mUseBodyPairContactCache              = s.useBodyPairContactCache != 0;
    r.mUseManifoldReduction                 = s.useManifoldReduction != 0;
    r.mUseLargeIslandSplitter               = s.useLargeIslandSplitter != 0;
    r.mAllowSleeping                        = s.allowSleeping != 0;
    r.mCheckActiveEdges                     = s.checkActiveEdges != 0;
    return r;
}

JOLTC_API void JoltC_PhysicsSystem_GetPhysicsSettings(const JoltC_PhysicsSystem* system, JoltC_PhysicsSettings* outSettings)
{
    if (!system || !outSettings) return;
    JOLTC_TRY_BEGIN
    *outSettings = fromJphPhysicsSettings(system->ptr->GetPhysicsSettings());
    JOLTC_TRY_END
}

JOLTC_API void JoltC_PhysicsSystem_SetPhysicsSettings(JoltC_PhysicsSystem* system, const JoltC_PhysicsSettings* settings)
{
    if (!system || !settings) return;
    JOLTC_TRY_BEGIN
    system->ptr->SetPhysicsSettings(toJphPhysicsSettings(*settings));
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  PhysicsSystem — activate bodies in AABB                                   */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_PhysicsSystem_ActivateBodiesInAABox(JoltC_PhysicsSystem* system, JoltC_Vec3 min, JoltC_Vec3 max)
{
    if (!system) return;
    JOLTC_TRY_BEGIN
    AABox box(toJphVec3(min), toJphVec3(max));
    system->ptr->GetBodyInterface().ActivateBodiesInAABox(box, BroadPhaseLayerFilter(), ObjectLayerFilter());
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  Listener SetProcs                                                         */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_BodyActivationListener_SetProcs(JoltC_BodyActivationListener* listener, JoltC_BodyActivationListener_Procs procs, void* userData)
{
    if (!listener || !listener->ptr) return;
    JOLTC_TRY_BEGIN
    listener->ptr->fnActivated   = procs.onBodyActivated;
    listener->ptr->fnDeactivated = procs.onBodyDeactivated;
    listener->ptr->userData      = userData;
    JOLTC_TRY_END
}

JOLTC_API void JoltC_ContactListener_SetProcs(JoltC_ContactListener* listener, JoltC_ContactListener_Procs procs, void* userData)
{
    if (!listener || !listener->ptr) return;
    JOLTC_TRY_BEGIN
    auto* cb = dynamic_cast<ContactListenerCallback*>(listener->ptr.get());
    if (cb) {
        cb->fnValidate  = procs.onContactValidate;
        cb->fnAdded     = procs.onContactAdded;
        cb->fnPersisted = procs.onContactPersisted;
        cb->fnRemoved   = procs.onContactRemoved;
        cb->userData    = userData;
    }
    JOLTC_TRY_END
}

JOLTC_API void JoltC_PhysicsStepListener_SetProcs(JoltC_PhysicsStepListener* listener, JoltC_PhysicsStepListener_Procs procs, void* userData)
{
    if (!listener || !listener->ptr) return;
    JOLTC_TRY_BEGIN
    listener->ptr->fn       = procs.onStep;
    listener->ptr->userData = userData;
    JOLTC_TRY_END
}

/* ========================================================================== */
/*  ObjectVsBroadPhaseLayerFilterTable                                        */
/* ========================================================================== */
JOLTC_API JoltC_ObjectVsBroadPhaseLayerFilter* JoltC_ObjectVsBroadPhaseLayerFilterTable_Create(
    const JoltC_BroadPhaseLayerInterface* bpInterface,
    uint32_t                              numBroadPhaseLayers,
    const JoltC_ObjectLayerPairFilter*    objectFilter,
    uint32_t                              numObjectLayers)
{
    if (!bpInterface || !objectFilter) return nullptr;
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_ObjectVsBroadPhaseLayerFilter();
    w->ptr = std::make_unique<ObjectVsBroadPhaseLayerFilterTable>(
        *bpInterface->ptr, numBroadPhaseLayers, *objectFilter->ptr, numObjectLayers);
    return w;
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  ObjectVsBroadPhaseLayerFilterMask                                         */
/* ========================================================================== */
JOLTC_API JoltC_ObjectVsBroadPhaseLayerFilter* JoltC_ObjectVsBroadPhaseLayerFilterMask_Create(
    const JoltC_BroadPhaseLayerInterface* bpInterface)
{
    if (!bpInterface) return nullptr;
    JOLTC_TRY_BEGIN
    auto* maskIface = dynamic_cast<const BroadPhaseLayerInterfaceMask*>(bpInterface->ptr.get());
    if (!maskIface) {
        joltc_set_last_error("BroadPhaseLayerInterface is not a BroadPhaseLayerInterfaceMask");
        return nullptr;
    }
    auto* w = new JoltC_ObjectVsBroadPhaseLayerFilter();
    w->ptr = std::make_unique<ObjectVsBroadPhaseLayerFilterMask>(*maskIface);
    return w;
    JOLTC_TRY_END
    return nullptr;
}

/* ========================================================================== */
/*  SoftBodyCreationSettings                                                  */
/* ========================================================================== */
} /* extern "C" -- temporarily close for SoftBody section */

extern "C" {

JOLTC_API JoltC_SoftBodyCreationSettings* JoltC_SoftBodyCreationSettings_Create(void)
{
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_SoftBodyCreationSettings_Impl();
    return reinterpret_cast<JoltC_SoftBodyCreationSettings*>(w);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_SoftBodyCreationSettings_Destroy(JoltC_SoftBodyCreationSettings* settings)
{
    delete reinterpret_cast<JoltC_SoftBodyCreationSettings_Impl*>(settings);
}

} /* extern "C" */
