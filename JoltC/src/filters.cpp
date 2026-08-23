/* JoltC - Query filter implementations
 * SPDX-License-Identifier: MIT
 */

#include <Jolt/Jolt.h>
#include "errors_internal.h"
#include "internal.h"
#include "wrappers.h"

#include <JoltC/filters.h>

using namespace JPH;

extern "C" {

/* -------------------------------------------------------------------------- */
/*  BroadPhaseLayerFilter                                                     */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_BroadPhaseLayerFilter* JoltC_BroadPhaseLayerFilter_Create(
    JoltC_BroadPhaseLayerFilterFn fn,
    void* userData)
{
    if (!fn) return nullptr;
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_BroadPhaseLayerFilter;
    auto callback = std::make_unique<BroadPhaseLayerFilterCallback>();
    callback->fn = fn;
    callback->userData = userData;
    w->callback = callback.get();
    w->ptr = std::move(callback);
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_BroadPhaseLayerFilter_Destroy(JoltC_BroadPhaseLayerFilter* filter) {
    delete filter;
}

/* -------------------------------------------------------------------------- */
/*  ObjectLayerFilter                                                         */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_ObjectLayerFilter* JoltC_ObjectLayerFilter_Create(
    JoltC_ObjectLayerFilterFn fn,
    void* userData)
{
    if (!fn) return nullptr;
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_ObjectLayerFilter;
    auto callback = std::make_unique<ObjectLayerFilterCallback>();
    callback->fn = fn;
    callback->userData = userData;
    w->callback = callback.get();
    w->ptr = std::move(callback);
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_ObjectLayerFilter_Destroy(JoltC_ObjectLayerFilter* filter) {
    delete filter;
}

/* -------------------------------------------------------------------------- */
/*  BodyFilter                                                                */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_BodyFilter* JoltC_BodyFilter_Create(
    JoltC_BodyFilterFn fn,
    JoltC_BodyFilterLockedFn fnLocked,
    void* userData)
{
    if (!fn) return nullptr;
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_BodyFilter;
    w->ptr = std::make_unique<BodyFilterCallback>();
    w->ptr->fn = fn;
    w->ptr->fnLocked = fnLocked;
    w->ptr->userData = userData;
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_BodyFilter_Destroy(JoltC_BodyFilter* filter) {
    delete filter;
}

/* -------------------------------------------------------------------------- */
/*  ShapeFilter                                                               */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_ShapeFilter* JoltC_ShapeFilter_Create(
    JoltC_ShapeFilterFn fn,
    JoltC_ShapeFilter2Fn fn2,
    void* userData)
{
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_ShapeFilter;
    w->ptr = std::make_unique<ShapeFilterCallback>();
    w->ptr->fn1 = fn;
    w->ptr->fn2 = fn2;
    w->ptr->userData = userData;
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_ShapeFilter_Destroy(JoltC_ShapeFilter* filter) {
    delete filter;
}

JOLTC_API JoltC_BodyID JoltC_ShapeFilter_GetBodyID2(const JoltC_ShapeFilter* filter) {
    if (!filter) return JOLTC_BODY_ID_INVALID;
    return filter->ptr->mBodyID2.GetIndexAndSequenceNumber();
}

JOLTC_API void JoltC_ShapeFilter_SetBodyID2(JoltC_ShapeFilter* filter, JoltC_BodyID bodyID) {
    if (!filter) return;
    filter->ptr->mBodyID2 = BodyID(bodyID);
}

/* -------------------------------------------------------------------------- */
/*  SimShapeFilter                                                            */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_SimShapeFilter* JoltC_SimShapeFilter_Create(
    JoltC_SimShapeFilterFn fn,
    void* userData)
{
    if (!fn) return nullptr;
    JOLTC_TRY_BEGIN
    auto* w = new JoltC_SimShapeFilter;
    w->ptr = std::make_unique<SimShapeFilterCallback>();
    w->ptr->fn = fn;
    w->ptr->userData = userData;
    return w;
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_SimShapeFilter_Destroy(JoltC_SimShapeFilter* filter) {
    delete filter;
}

/* -------------------------------------------------------------------------- */
/*  SetProcs - update function pointers on existing filter objects            */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_BroadPhaseLayerFilter_SetProcs(JoltC_BroadPhaseLayerFilter* filter, JoltC_BroadPhaseLayerFilter_Procs procs, void* userData)
{
    /* Only the callback kind has procs to write; a default filter obtained from the system does
     * not, and quietly ignoring the call beats writing through the wrong type. */
    if (!filter || !filter->callback) return;
    JOLTC_TRY_BEGIN
    filter->callback->fn       = procs.shouldCollide;
    filter->callback->userData = userData;
    JOLTC_TRY_END
}

JOLTC_API void JoltC_ObjectLayerFilter_SetProcs(JoltC_ObjectLayerFilter* filter, JoltC_ObjectLayerFilter_Procs procs, void* userData)
{
    if (!filter || !filter->callback) return;
    JOLTC_TRY_BEGIN
    filter->callback->fn       = procs.shouldCollide;
    filter->callback->userData = userData;
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyFilter_SetProcs(JoltC_BodyFilter* filter, JoltC_BodyFilter_Procs procs, void* userData)
{
    if (!filter || !filter->ptr) return;
    JOLTC_TRY_BEGIN
    filter->ptr->fn       = procs.shouldCollide;
    filter->ptr->fnLocked = procs.shouldCollideLocked;
    filter->ptr->userData = userData;
    JOLTC_TRY_END
}

JOLTC_API void JoltC_ShapeFilter_SetProcs(JoltC_ShapeFilter* filter, JoltC_ShapeFilter_Procs procs, void* userData)
{
    if (!filter || !filter->ptr) return;
    JOLTC_TRY_BEGIN
    filter->ptr->fn1      = procs.shouldCollide;
    filter->ptr->fn2      = procs.shouldCollide2;
    filter->ptr->userData = userData;
    JOLTC_TRY_END
}

JOLTC_API void JoltC_SimShapeFilter_SetProcs(JoltC_SimShapeFilter* filter, JoltC_SimShapeFilter_Procs procs, void* userData)
{
    if (!filter || !filter->ptr) return;
    JOLTC_TRY_BEGIN
    filter->ptr->fn       = procs.shouldCollide;
    filter->ptr->userData = userData;
    JOLTC_TRY_END
}

} /* extern "C" */
