/* JoltC - Init/Shutdown and common helpers
 * SPDX-License-Identifier: MIT
 */

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/Memory.h>
#include <Jolt/Core/IssueReporting.h>
#include <JoltC/common.h>
#include "errors_internal.h"

#include <cstdarg>
#include <cstdio>

using namespace JPH;

extern "C" {

JOLTC_API void JoltC_RegisterDefaultAllocator(void)
{
    JOLTC_TRY_BEGIN
    RegisterDefaultAllocator();
    JOLTC_TRY_END
}

JOLTC_API void JoltC_CreateFactory(void)
{
    JOLTC_TRY_BEGIN
    Factory::sInstance = new Factory();
    JOLTC_TRY_END
}

JOLTC_API void JoltC_DestroyFactory(void)
{
    JOLTC_TRY_BEGIN
    delete Factory::sInstance;
    Factory::sInstance = nullptr;
    JOLTC_TRY_END
}

JOLTC_API void JoltC_RegisterTypes(void)
{
    JOLTC_TRY_BEGIN
    JPH::RegisterTypes();
    JOLTC_TRY_END
}

JOLTC_API void JoltC_UnregisterTypes(void)
{
    JOLTC_TRY_BEGIN
    JPH::UnregisterTypes();
    JOLTC_TRY_END
}

JOLTC_API void JoltC_BodyCreationSettings_SetDefault(JoltC_BodyCreationSettings* settings)
{
    if (!settings) return;
    JOLTC_TRY_BEGIN

    settings->position     = JoltC_RVec3{0, 0, 0};
    settings->rotation     = JoltC_Quat{0, 0, 0, 1};
    settings->linearVelocity  = JoltC_Vec3{0, 0, 0};
    settings->angularVelocity = JoltC_Vec3{0, 0, 0};
    settings->userData        = 0;
    settings->objectLayer     = 0;
    settings->collisionGroup.groupFilter = nullptr;
    settings->collisionGroup.groupID = 0;
    settings->collisionGroup.subGroupID = 0;
    settings->motionType      = JOLTC_MOTION_TYPE_DYNAMIC;
    settings->allowedDOFs     = JOLTC_ALLOWED_DOFS_ALL;
    settings->allowDynamicOrKinematic     = JOLTC_FALSE;
    settings->isSensor                    = JOLTC_FALSE;
    settings->collideKinematicVsNonDynamic = JOLTC_FALSE;
    settings->useManifoldReduction        = JOLTC_TRUE;
    settings->applyGyroscopicForce        = JOLTC_FALSE;
    settings->motionQuality               = JOLTC_MOTION_QUALITY_DISCRETE;
    settings->enhancedInternalEdgeRemoval = JOLTC_FALSE;
    settings->allowSleeping               = JOLTC_TRUE;
    settings->friction         = 0.2f;
    settings->restitution      = 0.0f;
    settings->linearDamping    = 0.05f;
    settings->angularDamping   = 0.05f;
    settings->maxLinearVelocity  = 500.0f;
    settings->maxAngularVelocity = 0.25f * 3.14159265358979323846f * 60.0f;
    settings->gravityFactor    = 1.0f;
    settings->numVelocityStepsOverride = 0;
    settings->numPositionStepsOverride = 0;
    settings->overrideMassProperties = JOLTC_OVERRIDE_MASS_CALC_MASS_AND_INERTIA;
    settings->inertiaMultiplier = 1.0f;
    settings->massPropertiesOverride.mass = 0.0f;
    for (int i = 0; i < 16; i++) settings->massPropertiesOverride.inertia.m[i] = 0.0f;
    settings->shape             = nullptr;

    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  Init / Shutdown                                                           */
/* -------------------------------------------------------------------------- */
JOLTC_API int JoltC_Init(void)
{
    JOLTC_TRY_BEGIN
    RegisterDefaultAllocator();
    Factory::sInstance = new Factory;
    RegisterTypes();
    return 1;
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_Shutdown(void)
{
    JOLTC_TRY_BEGIN
    UnregisterTypes();
    delete Factory::sInstance;
    Factory::sInstance = nullptr;
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  Trace / Assert handlers                                                   */
/* -------------------------------------------------------------------------- */
static void (*s_traceHandler)(const char* message) = nullptr;

static void joltc_trace_impl(const char* inFMT, ...)
{
    if (!s_traceHandler) return;
    char buffer[1024];
    va_list args;
    va_start(args, inFMT);
    vsnprintf(buffer, sizeof(buffer), inFMT, args);
    va_end(args);
    s_traceHandler(buffer);
}

JOLTC_API void JoltC_SetTraceHandler(void (*handler)(const char* message))
{
    s_traceHandler = handler;
    Trace = handler ? joltc_trace_impl : nullptr;
}

#ifdef JPH_ENABLE_ASSERTS
static int (*s_assertHandler)(const char* expression, const char* message, const char* file, unsigned int line) = nullptr;

static bool joltc_assert_impl(const char* inExpression, const char* inMessage, const char* inFile, uint inLine)
{
    if (!s_assertHandler) return true;
    return s_assertHandler(inExpression, inMessage, inFile, static_cast<unsigned int>(inLine)) != 0;
}
#endif

JOLTC_API void JoltC_SetAssertFailureHandler(int (*handler)(const char* expression, const char* message, const char* file, unsigned int line))
{
#ifdef JPH_ENABLE_ASSERTS
    s_assertHandler = handler;
    AssertFailed = handler ? joltc_assert_impl : nullptr;
#else
    (void)handler;
#endif
}

} /* extern "C" */
