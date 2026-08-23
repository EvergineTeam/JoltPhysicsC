/* JoltC - Debug renderer: Jolt's own drawing reduced to three C callbacks
 * SPDX-License-Identifier: MIT
 */

#include <Jolt/Jolt.h>
#include <Jolt/Renderer/DebugRendererSimple.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyManager.h>

#include <JoltC/debug_renderer.h>
#include "internal.h"
#include "errors_internal.h"
#include "wrappers.h"

#include <string>

using namespace JPH;

/* Everything Jolt draws arrives here and leaves as a C callback. DebugRendererSimple already
 * reduces solid geometry to DrawTriangle calls; a missing triangle callback degrades further to
 * the three edges as lines, so a line-only renderer still sees every shape. */
class DebugRendererCallback final : public DebugRendererSimple
{
public:
    JoltC_DebugDrawLineFn fnLine = nullptr;
    JoltC_DebugDrawTriangleFn fnTriangle = nullptr;
    JoltC_DebugDrawText3DFn fnText = nullptr;
    void* userData = nullptr;

    void DrawLine(RVec3Arg inFrom, RVec3Arg inTo, ColorArg inColor) override
    {
        if (!fnLine) return;
        fnLine(userData, ToC(inFrom), ToC(inTo), inColor.GetUInt32());
    }

    void DrawTriangle(RVec3Arg inV1, RVec3Arg inV2, RVec3Arg inV3, ColorArg inColor, ECastShadow inCastShadow) override
    {
        if (fnTriangle)
        {
            fnTriangle(userData, ToC(inV1), ToC(inV2), ToC(inV3), inColor.GetUInt32(),
                       inCastShadow == ECastShadow::On ? JOLTC_TRUE : JOLTC_FALSE);
            return;
        }

        /* The wireframe fallback of DebugRendererSimple: three edges through DrawLine. */
        DebugRendererSimple::DrawTriangle(inV1, inV2, inV3, inColor, inCastShadow);
    }

    void DrawText3D(RVec3Arg inPosition, const string_view& inString, ColorArg inColor, float inHeight) override
    {
        if (!fnText) return;

        /* The view is not terminated; the callback wants a C string. */
        std::string text(inString);
        fnText(userData, ToC(inPosition), text.c_str(), inColor.GetUInt32(), inHeight);
    }

private:
    static JoltC_RVec3 ToC(RVec3Arg v)
    {
        JoltC_RVec3 r;
        r.x = (float)v.GetX();
        r.y = (float)v.GetY();
        r.z = (float)v.GetZ();
        return r;
    }
};

static inline DebugRendererCallback* asRenderer(JoltC_DebugRenderer* r) { return reinterpret_cast<DebugRendererCallback*>(r); }

extern "C" {

JOLTC_API JoltC_DebugRenderer* JoltC_DebugRenderer_Create(
    JoltC_DebugDrawLineFn drawLine,
    JoltC_DebugDrawTriangleFn drawTriangle,
    JoltC_DebugDrawText3DFn drawText3D,
    void* userData)
{
    if (!drawLine) return nullptr;
    JOLTC_TRY_BEGIN
    auto* renderer = new DebugRendererCallback();
    renderer->fnLine = drawLine;
    renderer->fnTriangle = drawTriangle;
    renderer->fnText = drawText3D;
    renderer->userData = userData;
    return reinterpret_cast<JoltC_DebugRenderer*>(renderer);
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_DebugRenderer_Destroy(JoltC_DebugRenderer* renderer)
{
    delete asRenderer(renderer);
}

JOLTC_API void JoltC_BodyDrawSettings_Init(JoltC_BodyDrawSettings* settings)
{
    if (!settings) return;
    BodyManager::DrawSettings defaults;
    settings->drawGetSupportFunction = defaults.mDrawGetSupportFunction ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawSupportDirection = defaults.mDrawSupportDirection ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawGetSupportingFace = defaults.mDrawGetSupportingFace ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawShape = defaults.mDrawShape ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawShapeWireframe = defaults.mDrawShapeWireframe ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawShapeColor = static_cast<JoltC_ShapeColor>(defaults.mDrawShapeColor);
    settings->drawBoundingBox = defaults.mDrawBoundingBox ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawCenterOfMassTransform = defaults.mDrawCenterOfMassTransform ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawWorldTransform = defaults.mDrawWorldTransform ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawVelocity = defaults.mDrawVelocity ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawMassAndInertia = defaults.mDrawMassAndInertia ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawSleepStats = defaults.mDrawSleepStats ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawSoftBodyVertices = defaults.mDrawSoftBodyVertices ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawSoftBodyVertexVelocities = defaults.mDrawSoftBodyVertexVelocities ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawSoftBodyEdgeConstraints = defaults.mDrawSoftBodyEdgeConstraints ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawSoftBodyBendConstraints = defaults.mDrawSoftBodyBendConstraints ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawSoftBodyVolumeConstraints = defaults.mDrawSoftBodyVolumeConstraints ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawSoftBodySkinConstraints = defaults.mDrawSoftBodySkinConstraints ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawSoftBodyLRAConstraints = defaults.mDrawSoftBodyLRAConstraints ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawSoftBodyRods = defaults.mDrawSoftBodyRods ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawSoftBodyRodStates = defaults.mDrawSoftBodyRodStates ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawSoftBodyRodBendTwistConstraints = defaults.mDrawSoftBodyRodBendTwistConstraints ? JOLTC_TRUE : JOLTC_FALSE;
    settings->drawSoftBodyPredictedBounds = defaults.mDrawSoftBodyPredictedBounds ? JOLTC_TRUE : JOLTC_FALSE;
}

static void fillDrawSettings(BodyManager::DrawSettings& out, const JoltC_BodyDrawSettings* in)
{
    if (!in) return;
    out.mDrawGetSupportFunction = in->drawGetSupportFunction != 0;
    out.mDrawSupportDirection = in->drawSupportDirection != 0;
    out.mDrawGetSupportingFace = in->drawGetSupportingFace != 0;
    out.mDrawShape = in->drawShape != 0;
    out.mDrawShapeWireframe = in->drawShapeWireframe != 0;
    out.mDrawShapeColor = static_cast<BodyManager::EShapeColor>(in->drawShapeColor);
    out.mDrawBoundingBox = in->drawBoundingBox != 0;
    out.mDrawCenterOfMassTransform = in->drawCenterOfMassTransform != 0;
    out.mDrawWorldTransform = in->drawWorldTransform != 0;
    out.mDrawVelocity = in->drawVelocity != 0;
    out.mDrawMassAndInertia = in->drawMassAndInertia != 0;
    out.mDrawSleepStats = in->drawSleepStats != 0;
    out.mDrawSoftBodyVertices = in->drawSoftBodyVertices != 0;
    out.mDrawSoftBodyVertexVelocities = in->drawSoftBodyVertexVelocities != 0;
    out.mDrawSoftBodyEdgeConstraints = in->drawSoftBodyEdgeConstraints != 0;
    out.mDrawSoftBodyBendConstraints = in->drawSoftBodyBendConstraints != 0;
    out.mDrawSoftBodyVolumeConstraints = in->drawSoftBodyVolumeConstraints != 0;
    out.mDrawSoftBodySkinConstraints = in->drawSoftBodySkinConstraints != 0;
    out.mDrawSoftBodyLRAConstraints = in->drawSoftBodyLRAConstraints != 0;
    out.mDrawSoftBodyRods = in->drawSoftBodyRods != 0;
    out.mDrawSoftBodyRodStates = in->drawSoftBodyRodStates != 0;
    out.mDrawSoftBodyRodBendTwistConstraints = in->drawSoftBodyRodBendTwistConstraints != 0;
    out.mDrawSoftBodyPredictedBounds = in->drawSoftBodyPredictedBounds != 0;
}

JOLTC_API void JoltC_PhysicsSystem_DrawBodies(JoltC_PhysicsSystem* system, const JoltC_BodyDrawSettings* settings, JoltC_DebugRenderer* renderer)
{
    if (!system || !system->ptr || !renderer) return;
    JOLTC_TRY_BEGIN
    BodyManager::DrawSettings drawSettings;
    fillDrawSettings(drawSettings, settings);
    system->ptr->DrawBodies(drawSettings, asRenderer(renderer));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_PhysicsSystem_DrawConstraints(JoltC_PhysicsSystem* system, JoltC_DebugRenderer* renderer)
{
    if (!system || !system->ptr || !renderer) return;
    JOLTC_TRY_BEGIN
    system->ptr->DrawConstraints(asRenderer(renderer));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_PhysicsSystem_DrawConstraintLimits(JoltC_PhysicsSystem* system, JoltC_DebugRenderer* renderer)
{
    if (!system || !system->ptr || !renderer) return;
    JOLTC_TRY_BEGIN
    system->ptr->DrawConstraintLimits(asRenderer(renderer));
    JOLTC_TRY_END
}

} /* extern "C" */
