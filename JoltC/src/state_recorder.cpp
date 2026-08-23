/* JoltC - Determinism and state: StateRecorderImpl, system snapshots, per-object SaveState
 * SPDX-License-Identifier: MIT
 */

#include <Jolt/Jolt.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Constraints/Constraint.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Character/Character.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>

#include <JoltC/state_recorder.h>
#include "internal.h"
#include "errors_internal.h"
#include "wrappers.h"

#include <cstring>

using namespace JPH;

/* The handle is a reinterpret_cast of StateRecorderImpl. Everything that only needs the
 * StateRecorder base still goes through the same handle: the only recorder this API creates is
 * the in-memory one. */
static inline StateRecorderImpl* asRecorder(JoltC_StateRecorder* h) { return reinterpret_cast<StateRecorderImpl*>(h); }
static inline const StateRecorderImpl* asRecorder(const JoltC_StateRecorder* h) { return reinterpret_cast<const StateRecorderImpl*>(h); }

extern "C" {

/* -------------------------------------------------------------------------- */
/*  StateRecorderImpl                                                         */
/* -------------------------------------------------------------------------- */
JOLTC_API JoltC_StateRecorder* JoltC_StateRecorderImpl_Create(void)
{
    JOLTC_TRY_BEGIN
    return reinterpret_cast<JoltC_StateRecorder*>(new StateRecorderImpl());
    JOLTC_TRY_END
    return nullptr;
}

JOLTC_API void JoltC_StateRecorderImpl_Destroy(JoltC_StateRecorder* recorder)
{
    delete asRecorder(recorder);
}

JOLTC_API void JoltC_StateRecorderImpl_Rewind(JoltC_StateRecorder* recorder)
{
    if (!recorder) return;
    JOLTC_TRY_BEGIN
    asRecorder(recorder)->Rewind();
    JOLTC_TRY_END
}

JOLTC_API void JoltC_StateRecorderImpl_Clear(JoltC_StateRecorder* recorder)
{
    if (!recorder) return;
    JOLTC_TRY_BEGIN
    asRecorder(recorder)->Clear();
    JOLTC_TRY_END
}

JOLTC_API uint64_t JoltC_StateRecorderImpl_GetDataSize(const JoltC_StateRecorder* recorder)
{
    if (!recorder) return 0;
    JOLTC_TRY_BEGIN
    /* tellp is not const on the underlying stream, harmless here. */
    return (uint64_t)const_cast<StateRecorderImpl*>(asRecorder(recorder))->GetDataSize();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API uint64_t JoltC_StateRecorderImpl_CopyData(const JoltC_StateRecorder* recorder, uint8_t* buffer, uint64_t capacity)
{
    if (!recorder) return 0;
    JOLTC_TRY_BEGIN
    std::string data = asRecorder(recorder)->GetData();
    if (buffer && capacity > 0)
    {
        uint64_t toCopy = data.size() < capacity ? (uint64_t)data.size() : capacity;
        std::memcpy(buffer, data.data(), (size_t)toCopy);
    }
    return (uint64_t)data.size();
    JOLTC_TRY_END
    return 0;
}

JOLTC_API void JoltC_StateRecorderImpl_SetData(JoltC_StateRecorder* recorder, const uint8_t* data, uint64_t size)
{
    if (!recorder || (!data && size > 0)) return;
    JOLTC_TRY_BEGIN
    StateRecorderImpl* impl = asRecorder(recorder);
    impl->Clear();
    if (size > 0)
        impl->WriteBytes(data, (size_t)size);
    JOLTC_TRY_END
}

JOLTC_API JoltC_Bool JoltC_StateRecorder_IsEOF(const JoltC_StateRecorder* recorder)
{
    if (!recorder) return JOLTC_TRUE;
    JOLTC_TRY_BEGIN
    return asRecorder(recorder)->IsEOF() ? JOLTC_TRUE : JOLTC_FALSE;
    JOLTC_TRY_END
    return JOLTC_TRUE;
}

JOLTC_API void JoltC_StateRecorder_SetValidating(JoltC_StateRecorder* recorder, JoltC_Bool validating)
{
    if (!recorder) return;
    asRecorder(recorder)->SetValidating(validating != 0);
}

JOLTC_API JoltC_Bool JoltC_StateRecorder_IsValidating(const JoltC_StateRecorder* recorder)
{
    if (!recorder) return JOLTC_FALSE;
    return asRecorder(recorder)->IsValidating() ? JOLTC_TRUE : JOLTC_FALSE;
}

/* -------------------------------------------------------------------------- */
/*  Whole-system state                                                        */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_PhysicsSystem_SaveState(const JoltC_PhysicsSystem* system, JoltC_StateRecorder* recorder, uint32_t stateFlags)
{
    if (!system || !system->ptr || !recorder) return;
    JOLTC_TRY_BEGIN
    system->ptr->SaveState(*asRecorder(recorder), static_cast<EStateRecorderState>(stateFlags));
    JOLTC_TRY_END
}

JOLTC_API JoltC_Bool JoltC_PhysicsSystem_RestoreState(JoltC_PhysicsSystem* system, JoltC_StateRecorder* recorder)
{
    if (!system || !system->ptr || !recorder) return JOLTC_FALSE;
    JOLTC_TRY_BEGIN
    return system->ptr->RestoreState(*asRecorder(recorder)) ? JOLTC_TRUE : JOLTC_FALSE;
    JOLTC_TRY_END
    return JOLTC_FALSE;
}

JOLTC_API void JoltC_PhysicsSystem_SaveBodyState(const JoltC_PhysicsSystem* system, const JoltC_Body* body, JoltC_StateRecorder* recorder)
{
    if (!system || !system->ptr || !body || !recorder) return;
    JOLTC_TRY_BEGIN
    system->ptr->SaveBodyState(*reinterpret_cast<const Body*>(body), *asRecorder(recorder));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_PhysicsSystem_RestoreBodyState(JoltC_PhysicsSystem* system, JoltC_Body* body, JoltC_StateRecorder* recorder)
{
    if (!system || !system->ptr || !body || !recorder) return;
    JOLTC_TRY_BEGIN
    system->ptr->RestoreBodyState(*reinterpret_cast<Body*>(body), *asRecorder(recorder));
    JOLTC_TRY_END
}

/* -------------------------------------------------------------------------- */
/*  Per-object state                                                          */
/* -------------------------------------------------------------------------- */
JOLTC_API void JoltC_Constraint_SaveState(const JoltC_Constraint* constraint, JoltC_StateRecorder* recorder)
{
    if (!constraint || !constraint->ptr || !recorder) return;
    JOLTC_TRY_BEGIN
    constraint->ptr->SaveState(*asRecorder(recorder));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_Constraint_RestoreState(JoltC_Constraint* constraint, JoltC_StateRecorder* recorder)
{
    if (!constraint || !constraint->ptr || !recorder) return;
    JOLTC_TRY_BEGIN
    constraint->ptr->RestoreState(*asRecorder(recorder));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_CharacterVirtual_SaveState(const JoltC_CharacterVirtual* character, JoltC_StateRecorder* recorder)
{
    if (!character || !character->ptr || !recorder) return;
    JOLTC_TRY_BEGIN
    character->ptr->SaveState(*asRecorder(recorder));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_CharacterVirtual_RestoreState(JoltC_CharacterVirtual* character, JoltC_StateRecorder* recorder)
{
    if (!character || !character->ptr || !recorder) return;
    JOLTC_TRY_BEGIN
    character->ptr->RestoreState(*asRecorder(recorder));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_Character_SaveState(const JoltC_Character* character, JoltC_StateRecorder* recorder)
{
    if (!character || !character->ptr || !recorder) return;
    JOLTC_TRY_BEGIN
    character->ptr->SaveState(*asRecorder(recorder));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_Character_RestoreState(JoltC_Character* character, JoltC_StateRecorder* recorder)
{
    if (!character || !character->ptr || !recorder) return;
    JOLTC_TRY_BEGIN
    character->ptr->RestoreState(*asRecorder(recorder));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_SoftBodyMotionProperties_SaveState(const JoltC_SoftBodyMotionProperties* motionProperties, JoltC_StateRecorder* recorder)
{
    if (!motionProperties || !recorder) return;
    JOLTC_TRY_BEGIN
    reinterpret_cast<const SoftBodyMotionProperties*>(motionProperties)->SaveState(*asRecorder(recorder));
    JOLTC_TRY_END
}

JOLTC_API void JoltC_SoftBodyMotionProperties_RestoreState(JoltC_SoftBodyMotionProperties* motionProperties, JoltC_StateRecorder* recorder)
{
    if (!motionProperties || !recorder) return;
    JOLTC_TRY_BEGIN
    reinterpret_cast<SoftBodyMotionProperties*>(motionProperties)->RestoreState(*asRecorder(recorder));
    JOLTC_TRY_END
}

} /* extern "C" */
