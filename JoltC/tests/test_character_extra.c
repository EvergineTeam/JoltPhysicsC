/* JoltC Test Suite — character.h, second suite (settings round-trips, lifetime,
 * null safety, structural queries)
 * SPDX-License-Identifier: MIT
 *
 * Why this file exists, and what it deliberately does NOT do
 * ----------------------------------------------------------
 * character.h is the least covered header in the wrapper, and CharacterVirtual is
 * one of the headers upstream JoltPhysics rewrites most between releases. A hand
 * repair of character.cpp during a Jolt bump is therefore likely, and the failure
 * that costs the most is one that compiles but is semantically wrong: a swapped
 * assignment in a settings struct, a mixed-up argument order, a getter reading the
 * wrong field.
 *
 * So every test here is chosen to be immune to solver behaviour:
 *   - settings in, accessor out, with asymmetric values so a swap cannot pass;
 *   - handle lifetime and destruction order;
 *   - the null guards that the implementation actually has;
 *   - structural queries (enum ranges, layers, shape identity, transform/accessor
 *     agreement) rather than trajectories.
 *
 * What is intentionally NOT asserted, and must stay that way: an exact position,
 * an exact velocity, or a specific ground-state classification after stepping.
 * Jolt changes character slope handling, sliding-along-walls behaviour and the
 * friction model between releases; a trajectory assertion here would fail
 * legitimately on such a bump and read as a regression. Direction and sign only
 * ("y decreased under gravity"), never magnitude.
 *
 * test_character.c already covers: CharacterVirtual create/destroy, position
 * get/set, linear velocity get/set, initial ground state, user data and one
 * ExtendedUpdate call. None of that is repeated here.
 */

#include "test_common.h"

void run_character_extra_tests(void);

/* ========================================================================== */
/*  Contact listener plumbing used by the listener test                       */
/* ========================================================================== */
typedef struct ListenerCounters {
    int validateCalls;
    int addedCalls;
    int persistedCalls;
    int wrongUserData;   /* set if a callback received a userData we never passed */
    int nullOutPointer;  /* set if the wrapper handed us a NULL out-parameter */
} ListenerCounters;

static ListenerCounters g_listenerCounters;

static void on_validate_cb(void* userData, JoltC_BodyID bodyID2, JoltC_Bool* outAccept)
{
    (void)bodyID2;
    if (userData != (void*)&g_listenerCounters) { g_listenerCounters.wrongUserData = 1; return; }
    if (outAccept == NULL) { g_listenerCounters.nullOutPointer = 1; return; }
    g_listenerCounters.validateCalls++;
    *outAccept = JOLTC_TRUE;
}

static void on_added_cb(void* userData, JoltC_BodyID bodyID2, JoltC_RVec3 contactPosition,
                        JoltC_Vec3 contactNormal, JoltC_Bool* outCanPushCharacter,
                        JoltC_Bool* outCanReceiveImpulses)
{
    (void)bodyID2; (void)contactPosition; (void)contactNormal;
    if (userData != (void*)&g_listenerCounters) { g_listenerCounters.wrongUserData = 1; return; }
    if (outCanPushCharacter == NULL || outCanReceiveImpulses == NULL) {
        g_listenerCounters.nullOutPointer = 1;
        return;
    }
    g_listenerCounters.addedCalls++;
}

static void on_persisted_cb(void* userData, JoltC_BodyID bodyID2, JoltC_RVec3 contactPosition,
                            JoltC_Vec3 contactNormal, JoltC_Bool* outCanPushCharacter,
                            JoltC_Bool* outCanReceiveImpulses)
{
    (void)bodyID2; (void)contactPosition; (void)contactNormal;
    if (userData != (void*)&g_listenerCounters) { g_listenerCounters.wrongUserData = 1; return; }
    if (outCanPushCharacter == NULL || outCanReceiveImpulses == NULL) {
        g_listenerCounters.nullOutPointer = 1;
        return;
    }
    g_listenerCounters.persistedCalls++;
}

/* ========================================================================== */
/*  Local helpers                                                             */
/* ========================================================================== */
#define IDENTITY_QUAT_INIT { 0.0f, 0.0f, 0.0f, 1.0f }

static int is_valid_ground_state(int gs)
{
    return gs == JOLTC_GROUND_STATE_ON_GROUND
        || gs == JOLTC_GROUND_STATE_ON_STEEP_GROUND
        || gs == JOLTC_GROUND_STATE_NOT_SUPPORTED
        || gs == JOLTC_GROUND_STATE_IN_AIR;
}

static int is_finite_vec3(JoltC_Vec3 v)
{
    return (v.x == v.x) && (v.y == v.y) && (v.z == v.z);   /* NaN-free */
}

/* ========================================================================== */
void run_character_extra_tests(void)
{
    /* ====================================================================== */
    /*  1. Settings defaults                                                  */
    /* ====================================================================== */

    /* The default values live in the wrapper itself (character.cpp), not in Jolt,
     * so they are a fair thing to pin exactly. Distinct numbers such as mass 70 vs
     * maxStrength 100 are the point: if the two assignments are ever swapped, this
     * test is what notices. */
    TEST_BEGIN("CharacterVirtualSettings_SetDefault values");
    {
        JoltC_CharacterVirtualSettings s;
        JoltC_CharacterVirtualSettings_SetDefault(&s);

        TEST_ASSERT_FLOAT_EQ(s.up.x, 0.0f, 1e-6f, "default up.x == 0");
        TEST_ASSERT_FLOAT_EQ(s.up.y, 1.0f, 1e-6f, "default up.y == 1");
        TEST_ASSERT_FLOAT_EQ(s.up.z, 0.0f, 1e-6f, "default up.z == 0");
        /* maxSlopeAngle is in RADIANS — 50 degrees. A degrees/radians mix-up in a
         * repair would show up right here. */
        TEST_ASSERT_FLOAT_EQ(s.maxSlopeAngle, 0.87266463f, 1e-5f, "default maxSlopeAngle == 50 deg in radians");
        TEST_ASSERT(s.enhancedInternalEdgeRemoval == JOLTC_FALSE, "default enhancedInternalEdgeRemoval false");
        TEST_ASSERT(s.shape == NULL, "default shape NULL");
        TEST_ASSERT_FLOAT_EQ(s.mass, 70.0f, 1e-4f, "default mass == 70");
        TEST_ASSERT_FLOAT_EQ(s.maxStrength, 100.0f, 1e-4f, "default maxStrength == 100");
        TEST_ASSERT_FLOAT_EQ(s.shapeOffset.x, 0.0f, 1e-6f, "default shapeOffset.x == 0");
        TEST_ASSERT_FLOAT_EQ(s.shapeOffset.y, 0.0f, 1e-6f, "default shapeOffset.y == 0");
        TEST_ASSERT_FLOAT_EQ(s.shapeOffset.z, 0.0f, 1e-6f, "default shapeOffset.z == 0");
        TEST_ASSERT(s.backFaceMode == JOLTC_BACK_FACE_COLLIDE, "default backFaceMode collide");
        TEST_ASSERT_FLOAT_EQ(s.predictiveContactDistance, 0.1f, 1e-6f, "default predictiveContactDistance == 0.1");
        TEST_ASSERT(s.maxCollisionIterations == 5, "default maxCollisionIterations == 5");
        TEST_ASSERT(s.maxConstraintIterations == 15, "default maxConstraintIterations == 15");
        TEST_ASSERT(s.minTimeRemaining > 0.0f && s.minTimeRemaining < 1.0f, "default minTimeRemaining in (0,1)");
        TEST_ASSERT(s.collisionTolerance > 0.0f && s.collisionTolerance < 1.0f, "default collisionTolerance in (0,1)");
        TEST_ASSERT_FLOAT_EQ(s.characterPadding, 0.02f, 1e-6f, "default characterPadding == 0.02");
        TEST_ASSERT(s.maxNumHits == 256, "default maxNumHits == 256");
        TEST_ASSERT(s.hitReductionCosMaxAngle > 0.0f && s.hitReductionCosMaxAngle <= 1.0f,
                    "default hitReductionCosMaxAngle is a cosine");
        TEST_ASSERT_FLOAT_EQ(s.penetrationRecoverySpeed, 1.0f, 1e-6f, "default penetrationRecoverySpeed == 1");
        TEST_ASSERT(s.innerBodyShape == NULL, "default innerBodyShape NULL");
        TEST_ASSERT(s.innerBodyLayer == 0, "default innerBodyLayer == 0");
    }
    TEST_END();

    /* _Init is documented as an alias of _SetDefault. Compared field by field
     * rather than with memcmp, because both structs contain padding. */
    TEST_BEGIN("CharacterVirtualSettings_Init matches SetDefault");
    {
        JoltC_CharacterVirtualSettings a;
        JoltC_CharacterVirtualSettings b;
        JoltC_CharacterVirtualSettings_SetDefault(&a);
        JoltC_CharacterVirtualSettings_Init(&b);

        TEST_ASSERT_FLOAT_EQ(a.up.y, b.up.y, 1e-6f, "Init up matches");
        TEST_ASSERT_FLOAT_EQ(a.maxSlopeAngle, b.maxSlopeAngle, 1e-6f, "Init maxSlopeAngle matches");
        TEST_ASSERT(a.enhancedInternalEdgeRemoval == b.enhancedInternalEdgeRemoval, "Init eier matches");
        TEST_ASSERT(a.shape == b.shape, "Init shape matches");
        TEST_ASSERT_FLOAT_EQ(a.mass, b.mass, 1e-6f, "Init mass matches");
        TEST_ASSERT_FLOAT_EQ(a.maxStrength, b.maxStrength, 1e-6f, "Init maxStrength matches");
        TEST_ASSERT(a.backFaceMode == b.backFaceMode, "Init backFaceMode matches");
        TEST_ASSERT_FLOAT_EQ(a.predictiveContactDistance, b.predictiveContactDistance, 1e-6f,
                             "Init predictiveContactDistance matches");
        TEST_ASSERT(a.maxCollisionIterations == b.maxCollisionIterations, "Init maxCollisionIterations matches");
        TEST_ASSERT(a.maxConstraintIterations == b.maxConstraintIterations, "Init maxConstraintIterations matches");
        TEST_ASSERT_FLOAT_EQ(a.minTimeRemaining, b.minTimeRemaining, 1e-9f, "Init minTimeRemaining matches");
        TEST_ASSERT_FLOAT_EQ(a.collisionTolerance, b.collisionTolerance, 1e-9f, "Init collisionTolerance matches");
        TEST_ASSERT_FLOAT_EQ(a.characterPadding, b.characterPadding, 1e-6f, "Init characterPadding matches");
        TEST_ASSERT(a.maxNumHits == b.maxNumHits, "Init maxNumHits matches");
        TEST_ASSERT_FLOAT_EQ(a.hitReductionCosMaxAngle, b.hitReductionCosMaxAngle, 1e-6f,
                             "Init hitReductionCosMaxAngle matches");
        TEST_ASSERT_FLOAT_EQ(a.penetrationRecoverySpeed, b.penetrationRecoverySpeed, 1e-6f,
                             "Init penetrationRecoverySpeed matches");
        TEST_ASSERT(a.innerBodyShape == b.innerBodyShape, "Init innerBodyShape matches");
        TEST_ASSERT(a.innerBodyLayer == b.innerBodyLayer, "Init innerBodyLayer matches");
    }
    TEST_END();

    TEST_BEGIN("CharacterSettings_SetDefault / _Init values");
    {
        JoltC_CharacterSettings a;
        JoltC_CharacterSettings b;
        JoltC_CharacterSettings_SetDefault(&a);
        JoltC_CharacterSettings_Init(&b);

        TEST_ASSERT_FLOAT_EQ(a.up.x, 0.0f, 1e-6f, "default up.x == 0");
        TEST_ASSERT_FLOAT_EQ(a.up.y, 1.0f, 1e-6f, "default up.y == 1");
        TEST_ASSERT_FLOAT_EQ(a.up.z, 0.0f, 1e-6f, "default up.z == 0");
        TEST_ASSERT_FLOAT_EQ(a.maxSlopeAngle, 0.87266463f, 1e-5f, "default maxSlopeAngle == 50 deg in radians");
        TEST_ASSERT(a.enhancedInternalEdgeRemoval == JOLTC_FALSE, "default enhancedInternalEdgeRemoval false");
        TEST_ASSERT(a.shape == NULL, "default shape NULL");
        TEST_ASSERT(a.layer == 0, "default layer == 0");
        /* Character defaults to mass 80 while CharacterVirtual defaults to 70 —
         * distinct on purpose, so a copy/paste between the two is visible. */
        TEST_ASSERT_FLOAT_EQ(a.mass, 80.0f, 1e-4f, "default mass == 80");
        TEST_ASSERT_FLOAT_EQ(a.friction, 0.2f, 1e-6f, "default friction == 0.2");
        TEST_ASSERT_FLOAT_EQ(a.gravityFactor, 1.0f, 1e-6f, "default gravityFactor == 1");

        TEST_ASSERT_FLOAT_EQ(a.maxSlopeAngle, b.maxSlopeAngle, 1e-6f, "Init maxSlopeAngle matches");
        TEST_ASSERT(a.layer == b.layer, "Init layer matches");
        TEST_ASSERT_FLOAT_EQ(a.mass, b.mass, 1e-6f, "Init mass matches");
        TEST_ASSERT_FLOAT_EQ(a.friction, b.friction, 1e-6f, "Init friction matches");
        TEST_ASSERT_FLOAT_EQ(a.gravityFactor, b.gravityFactor, 1e-6f, "Init gravityFactor matches");
    }
    TEST_END();

    TEST_BEGIN("ExtendedUpdateSettings_SetDefault values");
    {
        JoltC_ExtendedUpdateSettings s;
        JoltC_ExtendedUpdateSettings_SetDefault(&s);

        /* Signs carry the meaning here: step DOWN must be along -up, step UP along
         * +up. A swap of these two vectors is the classic repair accident and it
         * would make a character sink through stairs instead of climbing them. */
        TEST_ASSERT(s.stickToFloorStepDown.y < 0.0f, "stickToFloorStepDown points down");
        TEST_ASSERT_FLOAT_EQ(s.stickToFloorStepDown.x, 0.0f, 1e-6f, "stickToFloorStepDown.x == 0");
        TEST_ASSERT_FLOAT_EQ(s.stickToFloorStepDown.z, 0.0f, 1e-6f, "stickToFloorStepDown.z == 0");
        TEST_ASSERT(s.walkStairsStepUp.y > 0.0f, "walkStairsStepUp points up");
        TEST_ASSERT_FLOAT_EQ(s.walkStairsStepUp.x, 0.0f, 1e-6f, "walkStairsStepUp.x == 0");
        TEST_ASSERT_FLOAT_EQ(s.walkStairsStepUp.z, 0.0f, 1e-6f, "walkStairsStepUp.z == 0");
        TEST_ASSERT(s.walkStairsMinStepForward > 0.0f, "walkStairsMinStepForward positive");
        TEST_ASSERT(s.walkStairsStepForwardTest > s.walkStairsMinStepForward,
                    "walkStairsStepForwardTest exceeds min step forward");
        TEST_ASSERT(s.walkStairsCosAngleForwardContact > 0.0f && s.walkStairsCosAngleForwardContact < 1.0f,
                    "walkStairsCosAngleForwardContact is a cosine");
        TEST_ASSERT_FLOAT_EQ(s.walkStairsStepDownExtra.x, 0.0f, 1e-6f, "walkStairsStepDownExtra.x == 0");
        TEST_ASSERT_FLOAT_EQ(s.walkStairsStepDownExtra.y, 0.0f, 1e-6f, "walkStairsStepDownExtra.y == 0");
        TEST_ASSERT_FLOAT_EQ(s.walkStairsStepDownExtra.z, 0.0f, 1e-6f, "walkStairsStepDownExtra.z == 0");
    }
    TEST_END();

    TEST_BEGIN("Settings _SetDefault NULL safe");
    {
        JoltC_CharacterVirtualSettings_SetDefault(NULL);
        JoltC_CharacterVirtualSettings_Init(NULL);
        JoltC_CharacterSettings_SetDefault(NULL);
        JoltC_CharacterSettings_Init(NULL);
        JoltC_ExtendedUpdateSettings_SetDefault(NULL);
        TEST_ASSERT(1, "NULL settings pointers ignored");
    }
    TEST_END();

    /* ====================================================================== */
    /*  2. Creation honours every observable settings field                   */
    /* ====================================================================== */

    /* The flagship test of this file. Every field that has a matching accessor is
     * given a distinctive, asymmetric value and read back through the created
     * character. Nothing here depends on the solver. */
    TEST_BEGIN("CharacterVirtual_Create honours settings fields");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.65f, 0.3f);
        const JoltC_Shape* innerSphere = JoltC_SphereShape_Create(0.4f);
        TEST_ASSERT_NOT_NULL(capsule, "capsule shape not null");
        TEST_ASSERT_NOT_NULL(innerSphere, "inner sphere shape not null");

        JoltC_CharacterVirtualSettings s;
        JoltC_CharacterVirtualSettings_SetDefault(&s);
        s.up.x = 0.6f;  s.up.y = 0.8f;  s.up.z = 0.0f;   /* normalized but asymmetric */
        s.maxSlopeAngle = 0.6f;                          /* radians */
        s.enhancedInternalEdgeRemoval = JOLTC_TRUE;
        s.shape = capsule;
        s.mass = 55.5f;
        s.maxStrength = 123.25f;
        s.shapeOffset.x = 1.5f; s.shapeOffset.y = -2.25f; s.shapeOffset.z = 3.75f;
        s.backFaceMode = JOLTC_BACK_FACE_IGNORE;         /* no accessor — see report */
        s.predictiveContactDistance = 0.075f;            /* no accessor */
        s.maxCollisionIterations = 7;                    /* no accessor */
        s.maxConstraintIterations = 11;                  /* no accessor */
        s.minTimeRemaining = 2.5e-4f;                    /* no accessor */
        s.collisionTolerance = 2.0e-3f;                  /* no accessor */
        s.characterPadding = 0.035f;
        s.maxNumHits = 37;
        s.hitReductionCosMaxAngle = 0.875f;
        s.penetrationRecoverySpeed = 0.375f;
        s.innerBodyShape = innerSphere;
        s.innerBodyLayer = OBJ_LAYER_DYNAMIC;

        JoltC_RVec3 pos = { 1.5f, -2.25f, 3.75f };
        JoltC_Quat rot = IDENTITY_QUAT_INIT;
        JoltC_CharacterVirtual* cv = JoltC_CharacterVirtual_Create(&s, pos, rot, 0x1234ABCDu, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(cv, "CharacterVirtual created");

        JoltC_Vec3 up = JoltC_CharacterVirtual_GetUp(cv);
        TEST_ASSERT_FLOAT_EQ(up.x, 0.6f, 1e-5f, "up.x round-trips");
        TEST_ASSERT_FLOAT_EQ(up.y, 0.8f, 1e-5f, "up.y round-trips");
        TEST_ASSERT_FLOAT_EQ(up.z, 0.0f, 1e-5f, "up.z round-trips");

        /* maxSlopeAngle is stored as its cosine — this is the accessor that proves
         * the angle was taken as radians and not degrees. */
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetCosMaxSlopeAngle(cv), cosf(0.6f), 1e-5f,
                             "cos(maxSlopeAngle) round-trips");

        TEST_ASSERT(JoltC_CharacterVirtual_GetEnhancedInternalEdgeRemoval(cv) != JOLTC_FALSE,
                    "enhancedInternalEdgeRemoval round-trips");
        TEST_ASSERT(JoltC_CharacterVirtual_GetShape(cv) == capsule, "shape round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetMass(cv), 55.5f, 1e-4f, "mass round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetMaxStrength(cv), 123.25f, 1e-4f, "maxStrength round-trips");

        JoltC_Vec3 offset = JoltC_CharacterVirtual_GetShapeOffset(cv);
        TEST_ASSERT_FLOAT_EQ(offset.x, 1.5f, 1e-5f, "shapeOffset.x round-trips");
        TEST_ASSERT_FLOAT_EQ(offset.y, -2.25f, 1e-5f, "shapeOffset.y round-trips");
        TEST_ASSERT_FLOAT_EQ(offset.z, 3.75f, 1e-5f, "shapeOffset.z round-trips");

        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetCharacterPadding(cv), 0.035f, 1e-6f,
                             "characterPadding round-trips");
        TEST_ASSERT(JoltC_CharacterVirtual_GetMaxNumHits(cv) == 37, "maxNumHits round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetHitReductionCosMaxAngle(cv), 0.875f, 1e-6f,
                             "hitReductionCosMaxAngle round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetPenetrationRecoverySpeed(cv), 0.375f, 1e-6f,
                             "penetrationRecoverySpeed round-trips");

        /* position and user data were passed as separate arguments, not in the
         * settings struct — check the argument order too. */
        JoltC_RVec3 gotPos = JoltC_CharacterVirtual_GetPosition(cv);
        TEST_ASSERT_FLOAT_EQ(gotPos.x, 1.5f, 1e-4f, "position.x from create argument");
        TEST_ASSERT_FLOAT_EQ(gotPos.y, -2.25f, 1e-4f, "position.y from create argument");
        TEST_ASSERT_FLOAT_EQ(gotPos.z, 3.75f, 1e-4f, "position.z from create argument");

        /* innerBodyShape produced a real body, and innerBodyLayer reached it. This
         * is the only way innerBodyLayer is observable through the C API. */
        JoltC_BodyID innerID = JoltC_CharacterVirtual_GetInnerBodyID(cv);
        TEST_ASSERT(innerID != JOLTC_BODY_ID_INVALID, "innerBodyShape produced an inner body");
        TEST_ASSERT(JoltC_BodyInterface_GetObjectLayer(ctx.bodyInterface, innerID) == OBJ_LAYER_DYNAMIC,
                    "innerBodyLayer reached the inner body");

        /* SetInnerBodyShape only makes sense once an inner body exists (see report). */
        const JoltC_Shape* replacement = JoltC_SphereShape_Create(0.2f);
        JoltC_CharacterVirtual_SetInnerBodyShape(cv, replacement);
        TEST_ASSERT(JoltC_CharacterVirtual_GetInnerBodyID(cv) == innerID,
                    "inner body ID unchanged by SetInnerBodyShape");
        const JoltC_Shape* innerNow = JoltC_BodyInterface_GetShape(ctx.bodyInterface, innerID);
        TEST_ASSERT(innerNow == replacement, "SetInnerBodyShape replaced the inner body shape");
        if (innerNow) JoltC_Shape_Release(innerNow); /* GetShape hands back a reference */

        JoltC_CharacterVirtual_Destroy(cv);
        JoltC_Shape_Release(replacement);
        JoltC_Shape_Release(innerSphere);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    TEST_BEGIN("CharacterVirtual without inner body has invalid inner body ID");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        JoltC_CharacterVirtualSettings s;
        JoltC_CharacterVirtualSettings_SetDefault(&s);
        s.shape = capsule;

        JoltC_RVec3 pos = { 0.0f, 4.5f, 0.0f };
        JoltC_Quat rot = IDENTITY_QUAT_INIT;
        JoltC_CharacterVirtual* cv = JoltC_CharacterVirtual_Create(&s, pos, rot, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(cv, "CharacterVirtual created");
        TEST_ASSERT(JoltC_CharacterVirtual_GetInnerBodyID(cv) == JOLTC_BODY_ID_INVALID,
                    "no innerBodyShape means no inner body");

        JoltC_CharacterVirtual_Destroy(cv);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    TEST_BEGIN("CharacterVirtual_Create rejects incomplete arguments");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_RVec3 pos = { 0.0f, 1.0f, 0.0f };
        JoltC_Quat rot = IDENTITY_QUAT_INIT;

        JoltC_CharacterVirtualSettings s;
        JoltC_CharacterVirtualSettings_SetDefault(&s);   /* shape is NULL by default */
        TEST_ASSERT(JoltC_CharacterVirtual_Create(&s, pos, rot, 0, ctx.physicsSystem) == NULL,
                    "create without a shape returns NULL");
        TEST_ASSERT(JoltC_CharacterVirtual_Create(NULL, pos, rot, 0, ctx.physicsSystem) == NULL,
                    "create without settings returns NULL");

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        s.shape = capsule;
        TEST_ASSERT(JoltC_CharacterVirtual_Create(&s, pos, rot, 0, NULL) == NULL,
                    "create without a physics system returns NULL");

        JoltC_CharacterVirtual_Destroy(NULL);            /* documented no-op */
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* ====================================================================== */
    /*  3. Setter round-trips on a live CharacterVirtual                      */
    /* ====================================================================== */
    TEST_BEGIN("CharacterVirtual setter round-trips");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        JoltC_CharacterVirtualSettings s;
        JoltC_CharacterVirtualSettings_SetDefault(&s);
        s.shape = capsule;

        JoltC_RVec3 pos = { 0.0f, 5.0f, 0.0f };
        JoltC_Quat rot = IDENTITY_QUAT_INIT;
        JoltC_CharacterVirtual* cv = JoltC_CharacterVirtual_Create(&s, pos, rot, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(cv, "CharacterVirtual created");

        /* Every value differs from both the default and the other fields, so a
         * setter wired to the wrong member cannot pass. */
        JoltC_CharacterVirtual_SetMass(cv, 42.125f);
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetMass(cv), 42.125f, 1e-4f, "SetMass round-trips");

        JoltC_CharacterVirtual_SetMaxStrength(cv, 88.75f);
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetMaxStrength(cv), 88.75f, 1e-4f, "SetMaxStrength round-trips");
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetMass(cv), 42.125f, 1e-4f, "SetMaxStrength left mass alone");

        JoltC_CharacterVirtual_SetPenetrationRecoverySpeed(cv, 0.625f);
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetPenetrationRecoverySpeed(cv), 0.625f, 1e-6f,
                             "SetPenetrationRecoverySpeed round-trips");

        JoltC_CharacterVirtual_SetMaxNumHits(cv, 19);
        TEST_ASSERT(JoltC_CharacterVirtual_GetMaxNumHits(cv) == 19, "SetMaxNumHits round-trips");

        JoltC_CharacterVirtual_SetHitReductionCosMaxAngle(cv, 0.8125f);
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetHitReductionCosMaxAngle(cv), 0.8125f, 1e-6f,
                             "SetHitReductionCosMaxAngle round-trips");

        JoltC_Vec3 newOffset = { -1.5f, 2.25f, -3.75f };
        JoltC_CharacterVirtual_SetShapeOffset(cv, newOffset);
        JoltC_Vec3 gotOffset = JoltC_CharacterVirtual_GetShapeOffset(cv);
        TEST_ASSERT_FLOAT_EQ(gotOffset.x, -1.5f, 1e-5f, "SetShapeOffset x round-trips");
        TEST_ASSERT_FLOAT_EQ(gotOffset.y, 2.25f, 1e-5f, "SetShapeOffset y round-trips");
        TEST_ASSERT_FLOAT_EQ(gotOffset.z, -3.75f, 1e-5f, "SetShapeOffset z round-trips");

        JoltC_CharacterVirtual_SetEnhancedInternalEdgeRemoval(cv, JOLTC_TRUE);
        TEST_ASSERT(JoltC_CharacterVirtual_GetEnhancedInternalEdgeRemoval(cv) != JOLTC_FALSE,
                    "SetEnhancedInternalEdgeRemoval on");
        JoltC_CharacterVirtual_SetEnhancedInternalEdgeRemoval(cv, JOLTC_FALSE);
        TEST_ASSERT(JoltC_CharacterVirtual_GetEnhancedInternalEdgeRemoval(cv) == JOLTC_FALSE,
                    "SetEnhancedInternalEdgeRemoval off");

        JoltC_Vec3 newUp = { 0.0f, 0.0f, 1.0f };
        JoltC_CharacterVirtual_SetUp(cv, newUp);
        JoltC_Vec3 gotUp = JoltC_CharacterVirtual_GetUp(cv);
        TEST_ASSERT_FLOAT_EQ(gotUp.x, 0.0f, 1e-6f, "SetUp x round-trips");
        TEST_ASSERT_FLOAT_EQ(gotUp.y, 0.0f, 1e-6f, "SetUp y round-trips");
        TEST_ASSERT_FLOAT_EQ(gotUp.z, 1.0f, 1e-6f, "SetUp z round-trips");

        JoltC_CharacterVirtual_SetMaxSlopeAngle(cv, 0.9f);
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetCosMaxSlopeAngle(cv), cosf(0.9f), 1e-5f,
                             "SetMaxSlopeAngle stores the cosine of the radian angle");

        JoltC_CharacterVirtual_Destroy(cv);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* IsSlopeTooSteep is a definition, not a simulation: a surface whose normal is
     * the up vector is never too steep, and a vertical wall always is. Those two
     * ends stay true whatever upstream does to slope handling in between. */
    TEST_BEGIN("CharacterVirtual IsSlopeTooSteep at the extremes");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        JoltC_CharacterVirtualSettings s;
        JoltC_CharacterVirtualSettings_SetDefault(&s);
        s.shape = capsule;
        s.maxSlopeAngle = 0.7853982f;   /* 45 degrees in radians */

        JoltC_RVec3 pos = { 0.0f, 5.0f, 0.0f };
        JoltC_Quat rot = IDENTITY_QUAT_INIT;
        JoltC_CharacterVirtual* cv = JoltC_CharacterVirtual_Create(&s, pos, rot, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(cv, "CharacterVirtual created");

        JoltC_Vec3 flat = { 0.0f, 1.0f, 0.0f };
        JoltC_Vec3 wall = { 1.0f, 0.0f, 0.0f };
        TEST_ASSERT(JoltC_CharacterVirtual_IsSlopeTooSteep(cv, flat) == JOLTC_FALSE,
                    "a normal along up is not too steep");
        TEST_ASSERT(JoltC_CharacterVirtual_IsSlopeTooSteep(cv, wall) != JOLTC_FALSE,
                    "a vertical wall normal is too steep");

        JoltC_CharacterVirtual_Destroy(cv);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* ====================================================================== */
    /*  4. Transform accessors agree with the scalar accessors                */
    /* ====================================================================== */

    /* Compares two accessors against each other rather than against literals, so
     * it cannot go stale — but it still catches a row/column transposition in the
     * Mat44 conversion, which is exactly the kind of thing a hand repair breaks. */
    TEST_BEGIN("CharacterVirtual transforms agree with position/rotation");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        JoltC_CharacterVirtualSettings s;
        JoltC_CharacterVirtualSettings_SetDefault(&s);
        s.shape = capsule;

        JoltC_RVec3 pos = { 1.5f, -2.25f, 3.75f };
        JoltC_Quat rot = IDENTITY_QUAT_INIT;
        JoltC_CharacterVirtual* cv = JoltC_CharacterVirtual_Create(&s, pos, rot, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(cv, "CharacterVirtual created");

        /* Column-major: translation lives in m[12..14]. */
        JoltC_Mat44 world = JoltC_CharacterVirtual_GetWorldTransform(cv);
        TEST_ASSERT_FLOAT_EQ(world.m[12], 1.5f, 1e-4f, "world transform translation x");
        TEST_ASSERT_FLOAT_EQ(world.m[13], -2.25f, 1e-4f, "world transform translation y");
        TEST_ASSERT_FLOAT_EQ(world.m[14], 3.75f, 1e-4f, "world transform translation z");
        TEST_ASSERT_FLOAT_EQ(world.m[0], 1.0f, 1e-5f, "identity rotation leaves m[0] == 1");
        TEST_ASSERT_FLOAT_EQ(world.m[5], 1.0f, 1e-5f, "identity rotation leaves m[5] == 1");
        TEST_ASSERT_FLOAT_EQ(world.m[10], 1.0f, 1e-5f, "identity rotation leaves m[10] == 1");
        TEST_ASSERT_FLOAT_EQ(world.m[15], 1.0f, 1e-5f, "m[15] == 1");

        JoltC_Mat44 com = JoltC_CharacterVirtual_GetCenterOfMassTransform(cv);
        JoltC_RVec3 comPos = JoltC_CharacterVirtual_GetCenterOfMassPosition(cv);
        TEST_ASSERT_FLOAT_EQ(com.m[12], comPos.x, 1e-4f, "COM transform agrees with COM position x");
        TEST_ASSERT_FLOAT_EQ(com.m[13], comPos.y, 1e-4f, "COM transform agrees with COM position y");
        TEST_ASSERT_FLOAT_EQ(com.m[14], comPos.z, 1e-4f, "COM transform agrees with COM position z");

        /* 90 degrees about Y: (0, sin45, 0, cos45). */
        JoltC_Quat yaw = { 0.0f, 0.70710678f, 0.0f, 0.70710678f };
        JoltC_CharacterVirtual_SetRotation(cv, yaw);
        JoltC_Quat gotRot = JoltC_CharacterVirtual_GetRotation(cv);
        TEST_ASSERT_FLOAT_EQ(gotRot.x, 0.0f, 1e-5f, "rotation x round-trips");
        TEST_ASSERT_FLOAT_EQ(gotRot.y, 0.70710678f, 1e-5f, "rotation y round-trips");
        TEST_ASSERT_FLOAT_EQ(gotRot.z, 0.0f, 1e-5f, "rotation z round-trips");
        TEST_ASSERT_FLOAT_EQ(gotRot.w, 0.70710678f, 1e-5f, "rotation w round-trips");

        /* The translation column must be untouched by the rotation change. */
        world = JoltC_CharacterVirtual_GetWorldTransform(cv);
        JoltC_RVec3 gotPos = JoltC_CharacterVirtual_GetPosition(cv);
        TEST_ASSERT_FLOAT_EQ(world.m[12], gotPos.x, 1e-4f, "world transform still agrees on x");
        TEST_ASSERT_FLOAT_EQ(world.m[13], gotPos.y, 1e-4f, "world transform still agrees on y");
        TEST_ASSERT_FLOAT_EQ(world.m[14], gotPos.z, 1e-4f, "world transform still agrees on z");

        JoltC_CharacterVirtual_Destroy(cv);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* ====================================================================== */
    /*  5. SetShape                                                           */
    /* ====================================================================== */
    TEST_BEGIN("CharacterVirtual SetShape swaps the shape");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* big = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        const JoltC_Shape* small = JoltC_CapsuleShape_Create(0.35f, 0.15f);

        JoltC_CharacterVirtualSettings s;
        JoltC_CharacterVirtualSettings_SetDefault(&s);
        s.shape = big;

        JoltC_RVec3 pos = { 0.0f, 6.0f, 0.0f };   /* empty world, nothing to penetrate */
        JoltC_Quat rot = IDENTITY_QUAT_INIT;
        JoltC_CharacterVirtual* cv = JoltC_CharacterVirtual_Create(&s, pos, rot, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(cv, "CharacterVirtual created");
        TEST_ASSERT(JoltC_CharacterVirtual_GetShape(cv) == big, "starts with the big shape");

        /* Guards first: neither of these may touch the shape. */
        TEST_ASSERT(JoltC_CharacterVirtual_SetShape(cv, NULL, 1000.0f, ctx.tempAllocator) == JOLTC_FALSE,
                    "SetShape with NULL shape fails");
        TEST_ASSERT(JoltC_CharacterVirtual_SetShape(cv, small, 1000.0f, NULL) == JOLTC_FALSE,
                    "SetShape with NULL allocator fails");
        TEST_ASSERT(JoltC_CharacterVirtual_SetShape(NULL, small, 1000.0f, ctx.tempAllocator) == JOLTC_FALSE,
                    "SetShape with NULL character fails");
        TEST_ASSERT(JoltC_CharacterVirtual_GetShape(cv) == big, "rejected SetShape left the shape alone");

        /* Shrinking the shape in an otherwise empty world cannot introduce
         * penetration, so this must succeed in any Jolt version. */
        TEST_ASSERT(JoltC_CharacterVirtual_SetShape(cv, small, 1000.0f, ctx.tempAllocator) != JOLTC_FALSE,
                    "SetShape to a smaller shape succeeds");
        TEST_ASSERT(JoltC_CharacterVirtual_GetShape(cv) == small, "GetShape reports the new shape");

        JoltC_CharacterVirtual_Destroy(cv);
        JoltC_Shape_Release(small);
        JoltC_Shape_Release(big);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* ====================================================================== */
    /*  6. Structural queries in an empty world                               */
    /* ====================================================================== */

    /* With no bodies in the system there is nothing to touch, so every one of these
     * answers is forced by the absence of geometry rather than by the solver. */
    TEST_BEGIN("CharacterVirtual queries in an empty world");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        JoltC_CharacterVirtualSettings s;
        JoltC_CharacterVirtualSettings_SetDefault(&s);
        s.shape = capsule;

        JoltC_RVec3 pos = { 0.0f, 12.0f, 0.0f };
        JoltC_Quat rot = IDENTITY_QUAT_INIT;
        JoltC_CharacterVirtual* cv = JoltC_CharacterVirtual_Create(&s, pos, rot, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(cv, "CharacterVirtual created");

        JoltC_CharacterVirtual_RefreshContacts(cv, ctx.tempAllocator);
        TEST_ASSERT(JoltC_CharacterVirtual_GetNumActiveContacts(cv) == 0, "no contacts without bodies");
        TEST_ASSERT(JoltC_CharacterVirtual_IsSupported(cv) == JOLTC_FALSE, "not supported without bodies");
        TEST_ASSERT(JoltC_CharacterVirtual_GetGroundBodyID(cv) == JOLTC_BODY_ID_INVALID,
                    "no ground body without bodies");
        TEST_ASSERT(JoltC_CharacterVirtual_GetGroundUserData(cv) == 0, "no ground user data without bodies");
        TEST_ASSERT(JoltC_CharacterVirtual_GetMaxHitsExceeded(cv) == JOLTC_FALSE, "max hits not exceeded");
        TEST_ASSERT(is_valid_ground_state((int)JoltC_CharacterVirtual_GetGroundState(cv)),
                    "ground state is one of the four enum values");

        /* Reading these must not produce garbage even with no ground. */
        JoltC_RVec3 groundPos = JoltC_CharacterVirtual_GetGroundPosition(cv);
        JoltC_Vec3 groundNormal = JoltC_CharacterVirtual_GetGroundNormal(cv);
        JoltC_Vec3 groundVel = JoltC_CharacterVirtual_GetGroundVelocity(cv);
        TEST_ASSERT(groundPos.x == groundPos.x && groundPos.y == groundPos.y && groundPos.z == groundPos.z,
                    "ground position is not NaN");
        TEST_ASSERT(is_finite_vec3(groundNormal), "ground normal is not NaN");
        TEST_ASSERT(is_finite_vec3(groundVel), "ground velocity is not NaN");
        (void)JoltC_CharacterVirtual_GetGroundSubShapeID(cv);
        JoltC_CharacterVirtual_UpdateGroundVelocity(cv);   /* no ground: must not crash */

        /* GetActiveContact must respect its bounds check and leave the outputs
         * untouched when the index is out of range. */
        JoltC_Vec3 sentinelNormal = { 9.0f, 9.0f, 9.0f };
        JoltC_Vec3 sentinelVel = { 9.0f, 9.0f, 9.0f };
        uint32_t sentinelBody = 0x5A5A5A5Au;
        uint32_t sentinelSubShape = 0x5A5A5A5Au;
        JoltC_CharacterVirtual_GetActiveContact(cv, 0, &sentinelNormal, &sentinelVel,
                                                &sentinelBody, &sentinelSubShape);
        TEST_ASSERT_FLOAT_EQ(sentinelNormal.x, 9.0f, 1e-6f, "out-of-range GetActiveContact left normal alone");
        TEST_ASSERT_FLOAT_EQ(sentinelVel.y, 9.0f, 1e-6f, "out-of-range GetActiveContact left velocity alone");
        TEST_ASSERT(sentinelBody == 0x5A5A5A5Au, "out-of-range GetActiveContact left body id alone");
        TEST_ASSERT(sentinelSubShape == 0x5A5A5A5Au, "out-of-range GetActiveContact left sub shape id alone");

        /* No contacts means nothing to cancel: the desired velocity comes back
         * unchanged. (Asserted only in the zero-contact case on purpose — with
         * contacts present this is solver territory.) */
        JoltC_Vec3 desired = { 1.5f, -2.25f, 3.75f };
        JoltC_Vec3 cancelled = JoltC_CharacterVirtual_CancelVelocityTowardsSteepSlopes(cv, desired);
        TEST_ASSERT_FLOAT_EQ(cancelled.x, 1.5f, 1e-5f, "no slopes: x velocity passes through");
        TEST_ASSERT_FLOAT_EQ(cancelled.y, -2.25f, 1e-5f, "no slopes: y velocity passes through");
        TEST_ASSERT_FLOAT_EQ(cancelled.z, 3.75f, 1e-5f, "no slopes: z velocity passes through");

        /* No horizontal velocity means stair walking cannot apply, by definition. */
        JoltC_Vec3 still = { 0.0f, 0.0f, 0.0f };
        TEST_ASSERT(JoltC_CharacterVirtual_CanWalkStairs(cv, still) == 0,
                    "CanWalkStairs is false without horizontal velocity");

        /* Nothing below to stick to, so the sweep must fail and leave us put. */
        JoltC_Vec3 stepDown = { 0.0f, -0.5f, 0.0f };
        TEST_ASSERT(JoltC_CharacterVirtual_StickToFloor(cv, stepDown, ctx.tempAllocator) == 0,
                    "StickToFloor fails with no floor");
        JoltC_RVec3 after = JoltC_CharacterVirtual_GetPosition(cv);
        TEST_ASSERT_FLOAT_EQ(after.y, 12.0f, 1e-4f, "failed StickToFloor did not move the character");
        TEST_ASSERT(JoltC_CharacterVirtual_StickToFloor(cv, stepDown, NULL) == 0,
                    "StickToFloor without an allocator fails");

        /* A body id that was never created cannot have been collided with. */
        TEST_ASSERT(JoltC_CharacterVirtual_HasCollidedWithBody(cv, JOLTC_BODY_ID_INVALID) == JOLTC_FALSE,
                    "HasCollidedWithBody false for an invalid body");
        TEST_ASSERT(JoltC_CharacterVirtual_HasCollidedWith(cv, JOLTC_BODY_ID_INVALID) == 0,
                    "HasCollidedWith false for an invalid body");

        JoltC_CharacterVirtual_Destroy(cv);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* Overlap is geometry, not simulation: a capsule pushed into the only body in
     * the world must report that body. The contact's classification (ground? steep?
     * pushable?) is deliberately not asserted. */
    TEST_BEGIN("CharacterVirtual reports the body it overlaps");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_RVec3 floorPos = { 0.0f, 0.0f, 0.0f };   /* 1x1x1 box, top face at y = 0.5 */
        JoltC_BodyID floorID = create_test_box_body(&ctx, floorPos, JOLTC_MOTION_TYPE_STATIC,
                                                    JOLTC_ACTIVATION_DONT_ACTIVATE);
        TEST_ASSERT(floorID != JOLTC_BODY_ID_INVALID, "floor body created");
        JoltC_PhysicsSystem_OptimizeBroadPhase(ctx.physicsSystem);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f); /* half height 0.75 total */
        JoltC_CharacterVirtualSettings s;
        JoltC_CharacterVirtualSettings_SetDefault(&s);
        s.shape = capsule;

        /* Bottom of the capsule sits at 1.2 - 0.75 = 0.45, i.e. 0.05 inside the box. */
        JoltC_RVec3 pos = { 0.0f, 1.2f, 0.0f };
        JoltC_Quat rot = IDENTITY_QUAT_INIT;
        JoltC_CharacterVirtual* cv = JoltC_CharacterVirtual_Create(&s, pos, rot, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(cv, "CharacterVirtual created");

        JoltC_CharacterVirtual_RefreshContacts(cv, ctx.tempAllocator);
        uint32_t numContacts = JoltC_CharacterVirtual_GetNumActiveContacts(cv);
        TEST_ASSERT(numContacts >= 1, "an overlapping body produces at least one contact");

        if (numContacts >= 1) {
            JoltC_Vec3 normal = { 0.0f, 0.0f, 0.0f };
            JoltC_Vec3 contactVel = { 7.0f, 7.0f, 7.0f };
            uint32_t bodyId2 = JOLTC_BODY_ID_INVALID;
            uint32_t subShapeId2 = 0;
            JoltC_CharacterVirtual_GetActiveContact(cv, 0, &normal, &contactVel, &bodyId2, &subShapeId2);

            TEST_ASSERT(bodyId2 == floorID, "the contact names the only body in the world");
            float len = sqrtf(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
            TEST_ASSERT_FLOAT_EQ(len, 1.0f, 1e-3f, "contact normal is unit length");
            TEST_ASSERT(is_finite_vec3(contactVel), "contact velocity is not NaN");

            /* All-NULL outputs are legal and must not crash. */
            JoltC_CharacterVirtual_GetActiveContact(cv, 0, NULL, NULL, NULL, NULL);
        }

        JoltC_CharacterVirtual_Destroy(cv);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* Direction and sign only. Magnitudes, and the ground state that comes out of
     * this, are exactly what upstream is allowed to change.
     *
     * The caller integrates gravity, not Update. Jolt's own header is explicit: "it's your
     * own responsibility to apply gravity to the character velocity", and the gravity
     * argument "is only used when the character is standing on top of another object to
     * apply downward force". This test first assumed otherwise -- set the velocity to zero,
     * call Update twelve times with a gravity vector, expect a fall -- and it failed
     * identically on all three platforms, which is what a correct wrapper looks like when
     * the test is wrong. Left as a comment because it is exactly the assumption a consumer
     * of this API would make, and nothing in character.h contradicts it. */
    TEST_BEGIN("CharacterVirtual moves under caller-integrated gravity");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        JoltC_CharacterVirtualSettings s;
        JoltC_CharacterVirtualSettings_SetDefault(&s);
        s.shape = capsule;

        JoltC_RVec3 pos = { 1.5f, 20.0f, -3.25f };
        JoltC_Quat rot = IDENTITY_QUAT_INIT;
        JoltC_CharacterVirtual* cv = JoltC_CharacterVirtual_Create(&s, pos, rot, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(cv, "CharacterVirtual created");

        JoltC_Vec3 zero = { 0.0f, 0.0f, 0.0f };
        JoltC_CharacterVirtual_SetLinearVelocity(cv, zero);

        /* The pattern Jolt documents: read the velocity, add gravity * dt, write it back,
         * then Update. Exercising it here also covers the velocity round-trip through
         * twelve iterations rather than one. */
        JoltC_Vec3 gravity = { 0.0f, -9.81f, 0.0f };
        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 12; ++i) {
            JoltC_Vec3 v = JoltC_CharacterVirtual_GetLinearVelocity(cv);
            v.y += gravity.y * dt;
            JoltC_CharacterVirtual_SetLinearVelocity(cv, v);
            JoltC_CharacterVirtual_Update(cv, dt, gravity, ctx.tempAllocator);
        }

        JoltC_RVec3 nowPos = JoltC_CharacterVirtual_GetPosition(cv);
        JoltC_Vec3 nowVel = JoltC_CharacterVirtual_GetLinearVelocity(cv);
        TEST_ASSERT(nowPos.y < 20.0f, "y decreased once the caller applied gravity");
        TEST_ASSERT(nowVel.y < 0.0f, "vertical velocity is negative");
        TEST_ASSERT_FLOAT_EQ(nowPos.x, 1.5f, 1e-3f, "no horizontal drift in x");
        TEST_ASSERT_FLOAT_EQ(nowPos.z, -3.25f, 1e-3f, "no horizontal drift in z");
        TEST_ASSERT(is_valid_ground_state((int)JoltC_CharacterVirtual_GetGroundState(cv)),
                    "ground state stays a valid enum value");

        /* Update must ignore a missing allocator rather than dereference it. */
        JoltC_RVec3 beforeGuard = JoltC_CharacterVirtual_GetPosition(cv);
        JoltC_CharacterVirtual_Update(cv, 1.0f / 60.0f, gravity, NULL);
        JoltC_RVec3 afterGuard = JoltC_CharacterVirtual_GetPosition(cv);
        TEST_ASSERT_FLOAT_EQ(beforeGuard.y, afterGuard.y, 1e-6f, "Update without an allocator is a no-op");

        JoltC_CharacterVirtual_Destroy(cv);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* ====================================================================== */
    /*  7. Character IDs, character-vs-character collision                    */
    /* ====================================================================== */
    TEST_BEGIN("CharacterVirtual IDs are distinct");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        JoltC_CharacterVirtualSettings s;
        JoltC_CharacterVirtualSettings_SetDefault(&s);
        s.shape = capsule;

        JoltC_Quat rot = IDENTITY_QUAT_INIT;
        JoltC_RVec3 posA = { 0.0f, 8.0f, 0.0f };
        JoltC_RVec3 posB = { 4.0f, 8.0f, 0.0f };
        JoltC_CharacterVirtual* a = JoltC_CharacterVirtual_Create(&s, posA, rot, 0, ctx.physicsSystem);
        JoltC_CharacterVirtual* b = JoltC_CharacterVirtual_Create(&s, posB, rot, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(a, "character a created");
        TEST_ASSERT_NOT_NULL(b, "character b created");

        uint32_t idA = JoltC_CharacterVirtual_GetID(a);
        uint32_t idB = JoltC_CharacterVirtual_GetID(b);
        TEST_ASSERT(idA != idB, "two characters have different IDs");
        TEST_ASSERT(JoltC_CharacterVirtual_HasCollidedWithCharacter(a, idB) == 0,
                    "no character-vs-character collision recorded yet");

        JoltC_CharacterVirtual_Destroy(b);
        JoltC_CharacterVirtual_Destroy(a);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    TEST_BEGIN("CharacterVsCharacterCollision add/remove lifetime");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        JoltC_CharacterVirtualSettings s;
        JoltC_CharacterVirtualSettings_SetDefault(&s);
        s.shape = capsule;

        JoltC_Quat rot = IDENTITY_QUAT_INIT;
        JoltC_RVec3 posA = { 0.0f, 8.0f, 0.0f };
        JoltC_RVec3 posB = { 0.6f, 8.0f, 0.0f };
        JoltC_CharacterVirtual* a = JoltC_CharacterVirtual_Create(&s, posA, rot, 0, ctx.physicsSystem);
        JoltC_CharacterVirtual* b = JoltC_CharacterVirtual_Create(&s, posB, rot, 0, ctx.physicsSystem);

        JoltC_CharacterVsCharacterCollision* plain = JoltC_CharacterVsCharacterCollision_Create();
        JoltC_CharacterVsCharacterCollision* simple = JoltC_CharacterVsCharacterCollision_CreateSimple();
        TEST_ASSERT_NOT_NULL(plain, "Create returns a collision object");
        TEST_ASSERT_NOT_NULL(simple, "CreateSimple returns a collision object");
        TEST_ASSERT(plain != simple, "Create and CreateSimple return distinct objects");

        JoltC_CharacterVsCharacterCollisionSimple_AddCharacter(simple, a);
        JoltC_CharacterVsCharacterCollisionSimple_AddCharacter(simple, b);
        JoltC_CharacterVirtual_SetCharacterVsCharacterCollision(a, simple);
        JoltC_CharacterVirtual_SetCharacterVsCharacterCollision(b, simple);

        /* Guards. */
        JoltC_CharacterVsCharacterCollisionSimple_AddCharacter(NULL, a);
        JoltC_CharacterVsCharacterCollisionSimple_AddCharacter(simple, NULL);
        JoltC_CharacterVsCharacterCollisionSimple_RemoveCharacter(NULL, a);
        JoltC_CharacterVsCharacterCollisionSimple_RemoveCharacter(simple, NULL);
        JoltC_CharacterVirtual_SetCharacterVsCharacterCollision(NULL, simple);

        JoltC_CharacterVsCharacterCollisionSimple_RemoveCharacter(simple, b);
        JoltC_CharacterVsCharacterCollisionSimple_RemoveCharacter(simple, a);

        /* The characters hold a raw pointer to the collision object, so it has to be
         * detached before the object goes away. Nothing in the wrapper enforces this. */
        JoltC_CharacterVirtual_SetCharacterVsCharacterCollision(a, NULL);
        JoltC_CharacterVirtual_SetCharacterVsCharacterCollision(b, NULL);
        JoltC_CharacterVsCharacterCollision_Destroy(simple);
        JoltC_CharacterVsCharacterCollision_Destroy(plain);
        JoltC_CharacterVsCharacterCollision_Destroy(NULL);
        TEST_ASSERT(1, "collision object lifetime handled without crashing");

        JoltC_CharacterVirtual_Destroy(b);
        JoltC_CharacterVirtual_Destroy(a);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* ====================================================================== */
    /*  8. Contact listener plumbing                                          */
    /* ====================================================================== */

    /* Whether Jolt raises a contact callback for a given overlap, and how many
     * times, is version-dependent — so the call counts are NOT asserted. What is
     * asserted is the part the C binding owns: the userData round-trip and the
     * out-parameters, plus attach/detach/destroy without a crash. */
    TEST_BEGIN("CharacterContactListener plumbing");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        g_listenerCounters.validateCalls = 0;
        g_listenerCounters.addedCalls = 0;
        g_listenerCounters.persistedCalls = 0;
        g_listenerCounters.wrongUserData = 0;
        g_listenerCounters.nullOutPointer = 0;

        JoltC_RVec3 floorPos = { 0.0f, 0.0f, 0.0f };
        (void)create_test_box_body(&ctx, floorPos, JOLTC_MOTION_TYPE_STATIC, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_PhysicsSystem_OptimizeBroadPhase(ctx.physicsSystem);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        JoltC_CharacterVirtualSettings s;
        JoltC_CharacterVirtualSettings_SetDefault(&s);
        s.shape = capsule;

        JoltC_RVec3 pos = { 0.0f, 1.3f, 0.0f };
        JoltC_Quat rot = IDENTITY_QUAT_INIT;
        JoltC_CharacterVirtual* cv = JoltC_CharacterVirtual_Create(&s, pos, rot, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(cv, "CharacterVirtual created");

        JoltC_CharacterContactListener* listener =
            JoltC_CharacterContactListener_Create(on_validate_cb, on_added_cb, on_persisted_cb,
                                                  (void*)&g_listenerCounters);
        TEST_ASSERT_NOT_NULL(listener, "listener created");

        /* SetProcs is the alternative to passing the callbacks to Create. */
        JoltC_CharacterContactListener_Procs procs;
        procs.onValidate = on_validate_cb;
        procs.onAdded = on_added_cb;
        procs.onPersisted = on_persisted_cb;
        JoltC_CharacterContactListener_SetProcs(listener, procs, (void*)&g_listenerCounters);
        JoltC_CharacterContactListener_SetProcs(NULL, procs, (void*)&g_listenerCounters);   /* guard */

        JoltC_CharacterVirtual_SetListener(cv, listener);
        JoltC_CharacterVirtual_SetListener(NULL, listener);                                 /* guard */

        JoltC_ExtendedUpdateSettings eus;
        JoltC_ExtendedUpdateSettings_SetDefault(&eus);
        JoltC_Vec3 gravity = { 0.0f, -9.81f, 0.0f };
        JoltC_CharacterVirtual_RefreshContacts(cv, ctx.tempAllocator);
        for (int i = 0; i < 4; ++i)
            JoltC_CharacterVirtual_ExtendedUpdate(cv, 1.0f / 60.0f, gravity, &eus, ctx.tempAllocator);

        TEST_ASSERT(g_listenerCounters.wrongUserData == 0, "every callback received the userData we passed");
        TEST_ASSERT(g_listenerCounters.nullOutPointer == 0, "no callback received a NULL out-parameter");

        /* Guards on ExtendedUpdate itself. */
        JoltC_CharacterVirtual_ExtendedUpdate(cv, 1.0f / 60.0f, gravity, NULL, ctx.tempAllocator);
        JoltC_CharacterVirtual_ExtendedUpdate(cv, 1.0f / 60.0f, gravity, &eus, NULL);
        JoltC_CharacterVirtual_RefreshContacts(cv, NULL);
        TEST_ASSERT(1, "missing arguments ignored instead of dereferenced");

        /* Detach before destroying: Jolt keeps a raw pointer to the listener. */
        JoltC_CharacterVirtual_SetListener(cv, NULL);
        JoltC_CharacterContactListener_Destroy(listener);
        JoltC_CharacterContactListener_Destroy(NULL);

        JoltC_CharacterVirtual_Destroy(cv);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* ====================================================================== */
    /*  9. Non-virtual Character                                              */
    /* ====================================================================== */
    TEST_BEGIN("Character_Create honours settings fields");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.55f, 0.28f);
        JoltC_CharacterSettings s;
        JoltC_CharacterSettings_SetDefault(&s);
        s.up.x = 0.6f; s.up.y = 0.8f; s.up.z = 0.0f;   /* normalized, asymmetric */
        s.maxSlopeAngle = 0.75f;                        /* radians */
        s.enhancedInternalEdgeRemoval = JOLTC_TRUE;     /* no accessor — see report */
        s.shape = capsule;
        s.layer = OBJ_LAYER_DYNAMIC;
        s.mass = 55.0f;
        s.friction = 0.375f;
        s.gravityFactor = 0.625f;

        JoltC_RVec3 pos = { 1.5f, 3.25f, -2.75f };
        JoltC_Quat rot = IDENTITY_QUAT_INIT;
        JoltC_Character* c = JoltC_Character_Create(&s, pos, rot, 0xFEEDu, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(c, "Character created");

        JoltC_Vec3 up = JoltC_Character_GetUp(c);
        TEST_ASSERT_FLOAT_EQ(up.x, 0.6f, 1e-5f, "up.x round-trips");
        TEST_ASSERT_FLOAT_EQ(up.y, 0.8f, 1e-5f, "up.y round-trips");
        TEST_ASSERT_FLOAT_EQ(up.z, 0.0f, 1e-5f, "up.z round-trips");
        TEST_ASSERT(JoltC_Character_GetLayer(c) == OBJ_LAYER_DYNAMIC, "layer round-trips");

        JoltC_BodyID bodyID = JoltC_Character_GetBodyID(c);
        TEST_ASSERT(bodyID != JOLTC_BODY_ID_INVALID, "Character owns a body");
        /* friction and gravityFactor are only observable on the underlying body. */
        TEST_ASSERT_FLOAT_EQ(JoltC_BodyInterface_GetFriction(ctx.bodyInterface, bodyID), 0.375f, 1e-5f,
                             "friction reached the body");
        TEST_ASSERT_FLOAT_EQ(JoltC_BodyInterface_GetGravityFactor(ctx.bodyInterface, bodyID), 0.625f, 1e-5f,
                             "gravityFactor reached the body");
        TEST_ASSERT(JoltC_BodyInterface_GetObjectLayer(ctx.bodyInterface, bodyID) == OBJ_LAYER_DYNAMIC,
                    "layer reached the body");

        JoltC_RVec3 gotPos = JoltC_Character_GetPosition(c, JOLTC_TRUE);
        TEST_ASSERT_FLOAT_EQ(gotPos.x, 1.5f, 1e-4f, "position.x from create argument");
        TEST_ASSERT_FLOAT_EQ(gotPos.y, 3.25f, 1e-4f, "position.y from create argument");
        TEST_ASSERT_FLOAT_EQ(gotPos.z, -2.75f, 1e-4f, "position.z from create argument");

        /* maxSlopeAngle is only readable through the CharacterBase view. */
        JoltC_CharacterBase* base = JoltC_Character_AsBase(c);
        TEST_ASSERT_NOT_NULL(base, "AsBase returns a base handle");
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterBase_GetCosMaxSlopeAngle(base), cosf(0.75f), 1e-5f,
                             "cos(maxSlopeAngle) round-trips through the base view");
        JoltC_CharacterBase_Destroy(base);

        TEST_ASSERT(JoltC_Character_Create(NULL, pos, rot, 0, ctx.physicsSystem) == NULL,
                    "create without settings returns NULL");
        TEST_ASSERT(JoltC_Character_Create(&s, pos, rot, 0, NULL) == NULL,
                    "create without a physics system returns NULL");

        JoltC_Character_Destroy(c);
        JoltC_Character_Destroy(NULL);   /* documented no-op */
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    TEST_BEGIN("Character add/remove and layer round-trip");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        JoltC_CharacterSettings s;
        JoltC_CharacterSettings_SetDefault(&s);
        s.shape = capsule;
        s.layer = OBJ_LAYER_DYNAMIC;

        JoltC_RVec3 pos = { 0.0f, 2.5f, 0.0f };
        JoltC_Quat rot = IDENTITY_QUAT_INIT;
        JoltC_Character* c = JoltC_Character_Create(&s, pos, rot, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(c, "Character created");
        JoltC_BodyID bodyID = JoltC_Character_GetBodyID(c);

        TEST_ASSERT(JoltC_BodyInterface_IsAdded(ctx.bodyInterface, bodyID) == JOLTC_FALSE,
                    "body is not in the system before AddToPhysicsSystem");

        /* The layer round-trip happens while the body is still outside the
         * broadphase: only layers the test broadphase table knows about are valid,
         * and re-bucketing a live body is a separate concern from the round-trip
         * this test is about. */
        JoltC_Character_SetLayer(c, OBJ_LAYER_STATIC, JOLTC_TRUE);
        TEST_ASSERT(JoltC_Character_GetLayer(c) == OBJ_LAYER_STATIC, "SetLayer round-trips");
        TEST_ASSERT(JoltC_BodyInterface_GetObjectLayer(ctx.bodyInterface, bodyID) == OBJ_LAYER_STATIC,
                    "SetLayer reached the body");
        JoltC_Character_SetLayer(c, OBJ_LAYER_DYNAMIC, JOLTC_TRUE);
        TEST_ASSERT(JoltC_Character_GetLayer(c) == OBJ_LAYER_DYNAMIC, "SetLayer back round-trips");

        JoltC_Character_AddToPhysicsSystem(c, JOLTC_ACTIVATION_ACTIVATE, JOLTC_TRUE);
        TEST_ASSERT(JoltC_BodyInterface_IsAdded(ctx.bodyInterface, bodyID) != JOLTC_FALSE,
                    "body is in the system after AddToPhysicsSystem");
        JoltC_Character_Activate(c, JOLTC_TRUE);

        /* SetShape has no getter on Character; the body is where it lands. */
        const JoltC_Shape* smaller = JoltC_CapsuleShape_Create(0.35f, 0.15f);
        JoltC_Character_SetShape(c, smaller, 1000.0f, JOLTC_TRUE);
        const JoltC_Shape* bodyShape = JoltC_BodyInterface_GetShape(ctx.bodyInterface, bodyID);
        TEST_ASSERT(bodyShape == smaller, "SetShape reached the body");
        if (bodyShape) JoltC_Shape_Release(bodyShape);
        JoltC_Character_SetShape(c, NULL, 1000.0f, JOLTC_TRUE);   /* guard: must be ignored */

        /* One real step, then PostSimulation — the classification it produces is not
         * asserted, only that it stays inside the enum. */
        JoltC_PhysicsSystem_Update(ctx.physicsSystem, 1.0f / 60.0f, 1, ctx.tempAllocator, ctx.jobSystem);
        JoltC_Character_PostSimulation(c, 0.05f, JOLTC_TRUE);
        TEST_ASSERT(is_valid_ground_state((int)JoltC_Character_GetGroundState(c)),
                    "ground state is one of the four enum values");
        TEST_ASSERT(is_finite_vec3(JoltC_Character_GetGroundNormal(c)), "ground normal is not NaN");
        TEST_ASSERT(is_finite_vec3(JoltC_Character_GetGroundVelocity(c)), "ground velocity is not NaN");
        JoltC_RVec3 groundPos = JoltC_Character_GetGroundPosition(c);
        TEST_ASSERT(groundPos.y == groundPos.y, "ground position is not NaN");
        /* Nothing else exists in this world, so there can be no ground body. */
        TEST_ASSERT(JoltC_Character_GetGroundBodyID(c) == JOLTC_BODY_ID_INVALID,
                    "no ground body in an otherwise empty world");
        TEST_ASSERT(JoltC_Character_IsSupported(c) == JOLTC_FALSE, "not supported in an empty world");

        JoltC_Character_RemoveFromPhysicsSystem(c, JOLTC_TRUE);
        TEST_ASSERT(JoltC_BodyInterface_IsAdded(ctx.bodyInterface, bodyID) == JOLTC_FALSE,
                    "body left the system after RemoveFromPhysicsSystem");

        JoltC_Character_Destroy(c);
        JoltC_Shape_Release(smaller);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    TEST_BEGIN("Character position/rotation/velocity round-trips");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        JoltC_CharacterSettings s;
        JoltC_CharacterSettings_SetDefault(&s);
        s.shape = capsule;
        s.layer = OBJ_LAYER_DYNAMIC;

        JoltC_RVec3 pos = { 0.0f, 2.0f, 0.0f };
        JoltC_Quat rot = IDENTITY_QUAT_INIT;
        JoltC_Character* c = JoltC_Character_Create(&s, pos, rot, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(c, "Character created");
        JoltC_Character_AddToPhysicsSystem(c, JOLTC_ACTIVATION_DONT_ACTIVATE, JOLTC_TRUE);

        JoltC_RVec3 newPos = { 1.5f, -2.25f, 3.75f };
        JoltC_Character_SetPosition(c, newPos, JOLTC_ACTIVATION_DONT_ACTIVATE, JOLTC_TRUE);
        JoltC_RVec3 gotPos = JoltC_Character_GetPosition(c, JOLTC_TRUE);
        TEST_ASSERT_FLOAT_EQ(gotPos.x, 1.5f, 1e-4f, "SetPosition x round-trips");
        TEST_ASSERT_FLOAT_EQ(gotPos.y, -2.25f, 1e-4f, "SetPosition y round-trips");
        TEST_ASSERT_FLOAT_EQ(gotPos.z, 3.75f, 1e-4f, "SetPosition z round-trips");

        JoltC_Quat yaw = { 0.0f, 0.70710678f, 0.0f, 0.70710678f };
        JoltC_Character_SetRotation(c, yaw, JOLTC_ACTIVATION_DONT_ACTIVATE, JOLTC_TRUE);
        JoltC_Quat gotRot = JoltC_Character_GetRotation(c, JOLTC_TRUE);
        TEST_ASSERT_FLOAT_EQ(gotRot.y, 0.70710678f, 1e-5f, "SetRotation y round-trips");
        TEST_ASSERT_FLOAT_EQ(gotRot.w, 0.70710678f, 1e-5f, "SetRotation w round-trips");

        /* GetPositionAndRotation must agree with the individual getters. */
        JoltC_RVec3 pairPos = { 77.0f, 77.0f, 77.0f };
        JoltC_Quat pairRot = { 7.0f, 7.0f, 7.0f, 7.0f };
        JoltC_Character_GetPositionAndRotation(c, &pairPos, &pairRot);
        TEST_ASSERT_FLOAT_EQ(pairPos.x, gotPos.x, 1e-4f, "GetPositionAndRotation agrees on x");
        TEST_ASSERT_FLOAT_EQ(pairPos.y, gotPos.y, 1e-4f, "GetPositionAndRotation agrees on y");
        TEST_ASSERT_FLOAT_EQ(pairPos.z, gotPos.z, 1e-4f, "GetPositionAndRotation agrees on z");
        TEST_ASSERT_FLOAT_EQ(pairRot.y, gotRot.y, 1e-5f, "GetPositionAndRotation agrees on rotation y");
        TEST_ASSERT_FLOAT_EQ(pairRot.w, gotRot.w, 1e-5f, "GetPositionAndRotation agrees on rotation w");
        JoltC_Character_GetPositionAndRotation(c, NULL, NULL);          /* both out params optional */
        JoltC_Character_GetPositionAndRotation(NULL, &pairPos, &pairRot);
        TEST_ASSERT_FLOAT_EQ(pairPos.x, gotPos.x, 1e-4f, "NULL character left the outputs alone");

        /* SetPositionAndRotation is the combined setter (note: its activation
         * parameter is a plain int in the header, not JoltC_Activation). */
        JoltC_RVec3 combinedPos = { -4.5f, 5.5f, 6.25f };
        JoltC_Quat identity = IDENTITY_QUAT_INIT;
        JoltC_Character_SetPositionAndRotation(c, combinedPos, identity, (int)JOLTC_ACTIVATION_DONT_ACTIVATE);
        gotPos = JoltC_Character_GetPosition(c, JOLTC_TRUE);
        gotRot = JoltC_Character_GetRotation(c, JOLTC_TRUE);
        TEST_ASSERT_FLOAT_EQ(gotPos.x, -4.5f, 1e-4f, "SetPositionAndRotation x round-trips");
        TEST_ASSERT_FLOAT_EQ(gotPos.y, 5.5f, 1e-4f, "SetPositionAndRotation y round-trips");
        TEST_ASSERT_FLOAT_EQ(gotPos.z, 6.25f, 1e-4f, "SetPositionAndRotation z round-trips");
        TEST_ASSERT_FLOAT_EQ(gotRot.w, 1.0f, 1e-5f, "SetPositionAndRotation rotation round-trips");

        JoltC_Mat44 world = JoltC_Character_GetWorldTransform(c);
        TEST_ASSERT_FLOAT_EQ(world.m[12], gotPos.x, 1e-4f, "world transform agrees on x");
        TEST_ASSERT_FLOAT_EQ(world.m[13], gotPos.y, 1e-4f, "world transform agrees on y");
        TEST_ASSERT_FLOAT_EQ(world.m[14], gotPos.z, 1e-4f, "world transform agrees on z");

        JoltC_RVec3 comPos = JoltC_Character_GetCenterOfMassPosition(c, JOLTC_TRUE);
        TEST_ASSERT(comPos.y == comPos.y, "center of mass position is not NaN");

        /* Velocity is stored, not integrated, so exact round-trips are fair game. */
        JoltC_Vec3 vel = { 1.5f, 0.0f, -2.25f };
        JoltC_Character_SetLinearVelocity(c, vel, JOLTC_TRUE);
        JoltC_Vec3 gotVel = JoltC_Character_GetLinearVelocity(c, JOLTC_TRUE);
        TEST_ASSERT_FLOAT_EQ(gotVel.x, 1.5f, 1e-4f, "SetLinearVelocity x round-trips");
        TEST_ASSERT_FLOAT_EQ(gotVel.y, 0.0f, 1e-4f, "SetLinearVelocity y round-trips");
        TEST_ASSERT_FLOAT_EQ(gotVel.z, -2.25f, 1e-4f, "SetLinearVelocity z round-trips");

        JoltC_Vec3 extra = { 0.25f, 0.0f, 0.5f };
        JoltC_Character_AddLinearVelocity(c, extra, JOLTC_TRUE);
        gotVel = JoltC_Character_GetLinearVelocity(c, JOLTC_TRUE);
        TEST_ASSERT_FLOAT_EQ(gotVel.x, 1.75f, 1e-4f, "AddLinearVelocity accumulates x");
        TEST_ASSERT_FLOAT_EQ(gotVel.z, -1.75f, 1e-4f, "AddLinearVelocity accumulates z");

        JoltC_Vec3 linear = { -3.5f, 0.75f, 2.5f };
        JoltC_Vec3 angular = { 0.0f, 0.0f, 0.0f };
        JoltC_Character_SetLinearAndAngularVelocity(c, linear, angular);
        gotVel = JoltC_Character_GetLinearVelocity(c, JOLTC_TRUE);
        TEST_ASSERT_FLOAT_EQ(gotVel.x, -3.5f, 1e-4f, "SetLinearAndAngularVelocity x round-trips");
        TEST_ASSERT_FLOAT_EQ(gotVel.y, 0.75f, 1e-4f, "SetLinearAndAngularVelocity y round-trips");
        TEST_ASSERT_FLOAT_EQ(gotVel.z, 2.5f, 1e-4f, "SetLinearAndAngularVelocity z round-trips");

        /* Sign only: the exact delta depends on how mass is applied. */
        JoltC_Vec3 zero = { 0.0f, 0.0f, 0.0f };
        JoltC_Character_SetLinearVelocity(c, zero, JOLTC_TRUE);
        JoltC_Vec3 impulse = { 100.0f, 0.0f, 0.0f };
        JoltC_Character_AddImpulse(c, impulse, JOLTC_TRUE);
        gotVel = JoltC_Character_GetLinearVelocity(c, JOLTC_TRUE);
        TEST_ASSERT(gotVel.x > 0.0f, "AddImpulse along +x produces positive x velocity");

        JoltC_Character_RemoveFromPhysicsSystem(c, JOLTC_TRUE);
        JoltC_Character_Destroy(c);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    TEST_BEGIN("Character SetUp / SetMaxSlopeAngle round-trip");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        JoltC_CharacterSettings s;
        JoltC_CharacterSettings_SetDefault(&s);
        s.shape = capsule;
        s.layer = OBJ_LAYER_DYNAMIC;

        JoltC_RVec3 pos = { 0.0f, 2.0f, 0.0f };
        JoltC_Quat rot = IDENTITY_QUAT_INIT;
        JoltC_Character* c = JoltC_Character_Create(&s, pos, rot, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(c, "Character created");

        JoltC_Vec3 newUp = { 0.0f, 0.0f, 1.0f };
        JoltC_Character_SetUp(c, newUp);
        JoltC_Vec3 gotUp = JoltC_Character_GetUp(c);
        TEST_ASSERT_FLOAT_EQ(gotUp.x, 0.0f, 1e-6f, "SetUp x round-trips");
        TEST_ASSERT_FLOAT_EQ(gotUp.y, 0.0f, 1e-6f, "SetUp y round-trips");
        TEST_ASSERT_FLOAT_EQ(gotUp.z, 1.0f, 1e-6f, "SetUp z round-trips");

        JoltC_Character_SetMaxSlopeAngle(c, 0.45f);
        JoltC_CharacterBase* base = JoltC_Character_AsBase(c);
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterBase_GetCosMaxSlopeAngle(base), cosf(0.45f), 1e-5f,
                             "SetMaxSlopeAngle stores the cosine of the radian angle");
        JoltC_CharacterBase_Destroy(base);

        JoltC_Character_Destroy(c);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* ====================================================================== */
    /* 10. CharacterBase view                                                 */
    /* ====================================================================== */

    /* AsBase is a hand-written cast between two wrapper structs — precisely the
     * sort of code a repair can point at the wrong object without failing to
     * compile. Writing through the base view and reading back through the derived
     * accessors proves both handles refer to the same character. */
    TEST_BEGIN("CharacterVirtual_AsBase views the same character");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.5f, 0.25f);
        JoltC_CharacterVirtualSettings s;
        JoltC_CharacterVirtualSettings_SetDefault(&s);
        s.shape = capsule;
        s.maxSlopeAngle = 0.55f;

        JoltC_RVec3 pos = { 0.0f, 9.0f, 0.0f };
        JoltC_Quat rot = IDENTITY_QUAT_INIT;
        JoltC_CharacterVirtual* cv = JoltC_CharacterVirtual_Create(&s, pos, rot, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(cv, "CharacterVirtual created");

        JoltC_CharacterBase* base = JoltC_CharacterVirtual_AsBase(cv);
        TEST_ASSERT_NOT_NULL(base, "AsBase returns a base handle");
        TEST_ASSERT(JoltC_CharacterVirtual_AsBase(NULL) == NULL, "AsBase of NULL is NULL");

        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterBase_GetCosMaxSlopeAngle(base),
                             JoltC_CharacterVirtual_GetCosMaxSlopeAngle(cv), 1e-6f,
                             "base and derived agree on cos max slope angle");
        TEST_ASSERT((int)JoltC_CharacterBase_GetGroundState(base) == (int)JoltC_CharacterVirtual_GetGroundState(cv),
                    "base and derived agree on ground state");
        TEST_ASSERT((JoltC_CharacterBase_IsSupported(base) != 0) ==
                    (JoltC_CharacterVirtual_IsSupported(cv) != 0),
                    "base and derived agree on IsSupported");
        TEST_ASSERT(JoltC_CharacterBase_GetGroundBodyId(base) == JoltC_CharacterVirtual_GetGroundBodyID(cv),
                    "base and derived agree on ground body id");
        TEST_ASSERT(JoltC_CharacterBase_GetGroundSubShapeId(base) == JoltC_CharacterVirtual_GetGroundSubShapeID(cv),
                    "base and derived agree on ground sub shape id");
        TEST_ASSERT(JoltC_CharacterBase_GetGroundUserData(base) == JoltC_CharacterVirtual_GetGroundUserData(cv),
                    "base and derived agree on ground user data");

        JoltC_Vec3 baseGroundNormal = JoltC_CharacterBase_GetGroundNormal(base);
        JoltC_Vec3 cvGroundNormal = JoltC_CharacterVirtual_GetGroundNormal(cv);
        TEST_ASSERT_FLOAT_EQ(baseGroundNormal.y, cvGroundNormal.y, 1e-6f,
                             "base and derived agree on ground normal");
        JoltC_RVec3 baseGroundPos = JoltC_CharacterBase_GetGroundPosition(base);
        JoltC_RVec3 cvGroundPos = JoltC_CharacterVirtual_GetGroundPosition(cv);
        TEST_ASSERT_FLOAT_EQ(baseGroundPos.y, cvGroundPos.y, 1e-6f,
                             "base and derived agree on ground position");
        JoltC_Vec3 baseGroundVel = JoltC_CharacterBase_GetGroundVelocity(base);
        JoltC_Vec3 cvGroundVel = JoltC_CharacterVirtual_GetGroundVelocity(cv);
        TEST_ASSERT_FLOAT_EQ(baseGroundVel.y, cvGroundVel.y, 1e-6f,
                             "base and derived agree on ground velocity");

        /* Write through the base, read through the derived. */
        JoltC_Vec3 baseUp = { 1.0f, 0.0f, 0.0f };
        JoltC_CharacterBase_SetUp(base, baseUp);
        JoltC_Vec3 cvUp = JoltC_CharacterVirtual_GetUp(cv);
        TEST_ASSERT_FLOAT_EQ(cvUp.x, 1.0f, 1e-6f, "SetUp through the base reached the character");
        TEST_ASSERT_FLOAT_EQ(cvUp.y, 0.0f, 1e-6f, "SetUp through the base replaced the whole vector");
        JoltC_Vec3 fromBase = JoltC_CharacterBase_GetUp(base);
        TEST_ASSERT_FLOAT_EQ(fromBase.x, 1.0f, 1e-6f, "base GetUp agrees");

        JoltC_CharacterBase_SetMaxSlopeAngle(base, 0.35f);
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetCosMaxSlopeAngle(cv), cosf(0.35f), 1e-5f,
                             "SetMaxSlopeAngle through the base reached the character");

        /* With up = +x, a normal along +x is flat and one along +y is a wall. */
        JoltC_Vec3 alongUp = { 1.0f, 0.0f, 0.0f };
        JoltC_Vec3 perpendicular = { 0.0f, 1.0f, 0.0f };
        TEST_ASSERT(JoltC_CharacterBase_IsSlopeTooSteep(base, alongUp) == 0,
                    "base: a normal along up is not too steep");
        TEST_ASSERT(JoltC_CharacterBase_IsSlopeTooSteep(base, perpendicular) != 0,
                    "base: a perpendicular normal is too steep");

        /* Destroying the base view must not destroy the character. */
        JoltC_CharacterBase_Destroy(base);
        JoltC_CharacterBase_Destroy(NULL);
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetPosition(cv).y, 9.0f, 1e-4f,
                             "character still usable after the base view is destroyed");

        JoltC_CharacterVirtual_Destroy(cv);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* ====================================================================== */
    /* 11. Null safety on the accessors                                       */
    /* ====================================================================== */

    /* Only the guard values that carry meaning are asserted — "no ground body",
     * "not supported", "invalid layer", "in air". The wrapper also returns a
     * zero vector from some vector getters and (0,1,0) from others; that
     * inconsistency is reported rather than frozen into a test. */
    TEST_BEGIN("Character accessors are NULL safe");
    {
        JoltC_Vec3  vZero = { 0.0f, 0.0f, 0.0f };
        JoltC_Vec3  vUp = { 0.0f, 1.0f, 0.0f };
        JoltC_Vec3  vRight = { 1.0f, 0.0f, 0.0f };
        JoltC_Vec3  vAsym = { 1.5f, -2.25f, 3.75f };
        JoltC_RVec3 rZero = { 0.0f, 0.0f, 0.0f };
        JoltC_Quat  qIdentity = IDENTITY_QUAT_INIT;

        TEST_ASSERT(JoltC_CharacterVirtual_GetGroundState(NULL) == JOLTC_GROUND_STATE_IN_AIR,
                    "CharacterVirtual GetGroundState(NULL) is InAir");
        TEST_ASSERT(JoltC_CharacterVirtual_IsSupported(NULL) == JOLTC_FALSE,
                    "CharacterVirtual IsSupported(NULL) is false");
        TEST_ASSERT(JoltC_CharacterVirtual_GetGroundBodyID(NULL) == JOLTC_BODY_ID_INVALID,
                    "CharacterVirtual GetGroundBodyID(NULL) is invalid");
        TEST_ASSERT(JoltC_CharacterVirtual_GetInnerBodyID(NULL) == JOLTC_BODY_ID_INVALID,
                    "CharacterVirtual GetInnerBodyID(NULL) is invalid");
        TEST_ASSERT(JoltC_CharacterVirtual_GetGroundUserData(NULL) == 0,
                    "CharacterVirtual GetGroundUserData(NULL) is 0");
        TEST_ASSERT(JoltC_CharacterVirtual_GetNumActiveContacts(NULL) == 0,
                    "CharacterVirtual GetNumActiveContacts(NULL) is 0");
        TEST_ASSERT(JoltC_CharacterVirtual_GetShape(NULL) == NULL,
                    "CharacterVirtual GetShape(NULL) is NULL");
        TEST_ASSERT(JoltC_CharacterVirtual_GetMaxHitsExceeded(NULL) == JOLTC_FALSE,
                    "CharacterVirtual GetMaxHitsExceeded(NULL) is false");
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetMass(NULL), 0.0f, 1e-6f,
                             "CharacterVirtual GetMass(NULL) is 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetMaxStrength(NULL), 0.0f, 1e-6f,
                             "CharacterVirtual GetMaxStrength(NULL) is 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetCharacterPadding(NULL), 0.0f, 1e-6f,
                             "CharacterVirtual GetCharacterPadding(NULL) is 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetPenetrationRecoverySpeed(NULL), 0.0f, 1e-6f,
                             "CharacterVirtual GetPenetrationRecoverySpeed(NULL) is 0");
        TEST_ASSERT(JoltC_CharacterVirtual_GetMaxNumHits(NULL) == 0,
                    "CharacterVirtual GetMaxNumHits(NULL) is 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetCosMaxSlopeAngle(NULL), 0.0f, 1e-6f,
                             "CharacterVirtual GetCosMaxSlopeAngle(NULL) is 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterVirtual_GetHitReductionCosMaxAngle(NULL), 0.0f, 1e-6f,
                             "CharacterVirtual GetHitReductionCosMaxAngle(NULL) is 0");
        TEST_ASSERT(JoltC_CharacterVirtual_GetGroundSubShapeID(NULL) == 0,
                    "CharacterVirtual GetGroundSubShapeID(NULL) is 0");
        TEST_ASSERT(JoltC_CharacterVirtual_GetEnhancedInternalEdgeRemoval(NULL) == JOLTC_FALSE,
                    "CharacterVirtual GetEnhancedInternalEdgeRemoval(NULL) is false");
        TEST_ASSERT(JoltC_CharacterVirtual_CanWalkStairs(NULL, vRight) == 0,
                    "CharacterVirtual CanWalkStairs(NULL) is false");
        TEST_ASSERT(JoltC_CharacterVirtual_HasCollidedWithCharacter(NULL, 0) == 0,
                    "CharacterVirtual HasCollidedWithCharacter(NULL) is false");
        TEST_ASSERT(JoltC_CharacterVirtual_IsSlopeTooSteep(NULL, vUp) == JOLTC_FALSE,
                    "CharacterVirtual IsSlopeTooSteep(NULL) is false");

        /* Vector and matrix getters: exercised for crash-freedom, values not pinned. */
        (void)JoltC_CharacterVirtual_GetUp(NULL);
        (void)JoltC_CharacterVirtual_GetShapeOffset(NULL);
        (void)JoltC_CharacterVirtual_GetGroundNormal(NULL);
        (void)JoltC_CharacterVirtual_GetGroundPosition(NULL);
        (void)JoltC_CharacterVirtual_GetGroundVelocity(NULL);
        (void)JoltC_CharacterVirtual_GetCenterOfMassPosition(NULL);
        (void)JoltC_CharacterVirtual_GetWorldTransform(NULL);
        (void)JoltC_CharacterVirtual_GetCenterOfMassTransform(NULL);
        (void)JoltC_CharacterVirtual_CancelVelocityTowardsSteepSlopes(NULL, vAsym);
        JoltC_CharacterVirtual_UpdateGroundVelocity(NULL);
        JoltC_CharacterVirtual_SetInnerBodyShape(NULL, NULL);
        JoltC_CharacterVirtual_SetMass(NULL, 1.0f);
        JoltC_CharacterVirtual_SetMaxStrength(NULL, 1.0f);
        JoltC_CharacterVirtual_SetPenetrationRecoverySpeed(NULL, 1.0f);
        JoltC_CharacterVirtual_SetMaxNumHits(NULL, 1);
        JoltC_CharacterVirtual_SetHitReductionCosMaxAngle(NULL, 1.0f);
        JoltC_CharacterVirtual_SetEnhancedInternalEdgeRemoval(NULL, JOLTC_TRUE);
        JoltC_CharacterVirtual_SetShapeOffset(NULL, vZero);
        JoltC_CharacterVirtual_SetUp(NULL, vUp);
        JoltC_CharacterVirtual_SetMaxSlopeAngle(NULL, 0.5f);
        JoltC_CharacterVirtual_SetPosition(NULL, rZero);
        JoltC_CharacterVirtual_SetRotation(NULL, qIdentity);
        JoltC_CharacterVirtual_SetLinearVelocity(NULL, vZero);
        JoltC_CharacterVirtual_GetActiveContact(NULL, 0, NULL, NULL, NULL, NULL);

        /* Non-virtual Character. */
        TEST_ASSERT(JoltC_Character_GetGroundState(NULL) == JOLTC_GROUND_STATE_IN_AIR,
                    "Character GetGroundState(NULL) is InAir");
        TEST_ASSERT(JoltC_Character_IsSupported(NULL) == JOLTC_FALSE, "Character IsSupported(NULL) is false");
        TEST_ASSERT(JoltC_Character_GetGroundBodyID(NULL) == JOLTC_BODY_ID_INVALID,
                    "Character GetGroundBodyID(NULL) is invalid");
        TEST_ASSERT(JoltC_Character_GetBodyID(NULL) == JOLTC_BODY_ID_INVALID,
                    "Character GetBodyID(NULL) is invalid");
        TEST_ASSERT(JoltC_Character_GetLayer(NULL) == JOLTC_OBJECT_LAYER_INVALID,
                    "Character GetLayer(NULL) is the invalid layer");
        TEST_ASSERT(JoltC_Character_AsBase(NULL) == NULL, "Character AsBase(NULL) is NULL");
        (void)JoltC_Character_GetUp(NULL);
        (void)JoltC_Character_GetGroundNormal(NULL);
        (void)JoltC_Character_GetGroundVelocity(NULL);
        (void)JoltC_Character_GetGroundPosition(NULL);
        (void)JoltC_Character_GetPosition(NULL, JOLTC_TRUE);
        (void)JoltC_Character_GetRotation(NULL, JOLTC_TRUE);
        (void)JoltC_Character_GetCenterOfMassPosition(NULL, JOLTC_TRUE);
        (void)JoltC_Character_GetLinearVelocity(NULL, JOLTC_TRUE);
        (void)JoltC_Character_GetWorldTransform(NULL);
        JoltC_Character_SetUp(NULL, vUp);
        JoltC_Character_SetMaxSlopeAngle(NULL, 0.5f);
        JoltC_Character_SetLayer(NULL, OBJ_LAYER_DYNAMIC, JOLTC_TRUE);
        JoltC_Character_SetShape(NULL, NULL, 1.0f, JOLTC_TRUE);
        JoltC_Character_SetLinearVelocity(NULL, vZero, JOLTC_TRUE);
        JoltC_Character_AddLinearVelocity(NULL, vZero, JOLTC_TRUE);
        JoltC_Character_AddImpulse(NULL, vZero, JOLTC_TRUE);
        JoltC_Character_SetLinearAndAngularVelocity(NULL, vZero, vZero);
        JoltC_Character_SetPosition(NULL, rZero, JOLTC_ACTIVATION_DONT_ACTIVATE, JOLTC_TRUE);
        JoltC_Character_SetRotation(NULL, qIdentity, JOLTC_ACTIVATION_DONT_ACTIVATE, JOLTC_TRUE);
        JoltC_Character_SetPositionAndRotation(NULL, rZero, qIdentity,
                                               (int)JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_Character_AddToPhysicsSystem(NULL, JOLTC_ACTIVATION_DONT_ACTIVATE, JOLTC_TRUE);
        JoltC_Character_RemoveFromPhysicsSystem(NULL, JOLTC_TRUE);
        JoltC_Character_Activate(NULL, JOLTC_TRUE);
        JoltC_Character_PostSimulation(NULL, 0.05f, JOLTC_TRUE);

        /* CharacterBase. */
        TEST_ASSERT(JoltC_CharacterBase_GetGroundState(NULL) == JOLTC_GROUND_STATE_IN_AIR,
                    "CharacterBase GetGroundState(NULL) is InAir");
        TEST_ASSERT(JoltC_CharacterBase_IsSupported(NULL) == 0, "CharacterBase IsSupported(NULL) is false");
        TEST_ASSERT(JoltC_CharacterBase_GetGroundBodyId(NULL) == JOLTC_BODY_ID_INVALID,
                    "CharacterBase GetGroundBodyId(NULL) is invalid");
        TEST_ASSERT(JoltC_CharacterBase_GetGroundSubShapeId(NULL) == 0,
                    "CharacterBase GetGroundSubShapeId(NULL) is 0");
        TEST_ASSERT(JoltC_CharacterBase_GetGroundUserData(NULL) == 0,
                    "CharacterBase GetGroundUserData(NULL) is 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_CharacterBase_GetCosMaxSlopeAngle(NULL), 0.0f, 1e-6f,
                             "CharacterBase GetCosMaxSlopeAngle(NULL) is 0");
        TEST_ASSERT(JoltC_CharacterBase_IsSlopeTooSteep(NULL, vUp) == 0,
                    "CharacterBase IsSlopeTooSteep(NULL) is false");
        (void)JoltC_CharacterBase_GetUp(NULL);
        (void)JoltC_CharacterBase_GetGroundNormal(NULL);
        (void)JoltC_CharacterBase_GetGroundPosition(NULL);
        (void)JoltC_CharacterBase_GetGroundVelocity(NULL);
        JoltC_CharacterBase_SetUp(NULL, vUp);
        JoltC_CharacterBase_SetMaxSlopeAngle(NULL, 0.5f);

        TEST_ASSERT(1, "no accessor dereferenced a NULL handle");
    }
    TEST_END();
}
