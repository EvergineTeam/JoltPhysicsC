/* JoltC Test Suite -- shared test infrastructure
 * SPDX-License-Identifier: MIT
 */

#ifndef TEST_COMMON_H
#define TEST_COMMON_H

#include <JoltC/joltc.h>
#include <stdio.h>
#include <math.h>

/* ========================================================================== */
/*  Global counters                                                           */
/* ========================================================================== */
extern int g_tests_run;
extern int g_tests_passed;
extern int g_tests_failed;
extern int g_current_test_failed;

/* ========================================================================== */
/*  Layer constants                                                           */
/* ========================================================================== */
#define OBJ_LAYER_STATIC   0
#define OBJ_LAYER_DYNAMIC  1
#define NUM_OBJ_LAYERS     2

#define BP_LAYER_NON_MOVING 0
#define BP_LAYER_MOVING     1
#define NUM_BP_LAYERS       2

/* ========================================================================== */
/*  Test macros                                                               */
/* ========================================================================== */
#define TEST_BEGIN(name) \
    do { \
        g_tests_run++; \
        g_current_test_failed = 0; \
        printf("  [TEST] %-50s ", name);

#define TEST_END() \
        if (!g_current_test_failed) { \
            g_tests_passed++; \
            printf("PASS\n"); \
        } \
    } while (0)

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            if (!g_current_test_failed) { \
                g_current_test_failed = 1; \
                g_tests_failed++; \
                printf("FAIL\n"); \
            } \
            printf("    ASSERT FAILED: %s\n", msg); \
            printf("    at %s:%d\n", __FILE__, __LINE__); \
        } \
    } while (0)

#define TEST_ASSERT_NOT_NULL(ptr, msg) \
    TEST_ASSERT((ptr) != NULL, msg)

#define TEST_ASSERT_FLOAT_EQ(a, b, eps, msg) \
    TEST_ASSERT(fabsf((float)(a) - (float)(b)) < (eps), msg)

/* ========================================================================== */
/*  Physics system context for tests that need a full environment             */
/* ========================================================================== */
typedef struct TestPhysicsContext {
    JoltC_TempAllocator*                   tempAllocator;
    JoltC_JobSystem*                       jobSystem;
    JoltC_ObjectLayerPairFilter*           objectLayerPairFilter;
    JoltC_BroadPhaseLayerInterface*        broadPhaseLayerInterface;
    JoltC_ObjectVsBroadPhaseLayerFilter*   objectVsBroadPhaseLayerFilter;
    JoltC_PhysicsSystem*                   physicsSystem;
    JoltC_BodyInterface*                   bodyInterface;
} TestPhysicsContext;

void setup_physics_context(TestPhysicsContext* ctx);
void teardown_physics_context(TestPhysicsContext* ctx);

/* Helper: create a 1x1x1 dynamic or static box body at the given position */
JoltC_BodyID create_test_box_body(TestPhysicsContext* ctx, JoltC_RVec3 position,
                                  JoltC_MotionType motionType, JoltC_Activation activation);

/* ========================================================================== */
/*  Test suite entry points                                                   */
/* ========================================================================== */
void run_common_tests(void);
void run_math_tests(void);
void run_physics_system_tests(void);
void run_shape_tests(void);
void run_body_tests(void);
void run_constraint_tests(void);
void run_filter_tests(void);
void run_query_tests(void);
void run_character_tests(void);
void run_skeleton_tests(void);
void run_vehicle_tests(void);

/* Added ahead of the JoltPhysics 5.6.0 bump. Coverage was 234 of 1,280 functions and
 * body_access had none at all, so a hand repair could change behaviour with nothing to
 * notice. These four target what that bump actually touches. */
void run_body_access_tests(void);
void run_math_roundtrip_tests(void);
void run_character_extra_tests(void);
void run_shape_props_tests(void);
void run_skeleton_extra_tests(void);
void run_vehicle_extra_tests(void);
void run_vehicle_live_tests(void);

/* Added with the soft body surface (SoftBodySharedSettings + SoftBodyMotionProperties):
 * before it, a soft body could be created but never configured or read back. */
void run_soft_body_tests(void);

/* The phase 0 coverage repairs: honest shape type enums, real constraint GetSettings (which is
 * what makes ragdolls articulable from C), character simulation with filters, the soft body
 * contact listener, and query settings that finally connect to a query. */
void run_phase0_tests(void);

/* Phase 1: physics materials end to end (mesh, height field, character ground) and shape
 * introspection (triangle walk, leaf resolution, submerged volume, runtime terrain deformation). */
void run_phase1_tests(void);

/* Phase 2: the complete soft body surface: per-vertex attributes and the LRA constraints they
 * unlock, direct constraint construction, Cosserat rods, skinning, per-vertex runtime writes and
 * hand-stepped bodies via CustomUpdate. */
void run_phase2_tests(void);

/* Phase 3: determinism and state: StateRecorder in memory, whole-system rollback that replays to
 * bit-identical state, byte-shipped snapshots, single-body state, and the character that saves
 * alongside the world because the system snapshot cannot see it. */
void run_phase3_tests(void);

/* Phase 4: constraints and vehicles to one hundred percent: PathConstraint with its Hermite
 * paths, pulley rope control at runtime, gear/rack ratio arithmetic, body-space motor targets,
 * the three vehicle step callbacks and the tire telemetry fields. */
void run_phase4_tests(void);

/* Phase 5: character and system to one hundred percent: the full contact listener with the
 * character-versus-character family, supporting volume, MotionProperties completion, closest-hit
 * casts, internal edge removal, broad phase boxes, the census, combine functions, alternative
 * allocators and collision response estimation. */
void run_phase5_tests(void);

/* Phase 6: the debug renderer: Jolt's own drawing through three C callbacks. */
void run_phase6_tests(void);

#endif /* TEST_COMMON_H */
