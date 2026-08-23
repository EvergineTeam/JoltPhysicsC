/* JoltC Test Suite -- skeleton.h coverage beyond test_skeleton.c
 * SPDX-License-Identifier: MIT
 *
 * test_skeleton.c exercises 20 of the 82 functions in skeleton.h and stops at
 * construction: it adds joints, reads a count, and round-trips one joint state through a
 * pose. It never resolves a parent index, never builds a joint matrix, and never touches
 * SkeletonMapper, RagdollSettings or Ragdoll at all. This file covers those.
 *
 * Every expectation here was checked against the JoltPhysics 5.5.0 headers and sources
 * rather than assumed, because several of them are counter-intuitive:
 *
 *   - Skeleton::AddJoint(name, parentName) stores the name and leaves mParentJointIndex at
 *     -1. Only CalculateParentJointIndices() resolves it. AddJoint(name, parentIndex) is
 *     the opposite: it sets the index immediately and back-fills the parent's name.
 *   - SkeletalAnimation::GetDuration() reads the last keyframe of animated joint 0 only.
 *     Other joints do not contribute.
 *   - Ragdoll::GetPose() does not return what SetPose() was given. The root offset comes
 *     out as body 0's world position and joint matrix 0 comes out with a zero translation;
 *     every other joint is relative to that. Get-then-Set is a fixed point, but
 *     Set-then-Get is not the identity unless joint 0's translation was already zero.
 *   - SkeletonPose::CalculateLocalSpaceJointMatrices() ignores the hierarchy entirely; it
 *     is per-joint ToMatrix(). CalculateJointMatrices() is the one that composes parents.
 *
 * Values are asymmetric throughout, negatives included, and pairs of the same type are far
 * apart so that exchanging them is visible. Nothing here asserts a position, velocity or
 * ground state after a simulation step: 5.6.0 replaces the friction model, so any such
 * assertion would break for a legitimate reason. The body positions that are asserted are
 * read straight after an explicit placement call with no step in between, which is exact
 * arithmetic rather than solver output.
 */

#include "test_common.h"
#include <string.h>

void run_skeleton_extra_tests(void);

/* ========================================================================== */
/*  Helpers                                                                   */
/* ========================================================================== */

/* JoltC_Mat44 is column-major, so element (row, col) lives at m[col * 4 + row] and the
 * translation is the fourth column: m[12], m[13], m[14]. */
static void make_translation_matrix(JoltC_Mat44* out, float x, float y, float z)
{
    int i;
    for (i = 0; i < 16; ++i)
        out->m[i] = 0.0f;
    out->m[0]  = 1.0f;
    out->m[5]  = 1.0f;
    out->m[10] = 1.0f;
    out->m[12] = x;
    out->m[13] = y;
    out->m[14] = z;
    out->m[15] = 1.0f;
}

static void fill_matrix_with(JoltC_Mat44* out, float value)
{
    int i;
    for (i = 0; i < 16; ++i)
        out->m[i] = value;
}

static void assert_matrix_translation(const JoltC_Mat44* m, float x, float y, float z,
                                      const char* whatX, const char* whatY, const char* whatZ)
{
    TEST_ASSERT_FLOAT_EQ(m->m[12], x, 0.0005f, whatX);
    TEST_ASSERT_FLOAT_EQ(m->m[13], y, 0.0005f, whatY);
    TEST_ASSERT_FLOAT_EQ(m->m[14], z, 0.0005f, whatZ);
}

static JoltC_Quat identity_quat(void)
{
    JoltC_Quat q;
    q.x = 0.0f; q.y = 0.0f; q.z = 0.0f; q.w = 1.0f;
    return q;
}

/* ========================================================================== */
/*  Skeleton hierarchy                                                        */
/* ========================================================================== */

/* The two AddJoint overloads differ in a way no compiler can catch: passing a parent by
 * index resolves the index there and then, passing a parent by name leaves the index at -1
 * until CalculateParentJointIndices() runs. A repair that routes AddJoint3 through
 * AddJoint2 (or vice versa) compiles, keeps the joint count right, and silently changes
 * every algorithm downstream that reads mParentJointIndex. */
static void test_parent_index_resolution(void)
{
    TEST_BEGIN("AddJoint by index resolves now, by name resolves on demand");
    JoltC_Skeleton* sk = JoltC_Skeleton_Create();
    TEST_ASSERT_NOT_NULL(sk, "skeleton created");

    if (sk) {
        JoltC_SkeletonJoint joint;
        uint32_t pelvis = JoltC_Skeleton_AddJoint(sk, "pelvis");
        uint32_t spine  = JoltC_Skeleton_AddJoint2(sk, "spine", 0);
        uint32_t neck   = JoltC_Skeleton_AddJoint3(sk, "neck", "spine");

        TEST_ASSERT(pelvis == 0, "pelvis is joint 0");
        TEST_ASSERT(spine == 1, "AddJoint2 returns the new index, not the parent index");
        TEST_ASSERT(neck == 2, "AddJoint3 returns the new index");
        TEST_ASSERT(JoltC_Skeleton_GetJointCount(sk) == 3, "three joints");

        /* GetJoint hands back pointers into a shared per-thread buffer, so each result is
         * read before the next call rather than held. */
        JoltC_Skeleton_GetJoint(sk, 0, &joint);
        TEST_ASSERT(strcmp(joint.name, "pelvis") == 0, "joint 0 is pelvis");
        TEST_ASSERT(joint.parentJointIndex == -1, "a root joint has parent index -1");
        TEST_ASSERT(joint.parentName != NULL && joint.parentName[0] == '\0',
                    "a root joint has an empty parent name");

        JoltC_Skeleton_GetJoint(sk, 1, &joint);
        TEST_ASSERT(strcmp(joint.name, "spine") == 0, "joint 1 is spine");
        TEST_ASSERT(joint.parentJointIndex == 0,
                    "AddJoint2 sets the parent index immediately");
        TEST_ASSERT(strcmp(joint.parentName, "pelvis") == 0,
                    "AddJoint2 back-fills the parent name from the index");

        JoltC_Skeleton_GetJoint(sk, 2, &joint);
        TEST_ASSERT(strcmp(joint.name, "neck") == 0, "joint 2 is neck");
        TEST_ASSERT(strcmp(joint.parentName, "spine") == 0,
                    "AddJoint3 stores the parent name it was given");
        TEST_ASSERT(joint.parentJointIndex == -1,
                    "AddJoint3 leaves the parent index unresolved");

        JoltC_Skeleton_CalculateParentJointIndices(sk);

        JoltC_Skeleton_GetJoint(sk, 2, &joint);
        TEST_ASSERT(joint.parentJointIndex == 1,
                    "CalculateParentJointIndices resolves neck to spine");
        JoltC_Skeleton_GetJoint(sk, 1, &joint);
        TEST_ASSERT(joint.parentJointIndex == 0,
                    "CalculateParentJointIndices leaves an already-correct index alone");
        JoltC_Skeleton_GetJoint(sk, 0, &joint);
        TEST_ASSERT(joint.parentJointIndex == -1,
                    "an empty parent name resolves to -1, not to joint 0");

        TEST_ASSERT(JoltC_Skeleton_GetJointIndex(sk, "spine") == 1, "name lookup finds spine");
        TEST_ASSERT(JoltC_Skeleton_GetJointIndex(sk, "sPiNe") == -1,
                    "name lookup is case sensitive and misses report -1");
        TEST_ASSERT(JoltC_Skeleton_GetJointIndex(sk, "") == -1,
                    "the empty name is not a joint");

        JoltC_Skeleton_Destroy(sk);
    }
    TEST_END();
}

/* test_skeleton.c only ever sees AreJointsCorrectlyOrdered return true, and it returns
 * true for the wrong reason there: with unresolved parent indices every joint looks like a
 * root. This forces a real negative. */
static void test_joint_ordering_detects_a_child_before_its_parent(void)
{
    TEST_BEGIN("AreJointsCorrectlyOrdered rejects a child stored before its parent");
    JoltC_Skeleton* sk = JoltC_Skeleton_Create();
    if (sk) {
        /* Deliberately backwards: the child goes in first and names a parent that does not
         * exist yet. AddJoint3 does no lookup, so this is legal to build. */
        JoltC_Skeleton_AddJoint3(sk, "forearm", "upperarm");
        JoltC_Skeleton_AddJoint(sk, "upperarm");

        TEST_ASSERT(JoltC_Skeleton_AreJointsCorrectlyOrdered(sk) != 0,
                    "before resolution every parent index is -1, so ordering looks fine");

        JoltC_Skeleton_CalculateParentJointIndices(sk);
        TEST_ASSERT(JoltC_Skeleton_AreJointsCorrectlyOrdered(sk) == 0,
                    "once resolved, joint 0 pointing at joint 1 is rejected");

        JoltC_Skeleton_Destroy(sk);
    }
    TEST_END();
}

/* ========================================================================== */
/*  SkeletonPose                                                              */
/* ========================================================================== */

static void test_pose_skeleton_and_root_offset(void)
{
    TEST_BEGIN("SkeletonPose skeleton handle and root offset round-trip");
    JoltC_Skeleton* sk = JoltC_Skeleton_Create();
    JoltC_SkeletonPose* pose = JoltC_SkeletonPose_Create();

    if (sk && pose) {
        JoltC_RVec3 offset;
        JoltC_RVec3 got;

        TEST_ASSERT(JoltC_SkeletonPose_GetSkeleton(pose) == NULL,
                    "a fresh pose has no skeleton");
        TEST_ASSERT(JoltC_SkeletonPose_GetJointCount(pose) == 0,
                    "a fresh pose has no joints");

        JoltC_Skeleton_AddJoint(sk, "hips");
        JoltC_Skeleton_AddJoint2(sk, "chest", 0);
        JoltC_Skeleton_AddJoint2(sk, "head", 1);

        JoltC_SkeletonPose_SetSkeleton(pose, sk);
        TEST_ASSERT(JoltC_SkeletonPose_GetSkeleton(pose) == sk,
                    "GetSkeleton returns the handle that was set");
        TEST_ASSERT(JoltC_SkeletonPose_GetJointCount(pose) == 3,
                    "SetSkeleton sizes the pose to the skeleton");

        /* The root offset is stored verbatim; upstream applies it in Draw and in Ragdoll,
         * not in CalculateJointMatrices, so a pure round-trip is the correct expectation. */
        offset.x = 1.5f;
        offset.y = -2.25f;
        offset.z = 3.75f;
        JoltC_SkeletonPose_SetRootOffset(pose, offset);
        got = JoltC_SkeletonPose_GetRootOffset(pose);
        TEST_ASSERT_FLOAT_EQ(got.x, 1.5f, 0.0005f, "root offset x survives");
        TEST_ASSERT_FLOAT_EQ(got.y, -2.25f, 0.0005f, "root offset y survives, sign included");
        TEST_ASSERT_FLOAT_EQ(got.z, 3.75f, 0.0005f, "root offset z survives and is not x");
    }

    if (pose) JoltC_SkeletonPose_Destroy(pose);
    if (sk) JoltC_Skeleton_Destroy(sk);
    TEST_END();
}

/* Two joints written with values that are nowhere near each other, then both read back.
 * A single-joint test cannot tell a correct index from one that always writes slot 0. */
static void test_pose_joint_state_indexing(void)
{
    TEST_BEGIN("SkeletonPose joint states are stored per index");
    JoltC_Skeleton* sk = JoltC_Skeleton_Create();
    JoltC_SkeletonPose* pose = JoltC_SkeletonPose_Create();

    if (sk && pose) {
        JoltC_Vec3 t0, t1, outT;
        JoltC_Quat r0, r1, outR;

        JoltC_Skeleton_AddJoint(sk, "hips");
        JoltC_Skeleton_AddJoint2(sk, "chest", 0);
        JoltC_SkeletonPose_SetSkeleton(pose, sk);

        t0.x = 1.5f;  t0.y = -2.25f; t0.z = 3.75f;
        t1.x = -6.5f; t1.y = 8.125f; t1.z = -0.375f;
        r0 = identity_quat();
        /* All four components distinct and non-zero: (1,2,3,4) normalised by sqrt(30). */
        r1.x = 0.182574f; r1.y = 0.365148f; r1.z = 0.547723f; r1.w = 0.730297f;

        JoltC_SkeletonPose_SetJointState(pose, 0, t0, r0);
        JoltC_SkeletonPose_SetJointState(pose, 1, t1, r1);

        JoltC_SkeletonPose_GetJointState(pose, 0, &outT, &outR);
        TEST_ASSERT_FLOAT_EQ(outT.x, 1.5f, 0.0005f, "joint 0 translation x");
        TEST_ASSERT_FLOAT_EQ(outT.y, -2.25f, 0.0005f, "joint 0 translation y");
        TEST_ASSERT_FLOAT_EQ(outT.z, 3.75f, 0.0005f, "joint 0 translation z");
        TEST_ASSERT_FLOAT_EQ(outR.w, 1.0f, 0.0005f, "joint 0 rotation is still identity");

        JoltC_SkeletonPose_GetJointState(pose, 1, &outT, &outR);
        TEST_ASSERT_FLOAT_EQ(outT.x, -6.5f, 0.0005f,
                             "joint 1 translation x is not joint 0's");
        TEST_ASSERT_FLOAT_EQ(outT.y, 8.125f, 0.0005f, "joint 1 translation y");
        TEST_ASSERT_FLOAT_EQ(outT.z, -0.375f, 0.0005f, "joint 1 translation z");
        TEST_ASSERT_FLOAT_EQ(outR.x, 0.182574f, 0.0005f, "joint 1 rotation x");
        TEST_ASSERT_FLOAT_EQ(outR.y, 0.365148f, 0.0005f, "joint 1 rotation y");
        TEST_ASSERT_FLOAT_EQ(outR.z, 0.547723f, 0.0005f, "joint 1 rotation z");
        TEST_ASSERT_FLOAT_EQ(outR.w, 0.730297f, 0.0005f,
                             "joint 1 rotation w is not rotated into x");

        /* Each output pointer is optional and checked separately by the wrapper. */
        outT.x = -99.0f;
        JoltC_SkeletonPose_GetJointState(pose, 1, &outT, NULL);
        TEST_ASSERT_FLOAT_EQ(outT.x, -6.5f, 0.0005f,
                             "a null rotation pointer still fills the translation");

        /* Out of range is a no-op, not a write and not a crash. */
        outT.x = -99.0f;
        outR.w = -99.0f;
        JoltC_SkeletonPose_GetJointState(pose, 17, &outT, &outR);
        TEST_ASSERT_FLOAT_EQ(outT.x, -99.0f, 0.0005f,
                             "an out of range read leaves the output alone");
        JoltC_SkeletonPose_SetJointState(pose, -3, t0, r0);  /* must not crash */
    }

    if (pose) JoltC_SkeletonPose_Destroy(pose);
    if (sk) JoltC_Skeleton_Destroy(sk);
    TEST_END();
}

/* The single-matrix and bulk accessors are separate code paths over the same array. They
 * must agree, and both must clamp to the joint count rather than to the caller's count. */
static void test_pose_joint_matrix_accessors(void)
{
    TEST_BEGIN("SkeletonPose joint matrices: single, bulk and clamping");
    JoltC_Skeleton* sk = JoltC_Skeleton_Create();
    JoltC_SkeletonPose* pose = JoltC_SkeletonPose_Create();

    if (sk && pose) {
        JoltC_Mat44 in[4];
        JoltC_Mat44 out[4];
        JoltC_Mat44 single;

        JoltC_Skeleton_AddJoint(sk, "hips");
        JoltC_Skeleton_AddJoint2(sk, "chest", 0);
        JoltC_Skeleton_AddJoint2(sk, "head", 1);
        JoltC_SkeletonPose_SetSkeleton(pose, sk);

        make_translation_matrix(&in[0], 1.5f, -2.25f, 3.75f);
        make_translation_matrix(&in[1], -4.5f, 6.75f, -8.25f);
        make_translation_matrix(&in[2], 0.375f, 0.125f, -0.625f);
        make_translation_matrix(&in[3], 111.0f, 222.0f, 333.0f);  /* one past the end */

        /* Deliberately over-long: the pose has three joints, so the fourth must be
         * ignored. Writing it would corrupt whatever follows the array in Jolt's heap. */
        JoltC_SkeletonPose_SetJointMatrices(pose, in, 4);

        JoltC_SkeletonPose_GetJointMatrix(pose, 0, &single);
        assert_matrix_translation(&single, 1.5f, -2.25f, 3.75f,
                                  "matrix 0 x", "matrix 0 y", "matrix 0 z");
        JoltC_SkeletonPose_GetJointMatrix(pose, 2, &single);
        assert_matrix_translation(&single, 0.375f, 0.125f, -0.625f,
                                  "matrix 2 x", "matrix 2 y", "matrix 2 z");
        TEST_ASSERT_FLOAT_EQ(single.m[15], 1.0f, 0.0005f,
                             "the homogeneous element survives the round-trip");
        TEST_ASSERT_FLOAT_EQ(single.m[0], 1.0f, 0.0005f, "the rotation part is identity");
        TEST_ASSERT_FLOAT_EQ(single.m[1], 0.0f, 0.0005f,
                             "the translation did not land in the first column");

        /* Bulk read of the full set must agree with the per-index reads. */
        fill_matrix_with(&out[0], -99.0f);
        fill_matrix_with(&out[1], -99.0f);
        fill_matrix_with(&out[2], -99.0f);
        fill_matrix_with(&out[3], -99.0f);
        JoltC_SkeletonPose_GetJointMatrices(pose, out, 4);
        assert_matrix_translation(&out[0], 1.5f, -2.25f, 3.75f,
                                  "bulk matrix 0 x", "bulk matrix 0 y", "bulk matrix 0 z");
        assert_matrix_translation(&out[1], -4.5f, 6.75f, -8.25f,
                                  "bulk matrix 1 x", "bulk matrix 1 y", "bulk matrix 1 z");
        assert_matrix_translation(&out[2], 0.375f, 0.125f, -0.625f,
                                  "bulk matrix 2 x", "bulk matrix 2 y", "bulk matrix 2 z");
        TEST_ASSERT_FLOAT_EQ(out[3].m[12], -99.0f, 0.0005f,
                             "a read of 4 from a 3 joint pose leaves the 4th untouched");

        /* And a short read must stop where it was told to. */
        fill_matrix_with(&out[2], -77.0f);
        JoltC_SkeletonPose_GetJointMatrices(pose, out, 2);
        TEST_ASSERT_FLOAT_EQ(out[2].m[12], -77.0f, 0.0005f,
                             "a read of 2 does not write the third slot");

        /* Single-matrix write hits only the index it names. */
        make_translation_matrix(&single, -1.0f, 2.5f, -3.125f);
        JoltC_SkeletonPose_SetJointMatrix(pose, 1, &single);
        JoltC_SkeletonPose_GetJointMatrix(pose, 1, &single);
        assert_matrix_translation(&single, -1.0f, 2.5f, -3.125f,
                                  "overwritten matrix 1 x", "overwritten matrix 1 y",
                                  "overwritten matrix 1 z");
        JoltC_SkeletonPose_GetJointMatrix(pose, 0, &single);
        assert_matrix_translation(&single, 1.5f, -2.25f, 3.75f,
                                  "matrix 0 x untouched", "matrix 0 y untouched",
                                  "matrix 0 z untouched");

        /* Out of range on both accessors must not write and must not crash. */
        fill_matrix_with(&single, -55.0f);
        JoltC_SkeletonPose_GetJointMatrix(pose, 9, &single);
        TEST_ASSERT_FLOAT_EQ(single.m[12], -55.0f, 0.0005f,
                             "an out of range matrix read leaves the output alone");
        JoltC_SkeletonPose_SetJointMatrix(pose, -2, &in[0]);  /* must not crash */
    }

    if (pose) JoltC_SkeletonPose_Destroy(pose);
    if (sk) JoltC_Skeleton_Destroy(sk);
    TEST_END();
}

/* Pure algebra with a three-link chain and identity rotations, so composition reduces to
 * adding translations and every expected value is exact. This is the test that catches a
 * parent/child inversion in CalculateJointMatrices, and it also pins down the difference
 * between the model-space and local-space calls, which is easy to conflate. */
static void test_pose_matrix_composition_round_trip(void)
{
    TEST_BEGIN("CalculateJointMatrices composes parents, local space does not");
    JoltC_Skeleton* sk = JoltC_Skeleton_Create();
    JoltC_SkeletonPose* pose = JoltC_SkeletonPose_Create();

    if (sk && pose) {
        JoltC_Vec3 local0, local1, local2, outT;
        JoltC_Quat identity = identity_quat();
        JoltC_Quat outR;
        JoltC_Mat44 model;
        JoltC_Mat44 localSpace[3];

        /* Parents by index, so the indices are resolved without a separate call. */
        JoltC_Skeleton_AddJoint(sk, "hips");
        JoltC_Skeleton_AddJoint2(sk, "chest", 0);
        JoltC_Skeleton_AddJoint2(sk, "head", 1);
        TEST_ASSERT(JoltC_Skeleton_AreJointsCorrectlyOrdered(sk) != 0,
                    "the chain is ordered parents first");

        JoltC_SkeletonPose_SetSkeleton(pose, sk);

        local0.x = 1.5f;  local0.y = -2.25f; local0.z = 3.75f;
        local1.x = 0.5f;  local1.y = 0.25f;  local1.z = -0.75f;
        local2.x = -1.0f; local2.y = 2.0f;   local2.z = 4.0f;

        JoltC_SkeletonPose_SetJointState(pose, 0, local0, identity);
        JoltC_SkeletonPose_SetJointState(pose, 1, local1, identity);
        JoltC_SkeletonPose_SetJointState(pose, 2, local2, identity);

        /* Local space is per joint and hierarchy-free: each entry is just that joint's own
         * transform. Upstream's CalculateLocalSpaceJointMatrices does no composition. */
        JoltC_SkeletonPose_CalculateLocalSpaceJointMatrices(pose, localSpace);
        assert_matrix_translation(&localSpace[0], 1.5f, -2.25f, 3.75f,
                                  "local 0 x", "local 0 y", "local 0 z");
        assert_matrix_translation(&localSpace[1], 0.5f, 0.25f, -0.75f,
                                  "local 1 x stays local", "local 1 y stays local",
                                  "local 1 z stays local");
        assert_matrix_translation(&localSpace[2], -1.0f, 2.0f, 4.0f,
                                  "local 2 x stays local", "local 2 y stays local",
                                  "local 2 z stays local");

        /* Model space accumulates down the chain. With identity rotations the sums are
         * exact: (1.5,-2.25,3.75), then +(0.5,0.25,-0.75), then +(-1,2,4). */
        JoltC_SkeletonPose_CalculateJointMatrices(pose);
        JoltC_SkeletonPose_GetJointMatrix(pose, 0, &model);
        assert_matrix_translation(&model, 1.5f, -2.25f, 3.75f,
                                  "model 0 x", "model 0 y", "model 0 z");
        JoltC_SkeletonPose_GetJointMatrix(pose, 1, &model);
        assert_matrix_translation(&model, 2.0f, -2.0f, 3.0f,
                                  "model 1 x includes the parent",
                                  "model 1 y includes the parent",
                                  "model 1 z includes the parent");
        JoltC_SkeletonPose_GetJointMatrix(pose, 2, &model);
        assert_matrix_translation(&model, 1.0f, 0.0f, 7.0f,
                                  "model 2 x accumulates the whole chain",
                                  "model 2 y accumulates the whole chain",
                                  "model 2 z accumulates the whole chain");

        /* And back again. CalculateJointStates is documented as the inverse operation, so
         * the local translations must reappear exactly. */
        JoltC_SkeletonPose_CalculateJointStates(pose);
        JoltC_SkeletonPose_GetJointState(pose, 1, &outT, &outR);
        TEST_ASSERT_FLOAT_EQ(outT.x, 0.5f, 0.0005f, "joint 1 local x recovered");
        TEST_ASSERT_FLOAT_EQ(outT.y, 0.25f, 0.0005f, "joint 1 local y recovered");
        TEST_ASSERT_FLOAT_EQ(outT.z, -0.75f, 0.0005f, "joint 1 local z recovered");
        JoltC_SkeletonPose_GetJointState(pose, 2, &outT, &outR);
        TEST_ASSERT_FLOAT_EQ(outT.x, -1.0f, 0.0005f, "joint 2 local x recovered");
        TEST_ASSERT_FLOAT_EQ(outT.y, 2.0f, 0.0005f, "joint 2 local y recovered");
        TEST_ASSERT_FLOAT_EQ(outT.z, 4.0f, 0.0005f, "joint 2 local z recovered");
        TEST_ASSERT_FLOAT_EQ(outR.w, 1.0f, 0.0005f, "joint 2 rotation stays identity");
    }

    if (pose) JoltC_SkeletonPose_Destroy(pose);
    if (sk) JoltC_Skeleton_Destroy(sk);
    TEST_END();
}

/* ========================================================================== */
/*  SkeletalAnimation                                                         */
/* ========================================================================== */

/* The animation joints are added in the opposite order to the skeleton joints on purpose.
 * Sample() looks the target joint up by name, so a wrapper that used the animation's own
 * joint ordering as a pose index would put the values in the wrong slots and this test
 * would fail. It is also the only place where GetDuration's "joint 0 only" rule is
 * observable. */
static void test_animation_keyframes_and_sampling(void)
{
    TEST_BEGIN("SkeletalAnimation duration, keyframes, sampling and scaling");
    JoltC_SkeletalAnimation* anim = JoltC_SkeletalAnimation_Create();
    JoltC_Skeleton* sk = JoltC_Skeleton_Create();
    JoltC_SkeletonPose* pose = JoltC_SkeletonPose_Create();

    if (anim && sk && pose) {
        JoltC_Quat identity = identity_quat();
        JoltC_Vec3 chestFirst, chestLast, hipsFirst, hipsLast, outT;
        JoltC_Quat outR;

        /* Upstream defaults mIsLooping to true, so this is read before it is written. */
        TEST_ASSERT(JoltC_SkeletalAnimation_IsLooping(anim) != 0,
                    "a new animation loops by default");
        TEST_ASSERT_FLOAT_EQ(JoltC_SkeletalAnimation_GetDuration(anim), 0.0f, 0.0005f,
                             "an empty animation has no duration");

        /* Animation order: chest then hips. Skeleton order: hips then chest. */
        JoltC_SkeletalAnimation_AddAnimatedJoint(anim, "chest");
        JoltC_SkeletalAnimation_AddAnimatedJoint(anim, "hips");
        TEST_ASSERT(JoltC_SkeletalAnimation_GetAnimatedJointCount(anim) == 2,
                    "two animated joints");
        TEST_ASSERT_FLOAT_EQ(JoltC_SkeletalAnimation_GetDuration(anim), 0.0f, 0.0005f,
                             "joints without keyframes still have no duration");

        chestFirst.x = -1.5f; chestFirst.y = 0.25f; chestFirst.z = 3.0f;
        chestLast.x  = -0.5f; chestLast.y  = 2.25f; chestLast.z  = 1.0f;
        hipsFirst.x  = 0.0f;  hipsFirst.y  = 0.0f;  hipsFirst.z  = 0.0f;
        hipsLast.x   = 4.0f;  hipsLast.y   = -8.0f; hipsLast.z   = 2.0f;

        JoltC_SkeletalAnimation_AddKeyframe(anim, 0, 0.0f, chestFirst, identity);
        JoltC_SkeletalAnimation_AddKeyframe(anim, 0, 0.5f, chestLast, identity);
        JoltC_SkeletalAnimation_AddKeyframe(anim, 1, 0.0f, hipsFirst, identity);
        JoltC_SkeletalAnimation_AddKeyframe(anim, 1, 0.5f, hipsLast, identity);

        TEST_ASSERT_FLOAT_EQ(JoltC_SkeletalAnimation_GetDuration(anim), 0.5f, 0.0005f,
                             "duration is the last keyframe time of animated joint 0");

        /* An out of range joint index is guarded and changes nothing. */
        JoltC_SkeletalAnimation_AddKeyframe(anim, 6, 9.0f, hipsLast, identity);
        JoltC_SkeletalAnimation_AddKeyframe(anim, -1, 9.0f, hipsLast, identity);
        TEST_ASSERT_FLOAT_EQ(JoltC_SkeletalAnimation_GetDuration(anim), 0.5f, 0.0005f,
                             "a keyframe on a joint that does not exist is dropped");

        JoltC_Skeleton_AddJoint(sk, "hips");
        JoltC_Skeleton_AddJoint2(sk, "chest", 0);
        JoltC_SkeletonPose_SetSkeleton(pose, sk);

        /* Exactly halfway between the two keyframes: linear for translation, so exact. */
        JoltC_SkeletalAnimation_Sample(anim, 0.25f, pose);
        JoltC_SkeletonPose_GetJointState(pose, 0, &outT, &outR);
        TEST_ASSERT_FLOAT_EQ(outT.x, 2.0f, 0.0005f, "hips lands on skeleton slot 0, x");
        TEST_ASSERT_FLOAT_EQ(outT.y, -4.0f, 0.0005f, "hips lands on skeleton slot 0, y");
        TEST_ASSERT_FLOAT_EQ(outT.z, 1.0f, 0.0005f, "hips lands on skeleton slot 0, z");
        JoltC_SkeletonPose_GetJointState(pose, 1, &outT, &outR);
        TEST_ASSERT_FLOAT_EQ(outT.x, -1.0f, 0.0005f, "chest lands on skeleton slot 1, x");
        TEST_ASSERT_FLOAT_EQ(outT.y, 1.25f, 0.0005f, "chest lands on skeleton slot 1, y");
        TEST_ASSERT_FLOAT_EQ(outT.z, 2.0f, 0.0005f, "chest lands on skeleton slot 1, z");
        TEST_ASSERT_FLOAT_EQ(outR.w, 1.0f, 0.0005f,
                             "slerp between two identities is identity");

        /* Looping is on, so a time of 2.0 wraps to fmod(2.0, 0.5) == 0.0 and returns the
         * first keyframes. Turning looping off makes the same time clamp to the last. */
        JoltC_SkeletalAnimation_Sample(anim, 2.0f, pose);
        JoltC_SkeletonPose_GetJointState(pose, 0, &outT, &outR);
        TEST_ASSERT_FLOAT_EQ(outT.x, 0.0f, 0.0005f, "looping wraps 2.0 back to the start");
        TEST_ASSERT_FLOAT_EQ(outT.y, 0.0f, 0.0005f, "looping wraps 2.0 back to the start, y");

        JoltC_SkeletalAnimation_SetIsLooping(anim, JOLTC_FALSE);
        TEST_ASSERT(JoltC_SkeletalAnimation_IsLooping(anim) == 0, "looping turned off");
        JoltC_SkeletalAnimation_Sample(anim, 2.0f, pose);
        JoltC_SkeletonPose_GetJointState(pose, 0, &outT, &outR);
        TEST_ASSERT_FLOAT_EQ(outT.x, 4.0f, 0.0005f,
                             "without looping, past the end clamps to the last keyframe");
        TEST_ASSERT_FLOAT_EQ(outT.y, -8.0f, 0.0005f,
                             "the clamped y is the last keyframe, not the first");
        JoltC_SkeletonPose_GetJointState(pose, 1, &outT, &outR);
        TEST_ASSERT_FLOAT_EQ(outT.x, -0.5f, 0.0005f, "chest also clamps to its last key");
        TEST_ASSERT_FLOAT_EQ(outT.y, 2.25f, 0.0005f, "chest clamped y");

        /* ScaleJoints touches keyframe translations and nothing else, so the duration must
         * not move. A repair that scaled mTime instead would show up right here. */
        JoltC_SkeletalAnimation_ScaleJoints(anim, 0.5f);
        TEST_ASSERT_FLOAT_EQ(JoltC_SkeletalAnimation_GetDuration(anim), 0.5f, 0.0005f,
                             "scaling joints does not scale keyframe times");
        JoltC_SkeletalAnimation_Sample(anim, 2.0f, pose);
        JoltC_SkeletonPose_GetJointState(pose, 0, &outT, &outR);
        TEST_ASSERT_FLOAT_EQ(outT.x, 2.0f, 0.0005f, "hips translation halved, x");
        TEST_ASSERT_FLOAT_EQ(outT.y, -4.0f, 0.0005f, "hips translation halved, y");
        TEST_ASSERT_FLOAT_EQ(outT.z, 1.0f, 0.0005f, "hips translation halved, z");
        JoltC_SkeletonPose_GetJointState(pose, 1, &outT, &outR);
        TEST_ASSERT_FLOAT_EQ(outT.y, 1.125f, 0.0005f, "chest translation halved");
        TEST_ASSERT_FLOAT_EQ(outR.w, 1.0f, 0.0005f, "scaling leaves rotations alone");
    }

    if (pose) JoltC_SkeletonPose_Destroy(pose);
    if (sk) JoltC_Skeleton_Destroy(sk);
    if (anim) JoltC_SkeletalAnimation_Destroy(anim);
    TEST_END();
}

/* ========================================================================== */
/*  SkeletonMapper                                                            */
/* ========================================================================== */

/* Skeleton 1 must be the low detail one; upstream asserts it. Skeleton 2 here carries an
 * extra sibling leaf ("tail") so that "chest" maps from index 1 to index 2, which is what
 * makes an exchange of the two skeleton arguments detectable. The extra joint is a leaf
 * rather than an intermediate on purpose: an intermediate would build a chain, and chain
 * mapping runs Quat::sFromTo over pose directions, which is far more than a binding test
 * should be pinning down. */
static void build_mapper_skeletons(JoltC_Skeleton** outLow, JoltC_Skeleton** outHigh)
{
    JoltC_Skeleton* low = JoltC_Skeleton_Create();
    JoltC_Skeleton* high = JoltC_Skeleton_Create();

    if (low) {
        JoltC_Skeleton_AddJoint(low, "hips");
        JoltC_Skeleton_AddJoint2(low, "chest", 0);
    }
    if (high) {
        JoltC_Skeleton_AddJoint(high, "hips");
        JoltC_Skeleton_AddJoint2(high, "tail", 0);
        JoltC_Skeleton_AddJoint2(high, "chest", 0);
    }
    *outLow = low;
    *outHigh = high;
}

static void build_mapper_neutral_poses(JoltC_Mat44 low[2], JoltC_Mat44 high[3])
{
    /* The mapped joints sit at identical model space positions in both skeletons, so every
     * joint-1-to-2 transform comes out as identity and Map becomes a straight copy. */
    make_translation_matrix(&low[0], 0.0f, 0.0f, 0.0f);
    make_translation_matrix(&low[1], 0.0f, 1.0f, 0.0f);
    make_translation_matrix(&high[0], 0.0f, 0.0f, 0.0f);
    make_translation_matrix(&high[1], 0.5f, 0.0f, 0.0f);
    make_translation_matrix(&high[2], 0.0f, 1.0f, 0.0f);
}

static void test_mapper_initialize_and_map(void)
{
    TEST_BEGIN("SkeletonMapper initialize, map and reverse map");
    JoltC_Skeleton* low = NULL;
    JoltC_Skeleton* high = NULL;
    JoltC_SkeletonMapper* mapper = JoltC_SkeletonMapper_Create();

    build_mapper_skeletons(&low, &high);
    TEST_ASSERT_NOT_NULL(mapper, "mapper created");

    if (mapper && low && high) {
        JoltC_Mat44 neutralLow[2];
        JoltC_Mat44 neutralHigh[3];
        JoltC_Mat44 pose1Model[2];
        JoltC_Mat44 pose2Local[3];
        JoltC_Mat44 pose2Model[3];
        JoltC_Mat44 backToPose1[2];

        build_mapper_neutral_poses(neutralLow, neutralHigh);
        JoltC_SkeletonMapper_Initialize(mapper, low, neutralLow, high, neutralHigh);

        /* "hips" is 0 in both. "chest" is 1 in the low detail skeleton and 2 in the high
         * detail one, so this pair of assertions is what an exchanged sk1/sk2 fails. */
        TEST_ASSERT(JoltC_SkeletonMapper_GetMappedJointIndex(mapper, 0) == 0,
                    "hips maps to hips");
        TEST_ASSERT(JoltC_SkeletonMapper_GetMappedJointIndex(mapper, 1) == 2,
                    "chest maps from low index 1 to high index 2");
        TEST_ASSERT(JoltC_SkeletonMapper_GetMappedJointIndex(mapper, 4) == -1,
                    "a joint that does not exist maps to -1");

        make_translation_matrix(&pose1Model[0], 1.5f, -2.25f, 3.75f);
        make_translation_matrix(&pose1Model[1], 1.5f, -1.25f, 3.75f);
        make_translation_matrix(&pose2Local[0], 0.0f, 0.0f, 0.0f);
        make_translation_matrix(&pose2Local[1], 0.5f, 0.0f, -0.25f);
        make_translation_matrix(&pose2Local[2], 0.0f, 0.0f, 0.0f);
        fill_matrix_with(&pose2Model[0], -99.0f);
        fill_matrix_with(&pose2Model[1], -99.0f);
        fill_matrix_with(&pose2Model[2], -99.0f);

        JoltC_SkeletonMapper_Map(mapper, pose1Model, pose2Local, pose2Model);

        /* Mapped joints are copied in model space. The unmapped leaf gets its parent's
         * model transform times its own local transform. */
        assert_matrix_translation(&pose2Model[0], 1.5f, -2.25f, 3.75f,
                                  "mapped hips x", "mapped hips y", "mapped hips z");
        assert_matrix_translation(&pose2Model[2], 1.5f, -1.25f, 3.75f,
                                  "mapped chest x", "mapped chest y", "mapped chest z");
        assert_matrix_translation(&pose2Model[1], 2.0f, -2.25f, 3.5f,
                                  "unmapped tail x is parent times local",
                                  "unmapped tail y is parent times local",
                                  "unmapped tail z is parent times local");

        /* MapReverse uses only the direct mappings, so it must pull high index 2 back into
         * low index 1 and reproduce the input exactly. */
        fill_matrix_with(&backToPose1[0], -99.0f);
        fill_matrix_with(&backToPose1[1], -99.0f);
        JoltC_SkeletonMapper_MapReverse(mapper, pose2Model, backToPose1);
        assert_matrix_translation(&backToPose1[0], 1.5f, -2.25f, 3.75f,
                                  "reverse hips x", "reverse hips y", "reverse hips z");
        assert_matrix_translation(&backToPose1[1], 1.5f, -1.25f, 3.75f,
                                  "reverse chest x comes from high index 2",
                                  "reverse chest y comes from high index 2",
                                  "reverse chest z comes from high index 2");
    }

    if (mapper) JoltC_SkeletonMapper_Destroy(mapper);
    if (high) JoltC_Skeleton_Destroy(high);
    if (low) JoltC_Skeleton_Destroy(low);
    TEST_END();
}

static void test_mapper_locked_translations(void)
{
    TEST_BEGIN("SkeletonMapper locks the translations it is asked to");
    JoltC_Skeleton* low = NULL;
    JoltC_Skeleton* high = NULL;
    /* Two mappers: LockAllTranslations mutates the mapper's state and would change what
     * Map produces, so it does not share a mapper with the explicit-array case. */
    JoltC_SkeletonMapper* autoMapper = JoltC_SkeletonMapper_Create();
    JoltC_SkeletonMapper* explicitMapper = JoltC_SkeletonMapper_Create();

    build_mapper_skeletons(&low, &high);

    if (autoMapper && explicitMapper && low && high) {
        JoltC_Mat44 neutralLow[2];
        JoltC_Mat44 neutralHigh[3];
        JoltC_Bool locked[3];

        build_mapper_neutral_poses(neutralLow, neutralHigh);

        TEST_ASSERT(JoltC_SkeletonMapper_IsJointTranslationLocked(autoMapper, 1) == 0,
                    "nothing is locked before any lock call");

        /* LockAllTranslations requires Initialize first and deliberately excludes the
         * first mapped joint, because that joint is what positions the whole ragdoll. */
        JoltC_SkeletonMapper_Initialize(autoMapper, low, neutralLow, high, neutralHigh);
        JoltC_SkeletonMapper_LockAllTranslations(autoMapper, high, neutralHigh);
        TEST_ASSERT(JoltC_SkeletonMapper_IsJointTranslationLocked(autoMapper, 0) == 0,
                    "the root mapped joint stays unlocked");
        TEST_ASSERT(JoltC_SkeletonMapper_IsJointTranslationLocked(autoMapper, 1) != 0,
                    "a child of the root is locked");
        TEST_ASSERT(JoltC_SkeletonMapper_IsJointTranslationLocked(autoMapper, 2) != 0,
                    "the other child of the root is locked");

        /* The explicit form takes one flag per joint of skeleton 2. Only the last is set,
         * so a wrapper that read the array at the wrong offset, or that ignored it and
         * locked everything, fails two of these three. */
        locked[0] = JOLTC_FALSE;
        locked[1] = JOLTC_FALSE;
        locked[2] = JOLTC_TRUE;
        JoltC_SkeletonMapper_LockTranslations(explicitMapper, high, locked, neutralHigh);
        TEST_ASSERT(JoltC_SkeletonMapper_IsJointTranslationLocked(explicitMapper, 0) == 0,
                    "joint 0 was not requested and is not locked");
        TEST_ASSERT(JoltC_SkeletonMapper_IsJointTranslationLocked(explicitMapper, 1) == 0,
                    "joint 1 was not requested and is not locked");
        TEST_ASSERT(JoltC_SkeletonMapper_IsJointTranslationLocked(explicitMapper, 2) != 0,
                    "joint 2 was requested and is locked");
    }

    if (explicitMapper) JoltC_SkeletonMapper_Destroy(explicitMapper);
    if (autoMapper) JoltC_SkeletonMapper_Destroy(autoMapper);
    if (high) JoltC_Skeleton_Destroy(high);
    if (low) JoltC_Skeleton_Destroy(low);
    TEST_END();
}

/* ========================================================================== */
/*  RagdollSettings and Ragdoll                                               */
/* ========================================================================== */

/* Part positions, well apart and with mixed signs. */
#define PART0_X   1.5f
#define PART0_Y  -2.25f
#define PART0_Z   3.75f
#define PART1_X  -4.5f
#define PART1_Y   6.75f
#define PART1_Z  -8.25f

#define RAGDOLL_GROUP_ID  7u
#define RAGDOLL_USER_DATA 0xA1B2C3D4E5F60718ULL

/* The smallest ragdoll a C caller can build: skeleton, parts, shapes, layers, positions,
 * and nothing else. Deliberately does not call Stabilize or DisableParentChildCollisions.
 *
 * It exists because the fuller test below segfaults inside CreateRagdoll -- located with
 * markers, since CTest reports the whole suite as one test. Two hypotheses fit that: either
 * CreateRagdoll cannot cope with settings a C caller can legitimately produce, or one of the
 * two calls before it leaves the settings in a state it cannot cope with. Running the minimal
 * case separates them, and whichever way it lands the answer belongs in a test rather than in
 * a commit message. */
static void test_ragdoll_minimal_creation(void)
{
    TestPhysicsContext ctx;
    JoltC_Skeleton* sk;
    JoltC_RagdollSettings* rs;
    JoltC_Ragdoll* rd;
    const JoltC_Shape* shape;
    JoltC_Vec3 halfExtent;

    TEST_BEGIN("Ragdoll creation with nothing but parts and shapes");
    setup_physics_context(&ctx);

    sk = JoltC_Skeleton_Create();
    rs = JoltC_RagdollSettings_Create();
    halfExtent.x = 0.25f;
    halfExtent.y = 0.5f;
    halfExtent.z = 0.125f;
    shape = JoltC_BoxShape_Create(halfExtent, 0.0f);

    if (sk && rs && shape) {
        JoltC_RVec3 p0, p1;
        JoltC_Quat identity = identity_quat();

        JoltC_Skeleton_AddJoint(sk, "pelvis");
        JoltC_Skeleton_AddJoint2(sk, "spine", 0);
        JoltC_RagdollSettings_SetSkeleton(rs, sk);
        JoltC_RagdollSettings_ResizeParts(rs, 2);

        JoltC_RagdollSettings_SetPartShape(rs, 0, shape);
        JoltC_RagdollSettings_SetPartShape(rs, 1, shape);
        JoltC_RagdollSettings_SetPartMotionType(rs, 0, JOLTC_MOTION_TYPE_DYNAMIC);
        JoltC_RagdollSettings_SetPartMotionType(rs, 1, JOLTC_MOTION_TYPE_DYNAMIC);
        JoltC_RagdollSettings_SetPartObjectLayer(rs, 0, OBJ_LAYER_DYNAMIC);
        JoltC_RagdollSettings_SetPartObjectLayer(rs, 1, OBJ_LAYER_DYNAMIC);

        p0.x = PART0_X; p0.y = PART0_Y; p0.z = PART0_Z;
        p1.x = PART1_X; p1.y = PART1_Y; p1.z = PART1_Z;
        JoltC_RagdollSettings_SetPartPosition(rs, 0, p0);
        JoltC_RagdollSettings_SetPartPosition(rs, 1, p1);
        JoltC_RagdollSettings_SetPartRotation(rs, 0, identity);
        JoltC_RagdollSettings_SetPartRotation(rs, 1, identity);

        /* No SetPartMassProperties. A part left at Jolt's default computes its mass from the
         * shape, which is the path with the fewest of our own assumptions in it. */
        printf("[minimal: at CreateRagdoll] ");
        rd = JoltC_RagdollSettings_CreateRagdoll(rs, ctx.physicsSystem,
                                                 RAGDOLL_GROUP_ID, RAGDOLL_USER_DATA);
        printf("[minimal: created] ");
        TEST_ASSERT_NOT_NULL(rd, "a ragdoll with no constraints can still be created");

        if (rd) {
            TEST_ASSERT(JoltC_Ragdoll_GetBodyCount(rd) == 2, "two bodies, one per part");
            TEST_ASSERT(JoltC_Ragdoll_GetConstraintCount(rd) == 0,
                        "no constraints, because C cannot supply any");
            JoltC_Ragdoll_Destroy(rd);
        }
    }

    if (shape) JoltC_Shape_Release(shape);
    if (rs) JoltC_RagdollSettings_Destroy(rs);
    if (sk) JoltC_Skeleton_Destroy(sk);
    teardown_physics_context(&ctx);
    TEST_END();
}

static void test_ragdoll_settings_and_instance(void)
{
    TestPhysicsContext ctx;
    JoltC_Skeleton* sk;
    JoltC_RagdollSettings* rs;
    JoltC_Ragdoll* rd;
    const JoltC_Shape* shape;
    JoltC_Vec3 halfExtent;

    TEST_BEGIN("RagdollSettings parts and Ragdoll instance");
    setup_physics_context(&ctx);

    sk = JoltC_Skeleton_Create();
    rs = JoltC_RagdollSettings_Create();
    halfExtent.x = 0.25f;
    halfExtent.y = 0.5f;
    halfExtent.z = 0.125f;
    shape = JoltC_BoxShape_Create(halfExtent, 0.0f);

    TEST_ASSERT_NOT_NULL(rs, "ragdoll settings created");
    TEST_ASSERT_NOT_NULL(shape, "part shape created");

    if (sk && rs && shape) {
        /* Parents by index: CreateRagdoll indexes its body array with the parent joint
         * index, so an unresolved -1 would be an out of bounds read there. */
        JoltC_Skeleton_AddJoint(sk, "pelvis");
        JoltC_Skeleton_AddJoint2(sk, "spine", 0);

        TEST_ASSERT(JoltC_RagdollSettings_GetSkeleton(rs) == NULL,
                    "fresh settings have no skeleton");
        TEST_ASSERT(JoltC_RagdollSettings_GetPartCount(rs) == 0,
                    "fresh settings have no parts");

        JoltC_RagdollSettings_SetSkeleton(rs, sk);
        TEST_ASSERT(JoltC_RagdollSettings_GetSkeleton(rs) == sk,
                    "GetSkeleton returns the handle that was set");

        JoltC_RagdollSettings_ResizeParts(rs, 2);
        TEST_ASSERT(JoltC_RagdollSettings_GetPartCount(rs) == 2, "two parts");
        JoltC_RagdollSettings_ResizeParts(rs, -1);
        TEST_ASSERT(JoltC_RagdollSettings_GetPartCount(rs) == 2,
                    "a negative resize is rejected rather than wrapping to a huge size");

        JoltC_RagdollSettings_SetPartShape(rs, 0, shape);
        JoltC_RagdollSettings_SetPartShape(rs, 1, shape);
        JoltC_RagdollSettings_SetPartMotionType(rs, 0, JOLTC_MOTION_TYPE_DYNAMIC);
        JoltC_RagdollSettings_SetPartMotionType(rs, 1, JOLTC_MOTION_TYPE_DYNAMIC);
        JoltC_RagdollSettings_SetPartObjectLayer(rs, 0, OBJ_LAYER_DYNAMIC);
        JoltC_RagdollSettings_SetPartObjectLayer(rs, 1, OBJ_LAYER_DYNAMIC);
        /* Masses far apart so the stabiliser's ratio clamp has real work to do. */
        JoltC_RagdollSettings_SetPartMassProperties(rs, 0, 12.0f);
        JoltC_RagdollSettings_SetPartMassProperties(rs, 1, 3.0f);
        {
            JoltC_RVec3 p0, p1;
            JoltC_Quat identity = identity_quat();
            p0.x = PART0_X; p0.y = PART0_Y; p0.z = PART0_Z;
            p1.x = PART1_X; p1.y = PART1_Y; p1.z = PART1_Z;
            JoltC_RagdollSettings_SetPartPosition(rs, 0, p0);
            JoltC_RagdollSettings_SetPartPosition(rs, 1, p1);
            JoltC_RagdollSettings_SetPartRotation(rs, 0, identity);
            JoltC_RagdollSettings_SetPartRotation(rs, 1, identity);
        }

        /* There is no JoltC entry point anywhere that produces a
         * JoltC_TwoBodyConstraintSettings*, so null is the only value a C caller can pass
         * here. That is exercised for the guard, and it is why this ragdoll has no
         * constraints. */
        JoltC_RagdollSettings_SetPartToParent(rs, 1, 0, NULL);

        /* Every part setter is index-guarded. None of these may write or crash. */
        JoltC_RagdollSettings_SetPartShape(rs, 9, shape);
        JoltC_RagdollSettings_SetPartMotionType(rs, -1, JOLTC_MOTION_TYPE_STATIC);
        JoltC_RagdollSettings_SetPartObjectLayer(rs, 9, OBJ_LAYER_STATIC);
        JoltC_RagdollSettings_SetPartMassProperties(rs, -4, 1.0f);
        JoltC_RagdollSettings_SetPartToParent(rs, 9, 0, NULL);
        TEST_ASSERT(JoltC_RagdollSettings_GetPartCount(rs) == 2,
                    "out of range part writes do not change the part count");

        /* Both branches of DisableParentChildCollisions. With a pose supplied the wrapper
         * builds a matrix array sized from the part count and upstream runs real narrow
         * phase tests between the parts, so this is the branch worth walking; the null
         * form only installs the parent/child filter. There is no accessor for a part's
         * collision group, so the assertion is that neither call disturbs the settings. */
        {
            JoltC_Mat44 jointMatrices[2];
            make_translation_matrix(&jointMatrices[0], PART0_X, PART0_Y, PART0_Z);
            make_translation_matrix(&jointMatrices[1], PART1_X, PART1_Y, PART1_Z);
            printf("[at DisableCollisions] ");
            JoltC_RagdollSettings_DisableParentChildCollisions(rs, jointMatrices, 0.03125f);
            printf("[at DisableCollisions null] ");
            JoltC_RagdollSettings_DisableParentChildCollisions(rs, NULL, 0.0f);
            TEST_ASSERT(JoltC_RagdollSettings_GetPartCount(rs) == 2,
                        "disabling parent child collisions does not resize the parts");
            TEST_ASSERT(JoltC_RagdollSettings_GetSkeleton(rs) == sk,
                        "disabling parent child collisions does not detach the skeleton");
        }

        /* Documented to return true on success. It redistributes mass between parent and
         * child, which needs the shapes and the resolved parent indices above.
         *
         * Markers because this test segfaulted in CI and a suite name is not a line number.
         * Left in: the calls between them are the ones that reach deepest into upstream, and
         * the next crash here will name itself instead of costing a bisect. */
        printf("[at Stabilize] ");
        TEST_ASSERT(JoltC_RagdollSettings_Stabilize(rs) != 0,
                    "Stabilize succeeds on a two part chain");
        printf("[at CalcBodyToConstraint] ");

        /* With no constraint attached to either part, both entries are -1. This table has
         * to be built before Ragdoll::DriveToPoseUsingMotors is called: upstream indexes
         * it directly with no bounds check. */
        JoltC_RagdollSettings_CalculateBodyIndexToConstraintIndex(rs);
        TEST_ASSERT(JoltC_RagdollSettings_GetConstraintIndexForBodyIndex(rs, 0) == -1,
                    "the root part never has a constraint to a parent");
        TEST_ASSERT(JoltC_RagdollSettings_GetConstraintIndexForBodyIndex(rs, 1) == -1,
                    "a part with a null parent constraint reports -1");
        JoltC_RagdollSettings_CalculateConstraintIndexToBodyIdxPair(rs);

        printf("[at CreateRagdoll] ");
        rd = JoltC_RagdollSettings_CreateRagdoll(rs, ctx.physicsSystem,
                                                 RAGDOLL_GROUP_ID, RAGDOLL_USER_DATA);
        printf("[created] ");
        TEST_ASSERT_NOT_NULL(rd, "ragdoll created");

        if (rd) {
            JoltC_BodyID id0 = JoltC_Ragdoll_GetBodyID(rd, 0);
            JoltC_BodyID id1 = JoltC_Ragdoll_GetBodyID(rd, 1);

            TEST_ASSERT(JoltC_Ragdoll_GetBodyCount(rd) == 2, "two bodies, one per part");
            TEST_ASSERT(JoltC_Ragdoll_GetConstraintCount(rd) == 0,
                        "no constraints, because none could be supplied");
            TEST_ASSERT(JoltC_Ragdoll_GetRagdollSettings(rd) != NULL,
                        "the ragdoll reports the settings that built it");
            TEST_ASSERT(id0 != JOLTC_BODY_ID_INVALID, "body 0 is valid");
            TEST_ASSERT(id1 != JOLTC_BODY_ID_INVALID, "body 1 is valid");
            TEST_ASSERT(id0 != id1, "the two parts are different bodies");

            TEST_ASSERT(JoltC_BodyInterface_IsAdded(ctx.bodyInterface, id0) == 0,
                        "CreateRagdoll creates bodies without adding them");

            JoltC_Ragdoll_AddToPhysicsSystem(rd, JOLTC_ACTIVATION_DONT_ACTIVATE, JOLTC_TRUE);
            TEST_ASSERT(JoltC_BodyInterface_IsAdded(ctx.bodyInterface, id0) != 0,
                        "body 0 added");
            TEST_ASSERT(JoltC_BodyInterface_IsAdded(ctx.bodyInterface, id1) != 0,
                        "body 1 added");

            /* The collision group and the user data are adjacent integer parameters and
             * the wrapper reorders them for Jolt, whose own signature takes them in the
             * other order. If they were exchanged the user data would read as 7. */
            TEST_ASSERT(JoltC_BodyInterface_GetUserData(ctx.bodyInterface, id0)
                            == RAGDOLL_USER_DATA,
                        "the user data reached the bodies, not the collision group id");

            TEST_ASSERT(JoltC_BodyInterface_GetMotionType(ctx.bodyInterface, id0)
                            == JOLTC_MOTION_TYPE_DYNAMIC,
                        "the part motion type reached the body");
            TEST_ASSERT(JoltC_BodyInterface_GetObjectLayer(ctx.bodyInterface, id0)
                            == OBJ_LAYER_DYNAMIC,
                        "the part object layer reached the body");

            /* Read immediately after creation with no step in between, so this is the
             * placement that was requested rather than anything the solver produced. */
            {
                JoltC_RVec3 pos0 = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, id0);
                JoltC_RVec3 pos1 = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, id1);
                TEST_ASSERT_FLOAT_EQ(pos0.x, PART0_X, 0.0005f, "part 0 position x");
                TEST_ASSERT_FLOAT_EQ(pos0.y, PART0_Y, 0.0005f, "part 0 position y");
                TEST_ASSERT_FLOAT_EQ(pos0.z, PART0_Z, 0.0005f, "part 0 position z");
                TEST_ASSERT_FLOAT_EQ(pos1.x, PART1_X, 0.0005f,
                                     "part 1 position x is not part 0's");
                TEST_ASSERT_FLOAT_EQ(pos1.y, PART1_Y, 0.0005f, "part 1 position y");
                TEST_ASSERT_FLOAT_EQ(pos1.z, PART1_Z, 0.0005f, "part 1 position z");
            }

            TEST_ASSERT(JoltC_Ragdoll_IsActive(rd, JOLTC_TRUE) == 0,
                        "DontActivate leaves the ragdoll asleep");
            JoltC_Ragdoll_Activate(rd, JOLTC_TRUE);
            TEST_ASSERT(JoltC_Ragdoll_IsActive(rd, JOLTC_TRUE) != 0,
                        "Activate wakes the ragdoll");

            {
                JoltC_RVec3 rootPos;
                JoltC_Quat rootRot;
                rootPos.x = -99.0f;
                JoltC_Ragdoll_GetRootTransform(rd, &rootPos, &rootRot, JOLTC_TRUE);
                TEST_ASSERT_FLOAT_EQ(rootPos.x, PART0_X, 0.0005f,
                                     "the root transform is body 0, x");
                TEST_ASSERT_FLOAT_EQ(rootPos.y, PART0_Y, 0.0005f,
                                     "the root transform is body 0, y");
                TEST_ASSERT_FLOAT_EQ(rootPos.z, PART0_Z, 0.0005f,
                                     "the root transform is body 0, z");
                TEST_ASSERT_FLOAT_EQ(rootRot.w, 1.0f, 0.0005f,
                                     "the root rotation is the identity it was given");

                /* Each output is optional and separately guarded. */
                JoltC_Ragdoll_GetRootTransform(rd, NULL, &rootRot, JOLTC_TRUE);
                JoltC_Ragdoll_GetRootTransform(rd, &rootPos, NULL, JOLTC_TRUE);
            }

            /* SetPose places body i at rootOffset + jointMatrix[i].translation. GetPose is
             * not its inverse: the root offset comes back as body 0's world position and
             * joint matrix 0 comes back with a zero translation, with every other joint
             * relative to that. Both directions are pinned here because the asymmetry is
             * exactly what a hand repair is likely to "fix". */
            {
                JoltC_RVec3 rootOffset;
                JoltC_Mat44 jm[2];
                JoltC_Mat44 outJm[2];
                JoltC_RVec3 outRoot;
                JoltC_RVec3 pos0, pos1;

                rootOffset.x = 0.5f;
                rootOffset.y = -1.25f;
                rootOffset.z = 2.5f;
                make_translation_matrix(&jm[0], 0.25f, -0.5f, 0.125f);
                make_translation_matrix(&jm[1], -1.75f, 3.25f, -0.375f);

                JoltC_Ragdoll_SetPose2(rd, rootOffset, jm, JOLTC_TRUE);
                pos0 = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, id0);
                pos1 = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, id1);
                TEST_ASSERT_FLOAT_EQ(pos0.x, 0.75f, 0.0005f,
                                     "body 0 is root offset plus joint 0, x");
                TEST_ASSERT_FLOAT_EQ(pos0.y, -1.75f, 0.0005f,
                                     "body 0 is root offset plus joint 0, y");
                TEST_ASSERT_FLOAT_EQ(pos0.z, 2.625f, 0.0005f,
                                     "body 0 is root offset plus joint 0, z");
                TEST_ASSERT_FLOAT_EQ(pos1.x, -1.25f, 0.0005f,
                                     "body 1 is root offset plus joint 1, x");
                TEST_ASSERT_FLOAT_EQ(pos1.y, 2.0f, 0.0005f,
                                     "body 1 is root offset plus joint 1, y");
                TEST_ASSERT_FLOAT_EQ(pos1.z, 2.125f, 0.0005f,
                                     "body 1 is root offset plus joint 1, z");

                outRoot.x = -99.0f;
                fill_matrix_with(&outJm[0], -99.0f);
                fill_matrix_with(&outJm[1], -99.0f);
                JoltC_Ragdoll_GetPose2(rd, &outRoot, outJm, JOLTC_TRUE);
                TEST_ASSERT_FLOAT_EQ(outRoot.x, 0.75f, 0.0005f,
                                     "the returned root offset is body 0's world position");
                TEST_ASSERT_FLOAT_EQ(outRoot.y, -1.75f, 0.0005f,
                                     "the returned root offset absorbs joint 0, y");
                TEST_ASSERT_FLOAT_EQ(outRoot.z, 2.625f, 0.0005f,
                                     "the returned root offset absorbs joint 0, z");
                assert_matrix_translation(&outJm[0], 0.0f, 0.0f, 0.0f,
                                          "returned joint 0 translation is zeroed, x",
                                          "returned joint 0 translation is zeroed, y",
                                          "returned joint 0 translation is zeroed, z");
                assert_matrix_translation(&outJm[1], -2.0f, 3.75f, -0.5f,
                                          "returned joint 1 is relative to the root, x",
                                          "returned joint 1 is relative to the root, y",
                                          "returned joint 1 is relative to the root, z");
                TEST_ASSERT_FLOAT_EQ(outJm[1].m[0], 1.0f, 0.0005f,
                                     "the returned rotation block is identity");
                TEST_ASSERT_FLOAT_EQ(outJm[1].m[15], 1.0f, 0.0005f,
                                     "the returned homogeneous element is 1");
            }

            /* The pose-object overloads read and write the same data through a
             * SkeletonPose. Get followed by Set is a fixed point, which is the strongest
             * statement that can be made about the pair without a step. */
            {
                JoltC_SkeletonPose* pose = JoltC_SkeletonPose_Create();
                if (pose) {
                    JoltC_RVec3 offset, pos0, pos1;
                    JoltC_Mat44 jm;

                    JoltC_SkeletonPose_SetSkeleton(pose, sk);
                    JoltC_Ragdoll_GetPose(rd, pose, JOLTC_TRUE);

                    offset = JoltC_SkeletonPose_GetRootOffset(pose);
                    TEST_ASSERT_FLOAT_EQ(offset.x, 0.75f, 0.0005f,
                                         "GetPose writes the root offset into the pose");
                    TEST_ASSERT_FLOAT_EQ(offset.y, -1.75f, 0.0005f,
                                         "GetPose root offset y");
                    JoltC_SkeletonPose_GetJointMatrix(pose, 1, &jm);
                    assert_matrix_translation(&jm, -2.0f, 3.75f, -0.5f,
                                              "pose joint 1 x", "pose joint 1 y",
                                              "pose joint 1 z");

                    JoltC_Ragdoll_SetPose(rd, pose, JOLTC_TRUE);
                    pos0 = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, id0);
                    pos1 = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, id1);
                    TEST_ASSERT_FLOAT_EQ(pos0.x, 0.75f, 0.0005f,
                                         "get then set leaves body 0 where it was, x");
                    TEST_ASSERT_FLOAT_EQ(pos0.y, -1.75f, 0.0005f,
                                         "get then set leaves body 0 where it was, y");
                    TEST_ASSERT_FLOAT_EQ(pos1.x, -1.25f, 0.0005f,
                                         "get then set leaves body 1 where it was, x");
                    TEST_ASSERT_FLOAT_EQ(pos1.y, 2.0f, 0.0005f,
                                         "get then set leaves body 1 where it was, y");
                    TEST_ASSERT_FLOAT_EQ(pos1.z, 2.125f, 0.0005f,
                                         "get then set leaves body 1 where it was, z");

                    /* No constraints exist, so every constraint index is -1 and this walks
                     * the loop without reaching a motor. Nothing is asserted about the
                     * result: with a real motorised constraint the outcome would be a
                     * solver behaviour, which is off limits across this bump. */
                    JoltC_Ragdoll_DriveToPoseUsingMotors(rd, pose);
                    JoltC_Ragdoll_DriveToPoseUsingKinematics(rd, pose, 0.0625f, JOLTC_TRUE);
                    JoltC_Ragdoll_ResetWarmStart(rd);

                    JoltC_SkeletonPose_Destroy(pose);
                }
            }

            JoltC_Ragdoll_RemoveFromPhysicsSystem(rd, JOLTC_TRUE);
            TEST_ASSERT(JoltC_BodyInterface_IsAdded(ctx.bodyInterface, id0) == 0,
                        "removal takes the bodies back out of the system");

            /* Destroying the ragdoll destroys its bodies, so it has to happen while the
             * physics system is still alive. */
            JoltC_Ragdoll_Destroy(rd);
        }
    }

    if (shape) JoltC_Shape_Release(shape);
    if (rs) JoltC_RagdollSettings_Destroy(rs);
    if (sk) JoltC_Skeleton_Destroy(sk);
    teardown_physics_context(&ctx);
    TEST_END();
}

/* ========================================================================== */
/*  Null safety                                                               */
/* ========================================================================== */

/* Every one of these has an explicit guard in the wrapper. A repair that drops one turns a
 * caller's null handle into a crash inside the native library, which is the single worst
 * failure mode for a P/Invoke binding because the managed stack trace says nothing. */
static void test_null_handle_safety(void)
{
    TEST_BEGIN("Skeleton, pose, animation, mapper and ragdoll accessors accept null");
    {
        JoltC_SkeletonJoint joint;
        JoltC_Vec3 t;
        JoltC_Quat r;
        JoltC_Mat44 m;
        JoltC_RVec3 offset;

        joint.name = NULL;
        joint.parentName = NULL;
        joint.parentJointIndex = -42;
        JoltC_Skeleton_GetJoint(NULL, 0, &joint);
        TEST_ASSERT(joint.parentJointIndex == -42,
                    "GetJoint on a null skeleton writes nothing");

        TEST_ASSERT(JoltC_Skeleton_GetJointCount(NULL) == 0, "GetJointCount(NULL) is 0");
        TEST_ASSERT(JoltC_Skeleton_GetJointIndex(NULL, "x") == -1,
                    "GetJointIndex(NULL) is -1");
        TEST_ASSERT(JoltC_Skeleton_AreJointsCorrectlyOrdered(NULL) == 0,
                    "AreJointsCorrectlyOrdered(NULL) is false");
        JoltC_Skeleton_CalculateParentJointIndices(NULL);
        JoltC_Skeleton_Destroy(NULL);

        TEST_ASSERT(JoltC_SkeletonPose_GetSkeleton(NULL) == NULL,
                    "GetSkeleton(NULL) is null");
        TEST_ASSERT(JoltC_SkeletonPose_GetJointCount(NULL) == 0,
                    "pose GetJointCount(NULL) is 0");
        offset = JoltC_SkeletonPose_GetRootOffset(NULL);
        TEST_ASSERT_FLOAT_EQ(offset.x, 0.0f, 0.0005f, "GetRootOffset(NULL) is zero, x");
        TEST_ASSERT_FLOAT_EQ(offset.y, 0.0f, 0.0005f, "GetRootOffset(NULL) is zero, y");
        TEST_ASSERT_FLOAT_EQ(offset.z, 0.0f, 0.0005f, "GetRootOffset(NULL) is zero, z");
        t.x = 0.0f; t.y = 0.0f; t.z = 0.0f;
        r = identity_quat();
        JoltC_SkeletonPose_SetJointState(NULL, 0, t, r);
        JoltC_SkeletonPose_GetJointState(NULL, 0, &t, &r);
        JoltC_SkeletonPose_SetJointMatrix(NULL, 0, &m);
        JoltC_SkeletonPose_GetJointMatrix(NULL, 0, &m);
        JoltC_SkeletonPose_SetJointMatrices(NULL, &m, 1);
        JoltC_SkeletonPose_GetJointMatrices(NULL, &m, 1);
        JoltC_SkeletonPose_CalculateJointMatrices(NULL);
        JoltC_SkeletonPose_CalculateJointStates(NULL);
        JoltC_SkeletonPose_CalculateLocalSpaceJointMatrices(NULL, &m);
        JoltC_SkeletonPose_SetSkeleton(NULL, NULL);
        JoltC_SkeletonPose_Destroy(NULL);

        TEST_ASSERT_FLOAT_EQ(JoltC_SkeletalAnimation_GetDuration(NULL), 0.0f, 0.0005f,
                             "animation GetDuration(NULL) is 0");
        TEST_ASSERT(JoltC_SkeletalAnimation_IsLooping(NULL) == 0,
                    "animation IsLooping(NULL) is false");
        TEST_ASSERT(JoltC_SkeletalAnimation_GetAnimatedJointCount(NULL) == 0,
                    "animation GetAnimatedJointCount(NULL) is 0");
        JoltC_SkeletalAnimation_SetIsLooping(NULL, JOLTC_TRUE);
        JoltC_SkeletalAnimation_ScaleJoints(NULL, 2.0f);
        JoltC_SkeletalAnimation_AddAnimatedJoint(NULL, "x");
        JoltC_SkeletalAnimation_AddKeyframe(NULL, 0, 0.0f, t, r);
        JoltC_SkeletalAnimation_Sample(NULL, 0.0f, NULL);
        JoltC_SkeletalAnimation_Destroy(NULL);

        TEST_ASSERT(JoltC_SkeletonMapper_GetMappedJointIndex(NULL, 0) == -1,
                    "mapper GetMappedJointIndex(NULL) is -1");
        TEST_ASSERT(JoltC_SkeletonMapper_IsJointTranslationLocked(NULL, 0) == 0,
                    "mapper IsJointTranslationLocked(NULL) is false");
        JoltC_SkeletonMapper_Initialize(NULL, NULL, &m, NULL, &m);
        JoltC_SkeletonMapper_LockAllTranslations(NULL, NULL, &m);
        JoltC_SkeletonMapper_Map(NULL, &m, &m, &m);
        JoltC_SkeletonMapper_MapReverse(NULL, &m, &m);
        JoltC_SkeletonMapper_Destroy(NULL);

        TEST_ASSERT(JoltC_RagdollSettings_GetSkeleton(NULL) == NULL,
                    "settings GetSkeleton(NULL) is null");
        TEST_ASSERT(JoltC_RagdollSettings_GetPartCount(NULL) == 0,
                    "settings GetPartCount(NULL) is 0");
        TEST_ASSERT(JoltC_RagdollSettings_Stabilize(NULL) == 0,
                    "settings Stabilize(NULL) is false");
        TEST_ASSERT(JoltC_RagdollSettings_GetConstraintIndexForBodyIndex(NULL, 0) == -1,
                    "settings GetConstraintIndexForBodyIndex(NULL) is -1");
        TEST_ASSERT(JoltC_RagdollSettings_CreateRagdoll(NULL, NULL, 0u, 0u) == NULL,
                    "CreateRagdoll on null settings is null");
        JoltC_RagdollSettings_SetSkeleton(NULL, NULL);
        JoltC_RagdollSettings_ResizeParts(NULL, 1);
        JoltC_RagdollSettings_DisableParentChildCollisions(NULL, NULL, 0.0f);
        JoltC_RagdollSettings_CalculateBodyIndexToConstraintIndex(NULL);
        JoltC_RagdollSettings_CalculateConstraintIndexToBodyIdxPair(NULL);
        JoltC_RagdollSettings_Destroy(NULL);

        TEST_ASSERT(JoltC_Ragdoll_GetBodyCount(NULL) == 0, "ragdoll GetBodyCount(NULL) is 0");
        TEST_ASSERT(JoltC_Ragdoll_GetConstraintCount(NULL) == 0,
                    "ragdoll GetConstraintCount(NULL) is 0");
        TEST_ASSERT(JoltC_Ragdoll_GetBodyID(NULL, 0) == JOLTC_BODY_ID_INVALID,
                    "ragdoll GetBodyID(NULL) is the invalid id");
        TEST_ASSERT(JoltC_Ragdoll_GetConstraint(NULL, 0) == NULL,
                    "ragdoll GetConstraint(NULL) is null");
        TEST_ASSERT(JoltC_Ragdoll_GetRagdollSettings(NULL) == NULL,
                    "ragdoll GetRagdollSettings(NULL) is null");
        TEST_ASSERT(JoltC_Ragdoll_IsActive(NULL, JOLTC_TRUE) == 0,
                    "ragdoll IsActive(NULL) is false");
        JoltC_Ragdoll_AddToPhysicsSystem(NULL, JOLTC_ACTIVATION_ACTIVATE, JOLTC_TRUE);
        JoltC_Ragdoll_RemoveFromPhysicsSystem(NULL, JOLTC_TRUE);
        JoltC_Ragdoll_Activate(NULL, JOLTC_TRUE);
        JoltC_Ragdoll_ResetWarmStart(NULL);
        JoltC_Ragdoll_SetPose(NULL, NULL, JOLTC_TRUE);
        JoltC_Ragdoll_GetPose(NULL, NULL, JOLTC_TRUE);
        JoltC_Ragdoll_GetPose2(NULL, NULL, NULL, JOLTC_TRUE);
        JoltC_Ragdoll_DriveToPoseUsingMotors(NULL, NULL);
        JoltC_Ragdoll_DriveToPoseUsingKinematics(NULL, NULL, 0.0f, JOLTC_TRUE);
        JoltC_Ragdoll_GetRootTransform(NULL, NULL, NULL, JOLTC_TRUE);
        JoltC_Ragdoll_Destroy(NULL);
        {
            JoltC_RVec3 zero;
            zero.x = 0.0f;
            zero.y = 0.0f;
            zero.z = 0.0f;
            JoltC_Ragdoll_SetPose2(NULL, zero, &m, JOLTC_TRUE);
            JoltC_RagdollSettings_SetPartPosition(NULL, 0, zero);
            JoltC_RagdollSettings_SetPartRotation(NULL, 0, r);
            JoltC_RagdollSettings_SetPartShape(NULL, 0, NULL);
            JoltC_RagdollSettings_SetPartMotionType(NULL, 0, JOLTC_MOTION_TYPE_STATIC);
            JoltC_RagdollSettings_SetPartObjectLayer(NULL, 0, 0);
            JoltC_RagdollSettings_SetPartMassProperties(NULL, 0, 1.0f);
            JoltC_RagdollSettings_SetPartToParent(NULL, 0, 0, NULL);
        }
    }
    TEST_END();
}

/* ========================================================================== */
/*  Suite entry point                                                         */
/* ========================================================================== */
void run_skeleton_extra_tests(void)
{
    printf("\n=== Skeleton, pose, animation, mapper and ragdoll ===\n");
    test_parent_index_resolution();
    test_joint_ordering_detects_a_child_before_its_parent();
    test_pose_skeleton_and_root_offset();
    test_pose_joint_state_indexing();
    test_pose_joint_matrix_accessors();
    test_pose_matrix_composition_round_trip();
    test_animation_keyframes_and_sampling();
    test_mapper_initialize_and_map();
    test_mapper_locked_translations();
    test_ragdoll_minimal_creation();
    test_ragdoll_settings_and_instance();
    test_null_handle_safety();
}
