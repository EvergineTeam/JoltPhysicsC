/* JoltC Test Suite — skeleton.h API tests (Skeleton, SkeletonPose, SkeletalAnimation)
 * SPDX-License-Identifier: MIT
 */

#include "test_common.h"

void run_skeleton_tests(void)
{
    /* test_skeleton_create_add_joints */
    TEST_BEGIN("Skeleton create, add joints, get count");
    {
        JoltC_Skeleton* sk = JoltC_Skeleton_Create();
        TEST_ASSERT_NOT_NULL(sk, "Skeleton not null");

        uint32_t j0 = JoltC_Skeleton_AddJoint(sk, "root");
        uint32_t j1 = JoltC_Skeleton_AddJoint3(sk, "spine", "root");
        uint32_t j2 = JoltC_Skeleton_AddJoint3(sk, "head", "spine");
        TEST_ASSERT(j0 == 0, "root index == 0");
        TEST_ASSERT(j1 == 1, "spine index == 1");
        TEST_ASSERT(j2 == 2, "head index == 2");
        TEST_ASSERT(JoltC_Skeleton_GetJointCount(sk) == 3, "3 joints");

        JoltC_Skeleton_Destroy(sk);
    }
    TEST_END();

    /* test_skeleton_get_joint_info */
    TEST_BEGIN("Skeleton GetJoint and GetJointIndex");
    {
        JoltC_Skeleton* sk = JoltC_Skeleton_Create();
        JoltC_Skeleton_AddJoint(sk, "root");
        JoltC_Skeleton_AddJoint3(sk, "arm", "root");

        int idx = JoltC_Skeleton_GetJointIndex(sk, "arm");
        TEST_ASSERT(idx == 1, "arm index == 1");

        JoltC_SkeletonJoint joint;
        JoltC_Skeleton_GetJoint(sk, 1, &joint);
        TEST_ASSERT(joint.name != NULL, "joint name not null");

        JoltC_Skeleton_Destroy(sk);
    }
    TEST_END();

    /* test_skeleton_correctly_ordered */
    TEST_BEGIN("Skeleton AreJointsCorrectlyOrdered");
    {
        JoltC_Skeleton* sk = JoltC_Skeleton_Create();
        JoltC_Skeleton_AddJoint(sk, "root");
        JoltC_Skeleton_AddJoint3(sk, "child", "root");
        /* Parent before child → correctly ordered */
        TEST_ASSERT(JoltC_Skeleton_AreJointsCorrectlyOrdered(sk), "joints correctly ordered");

        JoltC_Skeleton_Destroy(sk);
    }
    TEST_END();

    /* test_skeleton_pose */
    TEST_BEGIN("SkeletonPose create, set skeleton, set/get joint state");
    {
        JoltC_Skeleton* sk = JoltC_Skeleton_Create();
        JoltC_Skeleton_AddJoint(sk, "root");
        JoltC_Skeleton_AddJoint3(sk, "child", "root");

        JoltC_SkeletonPose* pose = JoltC_SkeletonPose_Create();
        TEST_ASSERT_NOT_NULL(pose, "SkeletonPose not null");

        JoltC_SkeletonPose_SetSkeleton(pose, sk);
        TEST_ASSERT(JoltC_SkeletonPose_GetJointCount(pose) == 2, "pose has 2 joints");

        JoltC_Vec3 t = { 1.0f, 2.0f, 3.0f };
        JoltC_Quat r = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_SkeletonPose_SetJointState(pose, 0, t, r);

        JoltC_Vec3 outT;
        JoltC_Quat outR;
        JoltC_SkeletonPose_GetJointState(pose, 0, &outT, &outR);
        TEST_ASSERT_FLOAT_EQ(outT.x, 1.0f, 0.01f, "translation.x == 1");
        TEST_ASSERT_FLOAT_EQ(outR.w, 1.0f, 0.01f, "rotation.w == 1");

        JoltC_SkeletonPose_Destroy(pose);
        JoltC_Skeleton_Destroy(sk);
    }
    TEST_END();

    /* test_skeletal_animation */
    TEST_BEGIN("SkeletalAnimation create, add joint, looping");
    {
        JoltC_SkeletalAnimation* anim = JoltC_SkeletalAnimation_Create();
        TEST_ASSERT_NOT_NULL(anim, "SkeletalAnimation not null");

        JoltC_SkeletalAnimation_AddAnimatedJoint(anim, "root");
        TEST_ASSERT(JoltC_SkeletalAnimation_GetAnimatedJointCount(anim) == 1, "1 animated joint");

        JoltC_SkeletalAnimation_SetIsLooping(anim, JOLTC_TRUE);
        TEST_ASSERT(JoltC_SkeletalAnimation_IsLooping(anim), "looping == true");

        JoltC_SkeletalAnimation_Destroy(anim);
    }
    TEST_END();
}
