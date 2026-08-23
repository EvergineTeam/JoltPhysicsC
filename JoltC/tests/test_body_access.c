/* JoltC Test Suite -- body_access.h API tests (direct Body access, BodyLock,
 *                    MotionProperties)
 * SPDX-License-Identifier: MIT
 *
 * body_access.h is the widest module in the wrapper and had no test at all, so a
 * hand repair after a JoltPhysics bump could swap two float arguments or read the
 * wrong member and still compile. Almost every assertion below is therefore a
 * round-trip -- write a distinctive, asymmetric value, read it back -- or a
 * structural invariant. Nothing here asserts a position or a velocity that the
 * solver decides, so a new friction model or a different iteration order upstream
 * cannot make this file fail; only a wrapper that lost an argument, an order, or a
 * member can.
 *
 * Values are deliberately asymmetric (1.5, -2.25, 3.75 rather than 1, 1, 1) so
 * that a transposed pair of components is visible in the failure.
 */

#include "test_common.h"

/* test_common.h lists the suites that existed before this one. */
void run_body_access_tests(void);

/* ========================================================================== */
/*  Helpers                                                                   */
/* ========================================================================== */

static const JoltC_BodyLockInterface* body_lock_iface(TestPhysicsContext* ctx)
{
    return JoltC_PhysicsSystem_GetBodyLockInterface(ctx->physicsSystem);
}

/* Takes a write lock on one body. The caller destroys *outLock before touching
 * the BodyInterface again -- the interface takes the same mutexes. */
static JoltC_Body* lock_body_for_write(TestPhysicsContext* ctx, JoltC_BodyID id,
                                       JoltC_BodyLockWrite** outLock)
{
    *outLock = JoltC_BodyLockWrite_Create(body_lock_iface(ctx), id);
    return JoltC_BodyLockWrite_GetBody(*outLock);
}

static void remove_and_destroy(TestPhysicsContext* ctx, JoltC_BodyID id)
{
    JoltC_BodyInterface_RemoveBody(ctx->bodyInterface, id);
    JoltC_BodyInterface_DestroyBody(ctx->bodyInterface, id);
}

static JoltC_Quat identity_quat(void)
{
    JoltC_Quat q = { 0.0f, 0.0f, 0.0f, 1.0f };
    return q;
}

/* ========================================================================== */
/*  BodyLockInterface / BodyLock lifecycle                                    */
/* ========================================================================== */

static void test_lock_interface_available(void)
{
    TEST_BEGIN("BodyLockInterface from PhysicsSystem non-null");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        const JoltC_BodyLockInterface* locking =
            JoltC_PhysicsSystem_GetBodyLockInterface(ctx.physicsSystem);
        const JoltC_BodyLockInterface* noLock =
            JoltC_PhysicsSystem_GetBodyLockInterfaceNoLock(ctx.physicsSystem);

        TEST_ASSERT_NOT_NULL(locking, "GetBodyLockInterface non-null");
        TEST_ASSERT_NOT_NULL(noLock, "GetBodyLockInterfaceNoLock non-null");
        TEST_ASSERT(locking != noLock, "locking and no-lock interfaces are distinct");

        teardown_physics_context(&ctx);
    }
    TEST_END();
}

static void test_lock_read_and_write(void)
{
    TEST_BEGIN("BodyLockRead/Write reach the requested body");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 1.5f, 2.25f, -3.75f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC,
                                              JOLTC_ACTIVATION_ACTIVATE);
        const JoltC_BodyLockInterface* iface = body_lock_iface(&ctx);

        /* Read lock */
        JoltC_BodyLockRead* readLock = JoltC_BodyLockRead_Create(iface, id);
        TEST_ASSERT_NOT_NULL(readLock, "read lock created");
        const JoltC_Body* readBody = JoltC_BodyLockRead_GetBody(readLock);
        TEST_ASSERT_NOT_NULL(readBody, "read lock yields a body");
        if (readBody) {
            TEST_ASSERT(JoltC_Body_GetID(readBody) == id, "read: Body_GetID == requested id");
        }
        JoltC_BodyLockRead_Destroy(readLock);

        /* Write lock */
        JoltC_BodyLockWrite* writeLock = JoltC_BodyLockWrite_Create(iface, id);
        TEST_ASSERT_NOT_NULL(writeLock, "write lock created");
        JoltC_Body* writeBody = JoltC_BodyLockWrite_GetBody(writeLock);
        TEST_ASSERT_NOT_NULL(writeBody, "write lock yields a body");
        if (writeBody) {
            TEST_ASSERT(JoltC_Body_GetID(writeBody) == id, "write: Body_GetID == requested id");
        }
        JoltC_BodyLockWrite_Destroy(writeLock);

        /* Lock/Unlock spelling of the same thing */
        JoltC_BodyLockRead* lr = JoltC_BodyLockInterface_LockRead(iface, id);
        TEST_ASSERT_NOT_NULL(lr, "LockRead created");
        if (lr) {
            const JoltC_Body* b = JoltC_BodyLockRead_GetBody(lr);
            TEST_ASSERT_NOT_NULL(b, "LockRead yields a body");
            if (b) TEST_ASSERT(JoltC_Body_GetID(b) == id, "LockRead: id matches");
        }
        JoltC_BodyLockInterface_UnlockRead(iface, lr);

        JoltC_BodyLockWrite* lw = JoltC_BodyLockInterface_LockWrite(iface, id);
        TEST_ASSERT_NOT_NULL(lw, "LockWrite created");
        if (lw) {
            JoltC_Body* b = JoltC_BodyLockWrite_GetBody(lw);
            TEST_ASSERT_NOT_NULL(b, "LockWrite yields a body");
            if (b) TEST_ASSERT(JoltC_Body_GetID(b) == id, "LockWrite: id matches");
        }
        JoltC_BodyLockInterface_UnlockWrite(iface, lw);

        remove_and_destroy(&ctx, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}

static void test_lock_invalid_body_id(void)
{
    TEST_BEGIN("BodyLock on an invalid BodyID yields no body");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        const JoltC_BodyLockInterface* iface = body_lock_iface(&ctx);

        JoltC_BodyLockRead* readLock = JoltC_BodyLockRead_Create(iface, JOLTC_BODY_ID_INVALID);
        TEST_ASSERT_NOT_NULL(readLock, "read lock object still created");
        TEST_ASSERT(JoltC_BodyLockRead_GetBody(readLock) == NULL,
                    "failed read lock reports no body");
        JoltC_BodyLockRead_Destroy(readLock);

        JoltC_BodyLockWrite* writeLock = JoltC_BodyLockWrite_Create(iface, JOLTC_BODY_ID_INVALID);
        TEST_ASSERT_NOT_NULL(writeLock, "write lock object still created");
        TEST_ASSERT(JoltC_BodyLockWrite_GetBody(writeLock) == NULL,
                    "failed write lock reports no body");
        JoltC_BodyLockWrite_Destroy(writeLock);

        teardown_physics_context(&ctx);
    }
    TEST_END();
}

static void test_lock_multi_preserves_order(void)
{
    TEST_BEGIN("BodyLockMulti keeps the caller's body order");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 posA = { -2.5f, 1.0f, 0.0f };
        JoltC_RVec3 posB = { 4.5f, 1.0f, 0.0f };
        JoltC_BodyID idA = create_test_box_body(&ctx, posA, JOLTC_MOTION_TYPE_DYNAMIC,
                                               JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyID idB = create_test_box_body(&ctx, posB, JOLTC_MOTION_TYPE_DYNAMIC,
                                               JOLTC_ACTIVATION_ACTIVATE);
        TEST_ASSERT(idA != idB, "two distinct bodies");

        const JoltC_BodyLockInterface* iface = body_lock_iface(&ctx);
        JoltC_BodyID ids[2];
        ids[0] = idA;
        ids[1] = idB;

        JoltC_BodyLockMultiRead* multiRead =
            JoltC_BodyLockInterface_LockMultiRead(iface, ids, 2);
        TEST_ASSERT_NOT_NULL(multiRead, "multi read lock created");
        JoltC_BodyLockMultiRead_Destroy(multiRead);

        JoltC_BodyLockMultiWrite* multiWrite =
            JoltC_BodyLockInterface_LockMultiWrite(iface, ids, 2);
        TEST_ASSERT_NOT_NULL(multiWrite, "multi write lock created");
        if (multiWrite) {
            JoltC_Body* b0 = JoltC_BodyLockMultiWrite_GetBody(multiWrite, 0);
            JoltC_Body* b1 = JoltC_BodyLockMultiWrite_GetBody(multiWrite, 1);
            TEST_ASSERT_NOT_NULL(b0, "multi write index 0 non-null");
            TEST_ASSERT_NOT_NULL(b1, "multi write index 1 non-null");
            if (b0 && b1) {
                TEST_ASSERT(JoltC_Body_GetID(b0) == idA, "index 0 is the first requested body");
                TEST_ASSERT(JoltC_Body_GetID(b1) == idB, "index 1 is the second requested body");
            }
        }
        JoltC_BodyLockMultiWrite_Destroy(multiWrite);

        remove_and_destroy(&ctx, idA);
        remove_and_destroy(&ctx, idB);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}

/* ========================================================================== */
/*  Body - identity and classification                                        */
/* ========================================================================== */

static void test_body_classification_dynamic(void)
{
    TEST_BEGIN("Body classification queries (dynamic body)");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 5.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC,
                                              JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyLockWrite* lock = NULL;
        JoltC_Body* body = lock_body_for_write(&ctx, id, &lock);
        TEST_ASSERT_NOT_NULL(body, "body locked");

        if (body) {
            TEST_ASSERT(JoltC_Body_GetID(body) == id, "GetID round-trips the BodyID");
            TEST_ASSERT(JoltC_Body_GetBodyType(body) == JOLTC_BODY_TYPE_RIGID, "body type RIGID");
            TEST_ASSERT(JoltC_Body_IsRigidBody(body), "IsRigidBody");
            TEST_ASSERT(!JoltC_Body_IsSoftBody(body), "not IsSoftBody");
            TEST_ASSERT(JoltC_Body_IsDynamic(body), "IsDynamic");
            TEST_ASSERT(!JoltC_Body_IsStatic(body), "not IsStatic");
            TEST_ASSERT(!JoltC_Body_IsKinematic(body), "not IsKinematic");
            TEST_ASSERT(JoltC_Body_GetMotionType(body) == JOLTC_MOTION_TYPE_DYNAMIC,
                        "GetMotionType DYNAMIC");
            TEST_ASSERT(JoltC_Body_IsActive(body), "active after ACTIVATE");
            TEST_ASSERT(JoltC_Body_CanBeKinematicOrDynamic(body),
                        "a dynamic body can be kinematic or dynamic");
            TEST_ASSERT(JoltC_Body_IsInBroadPhase(body), "added body is in the broad phase");

            /* Value is up to the broad phase; the point is that it answers 0 or 1. */
            int cacheInvalid = JoltC_Body_IsCollisionCacheInvalid(body);
            TEST_ASSERT(cacheInvalid == 0 || cacheInvalid == 1,
                        "IsCollisionCacheInvalid returns a bool");

            /* Layers come from the table set up in setup_physics_context. */
            TEST_ASSERT(JoltC_Body_GetObjectLayer(body) == OBJ_LAYER_DYNAMIC,
                        "object layer is the dynamic layer");
            TEST_ASSERT(JoltC_Body_GetBroadPhaseLayer(body) == BP_LAYER_MOVING,
                        "broad phase layer is the moving layer");

            const JoltC_Shape* shape = JoltC_Body_GetShape(body);
            TEST_ASSERT_NOT_NULL(shape, "GetShape non-null");
            if (shape) {
                TEST_ASSERT(JoltC_Shape_GetSubType(shape) == JOLTC_SHAPE_SUB_TYPE_BOX,
                            "GetShape returns the box it was created with");
            }

            TEST_ASSERT_NOT_NULL(JoltC_Body_GetMotionProperties(body),
                                 "dynamic body has motion properties");
            TEST_ASSERT_NOT_NULL(JoltC_Body_GetMotionPropertiesUnchecked(body),
                                 "unchecked accessor agrees for a dynamic body");
        }

        JoltC_BodyLockWrite_Destroy(lock);
        remove_and_destroy(&ctx, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}

static void test_body_classification_static(void)
{
    TEST_BEGIN("Body classification queries (static body)");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, -1.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_STATIC,
                                              JOLTC_ACTIVATION_DONT_ACTIVATE);
        const JoltC_BodyLockInterface* iface = body_lock_iface(&ctx);
        JoltC_BodyLockWrite* lock = JoltC_BodyLockWrite_Create(iface, id);
        JoltC_Body* body = JoltC_BodyLockWrite_GetBody(lock);
        TEST_ASSERT_NOT_NULL(body, "static body locked");

        if (body) {
            TEST_ASSERT(JoltC_Body_IsStatic(body), "IsStatic");
            TEST_ASSERT(!JoltC_Body_IsDynamic(body), "not IsDynamic");
            TEST_ASSERT(!JoltC_Body_IsKinematic(body), "not IsKinematic");
            TEST_ASSERT(!JoltC_Body_IsActive(body), "static body is never active");
            TEST_ASSERT(JoltC_Body_GetMotionType(body) == JOLTC_MOTION_TYPE_STATIC,
                        "GetMotionType STATIC");
            TEST_ASSERT(JoltC_Body_GetObjectLayer(body) == OBJ_LAYER_STATIC,
                        "object layer is the static layer");
            TEST_ASSERT(JoltC_Body_GetBroadPhaseLayer(body) == BP_LAYER_NON_MOVING,
                        "broad phase layer is the non-moving layer");
            /* The unchecked accessor is the only one legal here, and it must say no. */
            TEST_ASSERT(JoltC_Body_GetMotionPropertiesUnchecked(body) == NULL,
                        "static body has no motion properties");
        }

        JoltC_BodyLockWrite_Destroy(lock);
        remove_and_destroy(&ctx, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}

/* ========================================================================== */
/*  Body - flag and scalar round-trips                                        */
/* ========================================================================== */

static void test_body_flag_roundtrips(void)
{
    TEST_BEGIN("Body boolean flags round-trip");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 5.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC,
                                              JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyLockWrite* lock = NULL;
        JoltC_Body* body = lock_body_for_write(&ctx, id, &lock);
        TEST_ASSERT_NOT_NULL(body, "body locked");

        if (body) {
            JoltC_Body_SetIsSensor(body, 1);
            TEST_ASSERT(JoltC_Body_IsSensor(body), "sensor set");
            JoltC_Body_SetIsSensor(body, 0);
            TEST_ASSERT(!JoltC_Body_IsSensor(body), "sensor cleared");

            JoltC_Body_SetCollideKinematicVsNonDynamic(body, 1);
            TEST_ASSERT(JoltC_Body_GetCollideKinematicVsNonDynamic(body),
                        "collide kinematic vs non-dynamic set");
            JoltC_Body_SetCollideKinematicVsNonDynamic(body, 0);
            TEST_ASSERT(!JoltC_Body_GetCollideKinematicVsNonDynamic(body),
                        "collide kinematic vs non-dynamic cleared");

            JoltC_Body_SetUseManifoldReduction(body, 1);
            TEST_ASSERT(JoltC_Body_GetUseManifoldReduction(body), "manifold reduction set");
            JoltC_Body_SetUseManifoldReduction(body, 0);
            TEST_ASSERT(!JoltC_Body_GetUseManifoldReduction(body), "manifold reduction cleared");

            JoltC_Body_SetApplyGyroscopicForce(body, 1);
            TEST_ASSERT(JoltC_Body_GetApplyGyroscopicForce(body), "gyroscopic force set");
            JoltC_Body_SetApplyGyroscopicForce(body, 0);
            TEST_ASSERT(!JoltC_Body_GetApplyGyroscopicForce(body), "gyroscopic force cleared");

            JoltC_Body_SetEnhancedInternalEdgeRemoval(body, 1);
            TEST_ASSERT(JoltC_Body_GetEnhancedInternalEdgeRemoval(body),
                        "enhanced internal edge removal set");
            JoltC_Body_SetEnhancedInternalEdgeRemoval(body, 0);
            TEST_ASSERT(!JoltC_Body_GetEnhancedInternalEdgeRemoval(body),
                        "enhanced internal edge removal cleared");

            JoltC_Body_SetAllowSleeping(body, 0);
            TEST_ASSERT(!JoltC_Body_GetAllowSleeping(body), "sleeping disallowed");
            JoltC_Body_SetAllowSleeping(body, 1);
            TEST_ASSERT(JoltC_Body_GetAllowSleeping(body), "sleeping allowed again");

            JoltC_Body_ResetSleepTimer(body);
            TEST_ASSERT(1, "ResetSleepTimer does not crash");
        }

        JoltC_BodyLockWrite_Destroy(lock);
        remove_and_destroy(&ctx, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}

static void test_body_pairwise_flags(void)
{
    TEST_BEGIN("Body pairwise flag queries agree with both bodies");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 posA = { -3.0f, 2.0f, 0.0f };
        JoltC_RVec3 posB = { 3.0f, 2.0f, 0.0f };
        JoltC_BodyID idA = create_test_box_body(&ctx, posA, JOLTC_MOTION_TYPE_DYNAMIC,
                                               JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyID idB = create_test_box_body(&ctx, posB, JOLTC_MOTION_TYPE_DYNAMIC,
                                               JOLTC_ACTIVATION_ACTIVATE);

        /* Two single write locks could collide on one striped mutex; the multi lock
         * is the supported way to hold two bodies at once. */
        JoltC_BodyID ids[2];
        ids[0] = idA;
        ids[1] = idB;
        JoltC_BodyLockMultiWrite* lock =
            JoltC_BodyLockInterface_LockMultiWrite(body_lock_iface(&ctx), ids, 2);
        JoltC_Body* a = JoltC_BodyLockMultiWrite_GetBody(lock, 0);
        JoltC_Body* b = JoltC_BodyLockMultiWrite_GetBody(lock, 1);
        TEST_ASSERT_NOT_NULL(a, "body A locked");
        TEST_ASSERT_NOT_NULL(b, "body B locked");

        if (a && b) {
            /* Only the both-on and both-off cases are asserted: whether the pairwise
             * query combines with AND or with OR is upstream's choice, but it must
             * agree with itself when both bodies agree. */
            JoltC_Body_SetUseManifoldReduction(a, 1);
            JoltC_Body_SetUseManifoldReduction(b, 1);
            TEST_ASSERT(JoltC_Body_GetUseManifoldReductionWithBody(a, b),
                        "manifold reduction on for both bodies");
            JoltC_Body_SetUseManifoldReduction(a, 0);
            JoltC_Body_SetUseManifoldReduction(b, 0);
            TEST_ASSERT(!JoltC_Body_GetUseManifoldReductionWithBody(a, b),
                        "manifold reduction off for both bodies");

            JoltC_Body_SetEnhancedInternalEdgeRemoval(a, 1);
            JoltC_Body_SetEnhancedInternalEdgeRemoval(b, 1);
            TEST_ASSERT(JoltC_Body_GetEnhancedInternalEdgeRemovalWithBody(a, b),
                        "edge removal on for both bodies");
            JoltC_Body_SetEnhancedInternalEdgeRemoval(a, 0);
            JoltC_Body_SetEnhancedInternalEdgeRemoval(b, 0);
            TEST_ASSERT(!JoltC_Body_GetEnhancedInternalEdgeRemovalWithBody(a, b),
                        "edge removal off for both bodies");

            /* NULL for the other body must not crash and must not claim agreement. */
            TEST_ASSERT(!JoltC_Body_GetUseManifoldReductionWithBody(a, NULL),
                        "pairwise manifold query tolerates a NULL partner");
            TEST_ASSERT(!JoltC_Body_GetEnhancedInternalEdgeRemovalWithBody(a, NULL),
                        "pairwise edge query tolerates a NULL partner");
        }

        JoltC_BodyLockMultiWrite_Destroy(lock);
        remove_and_destroy(&ctx, idA);
        remove_and_destroy(&ctx, idB);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}

static void test_body_motion_type_roundtrip(void)
{
    TEST_BEGIN("Body motion type round-trips through the enum");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 5.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC,
                                              JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyLockWrite* lock = NULL;
        JoltC_Body* body = lock_body_for_write(&ctx, id, &lock);
        TEST_ASSERT_NOT_NULL(body, "body locked");

        if (body) {
            TEST_ASSERT(JoltC_Body_GetMotionType(body) == JOLTC_MOTION_TYPE_DYNAMIC,
                        "starts DYNAMIC");
            JoltC_Body_SetMotionType(body, JOLTC_MOTION_TYPE_KINEMATIC);
            TEST_ASSERT(JoltC_Body_GetMotionType(body) == JOLTC_MOTION_TYPE_KINEMATIC,
                        "reads back KINEMATIC");
            TEST_ASSERT(JoltC_Body_IsKinematic(body), "IsKinematic follows the motion type");
            TEST_ASSERT(!JoltC_Body_IsDynamic(body), "no longer dynamic");
            /* Put it back before the body leaves the system. */
            JoltC_Body_SetMotionType(body, JOLTC_MOTION_TYPE_DYNAMIC);
            TEST_ASSERT(JoltC_Body_IsDynamic(body), "restored to DYNAMIC");
        }

        JoltC_BodyLockWrite_Destroy(lock);
        remove_and_destroy(&ctx, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}

static void test_body_scalar_roundtrips(void)
{
    TEST_BEGIN("Body friction, restitution and user data round-trip");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 5.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC,
                                              JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyLockWrite* lock = NULL;
        JoltC_Body* body = lock_body_for_write(&ctx, id, &lock);
        TEST_ASSERT_NOT_NULL(body, "body locked");

        if (body) {
            /* Distinct values, so friction and restitution cannot be swapped
             * unnoticed. Both stay inside the ranges Jolt accepts. */
            JoltC_Body_SetFriction(body, 0.375f);
            JoltC_Body_SetRestitution(body, 0.625f);
            TEST_ASSERT_FLOAT_EQ(JoltC_Body_GetFriction(body), 0.375f, 0.0001f,
                                 "friction == 0.375");
            TEST_ASSERT_FLOAT_EQ(JoltC_Body_GetRestitution(body), 0.625f, 0.0001f,
                                 "restitution == 0.625");

            JoltC_Body_SetUserData(body, 0x0123456789ABCDEFULL);
            TEST_ASSERT(JoltC_Body_GetUserData(body) == 0x0123456789ABCDEFULL,
                        "64-bit user data survives intact");
        }

        JoltC_BodyLockWrite_Destroy(lock);
        remove_and_destroy(&ctx, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}

/* ========================================================================== */
/*  Body - velocities                                                         */
/* ========================================================================== */

static void test_body_velocity_roundtrips(void)
{
    TEST_BEGIN("Body linear/angular velocity round-trip (incl. clamped)");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 10.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC,
                                              JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyLockWrite* lock = NULL;
        JoltC_Body* body = lock_body_for_write(&ctx, id, &lock);
        TEST_ASSERT_NOT_NULL(body, "body locked");

        if (body) {
            JoltC_Vec3 linear = { 1.5f, -2.25f, 3.75f };
            JoltC_Vec3 got = { 0.0f, 0.0f, 0.0f };
            JoltC_Body_SetLinearVelocity(body, &linear);
            JoltC_Body_GetLinearVelocity(body, &got);
            TEST_ASSERT_FLOAT_EQ(got.x, 1.5f, 0.0001f, "linear velocity x");
            TEST_ASSERT_FLOAT_EQ(got.y, -2.25f, 0.0001f, "linear velocity y");
            TEST_ASSERT_FLOAT_EQ(got.z, 3.75f, 0.0001f, "linear velocity z");

            JoltC_Vec3 angular = { -0.5f, 1.25f, -2.75f };
            JoltC_Body_SetAngularVelocity(body, &angular);
            JoltC_Body_GetAngularVelocity(body, &got);
            TEST_ASSERT_FLOAT_EQ(got.x, -0.5f, 0.0001f, "angular velocity x");
            TEST_ASSERT_FLOAT_EQ(got.y, 1.25f, 0.0001f, "angular velocity y");
            TEST_ASSERT_FLOAT_EQ(got.z, -2.75f, 0.0001f, "angular velocity z");

            /* Well below the default maxima, so the clamped setters must be
             * value-preserving too. */
            JoltC_Vec3 linearClamped = { -4.5f, 0.75f, 2.5f };
            JoltC_Body_SetLinearVelocityClamped(body, &linearClamped);
            JoltC_Body_GetLinearVelocity(body, &got);
            TEST_ASSERT_FLOAT_EQ(got.x, -4.5f, 0.0001f, "clamped linear velocity x");
            TEST_ASSERT_FLOAT_EQ(got.y, 0.75f, 0.0001f, "clamped linear velocity y");
            TEST_ASSERT_FLOAT_EQ(got.z, 2.5f, 0.0001f, "clamped linear velocity z");

            JoltC_Vec3 angularClamped = { 0.25f, -1.75f, 3.5f };
            JoltC_Body_SetAngularVelocityClamped(body, &angularClamped);
            JoltC_Body_GetAngularVelocity(body, &got);
            TEST_ASSERT_FLOAT_EQ(got.x, 0.25f, 0.0001f, "clamped angular velocity x");
            TEST_ASSERT_FLOAT_EQ(got.y, -1.75f, 0.0001f, "clamped angular velocity y");
            TEST_ASSERT_FLOAT_EQ(got.z, 3.5f, 0.0001f, "clamped angular velocity z");

            /* ResetMotion must clear both. */
            JoltC_Body_ResetMotion(body);
            JoltC_Body_GetLinearVelocity(body, &got);
            TEST_ASSERT_FLOAT_EQ(got.x, 0.0f, 0.0001f, "ResetMotion clears linear x");
            TEST_ASSERT_FLOAT_EQ(got.y, 0.0f, 0.0001f, "ResetMotion clears linear y");
            TEST_ASSERT_FLOAT_EQ(got.z, 0.0f, 0.0001f, "ResetMotion clears linear z");
            JoltC_Body_GetAngularVelocity(body, &got);
            TEST_ASSERT_FLOAT_EQ(got.x, 0.0f, 0.0001f, "ResetMotion clears angular x");
            TEST_ASSERT_FLOAT_EQ(got.y, 0.0f, 0.0001f, "ResetMotion clears angular y");
            TEST_ASSERT_FLOAT_EQ(got.z, 0.0f, 0.0001f, "ResetMotion clears angular z");
        }

        JoltC_BodyLockWrite_Destroy(lock);
        remove_and_destroy(&ctx, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}

static void test_body_point_velocity(void)
{
    TEST_BEGIN("Body point velocity composes linear and angular parts");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        /* At the origin, so the centre of mass of the 1x1x1 box is the origin and a
         * world point is also a point relative to the centre of mass. */
        JoltC_RVec3 pos = { 0.0f, 0.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC,
                                              JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyLockWrite* lock = NULL;
        JoltC_Body* body = lock_body_for_write(&ctx, id, &lock);
        TEST_ASSERT_NOT_NULL(body, "body locked");

        if (body) {
            /* Pure translation: every point moves with the body. */
            JoltC_Vec3 linear = { 1.5f, -2.25f, 3.75f };
            JoltC_Vec3 zero = { 0.0f, 0.0f, 0.0f };
            JoltC_Body_SetLinearVelocity(body, &linear);
            JoltC_Body_SetAngularVelocity(body, &zero);

            JoltC_Vec3 offset = { 2.0f, 3.0f, -4.0f };
            JoltC_Vec3 v = { 0.0f, 0.0f, 0.0f };
            JoltC_Body_GetPointVelocityCOM(body, &offset, &v);
            TEST_ASSERT_FLOAT_EQ(v.x, 1.5f, 0.001f, "translating body: point velocity x");
            TEST_ASSERT_FLOAT_EQ(v.y, -2.25f, 0.001f, "translating body: point velocity y");
            TEST_ASSERT_FLOAT_EQ(v.z, 3.75f, 0.001f, "translating body: point velocity z");

            JoltC_RVec3 worldPoint = { 2.0f, 3.0f, -4.0f };
            JoltC_Vec3 vw = { 0.0f, 0.0f, 0.0f };
            JoltC_Body_GetPointVelocity(body, &worldPoint, &vw);
            TEST_ASSERT_FLOAT_EQ(vw.x, 1.5f, 0.001f, "world point velocity x");
            TEST_ASSERT_FLOAT_EQ(vw.y, -2.25f, 0.001f, "world point velocity y");
            TEST_ASSERT_FLOAT_EQ(vw.z, 3.75f, 0.001f, "world point velocity z");

            /* Pure rotation about +Y: a point on +X moves towards -Z (omega x r).
             * Reversing the cross product would put it on +Z. */
            JoltC_Vec3 spinY = { 0.0f, 4.0f, 0.0f };
            JoltC_Body_SetLinearVelocity(body, &zero);
            JoltC_Body_SetAngularVelocity(body, &spinY);

            JoltC_Vec3 onX = { 1.0f, 0.0f, 0.0f };
            JoltC_Body_GetPointVelocityCOM(body, &onX, &v);
            TEST_ASSERT_FLOAT_EQ(v.x, 0.0f, 0.001f, "spinning body: point velocity x == 0");
            TEST_ASSERT_FLOAT_EQ(v.y, 0.0f, 0.001f, "spinning body: point velocity y == 0");
            TEST_ASSERT_FLOAT_EQ(v.z, -4.0f, 0.001f, "spinning body: point velocity z == -4");

            JoltC_RVec3 worldOnX = { 1.0f, 0.0f, 0.0f };
            JoltC_Body_GetPointVelocity(body, &worldOnX, &vw);
            TEST_ASSERT_FLOAT_EQ(vw.z, -4.0f, 0.001f,
                                 "world and COM point velocity agree at the same point");

            JoltC_Body_ResetMotion(body);
        }

        JoltC_BodyLockWrite_Destroy(lock);
        remove_and_destroy(&ctx, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}

/* ========================================================================== */
/*  Body - forces, torques, impulses                                          */
/* ========================================================================== */

static void test_body_force_and_torque(void)
{
    TEST_BEGIN("Body force/torque accumulate and reset");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 0.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC,
                                              JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyLockWrite* lock = NULL;
        JoltC_Body* body = lock_body_for_write(&ctx, id, &lock);
        TEST_ASSERT_NOT_NULL(body, "body locked");

        if (body) {
            JoltC_Vec3 got = { 0.0f, 0.0f, 0.0f };

            JoltC_Body_ResetForce(body);
            JoltC_Vec3 force = { 3.0f, -2.0f, 1.0f };
            JoltC_Body_AddForce(body, &force);
            JoltC_Body_GetAccumulatedForce(body, &got);
            TEST_ASSERT_FLOAT_EQ(got.x, 3.0f, 0.001f, "accumulated force x");
            TEST_ASSERT_FLOAT_EQ(got.y, -2.0f, 0.001f, "accumulated force y");
            TEST_ASSERT_FLOAT_EQ(got.z, 1.0f, 0.001f, "accumulated force z");

            JoltC_Body_AddForce(body, &force);
            JoltC_Body_GetAccumulatedForce(body, &got);
            TEST_ASSERT_FLOAT_EQ(got.x, 6.0f, 0.001f, "second AddForce accumulates x");
            TEST_ASSERT_FLOAT_EQ(got.y, -4.0f, 0.001f, "second AddForce accumulates y");
            TEST_ASSERT_FLOAT_EQ(got.z, 2.0f, 0.001f, "second AddForce accumulates z");

            JoltC_Body_ResetForce(body);
            JoltC_Body_GetAccumulatedForce(body, &got);
            TEST_ASSERT_FLOAT_EQ(got.x, 0.0f, 0.001f, "ResetForce clears x");
            TEST_ASSERT_FLOAT_EQ(got.y, 0.0f, 0.001f, "ResetForce clears y");
            TEST_ASSERT_FLOAT_EQ(got.z, 0.0f, 0.001f, "ResetForce clears z");

            JoltC_Body_ResetTorque(body);
            JoltC_Vec3 torque = { -1.5f, 2.5f, 0.75f };
            JoltC_Body_AddTorque(body, &torque);
            JoltC_Body_GetAccumulatedTorque(body, &got);
            TEST_ASSERT_FLOAT_EQ(got.x, -1.5f, 0.001f, "accumulated torque x");
            TEST_ASSERT_FLOAT_EQ(got.y, 2.5f, 0.001f, "accumulated torque y");
            TEST_ASSERT_FLOAT_EQ(got.z, 0.75f, 0.001f, "accumulated torque z");

            JoltC_Body_ResetTorque(body);
            JoltC_Body_GetAccumulatedTorque(body, &got);
            TEST_ASSERT_FLOAT_EQ(got.x, 0.0f, 0.001f, "ResetTorque clears x");
            TEST_ASSERT_FLOAT_EQ(got.y, 0.0f, 0.001f, "ResetTorque clears y");
            TEST_ASSERT_FLOAT_EQ(got.z, 0.0f, 0.001f, "ResetTorque clears z");

            /* A force of +3X applied 2 units along +Y from the centre of mass is a
             * torque of r x F = (0,2,0) x (3,0,0) = (0,0,-6). If force and position
             * were ever exchanged, the force would read (0,2,0) and the torque
             * would come out as (0,0,+6): both assertions below would fire. */
            JoltC_Body_ResetForce(body);
            JoltC_Body_ResetTorque(body);
            JoltC_Vec3 sideForce = { 3.0f, 0.0f, 0.0f };
            JoltC_RVec3 lever = { 0.0f, 2.0f, 0.0f };
            JoltC_Body_AddForceAtPosition(body, &sideForce, &lever);

            JoltC_Body_GetAccumulatedForce(body, &got);
            TEST_ASSERT_FLOAT_EQ(got.x, 3.0f, 0.001f, "AddForceAtPosition: force x == 3");
            TEST_ASSERT_FLOAT_EQ(got.y, 0.0f, 0.001f, "AddForceAtPosition: force y == 0");
            TEST_ASSERT_FLOAT_EQ(got.z, 0.0f, 0.001f, "AddForceAtPosition: force z == 0");

            JoltC_Body_GetAccumulatedTorque(body, &got);
            TEST_ASSERT_FLOAT_EQ(got.x, 0.0f, 0.001f, "AddForceAtPosition: torque x == 0");
            TEST_ASSERT_FLOAT_EQ(got.y, 0.0f, 0.001f, "AddForceAtPosition: torque y == 0");
            TEST_ASSERT_FLOAT_EQ(got.z, -6.0f, 0.001f, "AddForceAtPosition: torque z == -6");

            JoltC_Body_ResetForce(body);
            JoltC_Body_ResetTorque(body);
        }

        JoltC_BodyLockWrite_Destroy(lock);
        remove_and_destroy(&ctx, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}

static void test_body_impulses(void)
{
    TEST_BEGIN("Body impulses change velocity in the right direction");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 0.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC,
                                              JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyLockWrite* lock = NULL;
        JoltC_Body* body = lock_body_for_write(&ctx, id, &lock);
        TEST_ASSERT_NOT_NULL(body, "body locked");

        if (body) {
            JoltC_MotionProperties* mp = JoltC_Body_GetMotionProperties(body);
            TEST_ASSERT_NOT_NULL(mp, "motion properties available");
            if (mp) {
                /* Unit inverse mass makes the linear impulse arithmetic exact:
                 * dv = impulse * invMass. No simulation step is involved. */
                JoltC_MotionProperties_SetInverseMass(mp, 1.0f);
                JoltC_Body_ResetMotion(body);

                JoltC_Vec3 impulse = { 3.0f, -2.0f, 1.0f };
                JoltC_Vec3 v = { 0.0f, 0.0f, 0.0f };
                JoltC_Body_AddImpulse(body, &impulse);
                JoltC_Body_GetLinearVelocity(body, &v);
                TEST_ASSERT_FLOAT_EQ(v.x, 3.0f, 0.001f, "AddImpulse: dv x == 3");
                TEST_ASSERT_FLOAT_EQ(v.y, -2.0f, 0.001f, "AddImpulse: dv y == -2");
                TEST_ASSERT_FLOAT_EQ(v.z, 1.0f, 0.001f, "AddImpulse: dv z == 1");

                /* The box's inverse inertia is diagonal and its inertia rotation is
                 * the identity, so a +Y angular impulse spins it about +Y only. */
                JoltC_Body_ResetMotion(body);
                JoltC_Vec3 angularImpulse = { 0.0f, 1.5f, 0.0f };
                JoltC_Vec3 av = { 0.0f, 0.0f, 0.0f };
                JoltC_Body_AddAngularImpulse(body, &angularImpulse);
                JoltC_Body_GetAngularVelocity(body, &av);
                TEST_ASSERT(av.y > 0.0f, "AddAngularImpulse: spins about +Y");
                TEST_ASSERT_FLOAT_EQ(av.x, 0.0f, 0.001f, "AddAngularImpulse: no x spin");
                TEST_ASSERT_FLOAT_EQ(av.z, 0.0f, 0.001f, "AddAngularImpulse: no z spin");

                /* A +Z impulse applied 2 units along +X torques the body about -Y
                 * (r x p = (2,0,0) x (0,0,3) = (0,-6,0)). Exchanging impulse and
                 * position would move the body along +X and spin it about +Y. */
                JoltC_Body_ResetMotion(body);
                JoltC_Vec3 offsetImpulse = { 0.0f, 0.0f, 3.0f };
                JoltC_RVec3 offsetPoint = { 2.0f, 0.0f, 0.0f };
                JoltC_Body_AddImpulseAtPosition(body, &offsetImpulse, &offsetPoint);
                JoltC_Body_GetLinearVelocity(body, &v);
                JoltC_Body_GetAngularVelocity(body, &av);
                TEST_ASSERT_FLOAT_EQ(v.z, 3.0f, 0.001f, "AddImpulseAtPosition: dv z == 3");
                TEST_ASSERT_FLOAT_EQ(v.x, 0.0f, 0.001f, "AddImpulseAtPosition: dv x == 0");
                TEST_ASSERT(av.y < 0.0f, "AddImpulseAtPosition: torques about -Y");

                JoltC_Body_ResetMotion(body);
            }
        }

        JoltC_BodyLockWrite_Destroy(lock);
        remove_and_destroy(&ctx, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}

static void test_body_move_kinematic(void)
{
    TEST_BEGIN("Body MoveKinematic derives velocity from target and dt");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 0.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_KINEMATIC,
                                              JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyLockWrite* lock = NULL;
        JoltC_Body* body = lock_body_for_write(&ctx, id, &lock);
        TEST_ASSERT_NOT_NULL(body, "kinematic body locked");

        if (body) {
            TEST_ASSERT(JoltC_Body_IsKinematic(body), "body is kinematic");

            /* 4 metres up in 0.1 s is 40 m/s. A deltaTime that got multiplied
             * instead of divided would read 0.4 instead. */
            JoltC_RVec3 target = { 0.0f, 4.0f, 0.0f };
            JoltC_Quat targetRotation = identity_quat();
            JoltC_Body_MoveKinematic(body, &target, &targetRotation, 0.1f);

            JoltC_Vec3 v = { 0.0f, 0.0f, 0.0f };
            JoltC_Body_GetLinearVelocity(body, &v);
            TEST_ASSERT(v.y > 0.0f, "MoveKinematic moves towards the target");
            TEST_ASSERT_FLOAT_EQ(v.y, 40.0f, 0.1f, "MoveKinematic: 4 m in 0.1 s == 40 m/s");
            TEST_ASSERT_FLOAT_EQ(v.x, 0.0f, 0.001f, "MoveKinematic: no x drift");
            TEST_ASSERT_FLOAT_EQ(v.z, 0.0f, 0.001f, "MoveKinematic: no z drift");
        }

        JoltC_BodyLockWrite_Destroy(lock);
        remove_and_destroy(&ctx, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}

static void test_body_buoyancy_impulse(void)
{
    TEST_BEGIN("Body ApplyBuoyancyImpulse lifts a submerged body");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 0.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC,
                                              JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyLockWrite* lock = NULL;
        JoltC_Body* body = lock_body_for_write(&ctx, id, &lock);
        TEST_ASSERT_NOT_NULL(body, "body locked");

        if (body) {
            JoltC_Body_ResetMotion(body);

            /* Surface well above the box, so it is fully submerged. Buoyancy above
             * 1 means the body is lighter than the fluid it displaces, and with no
             * drag the only outcome can be an upward impulse. */
            JoltC_RVec3 surfacePosition = { 0.0f, 10.0f, 0.0f };
            JoltC_Vec3 surfaceNormal = { 0.0f, 1.0f, 0.0f };
            JoltC_Vec3 fluidVelocity = { 0.0f, 0.0f, 0.0f };
            JoltC_Vec3 gravity = { 0.0f, -9.81f, 0.0f };
            int applied = JoltC_Body_ApplyBuoyancyImpulse(
                body, &surfacePosition, &surfaceNormal,
                2.0f,   /* buoyancy: > 1 floats */
                0.0f,   /* linear drag */
                0.0f,   /* angular drag */
                &fluidVelocity, &gravity, 1.0f / 60.0f);

            TEST_ASSERT(applied != 0, "an impulse is applied to a submerged body");

            JoltC_Vec3 v = { 0.0f, 0.0f, 0.0f };
            JoltC_Body_GetLinearVelocity(body, &v);
            TEST_ASSERT(v.y > 0.0f, "buoyancy pushes the body up");

            JoltC_Body_ResetMotion(body);
        }

        JoltC_BodyLockWrite_Destroy(lock);
        remove_and_destroy(&ctx, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}

/* ========================================================================== */
/*  Body - transforms and bounds                                              */
/* ========================================================================== */

static void test_body_transforms(void)
{
    TEST_BEGIN("Body transforms report the position it was created at");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        /* Asymmetric and signed, so a transposed pair of columns or a swapped
         * component cannot pass. */
        JoltC_RVec3 pos = { 1.5f, -2.25f, 3.75f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC,
                                              JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyLockWrite* lock = NULL;
        JoltC_Body* body = lock_body_for_write(&ctx, id, &lock);
        TEST_ASSERT_NOT_NULL(body, "body locked");

        if (body) {
            JoltC_RVec3 gotPos = { 0.0f, 0.0f, 0.0f };
            JoltC_Body_GetPosition(body, &gotPos);
            TEST_ASSERT_FLOAT_EQ(gotPos.x, 1.5f, 0.001f, "position x");
            TEST_ASSERT_FLOAT_EQ(gotPos.y, -2.25f, 0.001f, "position y");
            TEST_ASSERT_FLOAT_EQ(gotPos.z, 3.75f, 0.001f, "position z");

            JoltC_Quat gotRot = { 1.0f, 1.0f, 1.0f, 0.0f };
            JoltC_Body_GetRotation(body, &gotRot);
            TEST_ASSERT_FLOAT_EQ(gotRot.x, 0.0f, 0.001f, "rotation x == 0");
            TEST_ASSERT_FLOAT_EQ(gotRot.y, 0.0f, 0.001f, "rotation y == 0");
            TEST_ASSERT_FLOAT_EQ(gotRot.z, 0.0f, 0.001f, "rotation z == 0");
            TEST_ASSERT_FLOAT_EQ(gotRot.w, 1.0f, 0.001f, "rotation w == 1");

            /* Mat44 is column-major, so the translation lives in m[12..14]. */
            JoltC_Mat44 world;
            JoltC_Body_GetWorldTransform(body, &world);
            TEST_ASSERT_FLOAT_EQ(world.m[0], 1.0f, 0.001f, "world transform m[0] == 1");
            TEST_ASSERT_FLOAT_EQ(world.m[5], 1.0f, 0.001f, "world transform m[5] == 1");
            TEST_ASSERT_FLOAT_EQ(world.m[10], 1.0f, 0.001f, "world transform m[10] == 1");
            TEST_ASSERT_FLOAT_EQ(world.m[12], 1.5f, 0.001f, "world transform translation x");
            TEST_ASSERT_FLOAT_EQ(world.m[13], -2.25f, 0.001f, "world transform translation y");
            TEST_ASSERT_FLOAT_EQ(world.m[14], 3.75f, 0.001f, "world transform translation z");

            /* The box is centred on its origin, so the centre of mass is the
             * position. */
            JoltC_RVec3 com = { 0.0f, 0.0f, 0.0f };
            JoltC_Body_GetCenterOfMassPosition(body, &com);
            TEST_ASSERT_FLOAT_EQ(com.x, 1.5f, 0.001f, "centre of mass x");
            TEST_ASSERT_FLOAT_EQ(com.y, -2.25f, 0.001f, "centre of mass y");
            TEST_ASSERT_FLOAT_EQ(com.z, 3.75f, 0.001f, "centre of mass z");

            JoltC_Mat44 comTransform;
            JoltC_Body_GetCenterOfMassTransform(body, &comTransform);
            TEST_ASSERT_FLOAT_EQ(comTransform.m[12], 1.5f, 0.001f, "COM transform translation x");
            TEST_ASSERT_FLOAT_EQ(comTransform.m[13], -2.25f, 0.001f, "COM transform translation y");
            TEST_ASSERT_FLOAT_EQ(comTransform.m[14], 3.75f, 0.001f, "COM transform translation z");

            /* With the identity rotation the inverse is a plain negation. */
            JoltC_Mat44 invComTransform;
            JoltC_Body_GetInverseCenterOfMassTransform(body, &invComTransform);
            TEST_ASSERT_FLOAT_EQ(invComTransform.m[12], -1.5f, 0.001f,
                                 "inverse COM transform translation x");
            TEST_ASSERT_FLOAT_EQ(invComTransform.m[13], 2.25f, 0.001f,
                                 "inverse COM transform translation y");
            TEST_ASSERT_FLOAT_EQ(invComTransform.m[14], -3.75f, 0.001f,
                                 "inverse COM transform translation z");

            /* Bounds of a 1x1x1 box: centred on the body, one unit across. The
             * extents are range-checked rather than pinned, in case upstream ever
             * pads them. */
            JoltC_AABox bounds;
            JoltC_Body_GetWorldSpaceBounds(body, &bounds);
            TEST_ASSERT(bounds.min.x < bounds.max.x, "bounds x is not inverted");
            TEST_ASSERT(bounds.min.y < bounds.max.y, "bounds y is not inverted");
            TEST_ASSERT(bounds.min.z < bounds.max.z, "bounds z is not inverted");
            TEST_ASSERT_FLOAT_EQ(0.5f * (bounds.min.x + bounds.max.x), 1.5f, 0.05f,
                                 "bounds centre x");
            TEST_ASSERT_FLOAT_EQ(0.5f * (bounds.min.y + bounds.max.y), -2.25f, 0.05f,
                                 "bounds centre y");
            TEST_ASSERT_FLOAT_EQ(0.5f * (bounds.min.z + bounds.max.z), 3.75f, 0.05f,
                                 "bounds centre z");
            TEST_ASSERT(bounds.max.x - bounds.min.x > 0.9f &&
                        bounds.max.x - bounds.min.x < 1.5f, "bounds width about 1");
            TEST_ASSERT(bounds.max.y - bounds.min.y > 0.9f &&
                        bounds.max.y - bounds.min.y < 1.5f, "bounds height about 1");
            TEST_ASSERT(bounds.max.z - bounds.min.z > 0.9f &&
                        bounds.max.z - bounds.min.z < 1.5f, "bounds depth about 1");

            /* A dynamic box has a positive definite, diagonal inverse inertia. */
            JoltC_Mat44 invInertia;
            JoltC_Body_GetInverseInertia(body, &invInertia);
            TEST_ASSERT(invInertia.m[0] > 0.0f, "inverse inertia m[0] > 0");
            TEST_ASSERT(invInertia.m[5] > 0.0f, "inverse inertia m[5] > 0");
            TEST_ASSERT(invInertia.m[10] > 0.0f, "inverse inertia m[10] > 0");
        }

        JoltC_BodyLockWrite_Destroy(lock);
        remove_and_destroy(&ctx, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}

/* ========================================================================== */
/*  MotionProperties                                                          */
/* ========================================================================== */

static void test_motion_properties_damping_and_mass(void)
{
    TEST_BEGIN("MotionProperties damping and mass round-trip");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 5.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC,
                                              JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyLockWrite* lock = NULL;
        JoltC_Body* body = lock_body_for_write(&ctx, id, &lock);
        TEST_ASSERT_NOT_NULL(body, "body locked");

        if (body) {
            JoltC_MotionProperties* mp = JoltC_Body_GetMotionProperties(body);
            TEST_ASSERT_NOT_NULL(mp, "motion properties available");
            if (mp) {
                TEST_ASSERT(JoltC_MotionProperties_GetAllowedDOFs(mp) == JOLTC_ALLOWED_DOFS_ALL,
                            "a plain dynamic body allows all six DOFs");

                /* Distinct values so linear and angular damping cannot be swapped. */
                JoltC_MotionProperties_SetLinearDamping(mp, 0.125f);
                JoltC_MotionProperties_SetAngularDamping(mp, 0.375f);
                TEST_ASSERT_FLOAT_EQ(JoltC_MotionProperties_GetLinearDamping(mp), 0.125f,
                                     0.0001f, "linear damping == 0.125");
                TEST_ASSERT_FLOAT_EQ(JoltC_MotionProperties_GetAngularDamping(mp), 0.375f,
                                     0.0001f, "angular damping == 0.375");

                JoltC_MotionProperties_SetInverseMass(mp, 0.5f);
                TEST_ASSERT_FLOAT_EQ(JoltC_MotionProperties_GetInverseMassUnchecked(mp), 0.5f,
                                     0.0001f, "inverse mass == 0.5");

                /* ScaleToMass takes a mass, not an inverse mass: 4 kg is 0.25. */
                JoltC_MotionProperties_ScaleToMass(mp, 4.0f);
                TEST_ASSERT_FLOAT_EQ(JoltC_MotionProperties_GetInverseMassUnchecked(mp), 0.25f,
                                     0.0001f, "ScaleToMass(4) gives inverse mass 0.25");
            }
        }

        JoltC_BodyLockWrite_Destroy(lock);
        remove_and_destroy(&ctx, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}

static void test_motion_properties_inertia(void)
{
    TEST_BEGIN("MotionProperties inertia round-trip");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 5.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC,
                                              JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyLockWrite* lock = NULL;
        JoltC_Body* body = lock_body_for_write(&ctx, id, &lock);
        TEST_ASSERT_NOT_NULL(body, "body locked");

        if (body) {
            JoltC_MotionProperties* mp = JoltC_Body_GetMotionProperties(body);
            TEST_ASSERT_NOT_NULL(mp, "motion properties available");
            if (mp) {
                /* Three different diagonal entries, so a permuted diagonal shows up. */
                JoltC_Vec3 diagonal = { 0.5f, 1.25f, 2.75f };
                JoltC_Quat inertiaRotation = identity_quat();
                JoltC_MotionProperties_SetInverseInertia(mp, &diagonal, &inertiaRotation);

                JoltC_Vec3 gotDiagonal = { 0.0f, 0.0f, 0.0f };
                JoltC_MotionProperties_GetInverseInertiaDiagonal(mp, &gotDiagonal);
                TEST_ASSERT_FLOAT_EQ(gotDiagonal.x, 0.5f, 0.0001f, "inverse inertia diagonal x");
                TEST_ASSERT_FLOAT_EQ(gotDiagonal.y, 1.25f, 0.0001f, "inverse inertia diagonal y");
                TEST_ASSERT_FLOAT_EQ(gotDiagonal.z, 2.75f, 0.0001f, "inverse inertia diagonal z");

                JoltC_Quat gotRotation = { 1.0f, 1.0f, 1.0f, 0.0f };
                JoltC_MotionProperties_GetInertiaRotation(mp, &gotRotation);
                TEST_ASSERT_FLOAT_EQ(gotRotation.x, 0.0f, 0.0001f, "inertia rotation x == 0");
                TEST_ASSERT_FLOAT_EQ(gotRotation.y, 0.0f, 0.0001f, "inertia rotation y == 0");
                TEST_ASSERT_FLOAT_EQ(gotRotation.z, 0.0f, 0.0001f, "inertia rotation z == 0");
                TEST_ASSERT_FLOAT_EQ(gotRotation.w, 1.0f, 0.0001f, "inertia rotation w == 1");

                /* SetMassProperties takes the allowed DOFs alongside the mass, and
                 * the DOF argument must not be dropped or reordered. The inertia is
                 * isotropic (column-major diagonal at 0, 5, 10, 15) so no claim is
                 * made here about how the principal axes come back out. */
                JoltC_MassProperties massProperties = {
                    2.0f,
                    { { 2.0f, 0.0f, 0.0f, 0.0f,
                        0.0f, 2.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 2.0f, 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f } }
                };
                JoltC_MotionProperties_SetMassProperties(mp, JOLTC_ALLOWED_DOFS_PLANE_2D,
                                                         &massProperties);
                TEST_ASSERT(JoltC_MotionProperties_GetAllowedDOFs(mp) == JOLTC_ALLOWED_DOFS_PLANE_2D,
                            "allowed DOFs round-trip as PLANE_2D");
                TEST_ASSERT_FLOAT_EQ(JoltC_MotionProperties_GetInverseMassUnchecked(mp), 0.5f,
                                     0.0001f, "SetMassProperties(mass 2) gives inverse mass 0.5");
            }
        }

        JoltC_BodyLockWrite_Destroy(lock);
        remove_and_destroy(&ctx, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}

/* ========================================================================== */
/*  Null safety - every entry point is documented as guarded                   */
/* ========================================================================== */

static void test_body_null_safety(void)
{
    TEST_BEGIN("Body functions tolerate a NULL body");
    {
        /* Queries fall back to the documented neutral answer. */
        TEST_ASSERT(JoltC_Body_GetID(NULL) == JOLTC_BODY_ID_INVALID, "GetID(NULL) is invalid");
        TEST_ASSERT(JoltC_Body_GetBodyType(NULL) == JOLTC_BODY_TYPE_RIGID,
                    "GetBodyType(NULL) is RIGID");
        TEST_ASSERT(!JoltC_Body_IsRigidBody(NULL), "IsRigidBody(NULL) is false");
        TEST_ASSERT(!JoltC_Body_IsSoftBody(NULL), "IsSoftBody(NULL) is false");
        TEST_ASSERT(!JoltC_Body_IsActive(NULL), "IsActive(NULL) is false");
        TEST_ASSERT(!JoltC_Body_IsStatic(NULL), "IsStatic(NULL) is false");
        TEST_ASSERT(!JoltC_Body_IsKinematic(NULL), "IsKinematic(NULL) is false");
        TEST_ASSERT(!JoltC_Body_IsDynamic(NULL), "IsDynamic(NULL) is false");
        TEST_ASSERT(!JoltC_Body_CanBeKinematicOrDynamic(NULL),
                    "CanBeKinematicOrDynamic(NULL) is false");
        TEST_ASSERT(!JoltC_Body_IsSensor(NULL), "IsSensor(NULL) is false");
        TEST_ASSERT(!JoltC_Body_GetCollideKinematicVsNonDynamic(NULL),
                    "GetCollideKinematicVsNonDynamic(NULL) is false");
        TEST_ASSERT(!JoltC_Body_GetUseManifoldReduction(NULL),
                    "GetUseManifoldReduction(NULL) is false");
        TEST_ASSERT(!JoltC_Body_GetUseManifoldReductionWithBody(NULL, NULL),
                    "GetUseManifoldReductionWithBody(NULL, NULL) is false");
        TEST_ASSERT(!JoltC_Body_GetApplyGyroscopicForce(NULL),
                    "GetApplyGyroscopicForce(NULL) is false");
        TEST_ASSERT(!JoltC_Body_GetEnhancedInternalEdgeRemoval(NULL),
                    "GetEnhancedInternalEdgeRemoval(NULL) is false");
        TEST_ASSERT(!JoltC_Body_GetEnhancedInternalEdgeRemovalWithBody(NULL, NULL),
                    "GetEnhancedInternalEdgeRemovalWithBody(NULL, NULL) is false");
        TEST_ASSERT(JoltC_Body_GetMotionType(NULL) == JOLTC_MOTION_TYPE_STATIC,
                    "GetMotionType(NULL) is STATIC");
        TEST_ASSERT(JoltC_Body_GetBroadPhaseLayer(NULL) == 0, "GetBroadPhaseLayer(NULL) is 0");
        TEST_ASSERT(JoltC_Body_GetObjectLayer(NULL) == JOLTC_OBJECT_LAYER_INVALID,
                    "GetObjectLayer(NULL) is invalid");
        TEST_ASSERT(!JoltC_Body_GetAllowSleeping(NULL), "GetAllowSleeping(NULL) is false");
        TEST_ASSERT_FLOAT_EQ(JoltC_Body_GetFriction(NULL), 0.0f, 0.0001f,
                             "GetFriction(NULL) is 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_Body_GetRestitution(NULL), 0.0f, 0.0001f,
                             "GetRestitution(NULL) is 0");
        TEST_ASSERT(!JoltC_Body_IsInBroadPhase(NULL), "IsInBroadPhase(NULL) is false");
        TEST_ASSERT(!JoltC_Body_IsCollisionCacheInvalid(NULL),
                    "IsCollisionCacheInvalid(NULL) is false");
        TEST_ASSERT(JoltC_Body_GetShape(NULL) == NULL, "GetShape(NULL) is NULL");
        TEST_ASSERT(JoltC_Body_GetMotionProperties(NULL) == NULL,
                    "GetMotionProperties(NULL) is NULL");
        TEST_ASSERT(JoltC_Body_GetMotionPropertiesUnchecked(NULL) == NULL,
                    "GetMotionPropertiesUnchecked(NULL) is NULL");
        TEST_ASSERT(JoltC_Body_GetUserData(NULL) == 0, "GetUserData(NULL) is 0");

        /* Out-parameter getters leave the destination alone. */
        JoltC_Vec3 sentinelVec = { -99.0f, -99.0f, -99.0f };
        JoltC_Body_GetLinearVelocity(NULL, &sentinelVec);
        JoltC_Body_GetAngularVelocity(NULL, &sentinelVec);
        JoltC_Body_GetAccumulatedForce(NULL, &sentinelVec);
        JoltC_Body_GetAccumulatedTorque(NULL, &sentinelVec);
        TEST_ASSERT_FLOAT_EQ(sentinelVec.x, -99.0f, 0.0001f,
                             "vector out-params untouched for a NULL body");

        JoltC_RVec3 sentinelPos = { -99.0f, -99.0f, -99.0f };
        JoltC_Body_GetPosition(NULL, &sentinelPos);
        JoltC_Body_GetCenterOfMassPosition(NULL, &sentinelPos);
        TEST_ASSERT_FLOAT_EQ(sentinelPos.y, -99.0f, 0.0001f,
                             "position out-params untouched for a NULL body");

        JoltC_Quat sentinelQuat = { -99.0f, -99.0f, -99.0f, -99.0f };
        JoltC_Body_GetRotation(NULL, &sentinelQuat);
        TEST_ASSERT_FLOAT_EQ(sentinelQuat.w, -99.0f, 0.0001f,
                             "rotation out-param untouched for a NULL body");

        JoltC_Mat44 sentinelMat;
        sentinelMat.m[12] = -99.0f;
        JoltC_Body_GetWorldTransform(NULL, &sentinelMat);
        JoltC_Body_GetCenterOfMassTransform(NULL, &sentinelMat);
        JoltC_Body_GetInverseCenterOfMassTransform(NULL, &sentinelMat);
        JoltC_Body_GetInverseInertia(NULL, &sentinelMat);
        TEST_ASSERT_FLOAT_EQ(sentinelMat.m[12], -99.0f, 0.0001f,
                             "matrix out-params untouched for a NULL body");

        JoltC_AABox sentinelBox;
        sentinelBox.min.x = -99.0f;
        JoltC_Body_GetWorldSpaceBounds(NULL, &sentinelBox);
        TEST_ASSERT_FLOAT_EQ(sentinelBox.min.x, -99.0f, 0.0001f,
                             "bounds out-param untouched for a NULL body");

        JoltC_Vec3 vec = { 1.0f, 2.0f, 3.0f };
        JoltC_RVec3 point = { 1.0f, 2.0f, 3.0f };
        JoltC_Quat quat = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_Body_GetPointVelocityCOM(NULL, &vec, &sentinelVec);
        JoltC_Body_GetPointVelocity(NULL, &point, &sentinelVec);

        /* Mutators simply do nothing. */
        JoltC_Body_SetIsSensor(NULL, 1);
        JoltC_Body_SetCollideKinematicVsNonDynamic(NULL, 1);
        JoltC_Body_SetUseManifoldReduction(NULL, 1);
        JoltC_Body_SetApplyGyroscopicForce(NULL, 1);
        JoltC_Body_SetEnhancedInternalEdgeRemoval(NULL, 1);
        JoltC_Body_SetMotionType(NULL, JOLTC_MOTION_TYPE_KINEMATIC);
        JoltC_Body_SetAllowSleeping(NULL, 1);
        JoltC_Body_ResetSleepTimer(NULL);
        JoltC_Body_SetFriction(NULL, 0.5f);
        JoltC_Body_SetRestitution(NULL, 0.5f);
        JoltC_Body_SetUserData(NULL, 1);
        JoltC_Body_SetLinearVelocity(NULL, &vec);
        JoltC_Body_SetLinearVelocityClamped(NULL, &vec);
        JoltC_Body_SetAngularVelocity(NULL, &vec);
        JoltC_Body_SetAngularVelocityClamped(NULL, &vec);
        JoltC_Body_AddForce(NULL, &vec);
        JoltC_Body_AddForceAtPosition(NULL, &vec, &point);
        JoltC_Body_AddTorque(NULL, &vec);
        JoltC_Body_ResetForce(NULL);
        JoltC_Body_ResetTorque(NULL);
        JoltC_Body_ResetMotion(NULL);
        JoltC_Body_AddImpulse(NULL, &vec);
        JoltC_Body_AddImpulseAtPosition(NULL, &vec, &point);
        JoltC_Body_AddAngularImpulse(NULL, &vec);
        JoltC_Body_MoveKinematic(NULL, &point, &quat, 1.0f / 60.0f);
        TEST_ASSERT(JoltC_Body_ApplyBuoyancyImpulse(NULL, &point, &vec, 1.0f, 0.0f, 0.0f,
                                                    &vec, &vec, 1.0f / 60.0f) == 0,
                    "ApplyBuoyancyImpulse(NULL) reports no impulse");
        TEST_ASSERT(1, "no NULL body call crashed");
    }
    TEST_END();
}

static void test_body_null_out_params(void)
{
    TEST_BEGIN("Body out-param functions tolerate a NULL destination");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        JoltC_RVec3 pos = { 0.0f, 5.0f, 0.0f };
        JoltC_BodyID id = create_test_box_body(&ctx, pos, JOLTC_MOTION_TYPE_DYNAMIC,
                                              JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyLockWrite* lock = NULL;
        JoltC_Body* body = lock_body_for_write(&ctx, id, &lock);
        TEST_ASSERT_NOT_NULL(body, "body locked");

        if (body) {
            JoltC_Vec3 vec = { 1.0f, 2.0f, 3.0f };
            JoltC_RVec3 point = { 1.0f, 2.0f, 3.0f };

            JoltC_Body_GetLinearVelocity(body, NULL);
            JoltC_Body_GetAngularVelocity(body, NULL);
            JoltC_Body_GetAccumulatedForce(body, NULL);
            JoltC_Body_GetAccumulatedTorque(body, NULL);
            JoltC_Body_GetPosition(body, NULL);
            JoltC_Body_GetRotation(body, NULL);
            JoltC_Body_GetWorldTransform(body, NULL);
            JoltC_Body_GetCenterOfMassPosition(body, NULL);
            JoltC_Body_GetCenterOfMassTransform(body, NULL);
            JoltC_Body_GetInverseCenterOfMassTransform(body, NULL);
            JoltC_Body_GetWorldSpaceBounds(body, NULL);
            JoltC_Body_GetInverseInertia(body, NULL);
            JoltC_Body_GetPointVelocityCOM(body, &vec, NULL);
            JoltC_Body_GetPointVelocityCOM(body, NULL, &vec);
            JoltC_Body_GetPointVelocity(body, &point, NULL);
            JoltC_Body_GetPointVelocity(body, NULL, &vec);

            /* NULL inputs to the mutators are ignored, not dereferenced. */
            JoltC_Body_SetLinearVelocity(body, NULL);
            JoltC_Body_SetLinearVelocityClamped(body, NULL);
            JoltC_Body_SetAngularVelocity(body, NULL);
            JoltC_Body_SetAngularVelocityClamped(body, NULL);
            JoltC_Body_AddForce(body, NULL);
            JoltC_Body_AddForceAtPosition(body, NULL, NULL);
            JoltC_Body_AddTorque(body, NULL);
            JoltC_Body_AddImpulse(body, NULL);
            JoltC_Body_AddImpulseAtPosition(body, NULL, NULL);
            JoltC_Body_AddAngularImpulse(body, NULL);
            JoltC_Body_MoveKinematic(body, NULL, NULL, 1.0f / 60.0f);
            TEST_ASSERT(JoltC_Body_ApplyBuoyancyImpulse(body, NULL, NULL, 1.0f, 0.0f, 0.0f,
                                                        NULL, NULL, 1.0f / 60.0f) == 0,
                        "ApplyBuoyancyImpulse with NULL vectors reports no impulse");
            TEST_ASSERT(1, "no NULL out-param call crashed");
        }

        JoltC_BodyLockWrite_Destroy(lock);
        remove_and_destroy(&ctx, id);
        teardown_physics_context(&ctx);
    }
    TEST_END();
}

static void test_motion_properties_null_safety(void)
{
    TEST_BEGIN("MotionProperties functions tolerate NULL");
    {
        TEST_ASSERT(JoltC_MotionProperties_GetAllowedDOFs(NULL) == JOLTC_ALLOWED_DOFS_ALL,
                    "GetAllowedDOFs(NULL) is ALL");
        TEST_ASSERT_FLOAT_EQ(JoltC_MotionProperties_GetLinearDamping(NULL), 0.0f, 0.0001f,
                             "GetLinearDamping(NULL) is 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_MotionProperties_GetAngularDamping(NULL), 0.0f, 0.0001f,
                             "GetAngularDamping(NULL) is 0");
        TEST_ASSERT_FLOAT_EQ(JoltC_MotionProperties_GetInverseMassUnchecked(NULL), 0.0f, 0.0001f,
                             "GetInverseMassUnchecked(NULL) is 0");

        JoltC_Vec3 sentinelVec = { -99.0f, -99.0f, -99.0f };
        JoltC_MotionProperties_GetInverseInertiaDiagonal(NULL, &sentinelVec);
        TEST_ASSERT_FLOAT_EQ(sentinelVec.x, -99.0f, 0.0001f,
                             "inverse inertia out-param untouched for NULL properties");

        JoltC_Quat sentinelQuat = { -99.0f, -99.0f, -99.0f, -99.0f };
        JoltC_MotionProperties_GetInertiaRotation(NULL, &sentinelQuat);
        TEST_ASSERT_FLOAT_EQ(sentinelQuat.w, -99.0f, 0.0001f,
                             "inertia rotation out-param untouched for NULL properties");

        JoltC_Vec3 diagonal = { 1.0f, 1.0f, 1.0f };
        JoltC_Quat rotation = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_MassProperties massProperties = {
            1.0f,
            { { 1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f } }
        };
        JoltC_MotionProperties_SetLinearDamping(NULL, 0.5f);
        JoltC_MotionProperties_SetAngularDamping(NULL, 0.5f);
        JoltC_MotionProperties_SetInverseMass(NULL, 0.5f);
        JoltC_MotionProperties_SetInverseInertia(NULL, &diagonal, &rotation);
        JoltC_MotionProperties_SetMassProperties(NULL, JOLTC_ALLOWED_DOFS_ALL, &massProperties);
        JoltC_MotionProperties_ScaleToMass(NULL, 2.0f);
        TEST_ASSERT(1, "no NULL MotionProperties call crashed");
    }
    TEST_END();
}

static void test_lock_null_safety(void)
{
    TEST_BEGIN("BodyLock functions tolerate NULL");
    {
        TEST_ASSERT(JoltC_PhysicsSystem_GetBodyLockInterface(NULL) == NULL,
                    "GetBodyLockInterface(NULL) is NULL");
        TEST_ASSERT(JoltC_PhysicsSystem_GetBodyLockInterfaceNoLock(NULL) == NULL,
                    "GetBodyLockInterfaceNoLock(NULL) is NULL");
        TEST_ASSERT(JoltC_BodyLockRead_Create(NULL, 0) == NULL,
                    "BodyLockRead_Create(NULL) is NULL");
        TEST_ASSERT(JoltC_BodyLockWrite_Create(NULL, 0) == NULL,
                    "BodyLockWrite_Create(NULL) is NULL");
        TEST_ASSERT(JoltC_BodyLockRead_GetBody(NULL) == NULL,
                    "BodyLockRead_GetBody(NULL) is NULL");
        TEST_ASSERT(JoltC_BodyLockWrite_GetBody(NULL) == NULL,
                    "BodyLockWrite_GetBody(NULL) is NULL");
        TEST_ASSERT(JoltC_BodyLockInterface_LockRead(NULL, 0) == NULL,
                    "LockRead(NULL) is NULL");
        TEST_ASSERT(JoltC_BodyLockInterface_LockWrite(NULL, 0) == NULL,
                    "LockWrite(NULL) is NULL");
        TEST_ASSERT(JoltC_BodyLockMultiWrite_GetBody(NULL, 0) == NULL,
                    "BodyLockMultiWrite_GetBody(NULL) is NULL");

        JoltC_BodyLockRead_Destroy(NULL);
        JoltC_BodyLockWrite_Destroy(NULL);
        JoltC_BodyLockInterface_UnlockRead(NULL, NULL);
        JoltC_BodyLockInterface_UnlockWrite(NULL, NULL);
        JoltC_BodyLockMultiRead_Destroy(NULL);
        JoltC_BodyLockMultiWrite_Destroy(NULL);

        /* The multi locks also reject an empty or absent id array. */
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);
        const JoltC_BodyLockInterface* iface = body_lock_iface(&ctx);
        JoltC_BodyID ids[2] = { 0, 1 };

        TEST_ASSERT(JoltC_BodyLockInterface_LockMultiRead(NULL, ids, 2) == NULL,
                    "LockMultiRead without an interface is NULL");
        TEST_ASSERT(JoltC_BodyLockInterface_LockMultiRead(iface, NULL, 2) == NULL,
                    "LockMultiRead without ids is NULL");
        TEST_ASSERT(JoltC_BodyLockInterface_LockMultiRead(iface, ids, 0) == NULL,
                    "LockMultiRead of zero bodies is NULL");
        TEST_ASSERT(JoltC_BodyLockInterface_LockMultiWrite(NULL, ids, 2) == NULL,
                    "LockMultiWrite without an interface is NULL");
        TEST_ASSERT(JoltC_BodyLockInterface_LockMultiWrite(iface, NULL, 2) == NULL,
                    "LockMultiWrite without ids is NULL");
        TEST_ASSERT(JoltC_BodyLockInterface_LockMultiWrite(iface, ids, 0) == NULL,
                    "LockMultiWrite of zero bodies is NULL");

        teardown_physics_context(&ctx);
    }
    TEST_END();
}

/* ========================================================================== */
/*  Suite entry point                                                         */
/* ========================================================================== */

void run_body_access_tests(void)
{
    printf("\n[SUITE] BodyAccess\n");

    test_lock_interface_available();
    test_lock_read_and_write();
    test_lock_invalid_body_id();
    test_lock_multi_preserves_order();

    test_body_classification_dynamic();
    test_body_classification_static();

    test_body_flag_roundtrips();
    test_body_pairwise_flags();
    test_body_motion_type_roundtrip();
    test_body_scalar_roundtrips();

    test_body_velocity_roundtrips();
    test_body_point_velocity();

    test_body_force_and_torque();
    test_body_impulses();
    test_body_move_kinematic();
    test_body_buoyancy_impulse();

    test_body_transforms();

    test_motion_properties_damping_and_mass();
    test_motion_properties_inertia();

    test_body_null_safety();
    test_body_null_out_params();
    test_motion_properties_null_safety();
    test_lock_null_safety();
}
