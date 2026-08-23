/* JoltC Test Suite -- the phase 0 repairs: enum values, real constraint settings, an articulated
 * ragdoll, filtered characters, soft body contact callbacks, and query settings that connect.
 * SPDX-License-Identifier: MIT
 *
 * Every case here covers something that used to exist and not work: shape types that reported the
 * wrong label, GetSettings stubs that made ragdolls impossible to articulate from C, a character
 * that collided with everything because its filters were never passed through, a soft body contact
 * listener that had no registration, and two query settings structs with an Init helper and no
 * function that accepted them.
 */

#include "test_common.h"

static int s_softValidateCalls;
static int s_softAddedCalls;
static int s_softContactVertices;

static JoltC_SoftBodyValidateResult soft_validate(void* userData, const JoltC_Body* softBody,
                                                  const JoltC_Body* otherBody,
                                                  JoltC_SoftBodyContactSettings* ioSettings)
{
    (void)userData; (void)softBody; (void)otherBody; (void)ioSettings;
    s_softValidateCalls++;
    return JOLTC_SOFT_BODY_VALIDATE_RESULT_ACCEPT_CONTACT;
}

static void soft_added(void* userData, const JoltC_Body* softBody, const JoltC_SoftBodyManifold* manifold)
{
    (void)userData; (void)softBody;
    s_softAddedCalls++;

    uint32_t count = JoltC_SoftBodyManifold_GetVertexCount(manifold);
    for (uint32_t i = 0; i < count; i++)
    {
        if (JoltC_SoftBodyManifold_HasContact(manifold, i))
        {
            s_softContactVertices++;
            JoltC_Vec3 point, normal;
            JoltC_SoftBodyManifold_GetLocalContactPoint(manifold, i, &point);
            JoltC_SoftBodyManifold_GetContactNormal(manifold, i, &normal);
        }
    }
}

static int s_collideHits;

static void collide_hit(void* userData, const JoltC_CollideShapeResult* result)
{
    (void)userData; (void)result;
    s_collideHits++;
}

void run_phase0_tests(void);

void run_phase0_tests(void)
{
    /* test_shape_type_labels_match_jolt */
    TEST_BEGIN("Plane and empty shapes report their own type");
    {
        const JoltC_Shape* plane = JoltC_PlaneShape_Create((JoltC_Vec3){ 0.0f, 1.0f, 0.0f }, 0.0f, 100.0f);
        TEST_ASSERT_NOT_NULL(plane, "plane shape created");
        TEST_ASSERT(JoltC_Shape_GetType(plane) == JOLTC_SHAPE_TYPE_PLANE,
                    "plane type is PLANE, not its neighbour");
        TEST_ASSERT(JoltC_Shape_GetSubType(plane) == JOLTC_SHAPE_SUB_TYPE_PLANE,
                    "plane sub type carries Jolt's value");
        JoltC_Shape_Release(plane);

        const JoltC_Shape* empty = JoltC_EmptyShape_Create((JoltC_Vec3){ 0.0f, 0.0f, 0.0f });
        TEST_ASSERT_NOT_NULL(empty, "empty shape created");
        TEST_ASSERT(JoltC_Shape_GetType(empty) == JOLTC_SHAPE_TYPE_EMPTY, "empty type is EMPTY");
        TEST_ASSERT(JoltC_Shape_GetSubType(empty) == JOLTC_SHAPE_SUB_TYPE_EMPTY, "empty sub type is 33");
        JoltC_Shape_Release(empty);

        const JoltC_Shape* tapered = JoltC_TaperedCylinderShapeSettings_CreateShape(0.5f, 0.2f, 0.4f, 0.05f);
        TEST_ASSERT_NOT_NULL(tapered, "tapered cylinder created");
        TEST_ASSERT(JoltC_Shape_GetSubType(tapered) == JOLTC_SHAPE_SUB_TYPE_TAPERED_CYLINDER,
                    "tapered cylinder sub type is 32");
        JoltC_Shape_Release(tapered);
    }
    TEST_END();

    /* test_constraint_get_settings_is_real */
    TEST_BEGIN("A live constraint hands its settings back");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_RVec3 posA = { 0.0f, 5.0f, 0.0f };
        JoltC_RVec3 posB = { 0.0f, 4.0f, 0.0f };
        JoltC_BodyID a = create_test_box_body(&ctx, posA, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);
        JoltC_BodyID b = create_test_box_body(&ctx, posB, JOLTC_MOTION_TYPE_DYNAMIC, JOLTC_ACTIVATION_ACTIVATE);

        JoltC_HingeConstraintSettings hinge;
        JoltC_HingeConstraintSettings_Init(&hinge);
        hinge.point1 = (JoltC_RVec3){ 0.0f, 4.5f, 0.0f };
        hinge.point2 = (JoltC_RVec3){ 0.0f, 4.5f, 0.0f };
        JoltC_Constraint* constraint = JoltC_HingeConstraint_Create(ctx.physicsSystem, a, b, &hinge);
        TEST_ASSERT_NOT_NULL(constraint, "hinge created");

        JoltC_TwoBodyConstraintSettings* settings = JoltC_HingeConstraint_GetSettings(constraint);
        TEST_ASSERT_NOT_NULL(settings, "GetSettings returns a real object, not the old NULL stub");
        JoltC_TwoBodyConstraintSettings_Release(settings);

        JoltC_Constraint_Destroy(constraint);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_ragdoll_articulates_from_c */
    TEST_BEGIN("A three bone ragdoll articulates and holds together");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        /* A floor to land on. */
        JoltC_Vec3 floorHalf = { 20.0f, 1.0f, 20.0f };
        const JoltC_Shape* floorShape = JoltC_BoxShape_Create(floorHalf, 0.05f);
        JoltC_RVec3 floorPos = { 0.0f, -1.0f, 0.0f };
        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_BodyCreationSettings* floorSettings = JoltC_BodyCreationSettings_Create3(
            floorShape, floorPos, identity, JOLTC_MOTION_TYPE_STATIC, OBJ_LAYER_STATIC);
        JoltC_BodyInterface_CreateAndAddBody(ctx.bodyInterface, floorSettings, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_BodyCreationSettings_Destroy(floorSettings);
        JoltC_Shape_Release(floorShape);

        /* Three bones in a chain: pelvis, torso, head. */
        JoltC_Skeleton* skeleton = JoltC_Skeleton_Create();
        uint32_t pelvis = JoltC_Skeleton_AddJoint(skeleton, "pelvis");
        uint32_t torso = JoltC_Skeleton_AddJoint2(skeleton, "torso", (int)pelvis);
        uint32_t head = JoltC_Skeleton_AddJoint2(skeleton, "head", (int)torso);
        (void)head;
        TEST_ASSERT(JoltC_Skeleton_AreJointsCorrectlyOrdered(skeleton), "joints ordered parent first");

        JoltC_RagdollSettings* ragdollSettings = JoltC_RagdollSettings_Create();
        JoltC_RagdollSettings_SetSkeleton(ragdollSettings, skeleton);
        JoltC_RagdollSettings_ResizeParts(ragdollSettings, 3);

        const JoltC_Shape* bone = JoltC_CapsuleShape_Create(0.15f, 0.1f);
        float heights[3] = { 3.0f, 3.5f, 4.0f };

        for (int i = 0; i < 3; i++)
        {
            JoltC_RagdollSettings_SetPartShape(ragdollSettings, i, bone);
            JoltC_RagdollSettings_SetPartPosition(ragdollSettings, i, (JoltC_RVec3){ 0.0f, heights[i], 0.0f });
            JoltC_RagdollSettings_SetPartRotation(ragdollSettings, i, identity);
            JoltC_RagdollSettings_SetPartMotionType(ragdollSettings, i, JOLTC_MOTION_TYPE_DYNAMIC);
            JoltC_RagdollSettings_SetPartObjectLayer(ragdollSettings, i, OBJ_LAYER_DYNAMIC);
        }

        /* The articulation that the NULL returning stubs made impossible: settings produced
         * detached from any body pair and handed to the parts. */
        for (int i = 1; i < 3; i++)
        {
            JoltC_SwingTwistConstraintSettings joint;
            JoltC_SwingTwistConstraintSettings_Init(&joint);
            float jointHeight = (heights[i - 1] + heights[i]) * 0.5f;
            joint.position1 = (JoltC_RVec3){ 0.0f, jointHeight, 0.0f };
            joint.position2 = (JoltC_RVec3){ 0.0f, jointHeight, 0.0f };
            joint.twistAxis1 = (JoltC_Vec3){ 0.0f, 1.0f, 0.0f };
            joint.twistAxis2 = (JoltC_Vec3){ 0.0f, 1.0f, 0.0f };
            joint.planeAxis1 = (JoltC_Vec3){ 1.0f, 0.0f, 0.0f };
            joint.planeAxis2 = (JoltC_Vec3){ 1.0f, 0.0f, 0.0f };
            joint.normalHalfConeAngle = 0.3f;
            joint.planeHalfConeAngle = 0.3f;
            joint.twistMinAngle = -0.2f;
            joint.twistMaxAngle = 0.2f;

            JoltC_TwoBodyConstraintSettings* settings = JoltC_SwingTwistConstraintSettings_CreateSettings(&joint);
            TEST_ASSERT_NOT_NULL(settings, "constraint settings produced without bodies");
            JoltC_RagdollSettings_SetPartToParent(ragdollSettings, i, i - 1, settings);
            JoltC_TwoBodyConstraintSettings_Release(settings);
        }

        JoltC_RagdollSettings_Stabilize(ragdollSettings);
        JoltC_Ragdoll* ragdoll = JoltC_RagdollSettings_CreateRagdoll(ragdollSettings, ctx.physicsSystem, 0, 0);
        TEST_ASSERT_NOT_NULL(ragdoll, "ragdoll created");
        TEST_ASSERT(JoltC_Ragdoll_GetBodyCount(ragdoll) == 3, "three bodies");
        TEST_ASSERT(JoltC_Ragdoll_GetConstraintCount(ragdoll) == 2, "two joints hold three bones");

        JoltC_Ragdoll_AddToPhysicsSystem(ragdoll, JOLTC_ACTIVATION_ACTIVATE, JOLTC_TRUE);

        for (int i = 0; i < 180; i++)
        {
            JoltC_PhysicsSystem_Update(ctx.physicsSystem, 1.0f / 60.0f, 1, ctx.tempAllocator, ctx.jobSystem);
        }

        /* Fell, landed, and stayed in one piece: every neighbouring pair within joint reach. */
        JoltC_RVec3 positions[3];
        for (int i = 0; i < 3; i++)
        {
            positions[i] = JoltC_BodyInterface_GetPosition(ctx.bodyInterface, JoltC_Ragdoll_GetBodyID(ragdoll, i));
            TEST_ASSERT(positions[i].y < 3.0f, "bone fell from its spawn");
            TEST_ASSERT(positions[i].y > -0.5f, "bone did not tunnel the floor");
        }

        for (int i = 1; i < 3; i++)
        {
            float dx = (float)(positions[i].x - positions[i - 1].x);
            float dy = (float)(positions[i].y - positions[i - 1].y);
            float dz = (float)(positions[i].z - positions[i - 1].z);
            float distance = (float)sqrt((double)((dx * dx) + (dy * dy) + (dz * dz)));
            TEST_ASSERT(distance < 1.0f, "joint held its bones together");
        }

        JoltC_Ragdoll_RemoveFromPhysicsSystem(ragdoll, JOLTC_TRUE);
        JoltC_Ragdoll_Destroy(ragdoll);
        JoltC_RagdollSettings_Destroy(ragdollSettings);
        JoltC_Shape_Release(bone);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_character_filters_are_obeyed */
    TEST_BEGIN("A filtered character falls through the layer it excludes");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        JoltC_Vec3 floorHalf = { 20.0f, 0.5f, 20.0f };
        const JoltC_Shape* floorShape = JoltC_BoxShape_Create(floorHalf, 0.05f);
        JoltC_RVec3 floorPos = { 0.0f, -0.5f, 0.0f };
        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_BodyCreationSettings* floorSettings = JoltC_BodyCreationSettings_Create3(
            floorShape, floorPos, identity, JOLTC_MOTION_TYPE_STATIC, OBJ_LAYER_STATIC);
        JoltC_BodyInterface_CreateAndAddBody(ctx.bodyInterface, floorSettings, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_BodyCreationSettings_Destroy(floorSettings);

        const JoltC_Shape* capsule = JoltC_CapsuleShape_Create(0.4f, 0.3f);

        JoltC_CharacterVirtualSettings settings;
        JoltC_CharacterVirtualSettings_Init(&settings);
        settings.shape = capsule;

        /* Two characters, same spot, same world: one simulates unfiltered and lands on the floor,
         * the other excludes the static layer and must fall straight through it. */
        JoltC_RVec3 spawn = { 0.0f, 2.0f, 0.0f };
        JoltC_CharacterVirtual* unfiltered = JoltC_CharacterVirtual_Create(&settings, spawn, identity, 0, ctx.physicsSystem);
        JoltC_CharacterVirtual* filtered = JoltC_CharacterVirtual_Create(&settings, spawn, identity, 0, ctx.physicsSystem);
        TEST_ASSERT_NOT_NULL(unfiltered, "unfiltered character created");
        TEST_ASSERT_NOT_NULL(filtered, "filtered character created");

        /* The system's own layer logic for the dynamic layer, which collides with statics. */
        JoltC_ObjectLayerFilter* dynamicSees = JoltC_PhysicsSystem_GetDefaultLayerFilter(ctx.physicsSystem, OBJ_LAYER_DYNAMIC);
        TEST_ASSERT_NOT_NULL(dynamicSees, "default layer filter obtained from the system");

        /* And a filter that refuses the static layer outright. */
        JoltC_ObjectLayerFilter* refuseStatics = JoltC_PhysicsSystem_GetDefaultLayerFilter(ctx.physicsSystem, OBJ_LAYER_STATIC);
        TEST_ASSERT_NOT_NULL(refuseStatics, "second default filter obtained");

        JoltC_Vec3 gravity = { 0.0f, -9.81f, 0.0f };

        /* The virtual character does not integrate gravity on its own -- the caller owns the
         * velocity -- so both get pushed downward explicitly. The update stops that motion at
         * whatever the filters let them stand on. */
        JoltC_Vec3 fall = { 0.0f, -5.0f, 0.0f };

        for (int i = 0; i < 120; i++)
        {
            JoltC_CharacterVirtual_SetLinearVelocity(unfiltered, fall);
            JoltC_CharacterVirtual_Update_WithFilters(unfiltered, 1.0f / 60.0f, gravity,
                NULL, dynamicSees, NULL, NULL, ctx.tempAllocator);

            /* The static layer's own filter says statics collide with moving things and not with
             * one another -- so viewed from a character it refuses the floor. */
            JoltC_CharacterVirtual_SetLinearVelocity(filtered, fall);
            JoltC_CharacterVirtual_Update_WithFilters(filtered, 1.0f / 60.0f, gravity,
                NULL, refuseStatics, NULL, NULL, ctx.tempAllocator);

            JoltC_PhysicsSystem_Update(ctx.physicsSystem, 1.0f / 60.0f, 1, ctx.tempAllocator, ctx.jobSystem);
        }

        JoltC_RVec3 unfilteredPos = JoltC_CharacterVirtual_GetPosition(unfiltered);
        JoltC_RVec3 filteredPos = JoltC_CharacterVirtual_GetPosition(filtered);

        TEST_ASSERT(unfilteredPos.y > -0.5f, "unfiltered character stands on the floor");
        TEST_ASSERT(filteredPos.y < -3.0f, "filtered character fell straight through it");

        JoltC_ObjectLayerFilter_Destroy(dynamicSees);
        JoltC_ObjectLayerFilter_Destroy(refuseStatics);
        JoltC_CharacterVirtual_Destroy(unfiltered);
        JoltC_CharacterVirtual_Destroy(filtered);
        JoltC_Shape_Release(capsule);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_soft_body_contact_listener_fires */
    TEST_BEGIN("Soft body contacts reach the listener with a readable manifold");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        s_softValidateCalls = 0;
        s_softAddedCalls = 0;
        s_softContactVertices = 0;

        JoltC_SoftBodyContactListener* listener = JoltC_SoftBodyContactListener_Create(soft_validate, soft_added, NULL);
        TEST_ASSERT_NOT_NULL(listener, "listener created");
        JoltC_PhysicsSystem_SetSoftBodyContactListener(ctx.physicsSystem, listener);

        JoltC_Vec3 floorHalf = { 20.0f, 1.0f, 20.0f };
        const JoltC_Shape* floorShape = JoltC_BoxShape_Create(floorHalf, 0.05f);
        JoltC_RVec3 floorPos = { 0.0f, -1.0f, 0.0f };
        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_BodyCreationSettings* floorSettings = JoltC_BodyCreationSettings_Create3(
            floorShape, floorPos, identity, JOLTC_MOTION_TYPE_STATIC, OBJ_LAYER_STATIC);
        JoltC_BodyInterface_CreateAndAddBody(ctx.bodyInterface, floorSettings, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_BodyCreationSettings_Destroy(floorSettings);
        JoltC_Shape_Release(floorShape);

        JoltC_SoftBodySharedSettings* cube = JoltC_SoftBodySharedSettings_CreateCube(4, 0.25f);
        JoltC_SoftBodyCreationSettings* creation = JoltC_SoftBodyCreationSettings_Create();
        JoltC_SoftBodyCreationSettings_SetSettings(creation, cube);
        JoltC_SoftBodyCreationSettings_SetPosition(creation, (JoltC_RVec3){ 0.0f, 2.0f, 0.0f });
        JoltC_SoftBodyCreationSettings_SetObjectLayer(creation, OBJ_LAYER_DYNAMIC);
        JoltC_Body* body = JoltC_BodyInterface_CreateSoftBody(ctx.bodyInterface, creation);
        JoltC_SoftBodyCreationSettings_Destroy(creation);
        JoltC_BodyID bodyId = JoltC_Body_GetID(body);
        JoltC_BodyInterface_AddBody(ctx.bodyInterface, bodyId, JOLTC_ACTIVATION_ACTIVATE);

        for (int i = 0; i < 120; i++)
        {
            JoltC_PhysicsSystem_Update(ctx.physicsSystem, 1.0f / 60.0f, 1, ctx.tempAllocator, ctx.jobSystem);
        }

        TEST_ASSERT(s_softValidateCalls > 0, "validate callback fired");
        TEST_ASSERT(s_softAddedCalls > 0, "contact added callback fired");
        TEST_ASSERT(s_softContactVertices > 0, "manifold reported touching vertices");

        JoltC_PhysicsSystem_SetSoftBodyContactListener(ctx.physicsSystem, NULL);
        JoltC_SoftBodyContactListener_Destroy(listener);
        JoltC_BodyInterface_RemoveBody(ctx.bodyInterface, bodyId);
        JoltC_BodyInterface_DestroyBody(ctx.bodyInterface, bodyId);
        JoltC_SoftBodySharedSettings_Release(cube);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_collide_shape_settings_connect */
    TEST_BEGIN("CollideShape2 honours max separation distance");
    {
        TestPhysicsContext ctx;
        setup_physics_context(&ctx);

        /* One sphere body at the origin, and a query sphere half a metre away: apart, but within
         * the separation distance the settings ask for. */
        const JoltC_Shape* bodySphere = JoltC_SphereShape_Create(0.5f);
        JoltC_RVec3 bodyPos = { 0.0f, 0.0f, 0.0f };
        JoltC_Quat identity = { 0.0f, 0.0f, 0.0f, 1.0f };
        JoltC_BodyCreationSettings* bodySettings = JoltC_BodyCreationSettings_Create3(
            bodySphere, bodyPos, identity, JOLTC_MOTION_TYPE_STATIC, OBJ_LAYER_STATIC);
        JoltC_BodyInterface_CreateAndAddBody(ctx.bodyInterface, bodySettings, JOLTC_ACTIVATION_DONT_ACTIVATE);
        JoltC_BodyCreationSettings_Destroy(bodySettings);

        const JoltC_NarrowPhaseQuery* query = JoltC_PhysicsSystem_GetNarrowPhaseQuery(ctx.physicsSystem);
        const JoltC_Shape* probe = JoltC_SphereShape_Create(0.5f);

        JoltC_Mat44 transform;
        JoltC_Mat44_Identity(&transform);
        transform.m[12] = 1.5f; /* half a metre of clearance between the two surfaces */

        JoltC_Vec3 unitScale = { 1.0f, 1.0f, 1.0f };
        JoltC_RVec3 origin = { 0.0f, 0.0f, 0.0f };

        s_collideHits = 0;
        JoltC_NarrowPhaseQuery_CollideShape2(query, probe, unitScale, transform, NULL, origin,
                                             collide_hit, NULL, NULL, NULL, NULL, NULL);
        TEST_ASSERT(s_collideHits == 0, "no hit with default settings: the spheres are apart");

        JoltC_CollideShapeSettings settings;
        JoltC_CollideShapeSettings_Init(&settings);
        settings.maxSeparationDistance = 1.0f;

        s_collideHits = 0;
        JoltC_NarrowPhaseQuery_CollideShape2(query, probe, unitScale, transform, &settings, origin,
                                             collide_hit, NULL, NULL, NULL, NULL, NULL);
        TEST_ASSERT(s_collideHits == 1, "one hit once the separation distance covers the gap");

        JoltC_Shape_Release(probe);
        JoltC_Shape_Release(bodySphere);
        teardown_physics_context(&ctx);
    }
    TEST_END();

    /* test_broad_phase_query_per_system */
    TEST_BEGIN("Two systems keep their own broad phase query handles");
    {
        TestPhysicsContext a, b;
        setup_physics_context(&a);
        setup_physics_context(&b);

        const JoltC_BroadPhaseQuery* qa = JoltC_PhysicsSystem_GetBroadPhaseQuery(a.physicsSystem);
        const JoltC_BroadPhaseQuery* qb = JoltC_PhysicsSystem_GetBroadPhaseQuery(b.physicsSystem);
        TEST_ASSERT_NOT_NULL(qa, "first system's query");
        TEST_ASSERT_NOT_NULL(qb, "second system's query");

        /* The old thread_local made these the same pointer, so the second Get invalidated the
         * first. Per system storage keeps both alive at once. */
        TEST_ASSERT(qa != qb, "handles are distinct");

        teardown_physics_context(&b);
        teardown_physics_context(&a);
    }
    TEST_END();
}
