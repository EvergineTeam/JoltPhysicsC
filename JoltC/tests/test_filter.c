/* JoltC Test Suite — filters.h API tests
 * SPDX-License-Identifier: MIT
 */

#include "test_common.h"

static JoltC_Bool test_ol_filter_fn(void* ud, JoltC_ObjectLayer layer)
{
    (void)ud;
    return layer == OBJ_LAYER_STATIC;
}

void run_filter_tests(void)
{
    /* test_object_layer_pair_filter_table */
    TEST_BEGIN("ObjectLayerPairFilterTable enable/disable/ShouldCollide");
    {
        JoltC_ObjectLayerPairFilter* f = JoltC_ObjectLayerPairFilterTable_Create(2);
        TEST_ASSERT_NOT_NULL(f, "Filter not null");

        /* Initially no collision enabled */
        TEST_ASSERT(!JoltC_ObjectLayerPairFilterTable_ShouldCollide(f, 0, 0), "0-0 off by default");

        JoltC_ObjectLayerPairFilterTable_EnableCollision(f, 0, 1);
        TEST_ASSERT(JoltC_ObjectLayerPairFilterTable_ShouldCollide(f, 0, 1), "0-1 enabled");

        JoltC_ObjectLayerPairFilterTable_DisableCollision(f, 0, 1);
        TEST_ASSERT(!JoltC_ObjectLayerPairFilterTable_ShouldCollide(f, 0, 1), "0-1 disabled");

        JoltC_ObjectLayerPairFilter_Destroy(f);
    }
    TEST_END();

    /* test_broadphase_layer_interface_table */
    TEST_BEGIN("BroadPhaseLayerInterfaceTable create and map");
    {
        JoltC_BroadPhaseLayerInterface* bpi = JoltC_BroadPhaseLayerInterfaceTable_Create(2, 2);
        TEST_ASSERT_NOT_NULL(bpi, "BPI not null");
        JoltC_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(bpi, 0, 0);
        JoltC_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(bpi, 1, 1);
        TEST_ASSERT(1, "Mapping no crash");
        JoltC_BroadPhaseLayerInterface_Destroy(bpi);
    }
    TEST_END();

    /* test_object_vs_broadphase_filter_table */
    TEST_BEGIN("ObjectVsBroadPhaseLayerFilterTable create");
    {
        JoltC_ObjectLayerPairFilter* olpf = JoltC_ObjectLayerPairFilterTable_Create(2);
        JoltC_ObjectLayerPairFilterTable_EnableCollision(olpf, 0, 1);
        JoltC_BroadPhaseLayerInterface* bpi = JoltC_BroadPhaseLayerInterfaceTable_Create(2, 2);
        JoltC_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(bpi, 0, 0);
        JoltC_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(bpi, 1, 1);

        JoltC_ObjectVsBroadPhaseLayerFilter* f = JoltC_ObjectVsBroadPhaseLayerFilterTable_Create(bpi, 2, olpf, 2);
        TEST_ASSERT_NOT_NULL(f, "OVBPLF not null");

        JoltC_ObjectVsBroadPhaseLayerFilter_Destroy(f);
        JoltC_BroadPhaseLayerInterface_Destroy(bpi);
        JoltC_ObjectLayerPairFilter_Destroy(olpf);
    }
    TEST_END();

    /* test_group_filter_table */
    TEST_BEGIN("GroupFilterTable enable/disable/IsCollisionEnabled");
    {
        JoltC_GroupFilter* gf = JoltC_GroupFilterTable_Create(2);
        TEST_ASSERT_NOT_NULL(gf, "GroupFilterTable not null");

        JoltC_GroupFilterTable_EnableCollision(gf, 0, 1);
        TEST_ASSERT(JoltC_GroupFilterTable_IsCollisionEnabled(gf, 0, 1), "0-1 enabled");

        JoltC_GroupFilterTable_DisableCollision(gf, 0, 1);
        TEST_ASSERT(!JoltC_GroupFilterTable_IsCollisionEnabled(gf, 0, 1), "0-1 disabled");

        JoltC_GroupFilter_Destroy(gf);
    }
    TEST_END();

    /* test_object_layer_pair_filter_mask */
    TEST_BEGIN("ObjectLayerPairFilterMask create, GetObjectLayer, GetGroup, GetMask");
    {
        JoltC_ObjectLayerPairFilter* f = JoltC_ObjectLayerPairFilterMask_Create();
        TEST_ASSERT_NOT_NULL(f, "Mask filter not null");

        JoltC_ObjectLayer ol = JoltC_ObjectLayerPairFilterMask_GetObjectLayer(1, 0xFF);
        uint32_t group = JoltC_ObjectLayerPairFilterMask_GetGroup(ol);
        uint32_t mask = JoltC_ObjectLayerPairFilterMask_GetMask(ol);
        TEST_ASSERT(group == 1, "group == 1");
        TEST_ASSERT(mask == 0xFF, "mask == 0xFF");

        JoltC_ObjectLayerPairFilter_Destroy(f);
    }
    TEST_END();

    /* test_broadphase_layer_interface_mask */
    TEST_BEGIN("BroadPhaseLayerInterfaceMask create and configure");
    {
        JoltC_BroadPhaseLayerInterface* bpi = JoltC_BroadPhaseLayerInterfaceMask_Create(2);
        TEST_ASSERT_NOT_NULL(bpi, "BPI mask not null");
        JoltC_BroadPhaseLayerInterfaceMask_ConfigureLayer(bpi, 0, 1, 0);
        JoltC_BroadPhaseLayerInterfaceMask_ConfigureLayer(bpi, 1, 2, 0);
        TEST_ASSERT(1, "ConfigureLayer no crash");
        JoltC_BroadPhaseLayerInterface_Destroy(bpi);
    }
    TEST_END();

    /* test_contact_listener */
    TEST_BEGIN("ContactListener create/destroy");
    {
        JoltC_ContactListener* l = JoltC_ContactListener_Create(NULL, NULL, NULL, NULL, NULL);
        TEST_ASSERT_NOT_NULL(l, "ContactListener not null");
        JoltC_ContactListener_Destroy(l);
    }
    TEST_END();

    /* test_body_activation_listener */
    TEST_BEGIN("BodyActivationListener create/destroy");
    {
        JoltC_BodyActivationListener* l = JoltC_BodyActivationListener_Create(NULL, NULL, NULL);
        TEST_ASSERT_NOT_NULL(l, "BodyActivationListener not null");
        JoltC_BodyActivationListener_Destroy(l);
    }
    TEST_END();

    /* test_object_layer_filter_callback */
    TEST_BEGIN("ObjectLayerFilter create with callback");
    {
        JoltC_ObjectLayerFilter* f = JoltC_ObjectLayerFilter_Create(test_ol_filter_fn, NULL);
        TEST_ASSERT_NOT_NULL(f, "ObjectLayerFilter not null");
        JoltC_ObjectLayerFilter_Destroy(f);
    }
    TEST_END();
}
