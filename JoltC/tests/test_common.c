/* JoltC Test Suite -- common.h API tests (init, error handling, trace)
 * SPDX-License-Identifier: MIT
 */

#include "test_common.h"

static void trace_handler(const char* message) { (void)message; }
static int assert_handler(const char* expr, const char* msg, const char* file, unsigned int line)
{
    (void)expr; (void)msg; (void)file; (void)line;
    return 0; /* 0 = don't break */
}

void run_common_tests(void)
{
    /* test_get_last_error_initially_null */
    TEST_BEGIN("GetLastError initially null or empty");
    {
        const char* err = JoltC_GetLastError();
        TEST_ASSERT(err == NULL || err[0] == '\0', "Expected no error after init");
    }
    TEST_END();

    /* test_clear_last_error */
    TEST_BEGIN("ClearLastError does not crash");
    {
        JoltC_ClearLastError();
        const char* err = JoltC_GetLastError();
        TEST_ASSERT(err == NULL || err[0] == '\0', "Expected no error after clear");
    }
    TEST_END();

    /* test_set_trace_handler */
    TEST_BEGIN("SetTraceHandler accepts callback");
    {
        JoltC_SetTraceHandler(trace_handler);
        JoltC_SetTraceHandler(NULL);           /* reset */
        TEST_ASSERT(1, "No crash");
    }
    TEST_END();

    /* test_set_assert_handler */
    TEST_BEGIN("SetAssertFailureHandler accepts callback");
    {
        JoltC_SetAssertFailureHandler(assert_handler);
        JoltC_SetAssertFailureHandler(NULL);   /* reset */
        TEST_ASSERT(1, "No crash");
    }
    TEST_END();
}
