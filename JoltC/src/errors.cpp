/* JoltC - Internal error handling implementation
 * SPDX-License-Identifier: MIT
 */

#include "errors_internal.h"
#include <JoltC/common.h>
#include <string>

static thread_local std::string g_lastError;

void joltc_set_last_error(const char* msg)
{
    if (msg)
        g_lastError = msg;
    else
        g_lastError.clear();
}

void joltc_clear_last_error()
{
    g_lastError.clear();
}

const char* joltc_get_last_error()
{
    return g_lastError.empty() ? nullptr : g_lastError.c_str();
}

/* Public API implementations */
extern "C" {

JOLTC_API const char* JoltC_GetLastError(void)
{
    return joltc_get_last_error();
}

JOLTC_API void JoltC_ClearLastError(void)
{
    joltc_clear_last_error();
}

} /* extern "C" */
