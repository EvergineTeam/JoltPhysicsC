/* JoltC - Internal error handling
 * SPDX-License-Identifier: MIT
 */

#ifndef JOLTC_ERRORS_INTERNAL_H
#define JOLTC_ERRORS_INTERNAL_H

#include <string>

/* Thread-local error state */
void        joltc_set_last_error(const char* msg);
void        joltc_clear_last_error();
const char* joltc_get_last_error();

/* Exception-catching macros for every extern "C" function body */
#define JOLTC_TRY_BEGIN try {
#define JOLTC_TRY_END                                               \
    } catch (const std::exception& e) { joltc_set_last_error(e.what()); } \
      catch (...) { joltc_set_last_error("Unknown C++ exception"); }

#endif /* JOLTC_ERRORS_INTERNAL_H */
