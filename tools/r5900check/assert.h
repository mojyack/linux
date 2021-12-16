// SPDX-License-Identifier: GPL-2.0

#ifndef R5900CHECK_ASSERT_H
#define R5900CHECK_ASSERT_H

#include "types.h"

/* Macro definitions from the Linux kernel. */

#define __compiletime_error(message) __attribute__((__error__(message)))

#ifdef __OPTIMIZE__
# define __compiletime_assert(condition, msg, prefix, suffix)		\
	do {								\
		extern void prefix ## suffix(void) __compiletime_error(msg); \
		if (!(condition))					\
			prefix ## suffix();				\
	} while (0)
#else
# define __compiletime_assert(condition, msg, prefix, suffix) do { } while (0)
#endif

#define _compiletime_assert(condition, msg, prefix, suffix)		\
	__compiletime_assert(condition, msg, prefix, suffix)

/**
 * compiletime_assert - break build and emit msg if condition is false
 * @condition: a compile-time constant condition to check
 * @msg:       a message to emit if condition is false
 *
 * In tradition of POSIX assert, this macro will break the build if the
 * supplied condition is *false*, emitting the supplied error message if the
 * compiler has support to do so.
 */
#define compiletime_assert(condition, msg)				\
	_compiletime_assert(condition, msg, __compiletime_assert_, __LINE__)

#endif /* R5900CHECK_ASSERT_H */
