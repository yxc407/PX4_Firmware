/****************************************************************************
 *
 *   Copyright (C) 2015-2016 TC Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name TC nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file log.h
 * Platform dependent logging/debug implementation
 */

#pragma once

#define _TC_LOG_LEVEL_DEBUG		0
#define _TC_LOG_LEVEL_INFO		1
#define _TC_LOG_LEVEL_WARN		2
#define _TC_LOG_LEVEL_ERROR		3
#define _TC_LOG_LEVEL_PANIC		4

// Used to silence unused variable warning
static inline void do_nothing(int level, ...)
{
	(void)level;
}

__BEGIN_DECLS

/**
 * initialize the orb logging. Logging to console still works without or before calling this.
 */
__EXPORT extern void tc_log_initialize(void);

__END_DECLS

/****************************************************************************
 * __tc_log_omit:
 * Compile out the message
 ****************************************************************************/
#define __tc_log_omit(level, FMT, ...)   do_nothing(level, ##__VA_ARGS__)

#if defined(__TC_QURT)
#include "qurt_log.h"
/****************************************************************************
 * Messages that should never be filtered or compiled out
 ****************************************************************************/
#define TC_INFO(FMT, ...) 	qurt_log(_TC_LOG_LEVEL_INFO, __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define TC_INFO_RAW(FMT, ...) 	qurt_log_raw(FMT, ##__VA_ARGS__)

#if defined(TRACE_BUILD)
/****************************************************************************
 * Extremely Verbose settings for a Trace build
 ****************************************************************************/
#define TC_PANIC(FMT, ...)	qurt_log(_TC_LOG_LEVEL_PANIC, __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define TC_ERR(FMT, ...)	qurt_log(_TC_LOG_LEVEL_ERROR, __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define TC_WARN(FMT, ...) 	qurt_log(_TC_LOG_LEVEL_WARN,  __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define TC_DEBUG(FMT, ...) 	qurt_log(_TC_LOG_LEVEL_DEBUG, __FILE__, __LINE__, FMT, ##__VA_ARGS__)

#elif defined(DEBUG_BUILD)
/****************************************************************************
 * Verbose settings for a Debug build
 ****************************************************************************/
#define TC_PANIC(FMT, ...)	qurt_log(_TC_LOG_LEVEL_PANIC, __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define TC_ERR(FMT, ...)	qurt_log(_TC_LOG_LEVEL_ERROR, __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define TC_WARN(FMT, ...) 	qurt_log(_TC_LOG_LEVEL_WARN, __FILE__, __LINE__,  FMT, ##__VA_ARGS__)
#define TC_DEBUG(FMT, ...) 	qurt_log(_TC_LOG_LEVEL_DEBUG, __FILE__, __LINE__, FMT, ##__VA_ARGS__)

#elif defined(RELEASE_BUILD)
/****************************************************************************
 * Non-verbose settings for a Release build to minimize strings in build
 ****************************************************************************/
#define TC_PANIC(FMT, ...)	qurt_log(_TC_LOG_LEVEL_PANIC, __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define TC_ERR(FMT, ...)	qurt_log(_TC_LOG_LEVEL_ERROR, __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define TC_WARN(FMT, ...) 	__tc_log_omit(_TC_LOG_LEVEL_WARN, FMT, ##__VA_ARGS__)
#define TC_DEBUG(FMT, ...) 	__tc_log_omit(_TC_LOG_LEVEL_DEBUG, FMT, ##__VA_ARGS__)

#else
/****************************************************************************
 * Medium verbose settings for a default build
 ****************************************************************************/
#define TC_PANIC(FMT, ...)	qurt_log(_TC_LOG_LEVEL_PANIC, __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define TC_ERR(FMT, ...)	qurt_log(_TC_LOG_LEVEL_ERROR, __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define TC_WARN(FMT, ...) 	qurt_log(_TC_LOG_LEVEL_WARN,  __FILE__, __LINE__, FMT, ##__VA_ARGS__)
#define TC_DEBUG(FMT, ...) 	__tc_log_omit(_TC_LOG_LEVEL_DEBUG, FMT, ##__VA_ARGS__)

#endif
#define TC_LOG_NAMED(name, FMT, ...) 	qurt_log( _TC_LOG_LEVEL_INFO, __FILE__, __LINE__, "%s " FMT, name, ##__VA_ARGS__)
#define TC_LOG_NAMED_COND(name, cond, FMT, ...) if( cond ) qurt_log( _TC_LOG_LEVEL_INFO, __FILE__, __LINE__, "%s " FMT, name,  ##__VA_ARGS__)

#else

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

#include <tc_platform_common/defines.h>
#include <drivers/drv_hrt.h>

__BEGIN_DECLS

__EXPORT extern const char *__tc_log_level_str[_TC_LOG_LEVEL_PANIC + 1];
__EXPORT void tc_log_modulename(int level, const char *moduleName, const char *fmt, ...)
__attribute__((format(printf, 3, 4)));
__EXPORT void tc_log_raw(int level, const char *fmt, ...)
__attribute__((format(printf, 2, 3)));
__EXPORT void tc_log_history(FILE *out);

#if __GNUC__
// Allow empty format strings.
#pragma GCC diagnostic ignored "-Wformat-zero-length"
#endif

__END_DECLS


/****************************************************************************
 * Implementation of log section formatting based on printf
 *
 * To write to a specific stream for each message type, open the streams and
 * set __tc__log_startline to something like:
 * 	printf(_tc_fd[level],
 *
 * Additional behavior can be added using "{\" for __tc__log_startline and
 * "}" for __tc__log_endline and any other required setup or teardown steps
 ****************************************************************************/
#define __tc__log_printcond(cond, ...)	    if (cond) printf(__VA_ARGS__)
#define __tc__log_printline(level, ...)    printf(__VA_ARGS__)

#define __tc__log_timestamp_fmt	"%-10" PRIu64 " "
#define __tc__log_timestamp_arg 	,hrt_absolute_time()
#define __tc__log_level_fmt		"%-5s "
#define __tc__log_level_arg(level)	,__tc_log_level_str[level]
#define __tc__log_thread_fmt		"%#X "
#define __tc__log_thread_arg		,(unsigned int)pthread_self()
#define __tc__log_modulename_fmt	"%-10s "
#define __tc__log_modulename_pfmt	"[%s] "
#define __tc__log_modulename_arg	,"[" MODULE_NAME "]"

#define __tc__log_file_and_line_fmt 	" (file %s line %u)"
#define __tc__log_file_and_line_arg 	, __FILE__, __LINE__
#define __tc__log_end_fmt 		"\n"

#ifdef __TC_POSIX
#define TC_LOG_COLORIZED_OUTPUT //if defined and output is a tty, colorize the output according to the log level
#endif /* __TC_POSIX */


/****************************************************************************
 * Output format macros
 * Use these to implement the code level macros below
 ****************************************************************************/


/****************************************************************************
 * __tc_log_named_cond:
 * Convert a message in the form:
 * 	TC_LOG_COND(__dbg_enabled, "val is %d", val);
 * to
 * 	printf("%-5s val is %d\n", "LOG", val);
 * if the first arg/condition is true.
 ****************************************************************************/
#define __tc_log_named_cond(name, cond, FMT, ...) \
	__tc__log_printcond(cond,\
			     "%s " \
			     FMT\
			     __tc__log_end_fmt \
			     ,name, ##__VA_ARGS__\
			    )

/****************************************************************************
 * __tc_log:
 * Convert a message in the form:
 * 	TC_WARN("val is %d", val);
 * to
 * 	printf("%-5s val is %d\n", __tc_log_level_str[3], val);
 ****************************************************************************/
#define __tc_log(level, FMT, ...) \
	__tc__log_printline(level,\
			     __tc__log_level_fmt \
			     FMT\
			     __tc__log_end_fmt \
			     __tc__log_level_arg(level), ##__VA_ARGS__\
			    )

/****************************************************************************
 * __tc_log_modulename:
 * Convert a message in the form:
 * 	TC_WARN("val is %d", val);
 * to
 * 	printf("%-5s [%s] val is %d\n", __tc_log_level_str[3],
 *		MODULENAME, val);
 ****************************************************************************/

#define __tc_log_modulename(level, fmt, ...) \
	do { \
		tc_log_modulename(level, MODULE_NAME, fmt, ##__VA_ARGS__); \
	} while(0)

/****************************************************************************
 * __tc_log_raw:
 * Convert a message in the form:
 * 	TC_INFO("val is %d", val);
 * to
 * 	printf("val is %d", val);
 *
 * This can be used for simple printfs with all the formatting control.
 ****************************************************************************/
#define __tc_log_raw(level, fmt, ...) \
	do { \
		tc_log_raw(level, fmt, ##__VA_ARGS__); \
	} while(0)


/****************************************************************************
 * __tc_log_timestamp:
 * Convert a message in the form:
 * 	TC_WARN("val is %d", val);
 * to
 * 	printf("%-5s %10lu val is %d\n", __tc_log_level_str[3],
 *		hrt_absolute_time(), val);
 ****************************************************************************/
#define __tc_log_timestamp(level, FMT, ...) \
	__tc__log_printline(level,\
			     __tc__log_level_fmt\
			     __tc__log_timestamp_fmt\
			     FMT\
			     __tc__log_end_fmt\
			     __tc__log_level_arg(level)\
			     __tc__log_timestamp_arg\
			     , ##__VA_ARGS__\
			    )

/****************************************************************************
 * __tc_log_timestamp_thread:
 * Convert a message in the form:
 * 	TC_WARN("val is %d", val);
 * to
 * 	printf("%-5s %10lu %#X val is %d\n", __tc_log_level_str[3],
 *		hrt_absolute_time(), pthread_self(), val);
 ****************************************************************************/
#define __tc_log_timestamp_thread(level, FMT, ...) \
	__tc__log_printline(level,\
			     __tc__log_level_fmt\
			     __tc__log_timestamp_fmt\
			     __tc__log_thread_fmt\
			     FMT\
			     __tc__log_end_fmt\
			     __tc__log_level_arg(level)\
			     __tc__log_timestamp_arg\
			     __tc__log_thread_arg\
			     , ##__VA_ARGS__\
			    )

/****************************************************************************
 * __tc_log_file_and_line:
 * Convert a message in the form:
 * 	TC_WARN("val is %d", val);
 * to
 * 	printf("%-5s val is %d (file %s line %u)\n",
 *		__tc_log_level_str[3], val, __FILE__, __LINE__);
 ****************************************************************************/
#define __tc_log_file_and_line(level, FMT, ...) \
	__tc__log_printline(level,\
			     __tc__log_level_fmt\
			     __tc__log_timestamp_fmt\
			     FMT\
			     __tc__log_file_and_line_fmt\
			     __tc__log_end_fmt\
			     __tc__log_level_arg(level)\
			     __tc__log_timestamp_arg\
			     , ##__VA_ARGS__\
			     __tc__log_file_and_line_arg\
			    )

/****************************************************************************
 * __tc_log_timestamp_file_and_line:
 * Convert a message in the form:
 * 	TC_WARN("val is %d", val);
 * to
 * 	printf("%-5s %-10lu val is %d (file %s line %u)\n",
 *		__tc_log_level_str[3], hrt_absolute_time(),
 *		val, __FILE__, __LINE__);
 ****************************************************************************/
#define __tc_log_timestamp_file_and_line(level, FMT, ...) \
	__tc__log_printline(level,\
			     __tc__log_level_fmt\
			     __tc__log_timestamp_fmt\
			     FMT\
			     __tc__log_file_and_line_fmt\
			     __tc__log_end_fmt\
			     __tc__log_level_arg(level)\
			     __tc__log_timestamp_arg\
			     , ##__VA_ARGS__\
			     __tc__log_file_and_line_arg\
			    )

/****************************************************************************
 * __tc_log_thread_file_and_line:
 * Convert a message in the form:
 * 	TC_WARN("val is %d", val);
 * to
 * 	printf("%-5s %#X val is %d (file %s line %u)\n",
 *		__tc_log_level_str[3], pthread_self(),
 *		val, __FILE__, __LINE__);
 ****************************************************************************/
#define __tc_log_thread_file_and_line(level, FMT, ...) \
	__tc__log_printline(level,\
			     __tc__log_level_fmt\
			     __tc__log_thread_fmt\
			     FMT\
			     __tc__log_file_and_line_fmt\
			     __tc__log_end_fmt\
			     __tc__log_level_arg(level)\
			     __tc__log_thread_arg\
			     , ##__VA_ARGS__\
			     __tc__log_file_and_line_arg\
			    )

/****************************************************************************
 * __tc_log_timestamp_thread_file_and_line:
 * Convert a message in the form:
 * 	TC_WARN("val is %d", val);
 * to
 * 	printf("%-5s %-10lu %#X val is %d (file %s line %u)\n",
 *		__tc_log_level_str[3], hrt_absolute_time(),
 *		pthread_self(), val, __FILE__, __LINE__);
 ****************************************************************************/
#define __tc_log_timestamp_thread_file_and_line(level, FMT, ...) \
	__tc__log_printline(level,\
			     __tc__log_level_fmt\
			     __tc__log_timestamp_fmt\
			     __tc__log_thread_fmt\
			     FMT\
			     __tc__log_file_and_line_fmt\
			     __tc__log_end_fmt\
			     __tc__log_level_arg(level)\
			     __tc__log_timestamp_arg\
			     __tc__log_thread_arg\
			     , ##__VA_ARGS__\
			     __tc__log_file_and_line_arg\
			    )


/****************************************************************************
 * Code level macros
 * These are the log APIs that should be used by the code
 ****************************************************************************/

/****************************************************************************
 * Messages that should never be filtered or compiled out
 ****************************************************************************/
#if defined(PRINTF_LOG)
#define TC_INFO(FMT, ...) 	printf(FMT "\n", ##__VA_ARGS__)
#else
#define TC_INFO(FMT, ...) 	__tc_log_modulename(_TC_LOG_LEVEL_INFO, FMT, ##__VA_ARGS__)
#endif

#if defined(__NUTTX) || defined(PRINTF_LOG)
#define TC_INFO_RAW		printf
#else
#define TC_INFO_RAW(FMT, ...) 	__tc_log_raw(_TC_LOG_LEVEL_INFO, FMT, ##__VA_ARGS__)
#endif

#if defined(PRINTF_LOG)
/****************************************************************************
 * Direct printf output for minimized dependencies
 ****************************************************************************/
#define TC_PANIC(FMT, ...)	printf("Panic: " FMT "\n", ##__VA_ARGS__)
#define TC_ERR(FMT, ...)	printf("Error: " FMT "\n", ##__VA_ARGS__)
#define TC_WARN(FMT, ...) 	printf("Warn: " FMT "\n", ##__VA_ARGS__)
#define TC_DEBUG(FMT, ...) 	printf(FMT "\n", ##__VA_ARGS__)

#elif defined(TRACE_BUILD)
/****************************************************************************
 * Extremely Verbose settings for a Trace build
 ****************************************************************************/
#define TC_PANIC(FMT, ...)	__tc_log_timestamp_thread_file_and_line(_TC_LOG_LEVEL_PANIC, FMT, ##__VA_ARGS__)
#define TC_ERR(FMT, ...)	__tc_log_timestamp_thread_file_and_line(_TC_LOG_LEVEL_ERROR, FMT, ##__VA_ARGS__)
#define TC_WARN(FMT, ...) 	__tc_log_timestamp_thread_file_and_line(_TC_LOG_LEVEL_WARN,  FMT, ##__VA_ARGS__)
#define TC_DEBUG(FMT, ...) 	__tc_log_timestamp_thread(_TC_LOG_LEVEL_DEBUG, FMT, ##__VA_ARGS__)

#elif defined(DEBUG_BUILD)
/****************************************************************************
 * Verbose settings for a Debug build
 ****************************************************************************/
#define TC_PANIC(FMT, ...)	__tc_log_timestamp_file_and_line(_TC_LOG_LEVEL_PANIC, FMT, ##__VA_ARGS__)
#define TC_ERR(FMT, ...)	__tc_log_timestamp_file_and_line(_TC_LOG_LEVEL_ERROR, FMT, ##__VA_ARGS__)
#define TC_WARN(FMT, ...) 	__tc_log_timestamp_file_and_line(_TC_LOG_LEVEL_WARN,  FMT, ##__VA_ARGS__)
#define TC_DEBUG(FMT, ...) 	__tc_log_timestamp(_TC_LOG_LEVEL_DEBUG, FMT, ##__VA_ARGS__)

#elif defined(RELEASE_BUILD)
/****************************************************************************
 * Non-verbose settings for a Release build to minimize strings in build
 ****************************************************************************/
#define TC_PANIC(FMT, ...)	__tc_log_modulename(_TC_LOG_LEVEL_PANIC, FMT, ##__VA_ARGS__)
#define TC_ERR(FMT, ...)	__tc_log_modulename(_TC_LOG_LEVEL_ERROR, FMT, ##__VA_ARGS__)
#define TC_WARN(FMT, ...) 	__tc_log_omit(_TC_LOG_LEVEL_WARN,  FMT, ##__VA_ARGS__)
#define TC_DEBUG(FMT, ...) 	__tc_log_omit(_TC_LOG_LEVEL_DEBUG, FMT, ##__VA_ARGS__)

#else
/****************************************************************************
 * Medium verbose settings for a default build
 ****************************************************************************/
#define TC_PANIC(FMT, ...)	__tc_log_modulename(_TC_LOG_LEVEL_PANIC, FMT, ##__VA_ARGS__)
#define TC_ERR(FMT, ...)	__tc_log_modulename(_TC_LOG_LEVEL_ERROR, FMT, ##__VA_ARGS__)
#define TC_WARN(FMT, ...) 	__tc_log_modulename(_TC_LOG_LEVEL_WARN,  FMT, ##__VA_ARGS__)

#ifndef TC_DEBUG
#define TC_DEBUG(FMT, ...) 	__tc_log_omit(_TC_LOG_LEVEL_DEBUG, FMT, ##__VA_ARGS__)
#endif

#endif
#define TC_LOG_NAMED(name, FMT, ...) 	__tc_log_named_cond(name, true, FMT, ##__VA_ARGS__)
#define TC_LOG_NAMED_COND(name, cond, FMT, ...) __tc_log_named_cond(name, cond, FMT, ##__VA_ARGS__)
#endif

#define TC_ANSI_COLOR_RED     "\x1b[31m"
#define TC_ANSI_COLOR_GREEN   "\x1b[32m"
#define TC_ANSI_COLOR_YELLOW  "\x1b[33m"
#define TC_ANSI_COLOR_BLUE    "\x1b[34m"
#define TC_ANSI_COLOR_MAGENTA "\x1b[35m"
#define TC_ANSI_COLOR_CYAN    "\x1b[36m"
#define TC_ANSI_COLOR_GRAY    "\x1B[37m"
#define TC_ANSI_COLOR_RESET   "\x1b[0m"

