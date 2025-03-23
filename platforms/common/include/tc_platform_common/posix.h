/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
 * @file tc_posix.h
 *
 * Includes POSIX-like functions for virtual character devices
 */

#pragma once

#include <tc_platform_common/defines.h>
#include <tc_platform_common/tasks.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>

#include "sem.h"

#define  TC_F_RDONLY 1
#define  TC_F_WRONLY 2

#ifdef __TC_NUTTX

#include <poll.h>

typedef struct pollfd tc_pollfd_struct_t;
typedef pollevent_t tc_pollevent_t;

#if defined(__cplusplus)
#define _GLOBAL ::
#else
#define _GLOBAL
#endif
#define tc_open 	_GLOBAL open
#define tc_close 	_GLOBAL close
#define tc_ioctl 	_GLOBAL ioctl
#define tc_write 	_GLOBAL write
#define tc_read 	_GLOBAL read
#define tc_poll 	_GLOBAL poll
#define tc_access 	_GLOBAL access
#define tc_getpid 	_GLOBAL getpid

#define  TC_STACK_OVERHEAD	0

#elif defined(__TC_POSIX)

#define	 TC_STACK_OVERHEAD	(1024 * 24)

#define tc_cache_aligned_data()
#define tc_cache_aligned_alloc malloc

__BEGIN_DECLS

typedef short tc_pollevent_t;

typedef struct {
	/* This part of the struct is POSIX-like */
	int		fd;       /* The descriptor being polled */
	tc_pollevent_t 	events;   /* The input event flags */
	tc_pollevent_t 	revents;  /* The output event flags */

	/* Required for TC compatibility */
	tc_sem_t   *sem;  	/* Pointer to semaphore used to post output event */
	void   *priv;     	/* For use by drivers */
} tc_pollfd_struct_t;

#ifndef POLLIN
#define POLLIN       (0x01)
#endif

#if defined(__TC_QURT)
// Qurt has no fsync implementation so need to declare one here
// and then define a fake one in the Qurt platform code.
void fsync(int fd);
// Qurt doesn't have a way to set the scheduler policy. It is always, essentially,
// SCHED_FIFO. So have to add a fake function for the code that tries to set it.
#include <pthread.h>
__EXPORT int pthread_attr_setschedpolicy(pthread_attr_t *attr, int policy);
// Qurt POSIX implementation doesn't define the SIGCONT signal so we just map it
// to a reasonable alternative
#define SIGCONT SIGALRM
#endif

__EXPORT int 		tc_open(const char *path, int flags, ...);
__EXPORT int 		tc_close(int fd);
__EXPORT ssize_t	tc_read(int fd, void *buffer, size_t buflen);
__EXPORT ssize_t	tc_write(int fd, const void *buffer, size_t buflen);
__EXPORT int		tc_ioctl(int fd, int cmd, unsigned long arg);
__EXPORT int		tc_poll(tc_pollfd_struct_t *fds, unsigned int nfds, int timeout);
__EXPORT int		tc_access(const char *pathname, int mode);
__EXPORT tc_task_t	tc_getpid(void);

__END_DECLS
#else
#error "No TARGET OS Provided"
#endif

// The stack size is intended for 32-bit architectures; therefore
// we often run out of stack space when pointers are larger than 4 bytes.
// Double the stack size on posix when we're on a 64-bit architecture.
// Most full-scale OS use 1-4K of memory from the stack themselves
#define TC_STACK_ADJUSTED(_s) (_s * (__SIZEOF_POINTER__ >> 2) + TC_STACK_OVERHEAD)

__BEGIN_DECLS

__EXPORT void		tc_show_files(void);

__END_DECLS
