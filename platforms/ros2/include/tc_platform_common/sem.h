/****************************************************************************
 *
 *   Copyright (c) 2016 TC Development Team. All rights reserved.
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
 * @file sem.h
 *
 * Synchronization primitive: Semaphore
 */

#pragma once

#include <semaphore.h>

#if !defined(__TC_NUTTX)
/* Values for protocol attribute */

#define SEM_PRIO_NONE             0
#define SEM_PRIO_INHERIT          1
#define SEM_PRIO_PROTECT          2
#define sem_setprotocol(s,p)
#endif

#if (defined(__TC_DARWIN) || defined(__TC_CYGWIN) || defined(__TC_POSIX) || defined(__TC_ROS2)) && !defined(__TC_QURT)

__BEGIN_DECLS

typedef struct {
	pthread_mutex_t lock;
	pthread_cond_t wait;
	int value;
} tc_sem_t;

__EXPORT int		tc_sem_init(tc_sem_t *s, int pshared, unsigned value);
__EXPORT int		tc_sem_setprotocol(tc_sem_t *s, int protocol);
__EXPORT int		tc_sem_wait(tc_sem_t *s);
__EXPORT int		tc_sem_trywait(tc_sem_t *sem);
__EXPORT int		tc_sem_timedwait(tc_sem_t *sem, const struct timespec *abstime);
__EXPORT int		tc_sem_post(tc_sem_t *s);
__EXPORT int		tc_sem_getvalue(tc_sem_t *s, int *sval);
__EXPORT int		tc_sem_destroy(tc_sem_t *s);

__END_DECLS

//#elif defined(__TC_QURT)

//typedef sem_t tc_sem_t;

//#define tc_sem_init		sem_init
//#define tc_sem_setprotocol sem_setprotocol
//#define tc_sem_wait		sem_wait
//#define tc_sem_trywait	sem_trywait
//#define tc_sem_post		sem_post
//#define tc_sem_getvalue	sem_getvalue
//#define tc_sem_destroy		sem_destroy

#else

typedef sem_t tc_sem_t;

__BEGIN_DECLS

#define tc_sem_init		sem_init
#define tc_sem_setprotocol	sem_setprotocol
#define tc_sem_wait		sem_wait
#define tc_sem_trywait		sem_trywait
#define tc_sem_post		sem_post
#define tc_sem_getvalue	sem_getvalue
#define tc_sem_destroy		sem_destroy

#if defined(__TC_QURT)
__EXPORT int		tc_sem_timedwait(tc_sem_t *sem, const struct timespec *abstime);
#else
#define tc_sem_timedwait	sem_timedwait
#endif

__END_DECLS

#endif
