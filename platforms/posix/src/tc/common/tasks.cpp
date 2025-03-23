/****************************************************************************
 *
 *   Copyright (C) 2015-2020 Mark Charlebois. All rights reserved.
 *   Author: @author Mark Charlebois <charlebm@gmail.com>
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
 * Implementation of existing task API for Linux
 */

#include <tc_platform_common/log.h>
#include <tc_platform_common/defines.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdbool.h>
#include <signal.h>
#include <fcntl.h>
#include <sched.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <limits.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <string>

#include <tc_platform_common/tasks.h>
#include <tc_platform_common/posix.h>
#include <systemlib/err.h>

#define TC_MAX_TASKS 50

pthread_t _shell_task_id = 0;
pthread_mutex_t task_mutex = PTHREAD_MUTEX_INITIALIZER;

struct task_entry {
	pthread_t pid{0};
	std::string name{};
	bool isused {false};
};

static task_entry taskmap[TC_MAX_TASKS] {};

typedef struct {
	tc_main_t entry;
	char name[16]; //pthread_setname_np is restricted to 16 chars
	int argc;
	char *argv[];
	// strings are allocated after the struct data
} pthdata_t;

static void *entry_adapter(void *ptr)
{
	pthdata_t *data = (pthdata_t *) ptr;

	// set the threads name
#ifdef __TC_DARWIN
	int rv = pthread_setname_np(data->name);
#else
	int rv = pthread_setname_np(pthread_self(), data->name);
#endif

	if (rv) {
		TC_ERR("tc_task_spawn_cmd: failed to set name of thread %d %d\n", rv, errno);
	}

	data->entry(data->argc, data->argv);
	free(ptr);
	TC_DEBUG("Before tc_task_exit");
	tc_task_exit(0);
	TC_DEBUG("After tc_task_exit");

	return nullptr;
}

tc_task_t tc_task_spawn_cmd(const char *name, int scheduler, int priority, int stack_size, tc_main_t entry,
			      char *const argv[])
{
	int argc = 0;
	unsigned int len = strlen(name) + 1;
	struct sched_param param = {};
	char *p = (char *)argv;

	// Calculate argc
	while (p != (char *)nullptr) {
		p = argv[argc];

		if (p == (char *)nullptr) {
			break;
		}

		++argc;
		len += strlen(p) + 1;
	}

	unsigned long structsize = sizeof(pthdata_t) + (argc + 2) * sizeof(char *);

	// not safe to pass stack data to the thread creation
	pthdata_t *taskdata = (pthdata_t *)malloc(structsize + len);

	if (taskdata == nullptr) {
		return -ENOMEM;
	}

	memset(taskdata, 0, structsize + len);

	strncpy(taskdata->name, name, 16);
	taskdata->name[15] = '\0';
	taskdata->entry = entry;
	taskdata->argc = argc + 1;

	char *offset = (char *)taskdata + structsize;

	// We match the NuttX task_spawn implementation which copies
	// the name into argv[0] in order to provide a consistent API
	// to all tasks/modules.
	taskdata->argv[0] = offset;
	strcpy(offset, name);
	offset += strlen(name) + 1;

	for (int i = 0; i < argc; ++i) {
		TC_DEBUG("arg %d %s\n", i, argv[i]);
		taskdata->argv[i + 1] = offset;
		strcpy(offset, argv[i]);
		offset += strlen(argv[i]) + 1;
	}

	// Must add NULL at end of argv
	taskdata->argv[argc + 1] = (char *)nullptr;

	TC_DEBUG("starting task %s", name);

	pthread_attr_t attr;
	int rv = pthread_attr_init(&attr);

	if (rv != 0) {
		TC_ERR("tc_task_spawn_cmd: failed to init thread attrs");
		free(taskdata);
		return (rv < 0) ? rv : -rv;
	}

#ifndef __TC_DARWIN

	if (stack_size < PTHREAD_STACK_MIN) {
		stack_size = PTHREAD_STACK_MIN;
	}

	rv = pthread_attr_setstacksize(&attr, TC_STACK_ADJUSTED(stack_size));

	if (rv != 0) {
		TC_ERR("pthread_attr_setstacksize to %d returned error (%d)", stack_size, rv);
		pthread_attr_destroy(&attr);
		free(taskdata);
		return (rv < 0) ? rv : -rv;
	}

#endif

	rv = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

	if (rv != 0) {
		TC_ERR("tc_task_spawn_cmd: failed to set inherit sched");
		pthread_attr_destroy(&attr);
		free(taskdata);
		return (rv < 0) ? rv : -rv;
	}

	rv = pthread_attr_setschedpolicy(&attr, scheduler);

	if (rv != 0) {
		TC_ERR("tc_task_spawn_cmd: failed to set sched policy");
		pthread_attr_destroy(&attr);
		free(taskdata);
		return (rv < 0) ? rv : -rv;
	}

#ifdef __TC_CYGWIN
	/* Priorities on Windows are defined a lot differently */
	priority = SCHED_PRIORITY_DEFAULT;
#endif

	param.sched_priority = priority;

	rv = pthread_attr_setschedparam(&attr, &param);

	if (rv != 0) {
		TC_ERR("tc_task_spawn_cmd: failed to set sched param");
		pthread_attr_destroy(&attr);
		free(taskdata);
		return (rv < 0) ? rv : -rv;
	}

	pthread_mutex_lock(&task_mutex);

	tc_task_t taskid = 0;

	int i;

	for (i = 0; i < TC_MAX_TASKS; ++i) {
		if (!taskmap[i].isused) {
			taskmap[i].name = name;
			taskmap[i].isused = true;
			taskid = i;
			break;
		}
	}

	if (i >= TC_MAX_TASKS) {
		pthread_attr_destroy(&attr);
		pthread_mutex_unlock(&task_mutex);
		free(taskdata);
		return -ENOSPC;
	}

	rv = pthread_create(&taskmap[taskid].pid, &attr, &entry_adapter, (void *) taskdata);

	if (rv != 0) {

		if (rv == EPERM) {
			//printf("WARNING: NOT RUNING AS ROOT, UNABLE TO RUN REALTIME THREADS\n");
			rv = pthread_create(&taskmap[taskid].pid, nullptr, &entry_adapter, (void *) taskdata);

			if (rv != 0) {
				TC_ERR("tc_task_spawn_cmd: failed to create thread for %s (%i): %s", name, rv, strerror(rv));
				taskmap[taskid].isused = false;
				pthread_attr_destroy(&attr);
				pthread_mutex_unlock(&task_mutex);
				free(taskdata);
				return (rv < 0) ? rv : -rv;
			}

		} else {
			pthread_attr_destroy(&attr);
			pthread_mutex_unlock(&task_mutex);
			free(taskdata);
			return (rv < 0) ? rv : -rv;
		}
	}

	pthread_attr_destroy(&attr);
	pthread_mutex_unlock(&task_mutex);

	return taskid;
}

int tc_task_delete(tc_task_t id)
{
	int rv = 0;
	pthread_t pid;
	TC_DEBUG("Called tc_task_delete");

	if (id < TC_MAX_TASKS && taskmap[id].isused) {
		pid = taskmap[id].pid;

	} else {
		return -EINVAL;
	}

	pthread_mutex_lock(&task_mutex);

	// If current thread then exit, otherwise cancel
	if (pthread_self() == pid) {
		pthread_join(pid, nullptr);
		taskmap[id].isused = false;
		pthread_mutex_unlock(&task_mutex);
		pthread_exit(nullptr);

	} else {
		rv = pthread_cancel(pid);
	}

	taskmap[id].isused = false;
	pthread_mutex_unlock(&task_mutex);

	return rv;
}

void tc_task_exit(int ret)
{
	pthread_t pid = pthread_self();

	// Get pthread ID from the opaque ID
	int i;

	for (i = 0; i < TC_MAX_TASKS; ++i) {
		if (taskmap[i].pid == pid) {
			pthread_mutex_lock(&task_mutex);
			taskmap[i].isused = false;
			break;
		}
	}

	if (i >= TC_MAX_TASKS)  {
		TC_ERR("tc_task_exit: self task not found!");

	} else {
		TC_DEBUG("tc_task_exit: %s", taskmap[i].name.c_str());
	}

	pthread_mutex_unlock(&task_mutex);

	pthread_exit((void *)(unsigned long)ret);
}

int tc_task_kill(tc_task_t id, int sig)
{
	int rv = 0;
	pthread_t pid;
	TC_DEBUG("Called tc_task_kill %d", sig);

	if (id < TC_MAX_TASKS && taskmap[id].isused && taskmap[id].pid != 0) {
		pthread_mutex_lock(&task_mutex);
		pid = taskmap[id].pid;
		pthread_mutex_unlock(&task_mutex);

	} else {
		return -EINVAL;
	}

	// If current thread then exit, otherwise cancel
	rv = pthread_kill(pid, sig);

	return rv;
}

void tc_show_tasks()
{
	int idx;
	int count = 0;

	TC_INFO("Active Tasks:");

	for (idx = 0; idx < TC_MAX_TASKS; idx++) {
		if (taskmap[idx].isused) {
			TC_INFO("   %-10s %lu", taskmap[idx].name.c_str(), (unsigned long)taskmap[idx].pid);
			count++;
		}
	}

	if (count == 0) {
		TC_INFO("   No running tasks");
	}

}

bool tc_task_is_running(const char *taskname)
{
	int idx;

	for (idx = 0; idx < TC_MAX_TASKS; idx++) {
		if (taskmap[idx].isused && (strcmp(taskmap[idx].name.c_str(), taskname) == 0)) {
			return true;
		}
	}

	return false;
}

tc_task_t tc_getpid()
{
	pthread_t pid = pthread_self();
	tc_task_t ret = -1;

	pthread_mutex_lock(&task_mutex);

	for (int i = 0; i < TC_MAX_TASKS; i++) {
		if (taskmap[i].isused && taskmap[i].pid == pid) {
			ret = i;
		}
	}

	pthread_mutex_unlock(&task_mutex);
	return ret;
}

const char *tc_get_taskname()
{
	pthread_t pid = pthread_self();
	const char *prog_name = "UnknownApp";

	pthread_mutex_lock(&task_mutex);

	for (int i = 0; i < TC_MAX_TASKS; i++) {
		if (taskmap[i].isused && taskmap[i].pid == pid) {
			prog_name = taskmap[i].name.c_str();
		}
	}

	pthread_mutex_unlock(&task_mutex);

	return prog_name;
}

int tc_prctl(int option, const char *arg2, tc_task_t pid)
{
	int rv = -1;

	switch (option) {
	case PR_SET_NAME:
		// set the threads name
#ifdef __TC_DARWIN
		rv = pthread_setname_np(arg2);
#else
		rv = pthread_setname_np(pthread_self(), arg2);
#endif
		break;

	default:
		TC_WARN("FAILED SETTING TASK NAME");
		break;
	}

	return rv;
}
