/****************************************************************************
 *
 * Copyright (c) 2015 Mark Charlebois. All rights reserved.
 * Copyright (c) 2018 TC Development Team. All rights reserved.
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

#include "cdev_platform.hpp"

#include "../CDev.hpp"

#include <tc_platform_common/log.h>
#include <tc_platform_common/posix.h>
#include <tc_platform_common/time.h>

#include <stdlib.h>

const cdev::tc_file_operations_t cdev::CDev::fops = {};

pthread_mutex_t devmutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t filemutex = PTHREAD_MUTEX_INITIALIZER;

struct tc_dev_t {
	char *name{nullptr};
	cdev::CDev *cdev{nullptr};

	tc_dev_t(const char *n, cdev::CDev *c) : cdev(c)
	{
		name = strdup(n);
	}

	~tc_dev_t()
	{
		free(name);
	}
private:
	tc_dev_t() = default;
};

#define TC_MAX_FD 512
static tc_dev_t *devmap[TC_MAX_FD] {};
static cdev::file_t filemap[TC_MAX_FD] {};

class VFile : public cdev::CDev
{
public:
	VFile(const char *fname, mode_t mode) : cdev::CDev(fname) {}
	~VFile() override = default;

	ssize_t write(cdev::file_t *handlep, const char *buffer, size_t buflen) override
	{
		// ignore what was written, but let pollers know something was written
		poll_notify(POLLIN);
		return buflen;
	}
};

static cdev::CDev *getDev(const char *path)
{
	pthread_mutex_lock(&devmutex);

	for (const auto &dev : devmap) {
		if (dev && (strcmp(dev->name, path) == 0)) {
			pthread_mutex_unlock(&devmutex);
			return dev->cdev;
		}
	}

	pthread_mutex_unlock(&devmutex);

	return nullptr;
}

static cdev::CDev *getFile(int fd)
{
	pthread_mutex_lock(&filemutex);
	cdev::CDev *dev = nullptr;

	if (fd < TC_MAX_FD && fd >= 0) {
		dev = filemap[fd].cdev;
	}

	pthread_mutex_unlock(&filemutex);
	return dev;
}

extern "C" {

	int register_driver(const char *name, const cdev::tc_file_operations_t *fops, cdev::mode_t mode, void *data)
	{
		TC_DEBUG("CDev::register_driver %s", name);
		int ret = -ENOSPC;

		if (name == nullptr || data == nullptr) {
			return -EINVAL;
		}

		pthread_mutex_lock(&devmutex);

		// Make sure the device does not already exist
		for (const auto &dev : devmap) {
			if (dev && (strcmp(dev->name, name) == 0)) {
				pthread_mutex_unlock(&devmutex);
				return -EEXIST;
			}
		}

		for (auto &dev : devmap) {
			if (dev == nullptr) {
				dev = new tc_dev_t(name, (cdev::CDev *)data);
				TC_DEBUG("Registered DEV %s", name);
				ret = TC_OK;
				break;
			}
		}

		pthread_mutex_unlock(&devmutex);

		if (ret != TC_OK) {
			TC_ERR("No free devmap entries - increase devmap size");
		}

		return ret;
	}

	int unregister_driver(const char *name)
	{
		TC_DEBUG("CDev::unregister_driver %s", name);
		int ret = -EINVAL;

		if (name == nullptr) {
			return -EINVAL;
		}

		pthread_mutex_lock(&devmutex);

		for (auto &dev : devmap) {
			if (dev && (strcmp(name, dev->name) == 0)) {
				delete dev;
				dev = nullptr;
				TC_DEBUG("Unregistered DEV %s", name);
				ret = TC_OK;
				break;
			}
		}

		pthread_mutex_unlock(&devmutex);

		return ret;
	}

	int tc_open(const char *path, int flags, ...)
	{
		TC_DEBUG("tc_open");
		cdev::CDev *dev = getDev(path);
		int ret = 0;
		int i;
		mode_t mode;

		if (!dev && (flags & TC_F_WRONLY) != 0 &&
		    strncmp(path, "/obj/", 5) != 0 &&
		    strncmp(path, "/dev/", 5) != 0) {
			va_list p;
			va_start(p, flags);
			mode = va_arg(p, int);
			va_end(p);

			// Create the file
			TC_DEBUG("Creating virtual file %s", path);
			dev = new VFile(path, mode);
			register_driver(path, nullptr, 0666, (void *)dev);
		}

		if (dev) {
			pthread_mutex_lock(&filemutex);

			for (i = 0; i < TC_MAX_FD; ++i) {
				if (filemap[i].cdev == nullptr) {
					filemap[i] = cdev::file_t(flags, dev);
					break;
				}
			}

			pthread_mutex_unlock(&filemutex);

			if (i < TC_MAX_FD) {
				ret = dev->open(&filemap[i]);

			} else {

				const unsigned NAMELEN = 32;
				char thread_name[NAMELEN] {};

				TC_WARN("%s: exceeded maximum number of file descriptors, accessing %s", thread_name, path);
#ifndef __TC_QURT
				int nret = pthread_getname_np(pthread_self(), thread_name, NAMELEN);

				if (nret || thread_name[0] == 0) {
					TC_WARN("failed getting thread name");
				}

#endif

				ret = -ENOENT;
			}

		} else {
			ret = -EINVAL;
		}

		if (ret < 0) {
			errno = -ret;
			return -1;
		}

		TC_DEBUG("tc_open fd = %d", i);
		return i;
	}

	int tc_close(int fd)
	{
		int ret;

		cdev::CDev *dev = getFile(fd);

		if (dev) {
			pthread_mutex_lock(&filemutex);
			ret = dev->close(&filemap[fd]);

			filemap[fd].cdev = nullptr;

			pthread_mutex_unlock(&filemutex);
			TC_DEBUG("tc_close fd = %d", fd);

		} else {
			ret = -EINVAL;
		}

		if (ret < 0) {
			ret = TC_ERROR;
		}

		return ret;
	}

	ssize_t tc_read(int fd, void *buffer, size_t buflen)
	{
		int ret;

		cdev::CDev *dev = getFile(fd);

		if (dev) {
			TC_DEBUG("tc_read fd = %d", fd);
			ret = dev->read(&filemap[fd], (char *)buffer, buflen);

		} else {
			ret = -EINVAL;
		}

		if (ret < 0) {
			ret = TC_ERROR;
		}

		return ret;
	}

	ssize_t tc_write(int fd, const void *buffer, size_t buflen)
	{
		int ret;

		cdev::CDev *dev = getFile(fd);

		if (dev) {
			TC_DEBUG("tc_write fd = %d", fd);
			ret = dev->write(&filemap[fd], (const char *)buffer, buflen);

		} else {
			ret = -EINVAL;
		}

		if (ret < 0) {
			ret = TC_ERROR;
		}

		return ret;
	}

	int tc_ioctl(int fd, int cmd, unsigned long arg)
	{
		TC_DEBUG("tc_ioctl fd = %d", fd);
		int ret = 0;

		cdev::CDev *dev = getFile(fd);

		if (dev) {
			ret = dev->ioctl(&filemap[fd], cmd, arg);

		} else {
			ret = -EINVAL;
		}

		return ret;
	}

	int tc_poll(tc_pollfd_struct_t *fds, unsigned int nfds, int timeout)
	{
		if (nfds == 0) {
			TC_WARN("tc_poll with no fds");
			return -1;
		}

		tc_sem_t sem;
		int count = 0;
		int ret = -1;

		const unsigned NAMELEN = 32;
		char thread_name[NAMELEN] {};

#ifndef __TC_QURT
		int nret = pthread_getname_np(pthread_self(), thread_name, NAMELEN);

		if (nret || thread_name[0] == 0) {
			TC_WARN("failed getting thread name");
		}

#endif

		TC_DEBUG("Called tc_poll timeout = %d", timeout);

		tc_sem_init(&sem, 0, 0);

		// sem use case is a signal
		tc_sem_setprotocol(&sem, SEM_PRIO_NONE);

		// Go through all fds and check them for a pollable state
		bool fd_pollable = false;

		for (unsigned int i = 0; i < nfds; ++i) {
			fds[i].sem     = &sem;
			fds[i].revents = 0;
			fds[i].priv    = nullptr;

			cdev::CDev *dev = getFile(fds[i].fd);

			// If fd is valid
			if (dev) {
				TC_DEBUG("%s: tc_poll: CDev->poll(setup) %d", thread_name, fds[i].fd);
				ret = dev->poll(&filemap[fds[i].fd], &fds[i], true);

				if (ret < 0) {
					TC_WARN("%s: tc_poll() error: %s", thread_name, strerror(errno));
					break;
				}

				if (ret >= 0) {
					fd_pollable = true;
				}
			}
		}

		// If any FD can be polled, lock the semaphore and
		// check for new data
		if (fd_pollable) {
			if (timeout > 0) {
				// Get the current time
				struct timespec ts;
				// Note, we can't actually use CLOCK_MONOTONIC on macOS
				// but that's hidden and implemented in tc_clock_gettime.
				tc_clock_gettime(CLOCK_MONOTONIC, &ts);

				// Calculate an absolute time in the future
				const unsigned billion = (1000 * 1000 * 1000);
				uint64_t nsecs = ts.tv_nsec + ((uint64_t)timeout * 1000 * 1000);
				ts.tv_sec += nsecs / billion;
				nsecs -= (nsecs / billion) * billion;
				ts.tv_nsec = nsecs;

				ret = tc_sem_timedwait(&sem, &ts);

				if (ret && errno != ETIMEDOUT) {
					TC_WARN("%s: tc_poll() sem error: %s", thread_name, strerror(errno));
				}

			} else if (timeout < 0) {
				tc_sem_wait(&sem);
			}

			// We have waited now (or not, depending on timeout),
			// go through all fds and count how many have data
			for (unsigned int i = 0; i < nfds; ++i) {

				cdev::CDev *dev = getFile(fds[i].fd);

				// If fd is valid
				if (dev) {
					TC_DEBUG("%s: tc_poll: CDev->poll(teardown) %d", thread_name, fds[i].fd);
					ret = dev->poll(&filemap[fds[i].fd], &fds[i], false);

					if (ret < 0) {
						TC_WARN("%s: tc_poll() 2nd poll fail", thread_name);
						break;
					}

					if (fds[i].revents) {
						count += 1;
					}
				}
			}
		}

		tc_sem_destroy(&sem);

		// Return the positive count if present,
		// return the negative error number if failed
		return (count) ? count : ret;
	}

	int tc_access(const char *pathname, int mode)
	{
		if (mode != F_OK) {
			errno = EINVAL;
			return -1;
		}

		cdev::CDev *dev = getDev(pathname);
		return (dev != nullptr) ? 0 : -1;
	}

	void tc_show_files()
	{
		TC_INFO("Files:");

		pthread_mutex_lock(&devmutex);

		for (const auto &dev : devmap) {
			if (dev) {
				TC_INFO_RAW("   %s\n", dev->name);
			}
		}

		pthread_mutex_unlock(&devmutex);
	}

} // extern "C"
