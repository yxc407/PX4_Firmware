/****************************************************************************
 *
 *   Copyright (c) 2018-2019 TC Development Team. All rights reserved.
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

#include <tc_platform_common/cli.h>
#include <tc_platform_common/getopt.h>

#include "CM8JL65.hpp"

/**
 * Local functions in support of the shell command.
 */
namespace cm8jl65
{

CM8JL65	*g_dev;

int reset(const char *port);
int start(const char *port, const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
int status();
int stop();
int usage();

/**
 * Reset the driver.
 */
int
reset(const char *port)
{
	if (stop() == TC_OK) {
		return start(port);
	}

	return TC_ERROR;
}

/**
 * Start the driver.
 */
int
start(const char *port, const uint8_t rotation)
{
	if (port == nullptr) {
		TC_ERR("invalid port");
		return TC_ERROR;
	}

	if (g_dev != nullptr) {
		TC_INFO("already started");
		return TC_OK;
	}

	// Instantiate the driver.
	g_dev = new CM8JL65(port, rotation);

	if (g_dev == nullptr) {
		TC_ERR("object instantiate failed");
		return TC_ERROR;
	}

	if (g_dev->init() != TC_OK) {
		TC_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return TC_ERROR;
	}

	return TC_OK;
}

/**
 * Print the driver status.
 */
int
status()
{
	if (g_dev == nullptr) {
		TC_ERR("driver not running");
		return TC_ERROR;
	}

	g_dev->print_info();

	return TC_OK;
}

/**
 * Stop the driver
 */
int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return TC_ERROR;
}

int
usage()
{
	TC_INFO("usage: cm8jl65 command [options]");
	TC_INFO("command:");
	TC_INFO("\treset|start|status|stop");
	TC_INFO("options:");
	TC_INFO("\t-R --rotation (%d)", distance_sensor_s::ROTATION_DOWNWARD_FACING);
	TC_INFO("\t-d --device_path");
	return TC_OK;
}

} // namespace cm8jl65


/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int cm8jl65_main(int argc, char *argv[])
{
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *device_path = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = tc_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R': {
				int rot = -1;

				if (tc_get_parameter_value(myoptarg, rot) != 0) {
					TC_ERR("rotation parsing failed");
					return -1;
				}

				rotation = (uint8_t)rot;
				break;
			}

		case 'd':
			device_path = myoptarg;
			break;

		default:
			TC_WARN("Unknown option!");
			return cm8jl65::usage();
		}
	}

	if (myoptind >= argc) {
		return cm8jl65::usage();
	}

	// Reset the driver.
	if (!strcmp(argv[myoptind], "reset")) {
		return cm8jl65::reset(device_path);
	}

	// Start/load the driver.
	if (!strcmp(argv[myoptind], "start")) {
		return cm8jl65::start(device_path, rotation);
	}

	// Print driver information.
	if (!strcmp(argv[myoptind], "status")) {
		return cm8jl65::status();
	}

	// Stop the driver
	if (!strcmp(argv[myoptind], "stop")) {
		return cm8jl65::stop();
	}

	return cm8jl65::usage();
}
