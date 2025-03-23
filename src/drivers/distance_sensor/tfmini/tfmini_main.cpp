/****************************************************************************
 *
 *   Copyright (c) 2017-2019 TC Development Team. All rights reserved.
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

#include "TFMINI.hpp"

#include <tc_platform_common/getopt.h>

/**
 * Local functions in support of the shell command.
 */
namespace tfmini
{

TFMINI	*g_dev{nullptr};

int start(const char *port, uint8_t rotation);
int status();
int stop();
int usage();

int
start(const char *port, uint8_t rotation)
{
	if (g_dev != nullptr) {
		TC_ERR("already started");
		return TC_OK;
	}

	// Instantiate the driver.
	g_dev = new TFMINI(port, rotation);

	if (g_dev == nullptr) {
		TC_ERR("driver start failed");
		return TC_ERROR;
	}

	if (OK != g_dev->init()) {
		TC_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return TC_ERROR;
	}

	return TC_OK;
}

int
status()
{
	if (g_dev == nullptr) {
		TC_ERR("driver not running");
		return 1;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return 0;
}

int stop()
{
	if (g_dev != nullptr) {
		TC_INFO("stopping driver");
		delete g_dev;
		g_dev = nullptr;
		TC_INFO("driver stopped");

	} else {
		TC_ERR("driver not running");
		return 1;
	}

	return TC_OK;
}

int
usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the Benewake TFmini LiDAR.

Most boards are configured to enable/start the driver on a specified UART using the SENS_TFMINI_CFG parameter.

Setup/usage information: https://docs.tc.io/main/en/sensor/tfmini.html

### Examples

Attempt to start driver on a specified serial device.
$ tfmini start -d /dev/ttyS1
Stop driver
$ tfmini stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tfmini", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test","Test driver (basic functional tests)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Print driver status");
	return TC_OK;
}

} // namespace

extern "C" __EXPORT int tfmini_main(int argc, char *argv[])
{
	int ch = 0;
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *device_path = TFMINI_DEFAULT_PORT;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = tc_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		case 'd':
			device_path = myoptarg;
			break;

		default:
			TC_WARN("Unknown option!");
			return TC_ERROR;
		}
	}

	if (myoptind >= argc) {
		TC_ERR("unrecognized command");
		return tfmini::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		if (strcmp(device_path, "") != 0) {
			return tfmini::start(device_path, rotation);

		} else {
			TC_WARN("Please specify device path!");
			return tfmini::usage();
		}

	} else if (!strcmp(argv[myoptind], "stop")) {
		return tfmini::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		return tfmini::status();
	}

	return tfmini::usage();
}
