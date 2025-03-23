/****************************************************************************
 *
 *   Copyright (c) 2013-2014 TC Development Team. All rights reserved.
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
 * @file mtd.c
 *
 * mtd service and utility app.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <tc_platform_common/tc_config.h>
#include <tc_platform_common/log.h>
#include <tc_platform_common/module.h>
#include <tc_platform_common/spi.h>
#include <tc_platform_common/tc_manifest.h>
#include <tc_platform_common/getopt.h>

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <sys/mount.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/drivers/drivers.h>

#include <arch/board/board.h>

#include "systemlib/tc_macros.h"
#include <parameters/param.h>

#include <board_config.h>

static int mft_status(void)
{
	return 0;
}

static void print_usage()
{
	PRINT_MODULE_DESCRIPTION("Utility interact with the manifest");

	PRINT_MODULE_USAGE_NAME("mfd", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("query", "Returns true if not existed");
}

extern "C" __EXPORT int mft_main(int argc, char *argv[])
{
	static const char *keys[] = TC_MFT_STR_TYPES;
	static const tc_manifest_types_e types[] = TC_MFT_TYPES;

	int myoptind = 1;
	const char *myoptarg = nullptr;
	int ch;

	int silent = 0;
	tc_manifest_types_e key = MFT;
	const char *value   =   nullptr;
	const char *subject =   nullptr;

	while ((ch = tc_getopt(argc, argv, "qk:v:s:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'q':
			silent = 1;
			break;

		case 'k':
			for (unsigned int k = 0; k < arraySize(keys); k++) {
				if (!strcmp(keys[k], myoptarg)) {
					key = types[k];
					break;
				}
			}

			break;

		case 'v':
			value = myoptarg;
			break;

		case 's':
			subject = myoptarg;
			break;

		default:
			print_usage();
			return -1;
			break;
		}
	}

	if (myoptind < argc) {
		int rv = 0;

		if (!strcmp(argv[myoptind], "status")) {
			return mft_status();
		}

		if (!strcmp(argv[myoptind], "query")) {
			if (value && subject) {
				rv = tc_mft_query(board_get_manifest(), key, subject, value);

				if (!silent) {
					printf("%s\n", value);
				}

				return rv;
			}

			return -EINVAL;
		}

	}

	print_usage();
	return 1;
}
