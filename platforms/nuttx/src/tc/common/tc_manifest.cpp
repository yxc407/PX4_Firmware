/****************************************************************************
 *
 *   Copyright (c) 2020 TC Development Team. All rights reserved.
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
 * @file tc_manifest.cpp
 *
 * manifest utilites
 *
 * @author David Sidrane <david.sidrane@nscdg.com>
 */

#ifndef MODULE_NAME
#define MODULE_NAME "TC_MANIFEST"
#endif

#include <tc_platform_common/tc_config.h>
#include <tc_platform_common/tc_manifest.h>
#include <tc_platform_common/log.h>

#include <errno.h>

__EXPORT const tc_mft_s *board_get_manifest(void) weak_function;

/* This is the default manifest when no MTD driver is installed */
static const tc_mft_entry_s mtd_mft = {
	.type = MTD,
};

static const tc_mft_s default_mft = {
	.nmft = 1,
	.mfts =  {
		&mtd_mft
	}
};


const tc_mft_s *board_get_manifest(void)
{
	return &default_mft;
}


__EXPORT int tc_mft_configure(const tc_mft_s *mft)
{

	if (mft != nullptr) {
		for (uint32_t m = 0; m < mft->nmft; m++) {
			switch (mft->mfts[m]->type) {
			case MTD:
				tc_mtd_config(static_cast<const tc_mtd_manifest_t *>(mft->mfts[m]->pmft));
				break;

			case MFT:
			default:
				break;
			}
		}
	}

	return 0;
}

__EXPORT int tc_mft_query(const tc_mft_s *mft, tc_manifest_types_e type,
			   const char *sub, const char *val)
{
	int rv = -EINVAL;

	if (mft != nullptr) {
		for (uint32_t m = 0; m < mft->nmft; m++) {
			if (mft->mfts[m]->type == type)
				switch (type) {
				case MTD:
					return tc_mtd_query(sub, val, nullptr);
					break;

				case MFT:
				default:
					rv = -ENODATA;
					break;
				}
		}
	}

	return rv;
}
