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
#pragma once
#include <stdint.h>

typedef enum  {
	MFT = 0,
	MTD = 1,
	LAST_MFT_TYPE
} tc_manifest_types_e;

/* must match  tc_manifest_types_e */
#define TC_MFT_TYPES     {MFT, MTD}
#define TC_MFT_STR_TYPES {"MFT", "MTD"}

typedef struct  {

	enum tc_bus_type {
		I2C = 0,
		SPI = 1,
		ONCHIP = 2,
	} bus_type;

	uint32_t devid;
} tc_mft_device_t;

#define TC_MK_I2C_DEVID(b,a) ((b) << 16 | ((a) & 0xffff))
#define TC_I2C_DEVID_BUS(d)  (((d) >> 16) & 0xffff)
#define TC_I2C_DEVID_ADDR(d) ((d) & 0xffff)

typedef struct {
	const tc_manifest_types_e type;
	const void           *pmft;
} tc_mft_entry_s;

typedef struct {
	const uint32_t        nmft;
	const tc_mft_entry_s *mfts[];
} tc_mft_s;

#include "tc_platform_common/mtd_manifest.h"


__BEGIN_DECLS
/************************************************************************************
 * Name: board_get_manifest
 *
 * Description:
 *   A board will provide this function to return the manifest
 *
 * Input Parameters:
 *  mft    - a pointer to the receive the manifest
 *
 * Returned Value:
 *   non zero if error
 *
 ************************************************************************************/

__EXPORT const tc_mft_s *board_get_manifest(void);

/************************************************************************************
 * Name: board_get_base_eeprom_mtd_manifest
 *
 * Description:
 *   A board will provide this function to return the mtd with eeprom manifest
 *
 * Returned Value:
 *   pointer to mtd manifest
 *
 ************************************************************************************/

__EXPORT const tc_mtd_manifest_t *board_get_base_eeprom_mtd_manifest(void);

/************************************************************************************
 * Name: tc_mft_configure
 *
 * Description:
 *   The Px4 layer will provide this interface to start/configure the
 *   hardware.
 *
 * Input Parameters:
 *  mft    - a pointer to the manifest
 *
 * Returned Value:
 *   non zero if error
 *
 ************************************************************************************/

__EXPORT int tc_mft_configure(const tc_mft_s *mft);

/************************************************************************************
 * Name: tc_mft_configure
 *
 * Description:
 *   The Px4 layer will provide this interface to start/configure the
 *   hardware.
 *
 * Input Parameters:
 *  mft    - a pointer to the manifest
 *
 * Returned Value:
 *   non zero if error
 *
 ************************************************************************************/

__EXPORT int tc_mft_query(const tc_mft_s *mft, tc_manifest_types_e type,
			   const char *sub, const char *val);
__END_DECLS
