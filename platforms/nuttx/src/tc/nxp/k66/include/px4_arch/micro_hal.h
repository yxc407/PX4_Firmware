/****************************************************************************
 *
 *   Copyright (c) 2019 TC Development Team. All rights reserved.
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


#include "../../../nxp_common/include/tc_arch/micro_hal.h"

__BEGIN_DECLS

#define TC_SOC_ARCH_ID             TC_SOC_ARCH_ID_KINETISK66

// Fixme: using ??
#define TC_BBSRAM_SIZE             2048
#define TC_HF_GETDESC_IOCTL        0
#define TC_NUMBER_I2C_BUSES        KINETIS_NI2C

#define GPIO_OUTPUT_SET             GPIO_OUTPUT_ONE
#define GPIO_OUTPUT_CLEAR           GPIO_OUTPUT_ZERO

#include <chip.h>
#include <kinetis_spi.h>
#include <kinetis_i2c.h>
#include <kinetis_uid.h>

/* Kinetis defines the 128 bit UUID as
 *  init32_t[4] that can be read as words
 *  init32_t[0] TC_CPU_UUID_ADDRESS[0] bits 127:96 (offset 0)
 *  init32_t[1] TC_CPU_UUID_ADDRESS[1] bits 95:64  (offset 4)
 *  init32_t[2] TC_CPU_UUID_ADDRESS[1] bits 63:32  (offset 8)
 *  init32_t[3] TC_CPU_UUID_ADDRESS[3] bits 31:0   (offset C)
 *
 *  TC uses the words in bigendian order MSB to LSB
 *   word  [0]    [1]    [2]   [3]
 *   bits 127:96  95-64  63-32, 31-00,
 */
#define TC_CPU_UUID_BYTE_LENGTH                KINETIS_UID_SIZE
#define TC_CPU_UUID_WORD32_LENGTH              (TC_CPU_UUID_BYTE_LENGTH/sizeof(uint32_t))

/* The mfguid will be an array of bytes with
 * MSD @ index 0 - LSD @ index TC_CPU_MFGUID_BYTE_LENGTH-1
 *
 * It will be converted to a string with the MSD on left and LSD on the right most position.
 */
#define TC_CPU_MFGUID_BYTE_LENGTH              TC_CPU_UUID_BYTE_LENGTH

/* define common formating across all commands */

#define TC_CPU_UUID_WORD32_FORMAT              "%08x"
#define TC_CPU_UUID_WORD32_SEPARATOR           ":"

#define TC_CPU_UUID_WORD32_UNIQUE_H            3 /* Least significant digits change the most */
#define TC_CPU_UUID_WORD32_UNIQUE_M            2 /* Middle High significant digits */
#define TC_CPU_UUID_WORD32_UNIQUE_L            1 /* Middle Low significant digits */
#define TC_CPU_UUID_WORD32_UNIQUE_N            0 /* Most significant digits change the least */

/*                                               Separator    nnn:nnn:nnnn     2 char per byte           term */
#define TC_CPU_UUID_WORD32_FORMAT_SIZE         (TC_CPU_UUID_WORD32_LENGTH-1+(2*TC_CPU_UUID_BYTE_LENGTH)+1)
#define TC_CPU_MFGUID_FORMAT_SIZE              ((2*TC_CPU_MFGUID_BYTE_LENGTH)+1)

/* bus_num is zero based on kinetis and must be translated from the legacy one based */

#define TC_BUS_OFFSET       1                  /* Kinetis buses are 0 based and adjustment is needed */

#define tc_spibus_initialize(bus_num_1based)   kinetis_spibus_initialize(TC_BUS_NUMBER_FROM_TC(bus_num_1based))

#define tc_i2cbus_initialize(bus_num_1based)   kinetis_i2cbus_initialize(TC_BUS_NUMBER_FROM_TC(bus_num_1based))
#define tc_i2cbus_uninitialize(pdev)           kinetis_i2cbus_uninitialize(pdev)

#define tc_arch_configgpio(pinset)             kinetis_pinconfig(pinset)
#define tc_arch_unconfiggpio(pinset)
#define tc_arch_gpioread(pinset)               kinetis_gpioread(pinset)
#define tc_arch_gpiowrite(pinset, value)       kinetis_gpiowrite(pinset, value)

/* kinetis_gpiosetevent is added at TC level */

int kinetis_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge, bool event, xcpt_t func, void *arg);

#define tc_arch_gpiosetevent(pinset,r,f,e,fp,a)  kinetis_gpiosetevent(pinset,r,f,e,fp,a)

#define _TC_MAKE_GPIO(pin_ftmx, io)    ((((uint32_t)(pin_ftmx)) & ~(_PIN_MODE_MASK | _PIN_OPTIONS_MASK)) |(io))
#define TC_MAKE_GPIO_INPUT(gpio) _TC_MAKE_GPIO(gpio, GPIO_PULLUP)
#define TC_MAKE_GPIO_EXTI(gpio) _TC_MAKE_GPIO(gpio, PIN_INT_BOTH | GPIO_PULLUP)
#define TC_MAKE_GPIO_OUTPUT_SET(gpio) _TC_MAKE_GPIO(gpio, GPIO_HIGHDRIVE)
#define TC_MAKE_GPIO_OUTPUT_CLEAR(gpio) _TC_MAKE_GPIO(gpio, GPIO_LOWDRIVE)

__END_DECLS
