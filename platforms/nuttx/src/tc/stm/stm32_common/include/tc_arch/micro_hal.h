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

#include <tc_platform/micro_hal.h>
// #include "../../../../../../ros2/src/tc/common/include/tc_platform/micro_hal.h"
//#include "/home/thuncloud/TC-Autopilot/platforms/ros2/src/tc/common/include/tc_platform/micro_hal.h"
__BEGIN_DECLS

#include <stm32_tim.h>
#include <stm32_spi.h>
#include <stm32_i2c.h>

/* STM32/32F7 defines the 96 bit UUID as
 *  init32_t[3] that can be read as bytes/half-words/words
 *  init32_t[0] TC_CPU_UUID_ADDRESS[0] bits 31:0  (offset 0)
 *  init32_t[1] TC_CPU_UUID_ADDRESS[1] bits 63:32 (offset 4)
 *  init32_t[2] TC_CPU_UUID_ADDRESS[3] bits 96:64 (offset 8)
 *
 * The original TC stm32 (legacy) based implementation **displayed** the
 * UUID as: ABCD EFGH IJKL
 * Where:
 *       A was bit 31 and D was bit 0
 *       E was bit 63 and H was bit 32
 *       I was bit 95 and L was bit 64
 *
 * Since the string was used by some manufactures to identify the units
 * it must be preserved.
 *
 * For new targets moving forward we will use
 *      IJKL EFGH ABCD
 */
#define TC_CPU_UUID_BYTE_LENGTH                12
#define TC_CPU_UUID_WORD32_LENGTH              (TC_CPU_UUID_BYTE_LENGTH/sizeof(uint32_t))

/* The mfguid will be an array of bytes with
 * MSD @ index 0 - LSD @ index TC_CPU_MFGUID_BYTE_LENGTH-1
 *
 * It will be converted to a string with the MSD on left and LSD on the right most position.
 */
#define TC_CPU_MFGUID_BYTE_LENGTH              TC_CPU_UUID_BYTE_LENGTH

/* By not defining TC_CPU_UUID_CORRECT_CORRELATION the following maintains the legacy incorrect order
 * used for selection of significant digits of the UUID in the TC code base.
 * This is done to avoid the ripple effects changing the IDs used on STM32 base platforms
 */
#if defined(TC_CPU_UUID_CORRECT_CORRELATION)
# define TC_CPU_UUID_WORD32_UNIQUE_H            0 /* Least significant digits change the most */
# define TC_CPU_UUID_WORD32_UNIQUE_M            1 /* Middle significant digits */
# define TC_CPU_UUID_WORD32_UNIQUE_L            2 /* Most significant digits change the least */
#else
/* Legacy incorrect ordering */
# define TC_CPU_UUID_WORD32_UNIQUE_H            2 /* Most significant digits change the least */
# define TC_CPU_UUID_WORD32_UNIQUE_M            1 /* Middle significant digits */
# define TC_CPU_UUID_WORD32_UNIQUE_L            0 /* Least significant digits change the most */
#endif

/*                                                  Separator    nnn:nnn:nnnn     2 char per byte           term */
#define TC_CPU_UUID_WORD32_FORMAT_SIZE         (TC_CPU_UUID_WORD32_LENGTH-1+(2*TC_CPU_UUID_BYTE_LENGTH)+1)
#define TC_CPU_MFGUID_FORMAT_SIZE              ((2*TC_CPU_MFGUID_BYTE_LENGTH)+1)

#define tc_savepanic(fileno, context, length)  stm32_bbsram_savepanic(fileno, context, length)

#define TC_BUS_OFFSET       0                  /* STM buses are 1 based no adjustment needed */
#define tc_spibus_initialize(bus_num_1based)   stm32_spibus_initialize(bus_num_1based)

#define tc_i2cbus_initialize(bus_num_1based)   stm32_i2cbus_initialize(bus_num_1based)
#define tc_i2cbus_uninitialize(pdev)           stm32_i2cbus_uninitialize(pdev)

#define tc_arch_configgpio(pinset)             stm32_configgpio(pinset)
#define tc_arch_unconfiggpio(pinset)           stm32_unconfiggpio(pinset)
#define tc_arch_gpioread(pinset)               stm32_gpioread(pinset)
#define tc_arch_gpiowrite(pinset, value)       stm32_gpiowrite(pinset, value)
#define tc_arch_gpiosetevent(pinset,r,f,e,fp,a)  stm32_gpiosetevent(pinset,r,f,e,fp,a)

#define TC_MAKE_GPIO_INPUT(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLUP))
#define TC_MAKE_GPIO_EXTI(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_EXTI|GPIO_INPUT|GPIO_PULLUP))
#define TC_MAKE_GPIO_OUTPUT_CLEAR(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR))
#define TC_MAKE_GPIO_OUTPUT_SET(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET))

#define TC_GPIO_PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_2MHz))

/* CAN bootloader usage */

#define TIMER_HRT_CYCLES_PER_US (STM32_HCLK_FREQUENCY/1000000)
#define TIMER_HRT_CYCLES_PER_MS (STM32_HCLK_FREQUENCY/1000)

/*  CAN_FiRx where (i=0..27|13, x=1, 2)
 *                      STM32_CAN1_FIR(i,x)
 * Using i = 2 does not requier there block
 * to be enabled nor FINIT in CAN_FMR to be set.
 * todo:Validate this claim on F2, F3
 */

#define crc_HiLOC       STM32_CAN1_FIR(2,1)
#define crc_LoLOC       STM32_CAN1_FIR(2,2)
#define signature_LOC   STM32_CAN1_FIR(3,1)
#define bus_speed_LOC   STM32_CAN1_FIR(3,2)
#define node_id_LOC     STM32_CAN1_FIR(4,1)

#if defined(CONFIG_ARMV7M_DCACHE)
#  define TC_ARCH_DCACHE_ALIGNMENT ARMV7M_DCACHE_LINESIZE
#  define tc_cache_aligned_data() aligned_data(ARMV7M_DCACHE_LINESIZE)
#  define tc_cache_aligned_alloc(s) memalign(ARMV7M_DCACHE_LINESIZE,(s))
#else
#  define tc_cache_aligned_data()
#  define tc_cache_aligned_alloc malloc
#endif


__END_DECLS
