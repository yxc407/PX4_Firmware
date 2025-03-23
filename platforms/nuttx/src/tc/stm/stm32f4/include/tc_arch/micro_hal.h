/****************************************************************************
 *
 *   Copyright (c) 2019 tc Development Team. All rights reserved.
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
 * 3. Neither the name tc nor the names of its contributors may be
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

#include "../../../stm32_common/include/tc_arch/micro_hal.h"

__BEGIN_DECLS

#include <stm32.h>
#define TC_SOC_ARCH_ID             TC_SOC_ARCH_ID_STM32F4
#define TC_FLASH_BASE  STM32_FLASH_BASE
#if defined(CONFIG_STM32_STM32F4XXX)
# include <stm32_bbsram.h>
# define TC_BBSRAM_SIZE STM32_BBSRAM_SIZE
# define TC_HF_GETDESC_IOCTL STM32_BBSRAM_GETDESC_IOCTL
# define HAS_BBSRAM CONFIG_STM32_BBSRAM
# define BBSRAM_FILE_COUNT CONFIG_STM32_BBSRAM_FILES
# define SAVE_CRASHDUMP CONFIG_STM32_SAVE_CRASHDUMP
#endif
#define TC_NUMBER_I2C_BUSES STM32_NI2C
#define TC_ADC_INTERNAL_TEMP_SENSOR_CHANNEL 18
__END_DECLS

