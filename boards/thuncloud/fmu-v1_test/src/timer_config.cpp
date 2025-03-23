/****************************************************************************
 *
 *   Copyright (C) 2012 TC Development Team. All rights reserved.
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

// #include <tc_arch/io_timer_hw_description.h>
#include "/home/thuncloud/TC-Autopilot/platforms/nuttx/src/tc/stm/stm32h7/include/tc_arch/io_timer_hw_description.h"
/* Timer allocation
 *
 * TIM1_CH1  T FMU_CH1
 * TIM1_CH2  T FMU_CH2
 * TIM1_CH3  T FMU_CH3
 * TIM1_CH4  T FMU_CH4
 *
 * TIM2_CH3  T FMU_CH5
 * TIM2_CH4  T FMU_CH6
 *
 * TIM3_CH2 T FMU_CH7
 * TIM3_CH4 T FMU_CH8
 */

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	// initIOTimer(Timer::Timer1),
	// initIOTimer(Timer::Timer2),
	// initIOTimer(Timer::Timer3),
	initIOTimer(Timer::Timer1, DMA{DMA::Index1}),
	initIOTimer(Timer::Timer2, DMA{DMA::Index1}),
	initIOTimer(Timer::Timer3, DMA{DMA::Index1}),
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel1}, {GPIO::PortA, GPIO::Pin8}),
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel2}, {GPIO::PortE, GPIO::Pin11}),
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel3}, {GPIO::PortE, GPIO::Pin13}),
	initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel4}, {GPIO::PortE, GPIO::Pin14}),
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel1}, {GPIO::PortA, GPIO::Pin5}),
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel3}, {GPIO::PortA, GPIO::Pin2}),
	initIOTimerChannel(io_timers, {Timer::Timer2, Timer::Channel4}, {GPIO::PortA, GPIO::Pin3}),
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel1}, {GPIO::PortA, GPIO::Pin6}),
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel2}, {GPIO::PortC, GPIO::Pin7}),
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel4}, {GPIO::PortB, GPIO::Pin1}),
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);
