/****************************************************************************
 *
 *   Copyright (c) 2012-2017 TC Development Team. All rights reserved.
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
 * @file registers.c
 *
 * Implementation of the TCIO register space.
 *
 * @author Lorenz Meier <lorenz@tc.io>
 */

#include <tc_platform_common/tc_config.h>

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <stm32_pwr.h>
#include <rc/dsm.h>
#include <rc/sbus.h>

#include "tcio.h"
#include "protocol.h"

static int	registers_set_one(uint8_t page, uint8_t offset, uint16_t value);
static void	pwm_configure_rates(uint16_t map, uint16_t defaultrate, uint16_t altrate);

/**
 * PAGE 0
 *
 * Static configuration parameters.
 */
static const uint16_t	r_page_config[] = {
	[TCIO_P_CONFIG_PROTOCOL_VERSION]	= TCIO_PROTOCOL_VERSION,
	[TCIO_P_CONFIG_HARDWARE_VERSION]	= 2,
	[TCIO_P_CONFIG_BOOTLOADER_VERSION]	= TCIO_BL_VERSION,
	[TCIO_P_CONFIG_MAX_TRANSFER]		= TCIO_MAX_TRANSFER_LEN,
	[TCIO_P_CONFIG_CONTROL_COUNT]		= TCIO_CONTROL_CHANNELS,
	[TCIO_P_CONFIG_ACTUATOR_COUNT]		= TCIO_SERVO_COUNT,
	[TCIO_P_CONFIG_RC_INPUT_COUNT]		= TCIO_RC_INPUT_CHANNELS,
	[TCIO_P_CONFIG_ADC_INPUT_COUNT]	= TCIO_ADC_CHANNEL_COUNT,
};

/**
 * PAGE 1
 *
 * Status values.
 */
volatile uint16_t	r_page_status[] = {
	[TCIO_P_STATUS_FREEMEM]		= 0,
	[TCIO_P_STATUS_CPULOAD]		= 0,
	[TCIO_P_STATUS_FLAGS]			= 0,
	[TCIO_P_STATUS_ALARMS]			= 0,
	[TCIO_P_STATUS_VSERVO]			= 0,
	[TCIO_P_STATUS_VRSSI]			= 0,
};

/**
 * PAGE 3
 *
 * Servo PWM values
 */
uint16_t		r_page_servos[TCIO_SERVO_COUNT];

/**
 * PAGE 4
 *
 * Raw RC input
 */
uint16_t		r_page_raw_rc_input[] = {
	[TCIO_P_RAW_RC_COUNT]			= 0,
	[TCIO_P_RAW_RC_FLAGS]			= 0,
	[TCIO_P_RAW_RC_NRSSI]			= 0,
	[TCIO_P_RAW_RC_DATA]			= 0,
	[TCIO_P_RAW_FRAME_COUNT]		= 0,
	[TCIO_P_RAW_LOST_FRAME_COUNT]		= 0,
	[TCIO_P_RAW_RC_BASE ...(TCIO_P_RAW_RC_BASE + TCIO_RC_INPUT_CHANNELS)] = 0
};

/**
 * Scratch page; used for registers that are constructed as-read.
 *
 * PAGE 6 Raw ADC input.
 * PAGE 7 PWM rate maps.
 */
uint16_t		r_page_scratch[32];

/**
 * PAGE 8
 *
 * RAW PWM values
 */
uint16_t		r_page_direct_pwm[TCIO_SERVO_COUNT];

/**
 * PAGE 100
 *
 * Setup registers
 */
volatile uint16_t	r_page_setup[] = {
	/* default to RSSI ADC functionality */
	[TCIO_P_SETUP_FEATURES]		= TCIO_P_SETUP_FEATURES_ADC_RSSI,
	[TCIO_P_SETUP_ARMING]			= 0,
	[TCIO_P_SETUP_PWM_RATES]		= 0,
	[TCIO_P_SETUP_PWM_DEFAULTRATE]		= 50,
	[TCIO_P_SETUP_PWM_ALTRATE]		= 200,
	[TCIO_P_SETUP_SBUS_RATE]		= 72,
	[TCIO_P_SETUP_VSERVO_SCALE]		= 10000,
	[TCIO_P_SETUP_SET_DEBUG]		= 0,
	[TCIO_P_SETUP_REBOOT_BL]		= 0,
	[TCIO_P_SETUP_CRC ...(TCIO_P_SETUP_CRC + 1)] = 0,
	[TCIO_P_SETUP_THERMAL] = TCIO_THERMAL_IGNORE,
	[TCIO_P_SETUP_ENABLE_FLIGHTTERMINATION] = 0,
	[TCIO_P_SETUP_PWM_RATE_GROUP0 ... TCIO_P_SETUP_PWM_RATE_GROUP3] = 0
};

#define TCIO_P_SETUP_FEATURES_VALID	(TCIO_P_SETUP_FEATURES_SBUS1_OUT | TCIO_P_SETUP_FEATURES_SBUS2_OUT | TCIO_P_SETUP_FEATURES_ADC_RSSI)

#define TCIO_P_SETUP_ARMING_VALID	(TCIO_P_SETUP_ARMING_FMU_ARMED | \
		TCIO_P_SETUP_ARMING_FMU_PREARMED | \
		TCIO_P_SETUP_ARMING_IO_ARM_OK | \
		TCIO_P_SETUP_ARMING_FAILSAFE_CUSTOM | \
		TCIO_P_SETUP_ARMING_LOCKDOWN | \
		TCIO_P_SETUP_ARMING_FORCE_FAILSAFE | \
		TCIO_P_SETUP_ARMING_TERMINATION_FAILSAFE)
#define TCIO_P_SETUP_RATES_VALID	((1 << TCIO_SERVO_COUNT) - 1)

/*
 * PAGE 104 uses r_page_servos.
 */

/**
 * PAGE 105
 *
 * Failsafe servo PWM values
 *
 * Disable pulses as default.
 */
uint16_t		r_page_servo_failsafe[TCIO_SERVO_COUNT] = { 0, 0, 0, 0, 0, 0, 0, 0 };

/**
 * PAGE 109
 *
 * disarmed PWM values for difficult ESCs
 *
 */
uint16_t		r_page_servo_disarmed[TCIO_SERVO_COUNT] = { 0, 0, 0, 0, 0, 0, 0, 0 };

int
registers_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values)
{
	switch (page) {
	/* handle raw PWM input */
	case TCIO_PAGE_DIRECT_PWM:

		/* copy channel data */
		while ((offset < TCIO_CONTROL_CHANNELS) && (num_values > 0)) {

			/* XXX range-check value? */
			if (*values != PWM_IGNORE_THIS_CHANNEL) {
				r_page_direct_pwm[offset] = *values;
			}

			offset++;
			num_values--;
			values++;
		}

		system_state.fmu_data_received_time = hrt_absolute_time();
		r_status_flags |= TCIO_P_STATUS_FLAGS_RAW_PWM;

		/* Trigger all timer's channels in Oneshot mode to fire
		 * the oneshots with updated values.
		 */
		up_pwm_update(0xff);

		break;

	/* handle setup for servo failsafe values */
	case TCIO_PAGE_FAILSAFE_PWM:

		/* copy channel data */
		while ((offset < TCIO_SERVO_COUNT) && (num_values > 0)) {

			if (*values == 0) {
				/* ignore 0 */
			} else if (*values < PWM_LOWEST_MIN) {
				r_page_servo_failsafe[offset] = PWM_LOWEST_MIN;

			} else if (*values > PWM_HIGHEST_MAX) {
				r_page_servo_failsafe[offset] = PWM_HIGHEST_MAX;

			} else {
				r_page_servo_failsafe[offset] = *values;
			}

			/* flag the failsafe values as custom */
			r_setup_arming |= TCIO_P_SETUP_ARMING_FAILSAFE_CUSTOM;

			offset++;
			num_values--;
			values++;
		}

		break;

	case TCIO_PAGE_DISARMED_PWM: {
			/* copy channel data */
			while ((offset < TCIO_SERVO_COUNT) && (num_values > 0)) {
				if (*values == 0) {
					/* 0 means disabling always PWM */
					r_page_servo_disarmed[offset] = 0;

				} else if (*values < PWM_LOWEST_MIN) {
					r_page_servo_disarmed[offset] = PWM_LOWEST_MIN;

				} else if (*values > PWM_HIGHEST_MAX) {
					r_page_servo_disarmed[offset] = PWM_HIGHEST_MAX;

				} else {
					r_page_servo_disarmed[offset] = *values;
				}

				offset++;
				num_values--;
				values++;
			}

			r_status_flags |= TCIO_P_STATUS_FLAGS_INIT_OK;
		}
		break;

	default:

		/* avoid offset wrap */
		if ((offset + num_values) > 255) {
			num_values = 255 - offset;
		}

		/* iterate individual registers, set each in turn */
		while (num_values--) {
			if (registers_set_one(page, offset, *values)) {
				return -1;
			}

			offset++;
			values++;
		}

		break;
	}

	return 0;
}

static int
registers_set_one(uint8_t page, uint8_t offset, uint16_t value)
{
	switch (page) {

	case TCIO_PAGE_STATUS:
		switch (offset) {
		case TCIO_P_STATUS_ALARMS:
			/* clear bits being written */
			r_status_alarms &= ~value;
			break;

		case TCIO_P_STATUS_FLAGS:

			/*
			 * Allow FMU override of arming state (to allow in-air restores),
			 * but only if the arming state is not in sync on the IO side.
			 */
			if (!(r_status_flags & TCIO_P_STATUS_FLAGS_ARM_SYNC)) {
				r_status_flags = value;

			}

			break;

		default:
			/* just ignore writes to other registers in this page */
			break;
		}

		break;

	case TCIO_PAGE_SETUP:
		switch (offset) {
		case TCIO_P_SETUP_FEATURES:

			value &= TCIO_P_SETUP_FEATURES_VALID;

			/* some of the options conflict - give S.BUS out precedence, then ADC RSSI, then PWM RSSI */

			/* switch S.Bus output pin as needed */
#ifdef ENABLE_SBUS_OUT
			ENABLE_SBUS_OUT(value & (TCIO_P_SETUP_FEATURES_SBUS1_OUT | TCIO_P_SETUP_FEATURES_SBUS2_OUT));

			/* disable the conflicting options with SBUS 1 */
			if (value & (TCIO_P_SETUP_FEATURES_SBUS1_OUT)) {
				value &= ~(TCIO_P_SETUP_FEATURES_ADC_RSSI | TCIO_P_SETUP_FEATURES_SBUS2_OUT);
			}

			/* disable the conflicting options with SBUS 2 */
			if (value & (TCIO_P_SETUP_FEATURES_SBUS2_OUT)) {
				value &= ~(TCIO_P_SETUP_FEATURES_ADC_RSSI | TCIO_P_SETUP_FEATURES_SBUS1_OUT);
			}

#endif

			/* disable the conflicting options with ADC RSSI */
			if (value & (TCIO_P_SETUP_FEATURES_ADC_RSSI)) {
				value &= ~(TCIO_P_SETUP_FEATURES_SBUS1_OUT | TCIO_P_SETUP_FEATURES_SBUS2_OUT);
			}

			/* apply changes */
			r_setup_features = value;

			break;

		case TCIO_P_SETUP_ARMING:

			value &= TCIO_P_SETUP_ARMING_VALID;

			/*
			 * If the failsafe termination flag is set, do not allow the autopilot to unset it
			 */
			value |= (r_setup_arming & TCIO_P_SETUP_ARMING_TERMINATION_FAILSAFE);

			/*
			 * If failsafe termination is enabled and force failsafe bit is set, do not allow
			 * the autopilot to clear it.
			 */
			if (r_setup_arming & TCIO_P_SETUP_ARMING_TERMINATION_FAILSAFE) {
				value |= (r_setup_arming & TCIO_P_SETUP_ARMING_FORCE_FAILSAFE);
			}

			r_setup_arming = value;

			break;

		case TCIO_P_SETUP_PWM_RATES:
			value &= TCIO_P_SETUP_RATES_VALID;
			pwm_configure_rates(value, r_setup_pwm_defaultrate, r_setup_pwm_altrate);
			break;

		case TCIO_P_SETUP_PWM_DEFAULTRATE:
			if (value < 25) {
				value = 25;
			}

			if (value > 400) {
				value = 400;
			}

			pwm_configure_rates(r_setup_pwm_rates, value, r_setup_pwm_altrate);
			break;

		case TCIO_P_SETUP_PWM_ALTRATE:

			/* For PWM constrain to [25,400]Hz
			 * For Oneshot there is no rate, 0 is therefore used to select Oneshot mode
			 */
			if (value != 0) {
				if (value < 25) {
					value = 25;
				}

				if (value > 400) {
					value = 400;
				}
			}

			pwm_configure_rates(r_setup_pwm_rates, r_setup_pwm_defaultrate, value);
			break;

		case TCIO_P_SETUP_SET_DEBUG:
			r_page_setup[TCIO_P_SETUP_SET_DEBUG] = value;
			isr_debug(0, "set debug %u\n", (unsigned)r_page_setup[TCIO_P_SETUP_SET_DEBUG]);
			break;

		case TCIO_P_SETUP_REBOOT_BL:

			// check the magic value
			if (value != TCIO_REBOOT_BL_MAGIC) {
				break;
			}

			// we schedule a reboot rather than rebooting
			// immediately to allow the IO board to ACK
			// the reboot command
			schedule_reboot(100000);
			break;

		case TCIO_P_SETUP_DSM:
			dsm_bind(value & 0x0f, (value >> 4) & 0xF);
			break;

		case TCIO_P_SETUP_SAFETY_BUTTON_ACK:
			// clear safety button pressed flag so it can be used again
			r_status_flags &= ~TCIO_P_STATUS_FLAGS_SAFETY_BUTTON_EVENT;

			break;

		case TCIO_P_SETUP_SAFETY_OFF:

			if (value) {
				r_status_flags |= TCIO_P_STATUS_FLAGS_SAFETY_OFF;

			} else {
				r_status_flags &= ~TCIO_P_STATUS_FLAGS_SAFETY_OFF;
			}

			break;

		case TCIO_P_SETUP_SBUS_RATE:
			r_page_setup[offset] = value;
			sbus1_set_output_rate_hz(value);
			break;

		case TCIO_P_SETUP_THERMAL:
		case TCIO_P_SETUP_ENABLE_FLIGHTTERMINATION:
			r_page_setup[offset] = value;
			break;

		case TCIO_P_SETUP_PWM_RATE_GROUP0:
		case TCIO_P_SETUP_PWM_RATE_GROUP1:
		case TCIO_P_SETUP_PWM_RATE_GROUP2:
		case TCIO_P_SETUP_PWM_RATE_GROUP3:

			/* For PWM constrain to [25,400]Hz
			 * For Oneshot there is no rate, 0 is therefore used to select Oneshot mode
			 */
			if (value != 0) {
				if (value < 25) {
					value = 25;
				}

				if (value > 400) {
					value = 400;
				}
			}

			if (up_pwm_servo_set_rate_group_update(offset - TCIO_P_SETUP_PWM_RATE_GROUP0, value) == OK) {
				r_page_setup[offset] = value;

			} else {
				r_status_alarms |= TCIO_P_STATUS_ALARMS_PWM_ERROR;
			}

			break;

		default:
			return -1;
		}

		break;

	case TCIO_PAGE_TEST:
		switch (offset) {
		case TCIO_P_TEST_LED:
			LED_AMBER(value & 1);
			break;
		}

		break;

	default:
		return -1;
	}

	return 0;
}

uint8_t last_page;
uint8_t last_offset;

int
registers_get(uint8_t page, uint8_t offset, uint16_t **values, unsigned *num_values)
{
#define SELECT_PAGE(_page_name)							\
	do {									\
		*values = (uint16_t *)&_page_name[0];				\
		*num_values = sizeof(_page_name) / sizeof(_page_name[0]);	\
	} while(0)

	switch (page) {

	/*
	 * Handle pages that are updated dynamically at read time.
	 */
	case TCIO_PAGE_STATUS:
		/* TCIO_P_STATUS_FREEMEM */

		/* XXX TCIO_P_STATUS_CPULOAD */

		/* TCIO_P_STATUS_FLAGS maintained externally */

		/* TCIO_P_STATUS_ALARMS maintained externally */

#ifdef ADC_VSERVO
		/* TCIO_P_STATUS_VSERVO */
		{
			unsigned counts = adc_measure(ADC_VSERVO);

			if (counts != 0xffff) {
				// use 3:1 scaling on 3.3V ADC input
				unsigned mV = counts * 9900 / 4096;
				r_page_status[TCIO_P_STATUS_VSERVO] = mV;
			}
		}

#endif
#ifdef ADC_RSSI
		/* TCIO_P_STATUS_VRSSI */
		{
			unsigned counts = adc_measure(ADC_RSSI);

			if (counts != 0xffff) {
				// use 1:1 scaling on 3.3V ADC input
				unsigned mV = counts * 3300 / 4096;
				r_page_status[TCIO_P_STATUS_VRSSI] = mV;
			}
		}
#endif
		/* XXX TCIO_P_STATUS_PRSSI */

		SELECT_PAGE(r_page_status);
		break;

	case TCIO_PAGE_RAW_ADC_INPUT:
		memset(r_page_scratch, 0, sizeof(r_page_scratch));

#ifdef ADC_VSERVO
		r_page_scratch[0] = adc_measure(ADC_VSERVO);
#endif
#ifdef ADC_RSSI
		r_page_scratch[1] = adc_measure(ADC_RSSI);
#endif
		SELECT_PAGE(r_page_scratch);
		break;

	case TCIO_PAGE_PWM_INFO:
		memset(r_page_scratch, 0, sizeof(r_page_scratch));

		for (unsigned i = 0; i < TCIO_SERVO_COUNT; i++) {
			r_page_scratch[TCIO_RATE_MAP_BASE + i] = up_pwm_servo_get_rate_group(i);
		}

		SELECT_PAGE(r_page_scratch);
		break;

	/*
	 * Pages that are just a straight read of the register state.
	 */

	/* status pages */
	case TCIO_PAGE_CONFIG:
		SELECT_PAGE(r_page_config);
		break;

	case TCIO_PAGE_SERVOS:
		SELECT_PAGE(r_page_servos);
		break;

	case TCIO_PAGE_RAW_RC_INPUT:
		SELECT_PAGE(r_page_raw_rc_input);
		break;

	/* readback of input pages */
	case TCIO_PAGE_SETUP:
		SELECT_PAGE(r_page_setup);
		break;

	case TCIO_PAGE_DIRECT_PWM:
		SELECT_PAGE(r_page_direct_pwm);
		break;

	case TCIO_PAGE_FAILSAFE_PWM:
		SELECT_PAGE(r_page_servo_failsafe);
		break;

	case TCIO_PAGE_DISARMED_PWM:
		SELECT_PAGE(r_page_servo_disarmed);
		break;

	default:
		return -1;
	}

#undef SELECT_PAGE
#undef COPY_PAGE

	last_page = page;
	last_offset = offset;

	/* if the offset is at or beyond the end of the page, we have no data */
	if (offset >= *num_values) {
		return -1;
	}

	/* correct the data pointer and count for the offset */
	*values += offset;
	*num_values -= offset;

	return 0;
}

/*
 * Helper function to handle changes to the PWM rate control registers.
 */
static void
pwm_configure_rates(uint16_t map, uint16_t defaultrate, uint16_t altrate)
{
	for (unsigned pass = 0; pass < 2; pass++) {
		for (unsigned group = 0; group < TCIO_SERVO_COUNT; group++) {

			/* get the channel mask for this rate group */
			uint32_t mask = up_pwm_servo_get_rate_group(group);

			if (mask == 0) {
				continue;
			}

			/* all channels in the group must be either default or alt-rate */
			uint32_t alt = map & mask;

			if (pass == 0) {
				/* preflight */
				if ((alt != 0) && (alt != mask)) {
					/* not a legal map, bail with an alarm */
					r_status_alarms |= TCIO_P_STATUS_ALARMS_PWM_ERROR;
					return;
				}

			} else {
				/* set it - errors here are unexpected */
				if (alt != 0) {
					if (up_pwm_servo_set_rate_group_update(group, r_setup_pwm_altrate) != OK) {
						r_status_alarms |= TCIO_P_STATUS_ALARMS_PWM_ERROR;
					}

				} else {
					if (up_pwm_servo_set_rate_group_update(group, r_setup_pwm_defaultrate) != OK) {
						r_status_alarms |= TCIO_P_STATUS_ALARMS_PWM_ERROR;
					}
				}
			}
		}
	}

	r_setup_pwm_rates = map;
	r_setup_pwm_defaultrate = defaultrate;
	r_setup_pwm_altrate = altrate;
}
