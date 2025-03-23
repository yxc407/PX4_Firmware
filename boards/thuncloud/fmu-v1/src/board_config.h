/****************************************************************************
 *
 *   Copyright (c) 2016, 2020 TC Development Team. All rights reserved.
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
 * @file board_config.h
 *
 * TCFMU-v6u internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <tc_platform_common/tc_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* Configuration ************************************************************************************/

// #define BOARD_HAS_LTC44XX_VALIDS      2 //  N Bricks
#define BOARD_HAS_USB_VALID           1 // LTC Has USB valid
#define BOARD_HAS_NBAT_V              2d // 2 Digital Voltage
#define BOARD_HAS_NBAT_I              2d // 2 Digital Current

/* TCFMU GPIOs ***********************************************************************************/

/* Trace Clock and D0-D3 are available on the trace connector
 *
 * TRACECLK PE2  - Dedicated       - Trace Connector Pin 1
 * TRACED0  PE3  - nLED_RED        - Trace Connector Pin 3
 * TRACED1  PE4  - nLED_GREEN      - Trace Connector Pin 5
 * TRACED2  PE5  - nLED_BLUE       - Trace Connector Pin 7
 * TRACED3  PC12 - UART5_TX_TELEM2 - Trace Connector Pin 8

 */
#undef TRACE_PINS

/* LEDs are driven with push open drain to support Anode to 5V or 3.3V or used as TRACE0-2 */

#if !defined(TRACE_PINS)
#  define GPIO_nLED_RED        /* PC13 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)
#  define GPIO_nLED_GREEN      /* PE12 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN12)
#  define GPIO_nLED_BLUE       /* PD15 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN15)

#  define BOARD_HAS_CONTROL_STATUS_LEDS      	1
#  define BOARD_OVERLOAD_LED     		LED_RED
#  define BOARD_ARMED_STATE_LED  		LED_BLUE

#else
#  if defined(CONFIG_STM32H7_UART5) && (GPIO_UART5_TX == GPIO_UART5_TX_3)
#    error Need to disable CONFIG_STM32H7_UART5 for Trace 3 (Retarget to CAN2 if need be)
#  endif
#endif


/* SPI */


/* I2C busses */

/* Devices on the onboard buses.
 *
 * Note that these are unshifted addresses.
 */
#define TC_I2C_OBDEV_SE050         0x48

// #define GPIO_I2C4_DRDY1_BMP388      /* PG5  */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN5)

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the tc Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */

#define ADC1_CH(n)                  (n)

/* N.B. there is no offset mapping needed for ADC3 because */
#define ADC3_CH(n)                  (n)

/* We are only use ADC3 for REV/VER.
 * ADC3_6V6 and ADC3_3V3 are mapped back to ADC1
 * To do this We are relying on PC2_C, PC3_C being connected to PC2, PC3
 * respectively by the SYSCFG_PMCR default of setting for PC3SO PC2SO PA1SO
 * PA0SO of 0.
 *
 *  0 Analog switch closed (pads are connected through the analog switch)
 *
 * So ADC3_INP0 is GPIO_ADC123_INP12
 *    ADC3_INP1 is GPIO_ADC12_INP13
 */

/* Define GPIO pins used as ADC N.B. Channel numbers must match below  */

#define TC_ADC_GPIO  \
	/* PC4  */  GPIO_ADC12_INP4, \
	/* PB0  */  GPIO_ADC12_INP9, \
	/* PC0  */  GPIO_ADC123_INP10, \
	/* PC5  */  GPIO_ADC12_INP8
	// /* PC3  */  GPIO_ADC3_INP1, /

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_SCALED_VDD_3V3_SENSORS_CHANNEL      /* PC4  */  ADC1_CH(4)
#define ADC_SCALED_V5_CHANNEL                   /* PC5  */  ADC1_CH(8)
#define ADC_ADC3_6V6_CHANNEL                    /* PB0  */  ADC1_CH(9)
#define ADC_ADC3_3V3_CHANNEL                    /* PC0  */  ADC1_CH(10)
// #define ADC_HW_VER_SENSE_CHANNEL                /* PH3  */  ADC3_CH(14)
// #define ADC_HW_REV_SENSE_CHANNEL                /* PH4  */  ADC3_CH(15)
#define ADC_HW_VER_SENSE_CHANNEL                /* PC4  */  ADC1_CH(4)
#define ADC_HW_REV_SENSE_CHANNEL                /* PC4  */  ADC1_CH(4)
#define ADC_BATTERY_VOLTAGE_CHANNEL             ADC_ADC3_3V3_CHANNEL
#define ADC_BATTERY_CURRENT_CHANNEL             ADC_ADC3_6V6_CHANNEL

#define ADC_CHANNELS \
	((1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL) | \
	 (1 << ADC_SCALED_V5_CHANNEL)               | \
	 (1 << ADC_ADC3_6V6_CHANNEL)                | \
	 (1 << ADC_ADC3_3V3_CHANNEL))

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define HW_REV_VER_ADC_BASE STM32_ADC3_BASE

#define SYSTEM_ADC_BASE STM32_ADC1_BASE

/* HW has to large of R termination on ADC todo:change when HW value is chosen */
#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* HW Version and Revision drive signals Default to 1 to detect */
#define BOARD_HAS_HW_VERSIONING

#define GPIO_HW_VER_REV_DRIVE  /* PD14 */ 	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN14)
#define GPIO_HW_REV_SENSE      			ADC_HW_REV_SENSE_CHANNEL
#define GPIO_HW_VER_SENSE      			ADC_HW_VER_SENSE_CHANNEL

#define HW_INFO_INIT_PREFIX   "V6U"
#define V6U00                 HW_VER_REV(0x0,0x0)

// /* PE6 is nARMED
//  *  The GPIO will be set as input while not armed HW will have external HW Pull UP.
//  *  While armed it shall be configured at a GPIO OUT set LOW
//  */
#define GPIO_nARMED_INIT     /* PA4 */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN4)
#define GPIO_nARMED          /* PA4 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN4)

#define BOARD_INDICATE_ARMED_STATE(on_armed)  tc_arch_configgpio((on_armed) ? GPIO_nARMED : GPIO_nARMED_INIT)

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   10 // 8


/* Power supply control and monitoring GPIOs */

#define GPIO_nPOWER_IN_A                /* PE2  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN2)
#define GPIO_nPOWER_IN_B                /* PE6  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN6)
// new edit
#define GPIO_nPOWER_IN_C                /* PG3  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN3)

#define GPIO_nVDD_BRICK1_VALID          GPIO_nPOWER_IN_A /* Brick 1 Is Chosen */
#define GPIO_nVDD_BRICK2_VALID          GPIO_nPOWER_IN_B /* Brick 2 Is Chosen  */
// new edit
#define GPIO_nVDD_BRICK3_VALID          GPIO_nPOWER_IN_C
#define BOARD_NUMBER_BRICKS             1
#define BOARD_ADC_BRICK_VALID           1

// #define BOARD_NUMBER_DIGITAL_BRICKS     2
// #define GPIO_nVDD_USB_VALID             GPIO_nPOWER_IN_C /* USB     Is Chosen */

// #define GPIO_VDD_5V_PERIPH_nEN          /* PA4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)
#define GPIO_VDD_5V_PERIPH_nEN          /* PE10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN1)
#define GPIO_VDD_5V_PERIPH_nOC          /* PE15 */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTE|GPIO_PIN15)
#define GPIO_VDD_5V_HIPOWER_nEN         /* PE0 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN0)
#define GPIO_VDD_5V_HIPOWER_nOC         /* PE1 */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTE|GPIO_PIN1)
#define GPIO_VDD_3V3_SENSORS_EN         /* PE7  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN7)
#define GPIO_VDD_3V3_SPEKTRUM_POWER_EN  /* PE8  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN8)
#define GPIO_VDD_3V3_SD_CARD_EN         /* PE9 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN9)

/* Spare GPIO */

// #define GPIO_PG6                        /* PG6  */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN6)
// #define GPIO_PG15                       /* PG15 */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN15)


/* Define True logic Power Control in arch agnostic form */

#define VDD_5V_PERIPH_EN(on_true)          tc_arch_gpiowrite(GPIO_VDD_5V_PERIPH_nEN, !(on_true))
#define VDD_5V_HIPOWER_EN(on_true)         tc_arch_gpiowrite(GPIO_VDD_5V_HIPOWER_nEN, !(on_true))
#define VDD_3V3_SENSORS_EN(on_true)        tc_arch_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, (on_true))
#define VDD_3V3_SPEKTRUM_POWER_EN(on_true) tc_arch_gpiowrite(GPIO_VDD_3V3_SPEKTRUM_POWER_EN, (on_true))
#define READ_VDD_3V3_SPEKTRUM_POWER_EN()   tc_arch_gpioread(GPIO_VDD_3V3_SPEKTRUM_POWER_EN)
#define VDD_3V3_SD_CARD_EN(on_true)        tc_arch_gpiowrite(GPIO_VDD_3V3_SD_CARD_EN, (on_true))


/* Tone alarm output */

#define TONE_ALARM_TIMER        15  /* Timer 14 */
#define TONE_ALARM_CHANNEL      1  /* PE5 GPIO_TIM15_CH1OUT_2 */

#define GPIO_BUZZER_1           /* PE5 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN5)

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         GPIO_TIM15_CH1OUT_2

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER               8  /* use timer8 for the HRT */

#define HRT_TIMER_CHANNEL       3  /* use capture/compare channel 3 */

#define HRT_PPM_CHANNEL         /* T8C1 */  1  /* use capture/compare channel 1 */
#define GPIO_PPM_IN             /* PC6 T8C1 */ GPIO_TIM8_CH1IN_1

/* RC Serial port */

#define RC_SERIAL_PORT                     "/dev/ttyS4"
#define RC_SERIAL_SINGLEWIRE
#define RC_SERIAL_SWAP_RXTX

// /* Input Capture Channels. */
// #define INPUT_CAP1_TIMER                  1
// #define INPUT_CAP1_CHANNEL     /* T1C2 */ 2
// #define GPIO_INPUT_CAP1        /*  PE11 */ GPIO_TIM1_CH2IN

// /* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
// #define PWMIN_TIMER                       4
// #define PWMIN_TIMER_CHANNEL    /* T4C2 */ 2
// #define GPIO_PWM_IN            /* PD13 */ GPIO_TIM4_CH2IN_2

// /* Power switch controls ******************************************************/

#define SPEKTRUM_POWER(_on_true)           VDD_3V3_SPEKTRUM_POWER_EN(_on_true)

#define GPIO_PPM_IN_AS_OUT             (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN6)
#define SPEKTRUM_RX_AS_GPIO_OUTPUT()   tc_arch_configgpio(GPIO_PPM_IN_AS_OUT)
#define SPEKTRUM_RX_AS_UART()          
#define SPEKTRUM_OUT(_one_true)        tc_arch_gpiowrite(GPIO_PPM_IN_AS_OUT, (_one_true))

#define SDIO_SLOTNO                    0
#define SDIO_MINOR                     0

#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_BOARDCTL) && \
   !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

#define BOARD_ADC_USB_CONNECTED (tc_arch_gpioread(GPIO_OTGFS_VBUS))

#define BOARD_ADC_SERVO_VALID     (1)

#if !defined(BOARD_HAS_LTC44XX_VALIDS) || BOARD_HAS_LTC44XX_VALIDS == 0
#  define BOARD_ADC_BRICK1_VALID  (1)
#  define BOARD_ADC_BRICK2_VALID  (0)
#elif BOARD_HAS_LTC44XX_VALIDS == 1
#  define BOARD_ADC_BRICK1_VALID  (!tc_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (0)
#elif BOARD_HAS_LTC44XX_VALIDS == 2
#  define BOARD_ADC_BRICK1_VALID  (!tc_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (!tc_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#elif BOARD_HAS_LTC44XX_VALIDS == 3
#  define BOARD_ADC_BRICK1_VALID  (!tc_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (!tc_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#  define BOARD_ADC_BRICK3_VALID  (!tc_arch_gpioread(GPIO_nVDD_BRICK3_VALID))
#elif BOARD_HAS_LTC44XX_VALIDS == 4
#  define BOARD_ADC_BRICK1_VALID  (!tc_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (!tc_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#  define BOARD_ADC_BRICK3_VALID  (!tc_arch_gpioread(GPIO_nVDD_BRICK3_VALID))
#  define BOARD_ADC_BRICK4_VALID  (!tc_arch_gpioread(GPIO_nVDD_BRICK4_VALID))
#else
#  error Unsupported BOARD_HAS_LTC44XX_VALIDS value
#endif

#define BOARD_ADC_PERIPH_5V_OC  (!tc_arch_gpioread(GPIO_VDD_5V_PERIPH_nOC))
#define BOARD_ADC_HIPOWER_5V_OC (!tc_arch_gpioread(GPIO_VDD_5V_HIPOWER_nOC))

#define BOARD_DMA_ALLOC_POOL_SIZE 5120

#define BOARD_HAS_ON_RESET 1
#define TC_GPIO_INIT_LIST {               \
		TC_ADC_GPIO,                      \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN2_TX,                     \
		GPIO_CAN2_RX,                     \
		GPIO_VDD_5V_PERIPH_nEN,           \
		GPIO_VDD_5V_PERIPH_nEN,           \
		GPIO_VDD_5V_HIPOWER_nEN,          \
		GPIO_VDD_3V3_SENSORS_EN,          \
		GPIO_VDD_3V3_SPEKTRUM_POWER_EN,   \
		GPIO_VDD_3V3_SD_CARD_EN		      \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define TC_I2C_BUS_MTD      4,5
// #define BOARD_OVERRIDE_I2C_DEVICE_EXTERNAL

// #define BOARD_NUM_IO_TIMERS 5
#define BOARD_NUM_IO_TIMERS 3

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int stm32_sdio_initialize(void);

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the TCFMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <tc_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
