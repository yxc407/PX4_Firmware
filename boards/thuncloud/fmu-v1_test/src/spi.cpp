/****************************************************************************
 *
 *   Copyright (C) 2020 TC Development Team. All rights reserved.
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

//#include <tc_arch/spi_hw_description.h>
#include "/home/thuncloud/TC-Autopilot/platforms/nuttx/src/tc/stm/stm32h7/include/tc_arch/spi_hw_description.h"
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

constexpr tc_spi_bus_t tc_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(SPI::Bus::SPI1, {
		initSPIDevice(DRV_IMU_DEVTYPE_MPU9250, SPI::CS{GPIO::PortE, GPIO::Pin3}, SPI::DRDY{GPIO::PortE, GPIO::Pin4}),	//CS1 DRDY1
		// initSPIDevice(DRV_IMU_DEVTYPE_ICM20948, SPI::CS{GPIO::PortE, GPIO::Pin3}, SPI::DRDY{GPIO::PortE, GPIO::Pin4}),	//CS1 DRDY1
	}),
	initSPIBus(SPI::Bus::SPI2, {
		// initSPIDevice(DRV_IMU_DEVTYPE_MPU9250, SPI::CS{GPIO::PortD, GPIO::Pin10}, SPI::DRDY{GPIO::PortD, GPIO::Pin11}),	//CS2 DRDY2
		initSPIDevice(DRV_IMU_DEVTYPE_ICM20948, SPI::CS{GPIO::PortD, GPIO::Pin10}, SPI::DRDY{GPIO::PortD, GPIO::Pin11}),	//CS2 DRDY2
		// initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortG, GPIO::Pin7})
	}),
	initSPIBus(SPI::Bus::SPI3, {
		initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortD, GPIO::Pin7})
	}),
	initSPIBus(SPI::Bus::SPI5, {
		initSPIDevice(DRV_IMU_DEVTYPE_MPU6500, SPI::CS{GPIO::PortF, GPIO::Pin6}, SPI::DRDY{GPIO::PortF, GPIO::Pin2}),	//CS5 DRDY5
	}),
	// initSPIBus(SPI::Bus::SPI4, {
	// 	initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortG, GPIO::Pin7})
	// }),
	// initSPIBusExternal(SPI::Bus::SPI6, {
	// 	initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin10}, SPI::DRDY{GPIO::PortD, GPIO::Pin11}),
	// 	initSPIConfigExternal(SPI::CS{GPIO::PortA, GPIO::Pin15}, SPI::DRDY{GPIO::PortD, GPIO::Pin12}),
	// }),
};

static constexpr bool unused = validateSPIConfig(tc_spi_buses);
