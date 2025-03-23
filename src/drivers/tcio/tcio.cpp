/****************************************************************************
 *
 *   Copyright (c) 2012-2022 TC Development Team. All rights reserved.
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
 * @file tcio.cpp
 * Driver for the TCIO board.
 *
 * TCIO is connected via DMA enabled high-speed UART.
 */
#include <tc_platform_common/defines.h>
#include <tc_platform_common/events.h>
#include <tc_platform_common/module.h>
#include <tc_platform_common/module_params.h>
#include <tc_platform_common/posix.h>
#include <tc_platform_common/tc_work_queue/ScheduledWorkItem.hpp>
#include <tc_platform_common/sem.hpp>

#include <crc32.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_io_heater.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_sbus.h>

#include <lib/circuit_breaker/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <lib/rc/dsm.h>
#include <lib/systemlib/mavlink_log.h>
#include <lib/button/ButtonPublisher.hpp>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/tcio_status.h>
#include <uORB/topics/parameter_update.h>

#include <debug.h>

#include <modules/tciofirmware/protocol.h>

#include "uploader.h"

#include "modules/dataman/dataman.h"

#include "tcio_driver.h"

#define TCIO_SET_DEBUG			_IOC(0xff00, 0)
#define TCIO_REBOOT_BOOTLOADER		_IOC(0xff00, 1)
#define TCIO_CHECK_CRC			_IOC(0xff00, 2)

static constexpr unsigned MIN_TOPIC_UPDATE_INTERVAL = 2500; // 2.5 ms -> 400 Hz

using namespace time_literals;

/**
 * The TCIO class.
 *
 * Encapsulates TCFMU to TCIO communications modeled as file operations.
 */
class TCIO : public cdev::CDev, public ModuleBase<TCIO>, public OutputModuleInterface
{
public:
	/**
	 * Constructor.
	 *
	 * Initialize all class variables.
	 */
	TCIO() = delete;
	explicit TCIO(device::Device *interface);

	~TCIO() override;

	/**
	 * Initialize the TCIO class.
	 *
	 * Retrieve relevant initial system parameters. Initialize TCIO registers.
	 */
	int		init() override;

	/**
	 * IO Control handler.
	 *
	 * Handle all IOCTL calls to the TCIO file descriptor.
	 *
	 * @param[in] filp file handle (not used). This function is always called directly through object reference
	 * @param[in] cmd the IOCTL command
	 * @param[in] the IOCTL command parameter (optional)
	 */
	int		ioctl(file *filp, int cmd, unsigned long arg) override;

	/**
	 * Print IO status.
	 *
	 * Print all relevant IO status information
	 *
	 */
	int			print_status();

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

	static int task_spawn(int argc, char *argv[]);

	/*
	 * To test what happens if IO stops receiving updates from FMU.
	 *
	 * @param is_fail	true for failure condition, false for normal operation.
	 */
	void			test_fmu_fail(bool is_fail) { _test_fmu_fail = is_fail; };

	uint16_t		system_status() const { return _status; }

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
			   unsigned num_control_groups_updated) override;

private:
	void Run() override;

	void updateDisarmed();
	void updateFailsafe();
	void updateTimerRateGroups();

	static int checkcrc(int argc, char *argv[]);
	static int bind(int argc, char *argv[]);

	static constexpr int TCIO_MAX_ACTUATORS = 8;

	device::Device *const _interface;

	unsigned		_hardware{0};		///< Hardware revision
	unsigned		_max_actuators{0};		///< Maximum # of actuators supported by TCIO
	unsigned		_max_controls{0};		///< Maximum # of controls supported by TCIO
	unsigned		_max_rc_input{0};		///< Maximum receiver channels supported by TCIO
	unsigned		_max_transfer{16};		///< Maximum number of I2C transfers supported by TCIO

	bool			_first_update_cycle{true};
	uint32_t    		_group_channels[TCIO_P_SETUP_PWM_RATE_GROUP3 - TCIO_P_SETUP_PWM_RATE_GROUP0 + 1] {};

	hrt_abstime		_poll_last{0};

	orb_advert_t		_mavlink_log_pub{nullptr};	///< mavlink log pub

	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
	perf_counter_t	_interface_read_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": interface read")};
	perf_counter_t	_interface_write_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": interface write")};

	/* cached IO state */
	uint16_t		_status{0};		///< Various IO status flags
	uint16_t		_alarms{0};		///< Various IO alarms
	uint16_t		_setup_arming{0};	///< last arming setup state
	uint16_t		_last_written_arming_s{0};	///< the last written arming state reg
	uint16_t		_last_written_arming_c{0};	///< the last written arming state reg

	uORB::Subscription	_t_actuator_armed{ORB_ID(actuator_armed)};		///< system armed control topic
	uORB::Subscription	_t_vehicle_command{ORB_ID(vehicle_command)};	///< vehicle command topic
	uORB::Subscription	_t_vehicle_status{ORB_ID(vehicle_status)};		///< vehicle status topic

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	hrt_abstime             _last_status_publish{0};

	uint16_t 		_rc_valid_update_count{0};

	bool			_param_update_force{true};	///< force a parameter update
	bool			_timer_rates_configured{false};

	/* advertised topics */
	uORB::PublicationMulti<input_rc_s>	_input_rc_pub{ORB_ID(input_rc)};
	uORB::Publication<tcio_status_s>	_tcio_status_pub{ORB_ID(tcio_status)};

	ButtonPublisher	_button_publisher;
	bool _previous_safety_off{false};

	bool			_lockdown_override{false};	///< override the safety lockdown

	int32_t		_thermal_control{-1}; ///< thermal control state
	bool			_analog_rc_rssi_stable{false}; ///< true when analog RSSI input is stable
	float			_analog_rc_rssi_volt{-1.f}; ///< analog RSSI voltage

	bool			_test_fmu_fail{false}; ///< To test what happens if IO loses FMU
	bool			_in_test_mode{false}; ///< true if PWM_SERVO_ENTER_TEST_MODE is active

	MixingOutput _mixing_output{"PWM_MAIN", TCIO_MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, true};

	bool _pwm_min_configured{false};
	bool _pwm_max_configured{false};
	bool _pwm_fail_configured{false};
	bool _pwm_dis_configured{false};
	bool _pwm_rev_configured{false};

	/**
	 * Update IO's arming-related state
	 */
	int			io_set_arming_state();

	/**
	 * Fetch status and alarms from IO
	 *
	 * Also publishes battery voltage/current.
	 */
	int			io_get_status();

	/**
	 * Fetch RC inputs from IO.
	 *
	 * @param input_rc	Input structure to populate.
	 * @return		OK if data was returned.
	 */
	int			io_publish_raw_rc();

	/**
	 * write register(s)
	 *
	 * @param page		Register page to write to.
	 * @param offset	Register offset to start writing at.
	 * @param values	Pointer to array of values to write.
	 * @param num_values	The number of values to write.
	 * @return		OK if all values were successfully written.
	 */
	int			io_reg_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values);

	/**
	 * write a register
	 *
	 * @param page		Register page to write to.
	 * @param offset	Register offset to write to.
	 * @param value		Value to write.
	 * @return		OK if the value was written successfully.
	 */
	int			io_reg_set(uint8_t page, uint8_t offset, const uint16_t value);

	/**
	 * read register(s)
	 *
	 * @param page		Register page to read from.
	 * @param offset	Register offset to start reading from.
	 * @param values	Pointer to array where values should be stored.
	 * @param num_values	The number of values to read.
	 * @return		OK if all values were successfully read.
	 */
	int			io_reg_get(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values);

	/**
	 * read a register
	 *
	 * @param page		Register page to read from.
	 * @param offset	Register offset to start reading from.
	 * @return		Register value that was read, or _io_reg_get_error on error.
	 */
	uint32_t		io_reg_get(uint8_t page, uint8_t offset);
	static const uint32_t	_io_reg_get_error = 0x80000000;

	/**
	 * modify a register
	 *
	 * @param page		Register page to modify.
	 * @param offset	Register offset to modify.
	 * @param clearbits	Bits to clear in the register.
	 * @param setbits	Bits to set in the register.
	 */
	int			io_reg_modify(uint8_t page, uint8_t offset, uint16_t clearbits, uint16_t setbits);

	/**
	 * Handle a status update from IO.
	 *
	 * Publish IO status information if necessary.
	 *
	 * @param status	The status register
	 */
	int			io_handle_status(uint16_t status);

	/**
	 * Handle issuing dsm bind ioctl to tcio.
	 *
	 * @param dsmMode	DSM2_BIND_PULSES, DSMX_BIND_PULSES, DSMX8_BIND_PULSES
	 */
	int			dsm_bind_ioctl(int dsmMode);

	/**
	 * Respond to a vehicle command with an ACK message
	 *
	 * @param cmd		The command that was executed or denied (inbound)
	 * @param result	The command result
	 */
	void			answer_command(const vehicle_command_s &cmd, uint8_t result);

	void update_params();

	DEFINE_PARAMETERS(
		(ParamInt<tc::params::PWM_SBUS_MODE>) _param_pwm_sbus_mode,
		(ParamInt<tc::params::RC_RSSI_PWM_CHAN>) _param_rc_rssi_pwm_chan,
		(ParamInt<tc::params::RC_RSSI_PWM_MAX>) _param_rc_rssi_pwm_max,
		(ParamInt<tc::params::RC_RSSI_PWM_MIN>) _param_rc_rssi_pwm_min,
		(ParamInt<tc::params::SENS_EN_THERMAL>) _param_sens_en_themal,
		(ParamInt<tc::params::SYS_HITL>) _param_sys_hitl,
		(ParamInt<tc::params::SYS_USE_IO>) _param_sys_use_io
	)
};

#define TCIO_DEVICE_PATH	"/dev/tcio"

TCIO::TCIO(device::Device *interface) :
	CDev(TCIO_DEVICE_PATH),
	OutputModuleInterface(MODULE_NAME, tc::serial_port_to_wq(TCIO_SERIAL_DEVICE)),
	_interface(interface)
{
	_mixing_output.setLowrateSchedulingInterval(20_ms);
}

TCIO::~TCIO()
{
	delete _interface;

	/* deallocate perfs */
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
	perf_free(_interface_read_perf);
	perf_free(_interface_write_perf);
}

bool TCIO::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			  unsigned num_outputs, unsigned num_control_groups_updated)
{
	if (!_test_fmu_fail) {
		/* output to the servos */
		io_reg_set(TCIO_PAGE_DIRECT_PWM, 0, outputs, num_outputs);
	}

	return true;
}

int TCIO::init()
{
	SmartLock lock_guard(_lock);

	/* do regular cdev init */
	int ret = CDev::init();

	if (ret != OK) {
		TC_ERR("init failed %d", ret);
		return ret;
	}

	/* get some parameters */
	unsigned protocol;
	hrt_abstime start_try_time = hrt_absolute_time();

	do {
		tc_usleep(2000);
		protocol = io_reg_get(TCIO_PAGE_CONFIG, TCIO_P_CONFIG_PROTOCOL_VERSION);
	} while (protocol == _io_reg_get_error && (hrt_elapsed_time(&start_try_time) < 700U * 1000U));

	/* if the error still persists after timing out, we give up */
	if (protocol == _io_reg_get_error) {
		mavlink_log_emergency(&_mavlink_log_pub, "Failed to communicate with IO, abort.\t");
		events::send(events::ID("tcio_comm_failed"), events::Log::Emergency,
			     "Failed to communicate with IO, aborting initialization");
		return -1;
	}

	if (protocol != TCIO_PROTOCOL_VERSION) {
		mavlink_log_emergency(&_mavlink_log_pub, "IO protocol/firmware mismatch, abort.\t");
		events::send(events::ID("tcio_proto_fw_mismatch"), events::Log::Emergency,
			     "IO protocol/firmware mismatch, aborting initialization");
		return -1;
	}

	_hardware      = io_reg_get(TCIO_PAGE_CONFIG, TCIO_P_CONFIG_HARDWARE_VERSION);
	_max_actuators = io_reg_get(TCIO_PAGE_CONFIG, TCIO_P_CONFIG_ACTUATOR_COUNT);
	_max_controls  = io_reg_get(TCIO_PAGE_CONFIG, TCIO_P_CONFIG_CONTROL_COUNT);
	_max_transfer  = io_reg_get(TCIO_PAGE_CONFIG, TCIO_P_CONFIG_MAX_TRANSFER) - 2;
	_max_rc_input  = io_reg_get(TCIO_PAGE_CONFIG, TCIO_P_CONFIG_RC_INPUT_COUNT);

	if ((_max_actuators < 1) || (_max_actuators > TCIO_MAX_ACTUATORS) ||
	    (_max_transfer < 16) || (_max_transfer > 255)  ||
	    (_max_rc_input < 1)  || (_max_rc_input > 255)) {

		TC_ERR("config read error");
		mavlink_log_emergency(&_mavlink_log_pub, "[IO] config read fail, abort.\t");
		events::send(events::ID("tcio_config_read_failed"), events::Log::Emergency,
			     "IO config read failed, aborting initialization");

		// ask IO to reboot into bootloader as the failure may
		// be due to mismatched firmware versions and we want
		// the startup script to be able to load a new IO
		// firmware

		// Now the reboot into bootloader mode should succeed.
		io_reg_set(TCIO_PAGE_SETUP, TCIO_P_SETUP_REBOOT_BL, TCIO_REBOOT_BL_MAGIC);
		return -1;
	}

	/* Set safety_off to false when FMU boot*/
	io_reg_set(TCIO_PAGE_SETUP, TCIO_P_SETUP_SAFETY_OFF, 0);

	if (_max_rc_input > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		_max_rc_input = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	uint16_t reg = 0;

	/* get IO's last seen FMU state */
	ret = io_reg_get(TCIO_PAGE_SETUP, TCIO_P_SETUP_ARMING, &reg, sizeof(reg));

	if (ret != OK) {
		return ret;
	}

	/* dis-arm IO before touching anything */
	io_reg_modify(TCIO_PAGE_SETUP, TCIO_P_SETUP_ARMING, TCIO_P_SETUP_ARMING_FMU_ARMED | TCIO_P_SETUP_ARMING_LOCKDOWN,
		      0);

	if (ret != OK) {
		mavlink_log_critical(&_mavlink_log_pub, "IO RC config upload fail\t");
		events::send(events::ID("tcio_io_rc_config_upload_failed"), events::Log::Critical,
			     "IO RC config upload failed, aborting initialization");
		return ret;
	}

	/* initialize _group_channels */
	for (uint8_t group = TCIO_P_SETUP_PWM_RATE_GROUP0; group <= TCIO_P_SETUP_PWM_RATE_GROUP3; ++group) {
		unsigned group_idx = group - TCIO_P_SETUP_PWM_RATE_GROUP0;
		_group_channels[group_idx] = io_reg_get(TCIO_PAGE_PWM_INFO, TCIO_RATE_MAP_BASE + group_idx);
	}

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	if (_param_sys_hitl.get() <= 0 && _param_sys_use_io.get() == 1) {
		_mixing_output.setMaxTopicUpdateRate(MIN_TOPIC_UPDATE_INTERVAL);
	}

	_tcio_status_pub.advertise();

	update_params();

	ScheduleNow();

	return OK;
}

void TCIO::updateDisarmed()
{
	uint16_t values[TCIO_MAX_ACTUATORS] {};

	for (unsigned i = 0; i < _max_actuators; i++) {
		values[i] = _mixing_output.disarmedValue(i);
	}

	io_reg_set(TCIO_PAGE_DISARMED_PWM, 0, values, _max_actuators);
}

void TCIO::updateFailsafe()
{
	uint16_t values[TCIO_MAX_ACTUATORS] {};

	for (unsigned i = 0; i < _max_actuators; i++) {
		values[i] = _mixing_output.actualFailsafeValue(i);
	}

	io_reg_set(TCIO_PAGE_FAILSAFE_PWM, 0, values, _max_actuators);
}

void TCIO::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	SmartLock lock_guard(_lock);

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	/* if we have new control data from the ORB, handle it */
	if (_param_sys_hitl.get() <= 0) {
		_mixing_output.update();
	}

	if (hrt_elapsed_time(&_poll_last) >= 20_ms) {
		/* run at 50 */
		_poll_last = hrt_absolute_time();

		/* pull status and alarms from IO */
		io_get_status();

		/* get raw R/C input from IO */
		io_publish_raw_rc();
	}

	if (_param_sys_hitl.get() <= 0) {
		/* check updates on uORB topics and handle it */
		if (_t_actuator_armed.updated()) {
			io_set_arming_state();

			// TODO: throttle
		}
	}

	if (!_mixing_output.armed().armed) {
		/* vehicle command */
		if (_t_vehicle_command.updated()) {
			vehicle_command_s cmd{};
			_t_vehicle_command.copy(&cmd);

			// Check for a DSM pairing command
			if (((unsigned int)cmd.command == vehicle_command_s::VEHICLE_CMD_START_RX_PAIR) && ((int)cmd.param1 == 0)) {
				int bind_arg;

				switch ((int)cmd.param2) {
				case 0:
					bind_arg = DSM2_BIND_PULSES;
					break;

				case 1:
					bind_arg = DSMX_BIND_PULSES;
					break;

				case 2:
				default:
					bind_arg = DSMX8_BIND_PULSES;
					break;
				}

				int dsm_ret = dsm_bind_ioctl(bind_arg);

				/* publish ACK */
				if (dsm_ret == OK) {
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

				} else {
					answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED);
				}
			}
		}

		/*
		 * If parameters have changed, re-send RC mappings to IO
		 */

		// check for parameter updates
		if (_parameter_update_sub.updated() || _param_update_force) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			_param_update_force = false;

			update_params();

			/* Check if the flight termination circuit breaker has been updated */
			bool disable_flighttermination = circuit_breaker_enabled("CBRK_FLIGHTTERM", CBRK_FLIGHTTERM_KEY);
			/* Tell IO that it can terminate the flight if FMU is not responding or if a failure has been reported by the FailureDetector logic */
			io_reg_set(TCIO_PAGE_SETUP, TCIO_P_SETUP_ENABLE_FLIGHTTERMINATION, !disable_flighttermination);

			if (_param_sens_en_themal.get() != _thermal_control || _param_update_force) {

				_thermal_control = _param_sens_en_themal.get();
				/* set power management state for thermal */
				uint16_t tctrl;

				if (_thermal_control < 0) {
					tctrl = TCIO_THERMAL_IGNORE;

				} else {
					tctrl = TCIO_THERMAL_OFF;
				}

				io_reg_set(TCIO_PAGE_SETUP, TCIO_P_SETUP_THERMAL, tctrl);
			}

			/* S.BUS output */
			if (_param_pwm_sbus_mode.get() == 1) {
				/* enable S.BUS 1 */
				io_reg_modify(TCIO_PAGE_SETUP, TCIO_P_SETUP_FEATURES, 0, TCIO_P_SETUP_FEATURES_SBUS1_OUT);

			} else if (_param_pwm_sbus_mode.get() == 2) {
				/* enable S.BUS 2 */
				io_reg_modify(TCIO_PAGE_SETUP, TCIO_P_SETUP_FEATURES, 0, TCIO_P_SETUP_FEATURES_SBUS2_OUT);

			} else {
				/* disable S.BUS */
				io_reg_modify(TCIO_PAGE_SETUP, TCIO_P_SETUP_FEATURES,
					      (TCIO_P_SETUP_FEATURES_SBUS1_OUT | TCIO_P_SETUP_FEATURES_SBUS2_OUT), 0);
			}
		}
	}

	// minimal backup scheduling
	ScheduleDelayed(20_ms);

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
	_first_update_cycle = false;
}

void TCIO::updateTimerRateGroups()
{
	if (_timer_rates_configured) { // avoid setting timer rates on each param update
		return;
	}

	_timer_rates_configured = true;

	int timer = 0;

	uint16_t timer_rates[TCIO_P_SETUP_PWM_RATE_GROUP3 - TCIO_P_SETUP_PWM_RATE_GROUP0 + 1] {};

	for (uint8_t offset = TCIO_P_SETUP_PWM_RATE_GROUP0; offset <= TCIO_P_SETUP_PWM_RATE_GROUP3; ++offset) {
		char param_name[17];
		snprintf(param_name, sizeof(param_name), "%s_TIM%u", _mixing_output.paramPrefix(), timer);

		int32_t tim_config = 0;
		param_t handle = param_find(param_name);

		if (handle == PARAM_INVALID) {
			break;
		}

		param_get(handle, &tim_config);

		if (tim_config > 0) {
			timer_rates[timer] = tim_config;

		} else if (tim_config == -1) { // OneShot
			timer_rates[timer] = 0;
		}

		++timer;
	}

	int ret = io_reg_set(TCIO_PAGE_SETUP, TCIO_P_SETUP_PWM_RATE_GROUP0, timer_rates, timer);

	if (ret != 0) {
		TC_ERR("io_reg_set failed (%i)", ret);
	}
}

void TCIO::update_params()
{
	uint32_t previously_set_functions = 0;

	for (size_t i = 0; i < _max_actuators; i++) {
		previously_set_functions |= (uint32_t)_mixing_output.isFunctionSet(i) << i;
	}

	updateParams();

	if (!_mixing_output.armed().armed) {

		// Automatically set the PWM rate and disarmed value when a channel is first set to a servo
		if (!_first_update_cycle) {
			for (size_t i = 0; i < _max_actuators; i++) {
				if ((previously_set_functions & (1u << i)) == 0 && _mixing_output.functionParamHandle(i) != PARAM_INVALID) {
					int32_t output_function;

					if (param_get(_mixing_output.functionParamHandle(i), &output_function) == 0) {
						if (output_function >= (int)OutputFunction::Servo1
						    && output_function <= (int)OutputFunction::ServoMax) { // Function got set to a servo
							int32_t val = 1500;
							TC_INFO("Setting channel %i disarmed to %i", (int) val, i);
							param_set(_mixing_output.disarmedParamHandle(i), &val);

							// If the whole timer group was not set previously, then set the pwm rate to 50 Hz
							for (int timer = 0; timer < (int)(sizeof(_group_channels) / sizeof(_group_channels[0])); ++timer) {

								uint32_t channels = _group_channels[timer];

								if ((channels & (1u << i)) == 0) {
									continue;
								}

								if ((channels & previously_set_functions) == 0) { // None of the channels was set
									char param_name[17];
									snprintf(param_name, sizeof(param_name), "%s_TIM%u", _mixing_output.paramPrefix(), timer);

									int32_t tim_config = 0;
									param_t handle = param_find(param_name);

									if (param_get(handle, &tim_config) == 0 && tim_config == 400) {
										tim_config = 50;
										TC_INFO("setting timer %i to %i Hz", timer, (int) tim_config);
										param_set(handle, &tim_config);
									}
								}
							}
						}

						// Motors need a minimum value that idles the motor
						if (output_function >= (int)OutputFunction::Motor1
						    && output_function <= (int)OutputFunction::MotorMax) { // Function got set to a motor
							int32_t val = 1100;
							TC_INFO("Setting channel %i minimum to %i", (int) val, i);
							param_set(_mixing_output.minParamHandle(i), &val);
							val = 1900;
							TC_INFO("Setting channel %i maximum to %i", (int) val, i);
							param_set(_mixing_output.maxParamHandle(i), &val);
						}
					}
				}
			}
		}

		// sync params to IO
		updateTimerRateGroups();
		updateFailsafe();
		updateDisarmed();
	}
}

void TCIO::answer_command(const vehicle_command_s &cmd, uint8_t result)
{
	/* publish ACK */
	uORB::Publication<vehicle_command_ack_s> vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
	vehicle_command_ack_s command_ack{};
	command_ack.command = cmd.command;
	command_ack.result = result;
	command_ack.target_system = cmd.source_system;
	command_ack.target_component = cmd.source_component;
	command_ack.timestamp = hrt_absolute_time();
	vehicle_command_ack_pub.publish(command_ack);
}

int
TCIO::io_set_arming_state()
{
	uint16_t set = 0;
	uint16_t clear = 0;

	actuator_armed_s armed;

	if (_t_actuator_armed.copy(&armed)) {
		if (armed.armed || armed.in_esc_calibration_mode) {
			set |= TCIO_P_SETUP_ARMING_FMU_ARMED;

		} else {
			clear |= TCIO_P_SETUP_ARMING_FMU_ARMED;
		}

		if (armed.prearmed) {
			set |= TCIO_P_SETUP_ARMING_FMU_PREARMED;

		} else {
			clear |= TCIO_P_SETUP_ARMING_FMU_PREARMED;
		}

		if ((armed.lockdown || armed.manual_lockdown) && !_lockdown_override) {
			set |= TCIO_P_SETUP_ARMING_LOCKDOWN;
			_lockdown_override = true;

		} else if (!(armed.lockdown || armed.manual_lockdown) && _lockdown_override) {
			clear |= TCIO_P_SETUP_ARMING_LOCKDOWN;
			_lockdown_override = false;
		}

		if (armed.force_failsafe) {
			set |= TCIO_P_SETUP_ARMING_FORCE_FAILSAFE;

		} else {
			clear |= TCIO_P_SETUP_ARMING_FORCE_FAILSAFE;
		}

		// XXX this is for future support in the commander
		// but can be removed if unneeded
		// if (armed.termination_failsafe) {
		// 	set |= TCIO_P_SETUP_ARMING_TERMINATION_FAILSAFE;
		// } else {
		// 	clear |= TCIO_P_SETUP_ARMING_TERMINATION_FAILSAFE;
		// }

		if (armed.ready_to_arm) {
			set |= TCIO_P_SETUP_ARMING_IO_ARM_OK;

		} else {
			clear |= TCIO_P_SETUP_ARMING_IO_ARM_OK;
		}
	}

	if (_last_written_arming_s != set || _last_written_arming_c != clear) {
		_last_written_arming_s = set;
		_last_written_arming_c = clear;
		return io_reg_modify(TCIO_PAGE_SETUP, TCIO_P_SETUP_ARMING, clear, set);
	}

	return 0;
}

int TCIO::io_handle_status(uint16_t status)
{
	int ret = 1;
	/**
	 * WARNING: This section handles in-air resets.
	 */

	/* check for IO reset - force it back to armed if necessary */
	if (!(status & TCIO_P_STATUS_FLAGS_ARM_SYNC)) {
		/* set the arming flag */
		ret = io_reg_modify(TCIO_PAGE_STATUS, TCIO_P_STATUS_FLAGS, 0, TCIO_P_STATUS_FLAGS_ARM_SYNC);

		/* set new status */
		_status = status;

	} else if (!(_status & TCIO_P_STATUS_FLAGS_ARM_SYNC)) {

		/* set the sync flag */
		ret = io_reg_modify(TCIO_PAGE_STATUS, TCIO_P_STATUS_FLAGS, 0, TCIO_P_STATUS_FLAGS_ARM_SYNC);
		/* set new status */
		_status = status;

	} else {
		ret = 0;

		/* set new status */
		_status = status;
	}

	/**
	 * Get and handle the safety button status
	 */
	const bool safety_button_pressed = status & TCIO_P_STATUS_FLAGS_SAFETY_BUTTON_EVENT;

	if (safety_button_pressed) {
		io_reg_set(TCIO_PAGE_SETUP, TCIO_P_SETUP_SAFETY_BUTTON_ACK, 0);
		_button_publisher.safetyButtonTriggerEvent();
	}

	/**
	 * Inform TCIO board about safety_off state for LED control
	 */
	vehicle_status_s vehicle_status;

	if (_t_vehicle_status.update(&vehicle_status)) {
		if (_previous_safety_off != vehicle_status.safety_off) {
			io_reg_set(TCIO_PAGE_SETUP, TCIO_P_SETUP_SAFETY_OFF, vehicle_status.safety_off);
			_previous_safety_off = vehicle_status.safety_off;
		}
	}

	return ret;
}

int TCIO::dsm_bind_ioctl(int dsmMode)
{
	// Do not bind if invalid pulses are provided
	if (dsmMode != DSM2_BIND_PULSES &&
	    dsmMode != DSMX_BIND_PULSES &&
	    dsmMode != DSMX8_BIND_PULSES) {
		TC_ERR("Unknown DSM mode: %d", dsmMode);
		return -EINVAL;
	}

	// Do not bind if armed
	bool armed = (_status & TCIO_P_SETUP_ARMING_FMU_ARMED);

	if (armed) {
		TC_ERR("Not binding DSM, system is armed.");
		return -EINVAL;
	}

	TC_INFO("Binding DSM%s RX", (dsmMode == DSM2_BIND_PULSES) ? "2" : ((dsmMode == DSMX_BIND_PULSES) ? "-X" : "-X8"));

	int ret = OK;
	ret |= io_reg_set(TCIO_PAGE_SETUP, TCIO_P_SETUP_DSM, dsm_bind_power_down);
	tc_usleep(500000);
	ret |= io_reg_set(TCIO_PAGE_SETUP, TCIO_P_SETUP_DSM, dsm_bind_set_rx_out);
	ret |= io_reg_set(TCIO_PAGE_SETUP, TCIO_P_SETUP_DSM, dsm_bind_power_up);
	tc_usleep(72000);
	ret |= io_reg_set(TCIO_PAGE_SETUP, TCIO_P_SETUP_DSM, dsm_bind_send_pulses | (dsmMode << 4));
	tc_usleep(50000);
	ret |= io_reg_set(TCIO_PAGE_SETUP, TCIO_P_SETUP_DSM, dsm_bind_reinit_uart);

	if (ret != OK) {
		TC_INFO("Binding DSM failed");
	}

	return ret;
}

int TCIO::io_get_status()
{
	/* get
	 * STATUS_FLAGS, STATUS_ALARMS, STATUS_VBATT, STATUS_IBATT,
	 * STATUS_VSERVO, STATUS_VRSSI
	 * in that order */
	uint16_t regs[6] {};
	int ret = io_reg_get(TCIO_PAGE_STATUS, TCIO_P_STATUS_FLAGS, &regs[0], sizeof(regs) / sizeof(regs[0]));

	if (ret != OK) {
		return ret;
	}

	const uint16_t STATUS_FLAGS  = regs[0];
	const uint16_t STATUS_ALARMS = regs[1];
	const uint16_t STATUS_VSERVO = regs[4];
	const uint16_t STATUS_VRSSI  = regs[5];

	io_handle_status(STATUS_FLAGS);

	const float rssi_v = STATUS_VRSSI * 0.001f; // voltage is scaled to mV

	if (_analog_rc_rssi_volt < 0.f) {
		_analog_rc_rssi_volt = rssi_v;
	}

	_analog_rc_rssi_volt = _analog_rc_rssi_volt * 0.99f + rssi_v * 0.01f;

	if (_analog_rc_rssi_volt > 2.5f) {
		_analog_rc_rssi_stable = true;
	}

	const uint16_t SETUP_ARMING = io_reg_get(TCIO_PAGE_SETUP, TCIO_P_SETUP_ARMING);

	if ((hrt_elapsed_time(&_last_status_publish) >= 1_s)
	    || (_status != STATUS_FLAGS)
	    || (_alarms != STATUS_ALARMS)
	    || (_setup_arming != SETUP_ARMING)
	   ) {

		tcio_status_s status{};

		status.voltage_v = STATUS_VSERVO * 0.001f; // voltage is scaled to mV
		status.rssi_v = rssi_v;

		status.free_memory_bytes = io_reg_get(TCIO_PAGE_STATUS, TCIO_P_STATUS_FREEMEM);

		// TCIO_P_STATUS_FLAGS
		status.status_outputs_armed   = STATUS_FLAGS & TCIO_P_STATUS_FLAGS_OUTPUTS_ARMED;
		status.status_rc_ok           = STATUS_FLAGS & TCIO_P_STATUS_FLAGS_RC_OK;
		status.status_rc_ppm          = STATUS_FLAGS & TCIO_P_STATUS_FLAGS_RC_PPM;
		status.status_rc_dsm          = STATUS_FLAGS & TCIO_P_STATUS_FLAGS_RC_DSM;
		status.status_rc_sbus         = STATUS_FLAGS & TCIO_P_STATUS_FLAGS_RC_SBUS;
		status.status_rc_st24         = STATUS_FLAGS & TCIO_P_STATUS_FLAGS_RC_ST24;
		status.status_rc_sumd         = STATUS_FLAGS & TCIO_P_STATUS_FLAGS_RC_SUMD;
		status.status_fmu_initialized = STATUS_FLAGS & TCIO_P_STATUS_FLAGS_FMU_INITIALIZED;
		status.status_fmu_ok          = STATUS_FLAGS & TCIO_P_STATUS_FLAGS_FMU_OK;
		status.status_raw_pwm         = STATUS_FLAGS & TCIO_P_STATUS_FLAGS_RAW_PWM;
		status.status_arm_sync        = STATUS_FLAGS & TCIO_P_STATUS_FLAGS_ARM_SYNC;
		status.status_init_ok         = STATUS_FLAGS & TCIO_P_STATUS_FLAGS_INIT_OK;
		status.status_failsafe        = STATUS_FLAGS & TCIO_P_STATUS_FLAGS_FAILSAFE;
		status.status_safety_button_event = STATUS_FLAGS & TCIO_P_STATUS_FLAGS_SAFETY_BUTTON_EVENT;

		// TCIO_P_STATUS_ALARMS
		status.alarm_rc_lost       = STATUS_ALARMS & TCIO_P_STATUS_ALARMS_RC_LOST;
		status.alarm_pwm_error     = STATUS_ALARMS & TCIO_P_STATUS_ALARMS_PWM_ERROR;

		// TCIO_P_SETUP_ARMING
		status.arming_io_arm_ok            = SETUP_ARMING & TCIO_P_SETUP_ARMING_IO_ARM_OK;
		status.arming_fmu_armed            = SETUP_ARMING & TCIO_P_SETUP_ARMING_FMU_ARMED;
		status.arming_fmu_prearmed         = SETUP_ARMING & TCIO_P_SETUP_ARMING_FMU_PREARMED;
		status.arming_failsafe_custom      = SETUP_ARMING & TCIO_P_SETUP_ARMING_FAILSAFE_CUSTOM;
		status.arming_lockdown             = SETUP_ARMING & TCIO_P_SETUP_ARMING_LOCKDOWN;
		status.arming_force_failsafe       = SETUP_ARMING & TCIO_P_SETUP_ARMING_FORCE_FAILSAFE;
		status.arming_termination_failsafe = SETUP_ARMING & TCIO_P_SETUP_ARMING_TERMINATION_FAILSAFE;

		for (unsigned i = 0; i < _max_actuators; i++) {
			status.pwm[i] = io_reg_get(TCIO_PAGE_SERVOS, i);
			status.pwm_disarmed[i] = io_reg_get(TCIO_PAGE_DISARMED_PWM, i);
			status.pwm_failsafe[i] = io_reg_get(TCIO_PAGE_FAILSAFE_PWM, i);
		}

		{
			int i = 0;

			for (uint8_t offset = TCIO_P_SETUP_PWM_RATE_GROUP0; offset <= TCIO_P_SETUP_PWM_RATE_GROUP3; ++offset) {
				// This is a bit different than below, setting the groups, not the channels
				status.pwm_rate_hz[i++] = io_reg_get(TCIO_PAGE_SETUP, offset);
			}
		}

		uint16_t raw_inputs = io_reg_get(TCIO_PAGE_RAW_RC_INPUT, TCIO_P_RAW_RC_COUNT);

		for (unsigned i = 0; (i < raw_inputs) && (i < _max_rc_input); i++) {
			status.raw_inputs[i] = io_reg_get(TCIO_PAGE_RAW_RC_INPUT, TCIO_P_RAW_RC_BASE + i);
		}

		status.timestamp = hrt_absolute_time();
		_tcio_status_pub.publish(status);

		_last_status_publish = status.timestamp;
	}

	_alarms = STATUS_ALARMS;
	_setup_arming = SETUP_ARMING;

	return ret;
}

int TCIO::io_publish_raw_rc()
{
	const uint16_t rc_valid_update_count = io_reg_get(TCIO_PAGE_RAW_RC_INPUT, TCIO_P_RAW_FRAME_COUNT);
	const bool rc_updated = (rc_valid_update_count != _rc_valid_update_count);
	_rc_valid_update_count = rc_valid_update_count;

	if (!rc_updated) {
		return 0;
	}

	input_rc_s input_rc{};
	input_rc.timestamp_last_signal = hrt_absolute_time();

	/* set the RC status flag ORDER MATTERS! */
	input_rc.rc_lost = !(_status & TCIO_P_STATUS_FLAGS_RC_OK);

	/* we don't have the status bits, so input_source has to be set elsewhere */
	input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_UNKNOWN;

	const unsigned prolog = (TCIO_P_RAW_RC_BASE - TCIO_P_RAW_RC_COUNT);
	uint16_t regs[input_rc_s::RC_INPUT_MAX_CHANNELS + prolog];

	/*
	 * Read the channel count and the first 9 channels.
	 *
	 * This should be the common case (9 channel R/C control being a reasonable upper bound).
	 */
	int ret = io_reg_get(TCIO_PAGE_RAW_RC_INPUT, TCIO_P_RAW_RC_COUNT, &regs[0], prolog + 9);

	if (ret != OK) {
		return ret;
	}

	/*
	 * Get the channel count any any extra channels. This is no more expensive than reading the
	 * channel count once.
	 */
	uint32_t channel_count = regs[TCIO_P_RAW_RC_COUNT];

	/* limit the channel count */
	if (channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	input_rc.timestamp = hrt_absolute_time();

	input_rc.rc_ppm_frame_length = regs[TCIO_P_RAW_RC_DATA];

	if (!_analog_rc_rssi_stable) {
		input_rc.rssi = regs[TCIO_P_RAW_RC_NRSSI];

	} else {
		float rssi_analog = ((_analog_rc_rssi_volt - 0.2f) / 3.0f) * 100.0f;

		if (rssi_analog > 100.0f) {
			rssi_analog = 100.0f;
		}

		if (rssi_analog < 0.0f) {
			rssi_analog = 0.0f;
		}

		input_rc.rssi = rssi_analog;
	}

	input_rc.rc_failsafe = (regs[TCIO_P_RAW_RC_FLAGS] & TCIO_P_RAW_RC_FLAGS_FAILSAFE);
	input_rc.rc_lost = !(regs[TCIO_P_RAW_RC_FLAGS] & TCIO_P_RAW_RC_FLAGS_RC_OK);
	input_rc.rc_lost_frame_count = regs[TCIO_P_RAW_LOST_FRAME_COUNT];
	input_rc.rc_total_frame_count = regs[TCIO_P_RAW_FRAME_COUNT];
	input_rc.channel_count = channel_count;


	/* FIELDS NOT SET HERE */
	/* input_rc.input_source is set after this call XXX we might want to mirror the flags in the RC struct */

	if (channel_count > 9) {
		ret = io_reg_get(TCIO_PAGE_RAW_RC_INPUT, TCIO_P_RAW_RC_BASE + 9, &regs[prolog + 9], channel_count - 9);

		if (ret != OK) {
			return ret;
		}
	}

	/* last thing set are the actual channel values as 16 bit values */
	for (unsigned i = 0; i < channel_count; i++) {
		input_rc.values[i] = regs[prolog + i];
	}

	/* zero the remaining fields */
	for (unsigned i = channel_count; i < (sizeof(input_rc.values) / sizeof(input_rc.values[0])); i++) {
		input_rc.values[i] = 0;
	}

	/* get RSSI from input channel */
	if (_param_rc_rssi_pwm_chan.get() > 0 && _param_rc_rssi_pwm_chan.get() <= input_rc_s::RC_INPUT_MAX_CHANNELS) {
		const auto &min = _param_rc_rssi_pwm_min.get();
		const auto &max = _param_rc_rssi_pwm_max.get();

		if (max - min != 0) {
			int rssi = ((input_rc.values[_param_rc_rssi_pwm_chan.get() - 1] - min) * 100) / (max - min);
			input_rc.rssi = math::constrain(rssi, 0, 100);
		}
	}

	/* sort out the source of the values */
	if (_status & TCIO_P_STATUS_FLAGS_RC_PPM) {
		input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_TCIO_PPM;

	} else if (_status & TCIO_P_STATUS_FLAGS_RC_DSM) {
		input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_TCIO_SPEKTRUM;

	} else if (_status & TCIO_P_STATUS_FLAGS_RC_SBUS) {
		input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_TCIO_SBUS;

	} else if (_status & TCIO_P_STATUS_FLAGS_RC_ST24) {
		input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_TCIO_ST24;
	}

	if (input_rc.input_source != input_rc_s::RC_INPUT_SOURCE_UNKNOWN) {

		input_rc.link_quality = -1;
		input_rc.rssi_dbm = NAN;

		_input_rc_pub.publish(input_rc);
	}

	return ret;
}

int TCIO::io_reg_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values)
{
	/* range check the transfer */
	if (num_values > ((_max_transfer) / sizeof(*values))) {
		TC_DEBUG("io_reg_set: too many registers (%u, max %u)", num_values, _max_transfer / 2);
		return -EINVAL;
	}

	perf_begin(_interface_write_perf);
	int ret = _interface->write((page << 8) | offset, (void *)values, num_values);
	perf_end(_interface_write_perf);

	if (ret != (int)num_values) {
		TC_DEBUG("io_reg_set(%" PRIu8 ",%" PRIu8 ",%u): error %d", page, offset, num_values, ret);
		return -1;
	}

	return OK;
}

int TCIO::io_reg_set(uint8_t page, uint8_t offset, uint16_t value)
{
	return io_reg_set(page, offset, &value, 1);
}

int TCIO::io_reg_get(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values)
{
	/* range check the transfer */
	if (num_values > ((_max_transfer) / sizeof(*values))) {
		TC_DEBUG("io_reg_get: too many registers (%u, max %u)", num_values, _max_transfer / 2);
		return -EINVAL;
	}

	perf_begin(_interface_read_perf);
	int ret = _interface->read((page << 8) | offset, reinterpret_cast<void *>(values), num_values);
	perf_end(_interface_read_perf);

	if (ret != (int)num_values) {
		TC_DEBUG("io_reg_get(%" PRIu8 ",%" PRIu8 ",%u): data error %d", page, offset, num_values, ret);
		return -1;
	}

	return OK;
}

uint32_t TCIO::io_reg_get(uint8_t page, uint8_t offset)
{
	uint16_t value;

	if (io_reg_get(page, offset, &value, 1) != OK) {
		return _io_reg_get_error;
	}

	return value;
}

int TCIO::io_reg_modify(uint8_t page, uint8_t offset, uint16_t clearbits, uint16_t setbits)
{
	uint16_t value = 0;
	int ret = io_reg_get(page, offset, &value, 1);

	if (ret != OK) {
		return ret;
	}

	value &= ~clearbits;
	value |= setbits;

	return io_reg_set(page, offset, value);
}

int TCIO::print_status()
{
	/* basic configuration */
	printf("protocol %" PRIu32 " hardware %" PRIu32 " bootloader %" PRIu32 " buffer %" PRIu32 "B crc 0x%04" PRIu32 "%04"
	       PRIu32 "\n",
	       io_reg_get(TCIO_PAGE_CONFIG, TCIO_P_CONFIG_PROTOCOL_VERSION),
	       io_reg_get(TCIO_PAGE_CONFIG, TCIO_P_CONFIG_HARDWARE_VERSION),
	       io_reg_get(TCIO_PAGE_CONFIG, TCIO_P_CONFIG_BOOTLOADER_VERSION),
	       io_reg_get(TCIO_PAGE_CONFIG, TCIO_P_CONFIG_MAX_TRANSFER),
	       io_reg_get(TCIO_PAGE_SETUP,  TCIO_P_SETUP_CRC),
	       io_reg_get(TCIO_PAGE_SETUP,  TCIO_P_SETUP_CRC + 1));

	printf("%" PRIu32 " controls %" PRIu32 " actuators %" PRIu32 " R/C inputs %" PRIu32 " analog inputs\n",
	       io_reg_get(TCIO_PAGE_CONFIG, TCIO_P_CONFIG_CONTROL_COUNT),
	       io_reg_get(TCIO_PAGE_CONFIG, TCIO_P_CONFIG_ACTUATOR_COUNT),
	       io_reg_get(TCIO_PAGE_CONFIG, TCIO_P_CONFIG_RC_INPUT_COUNT),
	       io_reg_get(TCIO_PAGE_CONFIG, TCIO_P_CONFIG_ADC_INPUT_COUNT));

	/* status */
	uORB::SubscriptionData<tcio_status_s> status_sub{ORB_ID(tcio_status)};
	status_sub.update();

	print_message(ORB_ID(tcio_status), status_sub.get());

	/* now clear alarms */
	io_reg_set(TCIO_PAGE_STATUS, TCIO_P_STATUS_ALARMS, 0x0000);

	printf("\n");

	uint16_t raw_inputs = io_reg_get(TCIO_PAGE_RAW_RC_INPUT, TCIO_P_RAW_RC_COUNT);
	printf("%" PRIu16 " raw R/C inputs", raw_inputs);

	for (unsigned i = 0; i < raw_inputs; i++) {
		printf(" %" PRIu32, io_reg_get(TCIO_PAGE_RAW_RC_INPUT, TCIO_P_RAW_RC_BASE + i));
	}

	printf("\n");
	uint16_t adc_inputs = io_reg_get(TCIO_PAGE_CONFIG, TCIO_P_CONFIG_ADC_INPUT_COUNT);
	printf("ADC inputs");

	for (unsigned i = 0; i < adc_inputs; i++) {
		printf(" %" PRIu32, io_reg_get(TCIO_PAGE_RAW_ADC_INPUT, i));
	}

	printf("\n");

	/* setup and state */
	uint16_t features = io_reg_get(TCIO_PAGE_SETUP, TCIO_P_SETUP_FEATURES);
	printf("features 0x%04" PRIx16 "%s%s%s\n", features,
	       ((features & TCIO_P_SETUP_FEATURES_SBUS1_OUT) ? " S.BUS1_OUT" : ""),
	       ((features & TCIO_P_SETUP_FEATURES_SBUS2_OUT) ? " S.BUS2_OUT" : ""),
	       ((features & TCIO_P_SETUP_FEATURES_ADC_RSSI) ? " RSSI_ADC" : "")
	      );

	printf("sbus rate %" PRIu32 "\n", io_reg_get(TCIO_PAGE_SETUP, TCIO_P_SETUP_SBUS_RATE));

	printf("debuglevel %" PRIu32 "\n", io_reg_get(TCIO_PAGE_SETUP, TCIO_P_SETUP_SET_DEBUG));

	/* IMU heater (Pixhawk 2.1) */
	uint16_t heater_level = io_reg_get(TCIO_PAGE_SETUP, TCIO_P_SETUP_THERMAL);

	if (heater_level != UINT16_MAX) {
		if (heater_level == TCIO_THERMAL_OFF) {
			printf("\nIMU heater off");

		} else {
			printf("\nIMU heater level %d", heater_level);
		}
	}

	printf("\n");

	_mixing_output.printStatus();
	return 0;
}

int TCIO::ioctl(file *filep, int cmd, unsigned long arg)
{
	SmartLock lock_guard(_lock);
	int ret = OK;

	/* regular ioctl? */
	switch (cmd) {
	case DSM_BIND_START:
		/* bind a DSM receiver */
		ret = dsm_bind_ioctl(arg);
		break;

	case TCIO_SET_DEBUG:
		TC_DEBUG("TCIO_SET_DEBUG");

		/* set the debug level */
		ret = io_reg_set(TCIO_PAGE_SETUP, TCIO_P_SETUP_SET_DEBUG, arg);
		break;

	case TCIO_REBOOT_BOOTLOADER:
		if (system_status() & TCIO_P_SETUP_ARMING_FMU_ARMED) {
			TC_ERR("not upgrading IO firmware, system is armed");
			return -EINVAL;

		}

		/* For Legacy TCIO Firmware only:
		 * If IO has already safety off it won't accept going into bootloader mode,
		 * therefore we need to set safety on first. */
		io_reg_set(TCIO_PAGE_SETUP, 14, 22027);

		/* reboot into bootloader - arg must be TCIO_REBOOT_BL_MAGIC */
		usleep(1);
		ret = io_reg_set(TCIO_PAGE_SETUP, TCIO_P_SETUP_REBOOT_BL, arg);

		if (ret != TC_OK) {
			TC_WARN("IO refused to reboot");
		}

		break;

	case TCIO_CHECK_CRC: {
			TC_DEBUG("TCIO_CHECK_CRC");

			/* check IO firmware CRC against passed value */
			uint32_t io_crc = 0;
			ret = io_reg_get(TCIO_PAGE_SETUP, TCIO_P_SETUP_CRC, (uint16_t *)&io_crc, 2);

			if (ret != OK) {
				return ret;
			}

			if (io_crc != arg) {
				TC_DEBUG("Firmware CRC mismatch 0x%08" PRIx32 " 0x%08lx", io_crc, arg);
				return -EINVAL;
			}

			break;
		}

	case SBUS_SET_PROTO_VERSION:
		TC_DEBUG("SBUS_SET_PROTO_VERSION");

		if (arg == 1) {
			ret = io_reg_modify(TCIO_PAGE_SETUP, TCIO_P_SETUP_FEATURES, 0, TCIO_P_SETUP_FEATURES_SBUS1_OUT);

		} else if (arg == 2) {
			ret = io_reg_modify(TCIO_PAGE_SETUP, TCIO_P_SETUP_FEATURES, 0, TCIO_P_SETUP_FEATURES_SBUS2_OUT);

		} else {
			ret = io_reg_modify(TCIO_PAGE_SETUP, TCIO_P_SETUP_FEATURES,
					    (TCIO_P_SETUP_FEATURES_SBUS1_OUT | TCIO_P_SETUP_FEATURES_SBUS2_OUT), 0);
		}

		break;

	case TCIO_HEATER_CONTROL:
		if (arg == (unsigned long)HEATER_MODE_DISABLED) {
			io_reg_set(TCIO_PAGE_SETUP, TCIO_P_SETUP_THERMAL, TCIO_THERMAL_IGNORE);

		} else if (arg == 1) {
			io_reg_set(TCIO_PAGE_SETUP, TCIO_P_SETUP_THERMAL, TCIO_THERMAL_FULL);

		} else {
			io_reg_set(TCIO_PAGE_SETUP, TCIO_P_SETUP_THERMAL, TCIO_THERMAL_OFF);

		}

		break;

	default:
		/* see if the parent class can make any use of it */
		ret = CDev::ioctl(filep, cmd, arg);
		break;
	}

	return ret;
}

static device::Device *get_interface()
{
	device::Device *interface = TCIO_serial_interface();

	if (interface != nullptr) {
		if (interface->init() != OK) {
			TC_ERR("interface init failed");
			delete interface;
			interface = nullptr;
		}
	}

	return interface;
}

int TCIO::checkcrc(int argc, char *argv[])
{
	/*
	  check IO CRC against CRC of a file
	 */
	if (argc < 1) {
		TC_WARN("usage: tcio checkcrc filename");
		return 1;
	}

	device::Device *interface = get_interface();

	if (interface == nullptr) {
		TC_ERR("interface allocation failed");
		return 1;
	}

	TCIO *dev = new TCIO(interface);

	if (dev == nullptr) {
		delete interface;
		TC_ERR("driver allocation failed");
		return 1;
	}

	int fd = ::open(argv[0], O_RDONLY);

	if (fd == -1) {
		delete dev;
		TC_ERR("open of %s failed: %d", argv[0], errno);
		return 1;
	}

	const uint32_t app_size_max = 0xf000;
	uint32_t fw_crc = 0;
	uint32_t nbytes = 0;

	while (true) {
		uint8_t buf[16];
		int n = ::read(fd, buf, sizeof(buf));

		if (n <= 0) { break; }

		fw_crc = crc32part(buf, n, fw_crc);
		nbytes += n;
	}

	::close(fd);

	while (nbytes < app_size_max) {
		uint8_t b = 0xff;
		fw_crc = crc32part(&b, 1, fw_crc);
		nbytes++;
	}

	int ret = dev->ioctl(nullptr, TCIO_CHECK_CRC, fw_crc);

	delete dev;

	if (ret != OK) {
		TC_WARN("check CRC failed: %d, CRC: %" PRIu32, ret, fw_crc);
		return 1;
	}

	TC_INFO("IO FW CRC match");
	return 0;
}

int TCIO::bind(int argc, char *argv[])
{
	int pulses;

	if (argc < 1) {
		TC_ERR("needs argument, use dsm2, dsmx or dsmx8");
		return 1;
	}

	if (!strcmp(argv[0], "dsm2")) {
		pulses = DSM2_BIND_PULSES;

	} else if (!strcmp(argv[0], "dsmx")) {
		pulses = DSMX_BIND_PULSES;

	} else if (!strcmp(argv[0], "dsmx8")) {
		pulses = DSMX8_BIND_PULSES;

	} else {
		TC_ERR("unknown parameter %s, use dsm2, dsmx or dsmx8", argv[0]);
		return 1;
	}

	// Test for custom pulse parameter
	if (argc > 1) {
		pulses = atoi(argv[1]);
	}

	get_instance()->ioctl(nullptr, DSM_BIND_START, pulses);
	return 0;
}

int TCIO::task_spawn(int argc, char *argv[])
{
	device::Device *interface = get_interface();

	if (interface == nullptr) {
		TC_ERR("Failed to create interface");
		return -1;
	}

	TCIO *instance = new TCIO(interface);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == TC_OK) {
			return TC_OK;
		}

	} else {
		TC_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return TC_ERROR;
}

int TCIO::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (!strcmp(verb, "checkcrc")) {
		if (is_running()) {
			TC_ERR("io must be stopped");
			return 1;
		}

		return checkcrc(argc - 1, argv + 1);
	}

	if (!strcmp(verb, "update")) {

		if (is_running()) {
			TC_ERR("io must be stopped");
			return 1;
		}

		constexpr unsigned MAX_RETRIES = 2;
		unsigned retries = 0;
		int ret = TC_ERROR;

		while (ret != OK && retries < MAX_RETRIES) {

			device::Device *interface = get_interface();

			if (interface == nullptr) {
				TC_ERR("interface allocation failed");
				return 1;
			}

			TCIO *dev = new TCIO(interface);

			if (dev == nullptr) {
				delete interface;
				TC_ERR("driver allocation failed");
				return 1;
			}

			retries++;
			// Sleep 200 ms before the next attempt
			usleep(200 * 1000);

			// Try to reboot
			ret = dev->ioctl(nullptr, TCIO_REBOOT_BOOTLOADER, TCIO_REBOOT_BL_MAGIC);
			delete dev;

			if (ret != OK) {
				TC_WARN("reboot failed - %d, still attempting upgrade", ret);
			}

			/* Assume we are using default paths */

			const char *fn[4] = TCIO_FW_SEARCH_PATHS;

			/* Override defaults if a path is passed on command line */
			if (argc > 1) {
				fn[0] = argv[1];
				fn[1] = nullptr;
			}

			TCIO_Uploader *up = new TCIO_Uploader();

			if (!up) {
				ret = -ENOMEM;

			} else {
				ret = up->upload(&fn[0]);
				delete up;
			}
		}

		switch (ret) {
		case OK:
			break;

		case -ENOENT:
			TC_ERR("TCIO firmware file not found");
			break;

		case -EEXIST:
		case -EIO:
			TC_ERR("error updating TCIO - check that bootloader mode is enabled");
			break;

		case -EINVAL:
			TC_ERR("verify failed - retry the update");
			break;

		case -ETIMEDOUT:
			TC_ERR("timed out waiting for bootloader - power-cycle and try again");
			break;

		default:
			TC_ERR("unexpected error %d", ret);
			break;
		}

		return ret;
	}


	/* commands below here require a started driver */
	if (!is_running()) {
		TC_ERR("not running");
		return 1;
	}

	if (!strcmp(verb, "debug")) {
		if (argc <= 1) {
			TC_ERR("usage: tcio debug LEVEL");
			return 1;
		}

		uint8_t level = atoi(argv[1]);
		int ret = get_instance()->ioctl(nullptr, TCIO_SET_DEBUG, level);

		if (ret != 0) {
			TC_ERR("SET_DEBUG failed: %d", ret);
			return 1;
		}

		TC_INFO("SET_DEBUG %" PRIu8 " OK", level);
		return 0;
	}

	if (!strcmp(verb, "bind")) {
		if (!is_running()) {
			TC_ERR("io must be running");
			return 1;
		}

		return bind(argc - 1, argv + 1);
	}

	if (!strcmp(verb, "sbus1_out")) {
		int ret = get_instance()->ioctl(nullptr, SBUS_SET_PROTO_VERSION, 1);

		if (ret != 0) {
			TC_ERR("S.BUS v1 failed (%i)", ret);
			return 1;
		}

		return 0;
	}

	if (!strcmp(verb, "sbus2_out")) {
		int ret = get_instance()->ioctl(nullptr, SBUS_SET_PROTO_VERSION, 2);

		if (ret != 0) {
			TC_ERR("S.BUS v2 failed (%i)", ret);
			return 1;
		}

		return 0;
	}

	if (!strcmp(verb, "test_fmu_fail")) {
		get_instance()->test_fmu_fail(true);
		return 0;
	}

	if (!strcmp(verb, "test_fmu_ok")) {
		get_instance()->test_fmu_fail(false);
		return 0;
	}

	return print_usage("unknown command");
}

int TCIO::print_usage(const char *reason)
{
	if (reason) {
		TC_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Output driver communicating with the IO co-processor.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tcio", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_COMMAND_DESCR("checkcrc", "Check CRC for a firmware file against current code on IO");
	PRINT_MODULE_USAGE_ARG("<filename>", "Firmware file", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("update", "Update IO firmware");
	PRINT_MODULE_USAGE_ARG("<filename>", "Firmware file", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("debug", "set IO debug level");
	PRINT_MODULE_USAGE_ARG("<debug_level>", "0=disabled, 9=max verbosity", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("bind", "DSM bind");
	PRINT_MODULE_USAGE_ARG("dsm2|dsmx|dsmx8", "protocol", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("sbus1_out", "enable sbus1 out");
	PRINT_MODULE_USAGE_COMMAND_DESCR("sbus2_out", "enable sbus2 out");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test_fmu_fail", "test: turn off IO updates");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test_fmu_ok", "re-enable IO updates");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


extern "C" __EXPORT int tcio_main(int argc, char *argv[])
{
	if (!TC_MFT_HW_SUPPORTED(TC_MFT_TCIO)) {
		TC_ERR("TCIO Not Supported");
		return -1;
	}
	return TCIO::main(argc, argv);
}
