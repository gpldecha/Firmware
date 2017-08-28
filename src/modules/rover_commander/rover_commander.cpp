/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 * @file rover_commander.cpp
 * based on commander.cpp
 *
 *
 * @author Guillaume de Chambrier <chambrierg@gmail.com>
 */

#include <rover_commander/rover_commander.hpp>

#include <cmath>	// NAN

/* commander module headers */
#include "accelerometer_calibration.h"
#include "airspeed_calibration.h"
#include "baro_calibration.h"
#include "calibration_routines.h"
#include "commander_helper.h"
#include "esc_calibration.h"
#include "gyro_calibration.h"
#include "mag_calibration.h"
#include "PreflightCheck.h"
#include "px4_custom_mode.h"
#include "rc_calibration.h"
#include "state_machine_helper.h"

/* PX4 headers */
#include <dataman/dataman.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <geo/geo.h>
#include <navigator/navigation.h>
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_shutdown.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <systemlib/circuit_breaker.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/param/param.h>
#include <systemlib/rc_check.h>
#include <systemlib/state_table.h>
#include <float.h>
#include <systemlib/hysteresis/hysteresis.h>


#include <board_config.h>

#include <sys/stat.h>
#include <string.h>
#include <math.h>
#include <poll.h>
#include <float.h>
#include <matrix/math.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/power_button_state.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

#include <uORB/topics/rover_commander_state.h>

#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status_flags.h>


RoverCommander::RoverCommander(){

}


static constexpr uint8_t COMMANDER_MAX_GPS_NOISE = 60;		/**< Maximum percentage signal to noise ratio allowed for GPS reception */

/* Decouple update interval and hysteresis counters, all depends on intervals */
#define COMMANDER_MONITORING_INTERVAL 10000
#define COMMANDER_MONITORING_LOOPSPERMSEC (1/(COMMANDER_MONITORING_INTERVAL/1000.0f))


#define POSITION_TIMEOUT		(1 * 1000 * 1000)	/**< consider the local or global position estimate invalid after 1000ms */
#define FAILSAFE_DEFAULT_TIMEOUT	(3 * 1000 * 1000)	/**< hysteresis time - the failsafe will trigger after 3 seconds in this state */
#define OFFBOARD_TIMEOUT		500000
#define DIFFPRESS_TIMEOUT		2000000

#define HOTPLUG_SENS_TIMEOUT		(8 * 1000 * 1000)	/**< wait for hotplug sensors to come online for upto 8 seconds */

// #define PRINT_INTERVAL	5000000
#define PRINT_MODE_REJECT_INTERVAL	500000

#define INAIR_RESTART_HOLDOFF_INTERVAL	500000

/* Controls the probation period which is the amount of time required for position and velocity checks to pass before the validity can be changed from false to true*/
#define POSVEL_PROBATION_TAKEOFF 30E6		/**< probation duration set at takeoff (usec) */
#define POSVEL_PROBATION_MIN 1E6		/**< minimum probation duration (usec) */
#define POSVEL_PROBATION_MAX 100E6		/**< maximum probation duration (usec) */
#define POSVEL_VALID_PROBATION_FACTOR 10	/**< the rate at which the probation duration is increased while checks are failing */

/* Mavlink log uORB handle */
static orb_advert_t mavlink_log_pub = nullptr;

static orb_advert_t power_button_state_pub = nullptr;

/* System autostart ID */
static int autostart_id;

/* flags */
static bool commander_initialized = false;
static volatile bool thread_should_exit = false;	/**< daemon exit flag */
static volatile bool thread_running = false;		/**< daemon status flag */
static int daemon_task;					/**< Handle of daemon task / thread */
static bool _usb_telemetry_active = false;
static hrt_abstime commander_boot_timestamp = 0;

static unsigned int leds_counter;
/* To remember when last notification was sent */
static uint64_t last_print_mode_reject_time = 0;

static systemlib::Hysteresis auto_disarm_hysteresis(false);

static float eph_threshold = 5.0f;	// Horizontal position error threshold (m)
static float epv_threshold = 10.0f;	// Vertivcal position error threshold (m)

static hrt_abstime gpos_last_update_time_us = 0; // last time a global position update was received (usec)

static struct vehicle_status_s status = {};
static struct battery_status_s battery = {};
static struct actuator_armed_s armed = {};
static struct safety_s safety = {};
static struct vehicle_control_mode_s control_mode = {};
static struct offboard_control_mode_s offboard_control_mode = {};
static struct home_position_s _home = {};
static int32_t _flight_mode_slots[manual_control_setpoint_s::MODE_SLOT_MAX];
static struct commander_state_s internal_state = {};

static uint8_t main_state_before_rtl = commander_state_s::MAIN_STATE_MAX;
static manual_control_setpoint_s sp_man = {};		///< the current manual control setpoint
static manual_control_setpoint_s _last_sp_man = {};	///< the manual control setpoint valid at the last mode switch

static struct cpuload_s cpuload = {};


static uint8_t main_state_prev = 0;
static bool warning_action_on = false;
static bool last_overload = false;

static struct status_flags_s status_flags = {};

static uint64_t rc_signal_lost_timestamp;		// Time at which the RC reception was lost

static float avionics_power_rail_voltage;		// voltage of the avionics power rail

static bool arm_without_gps = false;
static bool arm_mission_required = false;

static bool _last_condition_global_position_valid = false;

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 *
 * @ingroup apps
 */
extern "C" __EXPORT int commander_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
void usage(const char *reason);


/**
 * Mainloop of commander.
 */
int commander_thread_main(int argc, char *argv[]);

void control_status_leds(vehicle_status_s *status_local, const actuator_armed_s *actuator_armed, bool changed,
			 battery_status_s *battery_local, const cpuload_s *cpuload_local);

void check_valid(hrt_abstime timestamp, hrt_abstime timeout, bool valid_in, bool *valid_out, bool *changed);

transition_result_t set_main_state_rc(struct vehicle_status_s *status, vehicle_global_position_s *global_position, vehicle_local_position_s *local_position, bool *changed);

void set_control_mode();

void print_reject_mode(struct vehicle_status_s *current_status, const char *msg);

void print_status();

/**
 * Loop that runs at a lower rate and priority for calibration and parameter tasks.
 */
void *commander_low_prio_loop(void *arg);

static void answer_command(struct vehicle_command_s &cmd, unsigned result,
					orb_advert_t &command_ack_pub);

/* publish vehicle status flags from the global variable status_flags*/
static void publish_status_flags(orb_advert_t &vehicle_status_flags_pub);


int commander_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("already running");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("commander",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_DEFAULT + 40,
					     3700,
					     commander_thread_main,
					     (char * const *)&argv[0]);

		unsigned constexpr max_wait_us = 1000000;
		unsigned constexpr max_wait_steps = 2000;

		unsigned i;
		for (i = 0; i < max_wait_steps; i++) {
			usleep(max_wait_us / max_wait_steps);
			if (thread_running) {
				break;
			}
		}

		return !(i < max_wait_steps);
	}

	if (!strcmp(argv[1], "stop")) {

		if (!thread_running) {
			warnx("commander already stopped");
			return 0;
		}

		thread_should_exit = true;

		while (thread_running) {
			usleep(200000);
			warnx(".");
		}

		warnx("terminated.");

		return 0;
	}

	/* commands needing the app to run below */
	if (!thread_running) {
		warnx("\tcommander not started");
		return 1;
	}

	if (!strcmp(argv[1], "status")) {
		print_status();
		return 0;
	}

	if (!strcmp(argv[1], "calibrate")) {
		if (argc > 2) {
			int calib_ret = OK;
			if (!strcmp(argv[2], "mag")) {
				calib_ret = do_mag_calibration(&mavlink_log_pub);
			} else if (!strcmp(argv[2], "accel")) {
				calib_ret = do_accel_calibration(&mavlink_log_pub);
			} else if (!strcmp(argv[2], "gyro")) {
				calib_ret = do_gyro_calibration(&mavlink_log_pub);
			} else if (!strcmp(argv[2], "level")) {
				calib_ret = do_level_calibration(&mavlink_log_pub);
			} else if (!strcmp(argv[2], "esc")) {
				calib_ret = do_esc_calibration(&mavlink_log_pub, &armed);
			} else if (!strcmp(argv[2], "airspeed")) {
				calib_ret = do_airspeed_calibration(&mavlink_log_pub);
			} else {
				warnx("argument %s unsupported.", argv[2]);
			}

			if (calib_ret) {
				warnx("calibration failed, exiting.");
				return 1;
			} else {
				return 0;
			}
		} else {
			warnx("missing argument");
		}
	}

	usage("unrecognized command");
	return 1;
}

void usage(const char *reason)
{
	if (reason && *reason > 0) {
		PX4_INFO("%s", reason);
	}

    PX4_INFO("usage: commander {start|stop|status|calibrate|}\n");
}

void print_status()
{
	warnx("type: %s", (status.is_rotary_wing) ? "symmetric motion" : "forward motion");
	warnx("safety: USB enabled: %s, power state valid: %s", (status_flags.usb_connected) ? "[OK]" : "[NO]",
	      (status_flags.condition_power_input_valid) ? " [OK]" : "[NO]");
	warnx("avionics rail: %6.2f V", (double)avionics_power_rail_voltage);
	warnx("home: lat = %.7f, lon = %.7f, alt = %.2f, yaw: %.2f", _home.lat, _home.lon, (double)_home.alt, (double)_home.yaw);
	warnx("home: x = %.7f, y = %.7f, z = %.2f ", (double)_home.x, (double)_home.y, (double)_home.z);
	warnx("datalink: %s", (status.data_link_lost) ? "LOST" : "OK");

#ifdef __PX4_POSIX
	warnx("main state: %d", internal_state.main_state);
	warnx("nav state: %d", status.nav_state);
#endif

	/* read all relevant states */
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	struct vehicle_status_s state;
	orb_copy(ORB_ID(vehicle_status), state_sub, &state);

	const char *armed_str;

	switch (status.arming_state) {
	case vehicle_status_s::ARMING_STATE_INIT:
		armed_str = "INIT";
		break;

	case vehicle_status_s::ARMING_STATE_STANDBY:
		armed_str = "STANDBY";
		break;

	case vehicle_status_s::ARMING_STATE_ARMED:
		armed_str = "ARMED";
		break;

	case vehicle_status_s::ARMING_STATE_ARMED_ERROR:
		armed_str = "ARMED_ERROR";
		break;

	case vehicle_status_s::ARMING_STATE_STANDBY_ERROR:
		armed_str = "STANDBY_ERROR";
		break;

	case vehicle_status_s::ARMING_STATE_REBOOT:
		armed_str = "REBOOT";
		break;

	case vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE:
		armed_str = "IN_AIR_RESTORE";
		break;

	default:
		armed_str = "ERR: UNKNOWN STATE";
		break;
	}

	px4_close(state_sub);


	warnx("arming: %s", armed_str);
}

static orb_advert_t status_pub;


int commander_thread_main(int argc, char *argv[])
{
	/* not yet initialized */
	commander_initialized = false;

    // XXX for now just set sensors as initialized
	status_flags.condition_system_sensors_initialized = true;

#ifdef __PX4_NUTTX
	/* NuttX indicates 3 arguments when only 2 are present */
	argc -= 1;
	argv += 1;
#endif

	/* vehicle status topic */
	status = {};

	status.hil_state = vehicle_status_s::HIL_STATE_OFF;

	if (argc > 2) {
		if (!strcmp(argv[2],"--hil")) {
			status.hil_state = vehicle_status_s::HIL_STATE_ON;
		} else {
			PX4_ERR("Argument %s not supported, abort.", argv[2]);
			thread_should_exit = true;
		}
	}

	/* set parameters */
    param_t _param_autostart_id = param_find("SYS_AUTOSTART");
	param_t _param_rc_in_off = param_find("COM_RC_IN_MODE");
	param_t _param_eph = param_find("COM_HOME_H_T");
	param_t _param_epv = param_find("COM_HOME_V_T");
	param_t _param_fmode_1 = param_find("COM_FLTMODE1");

    /* pthread for slow low prio thread */
	pthread_t commander_low_prio_thread;

	/* initialize */
    /*if (led_init() != OK) {
		PX4_WARN("LED init failed");
	}
    */

    int power_button_state_sub = orb_subscribe(ORB_ID(power_button_state));
	{
		// we need to do an initial publication to make sure uORB allocates the buffer, which cannot happen
		// in IRQ context.
		power_button_state_s button_state;
		button_state.timestamp = 0;
		button_state.event = 0xff;
		power_button_state_pub = orb_advertise(ORB_ID(power_button_state), &button_state);
		orb_copy(ORB_ID(power_button_state), power_button_state_sub, &button_state);
	}

	// We want to accept RC inputs as default
    //status_flags.rc_input_blocked = false;
	status.rc_input_mode = vehicle_status_s::RC_IN_MODE_DEFAULT;
	internal_state.main_state = commander_state_s::MAIN_STATE_MANUAL;
	internal_state.timestamp = hrt_absolute_time();
	main_state_prev = commander_state_s::MAIN_STATE_MAX;
	status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
	status.arming_state = vehicle_status_s::ARMING_STATE_INIT;

	status.failsafe = false;

	status.timestamp = hrt_absolute_time();

	status_flags.usb_connected = false;

    /* publish initial state */
	status_pub = orb_advertise(ORB_ID(vehicle_status), &status);

	if (status_pub == nullptr) {
		warnx("ERROR: orb_advertise for topic vehicle_status failed (uorb app running?).\n");
		warnx("exiting.");
		px4_task_exit(PX4_ERROR);
	}

    /* Initialize armed with all true */
    memset(&armed, 1, sizeof(armed));
	/* armed topic */
	orb_advert_t armed_pub = orb_advertise(ORB_ID(actuator_armed), &armed);

	/* vehicle control mode topic */
	memset(&control_mode, 0, sizeof(control_mode));
	orb_advert_t control_mode_pub = orb_advertise(ORB_ID(vehicle_control_mode), &control_mode);

	/* home position */
    // orb_advert_t home_pub = nullptr;
    // memset(&_home, 0, sizeof(_home));

	orb_advert_t commander_state_pub = nullptr;

	orb_advert_t vehicle_status_flags_pub = nullptr;

    int ret;

	/* Start monitoring loop */
	unsigned counter = 0;

	bool low_battery_voltage_actions_done = false;
	bool critical_battery_voltage_actions_done = false;
	bool emergency_battery_voltage_actions_done = false;

	bool status_changed = true;
	bool param_init_forced = true;

	bool updated = false;

	/* Subscribe to safety topic */
	int safety_sub = orb_subscribe(ORB_ID(safety));
	memset(&safety, 0, sizeof(safety));
	safety.safety_switch_available = false;
	safety.safety_off = false;

    /* Subscribe to manual control data */
	int sp_man_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	memset(&sp_man, 0, sizeof(sp_man));

	/* Subscribe to offboard control data */
	int offboard_control_mode_sub = orb_subscribe(ORB_ID(offboard_control_mode));
	memset(&offboard_control_mode, 0, sizeof(offboard_control_mode));

	/* Subscribe to telemetry status topics */
	int telemetry_subs[ORB_MULTI_MAX_INSTANCES];
	uint64_t telemetry_last_heartbeat[ORB_MULTI_MAX_INSTANCES];
	uint64_t telemetry_last_dl_loss[ORB_MULTI_MAX_INSTANCES];
	bool telemetry_preflight_checks_reported[ORB_MULTI_MAX_INSTANCES];
	bool telemetry_lost[ORB_MULTI_MAX_INSTANCES];

	for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		telemetry_subs[i] = -1;
		telemetry_last_heartbeat[i] = 0;
		telemetry_last_dl_loss[i] = 0;
		telemetry_lost[i] = true;
		telemetry_preflight_checks_reported[i] = false;
	}

	/* Subscribe to global position */
	int global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	struct vehicle_global_position_s global_position;
	memset(&global_position, 0, sizeof(global_position));
	/* Init EPH and EPV */
	global_position.eph = 1000.0f;
	global_position.epv = 1000.0f;

	/* Subscribe to local position data */
	int local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	struct vehicle_local_position_s local_position = {};

	/* Subscribe to attitude data */
	int attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	struct vehicle_attitude_s attitude = {};

	/*
	 * The home position is set based on GPS only, to prevent a dependency between
	 * position estimator and commander. RAW GPS is more than good enough for a
	 * non-flying vehicle.
	 */

	/* Subscribe to GPS topic */
	int gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	struct vehicle_gps_position_s gps_position;
	memset(&gps_position, 0, sizeof(gps_position));
	gps_position.eph = FLT_MAX;
	gps_position.epv = FLT_MAX;

	/* Subscribe to sensor topic */
	int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	struct sensor_combined_s sensors;
	memset(&sensors, 0, sizeof(sensors));

	/* Subscribe to differential pressure topic */
	int diff_pres_sub = orb_subscribe(ORB_ID(differential_pressure));
	struct differential_pressure_s diff_pres;
	memset(&diff_pres, 0, sizeof(diff_pres));

	/* Subscribe to command topic */
	int cmd_sub = orb_subscribe(ORB_ID(vehicle_command));

	/* Subscribe to parameters changed topic */
	int param_changed_sub = orb_subscribe(ORB_ID(parameter_update));

	/* Subscribe to battery topic */
	int battery_sub = orb_subscribe(ORB_ID(battery_status));
	memset(&battery, 0, sizeof(battery));

	/* Subscribe to subsystem info topic */
	int subsys_sub = orb_subscribe(ORB_ID(subsystem_info));
	struct subsystem_info_s info;
	memset(&info, 0, sizeof(info));

	/* Subscribe to position setpoint triplet */
	int pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	struct position_setpoint_triplet_s pos_sp_triplet;
	memset(&pos_sp_triplet, 0, sizeof(pos_sp_triplet));

	/* Subscribe to system power */
	int system_power_sub = orb_subscribe(ORB_ID(system_power));
	struct system_power_s system_power;
	memset(&system_power, 0, sizeof(system_power));

	/* Subscribe to actuator controls (outputs) */
    //int actuator_controls_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	struct actuator_controls_s actuator_controls;
	memset(&actuator_controls, 0, sizeof(actuator_controls));

	int cpuload_sub = orb_subscribe(ORB_ID(cpuload));
	memset(&cpuload, 0, sizeof(cpuload));

	control_status_leds(&status, &armed, true, &battery, &cpuload);

	/* now initialized */
	commander_initialized = true;
	thread_running = true;

	commander_boot_timestamp = hrt_absolute_time();

	// Run preflight check
	int32_t rc_in_off = 0;
	bool hotplug_timeout = hrt_elapsed_time(&commander_boot_timestamp) > HOTPLUG_SENS_TIMEOUT;

	param_get(_param_autostart_id, &autostart_id);
	param_get(_param_rc_in_off, &rc_in_off);

    // int32_t arm_switch_is_button = 0;
    // param_get(_param_arm_switch_is_button, &arm_switch_is_button);

    arm_without_gps = true;
    arm_mission_required = false;

	status.rc_input_mode = rc_in_off;
	if (status.hil_state == vehicle_status_s::HIL_STATE_ON) {
		// HIL configuration selected: real sensors will be disabled
		status_flags.condition_system_sensors_initialized = false;
		set_tune_override(TONE_STARTUP_TUNE); //normal boot tune
	}

	int32_t datalink_loss_timeout = 10;
	float rc_loss_timeout = 0.5;
	int32_t datalink_regain_timeout = 0;
	float offboard_loss_timeout = 0.0f;


	/* RC override auto modes */
    //int32_t rc_override = 0;

	int32_t low_bat_action = 0;

	/* initialize low priority thread */
	pthread_attr_t commander_low_prio_attr;
	pthread_attr_init(&commander_low_prio_attr);
	pthread_attr_setstacksize(&commander_low_prio_attr, PX4_STACK_ADJUSTED(3000));

#ifndef __PX4_QURT
	// This is not supported by QURT (yet).
	struct sched_param param;
	(void)pthread_attr_getschedparam(&commander_low_prio_attr, &param);

	/* low priority */
	param.sched_priority = SCHED_PRIORITY_DEFAULT - 50;
	(void)pthread_attr_setschedparam(&commander_low_prio_attr, &param);
#endif

	pthread_create(&commander_low_prio_thread, &commander_low_prio_attr, commander_low_prio_loop, nullptr);
	pthread_attr_destroy(&commander_low_prio_attr);

	while (!thread_should_exit) {

		/* update parameters */
		orb_check(param_changed_sub, &updated);

		if (updated || param_init_forced) {

			/* parameters changed */
			struct parameter_update_s param_changed;
			orb_copy(ORB_ID(parameter_update), param_changed_sub, &param_changed);

			/* Autostart id */
			param_get(_param_autostart_id, &autostart_id);

            /* EPH / EPV */
            param_get(_param_eph, &eph_threshold);
            param_get(_param_epv, &epv_threshold);

			/* flight mode slots */
			param_get(_param_fmode_1, &_flight_mode_slots[0]);

			param_init_forced = false;
		}

		/* handle power button state */
		orb_check(power_button_state_sub, &updated);

		if (updated) {
			power_button_state_s button_state;
			orb_copy(ORB_ID(power_button_state), power_button_state_sub, &button_state);
			if (button_state.event == power_button_state_s::PWR_BUTTON_STATE_REQUEST_SHUTDOWN) {
				px4_shutdown_request(false, false);
			}
		}

		orb_check(sp_man_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(manual_control_setpoint), sp_man_sub, &sp_man);
		}

		orb_check(offboard_control_mode_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(offboard_control_mode), offboard_control_mode_sub, &offboard_control_mode);
		}

		if (offboard_control_mode.timestamp != 0 &&
		    offboard_control_mode.timestamp + OFFBOARD_TIMEOUT > hrt_absolute_time()) {
			if (status_flags.offboard_control_signal_lost) {
				status_flags.offboard_control_signal_lost = false;
				status_flags.offboard_control_loss_timeout = false;
				status_changed = true;
			}

		} else {
			if (!status_flags.offboard_control_signal_lost) {
				status_flags.offboard_control_signal_lost = true;
				status_changed = true;
			}

			/* check timer if offboard was there but now lost */
			if (!status_flags.offboard_control_loss_timeout && offboard_control_mode.timestamp != 0) {
				if (offboard_loss_timeout < FLT_EPSILON) {
					/* execute loss action immediately */
					status_flags.offboard_control_loss_timeout = true;

				} else {
					/* wait for timeout if set */
					status_flags.offboard_control_loss_timeout = offboard_control_mode.timestamp +
						OFFBOARD_TIMEOUT + offboard_loss_timeout * 1e6f < hrt_absolute_time();
				}

				if (status_flags.offboard_control_loss_timeout) {
					status_changed = true;
				}
			}
		}

		for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {

			if (telemetry_subs[i] < 0 && (OK == orb_exists(ORB_ID(telemetry_status), i))) {
				telemetry_subs[i] = orb_subscribe_multi(ORB_ID(telemetry_status), i);
			}

			orb_check(telemetry_subs[i], &updated);

			if (updated) {
				struct telemetry_status_s telemetry;
				memset(&telemetry, 0, sizeof(telemetry));

				orb_copy(ORB_ID(telemetry_status), telemetry_subs[i], &telemetry);

				/* perform system checks when new telemetry link connected */
				if (/* we first connect a link or re-connect a link after loosing it or haven't yet reported anything */
				    (telemetry_last_heartbeat[i] == 0 || (hrt_elapsed_time(&telemetry_last_heartbeat[i]) > 3 * 1000 * 1000)
				        || !telemetry_preflight_checks_reported[i]) &&
				    /* and this link has a communication partner */
				    (telemetry.heartbeat_time > 0) &&
				    /* and it is still connected */
				    (hrt_elapsed_time(&telemetry.heartbeat_time) < 2 * 1000 * 1000) &&
				    /* and the system is not already armed (and potentially flying) */
				    !armed.armed) {

					hotplug_timeout = hrt_elapsed_time(&commander_boot_timestamp) > HOTPLUG_SENS_TIMEOUT;
					/* flag the checks as reported for this link when we actually report them */
					telemetry_preflight_checks_reported[i] = hotplug_timeout;

					/* provide RC and sensor status feedback to the user */
					if (status.hil_state == vehicle_status_s::HIL_STATE_ON) {
						/* HITL configuration: check only RC input */
						(void)Commander::preflightCheck(&mavlink_log_pub, false, false, false, false, false,
								(status.rc_input_mode == vehicle_status_s::RC_IN_MODE_DEFAULT), false,
								 /* checkDynamic */ true, is_vtol(&status), /* reportFailures */ false, /* prearm */ false, hrt_elapsed_time(&commander_boot_timestamp));
					} else {
						/* check sensors also */
                        (void)Commander::preflightCheck(&mavlink_log_pub, true, true, true, true, false,
								(status.rc_input_mode == vehicle_status_s::RC_IN_MODE_DEFAULT), !arm_without_gps,
								 /* checkDynamic */ true, is_vtol(&status), /* reportFailures */ hotplug_timeout, /* prearm */ false, hrt_elapsed_time(&commander_boot_timestamp));
					}

				}

				/* set (and don't reset) telemetry via USB as active once a MAVLink connection is up */
				if (telemetry.type == telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_USB) {
					_usb_telemetry_active = true;
				}

				if (telemetry.heartbeat_time > 0) {
					telemetry_last_heartbeat[i] = telemetry.heartbeat_time;
				}
			}
		}

		orb_check(sensor_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(sensor_combined), sensor_sub, &sensors);

			/* Check if the barometer is healthy and issue a warning in the GCS if not so.
			 * Because the barometer is used for calculating AMSL altitude which is used to ensure
			 * vertical separation from other airtraffic the operator has to know when the
			 * barometer is inoperational.
			 * */
			hrt_abstime baro_timestamp = sensors.timestamp + sensors.baro_timestamp_relative;
			if (hrt_elapsed_time(&baro_timestamp) < FAILSAFE_DEFAULT_TIMEOUT) {
				/* handle the case where baro was regained */
				if (status_flags.barometer_failure) {
					status_flags.barometer_failure = false;
					status_changed = true;
					if (status_flags.ever_had_barometer_data) {
						mavlink_log_critical(&mavlink_log_pub, "baro healthy");
					}
					status_flags.ever_had_barometer_data = true;
				}

			} else {
				if (!status_flags.barometer_failure) {
					status_flags.barometer_failure = true;
					status_changed = true;
					mavlink_log_critical(&mavlink_log_pub, "baro failed");
				}
			}
		}

		orb_check(diff_pres_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(differential_pressure), diff_pres_sub, &diff_pres);
		}

		orb_check(system_power_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(system_power), system_power_sub, &system_power);

			if (hrt_elapsed_time(&system_power.timestamp) < 200000) {
				if (system_power.servo_valid &&
				    !system_power.brick_valid &&
				    !system_power.usb_connected) {
					/* flying only on servo rail, this is unsafe */
					status_flags.condition_power_input_valid = false;

				} else {
					status_flags.condition_power_input_valid = true;
				}

				/* copy avionics voltage */
				avionics_power_rail_voltage = system_power.voltage5V_v;

				/* if the USB hardware connection went away, reboot */
				if (status_flags.usb_connected && !system_power.usb_connected) {
					/*
					 * apparently the USB cable went away but we are still powered,
					 * so lets reset to a classic non-usb state.
					 */
					mavlink_log_critical(&mavlink_log_pub, "USB disconnected, rebooting.")
					usleep(400000);
					px4_shutdown_request(true, false);
				}

				/* finally judge the USB connected state based on software detection */
				status_flags.usb_connected = _usb_telemetry_active;
			}
		}

		check_valid(diff_pres.timestamp, DIFFPRESS_TIMEOUT, true, &(status_flags.condition_airspeed_valid), &status_changed);

		/* update safety topic */
		orb_check(safety_sub, &updated);

        // Check if quality checking of position accuracy and consistency is to be performed

        /* update global position estimate and check for timeout */
		bool gpos_updated =  false;
		orb_check(global_position_sub, &gpos_updated);
		if (gpos_updated) {
			orb_copy(ORB_ID(vehicle_global_position), global_position_sub, &global_position);
			gpos_last_update_time_us = hrt_absolute_time();
		}

		// Perform a separate timeout validity test on the global position data.
		// This is necessary because the global position message is by definition valid if published.
		if ((hrt_absolute_time() - gpos_last_update_time_us) > 1000000) {
			status_flags.condition_global_position_valid = false;
			status_flags.condition_global_velocity_valid = false;
		}

		/* update local position estimate */
		bool lpos_updated = false;
		orb_check(local_position_sub, &lpos_updated);

		if (lpos_updated) {
			/* position changed */
			orb_copy(ORB_ID(vehicle_local_position), local_position_sub, &local_position);
		}

		/* update attitude estimate */
		orb_check(attitude_sub, &updated);

		if (updated) {
			/* attitude changed */
			orb_copy(ORB_ID(vehicle_attitude), attitude_sub, &attitude);
		}

        if (!warning_action_on) {
			// store the last good main_state when not in an navigation
			// hold state
			main_state_before_rtl = internal_state.main_state;

		} else if (internal_state.main_state != commander_state_s::MAIN_STATE_AUTO_RTL
			&& internal_state.main_state != commander_state_s::MAIN_STATE_AUTO_LOITER
			&& internal_state.main_state != commander_state_s::MAIN_STATE_AUTO_LAND) {
			// reset flag again when we switched out of it
			warning_action_on = false;
		}

		orb_check(cpuload_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(cpuload), cpuload_sub, &cpuload);
		}

		/* update battery status */
		orb_check(battery_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(battery_status), battery_sub, &battery);

			/* only consider battery voltage if system has been running 6s (usb most likely detected) and battery voltage is valid */
			if (hrt_absolute_time() > commander_boot_timestamp + 6000000
			    && battery.voltage_filtered_v > 2.0f * FLT_EPSILON) {

				/* if battery voltage is getting lower, warn using buzzer, etc. */
				if (battery.warning == battery_status_s::BATTERY_WARNING_LOW &&
				   !low_battery_voltage_actions_done) {
					low_battery_voltage_actions_done = true;
					if (armed.armed) {
						mavlink_log_critical(&mavlink_log_pub, "LOW BATTERY, RETURN TO LAND ADVISED");
					} else {
						mavlink_log_critical(&mavlink_log_pub, "LOW BATTERY, TAKEOFF DISCOURAGED");
					}

					status_changed = true;
				} else if (!status_flags.usb_connected &&
					   battery.warning == battery_status_s::BATTERY_WARNING_CRITICAL &&
					   !critical_battery_voltage_actions_done) {
					critical_battery_voltage_actions_done = true;

					if (!armed.armed) {
						mavlink_log_critical(&mavlink_log_pub, "CRITICAL BATTERY, SHUT SYSTEM DOWN");

					} else {
						if (low_bat_action == 1 || low_bat_action == 3) {
							// let us send the critical message even if already in RTL
							if (TRANSITION_CHANGED == main_state_transition(&status, commander_state_s::MAIN_STATE_AUTO_RTL, main_state_prev, &status_flags, &internal_state)) {
								warning_action_on = true;
								mavlink_log_emergency(&mavlink_log_pub, "CRITICAL BATTERY, RETURNING TO LAND");

							} else {
								mavlink_log_emergency(&mavlink_log_pub, "CRITICAL BATTERY, RTL FAILED");
							}

						} else if (low_bat_action == 2) {
							if (TRANSITION_CHANGED == main_state_transition(&status, commander_state_s::MAIN_STATE_AUTO_LAND, main_state_prev, &status_flags, &internal_state)) {
								warning_action_on = true;
								mavlink_log_emergency(&mavlink_log_pub, "CRITICAL BATTERY, LANDING AT CURRENT POSITION");

							} else {
								mavlink_log_emergency(&mavlink_log_pub, "CRITICAL BATTERY, LANDING FAILED");
							}

						} else {
							mavlink_log_emergency(&mavlink_log_pub, "CRITICAL BATTERY, RETURN TO LAUNCH ADVISED!");
						}
					}

					status_changed = true;

				} else if (!status_flags.usb_connected &&
					   battery.warning == battery_status_s::BATTERY_WARNING_EMERGENCY &&
					   !emergency_battery_voltage_actions_done) {
					emergency_battery_voltage_actions_done = true;

					if (!armed.armed) {
						mavlink_log_critical(&mavlink_log_pub, "DANGEROUSLY LOW BATTERY, SHUT SYSTEM DOWN");
						usleep(200000);
						int ret_val = px4_shutdown_request(false, false);
						if (ret_val) {
							mavlink_log_critical(&mavlink_log_pub, "SYSTEM DOES NOT SUPPORT SHUTDOWN");
						} else {
							while(1) { usleep(1); }
						}

					} else {
						if (low_bat_action == 2 || low_bat_action == 3) {
							if (TRANSITION_CHANGED == main_state_transition(&status, commander_state_s::MAIN_STATE_AUTO_LAND, main_state_prev, &status_flags, &internal_state)) {
								warning_action_on = true;
								mavlink_log_emergency(&mavlink_log_pub, "DANGEROUS BATTERY LEVEL, LANDING IMMEDIATELY");

							} else {
								mavlink_log_emergency(&mavlink_log_pub, "DANGEROUS BATTERY LEVEL, LANDING FAILED");
							}

						} else {
							mavlink_log_emergency(&mavlink_log_pub, "DANGEROUS BATTERY LEVEL, LANDING ADVISED!");
						}
					}

					status_changed = true;
				}

				/* End battery voltage check */
			}
		}

		/* update subsystem */
		orb_check(subsys_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(subsystem_info), subsys_sub, &info);

			//warnx("subsystem changed: %d\n", (int)info.subsystem_type);

			/* mark / unmark as present */
			if (info.present) {
				status.onboard_control_sensors_present |= info.subsystem_type;

			} else {
				status.onboard_control_sensors_present &= ~info.subsystem_type;
			}

			/* mark / unmark as enabled */
			if (info.enabled) {
				status.onboard_control_sensors_enabled |= info.subsystem_type;

			} else {
				status.onboard_control_sensors_enabled &= ~info.subsystem_type;
			}

			/* mark / unmark as ok */
			if (info.ok) {
				status.onboard_control_sensors_health |= info.subsystem_type;

			} else {
				status.onboard_control_sensors_health &= ~info.subsystem_type;
			}

			status_changed = true;
		}

		/* update position setpoint triplet */
		orb_check(pos_sp_triplet_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(position_setpoint_triplet), pos_sp_triplet_sub, &pos_sp_triplet);
		}

		/*
		 * Check GPS fix quality. Note that this check augments the position validity
		 * checks and adds an additional level of protection.
		 */

		orb_check(gps_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(vehicle_gps_position), gps_sub, &gps_position);
		}

		/* Initialize map projection if gps is valid */
		if (!map_projection_global_initialized()
		    && (gps_position.eph < eph_threshold)
		    && (gps_position.epv < epv_threshold)
		    && hrt_elapsed_time((hrt_abstime *)&gps_position.timestamp) < 1e6) {
			/* set reference for global coordinates <--> local coordiantes conversion and map_projection */
			globallocalconverter_init((double)gps_position.lat * 1.0e-7, (double)gps_position.lon * 1.0e-7,
						  (float)gps_position.alt * 1.0e-3f, hrt_absolute_time());
		}

		/* check if GPS is ok */
		if (!status_flags.circuit_breaker_engaged_gpsfailure_check) {
			bool gpsIsNoisy = gps_position.noise_per_ms > 0 && gps_position.noise_per_ms < COMMANDER_MAX_GPS_NOISE;

			//Check if GPS receiver is too noisy while we are disarmed
			if (!armed.armed && gpsIsNoisy) {
				if (!status_flags.gps_failure) {
					mavlink_log_critical(&mavlink_log_pub, "GPS signal noisy");
					set_tune_override(TONE_GPS_WARNING_TUNE);

					//GPS suffers from signal jamming or excessive noise, disable GPS-aided flight
					status_flags.gps_failure = true;
					status_changed = true;
				}
			}

			// Check fix type and data freshness
			if (gps_position.fix_type >= 3 && hrt_elapsed_time(&gps_position.timestamp) < FAILSAFE_DEFAULT_TIMEOUT) {
				/* handle the case where gps was regained */
				if (status_flags.gps_failure && !gpsIsNoisy) {
					status_flags.gps_failure = false;
					status_changed = true;
					if (status_flags.condition_home_position_valid) {
						mavlink_log_critical(&mavlink_log_pub, "GPS fix regained");
					}
				}

			} else if (!status_flags.gps_failure) {
				status_flags.gps_failure = true;
				status_changed = true;
				if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
					mavlink_log_critical(&mavlink_log_pub, "GPS fix lost");
				}
			}

		}

        /* RC input check */
        if (!status_flags.rc_input_blocked && sp_man.timestamp != 0 &&
                (hrt_absolute_time() < sp_man.timestamp + (uint64_t)(rc_loss_timeout * 1e6f))) {
            /* handle the case where RC signal was regained */
            if (!status_flags.rc_signal_found_once) {
                status_flags.rc_signal_found_once = true;
                status_changed = true;

            } else {
                if (status.rc_signal_lost) {
                    mavlink_log_info(&mavlink_log_pub, "MANUAL CONTROL REGAINED after %llums",
                                     (hrt_absolute_time() - rc_signal_lost_timestamp) / 1000);
                    status_changed = true;
                }
            }

            status.rc_signal_lost = false;

            /* store last position lock state */
            _last_condition_global_position_valid = status_flags.condition_global_position_valid;

            /* check throttle kill switch */
            if (sp_man.kill_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
                /* set lockdown flag */
                if (!armed.manual_lockdown) {
                    mavlink_log_emergency(&mavlink_log_pub, "MANUAL KILL SWITCH ENGAGED");
                }
                armed.manual_lockdown = true;
            } else if (sp_man.kill_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
                if (armed.manual_lockdown) {
                    mavlink_log_emergency(&mavlink_log_pub, "MANUAL KILL SWITCH OFF");
                }
                armed.manual_lockdown = false;
            }
            /* no else case: do not change lockdown flag in unconfigured case */
        } else {
            if (!status_flags.rc_input_blocked && !status.rc_signal_lost) {
                mavlink_log_critical(&mavlink_log_pub, "MANUAL CONTROL LOST (at t=%llums)", hrt_absolute_time() / 1000);
                status.rc_signal_lost = true;
                rc_signal_lost_timestamp = sp_man.timestamp;
                status_changed = true;
            }
        }

		/* data links check */
		bool have_link = false;

		for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			if (telemetry_last_heartbeat[i] != 0 &&
			    hrt_elapsed_time(&telemetry_last_heartbeat[i]) < datalink_loss_timeout * 1e6) {
				/* handle the case where data link was gained first time or regained,
				 * accept datalink as healthy only after datalink_regain_timeout seconds
				 * */
				if (telemetry_lost[i] &&
				    hrt_elapsed_time(&telemetry_last_dl_loss[i]) > datalink_regain_timeout * 1e6) {

					/* report a regain */
					if (telemetry_last_dl_loss[i] > 0) {
						mavlink_and_console_log_info(&mavlink_log_pub, "data link #%i regained", i);
					} else if (telemetry_last_dl_loss[i] == 0) {
						/* new link */
					}

					/* got link again or new */
					status_flags.condition_system_prearm_error_reported = false;
					status_changed = true;

					telemetry_lost[i] = false;
					have_link = true;

				} else if (!telemetry_lost[i]) {
					/* telemetry was healthy also in last iteration
					 * we don't have to check a timeout */
					have_link = true;
				}

			} else {

				if (!telemetry_lost[i]) {
					/* only reset the timestamp to a different time on state change */
					telemetry_last_dl_loss[i]  = hrt_absolute_time();

					mavlink_and_console_log_info(&mavlink_log_pub, "data link #%i lost", i);
					telemetry_lost[i] = true;
				}
			}
		}

		if (have_link) {
			/* handle the case where data link was regained */
			if (status.data_link_lost) {
				status.data_link_lost = false;
				status_changed = true;
			}

		} else {
			if (!status.data_link_lost) {
				if (armed.armed) {
					mavlink_log_critical(&mavlink_log_pub, "ALL DATA LINKS LOST");
				}
				status.data_link_lost = true;
				status.data_link_lost_counter++;
				status_changed = true;
			}
		}


		/* handle commands last, as the system needs to be updated to handle them */
		orb_check(cmd_sub, &updated);

		if (updated) {
			struct vehicle_command_s cmd;
			/* got command */
			orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);
		}

		/* Get current timestamp */
		const hrt_abstime now = hrt_absolute_time();

		/* publish states (armed, control mode, vehicle status) at least with 5 Hz */
		if (counter % (200000 / COMMANDER_MONITORING_INTERVAL) == 0 || status_changed) {
			set_control_mode();
			control_mode.timestamp = now;
			orb_publish(ORB_ID(vehicle_control_mode), control_mode_pub, &control_mode);

			status.timestamp = now;
			orb_publish(ORB_ID(vehicle_status), status_pub, &status);

			armed.timestamp = now;

			/* set prearmed state if safety is off, or safety is not present and 5 seconds passed */
			if (safety.safety_switch_available) {

				/* safety is off, go into prearmed */
				armed.prearmed = safety.safety_off;
			} else {
				/* safety is not present, go into prearmed
				 * (all output drivers should be started / unlocked last in the boot process
				 * when the rest of the system is fully initialized)
				 */
				armed.prearmed = (hrt_elapsed_time(&commander_boot_timestamp) > 5 * 1000 * 1000);
			}
			orb_publish(ORB_ID(actuator_armed), armed_pub, &armed);
		}



		counter++;

		status_changed = false;

		/* publish vehicle_status_flags */
		publish_status_flags(vehicle_status_flags_pub);

		/* publish internal state for logging purposes */
		if (commander_state_pub != nullptr) {
			orb_publish(ORB_ID(commander_state), commander_state_pub, &internal_state);

		} else {
			commander_state_pub = orb_advertise(ORB_ID(commander_state), &internal_state);
		}

		usleep(COMMANDER_MONITORING_INTERVAL);
	}

	/* wait for threads to complete */
	ret = pthread_join(commander_low_prio_thread, nullptr);

	if (ret) {
		warn("join failed: %d", ret);
	}

	rgbled_set_color_and_mode(led_control_s::COLOR_WHITE, led_control_s::MODE_OFF);

	/* close fds */
	led_deinit();
	buzzer_deinit();
	px4_close(sp_man_sub);
	px4_close(offboard_control_mode_sub);
	px4_close(local_position_sub);
	px4_close(global_position_sub);
	px4_close(gps_sub);
	px4_close(sensor_sub);
	px4_close(safety_sub);
	px4_close(cmd_sub);
	px4_close(subsys_sub);
	px4_close(diff_pres_sub);
	px4_close(param_changed_sub);
	px4_close(battery_sub);

	thread_running = false;

	return 0;
}

void
check_valid(hrt_abstime timestamp, hrt_abstime timeout, bool valid_in, bool *valid_out, bool *changed)
{
	hrt_abstime t = hrt_absolute_time();
	bool valid_new = (t < timestamp + timeout && t > timeout && valid_in);

	if (*valid_out != valid_new) {
		*valid_out = valid_new;
		*changed = true;
	}
}

void control_status_leds(vehicle_status_s *status_local, const actuator_armed_s *actuator_armed,
	bool changed, battery_status_s *battery_local, const cpuload_s *cpuload_local)
{
	static hrt_abstime overload_start = 0;

	bool overload = (cpuload_local->load > 0.80f) || (cpuload_local->ram_usage > 0.98f);

	if (overload_start == 0 && overload) {
		overload_start = hrt_absolute_time();
	} else if (!overload) {
		overload_start = 0;
	}

	/* driving rgbled */
	if (changed || last_overload != overload) {
		uint8_t led_mode = led_control_s::MODE_OFF;
		uint8_t led_color = led_control_s::COLOR_WHITE;
		bool set_normal_color = false;
		bool hotplug_timeout = hrt_elapsed_time(&commander_boot_timestamp) > HOTPLUG_SENS_TIMEOUT;

		int overload_warn_delay = (status_local->arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 1000 : 250000;

		/* set mode */
		if (overload && ((hrt_absolute_time() - overload_start) > overload_warn_delay)) {
			led_mode = led_control_s::MODE_BLINK_FAST;
			led_color = led_control_s::COLOR_PURPLE;

		} else if (status_local->arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			led_mode = led_control_s::MODE_ON;
			set_normal_color = true;

		} else if (status_local->arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR ||
				(!status_flags.condition_system_sensors_initialized && hotplug_timeout)) {
			led_mode = led_control_s::MODE_BLINK_FAST;
			led_color = led_control_s::COLOR_RED;

		} else if (status_local->arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
			led_mode = led_control_s::MODE_BREATHE;
			set_normal_color = true;

		} else if (!status_flags.condition_system_sensors_initialized && !hotplug_timeout) {
			led_mode = led_control_s::MODE_BREATHE;
			set_normal_color = true;

		} else if (status_local->arming_state == vehicle_status_s::ARMING_STATE_INIT) {
			// if in init status it should not be in the error state
			led_mode = led_control_s::MODE_OFF;

		} else {	// STANDBY_ERROR and other states
			led_mode = led_control_s::MODE_BLINK_NORMAL;
			led_color = led_control_s::COLOR_RED;
		}

		if (set_normal_color) {
			/* set color */
			if (status.failsafe) {
				led_color = led_control_s::COLOR_PURPLE;

			} else if (battery_local->warning == battery_status_s::BATTERY_WARNING_LOW) {
				led_color = led_control_s::COLOR_AMBER;
			} else if (battery_local->warning == battery_status_s::BATTERY_WARNING_CRITICAL) {
				led_color = led_control_s::COLOR_RED;
			} else {
				if (status_flags.condition_home_position_valid && status_flags.condition_global_position_valid) {
					led_color = led_control_s::COLOR_GREEN;

				} else {
					led_color = led_control_s::COLOR_BLUE;
				}
			}
		}
		if (led_mode != led_control_s::MODE_OFF) {
			rgbled_set_color_and_mode(led_color, led_mode);
		}
	}

	last_overload = overload;

#if defined (CONFIG_ARCH_BOARD_PX4FMU_V1) || defined (CONFIG_ARCH_BOARD_PX4FMU_V4) || defined (CONFIG_ARCH_BOARD_CRAZYFLIE) || defined (CONFIG_ARCH_BOARD_AEROFC_V1) || defined (CONFIG_ARCH_BOARD_AEROCORE2)

	/* this runs at around 20Hz, full cycle is 16 ticks = 10/16Hz */
	if (actuator_armed->armed) {
		if (status.failsafe) {
			led_off(LED_BLUE);
			if (leds_counter % 5 == 0) {
				led_toggle(LED_GREEN);
			}
		} else {
			led_off(LED_GREEN);

			/* armed, solid */
			led_on(LED_BLUE);
		}

	} else if (actuator_armed->ready_to_arm) {
		led_off(LED_BLUE);
		/* ready to arm, blink at 1Hz */
		if (leds_counter % 20 == 0) {
			led_toggle(LED_GREEN);
		}

	} else {
		led_off(LED_BLUE);
		/* not ready to arm, blink at 10Hz */
		if (leds_counter % 2 == 0) {
			led_toggle(LED_GREEN);
		}
	}

#endif

	/* give system warnings on error LED */
	if (overload) {
		if (leds_counter % 2 == 0) {
			led_toggle(LED_AMBER);
		}

	} else {
		led_off(LED_AMBER);
	}

	leds_counter++;
}

transition_result_t set_main_state_rc(struct vehicle_status_s *status_local, vehicle_global_position_s *global_position, vehicle_local_position_s *local_position, bool *changed)
{
	/* set main state according to RC switches */
	transition_result_t res = TRANSITION_DENIED;

	// Note: even if status_flags.offboard_control_set_by_command is set
	// we want to allow rc mode change to take precidence.  This is a safety
	// feature, just in case offboard control goes crazy.

	/* manual setpoint has not updated, do not re-evaluate it */
	if (!(!_last_condition_global_position_valid &&
		status_flags.condition_global_position_valid)
		&& (((_last_sp_man.timestamp != 0) && (_last_sp_man.timestamp == sp_man.timestamp)) ||
		((_last_sp_man.offboard_switch == sp_man.offboard_switch) &&
		 (_last_sp_man.return_switch == sp_man.return_switch) &&
		 (_last_sp_man.mode_switch == sp_man.mode_switch) &&
		 (_last_sp_man.acro_switch == sp_man.acro_switch) &&
		 (_last_sp_man.rattitude_switch == sp_man.rattitude_switch) &&
		 (_last_sp_man.posctl_switch == sp_man.posctl_switch) &&
		 (_last_sp_man.loiter_switch == sp_man.loiter_switch) &&
		 (_last_sp_man.mode_slot == sp_man.mode_slot) &&
		 (_last_sp_man.stab_switch == sp_man.stab_switch) &&
		 (_last_sp_man.man_switch == sp_man.man_switch)))) {

		// store the last manual control setpoint set by the pilot in a manual state
		// if the system now later enters an autonomous state the pilot can move
		// the sticks to break out of the autonomous state

		if (!warning_action_on
			&& (internal_state.main_state == commander_state_s::MAIN_STATE_MANUAL ||
			internal_state.main_state == commander_state_s::MAIN_STATE_ALTCTL ||
			internal_state.main_state == commander_state_s::MAIN_STATE_POSCTL ||
			internal_state.main_state == commander_state_s::MAIN_STATE_ACRO ||
			internal_state.main_state == commander_state_s::MAIN_STATE_RATTITUDE ||
			internal_state.main_state == commander_state_s::MAIN_STATE_STAB)) {

			_last_sp_man.timestamp = sp_man.timestamp;
			_last_sp_man.x = sp_man.x;
			_last_sp_man.y = sp_man.y;
			_last_sp_man.z = sp_man.z;
			_last_sp_man.r = sp_man.r;
		}

		/* no timestamp change or no switch change -> nothing changed */
		return TRANSITION_NOT_CHANGED;
	}

	_last_sp_man = sp_man;

	/* offboard switch overrides main switch */
	if (sp_man.offboard_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_OFFBOARD, main_state_prev, &status_flags, &internal_state);

		if (res == TRANSITION_DENIED) {
			print_reject_mode(status_local, "OFFBOARD");
			/* mode rejected, continue to evaluate the main system mode */

		} else {
			/* changed successfully or already in this state */
			return res;
		}
	}

	/* RTL switch overrides main switch */
	if (sp_man.return_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_RTL, main_state_prev, &status_flags, &internal_state);

		if (res == TRANSITION_DENIED) {
			print_reject_mode(status_local, "AUTO RTL");

			/* fallback to LOITER if home position not set */
			res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_LOITER, main_state_prev, &status_flags, &internal_state);
		}

		if (res != TRANSITION_DENIED) {
			/* changed successfully or already in this state */
			return res;
		}

		/* if we get here mode was rejected, continue to evaluate the main system mode */
	}

	/* Loiter switch overrides main switch */
	if (sp_man.loiter_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_LOITER, main_state_prev, &status_flags, &internal_state);

		if (res == TRANSITION_DENIED) {
			print_reject_mode(status_local, "AUTO HOLD");

		} else {
			return res;
		}
	}

	/* we know something has changed - check if we are in mode slot operation */
	if (sp_man.mode_slot != manual_control_setpoint_s::MODE_SLOT_NONE) {

		if (sp_man.mode_slot >= sizeof(_flight_mode_slots) / sizeof(_flight_mode_slots[0])) {
			warnx("m slot overflow");
			return TRANSITION_DENIED;
		}

		int new_mode = _flight_mode_slots[sp_man.mode_slot];

		if (new_mode < 0) {
			/* slot is unused */
			res = TRANSITION_NOT_CHANGED;

		} else {
			res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

			/* ensure that the mode selection does not get stuck here */
			int maxcount = 5;

			/* enable the use of break */
			/* fallback strategies, give the user the closest mode to what he wanted */
			while (res == TRANSITION_DENIED && maxcount > 0) {

				maxcount--;

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_MISSION) {

					/* fall back to loiter */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode(status_local, "AUTO MISSION");
					res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_RTL) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode(status_local, "AUTO RTL");
					res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_LAND) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode(status_local, "AUTO LAND");
					res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_TAKEOFF) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode(status_local, "AUTO TAKEOFF");
					res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_AUTO_LOITER;
					print_reject_mode(status_local, "AUTO FOLLOW");
					res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_AUTO_LOITER) {

					/* fall back to position control */
					new_mode = commander_state_s::MAIN_STATE_POSCTL;
					print_reject_mode(status_local, "AUTO HOLD");
					res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_POSCTL) {

					/* fall back to altitude control */
					new_mode = commander_state_s::MAIN_STATE_ALTCTL;
					print_reject_mode(status_local, "POSITION CONTROL");
					res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_ALTCTL) {

					/* fall back to stabilized */
					new_mode = commander_state_s::MAIN_STATE_STAB;
					print_reject_mode(status_local, "ALTITUDE CONTROL");
					res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}

				if (new_mode == commander_state_s::MAIN_STATE_STAB) {

					/* fall back to manual */
					new_mode = commander_state_s::MAIN_STATE_MANUAL;
					print_reject_mode(status_local, "STABILIZED");
					res = main_state_transition(status_local, new_mode, main_state_prev, &status_flags, &internal_state);

					if (res != TRANSITION_DENIED) {
						break;
					}
				}
			}
		}

		return res;
	}

	/* offboard and RTL switches off or denied, check main mode switch */
	switch (sp_man.mode_switch) {
	case manual_control_setpoint_s::SWITCH_POS_NONE:
		res = TRANSITION_NOT_CHANGED;
		break;

	case manual_control_setpoint_s::SWITCH_POS_OFF:		// MANUAL
		if (sp_man.stab_switch == manual_control_setpoint_s::SWITCH_POS_NONE &&
			sp_man.man_switch == manual_control_setpoint_s::SWITCH_POS_NONE) {
			/*
			 * Legacy mode:
			 * Acro switch being used as stabilized switch in FW.
			 */
			if (sp_man.acro_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				/* manual mode is stabilized already for multirotors, so switch to acro
				 * for any non-manual mode
				 */
				if (status.is_rotary_wing && !status.is_vtol) {
					res = main_state_transition(status_local, commander_state_s::MAIN_STATE_ACRO, main_state_prev, &status_flags, &internal_state);

				} else if (!status.is_rotary_wing) {
					res = main_state_transition(status_local, commander_state_s::MAIN_STATE_STAB, main_state_prev, &status_flags, &internal_state);

				} else {
					res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, main_state_prev, &status_flags, &internal_state);
				}

			} else if (sp_man.rattitude_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				/* Similar to acro transitions for multirotors.  FW aircraft don't need a
				 * rattitude mode.*/
				if (status.is_rotary_wing) {
					res = main_state_transition(status_local, commander_state_s::MAIN_STATE_RATTITUDE, main_state_prev, &status_flags, &internal_state);

				} else {
					res = main_state_transition(status_local, commander_state_s::MAIN_STATE_STAB, main_state_prev, &status_flags, &internal_state);
				}

			} else {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, main_state_prev, &status_flags, &internal_state);
			}

		} else {
			/* New mode:
			 * - Acro is Acro
			 * - Manual is not default anymore when the manaul switch is assigned
			 */
			if (sp_man.man_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, main_state_prev, &status_flags, &internal_state);

			} else if (sp_man.acro_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_ACRO, main_state_prev, &status_flags, &internal_state);

			} else if (sp_man.rattitude_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_RATTITUDE, main_state_prev, &status_flags, &internal_state);

			} else if (sp_man.stab_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_STAB, main_state_prev, &status_flags, &internal_state);

			} else if (sp_man.man_switch == manual_control_setpoint_s::SWITCH_POS_NONE) {
				// default to MANUAL when no manual switch is set
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, main_state_prev, &status_flags, &internal_state);

			} else {
				// default to STAB when the manual switch is assigned (but off)
				res = main_state_transition(status_local, commander_state_s::MAIN_STATE_STAB, main_state_prev, &status_flags, &internal_state);
			}
		}

		// TRANSITION_DENIED is not possible here
		break;

	case manual_control_setpoint_s::SWITCH_POS_MIDDLE:		// ASSIST
		if (sp_man.posctl_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
			res = main_state_transition(status_local, commander_state_s::MAIN_STATE_POSCTL, main_state_prev, &status_flags, &internal_state);

			if (res != TRANSITION_DENIED) {
				break;	// changed successfully or already in this state
			}

			print_reject_mode(status_local, "POSITION CONTROL");
		}

		// fallback to ALTCTL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_ALTCTL, main_state_prev, &status_flags, &internal_state);

		if (res != TRANSITION_DENIED) {
			break;	// changed successfully or already in this mode
		}

		if (sp_man.posctl_switch != manual_control_setpoint_s::SWITCH_POS_ON) {
			print_reject_mode(status_local, "ALTITUDE CONTROL");
		}

		// fallback to MANUAL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, main_state_prev, &status_flags, &internal_state);
		// TRANSITION_DENIED is not possible here
		break;

	case manual_control_setpoint_s::SWITCH_POS_ON:			// AUTO
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_MISSION, main_state_prev, &status_flags, &internal_state);

		if (res != TRANSITION_DENIED) {
			break;	// changed successfully or already in this state
		}

		print_reject_mode(status_local, "AUTO MISSION");

		// fallback to LOITER if home position not set
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_AUTO_LOITER, main_state_prev, &status_flags, &internal_state);

		if (res != TRANSITION_DENIED) {
			break;  // changed successfully or already in this state
		}

		// fallback to POSCTL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_POSCTL, main_state_prev, &status_flags, &internal_state);

		if (res != TRANSITION_DENIED) {
			break;  // changed successfully or already in this state
		}

		// fallback to ALTCTL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_ALTCTL, main_state_prev, &status_flags, &internal_state);

		if (res != TRANSITION_DENIED) {
			break;	// changed successfully or already in this state
		}

		// fallback to MANUAL
		res = main_state_transition(status_local, commander_state_s::MAIN_STATE_MANUAL, main_state_prev, &status_flags, &internal_state);
		// TRANSITION_DENIED is not possible here
		break;

	default:
		break;
	}

	return res;
}

void set_control_mode(){

	/* set vehicle_control_mode according to set_navigation_state */
	control_mode.flag_armed = armed.armed;
    control_mode.flag_external_manual_override_ok = false;
	control_mode.flag_system_hil_enabled = status.hil_state == vehicle_status_s::HIL_STATE_ON;
    control_mode.flag_control_offboard_enabled = true;

	switch (status.nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_auto_enabled = false;
        control_mode.flag_control_rates_enabled = false;
        control_mode.flag_control_attitude_enabled = false;
		control_mode.flag_control_rattitude_enabled = false;
		control_mode.flag_control_altitude_enabled = false;
		control_mode.flag_control_climb_rate_enabled = false;
		control_mode.flag_control_position_enabled = false;
		control_mode.flag_control_velocity_enabled = false;
		control_mode.flag_control_acceleration_enabled = false;
		control_mode.flag_control_termination_enabled = false;
		break;
	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_rattitude_enabled = false;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_position_enabled = !status.in_transition_mode;
		control_mode.flag_control_velocity_enabled = !status.in_transition_mode;
		control_mode.flag_control_acceleration_enabled = false;
		control_mode.flag_control_termination_enabled = false;
		break;
    case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_offboard_enabled = true;

		/*
		 * The control flags depend on what is ignored according to the offboard control mode topic
		 * Inner loop flags (e.g. attitude) also depend on outer loop ignore flags (e.g. position)
		 */
		control_mode.flag_control_rates_enabled = !offboard_control_mode.ignore_bodyrate ||
			!offboard_control_mode.ignore_attitude ||
			!offboard_control_mode.ignore_position ||
			!offboard_control_mode.ignore_velocity ||
			!offboard_control_mode.ignore_acceleration_force;

		control_mode.flag_control_attitude_enabled = !offboard_control_mode.ignore_attitude ||
			!offboard_control_mode.ignore_position ||
			!offboard_control_mode.ignore_velocity ||
			!offboard_control_mode.ignore_acceleration_force;

		control_mode.flag_control_rattitude_enabled = false;

		control_mode.flag_control_acceleration_enabled = !offboard_control_mode.ignore_acceleration_force &&
		  !status.in_transition_mode;

		control_mode.flag_control_velocity_enabled = (!offboard_control_mode.ignore_velocity ||
			!offboard_control_mode.ignore_position) && !status.in_transition_mode &&
			!control_mode.flag_control_acceleration_enabled;

		control_mode.flag_control_climb_rate_enabled = (!offboard_control_mode.ignore_velocity ||
			!offboard_control_mode.ignore_position) && !control_mode.flag_control_acceleration_enabled;

		control_mode.flag_control_position_enabled = !offboard_control_mode.ignore_position && !status.in_transition_mode &&
		  !control_mode.flag_control_acceleration_enabled;

		control_mode.flag_control_altitude_enabled = (!offboard_control_mode.ignore_velocity ||
			!offboard_control_mode.ignore_position) && !control_mode.flag_control_acceleration_enabled;

		break;

	default:
		break;
	}
}

void
print_reject_mode(struct vehicle_status_s *status_local, const char *msg)
{
	hrt_abstime t = hrt_absolute_time();

	if (t - last_print_mode_reject_time > PRINT_MODE_REJECT_INTERVAL) {
		last_print_mode_reject_time = t;
		mavlink_log_critical(&mavlink_log_pub, "REJECT %s", msg);

		/* only buzz if armed, because else we're driving people nuts indoors
		they really need to look at the leds as well. */
		tune_negative(armed.armed);
	}
}

void answer_command(struct vehicle_command_s &cmd, unsigned result,
					orb_advert_t &command_ack_pub)
{
	switch (result) {
	case vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED:
		tune_positive(true);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_DENIED:
		tune_negative(true);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_FAILED:
		tune_negative(true);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		tune_negative(true);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED:
		tune_negative(true);
		break;

	default:
		break;
	}

	/* publish ACK */
	vehicle_command_ack_s command_ack = {
		.timestamp = 0,
		.command = cmd.command,
		.result = (uint8_t)result
	};

	if (command_ack_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_command_ack), command_ack_pub, &command_ack);

	} else {
		command_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &command_ack, vehicle_command_ack_s::ORB_QUEUE_LENGTH);
	}
}

void *commander_low_prio_loop(void *arg)
{
	/* Set thread name */
	px4_prctl(PR_SET_NAME, "commander_low_prio", px4_getpid());

	/* Subscribe to command topic */
	int cmd_sub = orb_subscribe(ORB_ID(vehicle_command));

	/* command ack */
	orb_advert_t command_ack_pub = nullptr;

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[1];

	/* use the gyro to pace output - XXX BROKEN if we are using the L3GD20 */
	fds[0].fd = cmd_sub;
	fds[0].events = POLLIN;

	while (!thread_should_exit) {
		/* wait for up to 1000ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret < 0) {
			/* this is undesirable but not much we can do - might want to flag unhappy status */
			warn("commander: poll error %d, %d", pret, errno);
			continue;
		} else if (pret != 0) {
			struct vehicle_command_s cmd;

			/* if we reach here, we have a valid command */
			orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);

			/* ignore commands the high-prio loop or the navigator handles */
			if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_MODE ||
			    cmd.command == vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM ||
			    cmd.command == vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF ||
			    cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_SERVO ||
			    cmd.command == vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED) {

				continue;
			}

			/* only handle low-priority commands here */
			switch (cmd.command) {

			case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
				if (is_safe(&safety, &armed)) {

					if (((int)(cmd.param1)) == 1) {
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						usleep(100000);
						/* reboot */
						px4_shutdown_request(true, false);

					} else if (((int)(cmd.param1)) == 3) {
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						usleep(100000);
						/* reboot to bootloader */
						px4_shutdown_request(true, true);

					} else {
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED, command_ack_pub);
					}

				} else {
					answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED, command_ack_pub);
				}

				break;

			case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION: {

					int calib_ret = PX4_ERROR;

					/* try to go to INIT/PREFLIGHT arming state */
					if (TRANSITION_DENIED == arming_state_transition(&status,
											 &battery,
											 &safety,
											 vehicle_status_s::ARMING_STATE_INIT,
											 &armed,
											 false /* fRunPreArmChecks */,
											 &mavlink_log_pub,
											 &status_flags,
											 avionics_power_rail_voltage,
											 arm_without_gps,
											 arm_mission_required,
											 hrt_elapsed_time(&commander_boot_timestamp))) {

						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED, command_ack_pub);
						break;
					} else {
						status_flags.condition_calibration_enabled = true;
					}

					if ((int)(cmd.param1) == 1) {
						/* gyro calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						calib_ret = do_gyro_calibration(&mavlink_log_pub);

					} else if ((int)(cmd.param1) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION ||
							(int)(cmd.param5) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION ||
							(int)(cmd.param7) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION) {
						/* temperature calibration: handled in events module */
						break;

					} else if ((int)(cmd.param2) == 1) {
						/* magnetometer calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						calib_ret = do_mag_calibration(&mavlink_log_pub);

					} else if ((int)(cmd.param3) == 1) {
						/* zero-altitude pressure calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED, command_ack_pub);

					} else if ((int)(cmd.param4) == 1) {
						/* RC calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						/* disable RC control input completely */
						status_flags.rc_input_blocked = true;
						calib_ret = OK;
						mavlink_log_info(&mavlink_log_pub, "CAL: Disabling RC IN");

					} else if ((int)(cmd.param4) == 2) {
						/* RC trim calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						calib_ret = do_trim_calibration(&mavlink_log_pub);

					} else if ((int)(cmd.param5) == 1) {
						/* accelerometer calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						calib_ret = do_accel_calibration(&mavlink_log_pub);
					} else if ((int)(cmd.param5) == 2) {
						// board offset calibration
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						calib_ret = do_level_calibration(&mavlink_log_pub);
					} else if ((int)(cmd.param6) == 1 || (int)(cmd.param6) == 2) {
						// TODO: param6 == 1 is deprecated, but we still accept it for a while (feb 2017)
						/* airspeed calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						calib_ret = do_airspeed_calibration(&mavlink_log_pub);

					} else if ((int)(cmd.param7) == 1) {
						/* do esc calibration */
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						calib_ret = do_esc_calibration(&mavlink_log_pub, &armed);

					} else if ((int)(cmd.param4) == 0) {
						/* RC calibration ended - have we been in one worth confirming? */
						if (status_flags.rc_input_blocked) {
							/* enable RC control input */
							status_flags.rc_input_blocked = false;
							mavlink_log_info(&mavlink_log_pub, "CAL: Re-enabling RC IN");
						}
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
						/* this always succeeds */
						calib_ret = OK;
					} else {
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED, command_ack_pub);
					}

					status_flags.condition_calibration_enabled = false;

					if (calib_ret == OK) {
						tune_positive(true);

						// Update preflight check status
						// we do not set the calibration return value based on it because the calibration
						// might have worked just fine, but the preflight check fails for a different reason,
						// so this would be prone to false negatives.

						bool checkAirspeed = false;
						bool hotplug_timeout = hrt_elapsed_time(&commander_boot_timestamp) > HOTPLUG_SENS_TIMEOUT;
						/* Perform airspeed check only if circuit breaker is not
						 * engaged and it's not a rotary wing */
						if (!status_flags.circuit_breaker_engaged_airspd_check &&
						    (!status.is_rotary_wing || status.is_vtol)) {
							checkAirspeed = true;
						}

						status_flags.condition_system_sensors_initialized = Commander::preflightCheck(&mavlink_log_pub, true, true, true, true, checkAirspeed,
							!(status.rc_input_mode >= vehicle_status_s::RC_IN_MODE_OFF), !arm_without_gps,
							/* checkDynamic */ true, is_vtol(&status), /* reportFailures */ hotplug_timeout, /* prearm */ false, hrt_elapsed_time(&commander_boot_timestamp));

						arming_state_transition(&status,
									&battery,
									&safety,
									vehicle_status_s::ARMING_STATE_STANDBY,
									&armed,
									false /* fRunPreArmChecks */,
									&mavlink_log_pub,
									&status_flags,
									avionics_power_rail_voltage,
									arm_without_gps,
									arm_mission_required,
									hrt_elapsed_time(&commander_boot_timestamp));

					} else {
						tune_negative(true);
					}

					break;
				}

			case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_STORAGE: {

					if (((int)(cmd.param1)) == 0) {
						int ret = param_load_default();

						if (ret == OK) {
							mavlink_log_info(&mavlink_log_pub, "settings loaded");
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);

						} else {
							mavlink_log_critical(&mavlink_log_pub, "settings load ERROR");

							/* convenience as many parts of NuttX use negative errno */
							if (ret < 0) {
								ret = -ret;
							}

							if (ret < 1000) {
								mavlink_log_critical(&mavlink_log_pub, "ERROR: %s", strerror(ret));
							}

							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED, command_ack_pub);
						}

					} else if (((int)(cmd.param1)) == 1) {

#ifdef __PX4_QURT
						// TODO FIXME: on snapdragon the save happens too early when the params
						// are not set yet. We therefore need to wait some time first.
						usleep(1000000);
#endif

						int ret = param_save_default();

						if (ret == OK) {
							/* do not spam MAVLink, but provide the answer / green led mechanism */
							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);

						} else {
							mavlink_log_critical(&mavlink_log_pub, "settings save error");

							/* convenience as many parts of NuttX use negative errno */
							if (ret < 0) {
								ret = -ret;
							}

							if (ret < 1000) {
								mavlink_log_critical(&mavlink_log_pub, "ERROR: %s", strerror(ret));
							}

							answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_FAILED, command_ack_pub);
						}
					} else if (((int)(cmd.param1)) == 2) {

						/* reset parameters and save empty file */
						param_reset_all();

						/* do not spam MAVLink, but provide the answer / green led mechanism */
						mavlink_log_critical(&mavlink_log_pub, "onboard parameters reset");
						answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
					}

					break;
				}

			case vehicle_command_s::VEHICLE_CMD_START_RX_PAIR:
				/* just ack, implementation handled in the IO driver */
				answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED, command_ack_pub);
				break;

			default:
				/* don't answer on unsupported commands, it will be done in main loop */
				break;
			}
		}
	}

	px4_close(cmd_sub);

	return nullptr;
}

void publish_status_flags(orb_advert_t &vehicle_status_flags_pub) {
	struct vehicle_status_flags_s v_flags;
	memset(&v_flags, 0, sizeof(v_flags));
	/* set condition status flags */
	if (status_flags.condition_calibration_enabled) {
		v_flags.conditions |= vehicle_status_flags_s::CONDITION_CALIBRATION_ENABLE_MASK;
	}
	if (status_flags.condition_system_sensors_initialized) {
		v_flags.conditions |= vehicle_status_flags_s::CONDITION_SYSTEM_SENSORS_INITIALIZED_MASK;
	}
	if (status_flags.condition_system_prearm_error_reported) {
		v_flags.conditions |= vehicle_status_flags_s::CONDITION_SYSTEM_PREARM_ERROR_REPORTED_MASK;
	}
	if (status_flags.condition_system_hotplug_timeout) {
		v_flags.conditions |= vehicle_status_flags_s::CONDITION_SYSTEM_HOTPLUG_TIMEOUT_MASK;
	}
	if (status_flags.condition_system_returned_to_home) {
		v_flags.conditions |= vehicle_status_flags_s::CONDITION_SYSTEM_RETURNED_TO_HOME_MASK;
	}
	if (status_flags.condition_auto_mission_available) {
		v_flags.conditions |= vehicle_status_flags_s::CONDITION_AUTO_MISSION_AVAILABLE_MASK;
	}
	if (status_flags.condition_global_position_valid) {
		v_flags.conditions |= vehicle_status_flags_s::CONDITION_GLOBAL_POSITION_VALID_MASK;
	}
	if (status_flags.condition_home_position_valid) {
		v_flags.conditions |= vehicle_status_flags_s::CONDITION_HOME_POSITION_VALID_MASK;
	}
	if (status_flags.condition_local_position_valid) {
		v_flags.conditions |= vehicle_status_flags_s::CONDITION_LOCAL_POSITION_VALID_MASK;
	}
	if (status_flags.condition_local_altitude_valid) {
		v_flags.conditions |= vehicle_status_flags_s::CONDITION_LOCAL_ALTITUDE_VALID_MASK;
	}
	if (status_flags.condition_local_altitude_valid) {
		v_flags.conditions |= vehicle_status_flags_s::CONDITION_LOCAL_ALTITUDE_VALID_MASK;
	}
	if (status_flags.condition_airspeed_valid) {
		v_flags.conditions |= vehicle_status_flags_s::CONDITION_AIRSPEED_VALID_MASK;
	}
	if (status_flags.condition_power_input_valid) {
		v_flags.conditions |= vehicle_status_flags_s::CONDITION_POWER_INPUT_VALID_MASK;
	}

	/* set circuit breaker status flags */
	if (status_flags.circuit_breaker_engaged_power_check) {
		v_flags.circuit_breakers |= vehicle_status_flags_s::CIRCUIT_BREAKER_ENGAGED_POWER_CHECK_MASK;
	}
	if (status_flags.circuit_breaker_engaged_airspd_check) {
		v_flags.circuit_breakers |= vehicle_status_flags_s::CIRCUIT_BREAKER_ENGAGED_AIRSPD_CHECK_MASK;
	}
	if (status_flags.circuit_breaker_engaged_enginefailure_check) {
		v_flags.circuit_breakers |= vehicle_status_flags_s::CIRCUIT_BREAKER_ENGAGED_ENGINEFAILURE_CHECK_MASK;
	}
	if (status_flags.circuit_breaker_engaged_gpsfailure_check) {
		v_flags.circuit_breakers |= vehicle_status_flags_s::CIRCUIT_BREAKER_ENGAGED_GPSFAILURE_CHECK_MASK;
	}
	if (status_flags.circuit_breaker_flight_termination_disabled) {
		v_flags.circuit_breakers |= vehicle_status_flags_s::CIRCUIT_BREAKER_FLIGHT_TERMINATION_DISABLED_MASK;
	}
	if (status_flags.circuit_breaker_engaged_usb_check) {
		v_flags.circuit_breakers |= vehicle_status_flags_s::CIRCUIT_BREAKER_ENGAGED_USB_CHECK_MASK;
	}

	/* set other status flags */
	if (status_flags.usb_connected) {
		v_flags.other_flags |= vehicle_status_flags_s::USB_CONNECTED_MASK;
	}
	if (status_flags.offboard_control_signal_found_once) {
		v_flags.other_flags |= vehicle_status_flags_s::OFFBOARD_CONTROL_SIGNAL_FOUND_ONCE_MASK;
	}
	if (status_flags.offboard_control_signal_lost) {
		v_flags.other_flags |= vehicle_status_flags_s::OFFBOARD_CONTROL_SIGNAL_LOST_MASK;
	}
	if (status_flags.offboard_control_set_by_command) {
		v_flags.other_flags |= vehicle_status_flags_s::OFFBOARD_CONTROL_SET_BY_COMMAND_MASK;
	}
	if (status_flags.offboard_control_loss_timeout) {
		v_flags.other_flags |= vehicle_status_flags_s::OFFBOARD_CONTROL_LOSS_TIMEOUT_MASK;
	}
	if (status_flags.rc_signal_found_once) {
		v_flags.other_flags |= vehicle_status_flags_s::RC_SIGNAL_FOUND_ONCE_MASK;
	}
	if (status_flags.rc_signal_lost_cmd) {
		v_flags.other_flags |= vehicle_status_flags_s::RC_SIGNAL_LOST_CMD_MASK;
	}
	if (status_flags.rc_input_blocked) {
		v_flags.other_flags |= vehicle_status_flags_s::RC_INPUT_BLOCKED_MASK;
	}
	if (status_flags.data_link_lost_cmd) {
		v_flags.other_flags |= vehicle_status_flags_s::DATA_LINK_LOST_CMD_MASK;
	}
	if (status_flags.vtol_transition_failure) {
		v_flags.other_flags |= vehicle_status_flags_s::VTOL_TRANSITION_FAILURE_MASK;
	}
	if (status_flags.vtol_transition_failure_cmd) {
		v_flags.other_flags |= vehicle_status_flags_s::VTOL_TRANSITION_FAILURE_CMD_MASK;
	}
	if (status_flags.gps_failure) {
		v_flags.other_flags |= vehicle_status_flags_s::GPS_FAILURE_MASK;
	}
	if (status_flags.gps_failure_cmd) {
		v_flags.other_flags |= vehicle_status_flags_s::GPS_FAILURE_CMD_MASK;
	}
	if (status_flags.barometer_failure) {
		v_flags.other_flags |= vehicle_status_flags_s::BAROMETER_FAILURE_MASK;
	}
	if (status_flags.ever_had_barometer_data) {
		v_flags.other_flags |= vehicle_status_flags_s::EVER_HAD_BAROMETER_DATA_MASK;
	}

	/* publish vehicle_status_flags */
	if (vehicle_status_flags_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_status_flags), vehicle_status_flags_pub, &v_flags);
	} else {
		vehicle_status_flags_pub = orb_advertise(ORB_ID(vehicle_status_flags), &v_flags);
	}
}
