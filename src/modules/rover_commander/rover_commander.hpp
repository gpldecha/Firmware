#ifndef ROVER_COMMANDER_H_
#define ROVER_COMMANDER_H_

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
#include "state_machine.h"


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

#include <uORB/uORB.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/power_button_state.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/vehicle_command.h>

#include <uORB/topics/rover_commander_state.h>
#include <uORB/topics/rover_control_mode.h>

#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_local_position.h>

#include <uORB/topics/vehicle_status_flags.h>

#include <uORB/topics/rover_status.h>




/* Decouple update interval and hysteresis counters, all depends on intervals */
#define COMMANDER_MONITORING_INTERVAL 10000
#define COMMANDER_MONITORING_LOOPSPERMSEC (1/(COMMANDER_MONITORING_INTERVAL/1000.0f))

#define POSITION_TIMEOUT                (1 * 1000 * 1000)	/**< consider the local or global position estimate invalid after 1000ms */
#define FAILSAFE_DEFAULT_TIMEOUT        (3 * 1000 * 1000)	/**< hysteresis time - the failsafe will trigger after 3 seconds in this state */
#define OFFBOARD_TIMEOUT                500000
#define HOTPLUG_SENS_TIMEOUT            (8 * 1000 * 1000)	/**< wait for hotplug sensors to come online for upto 8 seconds */


static orb_advert_t mavlink_log_pub = nullptr;


/* flags */
static volatile bool thread_should_exit = false;	/**< daemon exit flag */
static volatile bool thread_running = false;		/**< daemon status flag */
static int daemon_task;					/**< Handle of daemon task / thread */


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



class RoverCommander{

public:

    RoverCommander();

    void initialise();

    void run();

    void stop();

    void print_status();

 private:

    void init_params();

    void init_orb_subscribe();

    void init_orb_publish();

    void set_control_mode();

    void publish_status_flags();

private:

    void check_safety();

    void check_sensor();

    void check_local_position();

    void update_gps();

    void check_gps();

    void check_param_changed();

    void check_power_button();

    void check_offboard();

    void check_telemetry();

    void check_power();

    void check_battery();

    void check_subsys();

    void check_rc();

    void check_datalink();

    void check_cpu_load();

    void check_attitude();

    void check_cmd();

private:


     param_t _param_autostart_id, _param_rc_in_off, _param_eph, _param_epv, _param_fmode_1;
     param_t _param_offboard_loss_timeout;


     // uORB publisher

     orb_advert_t armed_pub;
     orb_advert_t control_mode_pub;
     orb_advert_t commander_state_pub;
     orb_advert_t vehicle_status_flags_pub;
     orb_advert_t status_pub;
     orb_advert_t power_button_state_pub;


     // uORB subscriber
     int power_button_state_sub;
     int safety_sub;
     int offboard_control_mode_sub;
     int global_position_sub;
     int local_position_sub;
     int attitude_sub;
     int gps_sub;
     int sensor_sub;
     int diff_pres_sub;
     int cmd_sub;
     int param_changed_sub;
     int battery_sub;
     int subsys_sub;
     int system_power_sub;
     int cpuload_sub;

     bool _last_condition_global_position_valid;
     bool updated;
     bool _usb_telemetry_active = false;
     bool low_battery_voltage_actions_done = false;
     bool critical_battery_voltage_actions_done = false;
     bool emergency_battery_voltage_actions_done = false;
     bool status_changed = true;
     bool param_init_forced = true;
     bool hotplug_timeout = false;

     actuator_controls_s actuator_controls;

     int telemetry_subs[ORB_MULTI_MAX_INSTANCES];
     uint64_t telemetry_last_heartbeat[ORB_MULTI_MAX_INSTANCES];
     uint64_t telemetry_last_dl_loss[ORB_MULTI_MAX_INSTANCES];
     bool telemetry_preflight_checks_reported[ORB_MULTI_MAX_INSTANCES];
     bool telemetry_lost[ORB_MULTI_MAX_INSTANCES];

     // parameters

     float eph_threshold;	// Horizontal position error threshold (m)
     float epv_threshold;	// Vertivcal position error threshold (m)
     float offboard_loss_timeout = 0.0f;


     // variables
     struct rover_status_s status;
     struct status_flags_s status_flags;

     struct vehicle_global_position_s global_position;
     struct vehicle_local_position_s local_position;
     struct vehicle_attitude_s attitude;
     struct vehicle_gps_position_s gps_position;
     struct sensor_combined_s sensors;
     struct differential_pressure_s diff_pres;
     struct subsystem_info_s info;
     struct system_power_s system_power;

     struct actuator_armed_s armed;
     struct safety_s safety;
     struct battery_status_s battery;

     struct rover_control_mode_s control_mode;
     struct offboard_control_mode_s offboard_control_mode;
     struct home_position_s _home;
     struct cpuload_s cpuload;


     hrt_abstime commander_boot_timestamp;

     //struct commander_state_s internal_state;

     manual_control_setpoint_s sp_man;		///< the current manual control setpoint


     int32_t _flight_mode_slots[manual_control_setpoint_s::MODE_SLOT_MAX];
     uint8_t main_state_prev;

     uint8_t main_state_before_rtl;




     hrt_abstime gpos_last_update_time_us = 0; // last time a global position update was received (usec)

     int32_t datalink_loss_timeout = 10;
     float rc_loss_timeout = 0.5;
     int32_t datalink_regain_timeout = 0;
     //float offboard_loss_timeout = 0.0f;
     int32_t low_bat_action = 0;

     /* System autostart ID */
     int autostart_id;

     uint64_t rc_signal_lost_timestamp;		// Time at which the RC reception was lost
     float avionics_power_rail_voltage;		// voltage of the avionics power rail

     static constexpr uint8_t COMMANDER_MAX_GPS_NOISE = 60;		/**< Maximum percentage signal to noise ratio allowed for GPS reception */





};

RoverCommander rover_commander;


#endif
