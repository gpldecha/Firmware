/* builtin command list - automatically generated, do not edit */
#include <nuttx/config.h>
#include <nuttx/binfmt/builtin.h>
#include <nuttx/config.h>
#if 78
extern int blinkm_main(int argc, char *argv[]);
extern int bmi160_main(int argc, char *argv[]);
extern int bmp280_main(int argc, char *argv[]);
extern int bst_main(int argc, char *argv[]);
extern int camera_trigger_main(int argc, char *argv[]);
extern int ets_airspeed_main(int argc, char *argv[]);
extern int frsky_telemetry_main(int argc, char *argv[]);
extern int gps_main(int argc, char *argv[]);
extern int hmc5883_main(int argc, char *argv[]);
extern int hott_sensors_main(int argc, char *argv[]);
extern int hott_telemetry_main(int argc, char *argv[]);
extern int iridiumsbd_main(int argc, char *argv[]);
extern int l3gd20_main(int argc, char *argv[]);
extern int lis3mdl_main(int argc, char *argv[]);
extern int ll40ls_main(int argc, char *argv[]);
extern int lsm303d_main(int argc, char *argv[]);
extern int mb12xx_main(int argc, char *argv[]);
extern int mkblctrl_main(int argc, char *argv[]);
extern int mpu6000_main(int argc, char *argv[]);
extern int mpu9250_main(int argc, char *argv[]);
extern int ms5611_main(int argc, char *argv[]);
extern int oreoled_main(int argc, char *argv[]);
extern int pwm_input_main(int argc, char *argv[]);
extern int pwm_out_sim_main(int argc, char *argv[]);
extern int px4flow_main(int argc, char *argv[]);
extern int fmu_main(int argc, char *argv[]);
extern int px4io_main(int argc, char *argv[]);
extern int rgbled_main(int argc, char *argv[]);
extern int sdp3x_airspeed_main(int argc, char *argv[]);
extern int sf0x_main(int argc, char *argv[]);
extern int sf1xx_main(int argc, char *argv[]);
extern int snapdragon_rc_pwm_main(int argc, char *argv[]);
extern int srf02_main(int argc, char *argv[]);
extern int adc_main(int argc, char *argv[]);
extern int tone_alarm_main(int argc, char *argv[]);
extern int tap_esc_main(int argc, char *argv[]);
extern int trone_main(int argc, char *argv[]);
extern int vmount_main(int argc, char *argv[]);
extern int sensors_main(int argc, char *argv[]);
extern int bl_update_main(int argc, char *argv[]);
extern int config_main(int argc, char *argv[]);
extern int dumpfile_main(int argc, char *argv[]);
extern int esc_calib_main(int argc, char *argv[]);
extern int hardfault_log_main(int argc, char *argv[]);
extern int led_control_main(int argc, char *argv[]);
extern int mixer_main(int argc, char *argv[]);
extern int motor_ramp_main(int argc, char *argv[]);
extern int motor_test_main(int argc, char *argv[]);
extern int mtd_main(int argc, char *argv[]);
extern int nshterm_main(int argc, char *argv[]);
extern int param_main(int argc, char *argv[]);
extern int perf_main(int argc, char *argv[]);
extern int pwm_main(int argc, char *argv[]);
extern int reboot_main(int argc, char *argv[]);
extern int sd_bench_main(int argc, char *argv[]);
extern int top_main(int argc, char *argv[]);
extern int listener_main(int argc, char *argv[]);
extern int ver_main(int argc, char *argv[]);
extern int commander_main(int argc, char *argv[]);
extern int send_event_main(int argc, char *argv[]);
extern int gpio_led_main(int argc, char *argv[]);
extern int load_mon_main(int argc, char *argv[]);
extern int mavlink_main(int argc, char *argv[]);
extern int uavcan_main(int argc, char *argv[]);
extern int camera_feedback_main(int argc, char *argv[]);
extern int attitude_estimator_q_main(int argc, char *argv[]);
extern int ekf2_main(int argc, char *argv[]);
extern int local_position_estimator_main(int argc, char *argv[]);
extern int position_estimator_inav_main(int argc, char *argv[]);
extern int gnd_att_control_main(int argc, char *argv[]);
extern int gnd_pos_control_main(int argc, char *argv[]);
extern int logger_main(int argc, char *argv[]);
extern int sdlog2_main(int argc, char *argv[]);
extern int dataman_main(int argc, char *argv[]);
extern int uorb_main(int argc, char *argv[]);
extern int micrortps_client_main(int argc, char *argv[]);
extern int serdis_main(int argc, char *argv[]);
extern int sercon_main(int argc, char *argv[]);

const struct builtin_s g_builtins[] = {
	{"blinkm", SCHED_PRIORITY_DEFAULT, 1024, blinkm_main},
	{"bmi160", SCHED_PRIORITY_DEFAULT, 1200, bmi160_main},
	{"bmp280", SCHED_PRIORITY_DEFAULT, 1024, bmp280_main},
	{"bst", SCHED_PRIORITY_DEFAULT, 1200, bst_main},
	{"camera_trigger", SCHED_PRIORITY_DEFAULT, 1200, camera_trigger_main},
	{"ets_airspeed", SCHED_PRIORITY_DEFAULT, 1200, ets_airspeed_main},
	{"frsky_telemetry", SCHED_PRIORITY_DEFAULT, 1200, frsky_telemetry_main},
	{"gps", SCHED_PRIORITY_DEFAULT, 1200, gps_main},
	{"hmc5883", SCHED_PRIORITY_DEFAULT, 1200, hmc5883_main},
	{"hott_sensors", SCHED_PRIORITY_DEFAULT, 1024, hott_sensors_main},
	{"hott_telemetry", SCHED_PRIORITY_DEFAULT, 1024, hott_telemetry_main},
	{"iridiumsbd", SCHED_PRIORITY_DEFAULT, 1024, iridiumsbd_main},
	{"l3gd20", SCHED_PRIORITY_DEFAULT, 1200, l3gd20_main},
	{"lis3mdl", SCHED_PRIORITY_DEFAULT, 1200, lis3mdl_main},
	{"ll40ls", SCHED_PRIORITY_DEFAULT, 1024, ll40ls_main},
	{"lsm303d", SCHED_PRIORITY_DEFAULT, 1200, lsm303d_main},
	{"mb12xx", SCHED_PRIORITY_DEFAULT, 1024, mb12xx_main},
	{"mkblctrl", SCHED_PRIORITY_DEFAULT, 1024, mkblctrl_main},
	{"mpu6000", SCHED_PRIORITY_DEFAULT, 1400, mpu6000_main},
	{"mpu9250", SCHED_PRIORITY_DEFAULT, 1200, mpu9250_main},
	{"ms5611", SCHED_PRIORITY_DEFAULT, 1200, ms5611_main},
	{"oreoled", SCHED_PRIORITY_DEFAULT, 1024, oreoled_main},
	{"pwm_input", SCHED_PRIORITY_DEFAULT, 1024, pwm_input_main},
	{"pwm_out_sim", SCHED_PRIORITY_DEFAULT, 1200, pwm_out_sim_main},
	{"px4flow", SCHED_PRIORITY_DEFAULT, 1200, px4flow_main},
	{"fmu", SCHED_PRIORITY_DEFAULT, 1200, fmu_main},
	{"px4io", SCHED_PRIORITY_DEFAULT, 1816, px4io_main},
	{"rgbled", SCHED_PRIORITY_DEFAULT, 1024, rgbled_main},
	{"sdp3x_airspeed", SCHED_PRIORITY_DEFAULT, 1200, sdp3x_airspeed_main},
	{"sf0x", SCHED_PRIORITY_DEFAULT, 1024, sf0x_main},
	{"sf1xx", SCHED_PRIORITY_DEFAULT, 1024, sf1xx_main},
	{"snapdragon_rc_pwm", SCHED_PRIORITY_DEFAULT, 1024, snapdragon_rc_pwm_main},
	{"srf02", SCHED_PRIORITY_DEFAULT, 1024, srf02_main},
	{"adc", SCHED_PRIORITY_DEFAULT, 1024, adc_main},
	{"tone_alarm", SCHED_PRIORITY_DEFAULT, 1024, tone_alarm_main},
	{"tap_esc", SCHED_PRIORITY_DEFAULT, 1024, tap_esc_main},
	{"trone", SCHED_PRIORITY_DEFAULT, 1200, trone_main},
	{"vmount", SCHED_PRIORITY_DEFAULT, 1024, vmount_main},
	{"sensors", SCHED_PRIORITY_MAX-5, 1300, sensors_main},
	{"bl_update", SCHED_PRIORITY_DEFAULT, 4096, bl_update_main},
	{"config", SCHED_PRIORITY_DEFAULT, 4096, config_main},
	{"dumpfile", SCHED_PRIORITY_DEFAULT, 4096, dumpfile_main},
	{"esc_calib", SCHED_PRIORITY_DEFAULT, 4096, esc_calib_main},
	{"hardfault_log", SCHED_PRIORITY_DEFAULT, 2048, hardfault_log_main},
	{"led_control", SCHED_PRIORITY_DEFAULT, 2500, led_control_main},
	{"mixer", SCHED_PRIORITY_DEFAULT, 4096, mixer_main},
	{"motor_ramp", SCHED_PRIORITY_DEFAULT, 1200, motor_ramp_main},
	{"motor_test", SCHED_PRIORITY_DEFAULT, 4096, motor_test_main},
	{"mtd", SCHED_PRIORITY_DEFAULT, 1024, mtd_main},
	{"nshterm", SCHED_PRIORITY_DEFAULT-30, 1500, nshterm_main},
	{"param", SCHED_PRIORITY_DEFAULT, 2500, param_main},
	{"perf", SCHED_PRIORITY_DEFAULT, 1800, perf_main},
	{"pwm", SCHED_PRIORITY_DEFAULT, 2500, pwm_main},
	{"reboot", SCHED_PRIORITY_DEFAULT, 800, reboot_main},
	{"sd_bench", SCHED_PRIORITY_DEFAULT, 2500, sd_bench_main},
	{"top", SCHED_PRIORITY_DEFAULT, 1700, top_main},
	{"listener", SCHED_PRIORITY_DEFAULT, 1800, listener_main},
	{"ver", SCHED_PRIORITY_DEFAULT, 1024, ver_main},
	{"commander", SCHED_PRIORITY_DEFAULT, 4096, commander_main},
	{"send_event", SCHED_PRIORITY_DEFAULT, 2200, send_event_main},
	{"gpio_led", SCHED_PRIORITY_DEFAULT, 1024, gpio_led_main},
	{"load_mon", SCHED_PRIORITY_DEFAULT, 1200, load_mon_main},
	{"mavlink", SCHED_PRIORITY_DEFAULT, 1200, mavlink_main},
	{"uavcan", SCHED_PRIORITY_DEFAULT, 3200, uavcan_main},
	{"camera_feedback", SCHED_PRIORITY_DEFAULT, 1024, camera_feedback_main},
	{"attitude_estimator_q", SCHED_PRIORITY_DEFAULT, 1200, attitude_estimator_q_main},
	{"ekf2", SCHED_PRIORITY_DEFAULT, 2500, ekf2_main},
	{"local_position_estimator", SCHED_PRIORITY_DEFAULT, 5700, local_position_estimator_main},
	{"position_estimator_inav", SCHED_PRIORITY_DEFAULT, 1200, position_estimator_inav_main},
	{"gnd_att_control", SCHED_PRIORITY_DEFAULT, 1200, gnd_att_control_main},
	{"gnd_pos_control", SCHED_PRIORITY_DEFAULT, 1200, gnd_pos_control_main},
	{"logger", SCHED_PRIORITY_MAX-30, 2200, logger_main},
	{"sdlog2", SCHED_PRIORITY_MAX-30, 1200, sdlog2_main},
	{"dataman", SCHED_PRIORITY_DEFAULT, 1200, dataman_main},
	{"uorb", SCHED_PRIORITY_DEFAULT, 2100, uorb_main},
	{"micrortps_client", SCHED_PRIORITY_DEFAULT, 4096, micrortps_client_main},
	{"serdis", SCHED_PRIORITY_DEFAULT, 2048, serdis_main},
	{"sercon", SCHED_PRIORITY_DEFAULT, 2048, sercon_main},

	{NULL, 0, 0, NULL}
};
const int g_builtin_count = 78;
#endif
