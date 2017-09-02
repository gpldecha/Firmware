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


RoverCommander::RoverCommander(){

    commander_boot_timestamp=0;
    power_button_state_pub = nullptr;
    eph_threshold = 5.0f;	// Horizontal position error threshold (m)
    epv_threshold = 10.0f;	// Vertivcal position error threshold (m)
    control_mode = {};
    offboard_control_mode = {};
    _home = {};
    main_state_prev = 0;
    //internal_state = {};
    //main_state_before_rtl = commander_state_s::MAIN_STATE_MAX;
    _last_condition_global_position_valid = false;
    //status_flags = {};

    sp_man = {};
    cpuload = {};
    status = {};
}


void RoverCommander::initialise(){

    init_params();


    // We want to accept RC inputs as default
    status_flags.rc_input_blocked = false;
   // internal_state.main_state = commander_state_s::MAIN_STATE_MANUAL;
    //internal_state.timestamp = hrt_absolute_time();
    //main_state_prev = commander_state_s::MAIN_STATE_MAX;

    status.rc_input_mode        = rover_status_s::RC_IN_MODE_DEFAULT;
    status.nav_state            = rover_status_s::NAVIGATION_STATE_MANUAL;
    status.arming_state         = rover_status_s::ARMING_STATE_ARMED;
    status.failsafe             = false;
    status.timestamp            = hrt_absolute_time();
    status.rc_input_mode        = 0;


    //status_flags.usb_connected = false;

    /* publish initial state */
    status_pub = orb_advertise(ORB_ID(vehicle_status), &status);

    if (status_pub == nullptr) {
        warnx("ERROR: orb_advertise for topic vehicle_status failed (uorb app running?).\n");
        warnx("exiting.");
        px4_task_exit(PX4_ERROR);
    }

    /* uORB advertisers */
    init_orb_publish();

    /* uORB Subscribes */
    init_orb_subscribe();

    /* now initialized */
    thread_running = true;

    /* initialize */
    if (led_init() != OK) {
        PX4_WARN("LED init failed");
    }
    uint8_t led_mode = led_control_s::MODE_ON;
    uint8_t led_color = led_control_s::COLOR_GREEN;
    rgbled_set_color_and_mode(led_color, led_mode);
    set_tune_override(TONE_STARTUP_TUNE); //normal boot tune

}

void RoverCommander::init_params(){
    _param_autostart_id = param_find("SYS_AUTOSTART");
    _param_rc_in_off = param_find("COM_RC_IN_MODE");
    _param_eph = param_find("COM_HOME_H_T");
    _param_epv = param_find("COM_HOME_V_T");
    _param_fmode_1 = param_find("COM_FLTMODE1");

    _param_offboard_loss_timeout = param_find("COM_OF_LOSS_T");
    param_get(_param_offboard_loss_timeout, &offboard_loss_timeout);

}

void RoverCommander::init_orb_publish(){
    /* Initialize armed with all true */  /* armed topic */
    memset(&armed, 1, sizeof(armed));
    armed_pub = orb_advertise(ORB_ID(actuator_armed), &armed);
    /* vehicle control mode topic */
    memset(&control_mode, 0, sizeof(control_mode));
    control_mode_pub = orb_advertise(ORB_ID(vehicle_control_mode), &control_mode);
    commander_state_pub = nullptr;
    vehicle_status_flags_pub = nullptr;
}

void RoverCommander::init_orb_subscribe(){

    // Power Button Pub/Sub
    power_button_state_sub = orb_subscribe(ORB_ID(power_button_state));
    // we need to do an initial publication to make sure uORB allocates the buffer, which cannot happen
    // in IRQ context.
    power_button_state_s button_state;
    button_state.timestamp = 0;
    button_state.event = 0xff;
    power_button_state_pub = orb_advertise(ORB_ID(power_button_state), &button_state);
    orb_copy(ORB_ID(power_button_state), power_button_state_sub, &button_state);


    // Safety topic
    safety_sub = orb_subscribe(ORB_ID(safety));
    memset(&safety, 0, sizeof(safety));
    safety.safety_switch_available = false;
    safety.safety_off = false;

    // Offboard control data topic
    offboard_control_mode_sub = orb_subscribe(ORB_ID(offboard_control_mode));
    memset(&offboard_control_mode, 0, sizeof(offboard_control_mode));

    /* Subscribe to telemetry status topics */
    for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
        telemetry_subs[i] = -1;
        telemetry_last_heartbeat[i] = 0;
        telemetry_last_dl_loss[i] = 0;
        telemetry_lost[i] = true;
        telemetry_preflight_checks_reported[i] = false;
    }

    // Global position topic
    global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
    memset(&global_position, 0, sizeof(global_position));
    /* Init EPH and EPV */
    global_position.eph = 1000.0f;
    global_position.epv = 1000.0f;

    // Local position topic
    local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

    // Attitude topic
    attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

    // GPS topic
    gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
    memset(&gps_position, 0, sizeof(gps_position));
    gps_position.eph = FLT_MAX;
    gps_position.epv = FLT_MAX;

    // Sensor topic
    sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
    memset(&sensors, 0, sizeof(sensors));

    // Differential pressure topic
    diff_pres_sub = orb_subscribe(ORB_ID(differential_pressure));
    memset(&diff_pres, 0, sizeof(diff_pres));

    // Command topic
    cmd_sub = orb_subscribe(ORB_ID(vehicle_command));

    // Parameter change topic
    param_changed_sub = orb_subscribe(ORB_ID(parameter_update));

    // Battery topic
    battery_sub = orb_subscribe(ORB_ID(battery_status));
    memset(&battery, 0, sizeof(battery));

    // Subsystem info topic
    subsys_sub = orb_subscribe(ORB_ID(subsystem_info));
    memset(&info, 0, sizeof(info));

    /* System power topic*/
    system_power_sub = orb_subscribe(ORB_ID(system_power));
    memset(&system_power, 0, sizeof(system_power));

    /* Subscribe to actuator controls (outputs) */
    memset(&actuator_controls, 0, sizeof(actuator_controls));

    cpuload_sub = orb_subscribe(ORB_ID(cpuload));
    memset(&cpuload, 0, sizeof(cpuload));

}

void RoverCommander::run(){

    commander_boot_timestamp = hrt_absolute_time();
    updated=false;
    unsigned counter = 0;

    PX4_INFO("staring main thread");

    while (!thread_should_exit) {


        check_safety();

        check_sensor();

        check_local_position();

        update_gps();

        check_gps();

        check_param_changed();

        check_power_button();

        check_offboard();

        check_telemetry();

        check_power();

        check_battery();

        check_subsys();

        check_rc();

        check_datalink();

        check_cpu_load();

        check_attitude();

        check_cmd();

        /* Get current timestamp */
        const hrt_abstime now = hrt_absolute_time();

        /* publish states (armed, control mode, vehicle status) at least with 5 Hz */
        if (counter % (200000 / COMMANDER_MONITORING_INTERVAL) == 0 || status_changed) {
            set_control_mode();
            control_mode.timestamp = now;
            orb_publish(ORB_ID(vehicle_control_mode), control_mode_pub, &control_mode);

            status.timestamp = now;
            orb_publish(ORB_ID(vehicle_status), status_pub, &status);

            print_status();

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
        publish_status_flags();

        usleep(COMMANDER_MONITORING_INTERVAL);
    }

}


void RoverCommander::check_safety(){
    orb_check(safety_sub, &updated);
}

void RoverCommander::check_sensor(){
    orb_check(sensor_sub, &updated);
}

/**
 * @brief RoverCommander::update_gps
 * Check if quality checking of position accuracy and consistency is to be performed
 */
void RoverCommander::update_gps(){
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
}

/**
 * @brief RoverCommander::check_gps
 * Check GPS fix quality. Note that this check augments the position validity
 * checks and adds an additional level of protection.
 */
void RoverCommander::check_gps(){
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
        globallocalconverter_init((double)gps_position.lat * 1.0e-7, (double)gps_position.lon * 1.0e-7, (float)gps_position.alt * 1.0e-3f, hrt_absolute_time());
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
            // handle the case where gps was regained
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
}

/**
 * @brief RoverCommander::check_local_position
 *  Update local position estimate
 */
void RoverCommander::check_local_position(){
    bool lpos_updated = false;
    orb_check(local_position_sub, &lpos_updated);
    if (lpos_updated) {
        /* position changed */
        orb_copy(ORB_ID(vehicle_local_position), local_position_sub, &local_position);
    }
}

void RoverCommander::check_power_button(){

    /* handle power button state */
    orb_check(power_button_state_sub, &updated);

    if (updated) {
        power_button_state_s button_state;
        orb_copy(ORB_ID(power_button_state), power_button_state_sub, &button_state);
        if (button_state.event == power_button_state_s::PWR_BUTTON_STATE_REQUEST_SHUTDOWN) {
            px4_shutdown_request(false, false);
        }
    }

}

void RoverCommander::check_param_changed(){
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
}

void RoverCommander::check_offboard(){
    orb_check(offboard_control_mode_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(offboard_control_mode), offboard_control_mode_sub, &offboard_control_mode);
    }
    if (offboard_control_mode.timestamp != 0 && offboard_control_mode.timestamp + OFFBOARD_TIMEOUT > hrt_absolute_time()) {
        if (status_flags.offboard_control_signal_lost) {
            status_flags.offboard_control_signal_lost = false;
            status_flags.offboard_control_loss_timeout = false;
            status_changed = true;
        }
    }else{
        if (!status_flags.offboard_control_signal_lost) {
            status_flags.offboard_control_signal_lost = true;
            status_changed = true;
        }
        /* check timer if offboard was there but now lost */
       if (!status_flags.offboard_control_loss_timeout && offboard_control_mode.timestamp != 0) {
            if (offboard_loss_timeout < FLT_EPSILON) {
                status_flags.offboard_control_loss_timeout = true;
            } else {
                status_flags.offboard_control_loss_timeout = offboard_control_mode.timestamp + OFFBOARD_TIMEOUT + offboard_loss_timeout * 1e6f < hrt_absolute_time();
            }
            if (status_flags.offboard_control_loss_timeout) {
                status_changed = true;
            }
        }
    }
}

void RoverCommander::check_telemetry(){
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
                (telemetry_last_heartbeat[i] == 0 || (hrt_elapsed_time(&telemetry_last_heartbeat[i]) > 3 * 1000 * 1000) || !telemetry_preflight_checks_reported[i]) &&
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
                    /* HITL configuration: check only RC input */
                    (void)Commander::preflightCheck(&mavlink_log_pub, false, false, false, false, false,
                            (status.rc_input_mode == vehicle_status_s::RC_IN_MODE_DEFAULT), false,
                             /* checkDynamic */ true, false, /* reportFailures */ false, /* prearm */ false, hrt_elapsed_time(&commander_boot_timestamp));

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
}

void RoverCommander::check_power(){
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
                 // apparently the USB cable went away but we are still powered,
                 // so lets reset to a classic non-usb state.
                mavlink_log_critical(&mavlink_log_pub, "USB disconnected, rebooting.")
                usleep(400000);
                px4_shutdown_request(true, false);
            }
            // finally judge the USB connected state based on software detection
            status_flags.usb_connected = _usb_telemetry_active;
        }
    }
}

void RoverCommander::check_battery(){
    /* update battery status */
    orb_check(battery_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(battery_status), battery_sub, &battery);

        /* only consider battery voltage if system has been running 6s (usb most likely detected) and battery voltage is valid */
        if (hrt_absolute_time() > commander_boot_timestamp + 6000000 && battery.voltage_filtered_v > 2.0f * FLT_EPSILON) {
            /* if battery voltage is getting lower, warn using buzzer, etc. */
            if (battery.warning == battery_status_s::BATTERY_WARNING_LOW && !low_battery_voltage_actions_done) {
                low_battery_voltage_actions_done = true;
                mavlink_log_critical(&mavlink_log_pub, "LOW BATTERY");
                status_changed = true;
            }else if (!status_flags.usb_connected && battery.warning == battery_status_s::BATTERY_WARNING_CRITICAL && !critical_battery_voltage_actions_done){
                critical_battery_voltage_actions_done = true;
                mavlink_log_critical(&mavlink_log_pub, "CRITICAL BATTERY");
                status_changed = true;
            }else if(!status_flags.usb_connected && battery.warning == battery_status_s::BATTERY_WARNING_EMERGENCY && !emergency_battery_voltage_actions_done){
                emergency_battery_voltage_actions_done = true;
                mavlink_log_critical(&mavlink_log_pub, "EMERGENCY BATTERY");
            } else {
                mavlink_log_critical(&mavlink_log_pub, "EMERGENCY WTF");
            }
            status_changed = true;
        }
    }

}

void RoverCommander::check_subsys(){
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
}

void RoverCommander::check_rc(){
    /* RC input check */
    if (!status_flags.rc_input_blocked && sp_man.timestamp != 0 && (hrt_absolute_time() < sp_man.timestamp + (uint64_t)(rc_loss_timeout * 1e6f))) {
        /* handle the case where RC signal was regained */
        if (!status_flags.rc_signal_found_once) {
            status_flags.rc_signal_found_once = true;
            status_changed = true;
        } else {
            if (status.rc_signal_lost) {
                mavlink_log_info(&mavlink_log_pub, "MANUAL CONTROL REGAINED after %llums", (hrt_absolute_time() - rc_signal_lost_timestamp) / 1000);
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
}

void RoverCommander::check_datalink(){
    /* data links check */
    bool have_link = false;

    for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
        if (telemetry_last_heartbeat[i] != 0 && hrt_elapsed_time(&telemetry_last_heartbeat[i]) < datalink_loss_timeout * 1e6) {
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
}

void RoverCommander::check_cpu_load(){
    orb_check(cpuload_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(cpuload), cpuload_sub, &cpuload);
    }
}

void RoverCommander::check_attitude(){

    /* update attitude estimate */
    orb_check(attitude_sub, &updated);

    if (updated) {
        /* attitude changed */
        orb_copy(ORB_ID(vehicle_attitude), attitude_sub, &attitude);
    }
}

void RoverCommander::check_cmd(){
    /* handle commands last, as the system needs to be updated to handle them */
    orb_check(cmd_sub, &updated);
    if (updated) {
        struct vehicle_command_s cmd;
        /* got command */
        orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);
    }
}

void RoverCommander::stop(){
    rgbled_set_color_and_mode(led_control_s::COLOR_WHITE, led_control_s::MODE_OFF);
    /* close fds */
    led_deinit();
    buzzer_deinit();
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
}

void RoverCommander::set_control_mode(){
    /* set vehicle_control_mode according to set_navigation_state */
    control_mode.flag_armed = armed.armed;
    control_mode.flag_control_manual_enabled = false;
    control_mode.flag_control_offboard_enabled = false;
    control_mode.flag_control_autonomous_enabled = false;

    switch (status.nav_state) {
    case rover_status_s::NAVIGATION_STATE_MANUAL:
        control_mode.flag_control_manual_enabled = true;
        break;
    case rover_status_s::NAVIGATION_STATE_OFFBOARD:
        control_mode.flag_control_offboard_enabled = true;
        break;
    case rover_status_s::NAVIGATION_STATE_AUTONOMOUS:
        control_mode.flag_control_autonomous_enabled = true;
        break;
    default:
        break;
    }
}

void RoverCommander::print_status()
{
    warnx("safety: USB enabled: %s, power state valid: %s", (status_flags.usb_connected) ? "[OK]" : "[NO]", (status_flags.condition_power_input_valid) ? " [OK]" : "[NO]");
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
    const char *nav_str;

    switch (status.arming_state) {
    case rover_status_s::ARMING_STATE_ARMED:
        armed_str = "ARMED";
        break;
    case rover_status_s::ARMING_STATE_REBOOT:
        armed_str = "REBOOT";
        break;
    default:
        armed_str = "ERR: UNKNOWN STATE";
        break;
    }
    warnx("arming: %s", armed_str);

    switch (status.nav_state) {
    case rover_status_s::NAVIGATION_STATE_MANUAL:
        nav_str = "MANUAL";
        break;
    case rover_status_s::NAVIGATION_STATE_OFFBOARD:
        nav_str = "OFFBOARD";
        break;
    case rover_status_s::NAVIGATION_STATE_AUTONOMOUS:
        nav_str = "AUTONOMOUS";
        break;
    default:
        nav_str = "ERR: UNKNOWN NAVIGATION STATE";
        break;
    }
    warnx("navigation: %s", nav_str);

    px4_close(state_sub);

}

void RoverCommander::publish_status_flags() {
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
    // set circuit breaker status flags
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
    // set other status flags
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
    // publish vehicle_status_flags
    if (vehicle_status_flags_pub != nullptr) {
        orb_publish(ORB_ID(vehicle_status_flags), vehicle_status_flags_pub, &v_flags);
    } else {
        vehicle_status_flags_pub = orb_advertise(ORB_ID(vehicle_status_flags), &v_flags);
    }
}


int commander_main(int argc, char *argv[]){
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
        rover_commander.print_status();
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
                //calib_ret = do_esc_calibration(&mavlink_log_pub, &armed);
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

int commander_thread_main(int argc, char *argv[]){

    rover_commander.initialise();
    rover_commander.run();
    rover_commander.stop();

    return 0;
}


void usage(const char *reason)
{
    if (reason && *reason > 0) {
        PX4_INFO("%s", reason);
    }

    PX4_INFO("usage: commander {start|stop|status|calibrate|}\n");
}



