// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <AP_Common.h>

// Global parameter class.
//
class Parameters {
public:
    // The version of the layout as described by the parameter enum.
    //
    // When changing the parameter enum in an incompatible fashion, this
    // value should be incremented by one.
    //
    // The increment will prevent old parameters from being used incorrectly
    // by newer code.
    //
    static const uint16_t        k_format_version = 140;

    // The parameter software_type is set up solely for ground station use
    // and identifies the software type (eg ArduPilotMega versus
    // ArduCopterMega)
    // GCS will interpret values 0-9 as ArduPilotMega.  Developers may use
    // values within that range to identify different branches.
    //
    static const uint16_t        k_software_type = 10;          // 0 for APM
                                                                // trunk
    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_software_type,
        k_param_ins_old,                        // *** Deprecated, remove with next eeprom number change
        k_param_ins,                            // libraries/AP_InertialSensor variables

        // simulation
        k_param_sitl = 10,

        // barometer object (needed for SITL)
        k_param_barometer,

        // scheduler object (for debugging)
        k_param_scheduler,

        // relay object
        k_param_relay,

        // EPM object
        k_param_epm,

        // BoardConfig object
        k_param_BoardConfig,

        // Misc
        //
        k_param_log_bitmask = 20,
        k_param_log_last_filenumber,            // *** Deprecated - remove
                                                // with next eeprom number
                                                // change
        k_param_toy_yaw_rate,                   // deprecated - remove
        k_param_crosstrack_min_distance,    // deprecated - remove with next eeprom number change
        k_param_rssi_pin,
        k_param_throttle_accel_enabled,     // deprecated - remove
        k_param_wp_yaw_behavior,
        k_param_acro_trainer,
        k_param_pilot_velocity_z_max,
        k_param_circle_rate,                // deprecated - remove
        k_param_sonar_gain,
        k_param_ch8_option,
        k_param_arming_check,
        k_param_sprayer,
        k_param_angle_max,
        k_param_gps_hdop_good,
        k_param_battery,
        k_param_fs_batt_mah,
        k_param_angle_rate_max,         // remove
        k_param_rssi_range,
        k_param_rc_feel_rp,
        k_param_NavEKF,                 // 41 - Extended Kalman Filter Inertial Navigation Group

        // 65: AP_Limits Library
        k_param_limits = 65,            // deprecated - remove
        k_param_gpslock_limit,          // deprecated - remove
        k_param_geofence_limit,         // deprecated - remove
        k_param_altitude_limit,         // deprecated - remove
        k_param_fence,
        k_param_gps_glitch,             // 70



        //
        // 100: Inertial Nav
        //
        k_param_inertial_nav = 100,
        k_param_wp_nav,
        k_param_attitude_control,
        k_param_pos_control,
        k_param_circle_nav,     // 104

        // 110: Telemetry control
        //
        k_param_gcs0 = 110,
        k_param_gcs1,
        k_param_sysid_this_mav,
        k_param_sysid_my_gcs,
        k_param_serial1_baud,
        k_param_telem_delay,
        k_param_gcs2,
        k_param_serial2_baud,

        //
        // 140: Sensor parameters
        //
        k_param_imu = 140, // deprecated - can be deleted
        k_param_battery_monitoring = 141,   // deprecated - can be deleted
        k_param_volt_div_ratio, // deprecated - can be deleted
        k_param_curr_amp_per_volt,  // deprecated - can be deleted
        k_param_input_voltage,  // deprecated - can be deleted
        k_param_pack_capacity,  // deprecated - can be deleted
        k_param_compass_enabled,
        k_param_compass,
        k_param_sonar_enabled,
        k_param_frame_orientation,
        k_param_optflow_enabled,
        k_param_fs_batt_voltage,
        k_param_ch7_option,
        k_param_auto_slew_rate,     // deprecated - can be deleted
        k_param_sonar_type,
        k_param_super_simple = 155,
        k_param_axis_enabled = 157, // deprecated - remove with next eeprom number change
        k_param_copter_leds_mode,   // deprecated - remove with next eeprom number change
        k_param_ahrs, // AHRS group // 159

        //
        // Batery monitoring parameters
        //
        k_param_battery_volt_pin = 168, // deprecated - can be deleted
        k_param_battery_curr_pin,   // 169 deprecated - can be deleted


        // 254,255: reserved
    };

    AP_Int16    format_version;
    AP_Int8     software_type;

    // Misc
    //
    AP_Int16    log_bitmask;
    AP_Int16    num_resets;
    AP_Int8     reset_switch_chan;
    AP_Int8     initial_mode;


    // Telemetry control
    //
    AP_Int16        sysid_this_mav;
    AP_Int16        sysid_my_gcs;
    AP_Int8         serial1_baud;
#if MAVLINK_COMM_NUM_BUFFERS > 2
    AP_Int8         serial2_baud;
#endif
    AP_Int8         telem_delay;

    AP_Int16        rtl_altitude;
    AP_Int8         sonar_enabled;
    AP_Int8         sonar_type;       // 0 = XL, 1 = LV,
                                      // 2 = XLL (XL with 10m range)
                                      // 3 = HRLV
    AP_Float        sonar_gain;

    AP_Int8         failsafe_battery_enabled;   // battery failsafe enabled
    AP_Float        fs_batt_voltage;            // battery voltage below which failsafe will be triggered
    AP_Float        fs_batt_mah;                // battery capacity (in mah) below which failsafe will be triggered

    AP_Int8         failsafe_gps_enabled;       // gps failsafe enabled
    AP_Int8         failsafe_gcs;               // ground station failsafe behavior
    AP_Int16        gps_hdop_good;              // GPS Hdop value at or below this value represent a good position

    AP_Int8         compass_enabled;
    AP_Int8         optflow_enabled;
    AP_Int8         super_simple;
    AP_Int16        rtl_alt_final;

    AP_Int8         rssi_pin;
    AP_Float        rssi_range;                 // allows to set max voltage for rssi pin such as 5.0, 3.3 etc. 
    AP_Int8         wp_yaw_behavior;            // controls how the autopilot controls yaw during missions
    AP_Int8         rc_feel_rp;                 // controls vehicle response to user input with 0 being extremely soft and 100 begin extremely crisp

    
    Parameters()
    {
    }
};

extern const AP_Param::Info        var_info[];

#endif // PARAMETERS_H

