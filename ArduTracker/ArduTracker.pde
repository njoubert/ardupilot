#define THISFIRMWARE "ArduTracker V0.1-dev"


////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdio.h>
#include <stdarg.h>

// Common dependencies
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Menu.h>
#include <AP_Param.h>
// AP_HAL
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>

// Application dependencies
#include <GCS.h>
#include <GCS_MAVLink.h>        // MAVLink GCS definitions
#include <AP_GPS.h>             // ArduPilot GPS library
#include <AP_GPS_Glitch.h>      // GPS glitch protection library
#include <DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library

#include <AP_AHRS.h>
#include <AP_NavEKF.h>

#include <Filter.h>             // Filter library
#include <AP_Buffer.h>          // APM FIFO Buffer

#include <AP_Airspeed.h>        // needed for AHRS build
#include <AP_Vehicle.h>         // needed for AHRS build
#include <AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <AP_Declination.h>     // ArduPilot Mega Declination Helper Library
#include <AP_Scheduler.h>       // main loop scheduler
#include <AP_Notify.h>          // Notify library
#include <AP_BattMonitor.h>     // Battery monitor library
#include <AP_BoardConfig.h>     // board configuration library


// AP_HAL to Arduino compatibility layer
#include "compat.h"
// Configuration
#include "defines.h"
#include "config.h"


// Local modules
#include "Parameters.h"


//Documentation of GLobals:
static union {
    struct {
        uint8_t home_is_set         : 1; // 0
        uint8_t simple_mode         : 2; // 1,2 // This is the state of simple mode : 0 = disabled ; 1 = SIMPLE ; 2 = SUPERSIMPLE
        uint8_t pre_arm_rc_check    : 1; // 3   // true if rc input pre-arm checks have been completed successfully
        uint8_t pre_arm_check       : 1; // 4   // true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
        uint8_t auto_armed          : 1; // 5   // stops auto missions from beginning until throttle is raised
        uint8_t logging_started     : 1; // 6   // true if dataflash logging has started
        uint8_t land_complete       : 1; // 7   // true if we have detected a landing
        uint8_t new_radio_frame     : 1; // 8       // Set true if we have new PWM data to act on from the Radio
        uint8_t CH7_flag            : 2; // 9,10   // ch7 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH8_flag            : 2; // 11,12   // ch8 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t usb_connected       : 1; // 13      // true if APM is powered from USB connection
        uint8_t rc_receiver_present : 1; // 14  // true if we have an rc receiver present (i.e. if we've ever received an update
        uint8_t compass_mot         : 1; // 15  // true if we are currently performing compassmot calibration
    };
    uint16_t value;
} ap;

// board specific config
static AP_BoardConfig BoardConfig;


////////////////////////////////////////////////////////////////////////////////
// AP_HAL instance
////////////////////////////////////////////////////////////////////////////////

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;



////////////////////////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////////////////////////
//
// Global parameters are all contained within the 'g' class.
//
static Parameters g;

// main loop scheduler
static AP_Scheduler scheduler;

// AP_Notify instance
static AP_Notify notify;

// used to detect MAVLink acks from GCS to stop compassmot
static uint8_t command_ack_counter;




////////////////////////////////////////////////////////////////////////////////
// Dataflash
////////////////////////////////////////////////////////////////////////////////
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
static DataFlash_APM2 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
static DataFlash_APM1 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
static DataFlash_File DataFlash("logs");
//static DataFlash_SITL DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
static DataFlash_File DataFlash("/fs/microsd/APM/LOGS");
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
static DataFlash_File DataFlash("logs");
#else
static DataFlash_Empty DataFlash;
#endif



////////////////////////////////////////////////////////////////////////////////
// Intertial Sensor
// the rate we run the main loop at
////////////////////////////////////////////////////////////////////////////////
#if MAIN_LOOP_RATE == 400
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_400HZ;
#else
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_100HZ;
#endif



////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
//
// There are three basic options related to flight sensor selection.
//
// - Normal flight mode. Real sensors are used.
// - HIL Attitude mode. Most sensors are disabled, as the HIL
//   protocol supplies attitude information directly.
// - HIL Sensors mode. Synthetic sensors are configured that
//   supply data from the simulation.
//

// All GPS access should be through this pointer.
static GPS         *g_gps;
#if GPS2_ENABLE
static GPS         *g_gps2;
#endif
static GPS_Glitch   gps_glitch(g_gps);


#if CONFIG_ADC == ENABLED
static AP_ADC_ADS7844 adc;
#endif

#if CONFIG_IMU_TYPE == CONFIG_IMU_MPU6000
static AP_InertialSensor_MPU6000 ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_OILPAN
static AP_InertialSensor_Oilpan ins(&adc);
#elif CONFIG_IMU_TYPE == CONFIG_IMU_SITL
static AP_InertialSensor_HIL ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_PX4
static AP_InertialSensor_PX4 ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_FLYMAPLE
AP_InertialSensor_Flymaple ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_L3G4200D
AP_InertialSensor_L3G4200D ins;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 // When building for SITL we use the HIL barometer and compass drivers
static AP_Baro_HIL barometer;
static AP_Compass_HIL compass;
static SITL sitl;
#else
// Otherwise, instantiate a real barometer and compass driver
#if CONFIG_BARO == AP_BARO_BMP085
static AP_Baro_BMP085 barometer;
#elif CONFIG_BARO == AP_BARO_PX4
static AP_Baro_PX4 barometer;
#elif CONFIG_BARO == AP_BARO_MS5611
#if CONFIG_MS5611_SERIAL == AP_BARO_MS5611_SPI
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
#elif CONFIG_MS5611_SERIAL == AP_BARO_MS5611_I2C
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
#else
   #error Unrecognized CONFIG_MS5611_SERIAL setting.
#endif
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
static AP_Compass_PX4 compass;
#else
static AP_Compass_HMC5843 compass;
#endif
#endif

// real GPS selection
#if   GPS_PROTOCOL == GPS_PROTOCOL_AUTO
AP_GPS_Auto     g_gps_driver(&g_gps);
#if GPS2_ENABLE
AP_GPS_UBLOX    g_gps2_driver;
#endif

 #elif GPS_PROTOCOL == GPS_PROTOCOL_NMEA
AP_GPS_NMEA     g_gps_driver;

 #elif GPS_PROTOCOL == GPS_PROTOCOL_SIRF
AP_GPS_SIRF     g_gps_driver;

 #elif GPS_PROTOCOL == GPS_PROTOCOL_UBLOX
AP_GPS_UBLOX    g_gps_driver;

 #elif GPS_PROTOCOL == GPS_PROTOCOL_MTK
AP_GPS_MTK      g_gps_driver;

 #elif GPS_PROTOCOL == GPS_PROTOCOL_MTK19
AP_GPS_MTK19    g_gps_driver;

 #elif GPS_PROTOCOL == GPS_PROTOCOL_NONE
AP_GPS_None     g_gps_driver;

#else
 #error Unrecognised GPS_PROTOCOL setting.
#endif // GPS PROTOCOL


////////////////////////////////////////////////////////////////////////////////
// AHRS
////////////////////////////////////////////////////////////////////////////////

// Inertial Navigation EKF
// #if AP_AHRS_NAVEKF_AVAILABLE
//AP_AHRS_NavEKF ahrs(ins, barometer, g_gps);
// #else
AP_AHRS_DCM ahrs(ins, barometer, g_gps);
// #endif



////////////////////////////////////////////////////////////////////////////////
// cliSerial
////////////////////////////////////////////////////////////////////////////////
// cliSerial isn't strictly necessary - it is an alias for hal.console. It may
// be deprecated in favor of hal.console in later releases.
static AP_HAL::BetterStream* cliSerial;

// N.B. we need to keep a static declaration which isn't guarded by macros
// at the top to cooperate with the prototype mangler.


////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;
static GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS];




////////////////////////////////////////////////////////////////////////////////
// GPS variables
////////////////////////////////////////////////////////////////////////////////
// We use atan2 and other trig techniques to calaculate angles
// We need to scale the longitude up to make these calcs work
// to account for decreasing distance between lines of longitude away from the equator
static float scaleLongUp = 1;
// Sometimes we need to remove the scaling for distance calcs
static float scaleLongDown = 1;



////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// This is the angle from the copter to the next waypoint in centi-degrees
static int32_t wp_bearing;
// The original bearing to the next waypoint.  used to point the nose of the copter at the next waypoint
static int32_t original_wp_bearing;
// The location of home in relation to the copter in centi-degrees
static int32_t home_bearing;
// distance between plane and home in cm
static int32_t home_distance;
// distance between plane and next waypoint in cm.
static uint32_t wp_distance;
// Register containing the index of the current navigation command in the mission script
static int16_t command_nav_index;
// Register containing the index of the previous navigation command in the mission script
// Used to manage the execution of conditional commands
static uint8_t prev_nav_index;
// Register containing the index of the current conditional command in the mission script
static uint8_t command_cond_index;
// Used to track the required WP navigation information
// options include
// NAV_ALTITUDE - have we reached the desired altitude?
// NAV_LOCATION - have we reached the desired location?
// NAV_DELAY    - have we waited at the waypoint the desired time?
static float lon_error, lat_error;      // Used to report how many cm we are from the next waypoint or loiter target position


////////////////////////////////////////////////////////////////////////////////
// Battery Sensors
////////////////////////////////////////////////////////////////////////////////
static AP_BattMonitor battery;


////////////////////////////////////////////////////////////////////////////////
// Altitude
////////////////////////////////////////////////////////////////////////////////
// The cm/s we are moving up or down based on filtered data - Positive = UP
static int16_t climb_rate;
// The altitude as reported by Sonar in cm - Values are 20 to 700 generally.
static int16_t sonar_alt;
static uint8_t sonar_alt_health;   // true if we can trust the altitude from the sonar
static float target_sonar_alt;      // desired altitude in cm above the ground
// The altitude as reported by Baro in cm - Values can be quite high
static int32_t baro_alt;




////////////////////////////////////////////////////////////////////////////////
// IMU variables
////////////////////////////////////////////////////////////////////////////////
// Integration time (in seconds) for the gyros (DCM algorithm)
// Updated with the fast loop
static float G_Dt = 0.02;


////////////////////////////////////////////////////////////////////////////////
// Inertial Navigation
////////////////////////////////////////////////////////////////////////////////
// #if AP_AHRS_NAVEKF_AVAILABLE
// static AP_InertialNav_NavEKF inertial_nav(ahrs, barometer, g_gps, gps_glitch);
// #else
static AP_InertialNav inertial_nav(ahrs, barometer, g_gps, gps_glitch);
//#endif


////////////////////////////////////////////////////////////////////////////////
// Performance monitoring
////////////////////////////////////////////////////////////////////////////////
static int16_t pmTest1;

// System Timers
// --------------
// Time in microseconds of main control loop
static uint32_t fast_loopTimer;
// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t mainLoop_count;



// setup the var_info table
AP_Param param_loader(var_info, WP_START_BYTE);

#if MAIN_LOOP_RATE == 400
/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 2.5ms units) and the maximum time they are expected to take (in
  microseconds)
  1    = 400hz
  2    = 200hz
  4    = 100hz
  8    = 50hz
  20   = 20hz
  40   = 10hz
  133  = 3hz
  400  = 1hz
  4000 = 0.1hz
  
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
//     { update_GPS,            8,     90 },
//     { update_batt_compass,  40,     72 },
//     { auto_trim,            40,     14 },
//     { update_altitude,      40,    100 },
//     { run_nav_updates,      40,     80 },
//     { update_thr_cruise,    40,     10 },
//     { three_hz_loop,       133,      9 },
//     { compass_accumulate,    8,     42 },
//     { barometer_accumulate,  8,     25 },
//     { update_notify,         8,     10 },
       { one_hz_loop,         400,     42 },
//     { gcs_check_input,	     8,    550 },
//     { gcs_send_heartbeat,  400,    150 },
//     { gcs_send_deferred,     8,    720 },
//     { gcs_data_stream_send,  8,    950 },
// #if COPTER_LEDS == ENABLED
//     { update_copter_leds,   40,      5 },
// #endif
//     { update_mount,          8,     45 },
//     { ten_hz_logging_loop,  40,     30 },
//     { fifty_hz_logging_loop, 8,     22 },
//     { perf_update,        4000,     20 },
};
#else
/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 10ms units) and the maximum time they are expected to take (in
  microseconds)
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    // { update_GPS,            2,     900 },
    // { update_batt_compass,  10,     720 },
    // { auto_trim,            10,     140 },
    // { update_altitude,      10,    1000 },
    // { run_nav_updates,      10,     800 },
    // { update_thr_cruise,     1,      50 },
    // { three_hz_loop,        33,      90 },
    // { compass_accumulate,    2,     420 },
    // { barometer_accumulate,  2,     250 },
    // { update_notify,         2,     100 },
       { one_hz_loop,         100,     420 },
    // { gcs_check_input,	     2,     550 },
    // { gcs_send_heartbeat,  100,     150 },
    // { gcs_send_deferred,     2,     720 },
    // { gcs_data_stream_send,  2,     950 },
    // { ten_hz_logging_loop,  10,     300 },
    // { fifty_hz_logging_loop, 2,     220 },
    // { perf_update,        1000,     200 },
};
#endif




void setup() 
{

    cliSerial = hal.console;

    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    init_ardupilot();

    //initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}



void loop()
{
    // // wait for an INS sample
    // if (!ins.wait_for_sample(1000)) {
    //     Log_Write_Error(ERROR_SUBSYSTEM_MAIN, ERROR_CODE_MAIN_INS_DELAY);
    //     return;
    // }
    uint32_t timer = micros();

    // // check loop time
    // perf_info_check_loop_time(timer - fast_loopTimer);


    // // for mainloop failure monitoring
    mainLoop_count++;

    // // Execute the fast loop
    // // ---------------------
    fast_loop();

    // // tell the scheduler one tick has passed
    scheduler.tick();

    // // run all the tasks that are due to run. Note that we only
    // // have to call this once per loop, as the tasks are scheduled
    // // in multiples of the main loop tick. So if they don't run on
    // // the first call to the scheduler they won't run on a later
    // // call until scheduler.tick() is called again
    uint32_t time_available = (timer + MAIN_LOOP_MICROS) - micros();
    scheduler.run(time_available);

}


// Main loop - 100hz
static void fast_loop()
{

    // IMU DCM Algorithm
    // --------------------
    ahrs.update();

    // Inertial Nav
    // --------------------
    inertial_nav.update(G_Dt);

}


// one_hz_loop - runs at 1Hz
static void one_hz_loop()
{

  float roll = ahrs.roll;
  float pitch = ahrs.pitch;
  float yaw = ahrs.yaw;

    cliSerial->printf_P(PSTR("Hello World! Roll: %f, Pitch: %f, Yaw: %f\n"), roll, pitch, yaw);

}


AP_HAL_MAIN();
