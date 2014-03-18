// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

static void do_erase_logs(void)
{
	gcs_send_text_P(SEVERITY_LOW, PSTR("Erasing logs\n"));
    DataFlash.EraseAll();
	gcs_send_text_P(SEVERITY_LOW, PSTR("Log erase complete\n"));
}

static int8_t
erase_logs(uint8_t argc, const Menu::arg *argv)
{
    in_mavlink_delay = true;
    do_erase_logs();
    in_mavlink_delay = false;
    return 0;
}
struct PACKED log_Current {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  throttle_out;
    uint32_t throttle_integrator;
    int16_t  battery_voltage;
    int16_t  current_amps;
    uint16_t board_voltage;
    float    current_total;
};

// Write an Current data packet
static void Log_Write_Current()
{
    struct log_Current pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CURRENT_MSG),
        time_ms             : hal.scheduler->millis(),
        throttle_out        : 0,
        throttle_integrator : 0,
        battery_voltage     : (int16_t) (battery.voltage() * 100.0f),
        current_amps        : (int16_t) (battery.current_amps() * 100.0f),
        board_voltage       : (uint16_t)(hal.analogin->board_voltage()*1000),
        current_total       : battery.current_total_mah()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));

    // also write power status
    DataFlash.Log_Write_Power();
}



struct PACKED log_Compass {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  mag_x;
    int16_t  mag_y;
    int16_t  mag_z;
    int16_t  offset_x;
    int16_t  offset_y;
    int16_t  offset_z;
    int16_t  motor_offset_x;
    int16_t  motor_offset_y;
    int16_t  motor_offset_z;
};

// Write a Compass packet
static void Log_Write_Compass()
{
    const Vector3f &mag_offsets = compass.get_offsets(0);
    const Vector3f &mag_motor_offsets = compass.get_motor_offsets(0);
    const Vector3f &mag = compass.get_field(0);
    struct log_Compass pkt = {
        LOG_PACKET_HEADER_INIT(LOG_COMPASS_MSG),
        time_ms         : hal.scheduler->millis(),
        mag_x           : (int16_t)mag.x,
        mag_y           : (int16_t)mag.y,
        mag_z           : (int16_t)mag.z,
        offset_x        : (int16_t)mag_offsets.x,
        offset_y        : (int16_t)mag_offsets.y,
        offset_z        : (int16_t)mag_offsets.z,
        motor_offset_x  : (int16_t)mag_motor_offsets.x,
        motor_offset_y  : (int16_t)mag_motor_offsets.y,
        motor_offset_z  : (int16_t)mag_motor_offsets.z
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
#if COMPASS_MAX_INSTANCES > 1
    if (compass.get_count() > 1) {
        const Vector3f &mag2_offsets = compass.get_offsets(1);
        const Vector3f &mag2_motor_offsets = compass.get_motor_offsets(1);
        const Vector3f &mag2 = compass.get_field(1);
        struct log_Compass pkt2 = {
            LOG_PACKET_HEADER_INIT(LOG_COMPASS2_MSG),
            time_ms         : hal.scheduler->millis(),
            mag_x           : (int16_t)mag2.x,
            mag_y           : (int16_t)mag2.y,
            mag_z           : (int16_t)mag2.z,
            offset_x        : (int16_t)mag2_offsets.x,
            offset_y        : (int16_t)mag2_offsets.y,
            offset_z        : (int16_t)mag2_offsets.z,
            motor_offset_x  : (int16_t)mag2_motor_offsets.x,
            motor_offset_y  : (int16_t)mag2_motor_offsets.y,
            motor_offset_z  : (int16_t)mag2_motor_offsets.z
        };
        DataFlash.WriteBlock(&pkt2, sizeof(pkt2));
    }
#endif
}


struct PACKED log_Attitude {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  control_roll;
    int16_t  roll;
    int16_t  control_pitch;
    int16_t  pitch;
    uint16_t control_yaw;
    uint16_t yaw;
};

// Write an attitude packet
static void Log_Write_Attitude()
{
    struct log_Attitude pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        time_ms         : hal.scheduler->millis(),
        control_roll    : 0,
        roll            : (int16_t)ahrs.roll_sensor,
        control_pitch   : 0,
        pitch           : (int16_t)ahrs.pitch_sensor,
        control_yaw     : 0,
        yaw             : (uint16_t)ahrs.yaw_sensor
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));

#if AP_AHRS_NAVEKF_AVAILABLE
    DataFlash.Log_Write_AHRS2(ahrs);
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    sitl.Log_Write_SIMSTATE(DataFlash);
#endif
}

struct PACKED log_Startup {
    LOG_PACKET_HEADER;
};

// Write Startup packet
static void Log_Write_Startup()
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Event {
    LOG_PACKET_HEADER;
    uint8_t id;
};

// Wrote an event packet
static void Log_Write_Event(uint8_t id)
{
    if (g.log_bitmask != 0) {
        struct log_Event pkt = {
            LOG_PACKET_HEADER_INIT(LOG_EVENT_MSG),
            id  : id
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int16_t data_value;
};

// Write an int16_t data packet
static void Log_Write_Data(uint8_t id, int16_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint16_t data_value;
};

// Write an uint16_t data packet
static void Log_Write_Data(uint8_t id, uint16_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int32_t data_value;
};

// Write an int32_t data packet
static void Log_Write_Data(uint8_t id, int32_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint32_t data_value;
};

// Write a uint32_t data packet
static void Log_Write_Data(uint8_t id, uint32_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;
    uint8_t id;
    float data_value;
};

// Write a float data packet
static void Log_Write_Data(uint8_t id, float value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}
struct PACKED log_Error {
    LOG_PACKET_HEADER;
    uint8_t sub_system;
    uint8_t error_code;
};

// Write an error packet
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code)
{
    struct log_Error pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ERROR_MSG),
        sub_system    : sub_system,
        error_code    : error_code,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

static void Log_Write_Baro(void)
{
    DataFlash.Log_Write_Baro(barometer);
}

static const struct LogStructure log_structure[] PROGMEM = {
    LOG_COMMON_STRUCTURES,
#if AUTOTUNE == ENABLED
    { LOG_AUTOTUNE_MSG, sizeof(log_AutoTune),
      "ATUN", "BBfffff",       "Axis,TuneStep,RateMin,RateMax,RPGain,RDGain,SPGain" },
    { LOG_AUTOTUNEDETAILS_MSG, sizeof(log_AutoTuneDetails),
      "ATDE", "cf",          "Angle,Rate" },
#endif
    { LOG_CURRENT_MSG, sizeof(log_Current),             
      "CURR", "IhIhhhf",     "TimeMS,ThrOut,ThrInt,Volt,Curr,Vcc,CurrTot" },
    { LOG_COMPASS_MSG, sizeof(log_Compass),             
      "MAG", "Ihhhhhhhhh",    "TimeMS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ" },
    { LOG_COMPASS2_MSG, sizeof(log_Compass),             
      "MAG2","Ihhhhhhhhh",    "TimeMS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ" },
    { LOG_ATTITUDE_MSG, sizeof(log_Attitude),       
      "ATT", "IccccCC",      "TimeMS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw" },
    { LOG_STARTUP_MSG, sizeof(log_Startup),         
      "STRT", "",            "" },
    { LOG_EVENT_MSG, sizeof(log_Event),         
      "EV",   "B",           "Id" },
    { LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),         
      "D16",   "Bh",         "Id,Value" },
    { LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),         
      "DU16",  "BH",         "Id,Value" },
    { LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),         
      "D32",   "Bi",         "Id,Value" },
    { LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),         
      "DU32",  "BI",         "Id,Value" },
    { LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),         
      "DFLT",  "Bf",         "Id,Value" },
    { LOG_ERROR_MSG, sizeof(log_Error),         
      "ERR",   "BB",         "Subsys,ECode" },
};

static void
print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode) {

    return;
}
// Read the DataFlash log memory
static void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page)
{
 #ifdef AIRFRAME_NAME
    cliSerial->printf_P(PSTR((AIRFRAME_NAME)));
 #endif

    cliSerial->printf_P(PSTR("\n" FIRMWARE_STRING
                             "\nFree RAM: %u\n"),
                        (unsigned) hal.util->available_memory());

    cliSerial->println_P(PSTR(HAL_BOARD_NAME));

	DataFlash.LogReadProcess(log_num, start_page, end_page, 
                             print_flight_mode,
                             cliSerial);
}

// start a new log
static void start_logging() 
{
    if (g.log_bitmask != 0) {
        if (!ap.logging_started) {
            ap.logging_started = true;
            in_mavlink_delay = true;
            DataFlash.StartNewLog();
            in_mavlink_delay = false;
            DataFlash.Log_Write_Message_P(PSTR(FIRMWARE_STRING));

#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
            DataFlash.Log_Write_Message_P(PSTR("PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION));
#endif

            // write system identifier as well if available
            char sysid[40];
            if (hal.util->get_system_id(sysid)) {
                DataFlash.Log_Write_Message(sysid);
            }

        }
        // enable writes
        DataFlash.EnableWrites(true);
    }
}

#else // LOGGING_ENABLED

static void Log_Write_Startup() {}
static void Log_Write_Cmd(uint8_t num, const struct Location *wp) {}
static void Log_Write_Mode(uint8_t mode) {}
static void Log_Write_IMU() {}
static void Log_Write_GPS() {}
#if AUTOTUNE_ENABLED == ENABLED
static void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float rate_min, float rate_max, float new_gain_rp, float new_gain_rd, float new_gain_sp) {}
static void Log_Write_AutoTuneDetails(int16_t angle_cd, float rate_cds) {}
#endif
static void Log_Write_Current() {}
static void Log_Write_Compass() {}
static void Log_Write_Attitude() {}
static void Log_Write_Data(uint8_t id, int16_t value){}
static void Log_Write_Data(uint8_t id, uint16_t value){}
static void Log_Write_Data(uint8_t id, int32_t value){}
static void Log_Write_Data(uint8_t id, uint32_t value){}
static void Log_Write_Data(uint8_t id, float value){}
static void Log_Write_Event(uint8_t id){}
static void Log_Write_Optflow() {}
static void Log_Write_Nav_Tuning() {}
static void Log_Write_Control_Tuning() {}
static void Log_Write_Performance() {}
static void Log_Write_Camera() {}
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code) {}
static void Log_Write_Baro(void);
static int8_t process_logs(uint8_t argc, const Menu::arg *argv) {
    return 0;
}

#endif // LOGGING_DISABLED
