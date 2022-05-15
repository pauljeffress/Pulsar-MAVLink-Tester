/*
 * mavlink_receive_fns.cpp
 *
 * This file only contains one function
 *   - mavlink_receive()
 *
 *
 */

#include "mavlink_fns.h"
#include <TimeLib.h>
#include "debug_fns.h"
#include "timer_fns.h"
#ifdef FMX
    #include "FmxSettings_fns.h"
#endif

// Define any globals that only the functions in this file need.

uint8_t sys1comp1_expectedSeqNum = 0; // used to keep track of MAVLink sequence numbers of msgs received from AP (sysID = 1, CompID = 1)
uint8_t sys1comp0_expectedSeqNum = 0; // used to keep track of MAVLink sequence numbers of msgs received from ADSB subcomponent (sysID = 1, CompID = 0)

// Define functions.

/*============================
 * mavlink_param_value_receive()
 *
 * Looks for a MAVLink PARAM_VALE (#22) msg, with specific param_id/name, in msgs received from the AP.
 * If it finds it, before we hit the timeout, it returns the value etc.
 *
 * This function is often called during a mavlink_get_one_param_from_ap(), after the specific
 * param has been requested.
 *
 *
 * INPUTS
 *  name        - the param_id/name of the param we want to get e.g "BATT_ARM_VOLT"
 *  valuetype   - is the value we are requesting an INT or a FLOAT? https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE
 *
 * OUTPUT
 *  Either value_int OR value_float will have been set if we were successful, depending on valuetype
 *  AND always set the other to 0 (just for tidiness)
 *
 * RETURNS
 *  TRUE    - if we got the param's value successfully from the AP
 *  FALSE   - if not
 *
 *============================*/
bool mavlink_receive_param_value(char *name, int32_t *value_int, float *value_float, uint8_t valuetype)
{
    debugPrintln("mavlink_receive_param_value() - Starting");

    debugPrint("mavlink_receive_param_value() - name:");
    Serial.println(name);
    debugPrint("mavlink_receive_param_value() - value_int:");
    Serial.println(*value_int);
    debugPrint("mavlink_receive_param_value() - value_float:");
    Serial.println(*value_float);
    debugPrint("mavlink_receive_param_value() - valuetype:");
    Serial.println(valuetype);

    mavlink_message_t msg;
    mavlink_status_t status;
    
    char param_i_want_to_get[16] = {}; // e.g. "BATT_ARM_VOLT";

    bool gotDesiredMsg = false;
    bool timedOutOuter = false; // track when we have timed out for the outer while()
    uint32_t startOuter = 0;

    bool gotFullMsg = false;
    bool timedOutInner = false; // track when we have timed out for the inner while()
    uint32_t startInner = 0;

    memcpy(param_i_want_to_get, name, sizeof(param_i_want_to_get));

    // The Outer while() - will keep looking for MAVLink PARAM_VALE (#22) msg AND with the desired param_id/name or timeout
    startOuter = millis(); // take a millis() timestamp when we start the outer while()
    while (!gotDesiredMsg && !timedOutOuter)
    {
        //debugPrint(" o ");
        // The Inner while() - will keep trying to assemble a Full MAVLink msg (of any type) or timeout
        startInner = millis(); // take a millis() timestamp when we start the inner while()
        timedOutInner = false;
        while (Serial1.available() && !gotFullMsg && !timedOutInner)
        {
            uint8_t c = Serial1.read();
            //  add new char to what we have so far
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) // and see of we have a full Mavlink msg yet
            {
                gotFullMsg = true; // will cause us to bust out of the while()
                //debugPrintln("F");
            }
            else
            {
                //debugPrintln("c");
            }

            // Check if we have timed out, which will be evaluated at next pass through the while()
            if ((millis() - startInner) >= (FMX_MAVLINK_RX_WINDOW_REGULAR_MS))
            {
                timedOutInner = true;
                debugPrintln("mavlink_receive_param_value() - WARNING - we timed out on Inner while()");
            }
        } // END of the inner while()

        // At this point we either;
        // a) have a Full Msg OR
        // b) there were no more chars available on the MAVLink serial link so we just exit OR
        // c) we timed out before gathering a Full Msg, so we just exit.

        if (gotFullMsg) // if we do then lets process it, else we exit.
        {
            debugPrint("msg#"); debugPrintInt(msg.msgid);
            gotFullMsg = false; // clear the flag for future re use.
            if (msg.msgid == MAVLINK_MSG_ID_PARAM_VALUE) //  #22  https://mavlink.io/en/messages/common.html#PARAM_VALUE
            {
                // decode the body of the PARAM_VALUE msg
                mavlink_param_value_t param_value;
                mavlink_msg_param_value_decode(&msg, &param_value);
                debugPrint("mavlink_receive_param_value() - got a full MAVLink PARAM_VALUE msg - param_id:"); Serial.println(param_value.param_id);
                if (param_value.param_id[0] == param_i_want_to_get[0]) // is it for the specific PARAM we are looking for?
                {
                    // XXX - the above if (...) needs to match the whole string, not just first char
                    debugPrintln("mavlink_receive_param_value() - and its the one we wanted");
                    gotDesiredMsg = true;
                    // print its guts
                    debugPrintln("PARAM_VALUE");
                    Serial.print("    param_id:");Serial.println(param_value.param_id);
                    Serial.print(" param_value:");Serial.println(param_value.param_value);
                    Serial.print("  param_type:");Serial.println(param_value.param_type);
                    Serial.print(" param_count:");Serial.println(param_value.param_count);
                    Serial.print(" param_index:");Serial.println(param_value.param_index);

                    // use its guts
                    if ((param_value.param_value >= MAV_PARAM_TYPE_UINT8) && (param_value.param_value <= MAV_PARAM_TYPE_INT32)) // i.e. an INT
                    {
                        *value_int = param_value.param_value;
                        *value_float = 0.0;
                    }
                    if (param_value.param_type == MAV_PARAM_TYPE_REAL32)
                    {
                        *value_float = param_value.param_value;
                        *value_int = 0;
                    }
                } // END - of IF we have the correct param_id
            }     // END - of IF we have a PARAM_VALUE mavlink packet
        }         // END - of IF we have a full mavlink packet

        // Check if we have timed out on the Outer while(), which will be evaluated at next pass through the while()
        if ((millis() - startOuter) >= (FMX_MAVLINK_RX_WINDOW_LONG_MS))
        {
            timedOutOuter = true;
            debugPrintln("mavlink_receive_param_value() - WARNING - we timed out on Outer while()");
        }

    } // END of while() outer loop

    debugPrint("mavlink_receive_param_value() - Complete. Result=");
    if (gotDesiredMsg)
        debugPrintln("TRUE");
    else
        debugPrintln("FALSE");

    return (gotDesiredMsg); // tell our caller if we succeeded in getting the PARAM_VALUE MAVlink msg, with the desired param_id/name.

} // END - mavlink_receive_param_value()

/*============================
 * mavlink_receive()
 *
 * Assembles inbound MAVlink messages (character by character) sent by serial communication from AutoPilot(AP) to Arduino.
 * It may take multiple calls of this function before it has assembled a Full MAVLink msg.
 * Once it has a Full MAVLink msg, we process it.
 * We only ever process a maximum of one msg per call of this function.
 * We use a timeout to ensure this function is timebound.
 *
 * This function is called for two reasons;
 * i) To receive messages the AP is streaming out its serial port, after we have requested then via mavlink_request_datastream()
 *  OR
 * ii) It could be to look for a specific response to a specific request from the Arduino to the AP. For example when I set a Mission.
 *
 *============================*/
void mavlink_receive()
{
    // debugPrintln("mavlink_receive() - Executing");
    mavlink_message_t msg;
    mavlink_status_t status;

    bool gotFullMsg = false;

    bool timedOut = false; // track when we have timed out.
    uint32_t start = 0;

    start = millis(); // take a millis() timestamp when we start.

    while (Serial1.available() && !gotFullMsg && !timedOut)
    {
        uint8_t c = Serial1.read();
        //  add new char to what we have so far
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) // and see of we have a full Mavlink msg yet
            gotFullMsg = true;                                    // will cause us to bust out of the while()

        // Check if we have timed out, which will be evaluated at next pass through the while()
        if ((millis() - start) >= (FMX_MAVLINK_RX_WINDOW_REGULAR_MS))
        {
            timedOut = true;
            debugPrintln("void mavlink_receive() - WARNING - we timed out");
        }
    }

    // At this point we either;
    // a) have a Full Msg OR
    // b) there were no more chars available on the MAVLink serial link so we just exit OR
    // c) we timed out before gathering a Full Msg, so we just exit.

    if (gotFullMsg && (msg.sysid == 1) && (msg.compid == 1)) // if we do AND its from the AutoPilot (not the ADSB module etc) then lets process it
    {

        // Check if we have missed any msg's my checking seq number of received MAVLink msg against last on from that sysID/compID.
        // Note (1): the first msg we receive from a particular sysID/compID will always fail the below test, thats ok and expected.
        // Note (2): my Cube Orange identifies itself as sysID 1, compID 1.  The ADSB controller that is on the Cube Orange carrier board
        // also talks MAVLink and identifies itself as sysID 1, compID 0.  So you see msgs from both in my streams.
        if ((msg.sysid == 1) && (msg.compid == 1))  // i.e we have message from my autopilot
        {
            if (msg.seq != (sys1comp1_expectedSeqNum))    // is it the next one in the sequence?
                debugPrintln("mavlink_receive() - WARNING - MSG(s) missed from sysID:1,compID:1 according to seq nums!");
            sys1comp1_expectedSeqNum = msg.seq+1;   // as its a uint8_t it will roll correctly at seq = 255, the next will be seq = 0.
        }
        // I have commented out the below check, as the ADSB controller does not emit sequential seq nums.
        // if ((msg.sysid == 1) && (msg.compid == 0))  // i.e we have message from my ADSB controller
        // {
        //     if (msg.seq != (sys1comp0_expectedSeqNum))    // is it the next one in the sequence?
        //         debugPrintln("mavlink_receive() - WARNING - MSG(s) missed from sysID:1,compID:0 according to seq nums!");
        //     sys1comp0_expectedSeqNum = msg.seq+1;   // as its a uint8_t it will roll correctly at seq = 255, the next will be seq = 0.
        // }
#ifdef MAVLINK_DEBUG
        debugPrint("MSG RCVD -");
        debugPrint(" magic:");
        debugPrintInt(msg.magic);
        debugPrint(" seq:");
        debugPrintInt(msg.seq);
        debugPrint(" sysid:");
        debugPrintInt(msg.sysid);
        debugPrint(" compid:");
        debugPrintInt(msg.compid);
        debugPrint(" msgid#:");
        debugPrintInt(msg.msgid);
#endif

        // Decode new message from autopilot
        switch (msg.msgid) // ArduPilot Rover specific code that sends these from the AP - https://github.com/ArduPilot/ardupilot/blob/master/Rover/GCS_Mavlink.cpp
                           // ArduPilot common code that sends these from the AP - https://github.com/ArduPilot/ardupilot/blob/477fb4c408fa0054f600e088fddcd0f8ab3bb4a9/libraries/GCS_MAVLink/GCS_Common.cpp#L5045
        {

        //============================
        case MAVLINK_MSG_ID_HEARTBEAT: //  #0  https://mavlink.io/en/messages/common.html#HEARTBEAT
        {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);

#ifdef MAVLINK_DEBUG
            debugPrint("=HEARTBEAT");
            Serial.print(" Type:");
            Serial.print(hb.type);
            Serial.print(" APclass:");
            Serial.print(hb.autopilot);
            Serial.print(" BaseMode:");
            Serial.print(hb.base_mode);
           if ((hb.base_mode == 1) || (hb.base_mode == 65))  // https://ardupilot.org/rover/docs/parameters.html#mode1
                Serial.print( " DISarmed");
            if ((hb.base_mode == 129) || (hb.base_mode == 193))
                Serial.print( " !!ARMED!!");
            Serial.print(" Custom/Flightmode:");
            Serial.print(hb.custom_mode);
            if (hb.custom_mode == 0)
                Serial.print( " MANUAL");
            if (hb.custom_mode == 4)
                Serial.print( " HOLD");
            if (hb.custom_mode == 10)
                Serial.print( " !!AUTO!!");
            Serial.print(" SysStat:");
            Serial.print(hb.system_status);
            Serial.print(" MavVer:");
            Serial.print(hb.mavlink_version);

#endif

            seconds_since_last_mavlink_heartbeat_rx = 0; // reset this timer as we just got a HEARTBEAT from the AP.

#ifdef FMX
            // Save things I'm interested in to FeatherMx data structure for use later.
            // Note, because the GCS and even the ADSB sub controller in the Cube baseboard are separate MAVLink "Components", they emit
            // their own HEARTBEAT msgs. SO below I need to check that I ONLY copy data from the AutoPilot HEARTBEATS, and not the GCS
            // or ADSB node HEARTBEATS.
            if (hb.type == MAV_TYPE_SURFACE_BOAT)
            {
                myFmxSettings.AP_BASEMODE = hb.base_mode;         // https://github.com/ArduPilot/ardupilot/blob/477fb4c408fa0054f600e088fddcd0f8ab3bb4a9/Rover/GCS_Mavlink.cpp#L15
                myFmxSettings.AP_CUSTOMMODE = hb.custom_mode;     // https://github.com/ArduPilot/ardupilot/blob/477fb4c408fa0054f600e088fddcd0f8ab3bb4a9/Rover/GCS_Mavlink.cpp#L52
                myFmxSettings.AP_SYSTEMSTATUS = hb.system_status; // https://github.com/ArduPilot/ardupilot/blob/477fb4c408fa0054f600e088fddcd0f8ab3bb4a9/Rover/GCS_Mavlink.cpp#L57
            }
#endif
            break;

        }

        //============================
        case MAVLINK_MSG_ID_PARAM_VALUE: //  #22  https://mavlink.io/en/messages/common.html#PARAM_VALUE
        {
            mavlink_param_value_t param_value;
            mavlink_msg_param_value_decode(&msg, &param_value);

#ifdef MAVLINK_DEBUG
            debugPrint("=PARAM_VALUE");
            Serial.print(" param_id:");
            Serial.print(param_value.param_id);
            Serial.print(" param_value:");
            Serial.print(param_value.param_value);
            Serial.print(" param_type:");
            Serial.print(param_value.param_type);
            Serial.print(" param_count");
            Serial.print(param_value.param_count);
            Serial.print(" param_index");
            Serial.print(param_value.param_index);
#endif

            break;
        }

        //============================
        case MAVLINK_MSG_ID_GPS_RAW_INT: //  #24  https://mavlink.io/en/messages/common.html#GPS_RAW_INT
        {
            mavlink_gps_raw_int_t packet;
            mavlink_msg_gps_raw_int_decode(&msg, &packet);

#ifdef MAVLINK_DEBUG
            time_t t = packet.time_usec / 1000000; // time from GPS in mavlink is uint64_t in microseconds
                                                   // so I divide by 1,000,000 to get it in seconds
                                                   // which is what the hour(), minute() etc functions expect
            debugPrint("=GPS_RAW_INT");
            Serial.print(" Time:");
            Serial.print(hour(t));
            Serial.print("h:");
            Serial.print(minute(t));
            Serial.print("m:");
            Serial.print(second(t));
            Serial.print("s FixType:");
            Serial.print(packet.fix_type);
            Serial.print(" Latitude:");
            Serial.print(packet.lat);
            Serial.print(" Longitude:");
            Serial.print(packet.lon);
            Serial.print(" GroundSpeed:");
            Serial.print(packet.vel);
            Serial.print(" CoG(deg):");
            Serial.print(packet.cog);
            Serial.print(" SatsVisible:");
            Serial.print(packet.satellites_visible);
#endif

#ifdef FMX
            // Save things I'm interested in to FeatherMx data structure for use later.
            myFmxSettings.AP_VEL = packet.vel;
            myFmxSettings.AP_COG = packet.cog;
            myFmxSettings.AP_SATS = packet.satellites_visible;
#endif
            break;
        }

        //============================
        case MAVLINK_MSG_ID_HWSTATUS: //  #165  https://mavlink.io/en/messages/ardupilotmega.html#HWSTATUS
        {
            mavlink_hwstatus_t packet;
            mavlink_msg_hwstatus_decode(&msg, &packet);

#ifdef MAVLINK_DEBUG
            debugPrint("=HWSTATUS");
            debugPrint(" Vcc:");
            Serial.print(packet.Vcc);
            debugPrint("mV I2Cerr:");
            Serial.print(packet.I2Cerr);
            debugPrint("errors");
#endif
            break;
        }

        //============================
        case MAVLINK_MSG_ID_POWER_STATUS: //  #125  https://mavlink.io/en/messages/common.html#POWER_STATUS
        {                                 // https://github.com/ArduPilot/ardupilot/blob/477fb4c408fa0054f600e088fddcd0f8ab3bb4a9/libraries/GCS_MAVLink/GCS_Common.cpp#L178
            mavlink_power_status_t packet;
            mavlink_msg_power_status_decode(&msg, &packet);

#ifdef MAVLINK_DEBUG
            debugPrint("=POWER_STATUS");
            debugPrint(" Vcc:");
            Serial.print(packet.Vcc);
            debugPrint("mV Vservo:");
            Serial.print(packet.Vservo);
            debugPrint("mV flags:");
            Serial.print(packet.flags);
#endif

#ifdef FMX
            // Save things I'm interested in to FeatherMx data structure for use later.
            myFmxSettings.AP_VCC = packet.Vcc;
            myFmxSettings.AP_VSERVO = packet.Vservo;
            myFmxSettings.AP_POWERFLAGS = packet.flags;
#endif
            break;
        }

        //============================
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: //  #33  https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT
        {
            mavlink_global_position_int_t packet;
            mavlink_msg_global_position_int_decode(&msg, &packet);

#ifdef MAVLINK_DEBUG
            debugPrint("=GLOBAL_POSITION_INT");
            debugPrint(" TimeSinceBoot:");
            Serial.print(packet.time_boot_ms);
            debugPrint("mS LAT:");
            Serial.print(packet.lat);
            debugPrint("degE7 LON:");
            Serial.print(packet.lon);
            debugPrint("degE7 MSL_ALT:");
            Serial.print(packet.alt);
            debugPrint("mm REL_ALT:");
            Serial.print(packet.relative_alt);
            debugPrint("mm VelX:");
            Serial.print(packet.vx);
            debugPrint("cm/s VelY:");
            Serial.print(packet.vy);
            debugPrint("cm/s VelZ:");
            Serial.print(packet.vz);
            debugPrint("cm/s Heading:");
            Serial.print(packet.hdg);
            debugPrint("cdeg");
#endif

#ifdef FMX
            // Save things I'm interested in to FeatherMx data structure for use later.
            myFmxSettings.AP_POSITIONTIMESTAMP = packet.time_boot_ms;
            myFmxSettings.AP_LAT = packet.lat;
            myFmxSettings.AP_LON = packet.lon;
            myFmxSettings.AP_VX = packet.vx;
            myFmxSettings.AP_VY = packet.vy;
            myFmxSettings.AP_HDG = packet.hdg;
#endif
            break;
        }

        //============================
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW: //  #35  https://mavlink.io/en/messages/common.html#RC_CHANNELS_RAW
        {
#ifdef MAVLINK_DEBUG
            debugPrint("=RC_CHANNELS_RAW - undecoded");
#endif

            break;
        }

        //============================
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: //  #36  https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW
        {
#ifdef MAVLINK_DEBUG
            debugPrint("=SERVO_OUTPUT_RAW - undecoded");
#endif

            break;
        }

        //============================
        case MAVLINK_MSG_ID_RC_CHANNELS: //  #65  https://mavlink.io/en/messages/common.html#RC_CHANNELS
        {
#ifdef MAVLINK_DEBUG
            debugPrint("=RC_CHANNELS - undecoded");
#endif

            break;
        }

        //============================
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: //  #66  https://mavlink.io/en/messages/common.html#REQUEST_DATA_STREAM
        {
            mavlink_request_data_stream_t packet;
            mavlink_msg_request_data_stream_decode(&msg, &packet);

#ifdef MAVLINK_DEBUG
            debugPrint("=REQUEST_DATA_STREAM");
            debugPrint(" target_system:");
            Serial.print(packet.target_system);
            debugPrint("target_comp:");
            Serial.print(packet.target_component);
            debugPrint("req_stream_id:");
            Serial.print(packet.req_stream_id);
            debugPrint("req_message_rate:");
            Serial.print(packet.req_message_rate);
            debugPrint("start_stop:");
            Serial.print(packet.start_stop);
#endif
            break;
        }

        //============================
        case MAVLINK_MSG_ID_TIMESYNC: //  #111  https://mavlink.io/en/messages/common.html#TIMESYNC
        {
#ifdef MAVLINK_DEBUG
            debugPrint("=TIMESYNC - undecoded");
#endif

            break;
        }

        //============================
        case MAVLINK_MSG_ID_SYS_STATUS: //  #1  https://mavlink.io/en/messages/common.html#SYS_STATUS
        {                               //
            mavlink_sys_status_t packet;
            mavlink_msg_sys_status_decode(&msg, &packet);

#ifdef MAVLINK_DEBUG
            debugPrint("=SYS_STATUS");
            debugPrint(" onboard_control_sensors_present:");
            Serial.print(packet.onboard_control_sensors_present);
            debugPrint(" onboard_control_sensors_enabled:");
            Serial.print(packet.onboard_control_sensors_enabled);
            debugPrint(" onboard_control_sensors_health:");
            Serial.print(packet.onboard_control_sensors_health);
            debugPrint(" load:");
            Serial.print(packet.load);
#endif

#ifdef FMX
            // Save things I'm interested in to FeatherMx data structure for use later.
            myFmxSettings.AP_SENSORSPRESENT = packet.onboard_control_sensors_present;
            myFmxSettings.AP_SENSORSENABLED = packet.onboard_control_sensors_enabled;
            myFmxSettings.AP_SENSORSHEALTH = packet.onboard_control_sensors_health;
            myFmxSettings.AP_LOAD = packet.load;
#endif
            break;
        }

        //============================
        case MAVLINK_MSG_ID_SYSTEM_TIME: //  #2  https://mavlink.io/en/messages/common.html#SYSTEM_TIME
        {
            mavlink_system_time_t packet;
            mavlink_msg_system_time_decode(&msg, &packet);

#ifdef MAVLINK_DEBUG
            debugPrint("=SYSTEM_TIME");
            debugPrint(" time_unix_usec:");
            Serial.print("I can't print uint64_t's here");
            debugPrint(" time_boot_ms:");
            Serial.print(packet.time_boot_ms);
#endif

#ifdef FMX
            // Save things I'm interested in to FeatherMx data structure for use later.
            myFmxSettings.AP_TIMEUNIXUSEC = packet.time_unix_usec;
            myFmxSettings.AP_TIMEBOOTMS = packet.time_boot_ms;
#endif
            break;
        }

        //============================
        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: // https://mavlink.io/en/messages/common.html#NAV_CONTROLLER_OUTPUT
        {                                          // https://github.com/ArduPilot/ardupilot/blob/477fb4c408fa0054f600e088fddcd0f8ab3bb4a9/Rover/GCS_Mavlink.cpp#L100
            mavlink_nav_controller_output_t packet;
            mavlink_msg_nav_controller_output_decode(&msg, &packet);

#ifdef MAVLINK_DEBUG
            debugPrint("=NAV_CONTROLLER_OUTPUT");
            debugPrint(" nav_bearing:");
            Serial.print(packet.nav_bearing);
            debugPrint(" target_bearing:");
            Serial.print(packet.target_bearing);
            debugPrint(" wp_dist:");
            Serial.print(packet.wp_dist);
#endif

#ifdef FMX
            // Save things I'm interested in to FeatherMx data structure for use later.
            myFmxSettings.AP_NAVBEARING = packet.nav_bearing;
            myFmxSettings.AP_TARGETBEARING = packet.target_bearing;
            myFmxSettings.AP_WPDIST = packet.wp_dist;
#endif
            break;
        }

        //============================
        case MAVLINK_MSG_ID_BATTERY_STATUS: //  #147  https://mavlink.io/en/messages/common.html#BATTERY_STATUS
        {                                   // https://github.com/ArduPilot/ardupilot/blob/477fb4c408fa0054f600e088fddcd0f8ab3bb4a9/libraries/GCS_MAVLink/GCS_Common.cpp#L217
                                            // https://github.com/ArduPilot/ardupilot/blob/477fb4c408fa0054f600e088fddcd0f8ab3bb4a9/libraries/GCS_MAVLink/GCS_Common.cpp#L304
            mavlink_battery_status_t packet;
            mavlink_msg_battery_status_decode(&msg, &packet);

#ifdef MAVLINK_DEBUG
            debugPrint("=BATTERY_STATUS");
            debugPrint(" voltages[0]:");
            Serial.print(packet.voltages[0]);
            debugPrint(" voltages[1]:");
            Serial.print(packet.voltages[1]);
            debugPrint(" current_battery:");
            Serial.print(packet.current_battery);
#endif

#ifdef FMX
            // Save things I'm interested in to FeatherMx data structure for use later.
            myFmxSettings.AP_VOLTAGES[0] = packet.voltages[0];
            myFmxSettings.AP_VOLTAGES[1] = packet.voltages[1];
            myFmxSettings.AP_CURRENTBATTERY = packet.current_battery;
#endif
            break;
        }

        //============================
        case MAVLINK_MSG_ID_AUTOPILOT_VERSION: //  #148  https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION
        {
            mavlink_autopilot_version_t packet;
            mavlink_msg_autopilot_version_decode(&msg, &packet);

#ifdef MAVLINK_DEBUG
            debugPrint("=AUTOPILOT_VERSION");
            debugPrint(" vendor_id:");
            Serial.print(packet.vendor_id);
            debugPrint(" product_id:");
            Serial.print(packet.product_id);
#endif

#ifdef FMX
            // Save things I'm interested in to FeatherMx data structure for use later.
            myFmxSettings.AP_VENDORID = packet.vendor_id;
            myFmxSettings.AP_PRODUCTID = packet.product_id;
#endif
            break;
        }

        //============================
        case MAVLINK_MSG_ID_COMMAND_ACK: //  #77  https://mavlink.io/en/messages/common.html#COMMAND_ACK
        {
            mavlink_command_ack_t ca;
            mavlink_msg_command_ack_decode(&msg, &ca);

#ifdef MAVLINK_DEBUG
            debugPrint("=COMMAND_ACK");
            Serial.print(" command:");
            Serial.print(ca.command);
            Serial.print(" result:");
            Serial.print(ca.result);
#endif
            break;
        }

        //============================
        case MAVLINK_MSG_ID_STATUSTEXT: //  #253  https://mavlink.io/en/messages/common.html#STATUSTEXT
        {
            mavlink_statustext_t packet;
            mavlink_msg_statustext_decode(&msg, &packet);

#ifdef MAVLINK_DEBUG
            debugPrint("=STATUSTEXT");
            debugPrint(" severity:");
            Serial.print(packet.severity);
            debugPrint(" text:");
            Serial.print(packet.text);
#endif
            break;
        }

        //============================
        // DEFAULT - should not happen, but programing it defensively
        default:
            Serial.print(" - WARNING - we hit the default: in mavlink packet decode switch");
            break;

        } // END - of msg decoder switch

#ifdef MAVLINK_DEBUG
        debugPrintln("");
#endif

    } // END - of IF we have a full mavlink packet lets process it

} // END - mavlink_receive()
