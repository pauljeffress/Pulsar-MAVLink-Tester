/*
 * mavlink_receive_fns.cpp
 *
 * This file only contains one function
 *   - mavlink_receive()
 *
 *
 */

#include "mavlink_fns.h"

#include "debug_fns.h"
#include "timer_fns.h"

// Define any globals that only the functions in this file need.

uint8_t sys1comp1_expectedSeqNum = 0;
uint8_t sys1comp0_expectedSeqNum = 0;


// Define functions.

/*============================
 * mavlink_receive()
 *
 * Function called to read any MAVlink messages sent by serial communication from AutoPilot(AP) to Arduino.
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
    // debugPrintln("char?");
    while (Serial1.available() && !gotFullMsg) // xxx - I should prob put a time limiter on this WHILE, I think the only reason
                                               // it is not hogging the CPU is because the AutoPilot cube only sends msgs each
                                               // second, and then pauses I think, is why the WHILE breaks out.
    {
        uint8_t c = Serial1.read();
        // debugPrintln("got char");
        //  add new char to what we have so far and see of we have a full Mavlink msg yet
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) // if we do then bust out of this char collecting loop.
            gotFullMsg = true;
    }

    // At this point we either have a Full Msg OR there were no more chars available on the MAVLink serial link.

    if (gotFullMsg) // if we do then lets process it
    {
#ifdef MAVLINK_DEBUG
        // debugs to show the msg # of the ones I'm seeing but not interested in.
        
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

        debugPrint("mavlink_receive() - MSG RCVD -");
        debugPrint(" magic:");
        debugPrintInt(msg.magic);
        debugPrint(" seq:");
        debugPrintInt(msg.seq);
        debugPrint(" src sysid:");
        debugPrintInt(msg.sysid);
        debugPrint(" src compid:");
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
            Serial.print(" Autopilot:");
            Serial.print(hb.autopilot);
            Serial.print(" BaseMode:");
            Serial.print(hb.base_mode);
            Serial.print(" CustomMode/Flightmode:");
            Serial.print(hb.custom_mode);
            Serial.print(" SystemStatus:");
            Serial.print(hb.system_status);
            Serial.print(" MavlinkVersion:");
            Serial.print(hb.mavlink_version);
#endif

            seconds_since_last_mavlink_heartbeat_rx = 0; // reset this timer as we just got a HEARTBEAT from the AP.


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

            // Save things I'm interested in to FeatherMx data structure for use later.
            // See example of this in "case MAVLINK_MSG_ID_HEARTBEAT:" above.

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
        // DEFAULT - should not happen, but programing it defensively
        default:
            Serial.println("mavlink_receive() - WARNING - we hit the default: in mavlink packet decode switch");
            break;

        } // END - of msg decoder switch

#ifdef MAVLINK_DEBUG
        debugPrintln("");
#endif
    } // END - of IF we have a full mavlink packet lets process it

} // END - mavlink_receive()
