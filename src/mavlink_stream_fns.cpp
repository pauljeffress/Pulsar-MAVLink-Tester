/*
 * mavlink_stream_fns.cpp
 *
 * mavlink specific functions
 *
 *
 */

#include "mavlink_fns.h"

#include "debug_fns.h"
#include "timer_fns.h"

// Define any globals that only the functions in this file need.

// Define functions.

/*
 * mavlink_request_datastream(...)
 * ===============================
 * Tells AP to turn on streaming of one of the predefined sets of
 * parameters, as defined in https://mavlink.io/en/messages/common.html#MAV_DATA_STREAM
 * 
 * INPUTS
 *    data_stream = https://mavlink.io/en/messages/common.html#MAV_DATA_STREAM
 *                  e.g MAV_DATA_STREAM_ALL or MAV_DATA_STREAM_RAW_SENSORS
 */
void mavlink_request_datastream(uint8_t data_stream)
{
    debugPrintln("mavlink_request_datastream() - Executing");
    // source and dest MAVLink addressing info.
    uint8_t _system_id = FMX_SYS_ID;        // MAVLink System ID of this device.
    uint8_t _component_id = FMX_COMP_ID;    // MAVLink Component ID of this device.
    uint8_t _target_system = AP_SYS_ID;     // MAVLink System ID of the autopilot.
    uint8_t _target_component = AP_COMP_ID; // MAVLink Component ID of the autopilot.

    uint8_t _req_stream_id = data_stream;   // MAV_DATA_STREAM_ALL;
    uint16_t _req_message_rate = 0x01;                    // number of times per second to request the data in hex
    uint8_t _start_stop = 1;                              // 1 = start, 0 = stop

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack and send the message
    mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); // Send the message (.write sends as bytes)
    Serial1.write(buf, len);                              // Write data to serial port
    
    debugPrintln("mavlink_request_datastream() - Complete");
} // END - mavlink_request_datastream()


/*
 * mavlink_unrequest_datastream(...)
 * ===============================
 * Tells AP to turn OFF streaming of one of the predefined sets of
 * parameters, as defined in https://mavlink.io/en/messages/common.html#MAV_DATA_STREAM
 * 
 * INPUTS
 *    data_stream = https://mavlink.io/en/messages/common.html#MAV_DATA_STREAM
 *                  e.g MAV_DATA_STREAM_ALL or MAV_DATA_STREAM_RAW_SENSORS
 */
void mavlink_unrequest_datastream(uint8_t data_stream)
{
    debugPrintln("mavlink_unrequest_datastream() - Executing");
    // source and dest MAVLink addressing info.
    uint8_t _system_id = FMX_SYS_ID;        // MAVLink System ID of this device.
    uint8_t _component_id = FMX_COMP_ID;    // MAVLink Component ID of this device.
    uint8_t _target_system = AP_SYS_ID;     // MAVLink System ID of the autopilot.
    uint8_t _target_component = AP_COMP_ID; // MAVLink Component ID of the autopilot.

    uint8_t _req_stream_id = data_stream; // use MAV_DATA_STREAM_ALL to stop all streams;
    uint16_t _req_message_rate = 0x01;                    // number of times per second to request the data in hex
    uint8_t _start_stop = 0;                              // 1 = start, 0 = stop

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack and send the message
    mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); // Send the message (.write sends as bytes)
    Serial1.write(buf, len);                              // Write data to serial port
    
    debugPrintln("mavlink_unrequest_datastream() - Complete");
} // END - mavlink_unrequest_datastream()


/*============================
 * mavlink_request_streaming_params_from_ap()
 *
 * Asks the AP to stream certain MAVLink messages, at a regular interval, on the Serial
 * link to the FMX.
 *
 * The FMX then uses other functions to then pull out what it wants from those streamed messages.
 *
 *============================*/
void mavlink_request_streaming_params_from_ap()
{
    debugPrintln("mavlink_request_streaming_params_from_ap() - Executing");
    
    // Prep source and dest MAVLink addressing info, to be used in below actions.
    // ==========================================================================
    uint8_t _system_id = FMX_SYS_ID;        // MAVLink System ID of this device.
    uint8_t _component_id = FMX_COMP_ID;    // MAVLink Component ID of this device.
    uint8_t _target_system = AP_SYS_ID;     // MAVLink System ID of the autopilot.
    uint8_t _target_component = AP_COMP_ID; // MAVLink Component ID of the autopilot.

    // Request just the individual MAVLink messages I am interested in and at a suitable interval
    // ==========================================================================================

    // NOTES
    // -----
    // ORIGINALLY I tried to use the MAV_CMD_REQUEST_MESSAGE method in a COMMAND_LONG (#76) (https://mavlink.io/en/messages/common.html#COMMAND_LONG) but I never
    // seemed to get a response, or even a COMMAND_ACK (#77) https://mavlink.io/en/messages/common.html#COMMAND_ACK back.
    // This was the method suggested in the ArduPilot doco at https://ardupilot.org/dev/docs/mavlink-requesting-data.html#using-request-message
    // but it either did not work or I was doing something wrong????
    //
    // The code I tried to use....
    // Create components of the MAVLink COMMAND_LONG message - https://mavlink.io/en/messages/common.html#COMMAND_LONG
    // uint16_t _cl_command      = MAV_CMD_REQUEST_MESSAGE; // MAV_CMD_SET_MESSAGE_INTERVAL; // https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE
    // uint8_t _cl_confirmation  = 0; // always 0 for first transmission, then incremented. https://mavlink.io/en/services/command.html#COMMAND_LONG
    // float   _cl_param1 = MAVLINK_MSG_ID_POWER_STATUS;   // MAVLink Message ID
    // float   _cl_param2 = 0; // Not used.
    // float   _cl_param3 = 0; // Not used, so set to zero.
    // float   _cl_param4 = 0; // Not used, so set to zero.
    // float   _cl_param5 = 0; // Not used, so set to zero.
    // float   _cl_param6 = 0; // Not used, so set to zero.
    // float   _cl_param7 = 0; // Not used, so set to zero.
    //

    // SO INSTEAD I fell back to using the streaming method (AP streams messages at regular intervals) and
    // used MAV_CMD_SET_MESSAGE_INTERVAL method in a COMMAND_LONG (#76) (https://mavlink.io/en/messages/common.html#COMMAND_LONG)
    //        see how ArduPilot code handles that request here https://github.com/ArduPilot/ardupilot/blob/477fb4c408fa0054f600e088fddcd0f8ab3bb4a9/libraries/GCS_MAVLink/GCS_Common.cpp#L915
    //                                                and here https://github.com/ArduPilot/ardupilot/blob/477fb4c408fa0054f600e088fddcd0f8ab3bb4a9/libraries/GCS_MAVLink/GCS_Common.cpp#L1288
    // to ask the AP to stream the messages I wanted at a regular interval, and that seemed to work.
    // The nominated messages would start flowing and I would also get the correct
    // COMMAND_ACK (#77) https://mavlink.io/en/messages/common.html#COMMAND_ACK back from the AutoPilot, acknowledging the COMMAND_LONG's
    // I sent it to set those message intervals.

    // The code below places all those requests...

    // Build 1st COMMAND_LONG / MAV_CMD_SET_MESSAGE_INTERVAL message.
    // components of the MAVLink COMMAND_LONG message - https://mavlink.io/en/messages/common.html#COMMAND_LONG
    uint16_t _cl_command = MAV_CMD_SET_MESSAGE_INTERVAL; // https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL
    uint8_t _cl_confirmation = 0;                        // always 0 for first transmission, then incremented. https://mavlink.io/en/services/command.html#COMMAND_LONG
    float _cl_param1 = MAVLINK_MSG_ID_POWER_STATUS;      // MAVLink Message ID
    float _cl_param2 = 1000000;                          // Interval (uS) between messages e.g. 1sec interval = 1000000uS
    float _cl_param3 = 0;                                // Not used, so set to zero.
    float _cl_param4 = 0;                                // Not used, so set to zero.
    float _cl_param5 = 0;                                // Not used, so set to zero.
    float _cl_param6 = 0;                                // Not used, so set to zero.
    float _cl_param7 = 0;                                // Not used, so set to zero.

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack and send the 1st message
    // Request MAVLINK_MSG_ID_POWER_STATUS (#125) - https://mavlink.io/en/messages/common.html#POWER_STATUS
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                              // Write data to serial port byte by byte.
    delay(500);                                           // give AP 1/2 sec to process before we send next CMD, don't want to DOS it.

    // Modify the COMMAND_LONG / MAV_CMD_SET_MESSAGE_INTERVAL message for the next message I want to request, then pack and send it.
    // Request MAVLINK_MSG_ID_HWSTATUS (#165) - https://mavlink.io/en/messages/ardupilotmega.html#HWSTATUS
    _cl_param1 = MAVLINK_MSG_ID_HWSTATUS; // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.
    delay(500);

    // Request MAVLINK_MSG_ID_GPS_RAW_INT (#24) - https://mavlink.io/en/messages/common.html#GPS_RAW_INT
    _cl_param1 = MAVLINK_MSG_ID_GPS_RAW_INT; // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.
    delay(500);

    // Request MAVLINK_MSG_ID_GLOBAL_POSITION_INT (#33) - https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT
    _cl_param1 = MAVLINK_MSG_ID_GLOBAL_POSITION_INT; // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.
    delay(500);

    // Request MAVLINK_MSG_ID_SYS_STATUS (#1) - https://mavlink.io/en/messages/common.html#SYS_STATUS
    _cl_param1 = MAVLINK_MSG_ID_SYS_STATUS; // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.
    delay(500);

    // Request MAVLINK_MSG_ID_SYSTEM_TIME (#2) - https://mavlink.io/en/messages/common.html#SYSTEM_TIME
    _cl_param1 = MAVLINK_MSG_ID_SYSTEM_TIME; // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.
    delay(500);

    // Request MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT (#62) - https://mavlink.io/en/messages/common.html#NAV_CONTROLLER_OUTPUT
    _cl_param1 = MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT; // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.
    delay(500);

    // Request MAVLINK_MSG_ID_BATTERY_STATUS (#147) - https://mavlink.io/en/messages/common.html#BATTERY_STATUS
    _cl_param1 = MAVLINK_MSG_ID_BATTERY_STATUS; // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.
    delay(500);

    // Request MAVLINK_MSG_ID_AUTOPILOT_VERSION (#148) - https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION
    _cl_param1 = MAVLINK_MSG_ID_AUTOPILOT_VERSION; // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.
    delay(500);

    debugPrintln("mavlink_request_streaming_params_from_ap() - Complete");
} // END - mavlink_request_streaming_params_from_ap()

/*============================
 * mavlink_unrequest_streaming_params_from_ap()
 *
 * Asks the AP to stop streaming certain MAVLink messages, that I had previously requested
 *
 * Reduces noise on the serial MAVLink connection to the FMX, makes it easier to work with.
 * It also helps keep the associated serial port buffers from being constantly bombarded
 * with data which pre fills those buffers such that when we actually want something in particular
 * we may miss it.
 *============================*/
void mavlink_unrequest_streaming_params_from_ap()
{
    debugPrintln("mavlink_unrequest_streaming_params_from_ap() - Executing");
    
    // Prep source and dest MAVLink addressing info, to be used in below actions.
    // ==========================================================================
    uint8_t _system_id = FMX_SYS_ID;        // MAVLink System ID of this device.
    uint8_t _component_id = FMX_COMP_ID;    // MAVLink Component ID of this device.
    uint8_t _target_system = AP_SYS_ID;     // MAVLink System ID of the autopilot.
    uint8_t _target_component = AP_COMP_ID; // MAVLink Component ID of the autopilot.

    // Build 1st COMMAND_LONG / MAV_CMD_SET_MESSAGE_INTERVAL message.
    // ==============================================================
    // components of the MAVLink COMMAND_LONG message - https://mavlink.io/en/messages/common.html#COMMAND_LONG
    uint16_t _cl_command = MAV_CMD_SET_MESSAGE_INTERVAL; // https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL
    uint8_t _cl_confirmation = 0;                        // always 0 for first transmission, then incremented. https://mavlink.io/en/services/command.html#COMMAND_LONG
    float _cl_param1 = MAVLINK_MSG_ID_POWER_STATUS;      // MAVLink Message ID
    float _cl_param2 = -1;                               // Interval (uS) between messages e.g. 1sec interval = 1000000uS or -1 to disable
    float _cl_param3 = 0;                                // Not used, so set to zero.
    float _cl_param4 = 0;                                // Not used, so set to zero.
    float _cl_param5 = 0;                                // Not used, so set to zero.
    float _cl_param6 = 0;                                // Not used, so set to zero.
    float _cl_param7 = 0;                                // Not used, so set to zero.

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack and send the 1st message
    // =============================
    // UnRequest MAVLINK_MSG_ID_POWER_STATUS (#125) - https://mavlink.io/en/messages/common.html#POWER_STATUS
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len); //Write data to serial port byte by byte.
    delay(500); // give AP 1/2 sec to process before we send next CMD, don't want to DOS it.

    // Pack and send remaining messages
    // ================================
    // Modify the COMMAND_LONG / MAV_CMD_SET_MESSAGE_INTERVAL message for the next message I want to request, then pack and send it.
    // UnRequest MAVLINK_MSG_ID_HWSTATUS (#165) - https://mavlink.io/en/messages/ardupilotmega.html#HWSTATUS
    _cl_param1 = MAVLINK_MSG_ID_HWSTATUS; // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.
    delay(500);

    // UnRequest MAVLINK_MSG_ID_GPS_RAW_INT (#24) - https://mavlink.io/en/messages/common.html#GPS_RAW_INT
    _cl_param1 = MAVLINK_MSG_ID_GPS_RAW_INT; // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.
    delay(500);

    // UnRequest MAVLINK_MSG_ID_GLOBAL_POSITION_INT (#33) - https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT
    _cl_param1 = MAVLINK_MSG_ID_GLOBAL_POSITION_INT; // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.
    delay(500);

    // UnRequest MAVLINK_MSG_ID_SYS_STATUS (#1) - https://mavlink.io/en/messages/common.html#SYS_STATUS
    _cl_param1 = MAVLINK_MSG_ID_SYS_STATUS; // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.
    delay(500);

    // UnRequest MAVLINK_MSG_ID_SYSTEM_TIME (#2) - https://mavlink.io/en/messages/common.html#SYSTEM_TIME
    _cl_param1 = MAVLINK_MSG_ID_SYSTEM_TIME; // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.
    delay(500);

    // UnRequest MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT (#62) - https://mavlink.io/en/messages/common.html#NAV_CONTROLLER_OUTPUT
    _cl_param1 = MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT; // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.
    delay(500);

    // UnRequest MAVLINK_MSG_ID_BATTERY_STATUS (#147) - https://mavlink.io/en/messages/common.html#BATTERY_STATUS
    _cl_param1 = MAVLINK_MSG_ID_BATTERY_STATUS; // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.
    delay(500);

    // UnRequest MAVLINK_MSG_ID_AUTOPILOT_VERSION (#148) - https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION
    _cl_param1 = MAVLINK_MSG_ID_AUTOPILOT_VERSION; // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.
    delay(500);

    // UnRequest MAVLINK_MSG_ID_RC_CHANNELS_RAW (#35) - https://mavlink.io/en/messages/common.html#RC_CHANNELS_RAW
    _cl_param1 = MAVLINK_MSG_ID_RC_CHANNELS_RAW; // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.
    delay(500);

    // UnRequest MAVLINK_MSG_ID_SERVO_OUTPUT_RAW (#36) - https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW
    _cl_param1 = MAVLINK_MSG_ID_SERVO_OUTPUT_RAW; // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.
    delay(500);

    // UnRequest MAVLINK_MSG_ID_RC_CHANNELS (#65) - https://mavlink.io/en/messages/common.html#RC_CHANNELS
    _cl_param1 = MAVLINK_MSG_ID_RC_CHANNELS; // MAVLink Message ID
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.
    delay(500);

    debugPrintln("mavlink_unrequest_streaming_params_from_ap() - Complete");
} // END - mavlink_unrequest_streaming_params_from_ap()
