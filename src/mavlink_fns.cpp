/*
 * mavlink_fns.cpp
 *
 * Assorted MAVLink functions
 *
 *
 */

#include "mavlink_fns.h"

#include "debug_fns.h"
#include "timer_fns.h"

// Define any globals that only the functions in this file need.

// Define functions.

/*============================
 * initMAVLinkSettings()
 *
 * Initialise anything that needs initialising to do with MAVLink.
 *============================*/
void initMAVLinkSettings()
{
    global_mission_init();
}

/*============================
 * mavlink_fmx_send_heartbeat_to_ap()
 *
 * Sends the correctly formatted MAVLink HEARTBEAT msg from FMX to AP
 * https://mavlink.io/en/messages/common.html#HEARTBEAT
 *============================*/
void mavlink_fmx_send_heartbeat_to_ap()
{
    // debugPrintln("mavlink_fmx_send_heartbeat_to_ap() - Executing");
    //  source and dest MAVLink addressing info.
    uint8_t _system_id = FMX_SYS_ID;     // MAVLink System ID of this device.
    uint8_t _component_id = FMX_COMP_ID; // MAVLink Component ID of this device.

    // components of the MAVLink HEARTBEAT message - https://mavlink.io/en/messages/common.html#HEARTBEAT
    uint8_t _hb_type = MAV_TYPE_SURFACE_BOAT;      // https://mavlink.io/en/messages/common.html#MAV_TYPE
    uint8_t _hb_autopilot = MAV_AUTOPILOT_INVALID; // https://mavlink.io/en/messages/common.html#MAV_AUTOPILOT
    uint8_t _hb_basemode = 0;                      // https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG
                                                   // None of the options looked appropriate so I have set to 0 for now.
    uint32_t _hb_custom_mode = 0;                  // None of the options looked appropriate so I have set to 0 for now.
    uint8_t _hb_system_status = MAV_STATE_ACTIVE;  // https://mavlink.io/en/messages/common.html#MAV_STATE

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_heartbeat_pack(_system_id, _component_id, &msg, _hb_type, _hb_autopilot, _hb_basemode, _hb_custom_mode, _hb_system_status);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                              // Write data to serial port byte by byte.

} // END - mavlink_fmx_send_heartbeat_to_ap()

/*============================
 * mavlink_test_request_one_param_from_ap()
 *
 * A basic test of reading one param from the AP using the "PARAM_REQUEST_READ" #20 msg method. https://mavlink.io/en/messages/common.html#PARAM_REQUEST_READ
 *
 * The parameter we want test the request with is hard coded in this function as "BATT_ARM_VOLT".
 * See mavlink_request_one_param_on_ap() for a function that lets you request any param.
 * 
 *============================*/
void mavlink_test_request_one_param_from_ap()
{
    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint16_t len;

    // MAV_TYPE
    // BATT_ARM_VOLT
    const char param_i_want[16] = "BATT_ARM_VOLT";

    debugPrintln("mavlink_test_request_one_param_from_ap() - START");

    mavlink_msg_param_request_read_pack(FMX_SYS_ID, FMX_COMP_ID, &msg, AP_SYS_ID, AP_COMP_ID, param_i_want, -1);

    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.

    debugPrintln("mavlink_test_request_one_param_from_ap() - END");

} // END - mavlink_test_request_one_param_from_ap()

/*============================
 * mavlink_test_set_one_param_on_ap()
 *
 * A basic test of setting one param on the AP using the "PARAM_SET" #23 msg method. https://mavlink.io/en/messages/common.html#PARAM_SET
 *
 * The parameter we want test set, is hard coded in this function as "BATT_ARM_VOLT".
 * See mavlink_set_one_param_on_ap() for a function that lets you request any param.
 * 
 *============================*/
void mavlink_test_set_one_param_on_ap()
{
    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint16_t len;

    const char param_i_want_to_set[16] = "BATT_ARM_VOLT";
    float param_value = 10.5; // as in xx.x volts

    debugPrintln("mavlink_test_set_one_param_on_ap() - START");

    mavlink_msg_param_set_pack(FMX_SYS_ID, FMX_COMP_ID, &msg, AP_SYS_ID, AP_COMP_ID, param_i_want_to_set, param_value, MAV_PARAM_TYPE_REAL32);

    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.

    debugPrintln("mavlink_test_set_one_param_on_ap() - END");

} // END - mavlink_test_set_one_param_on_ap()

/*============================
 * mavlink_set_one_param_on_ap()
 *
 * Set one param on the AP using the "PARAM_SET" #23 msg method. https://mavlink.io/en/messages/common.html#PARAM_SET
 *
 *============================*/
bool mavlink_set_one_param_on_ap(char *name, int32_t value_int, float value_float, uint8_t valuetype)
{
    bool result = false; // unless we successfully execute the SET, result will be false.

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint16_t len;

    char param_i_want_to_set[16] = {}; // "BATT_ARM_VOLT";

    // check valuetype is valid AND execute appropriate the mavlink command (INT or FLOAT)
    if ((valuetype >= MAV_PARAM_TYPE_UINT8) && (valuetype <= MAV_PARAM_TYPE_INT32)) // i.e. an INT
    {
        debugPrintln("set_one_param_from_ap() - Starting INT version");
        memcpy(param_i_want_to_set, name, sizeof(param_i_want_to_set));

        mavlink_msg_param_set_pack(FMX_SYS_ID, FMX_COMP_ID, &msg, AP_SYS_ID, AP_COMP_ID, param_i_want_to_set, value_int, valuetype);
        debugPrint("set_one_param_from_ap() - param_i_want_to_set:");
        Serial.println(param_i_want_to_set);
        len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
        Serial1.write(buf, len);                     // Write data to serial port byte by byte.

        result = true;
        debugPrintln("set_one_param_from_ap() - END");
    }

    if (valuetype == MAV_PARAM_TYPE_REAL32) // i.e. a FLOAT
    {
    }

    return (result);

} // END - mavlink_set_one_param_on_ap()

/*============================
 * mavlink_set_arm_ap()
 *
 * A basic test of Arming the AP using the MAV_CMD_COMPONENT_ARM_DISARM within COMMAND_LONG
 * As discussed in ArduPilot doco - https://ardupilot.org/dev/docs/mavlink-arming-and-disarming.html#arming-and-disarming
 *
 * Note: ArduPilot Pre-arm safety checks may impact this working - https://ardupilot.org/rover/docs/common-prearm-safety-checks.html
 *============================*/
void mavlink_set_arm_ap()
{
    debugPrintln("mavlink_set_arm_ap() - START");

    // Prep source and dest MAVLink addressing info, to be used in below actions.
    uint8_t _system_id = FMX_SYS_ID;        // MAVLink System ID of this device.
    uint8_t _component_id = FMX_COMP_ID;    // MAVLink Component ID of this device.
    uint8_t _target_system = AP_SYS_ID;     // MAVLink System ID of the autopilot.
    uint8_t _target_component = AP_COMP_ID; // MAVLink Component ID of the autopilot.

    // Build the COMMAND_LONG / MAV_CMD_COMPONENT_ARM_DISARM message.
    // components of the MAVLink COMMAND_LONG message - https://mavlink.io/en/messages/common.html#COMMAND_LONG
    uint16_t _cl_command = MAV_CMD_COMPONENT_ARM_DISARM; // https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
    uint8_t _cl_confirmation = 0;                        // always 0 for first transmission, then incremented. https://mavlink.io/en/services/command.html#COMMAND_LONG
    float _cl_param1 = 1;                                // Arm - 0=disarm, 1=arm
    float _cl_param2 = 21196;                            // Force - 0: arm-disarm unless prevented by safety checks (i.e. when landed), 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)
    float _cl_param3 = 0;                                // Not used, so set to zero.
    float _cl_param4 = 0;                                // Not used, so set to zero.
    float _cl_param5 = 0;                                // Not used, so set to zero.
    float _cl_param6 = 0;                                // Not used, so set to zero.
    float _cl_param7 = 0;                                // Not used, so set to zero.

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack and send the message
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                              // Write data to serial port byte by byte.

    // xxx - nowhere did I confirm this worked...I should request state of the AP and confirm success or failure and return that to callee.
    //          or should I have a standalone function get_ap_arm_status and use that up in the callee of this function.
    //          or is it in HEARTBEATs?

    debugPrintln("mavlink_set_arm_ap() - END");

} // END - mavlink_set_arm_ap()

/*============================
 * mavlink_set_disarm_ap()
 *
 * A basic test of DISarming the AP using the MAV_CMD_COMPONENT_ARM_DISARM within COMMAND_LONG
 * As discussed in ArduPilot doco - https://ardupilot.org/dev/docs/mavlink-arming-and-disarming.html#arming-and-disarming
 *
 *============================*/
void mavlink_set_disarm_ap()
{
    debugPrintln("mavlink_set_disarm_ap() - START");

    // Prep source and dest MAVLink addressing info, to be used in below actions.
    uint8_t _system_id = FMX_SYS_ID;        // MAVLink System ID of this device.
    uint8_t _component_id = FMX_COMP_ID;    // MAVLink Component ID of this device.
    uint8_t _target_system = AP_SYS_ID;     // MAVLink System ID of the autopilot.
    uint8_t _target_component = AP_COMP_ID; // MAVLink Component ID of the autopilot.

    // Build the COMMAND_LONG / MAV_CMD_COMPONENT_ARM_DISARM message.
    // components of the MAVLink COMMAND_LONG message - https://mavlink.io/en/messages/common.html#COMMAND_LONG
    uint16_t _cl_command = MAV_CMD_COMPONENT_ARM_DISARM; // https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
    uint8_t _cl_confirmation = 0;                        // always 0 for first transmission, then incremented. https://mavlink.io/en/services/command.html#COMMAND_LONG
    float _cl_param1 = 0;                                // Arm - 0=disarm, 1=arm
    float _cl_param2 = 21196;                            // Force - 0: arm-disarm unless prevented by safety checks (i.e. when landed), 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)
    float _cl_param3 = 0;                                // Not used, so set to zero.
    float _cl_param4 = 0;                                // Not used, so set to zero.
    float _cl_param5 = 0;                                // Not used, so set to zero.
    float _cl_param6 = 0;                                // Not used, so set to zero.
    float _cl_param7 = 0;                                // Not used, so set to zero.

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack and send the message
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                              // Write data to serial port byte by byte.

    debugPrintln("mavlink_set_disarm_ap() - END");

} // END - mavlink_set_disarm_ap()

/*============================
 * mavlink_set_flightmode_ap()
 *
 * Change the ArduPilot Flightmode of the AP.
 * We issue a COMMAND_LONG containing the command MAV_CMD_DO_SET_MODE (#176) https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE
 * As discussed in ArduPilot doco - https://ardupilot.org/dev/docs/mavlink-get-set-flightmode.html
 *
 * The Flightmodes are defined in ardupilotmega.h from the MAVLink library. https://github.com/mavlink/c_library_v1/blob/079d909ee331205d47652b208057d265338dc839/ardupilotmega/ardupilotmega.h#L906
 *   ROVER_MODE_MANUAL=0
 *   ROVER_MODE_ACRO=1
 *   ROVER_MODE_STEERING=3
 *   ROVER_MODE_HOLD=4
 *   ROVER_MODE_LOITER=5
 *   ROVER_MODE_FOLLOW=6
 *   ROVER_MODE_SIMPLE=7
 *   ROVER_MODE_AUTO=10
 *   ROVER_MODE_RTL=11
 *   ROVER_MODE_SMART_RTL=12
 *   ROVER_MODE_GUIDED=15
 *   ROVER_MODE_INITIALIZING=16
 *   ROVER_MODE_ENUM_END=17
 *============================*/
void mavlink_set_flightmode_ap(uint8_t desired_flightmode)
{
    debugPrintln("mavlink_set_flightmode_ap() - START");

    Serial.print("mavlink_set_flightmode_ap() - Desired Flightmode:");
    Serial.println(desired_flightmode);

    // Prep source and dest MAVLink addressing info, to be used in below actions.
    uint8_t _system_id = FMX_SYS_ID;        // MAVLink System ID of this device.
    uint8_t _component_id = FMX_COMP_ID;    // MAVLink Component ID of this device.
    uint8_t _target_system = AP_SYS_ID;     // MAVLink System ID of the autopilot.
    uint8_t _target_component = AP_COMP_ID; // MAVLink Component ID of the autopilot.

    // Build the COMMAND_LONG / MAV_CMD_DO_SET_MODE message.
    // components of the MAVLink COMMAND_LONG message - https://mavlink.io/en/messages/common.html#COMMAND_LONG
    uint16_t _cl_command = MAV_CMD_DO_SET_MODE; // https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE
    uint8_t _cl_confirmation = 0;               // always 0 for first transmission, then incremented. https://mavlink.io/en/services/command.html#COMMAND_LONG
    float _cl_param1 = 1;                       // for ArduPilot, always MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1   (according to https://ardupilot.org/dev/docs/mavlink-get-set-flightmode.html#set-the-flightmode-with-mav-cmd-do-set-mode)
    float _cl_param2 = (float)desired_flightmode;      // Flightmode                        (according to https://ardupilot.org/dev/docs/mavlink-get-set-flightmode.html#set-the-flightmode-with-mav-cmd-do-set-mode)
                                                // and see here for mode names and numbers for ROVER https://github.com/ardupilot/ardupilot/blob/master/Rover/mode.h#L21
    float _cl_param3 = 0;                       // Not used, so set to zero.
    float _cl_param4 = 0;                       // Not used, so set to zero.
    float _cl_param5 = 0;                       // Not used, so set to zero.
    float _cl_param6 = 0;                       // Not used, so set to zero.
    float _cl_param7 = 0;                       // Not used, so set to zero.

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack and send the message
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                              // Write data to serial port byte by byte.

    debugPrintln("mavlink_set_flightmode_ap() - END");

} // END - mavlink_set_flightmode_ap()


/*============================
 * mavlink_cmd_preflight_reboot_shutdown_ap()
 *
 * Change the ArduPilot Flightmode of the AP.
 * We issue a COMMAND_LONG containing the command MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (246) https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
 *
 *============================*/
void mavlink_cmd_preflight_reboot_ap()
{
    debugPrintln("mavlink_cmd_preflight_reboot_ap() - START");

    // Prep source and dest MAVLink addressing info, to be used in below actions.
    uint8_t _system_id = FMX_SYS_ID;        // MAVLink System ID of this device.
    uint8_t _component_id = FMX_COMP_ID;    // MAVLink Component ID of this device.
    uint8_t _target_system = AP_SYS_ID;     // MAVLink System ID of the autopilot.
    uint8_t _target_component = AP_COMP_ID; // MAVLink Component ID of the autopilot.

    // Build the COMMAND_LONG / MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN message.
    // components of the MAVLink COMMAND_LONG message - https://mavlink.io/en/messages/common.html#COMMAND_LONG
    uint16_t _cl_command = MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN; // https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    uint8_t _cl_confirmation = 0;               // always 0 for first transmission, then incremented. https://mavlink.io/en/services/command.html#COMMAND_LONG
    float _cl_param1 = 1;                       // 0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.
    float _cl_param2 = 0;                       // Not used, so set to zero.
    float _cl_param3 = 0;                       // Not used, so set to zero.
    float _cl_param4 = 0;                       // Not used, so set to zero.
    float _cl_param5 = 0;                       // Not used, so set to zero.
    float _cl_param6 = 0;                       // Not used, so set to zero.
    float _cl_param7 = 0;                       // Not used, so set to zero.

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack and send the message
    mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, _cl_command, _cl_confirmation, _cl_param1, _cl_param2, _cl_param3, _cl_param4, _cl_param5, _cl_param6, _cl_param7);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                              // Write data to serial port byte by byte.

    debugPrintln("mavlink_cmd_preflight_reboot_ap() - END");

} // END - mavlink_cmd_preflight_reboot_ap()

