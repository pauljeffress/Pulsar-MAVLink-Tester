/*
 * mavlink_param_set_fns.cpp
 *
 *
 */

#include "mavlink_fns.h"

#include "debug_fns.h"
#include "timer_fns.h"

#ifdef FMX
    #include "FmxSettings_fns.h"
#endif

// Define any globals that only the functions in this file need.

// Define functions.


/*============================
 * mavlink_set_one_param_on_ap()
 *
 * Set one param on the AP using the "PARAM_SET" #23 msg method. https://mavlink.io/en/messages/common.html#PARAM_SET
 *
 *============================*/
bool mavlink_set_one_param_on_ap(char *name, int32_t value_int, float value_float, uint8_t valuetype)
{
    debugPrintln("mavlink_set_one_param_on_ap() - Starting");
    
    bool result = false; // unless we successfully execute the SET, result will be false.

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint16_t len;

    char param_i_want_to_set[16] = {}; // e.g. "BATT_ARM_VOLT";

    // check valuetype is valid AND execute appropriate mavlink command (INT or FLOAT)
    if ((valuetype >= MAV_PARAM_TYPE_UINT8) && (valuetype <= MAV_PARAM_TYPE_INT32)) // i.e. an INT
    {
        debugPrintln("mavlink_set_one_param_on_ap() - Starting INT");
        memcpy(param_i_want_to_set, name, sizeof(param_i_want_to_set));

        mavlink_msg_param_set_pack(FMX_SYS_ID, FMX_COMP_ID, &msg, AP_SYS_ID, AP_COMP_ID, param_i_want_to_set, value_int, valuetype);
        debugPrint("mavlink_set_one_param_on_ap() - param_i_want_to_set:");
        Serial.println(param_i_want_to_set);
        len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
        Serial1.write(buf, len);                     // Write data to serial port byte by byte.

        result = true;  // indicate that we did send the MAVLink PARAM_SET msg
        debugPrintln("mavlink_set_one_param_on_ap() - Complete INT");
    }

    if (valuetype == MAV_PARAM_TYPE_REAL32) // i.e. a FLOAT
    {
        debugPrintln("mavlink_set_one_param_on_ap() - Starting FLOAT");
        memcpy(param_i_want_to_set, name, sizeof(param_i_want_to_set));

        mavlink_msg_param_set_pack(FMX_SYS_ID, FMX_COMP_ID, &msg, AP_SYS_ID, AP_COMP_ID, param_i_want_to_set, value_float, valuetype);
        debugPrint("mavlink_set_one_param_on_ap() - param_i_want_to_set:");
        Serial.println(param_i_want_to_set);
        len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
        Serial1.write(buf, len);                     // Write data to serial port byte by byte.

        result = true;  // indicate that we did send the MAVLink PARAM_SET msg
        debugPrintln("mavlink_set_one_param_on_ap() - Complete FLOAT");
    }
    
    debugPrint("mavlink_set_one_param_on_ap() - Complete. Result=");
    if (result) debugPrintln("TRUE");
    else  debugPrintln("FALSE");
    
    return (result);

} // END - mavlink_set_one_param_on_ap()


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
 