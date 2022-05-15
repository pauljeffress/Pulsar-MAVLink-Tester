/*
 * mavlink_param_get_fns.cpp
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
 * mavlink_get_one_param_from_ap()
 *
 * Get one param from the AP using the "PARAM_REQUEST_READ" #20 msg method. https://mavlink.io/en/messages/common.html#PARAM_REQUEST_READ
 * to place the request and then look for the subsequent response in the inbound MAVLink msgs from the AP.
 * 
 * INPUTS
 *  name        - REQUIRED - the param_id/name of the param we want to get e.g "BATT_ARM_VOLT"
 *  value_int   - see OUTPUT below
 *  value_float - see OUTPUT below
 *  valuetype   - is the value we are requesting an INT or a FLOAT? https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE
 * 
 * OUTPUT
 *  Either value_int OR value_float will have been set if we were successful, depending on valuetype,
 *  AND always sets the other to 0 (just for tidiness)
 * 
 * RETURNS
 *  TRUE    - if we got the param's value successfully from the AP
 *  FALSE   - if not
 *============================*/
bool mavlink_get_one_param_from_ap(char *name, int32_t *value_int, float *value_float, uint8_t valuetype)
{
    debugPrintln("mavlink_get_one_param_from_ap() - Starting");

    debugPrint("mavlink_get_one_param_from_ap() - name:");Serial.println(name);
    debugPrint("mavlink_get_one_param_from_ap() - value_int:");Serial.println(*value_int);
    debugPrint("mavlink_get_one_param_from_ap() - value_float:");Serial.println(*value_float);
    debugPrint("mavlink_get_one_param_from_ap() - valuetype:");Serial.println(valuetype);
    
    bool result = false; // unless we successfully execute the GET, result will be false.

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint16_t len;

    char param_i_want_to_get[16] = {}; // e.g. "BATT_ARM_VOLT";

    // ask the AP to tell us the value of the param we want. Assemble and send a PARAM_REQUEST_READ #20 msg. https://mavlink.io/en/messages/common.html#PARAM_REQUEST_READ
    memcpy(param_i_want_to_get, name, sizeof(param_i_want_to_get));
    mavlink_msg_param_request_read_pack(FMX_SYS_ID, FMX_COMP_ID, &msg, AP_SYS_ID, AP_COMP_ID, param_i_want_to_get, -1);
    debugPrint("mavlink_get_one_param_from_ap() - We have requested param_i_want_to_get:");
    Serial.println(param_i_want_to_get);
    len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.
    Serial1.write(buf, len);                     // Write data to serial port byte by byte.

    // look for a response from the AP
    if ((valuetype >= MAV_PARAM_TYPE_UINT8) && (valuetype <= MAV_PARAM_TYPE_INT32)) // check valuetype is valid AND execute appropriate mavlink command (INT or FLOAT)
    {
        debugPrintln("mavlink_get_one_param_from_ap() - Expecting INT response");
        
        *value_float = 0.0;  // as we are looking for INT, just set the unused float to zero.
        
        // look for the desired response in an inbound PARAM_VALUE (#22) MAVLink msg that contains param_id/name that we want, from the AP.
        result = mavlink_receive_param_value(name, value_int, value_float, valuetype); // should it be value_int or */&value_int???
        if (result)
        {
            debugPrintln("mavlink_get_one_param_from_ap() - We heard the param value INT we wanted.");
            debugPrint("mavlink_get_one_param_from_ap() - It is :");debugPrintlnInt(*value_int);
        }
        else
            debugPrintln("mavlink_get_one_param_from_ap() - We DID NOT hear the param value INT we wanted.");

        debugPrintln("mavlink_get_one_param_from_ap() - Complete INT");
    }

    if (valuetype == MAV_PARAM_TYPE_REAL32) // i.e. a FLOAT
    {
        debugPrintln("mavlink_get_one_param_from_ap() - Expecting FLOAT response");
        
        *value_int = 0;  // as we are looking for FLOAT, just set the unused int to zero.
        
        // look for the desired response in an inbound PARAM_VALUE (#22) MAVLink msg that contains param_id/name that we want, from the AP.
        result = mavlink_receive_param_value(name, value_int, value_float, valuetype); // should it be value_int or */&value_int???
        if (result)
        {
            debugPrintln("mavlink_get_one_param_from_ap() - We heard the param value FLOAT we wanted.");
            debugPrint("mavlink_get_one_param_from_ap() - It is :");debugPrintlnInt(*value_float);
        }
        else
            debugPrintln("mavlink_get_one_param_from_ap() - We DID NOT hear the param value FLOAT we wanted.");

        debugPrintln("mavlink_get_one_param_from_ap() - Complete FLOAT");
    }

    debugPrint("mavlink_get_one_param_from_ap() - Complete. Result=");
    if (result) debugPrintln("TRUE");
    else  debugPrintln("FALSE");
    
    return (result);

} // END - mavlink_get_one_param_from_ap()



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