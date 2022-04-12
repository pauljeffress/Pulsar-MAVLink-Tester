/*
 * mavlink_mission_fns.cpp
 *
 * mavlink specific functions - all the ones related to MISSIONs
 *
 *
 */

#include "mavlink_fns.h"

#include "debug_fns.h"

/*
 * I was able to remove the following hack by simply adding
 *
 * build_flags =
 *   -DSerial1=Serial
 *
 * to the platformio.ini for the AGT project.
 *
 */
// #ifndef Serial1
//     #define Serial1 Serial      // This is a complete hack because this file gets compiled no matter what I do, when I compile the AGT code.
//                             // And as the AGT does not have a Serial1 it fails to compile...even though we don't even call this function
//                             // from the AGT.
//                             // So to sort of fake it for the compiler I am just defining Serial1 to be Serial if Serial1 does not already exist.
//                             // On a Feather that has Serial1, my #define should be skipped.
// #endif

// Define any globals that only the functions in this file need.

GlobalMission_t global_mission;              // structure to hold a MAVLink mission for transfer to/from the AP.
GlobalMissionSubset_t global_mission_subset; // structure to hold a subset of MAVLink mission for transfer between AGT & FMX.

// Define functions.

/*============================
 * mission_clear_all_ap()
 *
 * A basic test of telling the AP to delete all mission items.
 * To do this we are using the MAVLink Message MISSION_CLEAR_ALL (#45) https://mavlink.io/en/messages/common.html#MISSION_CLEAR_ALL
 * As this is a stand alone message, and does need to be packed into a COMMAND_LONG
 * its very simple to build and send.
 *
 * Mavlink.io documentation
 *  - in general here - https://mavlink.io/en/services/mission.html#clear_mission
 *  - the specific MAV MSG is - MISSION_CLEAR_ALL ( #45 ) https://mavlink.io/en/messages/common.html#MISSION_CLEAR_ALL
 *
 * ArduPilot documentation
 *  - https://ardupilot.org/dev/docs/mavlink-mission-upload-download.html
 *
 * Returns: PulsarMavMissionResult_t
 *============================*/
PulsarMavMissionResult_t mission_clear_all_ap()
{
    debugPrintln("mission_clear_all_ap() - START");

    PulsarMavMissionResult_t pmmr; // local struct to hold results ready for return to callee.
    pmmr.pr = NO_ERROR;
    pmmr.mmr = MAV_MISSION_ERROR;

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_mission_clear_all_pack(FMX_SYS_ID, FMX_COMP_ID, &msg, AP_SYS_ID, AP_COMP_ID);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.

    uint8_t counter = 1; // Count the amount of times we try to do the below MAVLink process

    // Make a set number of attempts to send it to the AP and get a response from AP
    while ((pmmr.mmr != MAV_MISSION_ACCEPTED) && (counter <= FMX_MAVLINK_MAX_TRYS))
    {
        Serial1.write(buf, len); // Write data to serial port byte by byte.
        debugPrint("mission_clear_all_ap() - MISSION_CLEAR_ALL (#45) sent to AP - try #");
        debugPrintlnInt(counter);

        // Check if the AP responded with a MISSION_ACK (#47) - https://mavlink.io/en/messages/common.html#MISSION_ACK
        pmmr = mavlink_get_MISSION_ACK();

        // Now check pmmr.pr and pmmr.mmr to see how we went and determine next steps
        if (pmmr.pr == TIMEOUT)
        {
            // looks like we timed out on prev attempt, the while() will try again if we have any more trys left.
            debugPrintln("mission_clear_all_ap() - TIMED OUT waiting for MISSION ACK");
            counter++; // increment counter
        }
        else if ((pmmr.pr == SUCCESS) && (pmmr.mmr == MAV_MISSION_ACCEPTED))
        {
            // looks like we got the packet we were looking for and the result within it said our original message was ACCEPTED
            // this is a great outcome.  No need to make another try.
            debugPrintln("mission_clear_all_ap() - SUCCESS - Received MISSION_ACK containing MAV_MISSION_ACCEPTED");
        }
        else if ((pmmr.pr == SUCCESS) && (pmmr.mmr != MAV_MISSION_ACCEPTED))
        {
            // Looks like we got the packet we were looking for BUT the AP had a problem with the request.
            // The while() will try again if we have any more trys left.
            debugPrint("mission_clear_all_ap() - Received MISSION_ACK containing error MAV_MISSION_RESULT=");
            debugPrintlnInt((uint8_t)pmmr.mmr);
            counter++; // increment counter
        }
    } // END - while()

    if (counter > FMX_MAVLINK_MAX_TRYS)
        debugPrintln("mission_clear_all_ap() - Maximum trys exceeded...lets move on.");

    debugPrint("mavlink_get_MISSION_ACK() - pmmr.pr:");
    Serial.println(pmmr.pr);
    debugPrint("mavlink_get_MISSION_ACK() - pmmr.mmr:");
    Serial.println(pmmr.mmr);

    debugPrintln("mission_clear_all_ap() - END");

    return pmmr;

} // END - mission_clear_all_ap()

/*============================
 * mavlink_get_MISSION_ACK()
 *
 * Listen to MAVLink data, for a fixed period of time at most, and look only for a
 * MISSION_ACK (#47) - https://mavlink.io/en/messages/common.html#MISSION_ACK
 *
 * Returns: PulsarMavMissionResult_t
 *============================*/
PulsarMavMissionResult_t mavlink_get_MISSION_ACK()
{
    debugPrint("mavlink_get_MISSION_ACK() - starting at Millis:");
    Serial.println(millis());

    PulsarMavMissionResult_t pmmr; // local struct to hold results ready for return to callee.
    pmmr.pr = NOT_SET;
    pmmr.mmr = MAV_MISSION_ACCEPTED;
    pmmr.u32value = 0;

    mavlink_message_t msg;
    mavlink_status_t status;
    bool gotFullMsg = false;
    bool we_got_expected_packet = false;
    boolean we_timed_out = false; // Flag to track if we timed out when waiting for the MISSION_ACK.

    // Explanation of the following while() loop
    // I need to ensure that the mavlink_receive() function gets a solid couple of contiguous
    // seconds to ensure it gets a good chance to snap up lots of mavlink data and form packets.
    // Also, as I am concerned about this while loop playing up when millis() rolls over to zero,
    // I am using the extra "&& (millis() > FMX_MAVLINK_RX_WINDOW_MS)" in the while() to jump that bit close to 0.

    uint32_t start = millis(); // xxx - need to review how I'm timing this loop...seems clunky. Also need to take any constants and set the as #defines.

    while ((!we_timed_out) && (!we_got_expected_packet)) // Keep looping until either
                                                         //      we timed out OR
                                                         //      we got the packet we were looking for.
    {
        // debugPrint("O");
        //  grab any available chars from the MAVLink interface,
        //  add them to what we have so far,
        //  and see if we have a full mavlink message
        while (Serial1.available() && !gotFullMsg) // xxx - I should prob put a time limiter on this WHILE, I think the only reason
                                                   // it is not hogging the CPU is because the AutoPilot cube only sends msgs each
                                                   // second, and then pauses I think, is why the WHILE breaks out.
        {
            // debugPrint("i");
            uint8_t c = Serial1.read();
            // debugPrintln("got char");
            //  add new char to what we have so far and see of we have a full Mavlink msg yet
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) // if we do then bust out of this char collecting loop.
                gotFullMsg = true;
        }

        // At this point we either have a Full Msg OR there were no more chars available on the MAVLink serial link.

        if (gotFullMsg) // if we do then lets process it
        {
            // debugPrint("F");
            //  Check if the new full message is the one we are looking for
            if (msg.msgid == MAVLINK_MSG_ID_MISSION_ACK) // #47  https://mavlink.io/en/messages/common.html#MISSION_ACK
            {
                debugPrint("M");
                // Decode the message into a packet we can work with
                mavlink_mission_ack_t packet;
                mavlink_msg_mission_ack_decode(&msg, &packet);

#ifdef MAVLINK_DEBUG
                debugPrint("mavlink_get_MISSION_ACK() - MSG RCVD -");
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

#ifdef MAVLINK_DEBUG
                debugPrint("=MISSION_ACK");
                debugPrint(" target_system:");
                Serial.print(packet.target_system);
                debugPrint(" target_component:");
                Serial.print(packet.target_component);
                debugPrint(" type(MAV_MISSION_RESULT):");
                Serial.println(packet.type);
#endif

                // Action or save things I'm interested in to FeatherMx data structure for use later.
                pmmr.mmr = (MAV_MISSION_RESULT)packet.type; // copy out the MAV_MISSION_RESULT (https://mavlink.io/en/messages/common.html#MAV_MISSION_RESULT)
                                                            // from the MISSION_ACK message

                // flag this for later in this function.
                we_got_expected_packet = true;

            }    // END - if (msg.msgid == MAVLINK_MSG_ID_MISSION_ACK)
            else // we got a full packet, but it was not the specific one we were looking for.
            {
                // flag this for later in this function.
                we_got_expected_packet = false;
#ifdef MAVLINK_DEBUG
                debugPrintln(" "); // ensures earlier debug prints are CR/LF'd if the packet was not the one we wanted.
#endif
            }
            gotFullMsg = false;
        } // END - if (gotFullMsg)

        // check timer and set flag accordingly
        if (millis() > (start + (FMX_MAVLINK_RX_WINDOW_LONG_MS - 1000)) && (millis() > FMX_MAVLINK_RX_WINDOW_LONG_MS)) // have we timed out yet?
            we_timed_out = true;

    } // END - while((!we_timed_out) && ((.....

    if (we_timed_out)
        pmmr.pr = TIMEOUT;
    else
        pmmr.pr = SUCCESS;

    // To get to here we either timed out or got the message we wanted.
    // Either way, pmmr will contain the appropriate results/status, ready for us to
    // return to our callee

    debugPrint("mavlink_get_MISSION_ACK() - ending at Millis:");
    Serial.println(millis());

    debugPrint("mavlink_get_MISSION_ACK() - pmmr.pr:");
    Serial.println(pmmr.pr);
    debugPrint("mavlink_get_MISSION_ACK() - pmmr.mmr(UNUSED):");
    Serial.println(pmmr.mmr);
    debugPrint("mavlink_get_MISSION_ACK() - pmmr.u32value:");
    Serial.println(pmmr.u32value);

    return pmmr;

} // END - mavlink_get_MISSION_ACK()

/*============================
 * mavlink_get_MISSION_REQUEST_INT()
 *
 * Listen to MAVLink data, for a fixed period of time at most, and look for EITHER a
 * MISSION_REQUEST_INT (#51) - https://mavlink.io/en/messages/common.html#MISSION_REQUEST_INT
 * or a MISSION_REQUEST ( #40 ) - https://mavlink.io/en/messages/common.html#MISSION_REQUEST
 *
 * I am looking for both as #51 has replaced #40 (deprecated), but ArduPilot still seems to be
 * sending #40's, so this way my code will be more robust and handle both forms.
 *
 * Returns: PulsarMavMissionResult_t
 *============================*/
PulsarMavMissionResult_t mavlink_get_MISSION_REQUEST_INT()
{
    debugPrint("mavlink_get_MISSION_REQUEST_INT() - starting at Millis:");
    Serial.println(millis());

    PulsarMavMissionResult_t pmmr; // local struct to hold results ready for return to callee.
    pmmr.pr = NOT_SET;
    pmmr.mmr = MAV_MISSION_ACCEPTED;
    pmmr.u32value = 0;

    mavlink_message_t msg;
    mavlink_status_t status;
    bool gotFullMsg = false;
    bool we_got_expected_packet = false;
    boolean we_timed_out = false; // Flag to track if we timed out when waiting for the MISSION_ACK.

    // Explanation of the following while() loop
    // I need to ensure that the mavlink_receive() function gets a solid couple of contiguous
    // seconds to ensure it gets a good chance to snap up lots of mavlink data and form packets.
    // Also, as I am concerned about this while loop playing up when millis() rolls over to zero,
    // I am using the extra "&& (millis() > FMX_MAVLINK_RX_WINDOW_MS)" in the while() to jump that bit close to 0.

    uint32_t start = millis(); // xxx - need to review how I'm timing this loop...seems clunky. Also need to take any constants and set the as #defines.

    while ((!we_timed_out) && (!we_got_expected_packet)) // Keep looping until either
                                                         //      we timed out OR
                                                         //      we got the packet we were looking for.
    {
        // debugPrint("O");
        //  grab any available chars from the MAVLink interface,
        //  add them to what we have so far,
        //  and see if we have a full mavlink message
        while (Serial1.available() && !gotFullMsg) // xxx - I should prob put a time limiter on this WHILE, I think the only reason
                                                   // it is not hogging the CPU is because the AutoPilot cube only sends msgs each
                                                   // second, and then pauses I think, is why the WHILE breaks out.
        {
            // debugPrint("i");
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
            debugPrint("mavlink_get_MISSION_REQUEST_INT() - MSG RCVD -");
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

            // debugPrint("F");
            //  Check if the new full message is the one of the ones we are looking for
            if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT) // #51 https://mavlink.io/en/messages/common.html#MISSION_REQUEST_INT
            {
                // debugPrint("M");
                //  Decode the message into a packet we can work with
                mavlink_mission_request_int_t packet;
                mavlink_msg_mission_request_int_decode(&msg, &packet);

#ifdef MAVLINK_DEBUG
                debugPrint("=MISSION_REQUEST_INT");
                debugPrint(" target_system:");
                Serial.print(packet.target_system);
                debugPrint(" target_component:");
                Serial.print(packet.target_component);
                debugPrint(" seq:");
                Serial.println(packet.seq);
#endif

                // Action or save things I'm interested in to FeatherMx data structure for use later.
                pmmr.u32value = (uint32_t)packet.seq; // copy out the Sequence number
                                                      // from the MISSION_REQUEST_INT message

                // flag this for later in this function.
                we_got_expected_packet = true;
            }
            else if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST) // #40 https://mavlink.io/en/messages/common.html#MISSION_REQUEST
            {
                // debugPrint("M");
                //  Decode the message into a packet we can work with
                mavlink_mission_request_t packet;
                mavlink_msg_mission_request_decode(&msg, &packet);

#ifdef MAVLINK_DEBUG
                debugPrint("=MISSION_REQUEST");
                debugPrint(" target_system:");
                Serial.print(packet.target_system);
                debugPrint(" target_component:");
                Serial.print(packet.target_component);
                debugPrint(" seq:");
                Serial.println(packet.seq);
#endif

                // Action or save things I'm interested in to FeatherMx data structure for use later.
                pmmr.u32value = (uint32_t)packet.seq; // copy out the Sequence number
                                                      // from the MISSION_REQUEST_INT message

                // flag this for later in this function.
                we_got_expected_packet = true;
            }
            else if (msg.msgid == MAVLINK_MSG_ID_MISSION_ACK) // #47  https://mavlink.io/en/messages/common.html#MISSION_ACK
            {
                // Decode the message into a packet we can work with
                mavlink_mission_ack_t packet;
                mavlink_msg_mission_ack_decode(&msg, &packet);

#ifdef MAVLINK_DEBUG
                debugPrint("=MISSION_ACK");
                debugPrint(" target_system:");
                Serial.print(packet.target_system);
                debugPrint(" target_component:");
                Serial.print(packet.target_component);
                debugPrint(" type(MAV_MISSION_RESULT):");
                Serial.println(packet.type);
#endif

                // Action or save things I'm interested in to FeatherMx data structure for use later.
                pmmr.mmr = (MAV_MISSION_RESULT)packet.type; // copy out the MAV_MISSION_RESULT (https://mavlink.io/en/messages/common.html#MAV_MISSION_RESULT)
                                                            // from the MISSION_ACK message

                pmmr.pr = FAILED; // xxx - this may need to be check?
                debugPrint("mavlink_get_MISSION_REQUEST_INT() - !!!! MISSION_ACK RCVD -");
                debugPrint(" mmr:");
                debugPrintlnInt(packet.type);
            }
            else // we got a full packet, but it was not the specific one we were looking for.
            {
                // flag this for later in this function.
                we_got_expected_packet = false;
#ifdef MAVLINK_DEBUG
                debugPrintln(" "); // ensures earlier debug prints are CR/LF'd if the packet was not the one we wanted.
#endif
            }
            gotFullMsg = false;
        } // END - if (gotFullMsg)

        // check timer and set flag accordingly
        if (millis() > (start + (FMX_MAVLINK_RX_WINDOW_LONG_MS - 1000)) && (millis() > FMX_MAVLINK_RX_WINDOW_LONG_MS)) // have we timed out yet?
            we_timed_out = true;

    } // END - while((!we_timed_out) && ((.....

    if (we_timed_out)
        pmmr.pr = TIMEOUT;
    else
        pmmr.pr = SUCCESS;

    // To get to here we either timed out or got the message we wanted.
    // Either way, pmmr will contain the appropriate results/status, ready for us to
    // return to our callee

    debugPrint("mavlink_get_MISSION_REQUEST_INT() - ending at Millis:");
    Serial.println(millis());

    debugPrint("mavlink_get_MISSION_REQUEST_INT() - pmmr.pr:");
    Serial.println(pmmr.pr);
    debugPrint("mavlink_get_MISSION_REQUEST_INT() - pmmr.mmr(UNUSED):");
    Serial.println(pmmr.mmr);
    debugPrint("mavlink_get_MISSION_REQUEST_INT() - pmmr.u32value:");
    Serial.println(pmmr.u32value);

    return pmmr;

} // END - mavlink_get_MISSION_REQUEST_INT()

/*============================
 * global_mission_init()
 *
 * "zero out" all values in our global_mission
 *
 *============================*/
void global_mission_init()
{
    debugPrintln("global_mission_init() - START");

    global_mission.count = 0;

    for (int i = 0; i < PULSAR_MISSION_MAX_ITEMS; i++)
    {
        global_mission.pulsar_mission_item[i].sequence_number = PULSAR_MISSION_ITEM_SEQ_PLACEHOLDER; // set to this value so I can test for it as needed.
        global_mission.pulsar_mission_item[i].frame = MAV_FRAME_GLOBAL;                              //  https://mavlink.io/en/messages/common.html#MAV_FRAME
        global_mission.pulsar_mission_item[i].command = MAV_CMD_DO_WINCH;                            //  WINCH is just a placeholder that I never use, so I can test for it as needed.
        global_mission.pulsar_mission_item[i].is_current_item = false;
        global_mission.pulsar_mission_item[i].auto_continue = false;
        global_mission.pulsar_mission_item[i].param1 = 0.0;
        global_mission.pulsar_mission_item[i].param2 = 0.0;
        global_mission.pulsar_mission_item[i].param3 = 0.0;
        global_mission.pulsar_mission_item[i].param4 = 0.0;
        global_mission.pulsar_mission_item[i].x = 0;
        global_mission.pulsar_mission_item[i].y = 0;
        global_mission.pulsar_mission_item[i].z = 0.0;
    }

    global_mission.status = INITIALISED;

    debugPrintln("global_mission_init() - FINISHED");
} // END - global_mission_init()

/*============================
 * global_mission_print()
 *
 * print contents of our global_mission
 *
 *============================*/
void global_mission_print()
{
    debugPrintln("-------------------------------------");
    debugPrintln("global_mission_print() - START");

    debugPrint("global_mission.status=");
    debugPrintlnInt(global_mission.status);
    debugPrint("global_mission.count=");
    debugPrintlnInt(global_mission.count);

    for (int i = 0; i <= global_mission.count; i++)
    {
        debugPrintln(" ");
        debugPrint("array element [");
        debugPrintInt(i);
        debugPrintln("]");
        debugPrintln("----------------------------");
        debugPrint("sequence_number=");
        debugPrintlnInt(global_mission.pulsar_mission_item[i].sequence_number);
        debugPrint("frame=");
        debugPrintlnInt(global_mission.pulsar_mission_item[i].frame);
        debugPrint("command=");
        debugPrintlnInt(global_mission.pulsar_mission_item[i].command);
        debugPrint("is_current_item=");
        debugPrintlnInt(global_mission.pulsar_mission_item[i].is_current_item);
        debugPrint("autocontinue=");
        debugPrintlnInt(global_mission.pulsar_mission_item[i].auto_continue);
        debugPrint("param1=");
        debugPrintlnInt(global_mission.pulsar_mission_item[i].param1);
        debugPrint("param2=");
        debugPrintlnInt(global_mission.pulsar_mission_item[i].param2);
        debugPrint("param3=");
        debugPrintlnInt(global_mission.pulsar_mission_item[i].param3);
        debugPrint("param4=");
        debugPrintlnInt(global_mission.pulsar_mission_item[i].param4);
        debugPrint("x=");
        debugPrintlnInt(global_mission.pulsar_mission_item[i].x);
        debugPrint("y=");
        debugPrintlnInt(global_mission.pulsar_mission_item[i].y);
        debugPrint("z=");
        debugPrintlnInt(global_mission.pulsar_mission_item[i].z);
    }

    debugPrintln("global_mission_print() - FINISHED");
    debugPrintln("-------------------------------------");
} // END - global_mission_print()

/*============================
 * global_mission_subset_print()
 *
 * print contents of our global_mission_subset
 *
 *============================*/
void global_mission_subset_print()
{
    debugPrintln("-------------------------------------");
    debugPrintln("global_mission_subset_print() - START");

    debugPrint("global_mission_subset.status=");
    debugPrintlnInt(global_mission_subset.status);
    debugPrint("global_mission_subset.count=");
    debugPrintlnInt(global_mission_subset.count);

    for (int i = 0; i <= global_mission_subset.count; i++)
    {
        debugPrintln(" ");
        debugPrint("array element [");
        debugPrintInt(i);
        debugPrintln("]");
        debugPrintln("----------------------------");
        debugPrint("sequence_number=");
        debugPrintlnInt(global_mission_subset.pulsar_mission_item_subset[i].sequence_number);
        debugPrint("frame=");
        debugPrintlnInt(global_mission_subset.pulsar_mission_item_subset[i].frame);
        debugPrint("command=");
        debugPrintlnInt(global_mission_subset.pulsar_mission_item_subset[i].command);
        debugPrint("is_current_item=");
        debugPrintlnInt(global_mission_subset.pulsar_mission_item_subset[i].is_current_item);
        debugPrint("autocontinue=");
        debugPrintlnInt(global_mission_subset.pulsar_mission_item_subset[i].auto_continue);

        debugPrint("x=");
        debugPrintlnInt(global_mission_subset.pulsar_mission_item_subset[i].x);
        debugPrint("y=");
        debugPrintlnInt(global_mission_subset.pulsar_mission_item_subset[i].y);
    }

    debugPrintln("global_mission_subset_print() - FINISHED");
    debugPrintln("----------------------------------------");
} // END - global_mission_subset_print()

/*============================
 * global_mission_valid()
 *
 * review our global_mission and check it is a complete and
 * valid mission, ready for upload to the AP for example.
 *
 *============================*/
bool global_mission_valid()
{
    debugPrintln("global_mission_valid() - START");

    // xxx - insert code to check !!!!!
    debugPrintln("global_mission_valid() - I need to write this code!!!!!!");

    debugPrintln("global_mission_valid() - FINISHED");

    return true;
} // END - global_mission_init()

/*============================
 * global_mission_dummy_load()
 *
 * put my test values in our global_mission struct purely for testing.
 *
 *============================*/
void global_mission_dummy_load()
{
    debugPrintln("global_mission_dummy_load() - START");

    global_mission_init(); // zero it out before we load in test data.

    // populate dummy item0
    global_mission.pulsar_mission_item[0].sequence_number = 0;
    global_mission.pulsar_mission_item[0].frame = MAV_FRAME_GLOBAL;       //  always this value, never change.
    global_mission.pulsar_mission_item[0].command = MAV_CMD_NAV_WAYPOINT; //  MAV_CMD_NAV_WAYPOINT (16 ) - https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
    global_mission.pulsar_mission_item[0].is_current_item = false;
    global_mission.pulsar_mission_item[0].auto_continue = true;
    global_mission.pulsar_mission_item[0].param1 = 0.0;   // Hold - not used with Rover
    global_mission.pulsar_mission_item[0].param2 = 0.0;   // Accept Radius - not used with Rover
    global_mission.pulsar_mission_item[0].param3 = 0.0;   // Pass Radius - not used with Rover
    global_mission.pulsar_mission_item[0].param4 = 0.0;   // Yaw - not used with Rover
    global_mission.pulsar_mission_item[0].x = -337700000; // Latitude - level with my house
    global_mission.pulsar_mission_item[0].y = 1512700000; // Longitude - some distance east off the coast
    global_mission.pulsar_mission_item[0].z = 0.0;        // Altitude - not used with Rover.

    // populate dummy item1
    global_mission.pulsar_mission_item[1].sequence_number = 1;
    global_mission.pulsar_mission_item[1].frame = MAV_FRAME_GLOBAL;       //  always this value, never change.
    global_mission.pulsar_mission_item[1].command = MAV_CMD_NAV_WAYPOINT; //  MAV_CMD_NAV_WAYPOINT (16 ) - https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
    global_mission.pulsar_mission_item[1].is_current_item = false;
    global_mission.pulsar_mission_item[1].auto_continue = true;
    global_mission.pulsar_mission_item[1].param1 = 100;   // Hold - not used with Rover
    global_mission.pulsar_mission_item[1].param2 = 0.0;   // Accept Radius - not used with Rover
    global_mission.pulsar_mission_item[1].param3 = 0.0;   // Pass Radius - not used with Rover
    global_mission.pulsar_mission_item[1].param4 = 0.0;   // Yaw - not used with Rover
    global_mission.pulsar_mission_item[1].x = -340000000; // Latitude - level with my house
    global_mission.pulsar_mission_item[1].y = 1530000000; // Longitude - some distance east off the coast
    global_mission.pulsar_mission_item[1].z = 0.0;        // Altitude - not used with Rover.

    // populate dummy item2
    global_mission.pulsar_mission_item[2].sequence_number = 2;
    global_mission.pulsar_mission_item[2].frame = MAV_FRAME_GLOBAL;       //  always this value, never change.
    global_mission.pulsar_mission_item[2].command = MAV_CMD_NAV_WAYPOINT; //  MAV_CMD_NAV_WAYPOINT (16 ) - https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
    global_mission.pulsar_mission_item[2].is_current_item = false;
    global_mission.pulsar_mission_item[2].auto_continue = true;
    global_mission.pulsar_mission_item[2].param1 = 200;   // Hold - not used with Rover
    global_mission.pulsar_mission_item[2].param2 = 0.0;   // Accept Radius - not used with Rover
    global_mission.pulsar_mission_item[2].param3 = 0.0;   // Pass Radius - not used with Rover
    global_mission.pulsar_mission_item[2].param4 = 0.0;   // Yaw - not used with Rover
    global_mission.pulsar_mission_item[2].x = -330000000; // Latitude - same as previous waypoint
    global_mission.pulsar_mission_item[2].y = 1530000000; // Longitude - additional distance east off the coast
    global_mission.pulsar_mission_item[2].z = 0.0;        // Altitude - not used with Rover.

    // populate dummy item3
    global_mission.pulsar_mission_item[3].sequence_number = 3;
    global_mission.pulsar_mission_item[3].frame = MAV_FRAME_GLOBAL;       //  always this value, never change.
    global_mission.pulsar_mission_item[3].command = MAV_CMD_NAV_WAYPOINT; //  MAV_CMD_NAV_WAYPOINT (16 ) - https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
    global_mission.pulsar_mission_item[3].is_current_item = false;
    global_mission.pulsar_mission_item[3].auto_continue = false;
    global_mission.pulsar_mission_item[3].param1 = 300;   // Hold - not used with Rover
    global_mission.pulsar_mission_item[3].param2 = 0.0;   // Accept Radius - not used with Rover
    global_mission.pulsar_mission_item[3].param3 = 0.0;   // Pass Radius - not used with Rover
    global_mission.pulsar_mission_item[3].param4 = 0.0;   // Yaw - not used with Rover
    global_mission.pulsar_mission_item[3].x = -330000000; // Latitude - North of level with my house
    global_mission.pulsar_mission_item[3].y = 1520000000; // Longitude - same as previous waypoint
    global_mission.pulsar_mission_item[3].z = 0.0;        // Altitude - not used with Rover.

    // populate additional mission struct items
    global_mission.count = 4; // includes the "[0]" item.
    global_mission.status = READY_TO_UPLOAD;

    debugPrintln("global_mission_dummy_load() - FINISHED");
} // END - global_mission_dummy_load()

/*============================
 * mission_upload_to_ap()
 *
 * My function to attemp (only once) the whole process of the FMX uploading a complete mission to the AP.
 * If you want to try the upload multiple times, then call this function multiple times.
 *
 * First we check we even have a valid mission pre loaded into out global variable struct "global_mission".
 * Then we attemp the upload to the AP based on the flow diagram
 * here - https://mavlink.io/en/services/mission.html#uploading_mission
 *
 * returns result of the upload attempt in type MAV_MISSION_RESULT - https://mavlink.io/en/messages/common.html#MAV_MISSION_RESULT
 *============================*/
PulsarMavMissionResult_t mission_upload_to_ap()
{
    debugPrintln("mission_upload_to_ap() - START");

    // Create and initialise local result/return related structs
    PulsarMavMissionResult_t my_pmmr;  // local struct to receive results from subordinate function(s)
    PulsarMavMissionResult_t sub_pmmr; // local struct to hold results ready for return to callee.
    my_pmmr.pr = NO_ERROR;
    my_pmmr.mmr = MAV_MISSION_ACCEPTED;
    my_pmmr.u32value = 0;
    sub_pmmr.pr = NO_ERROR;
    sub_pmmr.mmr = MAV_MISSION_ACCEPTED;
    sub_pmmr.u32value = 0;

    // Prep required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Create and initialise local state machine vars.
    PULSAR_MISSION_UPLOAD_SM_STATE state = SEND_MISSION_COUNT; // we always start at this state.
    boolean run_machine = true;                                // simple flag to end execution of this state machine.

    // Create and initialise other local vars
    uint16_t mission_item_num; // to specify a particular mission item in the overall mission.
    mission_item_num = 0;      // XXX - following text may have proven incorrect - 1st mission item slot on the AP.  ArduPilot uses slot 0 for Home Location.

    // Prework
    // Check we have a valid mission in our global mission variable before we attempt to upload it to the AP
    if (global_mission_valid())
    {
        state = SEND_MISSION_COUNT; // starting state for the state machine.
        run_machine = true;         // flag to run state machine or not.
        debugPrint("mission_upload_to_ap() - mission looks READY_TO_UPLOAD - count=");
        debugPrintlnInt(global_mission.count);
    }
    else
    {
        run_machine = false;                   // skip whole state machine
        my_pmmr.pr = STORED_MISSION_NOT_VALID; // set result correctly, ready for return to callee
        my_pmmr.mmr = MAV_MISSION_ERROR;       // set result correctly, ready for return to callee
        state = UPLOAD_END;                    // should not be used but I am setting it defensively.
        debugPrintln("mission_upload_to_ap() - mission INVALID - we will abort upload");
    }

    // ==================
    // State Machine - mission_upload_to_ap()
    // ==================
    while (run_machine)
    {
        debugPrintln("mission_upload_to_ap() - Iteration of State Machine");

        switch (state)
        {
        // =====================
        case SEND_MISSION_COUNT:
            // =====================
            {
                debugPrintln("mission_upload_to_ap() - case SEND_MISSION_COUNT:");

                // FMX > AP - MISSION_COUNT ( #44 ) - https://mavlink.io/en/messages/common.html#MISSION_COUNT
                // Tell the AP we want to upload a mission of COUNT items.
                // ========================

                // Pack the message
                mavlink_msg_mission_count_pack(FMX_SYS_ID, FMX_COMP_ID, &msg, AP_SYS_ID, AP_COMP_ID, global_mission.count);
                uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.

                // Send the message
                Serial1.write(buf, len); // Write data to serial port

                state = AWAIT_ITEM_REQUEST; // Set next state

                break;
            }

        // =====================
        case AWAIT_ITEM_REQUEST:
            // =====================
            {
                debugPrintln("mission_upload_to_ap() - case AWAIT_ITEM_REQUEST:");

                // Parse MAVLink looking for MAV MSG MISSION_REQUEST_INT (#51) https://mavlink.io/en/messages/common.html#MISSION_REQUEST_INT

                // Look for the specific message
                sub_pmmr = mavlink_get_MISSION_REQUEST_INT();
                // on return if successful sub_pmmr.pr = SUCCESS then sub_pmmr.u32value will contain the requested seq num.
                // or if                   sub_pmmr.pr != SUCCESS, then sub_pmmr.u32value is irrelevant.

                // Process results and determine next state.
                if (sub_pmmr.pr == TIMEOUT)
                {
                    // We TIMED OUT before getting a MISSION_REQUEST_INT :(
                    debugPrintln("mission_upload_to_ap() - case AWAIT_ITEM_REQUEST: We TIMED OUT before getting a MISSION_REQUEST_INT :(");
                    my_pmmr.pr = TIMEOUT;       // set result correctly, ready for return to callee
                    my_pmmr.mmr = sub_pmmr.mmr; // copy sub mmr into my mmr, ready for return to callee
                    my_pmmr.u32value = 0;       // set result to zero, just to be clean, ready for return to callee
                    state = UPLOAD_END;         // We timed out, so go to state UPLOAD_END next.
                }
                else if ((sub_pmmr.pr == SUCCESS) && (sub_pmmr.mmr == MAV_MISSION_ACCEPTED) && (sub_pmmr.u32value == mission_item_num))
                {
                    // Everything is looking perfect
                    debugPrintln("mission_upload_to_ap() - case AWAIT_ITEM_REQUEST: Everything is looking perfect");
                    state = SEND_MISSION_ITEM; // Set next state
                }
                else if ((sub_pmmr.pr == SUCCESS) && (sub_pmmr.mmr == MAV_MISSION_ACCEPTED) && (sub_pmmr.u32value != mission_item_num))
                {
                    debugPrintln("mission_upload_to_ap() - case  AWAIT_ITEM_REQUEST: We did get a MISSION_REQUEST_INT but its asking for an item thats not what we expect :(");
                    // We did get a MISSION_REQUEST_INT but its asking for an item thats not what we expect :(
                    my_pmmr.pr = ITEM_SEQUENCE_ERROR; // set result correctly, ready for return to callee
                    my_pmmr.mmr = sub_pmmr.mmr;       // copy sub mmr into my mmr, ready for return to callee
                    my_pmmr.u32value = 0;             // set result to zero, just to be clean, ready for return to callee
                    state = UPLOAD_END;               // Something was wrong in the responses, so go to state UPLOAD_END next.
                }

                break;
            }
        case SEND_MISSION_ITEM:
        {
            debugPrint("mission_upload_to_ap() - case SEND_MISSION_ITEM: - item[");
            debugPrintInt(mission_item_num);
            debugPrintln("]");

            // FMX > AP - MISSION_ITEM_INT ( #73 ) - https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT
            // Send the AP the particular mission item it asked for.
            // ========================

            debugPrint("mission_upload_to_ap() - case SEND_MISSION_ITEM: - sequence_number[");
            debugPrintInt(global_mission.pulsar_mission_item[mission_item_num].sequence_number);
            debugPrintln("]");

            // Pack the message
            mavlink_msg_mission_item_int_pack(FMX_SYS_ID, FMX_COMP_ID, &msg, AP_SYS_ID, AP_COMP_ID,
                                              global_mission.pulsar_mission_item[mission_item_num].sequence_number, // seq - Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4).
                                              global_mission.pulsar_mission_item[mission_item_num].frame,           // frame - MAV_FRAME - The coordinate system of the waypoint.
                                              global_mission.pulsar_mission_item[mission_item_num].command,         // command - MAV_CMD - The scheduled action for the waypoint.
                                              global_mission.pulsar_mission_item[mission_item_num].is_current_item, // current - false:0, true:1
                                              global_mission.pulsar_mission_item[mission_item_num].auto_continue,   // autocontinue - Autocontinue to next waypoint
                                              global_mission.pulsar_mission_item[mission_item_num].param1,          // not used with Rover
                                              global_mission.pulsar_mission_item[mission_item_num].param2,          // not used with Rover
                                              global_mission.pulsar_mission_item[mission_item_num].param3,          // not used with Rover
                                              global_mission.pulsar_mission_item[mission_item_num].param4,          // not used with Rover
                                              global_mission.pulsar_mission_item[mission_item_num].x,               // x/Latitude
                                              global_mission.pulsar_mission_item[mission_item_num].y,               // y/Longitude
                                              global_mission.pulsar_mission_item[mission_item_num].z);              // not used with Rover

            uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); // put message into our send buffer and also get it's size in bytes.

            // Send the message
            Serial1.write(buf, len); // Write data to serial port

            // Decide next state
            if (mission_item_num < (global_mission.count - 1)) // its (global_mission.count - 1) because the mission_item_num starts at 0
                                                               // so if the mission has 5 items, we need to wrap up here when mission_item_num
                                                               // reaches 4.
            {
                mission_item_num++;
                state = AWAIT_ITEM_REQUEST; // Set next state
            }
            else
                state = AWAIT_MISSION_ACK; // Set next state

            break;
        }

        // ====================
        case AWAIT_MISSION_ACK:
            // ====================
            {
                debugPrintln("mission_upload_to_ap() - case AWAIT_MISSION_ACK:");

                // Parse MAVLink looking for MAV MSG MISSION_ACK ( #47 ) https://mavlink.io/en/messages/common.html#MISSION_ACK

                // Look for the specific message
                sub_pmmr = mavlink_get_MISSION_ACK();
                // on return if successful sub_pmmr.pr = SUCCESS  or TIMEOUT

                // Process results and determine next state.
                if (sub_pmmr.pr == TIMEOUT)
                {
                    // We TIMED OUT before getting a MISSION_ACK :(
                    my_pmmr.pr = TIMEOUT;       // set result correctly, ready for return to callee
                    my_pmmr.mmr = sub_pmmr.mmr; // copy sub mmr into my mmr, ready for return to callee
                    my_pmmr.u32value = 0;       // set result to zero, just to be clean, ready for return to callee
                    state = UPLOAD_END;         // We timed out, so go to state UPLOAD_END next.
                }
                else if ((sub_pmmr.pr == SUCCESS) && (sub_pmmr.mmr == MAV_MISSION_ACCEPTED))
                {
                    // Everything is looking perfect - we got a MISSION_ACK and its MMR = MAV_MISSION_ACCEPTED
                    my_pmmr.pr = SUCCESS;       // set result correctly, ready for return to callee
                    my_pmmr.mmr = sub_pmmr.mmr; // copy sub mmr into my mmr, ready for return to callee
                    my_pmmr.u32value = 0;       // set result to zero, just to be clean, ready for return to callee
                    state = UPLOAD_END;         // We are all done with the Upload, so go to state UPLOAD_END next.
                }
                else if ((sub_pmmr.pr == SUCCESS) && (sub_pmmr.mmr == MAV_MISSION_ACCEPTED))
                {
                    // We did get a MISSION_ACK but its MAV_MISSION_RESULT contained an error :(
                    my_pmmr.pr = FAILED;        // set result correctly, ready for return to callee
                    my_pmmr.mmr = sub_pmmr.mmr; // copy sub mmr into my mmr, ready for return to callee
                    my_pmmr.u32value = 0;       // set result to zero, just to be clean, ready for return to callee
                    state = UPLOAD_END;         // Something was wrong in the responses, so go to state UPLOAD_END next.
                }
                break;
            }

        // =============
        case UPLOAD_END:
            // =============
            {
                debugPrintln("mission_upload_to_ap() - case UPLOAD_END:");
                /* any tidy up code before we exit the state machine */

                run_machine = false; // end the whole state machine
                break;
            }
        default:
            break;

        } // END - switch(state)

    } // END - while(run_machine)

    debugPrint("mission_upload_to_ap() -       sub_pmmr.pr:");
    Serial.println(sub_pmmr.pr);
    debugPrint("mission_upload_to_ap() -      sub_pmmr.mmr:");
    Serial.println(sub_pmmr.mmr);
    debugPrint("mission_upload_to_ap() - sub_pmmr.u32value:");
    Serial.println(sub_pmmr.u32value);

    debugPrint("mission_upload_to_ap() -        my_pmmr.pr:");
    Serial.println(my_pmmr.pr);
    debugPrint("mission_upload_to_ap() -       my_pmmr.mmr:");
    Serial.println(my_pmmr.mmr);
    debugPrint("mission_upload_to_ap() -  my_pmmr.u32value:");
    Serial.println(my_pmmr.u32value);

    debugPrintln("mission_upload_to_ap() - FINISHING");

    return my_pmmr;

} // END - mission_upload_to_ap()