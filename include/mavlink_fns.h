/*
 * mavlink_fns.h
 *
 * header file for my various mavlink functions.
 *
 */

#ifndef MAVLINK_FNS_H
#define MAVLINK_FNS_H

#include "Arduino.h"    // helps with the "types" used here.
#include <TimeLib.h>    // https://github.com/PaulStoffregen/Time

// Below mavlink include draws from the c_library_v1 library I manually added to this PlatformIO project in the "include" folder.
#include <ardupilotmega/mavlink.h> // The Mavlink library for the "ardupilotmega" dialect (which sub's in the "common" dialect too)

// MAVLink Specific Debugs
#define MAVLINK_DEBUG  // uncomment if you want the various MAVLINK routines to execute their debugPrint... statements

// MAVLink IDs - https://ardupilot.org/dev/docs/mavlink-basics.html
#define FMX_SYS_ID 1    // MAVLink System ID of this device. For example a companion computer (Arduino, RasPi etc) which is sending a heartbeat.
                        // I am setting this to 1, as all electronics on the drone itself should have the same SystemID as
                        // the AutoPilot, and it has sysID = 1.
#define FMX_COMP_ID 100 // MAVLink Component ID of this device. For example a companion computer (Arduino, RasPi etc) which is sending a heartbeat.
                        // I am setting this to 100, as it can be from 1 to 255 but the AutoPilot has compID = 1 so can't use that.
#define AP_SYS_ID 1     // MAVLink System ID of the autopilot.
#define AP_COMP_ID 1    // MAVLink Component ID of the autopilot.

#define FMX_MAVLINK_MAX_TRYS 3                // How many times the FMX should try to execute a MAVLink message and get a positive result/ack.
#define FMX_MAVLINK_RX_WINDOW_REGULAR_MS 4000 // milliSeconds - when we are parsing inbound MAVLink data from the AP, typically looking for or expecting
                                              // something specific, how long should we parse for.
#define FMX_MAVLINK_RX_WINDOW_LONG_MS 8000    // milliSeconds - similar role to FMX_MAVLINK_RX_WINDOW_REGULAR_MS above but for when I want to wait longer

#define PULSAR_MISSION_MAX_ITEMS 4             // the max number of mission items the FMX can work with.
#define PULSAR_MISSION_ITEM_SEQ_PLACEHOLDER 66 // I use this when I initialise a GlobalMission_t as it is outside of the range
                                               // I will ever use.  So I can test for it (for example when checking if my struct
                                               // has a valid mission in it)and act accordingly.

/* define any enums */

typedef enum PULSAR_STORED_MISSION_STATUS
{
    UNKNOWN,          // FMX mission structure is in an unknown state, next state must be INITIALISED.
    INITIALISED,      // FMX mission structure has been initialised (e.g. zero'd), ready for a mission to be loaded.
    PARTIAL,          // FMX only has part of a mission, its not complete yet.
    INVALID,          // FMX mission structure is an invalid mission.
    READY_TO_UPLOAD,  // FMX has a complete mission ready for upload to AP.
    UPLOAD_FAILED,    // FMX has attempted to upload the current mission and failed.
    UPLOAD_SUCCEEDED, // FMX has attempted to upload the current mission and succeeded.
} PULSAR_STORED_MISSION_STATUS;

typedef enum PULSAR_MISSION_UPLOAD_SM_STATE // State machine states for mission_upload_to_ap()
{                                           // see flow diagram here - https://mavlink.io/en/services/mission.html#uploading_mission
    SEND_MISSION_COUNT,
    AWAIT_ITEM_REQUEST,
    SEND_MISSION_ITEM,
    AWAIT_MISSION_ACK,
    UPLOAD_END,
} PULSAR_MISSION_UPLOAD_SM_STATE;

typedef enum PULSAR_RESULT // set of results/errors unique to my Pulsar project
{
    NOT_SET,
    NO_ERROR,
    TIMEOUT,
    SEQUENCE_ERROR,
    SUCCESS,
    FAILED,
    MISSION_REQUEST_INT_TIMEOUT_ERROR,
    MISSION_ACK_TIMEOUT_ERROR,
    ITEM_SEQUENCE_ERROR,
    STORED_MISSION_NOT_VALID,
} PULSAR_RESULT;

/* define any struct's */

typedef struct // PulsarMissionItem_t - used to populate MISSION_ITEM_INT ( #73 ) https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT
{
    // struct built based on the types defined at MISSION_ITEM_INT ( #73 ) https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT
    // and the https://github.com/mavlink/qgroundcontrol/blob/master/src/MissionManager/MissionItem.h code.

    uint16_t sequence_number; // Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4).
                              // ArduPilot reserves seq=0 as follows "The first mission sequence number (seq==0) is populated with the home position of the vehicle instead of the first mission item."
                              // according to https://mavlink.io/en/services/mission.html
    MAV_FRAME frame;          // The coordinate system of the waypoint.
    MAV_CMD command;          // The scheduled action for the waypoint.
    uint8_t is_current_item;  // used like a BOOLEAN - false:0, true:1
    uint8_t auto_continue;    // used like a BOOLEAN - false:0, true:1 - Autocontinue to next waypoint
    float param1;             // PARAM1, see MAV_CMD enum
    float param2;             // PARAM1, see MAV_CMD enum
    float param3;             // PARAM1, see MAV_CMD enum
    float param4;             // PARAM1, see MAV_CMD enum
    int32_t x;                // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
    int32_t y;                // PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
    float z;                  // PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
} PulsarMissionItem_t;

typedef struct // GlobalMission_t  - holds a complete mission
{
    PULSAR_STORED_MISSION_STATUS status;                               // the state of the stored mission in this mission structure. UNKNOWN until explicitly set to something else.
    uint16_t count;                                                    // the number of MISSION_ITEMS we have in this mission
    PulsarMissionItem_t pulsar_mission_item[PULSAR_MISSION_MAX_ITEMS]; // the array of mission items structs themselves.
} GlobalMission_t;

typedef struct // PulsarMissionItemSubset_t - used to transport just the mission item fields Pulsar needs.
{
    // struct is a subset of my struct PulsarMissionItem_t
    uint16_t sequence_number; // Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4).
                              // ArduPilot reserves seq=0 as follows "The first mission sequence number (seq==0) is populated with the home position of the vehicle instead of the first mission item."
                              // according to https://mavlink.io/en/services/mission.html
    MAV_FRAME frame;          // The coordinate system of the waypoint.
    MAV_CMD command;          // The scheduled action for the waypoint.
    uint8_t is_current_item;  // used like a BOOLEAN - false:0, true:1
    uint8_t auto_continue;    // used like a BOOLEAN - false:0, true:1 - Autocontinue to next waypoint

    int32_t x; // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
    int32_t y; // PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7

} PulsarMissionItemSubset_t;

typedef struct // GlobalMissionSubset_t  - holds a subset of complete mission
{
    // struct is a subset of my struct GlobalMission_t
    PULSAR_STORED_MISSION_STATUS status;                                            // the state of the stored mission in this mission structure. UNKNOWN until explicitly set to something else.
    uint16_t count;                                                                 // the number of MISSION_ITEMS we have in this mission
    PulsarMissionItemSubset_t pulsar_mission_item_subset[PULSAR_MISSION_MAX_ITEMS]; // the array of mission items structs themselves.
} GlobalMissionSubset_t;

typedef struct // PulsarMavMissionResult_t - a struct to store a MAV_MISSION_RESULT and some additional info specific to my project
{
    PULSAR_RESULT pr;       // See ENUM above
    MAV_MISSION_RESULT mmr; // See https://mavlink.io/en/messages/common.html#MAV_MISSION_RESULT
    uint32_t u32value;      // Any int I want to bring back in a return/result
} PulsarMavMissionResult_t;

/* global variables */

extern GlobalMission_t global_mission;
extern GlobalMissionSubset_t global_mission_subset;

/* function pre defines */

// from mavlink_fns.cpp
void initMAVLinkSettings();
void mavlink_fmx_send_heartbeat_to_ap();
void mavlink_test_request_one_param_from_ap();
void mavlink_test_set_one_param_on_ap();
bool mavlink_set_one_param_on_ap(char *name, int32_t value_int, float value_float, uint8_t valuetype);
void mavlink_set_arm_ap();
void mavlink_set_disarm_ap();
void mavlink_set_flightmode_ap(uint8_t desired_flightmode);
void mavlink_cmd_preflight_reboot_ap();
// from mavlink_stream_fns.cpp
void mavlink_request_datastream();
void mavlink_unrequest_datastream();
void mavlink_request_streaming_params_from_ap();
void mavlink_unrequest_streaming_params_from_ap();
// from mavlink_receive_fns.cpp
void mavlink_receive();
// from mavlink_mission_fns.cpp
PulsarMavMissionResult_t mission_clear_all_ap();
PulsarMavMissionResult_t mission_upload_to_ap();
PulsarMavMissionResult_t mavlink_get_MISSION_ACK();
bool global_mission_valid();
void global_mission_init();
void global_mission_print();
void global_mission_subset_print();
void global_mission_dummy_load();

#endif

// END - mavlink_fns.h