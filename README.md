# Pulsar MAVLink Tester

My goal here is twofold;  
**First** - I'd like to pull together all I have learnt and achieved when talking MAVLink between an Arduino board and ArduPilot running on my Flight Controller (Cube Orange).

**Second** - I'd like to share it in the hope that someone else can benefit from it, just like I have from everyone else out there doing this stuff before me.  In particular, those that started (Juan Pedro López & Pedro Albuquerque....thank you, thank you, thank you) and answered questions in the following two threads in the ArduPilot forum.  I would highly encourage anyone interested in this space to read both of those threads from start to finish, you will learn a lot along the way.

MAVLink and Arduino: step by step
https://discuss.ardupilot.org/t/mavlink-and-arduino-step-by-step/25566

MAVLink Step by Step
https://discuss.ardupilot.org/t/mavlink-step-by-step/9629

## Table of Contents
- [Pulsar MAVLink Tester](#pulsar-mavlink-tester)
  - [Table of Contents](#table-of-contents)
  - [Operation](#operation)
  - [Coding Environment](#coding-environment)
  - [Arduino to AutoPilot Setup](#arduino-to-autopilot-setup)


## Operation

Once the code is running on your Arduino, open the Serial Monitor and press 'm' to open the menu system in the tester.
You should see a menu something like this;
```
Pulsar MAVLink Tester
========================='
[1].......Toggle mavlink_receive() in loop(): [OFF]
[2].......STREAM   - mavlink_request_datastream>(MAV_DATA_STREAM_RAW_SENSORS)
[3].......STREAM   - mavlink_unrequest_datastream(MAV_DATA_STREAM_ALL);
[4].......STREAM   - mavlink_request_streaming_params_from_ap()
[5].......STREAM   - mavlink_unrequest_streaming_params_from_ap()

[6].......ARM      - mavlink_set_arm_ap()
[7].......DISarm   - mavlink_set_disarm_ap()

[8].......SET      - mavlink_test_set_one_param_on_ap()
[9].......REQUEST  - mavlink_test_request_one_param_on_ap()

[r].......REBOOT   - mavlink_cmd_preflight_reboot_ap()

[M].......FLT MODE - mavlink_set_flightmode_ap(0)- MANUAL
[H].......FLT MODE - mavlink_set_flightmode_ap(4)- HOLD
[A].......FLT MODE - mavlink_set_flightmode_ap(10)- AUTO

[C].......MISSION  - clear local data
[L].......MISSION  - load dummy local data
[P].......MISSION  - print my local data
[U].......MISSION  - upload local data to AP
[E].......MISSION  - erase AP's mission

[x].......Exit this Menu

Enter your choice/number: 

```

I'd suggest you start by using option 1, to turn on the mavlink_receive() function in loop(), and then press 'x' to get out of the menus.

This will just display and decode what it can from your autopilot if it is correctly connected to the Arduino.  If you don't see any MAVLink messages check your wiring and also check that you are using the right Serialx port in setup().  In my case I have Serial1 wired to the autopilot, you may be using another port?

If you have checked the wiring and the Serialx port number and you still don't see anything, perhaps you have not configured your autopilot to talk MAVLink over the port you are connected to it on?  I use Mission Planner to do this on my system.

Ultimately you should see something like this, but it will be different for your system.

```
mavlink_receive() - WARNING - MSG(s) missed from sysID:1,compID:1 according to seq nums!
MSG RCVD - magic:254 seq:32 sysid:1 compid:1 msgid#:0=HEARTBEAT Type:11 APclass:3 BaseMode:65 DISarmed Custom/Flightmode:0 MANUAL SysStat:5 MavVer:3
MSG RCVD - magic:254 seq:33 sysid:1 compid:1 msgid#:0=HEARTBEAT Type:11 APclass:3 BaseMode:65 DISarmed Custom/Flightmode:0 MANUAL SysStat:5 MavVer:3
MSG RCVD - magic:254 seq:34 sysid:1 compid:1 msgid#:0=HEARTBEAT Type:11 APclass:3 BaseMode:65 DISarmed Custom/Flightmode:0 MANUAL SysStat:5 MavVer:3
MSG RCVD - magic:254 seq:35 sysid:1 compid:1 msgid#:0=HEARTBEAT Type:11 APclass:3 BaseMode:65 DISarmed Custom/Flightmode:0 MANUAL SysStat:5 MavVer:3
MSG RCVD - magic:254 seq:36 sysid:1 compid:1 msgid#:0=HEARTBEAT Type:11 APclass:3 BaseMode:65 DISarmed Custom/Flightmode:0 MANUAL SysStat:5 MavVer:3
MSG RCVD - magic:254 seq:37 sysid:1 compid:1 msgid#:0=HEARTBEAT Type:11 APclass:3 BaseMode:65 DISarmed Custom/Flightmode:0 MANUAL SysStat:5 MavVer:3
MSG RCVD - magic:254 seq:38 sysid:1 compid:1 msgid#:0=HEARTBEAT Type:11 APclass:3 BaseMode:65 DISarmed Custom/Flightmode:0 MANUAL SysStat:5 MavVer:3
MSG RCVD - magic:254 seq:39 sysid:1 compid:1 msgid#:111=TIMESYNC - undecoded
MSG RCVD - magic:254 seq:40 sysid:1 compid:1 msgid#:0=HEARTBEAT Type:11 APclass:3 BaseMode:65 DISarmed Custom/Flightmode:0 MANUAL SysStat:5 MavVer:3
MSG RCVD - magic:254 seq:41 sysid:1 compid:1 msgid#:0=HEARTBEAT Type:11 APclass:3 BaseMode:65 DISarmed Custom/Flightmode:0 MANUAL SysStat:5 MavVer:3
mavlink_receive() - WARNING - MSG(s) missed from sysID:1,compID:1 according to seq nums!
MSG RCVD - magic:254 seq:160 sysid:1 compid:1 msgid#:0=HEARTBEAT Type:11 APclass:3 BaseMode:65 DISarmed Custom/Flightmode:0 MANUAL SysStat:5 MavVer:3
MSG RCVD - magic:254 seq:161 sysid:1 compid:1 msgid#:0=HEARTBEAT Type:11 APclass:3 BaseMode:65 DISarmed Custom/Flightmode:0 MANUAL SysStat:5 MavVer:3
MSG RCVD - magic:254 seq:162 sysid:1 compid:1 msgid#:0=HEARTBEAT Type:11 APclass:3 BaseMode:65 DISarmed Custom/Flightmode:0 MANUAL SysStat:5 MavVer:3
MSG RCVD - magic:254 seq:163 sysid:1 compid:1 msgid#:0=HEARTBEAT Type:11 APclass:3 BaseMode:65 DISarmed Custom/Flightmode:0 MANUAL SysStat:5 MavVer:3
MSG RCVD - magic:254 seq:164 sysid:1 compid:1 msgid#:0=HEARTBEAT Type:11 APclass:3 BaseMode:65 DISarmed Custom/Flightmode:0 MANUAL SysStat:5 MavVer:3
MSG RCVD - magic:254 seq:165 sysid:1 compid:1 msgid#:0=HEARTBEAT Type:11 APclass:3 BaseMode:65 DISarmed Custom/Flightmode:0 MANUAL SysStat:5 MavVer:3
MSG RCVD - magic:254 seq:166 sysid:1 compid:1 msgid#:0=HEARTBEAT Type:11 APclass:3 BaseMode:65 DISarmed Custom/Flightmode:0 MANUAL SysStat:5 MavVer:3
MSG RCVD - magic:254 seq:167 sysid:1 compid:1 msgid#:0=HEARTBEAT Type:11 APclass:3 BaseMode:65 DISarmed Custom/Flightmode:0 MANUAL SysStat:5 MavVer:3
```

## Coding Environment

I use [PlatformIO](https://platformio.org) rather than the Arduino IDE.  I find it awesome in so many way, yes there is a little bit of learning to get yourself going, but once you have, you will never go back to the Arduino IDE. There are great guides on it elsewhere, gut you may want to have a look at the my [platformio.ini](https://github.com/pauljeffress/Pulsar-MAVLink-Tester/blob/master/platformio.ini) file in this GitHub repository to see any specific settings I have made.

## Arduino to AutoPilot Setup

My [Adafruit Feather M4 CAN](https://www.adafruit.com/product/4759) Arduino board is wired (TX, RX, GND) to my [Cube Orange](https://ardupilot.org/copter/docs/common-thecubeorange-overview.html) GPS 2 port. Both devices operate at 3.3v so no level shifters are required. My Arduino has a 2nd physical serial port, its known as Serial1. That way I can keep the 1st serial port (known as Serial) free so I can use it as a monitor port and watch the code execute etc.

Make sure you connect as follows;

|AutoPilot|       |Arduino|
|---------|-------|-------|
|*GPS2 Port*|       |*Serial1*|
| | | |
|GND| ----------- |GND|
|TX| ----------- |RX|
|RX| ----------- |TX|

On the Arduino I configure Serial1 to operate at 57600bps with

    Serial1.begin(57600);

On the AutoPilot I use Mission Planner to configure ArduPilot and set GPS 2 port to 57600 and MAVLink v1.

