# Pulsar MAVLink Tester

My goal here is twofold;  
**First** - I'd like to pull together all I have learnt and achieved when talking MAVLink between an Arduino board and ArduPilot running on my Flight Controller (Cube Orange).

**Second** - I'd like to share it in the hope that someone else can benefit from it, just like I have from everyone else out there doing this stuff before me.  In particular, the those that started (Juan Pedro López & Pedro Albuquerque....thank you, thank you, thank you) and answered questions in the following two threads in the ArduPilot forum.  I would highly encourage anyone interested in this space to read both of those threads from start to finish, you will learn a lot along the way.

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

> Pulsar MAVLink Tester  
>   
>[1].......Toggle mavlink_receive() in loop(): [OFF]  
>[2].......mavlink_request_datastream()  
>[3].......mavlink_unrequest_datastream()  
>[4].......mavlink_request_streaming_params_from_ap()  
>[5].......mavlink_unrequest_streaming_params_from_ap()  
>[6].......mavlink_set_arm_ap()  
>[7].......mavlink_set_disarm_ap()  
>[8].......mavlink_test_set_one_param_on_ap()  
>[9].......mavlink_test_request_one_param_on_ap()  
>  
>[d].......Toggle Debug - state: [OFF]  
>  
>[x].......Exit this Menu  
>  
>Enter your choice/number:   
>:::::


I'd suggest you start by using option 1, to turn on the mavlink_receive() function in loop(), and then press 'x' to get out of the menus.
This will just display and decode what it can from your autopilot if it is correctly connected to the Arduino.  If you don't see any MAVLink messages check your wiring and also check that you are using the right Serialx port in setup().  In my case I have Serial1 wired to the autopilot, you may be using another port?
If you have checked the wiring and the Serialx port number and you still don't see anything, perhaps you have not configured your autopilot to talk MAVLink over the port you are connected to it on?  I use Mission Planner to do this on my system.

Ultimately you should see something like this, but it will be different for your system.

>mavlink_receive() - WARNING - MSG(s) missed from sysID:1,compID:1 >according to seq nums!  
>mavlink_receive() - MSG RCVD - magic:254 seq:203 src sysid:1 src compid:1 >msgid#:36=SERVO_OUTPUT_RAW - undecoded  
>mavlink_receive() - MSG RCVD - magic:254 seq:204 src sysid:1 src compid:1 >msgid#:65=RC_CHANNELS - undecoded  
>mavlink_receive() - MSG RCVD - magic:254 seq:205 src sysid:1 src compid:1 >msgid#:35=RC_CHANNELS_RAW - undecoded  
>mavlink_receive() - MSG RCVD - magic:254 seq:206 src sysid:1 src compid:1 >msgid#:0=HEARTBEAT Type:11 Autopilot:3 BaseMode:1 CustomMode/Flightmode:4 >SystemStatus:5 MavlinkVersion:3  
>mavlink_receive() - MSG RCVD - magic:254 seq:207 src sysid:1 src compid:1 >msgid#:36=SERVO_OUTPUT_RAW - undecoded  
>mavlink_receive() - MSG RCVD - magic:254 seq:208 src sysid:1 src compid:1 >msgid#:65=RC_CHANNELS - undecoded  
>mavlink_receive() - MSG RCVD - magic:254 seq:209 src sysid:1 src compid:1 >msgid#:35=RC_CHANNELS_RAW - undecoded  
>mavlink_receive() - MSG RCVD - magic:254 seq:120 src sysid:1 src compid:0 >msgid#:0=HEARTBEAT Type:27 Autopilot:8 BaseMode:4 CustomMode/Flightmode:0 >SystemStatus:4 MavlinkVersion:3  
>mavlink_receive() - MSG RCVD - magic:254 seq:210 src sysid:1 src compid:1 >msgid#:111=TIMESYNC - undecoded  
>mavlink_receive() - MSG RCVD - magic:254 seq:211 src sysid:1 src compid:1 >msgid#:36=SERVO_OUTPUT_RAW - undecoded  
>mavlink_receive() - WARNING - MSG(s) missed from sysID:1,compID:1 >according to seq nums!  
>mavlink_receive() - MSG RCVD - magic:254 seq:127 src sysid:1 src compid:1 >msgid#:65=RC_CHANNELS - undecoded  
>mavlink_receive() - MSG RCVD - magic:254 seq:128 src sysid:1 src compid:1 >msgid#:35=RC_CHANNELS_RAW - undecoded  
>mavlink_receive() - MSG RCVD - magic:254 seq:129 src sysid:1 src compid:1 >msgid#:0=HEARTBEAT Type:11 Autopilot:3 BaseMode:1 CustomMode/Flightmode:4 >SystemStatus:5 MavlinkVersion:3  
>mavlink_receive() - MSG RCVD - magic:254 seq:130 src sysid:1 src compid:1 >msgid#:36=SERVO_OUTPUT_RAW - undecoded  
>mavlink_receive() - MSG RCVD - magic:254 seq:131 src sysid:1 src compid:1 >msgid#:65=RC_CHANNELS - undecoded  
>mavlink_receive() - MSG RCVD - magic:254 seq:132 src sysid:1 src compid:1 >msgid#:35=RC_CHANNELS_RAW - undecoded  
>mavlink_receive() - MSG RCVD - magic:254 seq:120 src sysid:1 src compid:0 >msgid#:0=HEARTBEAT Type:27 Autopilot:8 BaseMode:4 CustomMode/Flightmode:0 >SystemStatus:4 MavlinkVersion:3  
>mavlink_receive() - MSG RCVD - magic:254 seq:133 src sysid:1 src compid:1 >msgid#:36=SERVO_OUTPUT_RAW - undecoded  
>mavlink_receive() - MSG RCVD - magic:254 seq:134 src sysid:1 src compid:1 >msgid#:65=RC_CHANNELS - undecoded  
>mavlink_receive() - MSG RCVD - magic:254 seq:135 src sysid:1 src compid:1 >msgid#:35=RC_CHANNELS_RAW - undecoded  



## Coding Environment

I use [PlatformIO](https://platformio.org) rather than the Arduino IDE.  I find it awesome in so many way, yes there is a little bit of learning to get yourself going, but once you have, you will never go back to the Arduino IDE. There are great guides on it elsewhere, gut you may want to have a look at the my [platformio.ini](https://github.com/pauljeffress/Pulsar-MAVLink-Tester/blob/master/platformio.ini) file in this GitHub repository to see any specific settings I have made.

## Arduino to AutoPilot Setup

My [Adafruit Feather M4 CAN](https://www.adafruit.com/product/4759) Arduino board is wired (TX, RX, GND) to my [Cube Orange](https://ardupilot.org/copter/docs/common-thecubeorange-overview.html) GPS 2 port. Both devices operate at 3.3v so no level shifters are required. My Arduino has a 2nd physical serial port, its known as Serial1. That way I can keep the 1st serial port (known as Serial) free so I can use it as a monitor port and watch the code execute etc.

Make sure you connect as follows;

AutoPilot       Arduino

GPS2 Port       Serial1

GND ----------- GND

TX  ----------- RX

RX  ----------- TX

On the Arduino I configure Serial1 to operate at 57600bps with

    Serial1.begin(57600);

On the AutoPilot I use Mission Planner to configure ArduPilot and set GPS 2 port to 57600 and MAVLink v1.

