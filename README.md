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
  - [Coding Environment](#coding-environment)
  - [Arduino to AutoPilot Setup](#arduino-to-autopilot-setup)


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

