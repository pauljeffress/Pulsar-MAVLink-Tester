/*
 *  setup.cpp
 */

#include "debug_fns.h"

/*
 *  setup()
 */
void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);   // Initialise LED pin

    Serial.begin(115200);   // Initialise serial monitor/console port
    delay(5000);            // wait for Serial to come up so we don't miss initial messages
    enableDebugging(Serial);// enable my debugging print functions

    Serial.println("Pulsar MAVLink Tester");

    Serial1.begin(57600);   // Initialise Serial1 port at 57600bps
                            // Arduino 2nd serial port (a.k.a Serial1) is wired to AutoPilot GPS2 port (see README.md for additional details)
}
