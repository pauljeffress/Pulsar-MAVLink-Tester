/*
 * main.cpp
 */

#include <Arduino.h>
#include <common/mavlink.h>
#include "mavlink_fns.h"
#include "debug_fns.h"


// function pre defines, to aid compilation.


/*
 *  setup()
 */
void setup()
{
    Serial.begin(115200);   // Initialise serial monitor/console port
    delay(5000);            // wait for Serial to come up so we don't miss initial messages
    enableDebugging(Serial);// enable my debugging print functions

    Serial.println("Pulsar MAVLink Tester");

    Serial1.begin(57600);   // Initialise Serial1 port at 57600bps
                            // Arduino 2nd serial port (a.k.a Serial1) is wired to AutoPilot GPS2 port (see README.md for additional details)

    mavlink_request_datastream();   // 
}

/*
 *  loop()
 */
void loop()
{
    mavlink_receive();


} // END - loop()





