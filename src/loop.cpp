/*
 *  loop.cpp
 */

#include "main.h"
#include "mavlink_fns.h"
#include "SerSrvMnu_fns.h"

/*
 *  loop()
 */
void loop()
{
    int ledState = 0;
    
    menuDo();   // check for and action user menus.
    
    //Serial.print(".");  // Print stuff to show we are running...

    if (MavRecOn)   mavlink_receive();


    // Some more actions to execute to show loop() is running...
    // Non blocking LED toggler
    if((millis() % 2000) > 1000) ledState = 1;
    else                         ledState = 0;
    digitalWrite(LED_BUILTIN, ledState);

} // END - loop()