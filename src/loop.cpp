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
    menuDo();   // check for and action user menus.
    
    Serial.print(".");  // Print stuff to show we are running...

    if (MavRecOn)   mavlink_receive();





    // Example of some debug statements
    if (DebugNormalOperation == true)
    {
        Serial.println("loop() - Debug debug debug...");
    }

    // Some more fake actions to execute to show we are running...
    digitalWrite(LED_BUILTIN, HIGH); // Switch the builtin led on
    delay(250);
    digitalWrite(LED_BUILTIN, LOW); // Switch the builtin led off
    delay(100);

} // END - loop()