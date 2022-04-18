
/*
 * SerSrvMnu_fns.cpp
 *
 * Functions that support the Serial Service Menu capability
 * from https://create.arduino.cc/projecthub/andreas_waldherr/serial-service-menu-91c5c1
 * with slight mods by me (p.jeffress)
 *
 */

#include "SerSrvMnu_fns.h"
#include "main.h"
#include "mavlink_fns.h"

// =======================
// Globals for this module
// =======================
const int MaxSerRx = 64;
boolean SerSess = false;          // Serial Session = false upon start
unsigned long SesTimeout = 60000; // Normal operation is resumed after Serial Session timeout

// debugging state globals
boolean DebugNormalOperation = false;

// =========================
// Functions for this module
// =========================

/*
 * Display the menu
 */
void menuPrint()
{
    menuClearScreen();
    Serial.println("- Pulsar MAVLink Tester -");
    Serial.println("=========================");
    Serial.println();
    Serial.print("[1].......Toggle mavlink_receive() in loop(): ");
    if (MavRecOn)
        Serial.println("[ON]");
    else
        Serial.println("[OFF]");
    Serial.println("[2].......mavlink_request_datastream()");
    Serial.println("[3].......mavlink_unrequest_datastream()");
    Serial.println("[4].......mavlink_request_streaming_params_from_ap()");
    Serial.println("[5].......mavlink_unrequest_streaming_params_from_ap()");
    Serial.println("[6].......mavlink_set_arm_ap()");
    Serial.println("[7].......mavlink_set_disarm_ap()");
    Serial.println("[8].......mavlink_test_set_one_param_on_ap()");
    Serial.println("[9].......mavlink_test_request_one_param_on_ap()");
    Serial.println("[a].......mavlink_set_flightmode_ap(0)- MANUAL");
    Serial.println("[b].......mavlink_set_flightmode_ap(3)- STEERING");
    Serial.println("[r].......mavlink_cmd_preflight_reboot_ap()");

    Serial.println();

    Serial.println("[x].......Exit this Menu");
    Serial.println();
    Serial.print("Enter your choice/number: ");

} // END - menuPrint()

/*
 * Check for menu start key and if detected,
 * then display the menu.
 */
void menuDo()
{
    char rc; // char we read into

    // If 'm' was detected, then start the menus
    if (SerSess == false && Serial.available())
    {
        rc = Serial.read();
        if (rc == 'm')
        {
            Serial.println();
            Serial.println("loop() paused - entering menu system");
            SerSess = true; // flag that the serial session is established
        }
    }

    // If a session was established, display the Service Menu and get the users input
    while (SerSess == true)
    {
        menuPrint();                         // Display the Menu
        String MenuChoice = menuSerRx(true); // Wait for the selection
        Serial.println();
        Serial.println();

        if (MenuChoice == "1")
        {
            menuClearScreen();
            MavRecOn = !MavRecOn;   // toggle it
        }
        else if (MenuChoice == "2")
        {
            menuClearScreen();
            mavlink_request_datastream();
            menuGoBack(); // display [0] to exit msg and wait
        }
        else if (MenuChoice == "3")
        {
            menuClearScreen();
            mavlink_unrequest_datastream();
            menuGoBack(); // display [0] to exit msg and wait
        }
        else if (MenuChoice == "4")
        {
            // This menu item will run and then immediately return to the main menu.
            menuClearScreen();
            mavlink_request_streaming_params_from_ap();
        }
        else if (MenuChoice == "5")
        {
            // This menu item will run and then immediately return to the main menu.
            menuClearScreen();
            mavlink_unrequest_streaming_params_from_ap();
        }
        else if (MenuChoice == "6")
        {
            // This menu item will run and then immediately return to the main menu.
            menuClearScreen();
            mavlink_set_arm_ap();
        }
        else if (MenuChoice == "7")
        {
            // This menu item will run and then immediately return to the main menu.
            menuClearScreen();
            mavlink_set_disarm_ap();
        }
        else if (MenuChoice == "8")
        {
            menuClearScreen();
            MavRecOn = true;    // when you set a param on the AP, it will respond with a MAVLink msg
                                // to confirm it was actually set.  So in order to see that msg we need
                                // our mavlink_receive() function to be call in loop().
            mavlink_test_set_one_param_on_ap();
            menuExit();     // Immediately exit menu system (because we want to catch the results of the above command)
        }
        else if (MenuChoice == "9")
        {
            menuClearScreen();
            MavRecOn = true;    // in order to see the result of the below command, we need
                                // our mavlink_receive() function to be call in loop().
            mavlink_test_request_one_param_from_ap();
            menuExit();     // Immediately exit menu system (because we want to catch the results of the above command)
        }
        else if (MenuChoice == "a")
        {
            menuClearScreen();
            MavRecOn = true;    // in order to see the result of the below command, we need
                                // our mavlink_receive() function to be call in loop().
            mavlink_set_flightmode_ap(0);   // 0 = MANUAL (see https://github.com/ardupilot/ardupilot/blob/master/Rover/mode.h#L21)
            menuExit();     // Immediately exit menu system (because we want to catch the results of the above command)
        }
        else if (MenuChoice == "b")
        {
            menuClearScreen();
            MavRecOn = true;    // in order to see the result of the below command, we need
                                // our mavlink_receive() function to be call in loop().
            mavlink_set_flightmode_ap(3);   // 3 = STEERING (see https://github.com/ardupilot/ardupilot/blob/master/Rover/mode.h#L21)
            menuExit();     // Immediately exit menu system (because we want to catch the results of the above command)
        }
        else if (MenuChoice == "r")
        {
            menuClearScreen();
            MavRecOn = true;    // in order to see the result of the below command, we need
                                // our mavlink_receive() function to be call in loop().
            mavlink_cmd_preflight_reboot_ap();   
            menuExit();     // Immediately exit menu system (because we want to catch the results of the above command)
        }
        else if (MenuChoice == "x")
        {
            menuExit();
        }
    }
} // END - menuDo()

// Three examples of how to use menus.
// if (MenuChoice == "1")
// {
//     menuClearScreen();
//     //arduinoStatus();
//     menuGoBack();   // when finished, display [0] to exit msg and wait for input.
// }
// else if (MenuChoice == "2")
// {
//     menuClearScreen();
//     //resetProcessor();
//                          // when finished, just return to menu.
// }
// else if (MenuChoice == "3")
// {
//     bool stay = true; // run the while loop at least once.
//     while (stay == true)
//     {
//         menuClearScreen();
//         //showTempHumid();
//         stay = menuGoBackOrRefresh(); // when finished, display [0] to exit or [Enter] to refresh.
//     }
// }

/*
 * Get characters from Serial Port or timeout if nothing entered.
 */
String menuSerRx(boolean Echo)
{
    unsigned long SessStart = millis();
    const byte numChars = MaxSerRx;
    char receivedChars[numChars]; // an array to store the received data
    boolean newData = false;
    static byte ndx = 0;
    char endMarker1 = '\r'; // Some terminal client apps send a CR
    char endMarker2 = '\n'; // Others just a LF character, therefore we need to check for both
    char rc;
    while (newData == false)
    {
        if (millis() - SessStart > SesTimeout)
        {
            SerSess = false;
            menuClearScreen();
            Serial.print("The menu session has timed out (");
            Serial.print(SesTimeout / 1000);
            Serial.println(" seconds)...");
            menuExit();
            break;
        }
        while (Serial.available())
        {
            SessStart = millis();
            rc = Serial.read();
            if (rc == endMarker1 or rc == endMarker2)
            { // if a CR or LF was received
                // Serial.println("CR or LF detected");
                receivedChars[ndx] = '\0'; // terminate the character array (string)...
                ndx = 0;
                newData = true;
                char temp = Serial.read();
                // goto ReturnReceivedString; // return everything
            }
            else if (rc == 127)
            {                     // A DEL character (decimal 127) was received
                ndx = ndx - 1;    // Set the Array-Pointer back to delete the last character
                Serial.print(rc); // Echo del DEL character back that the Terminal Client app
            }                     // removes the last character from the screen
            else
            {
                receivedChars[ndx] = rc; // Receive normal characters...
                ndx++;
                if (ndx >= numChars)
                {
                    ndx = numChars - 1;
                }

                if (Echo == true)
                { // Normal echo
                    Serial.print(rc);
                }
                else
                { // Hide echo, for example if user types in a password...
                    Serial.print("*");
                }
            }
        }
    }
    return receivedChars;
} // END - menuSerRx()

/*
 * Clear terminal screen if supported, else do single println()
 */
void menuClearScreen()
{
    Serial.println();
    // temp commented out as it messes up in the PlatformIO console and I have yet to sort it.
    // Serial.write(0x1B);  // Send - ESC
    // Serial.print("[2J"); // Send - Clear Screen
    // Serial.write(0x1B);  // Send - ESC
    // Serial.print("[H");  // Send - Curser to top left
    // Serial.println(" ");
}

/*
 * Display msg expaining how to go back to main menu and
 * pause until the user does it
 */
void menuGoBack()
{
    Serial.println();
    Serial.println("[x].......go back to Arduino Menu");
    Serial.print("Enter your choice/number: ");
    while (SerSess == true)
    {
        if (menuSerRx(true).length() > 0)
        {
            char temp = Serial.read();
            Serial.println();
            break;
        }
    }
}

/*
 * Display msg expaining how to go back to main Menu or refresh,
 * and pause until the user chooses.
 *
 * Returns
 *      true for Refresh
 *      false for exit
 */
bool menuGoBackOrRefresh()
{
    Serial.println();
    Serial.println("[x].......go back to Arduino Menu");
    Serial.println("[enter]...to refresh");
    Serial.print("Enter your choice/number: ");
    if (menuSerRx(true).length() > 0)
    {
        char temp = Serial.read();
        Serial.println();
        return (false);
    }
    else
    {
        Serial.println();
        Serial.println();
        return (true);
    }
}

/*
 * Exit Menu, resume normal operation
 */
void menuExit()
{
    menuClearScreen();
    Serial.println("Press 'm' to open menu again.");
    Serial.println("loop() operation has resumed...");
    Serial.println();

    SerSess = false; // clear flag that holds us in the menu system.
}

/*
 * Toggle the debugging status
 */
void menuToggleDebug()
{
    menuClearScreen();
    if (DebugNormalOperation == false)
    {
        Serial.println("Debugs ENABLED");
        DebugNormalOperation = true;
        delay(2000);
    }
    else
    {
        Serial.println("Debugs disabled");
        DebugNormalOperation = false;
        delay(2000);
    }
}

// END - SerSrvMnu_fns.cpp