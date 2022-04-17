/*
 * SerSrvMnu.h
 * 
 */

#ifndef SERSRVMNU_FNS_H
#define SERSRVMNU_FNS_H

#include <Arduino.h>

/* defines */

/* define any enums */

/* define any struct's */

/* extern global variables */
extern const int MaxSerRx;
extern boolean SerSess;
extern unsigned long SesTimeout;

extern boolean DebugNormalOperation;

/* function pre defines */
void menuDo();
String menuSerRx(boolean Echo);
void menuPrint();
void menuClearScreen();
void menuGoBack();
bool menuGoBackOrRefresh();
void menuExit();
void menuToggleDebug();


#endif
// END - misc_fns.h



