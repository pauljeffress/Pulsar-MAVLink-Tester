/*
 * debug_fns.h
 * 
 * Header for debug_fns.cpp
 * 
 */

#ifndef DEBUG_FNS_H
#define DEBUG_FNS_H


#include <arduino.h>

/* extern my global vars */
extern bool _printDebug;
extern Stream *_debugSerial;

/* function pre defines */
void enableDebugging(Stream &debugPort);
void disableDebugging(void);
void debugPrint(const char *message);
void debugPrintln(const char *message);
void debugPrintInt(int32_t number);
void debugPrintlnInt(int32_t number);
void debugPrintFlt(float number);
void debugPrintlnFlt(float number);

#endif