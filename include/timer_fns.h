/*
 * timer_fns.h
 * 
 * Header for timer_fns.cpp
 * 
 */
#ifndef TIMER_FNS_H
#define TIMER_FNS_H

#include <arduino.h>


/* extern my global vars */
extern volatile uint32_t mytimercounter;
extern uint32_t mytimercounter_last;

extern unsigned long oneSecCounter;
extern unsigned long oneSecCounter_last;

extern volatile int loop_step;
extern int assess_step;
extern uint32_t assess_iterations_counter;
extern uint32_t assess_iterations_counter_last;

extern long iterationCounter;

extern volatile unsigned long seconds_since_reset_or_powercycle;
extern volatile unsigned long seconds_since_last_wake;
extern volatile unsigned long seconds_since_last_CAN_tx;
extern volatile unsigned long seconds_since_last_ap_tx;
extern volatile unsigned long seconds_since_last_ap_rx;
extern volatile unsigned long seconds_since_last_agt_tx;
extern volatile unsigned long seconds_since_last_agt_rx;
extern volatile unsigned long seconds_since_last_fmx_tx;
extern volatile unsigned long seconds_since_last_fmx_rx;
extern volatile unsigned long seconds_since_last_sensors_read;
extern volatile unsigned long seconds_since_last_page_update;
extern volatile unsigned long seconds_since_last_page_change;
extern volatile unsigned long seconds_since_last_mavlink_heartbeat_tx;
extern volatile unsigned long seconds_since_last_mavlink_heartbeat_rx;
extern volatile unsigned long seconds_since_last_mavlink_req;
extern volatile unsigned long seconds_since_last_gps_read;
extern volatile unsigned long seconds_since_last_iridium_tx;
extern volatile unsigned long seconds_since_last_iridium_rx;
extern volatile unsigned long seconds_since_last_alarmtx;



/* function pre defines */
uint32_t daysFromSeconds(uint32_t s);
uint32_t hoursFromSeconds(uint32_t s);
uint32_t minutesFromSeconds(uint32_t s);
uint32_t secondsFromSeconds(uint32_t s);
uint32_t seconds();
void timerSetup(void);
void timerIncrementer(void);

#endif
