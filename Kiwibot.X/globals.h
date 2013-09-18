/* 
 * File:   globals.h
 * Author: Srin
 *
 * Created on April 26, 2013, 6:12 PM
 */
#ifndef GLOBALS_H
#define	GLOBALS_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <plib.h>
#include <math.h>
#include "NU32.h"

#define VOLTS_PER_SAMPLE (3.3/1024)
#define CORE_TICK_TIME 25    // nanoseconds between core ticks
#define SAMPLE_TIME 1000       // 10 core timer ticks = 250 ns
#define DELAY_TICKS 20000000 // delay 1/2 sec, 20 M core ticks, between messages
#define MAX_MSG_LGTH 100   // Set maximum message length
#define SENSOR_COUNT 16
#define CORE_TICKS_PER_SECOND 80000000
#define CTPS_TIMES_10 800000000
struct path_mark {
   int start;
   int end;
   int len;
};

struct movement_data{
    float speeds[3];
    float ref_speeds[3];
    int curr_state[3];
    int prev_state[3];
    int counts[3];
    int prev_10_time[3];
    int prev_10_counts[3];
    int curr_10_time[3];
    int dir[3];
    float motor_values[3];
    float motor_targets[3];
    float prev_motor_targets[3];
};
struct target{
    float desired_components[3];
    float target_degs;
    float prev_target_degs;
    float target_rads;
};

void waitFor(int ticks);
#ifdef	__cplusplus
}
#endif

#endif	/* GLOBALS_H */

