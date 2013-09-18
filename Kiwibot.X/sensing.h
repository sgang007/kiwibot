/* 
 * File:   sensing.h
 * Author: Srin
 *
 * Created on April 26, 2013, 6:11 PM
 */

#ifndef SENSING_H
#define	SENSING_H
#include "globals.h"
#ifdef	__cplusplus
extern "C" {
#endif


void initSensing();

int ReadPin(int pinNum);
int readSensor(int num);

//gets middle sensor when following path
int getMidSensor(int first,int last);
void getFrontPath(struct path_mark* fp);
void printAngles();

#ifdef	__cplusplus
}
#endif

#endif	/* SENSING_H */

