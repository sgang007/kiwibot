/* 
 * File:   movement.h
 * Author: Srin
 *
 * Created on April 26, 2013, 6:12 PM
 */
#ifndef MOVEMENT_H
#define	MOVEMENT_H
#ifdef	__cplusplus
extern "C" {
#endif
#include "globals.h"


//functions in movement.h
void moveAt(struct target* t,struct movement_data*ms);
float deg2Rads(float angle);
void initMovement();
void calcVectors(struct target*t, struct movement_data* ms);
void getAngleOfPath(struct path_mark* f, struct target * t);
void initMotors();
void setMotors();
void setMotor(int motor_num, float speed);
void setMotor1(float speed, int direction);
void setMotor2(float speed, int direction);
void setMotor3(float speed, int direction);
void stop();
void PID(struct movement_data*ms);
#ifdef	__cplusplus
}
#endif

#endif	/* MOVEMENT_H */

