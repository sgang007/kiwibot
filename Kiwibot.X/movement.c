#include "movement.h"
//get angle and power motors in a way that it moves at that angle
char move_msg[100];

//Variables used throughout movement functions
float angles[SENSOR_COUNT];
float rads[SENSOR_COUNT];
int const MEASURE_TIME = 200000;
//constant matrix assigned at runtime used to calculate desired components
float movement_matrix[3][3];
//angle of the three motors, constants chosen dependant on reference point
float motor_angles[3];
//desired vx, vy, rw

void moveAt(struct target* t,struct movement_data *ms) {
    //desired x speed, yspeed, rotation

    calcVectors(t,ms);
        sprintf(move_msg, "desired components %f %f %f \n", t->desired_components[0],
                t->desired_components[1], t->desired_components[2]);

    NU32_WriteUART1(move_msg);
      sprintf(move_msg, "motor values %f %f %f\n", ms->motor_values[0],
                ms->motor_values[1], ms->motor_values[2]);
    NU32_WriteUART1(move_msg);
    

    //get mark or angle? decide between getting angle from mark here or in sensing
    //then use velocity information from encoders to figure what the motors need
    //to be set to to move at that angle
}
void initAngles(){
    int i;
    for (i = 0; i < SENSOR_COUNT; i++) {
        angles[i] = 236.25 - 22.5 * i;
        if (angles[i] < 0) {
            angles[i] += 360;
        }
}
}
void initMovement() {
    //set up angles
    int i;
    float circ = 2 * 6.28318530718;
    for (i = 0; i < SENSOR_COUNT; i++) {
        angles[i] = 236.25 - 22.5 * i;
        if (angles[i] < 0) {
            angles[i] += 360;
        }
        rads[i] = deg2Rads(angles[i]);
        sprintf(move_msg, "angle at sensor %d is %f\n",i,angles[i]);
        NU32_WriteUART1(move_msg);
        rads[i] = ((float) circ) / ((float) SENSOR_COUNT) * i;
    }
    motor_angles[0] = deg2Rads(90);
    motor_angles[1] = deg2Rads(330);
    motor_angles[2] = deg2Rads(210);
    initMotors();

}

void setMotor(int motor_num, float speed) {
    switch (motor_num) {
        case 0:
            if (speed < 0) {
                LATGbits.LATG12 = 0;
                OC1RS = 1999 * -1 * speed;
            }else{
                LATGbits.LATG12 = 1;
                OC1RS = 1999 - 1999 * speed;
            }
            break;
        case 1:
            if (speed < 0) {
                LATGbits.LATG13 = 0;
                OC2RS = 1999 * -1* speed;
            }else{
                LATGbits.LATG13 = 1;
                OC2RS = 1999 - 1999 * speed;
            }
            break;
        case 2:
            if (speed < 0) {
                LATGbits.LATG14 = 0;
                OC3RS = 1999 * -1 * speed;
            }else{
                LATGbits.LATG14 = 1;
                OC3RS = 1999 - 1999 * speed;
            }
            break;


    }
}

void setMotor1(float speed, int dir) {
    if (dir == 0) {
        LATGbits.LATG12 = 0;
        OC1RS = 1999 * speed;
    }
    if (dir == 1) {
        LATGbits.LATG12 = 1;
        OC1RS = 1999 - 1999 * speed;
    }
}

void setMotor2(float speed, int dir) {
    if (dir == 0) {
        LATGbits.LATG13 = 0;
        OC2RS = 1999 * speed;
    }

    if (dir == 1) {
        LATGbits.LATG13 = 1;
        OC2RS = 1999 - 1999 * speed;
    }
}

void setMotor3(float speed, int dir) {
    if (dir == 0) {
        LATGbits.LATG14 = 0;
        OC3RS = 1999 * speed;
    }

    if (dir == 1) {
        LATGbits.LATG14 = 1;
        OC3RS = 1999 - 1999 * speed;
    }
}

void initMotors() {
    // DIR for motors 1 - 3
    TRISGbits.TRISG12 = 0;
    TRISGbits.TRISG13 = 0;
    TRISGbits.TRISG14 = 0;

    //encoder lines A,B for motors 1-2
    TRISEbits.TRISE0 = 1;
    TRISEbits.TRISE1 = 1;
    TRISEbits.TRISE2 = 1;
    TRISEbits.TRISE3 = 1;
    TRISEbits.TRISE4 = 1;
    TRISEbits.TRISE5 = 1;

    // use TIMER2 for the PWM
    T2CONbits.TCKPS = 2; // Timer2 prescaler N=4 (1:4)
    PR2 = 1999; // period = (PR2+1) * N * 12.5 ns = 100 us, 10 kHz
    TMR2 = 0; // initial TMR2 count is 0

    // use OC5 (on D4)
    OC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
    OC1RS = 500; // duty cycle = OC1RS/(PR2+1) = 25%
    OC1R = 500; // initialize before turning OC1 on; then it is read-only

    OC2CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
    OC2RS = 500; // duty cycle = OC1RS/(PR2+1) = 25%
    OC2R = 500; // initialize before turning OC1 on; then it is read-only

    OC3CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
    OC3RS = 500; // duty cycle = OC1RS/(PR2+1) = 25%
    OC3R = 500; // initialize before turning OC1 on; then it is read-only

    // turn them on
    T2CONbits.ON = 1; // turn on Timer2
    OC1CONbits.ON = 1; // turn on OC1
    OC2CONbits.ON = 1; // turn on OC2
    OC3CONbits.ON = 1; // turn on OC3

    // initially off
    OC1RS = 0;
    OC2RS = 0;
    OC3RS = 0;
    LATGbits.LATG12 = 1;
    LATGbits.LATG13 = 1;
    LATGbits.LATG14 = 1;
}

void calcVectors(struct target* t, struct movement_data* ms) {
    int i;
    int j;
    t->desired_components[0] = cosf(t->target_rads);
    t->desired_components[1] = sinf(t->target_rads);
    t->desired_components[2] = 0;
    ms->motor_targets[0] = -.666666f * t->desired_components[0];
    ms->motor_targets[1] = .33333f * t->desired_components[0] + .57735f * t->desired_components[1];
    ms->motor_targets[2] = -.33333f * t->desired_components[0]+ .57735f * t->desired_components[1];
}

void setMotors(struct movement_data* ms) {
    setMotor(0,ms->motor_values[0]);
    setMotor(1,ms->motor_values[1]);
    setMotor(2,ms->motor_values[2]);
}

float deg2Rads(float angle) {
    return angle * 3.14159 / 180;
}

void getAngleOfPath(struct path_mark* f, struct target* t) {
    int s = f->start;
    int e = f->end;
    t->target_degs = (angles[s] + angles[e]) / 2;
    sprintf(move_msg, "from %d to %d with %f %f is an angle of %f\n",f->start,f->end,angles[s],angles[e],t->target_degs);
    NU32_WriteUART1(move_msg);
}
//map each speed value to a duty cycle
//but, the the wheel with the highest speed in a given command uses this set value
//the other two are only set to this initially, then are adjusted with pid
//to maintain the speed ratio to the fastest wheel
void measureSpeeds(struct movement_data* ms) {
    int i = 0;
    int count_diff[3];
    for(i = 0;i < 3;i++){
        count_diff[i] = ms->counts[i];
    }
    waitFor(MEASURE_TIME);
    //since we are measuring the same for all, and we only need relative speeds,
    //do we need to do any dividing for speed?
    for(i = 0;i < 3;i++){
        count_diff[i] = ms->counts[i] - count_diff[i];
        ms->speeds[i] = count_diff[i];
    }
    
}
/*
void PID_once(struct movement_data* ms){
    float ratios[3] = {0,0,0};
    float max_wheel_speed = 0;
    float err[3] = {0,0,0};
    float prev_err[3] = {0,0,0};
    int i;
    int max = 0;

    for(i = 0;i < 3;i++){
       if(fabs(ms->motor_values[i])>fabs(ms->motor_values[max])){
           max = i;
       }
    }
    max_wheel_speed = ms->motor_values[max];
    if(max_wheel_speed<0){
        upscaled_vector = -upscaled_speed;
    }else{
        upscaled_vector = upscaled_speed;
    }
    ratios[max] = 1;
    for(i = 0;i < 3;i++){
        if(i!=max) ratios[i] = ms->motor_values[i]/max_wheel_speed;
        ms->motor_values[i] = ratios[i] * upscaled_vector;
    }

    for(i = 0;i < 3;i++){
        setMotor(i,ms->motor_values[i]);
    }
    for(i = 0;i < 3;i++){
        //goals are set to a ratio of the fastest
        if(i!=max){
            ms->ref_speeds[i] = ms->speeds[max]*ratios[i];
            err[i] = ms->ref_speeds[i] - ms->speeds[i];
            //previous motor value gets a bump if not fast enough, or slowed if it is
            //next iteration uses this value, if the new speed was good enough, then it is maintained since the err term will be zero
            ms->motor_values[i] = ms->motor_values[i] + err[i]*kp + (err[i]-prev_err[i])*ki;
            prev_err[i] = err[i];
        }
        setMotor(i,ms->motor_values[i]);
    }
}
void PID(struct movement_data* ms) {
    float ratios[3] = {0,0,0};
    float max_wheel_speed = 0;
    float err[3] = {0,0,0};
    float prev_err[3] = {0,0,0};
    int i;
    int max = 0;
    
    for(i = 0;i < 3;i++){
       if(fabs(ms->motor_values[i])>fabs(ms->motor_values[max])){
           max = i;
       }
    }
    //either set ratios as ratio of fastest wheel, so it is always 1 and
    //the it's speed is upscaled to the max value and sign
    //or all three ratios are a ratio of the upscale target so the ratio[max]
    //handles the sign
    max_wheel_speed = ms->motor_values[max];
    if(max_wheel_speed<0){
        upscaled_vector = -upscaled_speed;
    }else{
        upscaled_vector = upscaled_speed;
    }
    ratios[max] = 1;
    for(i = 0;i < 3;i++){
        if(i!=max) ratios[i] = ms->motor_values[i]/max_wheel_speed;
        ms->motor_values[i] = ratios[i] * upscaled_vector;
    }

    //essentially sets all motors to their motor values, but written this way since PID will use this way later
    for(i = 0;i < 3;i++){
        setMotor(i,ms->motor_values[i]);
    }
    sprintf(move_msg,"ratios: %f %f %f\n",ratios[0],ratios[1],ratios[2]);
    NU32_WriteUART1(move_msg);
    //wait a sec before measuring?
    waitFor(1000000);
    while(1){
        //now that we have ratios, main PID loop
        //measure speeds, if other two wheels are not ratio of fastest wheel, correct
        measureSpeeds(ms);
        //waitFor(1000000);
        for(i = 0;i < 3;i++){
            //goals are set to a ratio of the fastest
            if(i!=max){
                ms->ref_speeds[i] = ms->speeds[max]*ratios[i];
                err[i] = ms->ref_speeds[i] - ms->speeds[i];
                //previous motor value gets a bump if not fast enough, or slowed if it is
                //next iteration uses this value, if the new speed was good enough, then it is maintained since the err term will be zero
                ms->motor_values[i] = ms->motor_values[i] + err[i]*kp + (err[i]-prev_err[i])*ki;
                prev_err[i] = err[i];
            }
            setMotor(i,ms->motor_values[i]);
        }
        sprintf(move_msg, "motor values %f %f %f and refs %f %f %f\n so set to: %f %f %f \n", ms->speeds[0],
                ms->speeds[1],ms->speeds[2],ms->ref_speeds[0],ms->ref_speeds[1],
                ms->ref_speeds[2],ms->motor_values[0],ms->motor_values[1],ms->motor_values[2]);
    NU32_WriteUART1(move_msg);

    }
}
 */
void stop(){
    setMotor1(0,0);
    setMotor2(0,0);
    setMotor3(0,0);
}