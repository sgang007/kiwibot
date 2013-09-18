#include "globals.h"
#include "sensing.h"
#include "movement.h"
//.5
//.01
//.005
void doStuff();
struct path_mark FP;
struct target T = {.prev_target_degs = 270,.target_degs = 270,.desired_components = {0,0,0}};
char dbg_message[100];
struct movement_data MS = {.speeds =
    {0, 0, 0}, .ref_speeds =
    {0, 0, 0}, .curr_state =
    {0, 0, 0}, .prev_state =
    {0, 0, 0}, .counts =
    {0, 0, 0}, .prev_10_counts =
    {0, 0, 0}, .prev_10_time =
    {0, 0, 0}, .curr_10_time =
    {0, 0, 0}, .motor_values =
    {0, 0, 0}, .motor_targets =
    {0, 0, 0}, .prev_motor_targets =
    {0, 0, 0}, .dir =
    {CTPS_TIMES_10, CTPS_TIMES_10, CTPS_TIMES_10}};
int encoder_states[2][2] = {
    {0, 3},
    {1, 2}
};
int curr_state;
int prev_state;
int temp_time;
const float upscaled_speed = .5;
float upscaled_vector = .5;
const float ref_speed = 30;
float ref_vec = 30;
const float kp = 0.01;
const float ki = 0.005;
const float kd;

void __ISR(_TIMER_1_VECTOR, IPL7AUTO)Timer1Handler(void) {
    MS.curr_state[0] = encoder_states[PORTEbits.RE0][PORTEbits.RE1];
    MS.curr_state[1] = encoder_states[PORTEbits.RE2][PORTEbits.RE3];
    MS.curr_state[2] = encoder_states[PORTEbits.RE5][PORTEbits.RE4];
    int i;
    for (i = 0; i < 3; i++) {
        if (MS.curr_state[i] > MS.prev_state[i]) {
            if (MS.dir[i] == -CTPS_TIMES_10) {
                //MS.counts[i] = 10 - (MS.counts[i] % 10);
                MS.dir[i] == CTPS_TIMES_10;
            }
            MS.counts[i]++;
        } else if (MS.curr_state[i] < MS.prev_state[i]) {
            if (MS.dir[i] == CTPS_TIMES_10) {
                //MS.counts[i] = 10 - MS.counts[i] % 10;
                MS.dir[i] == -CTPS_TIMES_10;
            }
            MS.counts[i]--;
        }
        MS.prev_state[i] = MS.curr_state[i];
        if ((MS.counts[i] % 10 == 0) && (MS.counts != MS.prev_10_counts)) {
            MS.prev_10_counts[i] = MS.counts[i];
            MS.curr_10_time[i] = ReadCoreTimer();
            if (MS.curr_10_time[i] < MS.prev_10_time[i]) {
                //TODO
            }
            //10 encoder counts, divided by change in core ticks,
            //times number of coreticks in a second (80 mil)
            //since 10 and 80 mil constant, change dir
            //MS.speeds[i] = MS.dir[i] / (MS.curr_10_time[i] - MS.prev_10_time[i]);
            MS.prev_10_time[i] = MS.curr_10_time[i];
        }
    }
    IFS0bits.T1IF = 0;
}

void main(void) {
    NU32_Startup();
    //read sensors
    //get angle of movement from sensor marks
    //keep up encoder counts, velocity vectors of the three motors
    //incorporate velocity information and use angle heading to set motor vectors
    // cache on, min flash wait, interrupts on, LED/button init, UART init
    T1CON = 0x0; // Stop timer and clear control register,
    // prescaler at 1:1,internal clock source
    TMR1 = 0x0; // Clear timer register
    PR1 = 100000; // Load period register
    IPC1bits.T1IP = 7;
    IPC1bits.T1IS = 3;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    T1CONSET = 0x8000; // Start timer
    //NU32_WriteUART1("Initialized\n");
    initMovement();
    //NU32_WriteUART1("Movement initialized\n");
    initSensing();
    //NU32_WriteUART1("Sensing initialized\n");
    int motor_choice, dir;
    float spd;
    stop();
    doStuff();
}

void doStuff() {
    float ratios[3] = {0, 0, 0};
    float max_wheel_speed = 0;
    float err[3] = {0, 0, 0};
    float prev_err[3] = {0, 0, 0};
    int i;
    int change = 0;
    int max = 0;

    while (1) {
        initAngles();
        getFrontPath(&FP);
        if(FP.len==-1){
            NU32_WriteUART1("lost\n");
            if(T.prev_target_degs < 180){
                T.target_degs = T.prev_target_degs + 180;
            }else{
                T.target_degs = T.prev_target_degs - 180;
            }
            
        }else{
            T.prev_target_degs = T.target_degs;
            getAngleOfPath(&FP,&T);
        }

        sprintf(dbg_message,"target degrees: %f\n",T.target_degs);
        NU32_WriteUART1(dbg_message);
        T.target_rads = deg2Rads(T.target_degs);

        calcVectors(&T, &MS);
        change = 0;
        for (i = 0; i < 3; i++) {
            if (MS.prev_motor_targets[i] != MS.motor_targets[i]) {
                change = 1;
                break;
            }
        }
        for (i = 0; i < 3; i++) {
            MS.prev_motor_targets[i] = MS.motor_targets[i];
        }
        //only have to recalculate ratios and shit if motor targets are different
        if (change) {
            for (i = 0; i < 3; i++) {
                if (fabs(MS.motor_targets[i]) > fabs(MS.motor_targets[max])) {
                    max = i;
                }
            }

            max_wheel_speed = MS.motor_targets[max];
            if (max_wheel_speed < 0) {
                upscaled_vector = -upscaled_speed;
            } else {
                upscaled_vector = upscaled_speed;
            }
            ratios[max] = 1;
            for (i = 0; i < 3; i++) {
                if (i != max) ratios[i] = MS.motor_targets[i] / max_wheel_speed;
                MS.motor_values[i] = ratios[i] * upscaled_vector;
            }

            for (i = 0; i < 3; i++) {
                setMotor(i, MS.motor_values[i]);
            }
        }
        measureSpeeds(&MS);
        for (i = 0; i < 3; i++) {
            //goals are set to a ratio of the fastest
            if (i != max) {
                MS.ref_speeds[i] = MS.speeds[max] * ratios[i];
                err[i] = MS.ref_speeds[i] - MS.speeds[i];
                //previous motor value gets a bump if not fast enough, or slowed if it is
                //next iteration uses this value, if the new speed was good enough, then it is maintained since the err term will be zero
                MS.motor_values[i] = MS.motor_values[i] + err[i] * kp + (err[i] - prev_err[i]) * ki;
                prev_err[i] = err[i];
            }
            setMotor(i, MS.motor_values[i]);
        }
    }

}