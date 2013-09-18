#include "sensing.h"

//Variables used throughout sensing functions
int marks[SENSOR_COUNT * 2];
int readings[SENSOR_COUNT];
int thresholds[SENSOR_COUNT] ={800, //0
    700, //1
    860, //2
    770, //3
    750, //4
    880, //5
    800, //6
    700, //7
    700, //8
    800, //9
    725, //10
    825, //11
    825, //12
    675, //13
    860, //14
    875, //15
};


const int threshold_value = 800;
char msg[100];
//on white paper
//0: 658
//1: 497
//2: 792
//3: 684
//4: 551
//5: 819
//6: 648
//7: 499
//8: 460
//9: 652
//10: 447
//11: 699
//12: 765
//13: 547
//14: 675
//15: 828

//on black tape
//0: 929
//1: 824
//2: 955
//3: 867
//4: 900
//5: 955
//6: 931
//7: 800
//8: 897
//9: 932
//10: 875
//11: 947
//12: 924
//13: 767
//14: 1016
//15: 938

void initSensing() {
    //6.8k ohm resistor was best
    int i = 0;
    for (i = 0; i < SENSOR_COUNT; i++) {
        marks[i] = -1;
        thresholds[i] =300;
    }
    //setup an input
    TRISBSET = 0xFFFF;
    AD1PCFG = 0x07FF; // bits 11, 12, 13, 14 and 15 are 0, so AN14 and AN15 are AN inputs
    AD1CON3bits.ADCS = 3; // ADC clock period is Tad = 2 * ADCS * Tpb
    AD1CON1bits.ADON = 1; // turn on A/D converter
    //select phot
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
    TRISAbits.TRISA2 = 0;
    TRISAbits.TRISA3 = 0;

}

int readSensor(int num) {
    //select mux output
    LATACLR = 0xFFFF;
    LATASET = ~(15 - num);
    //read analog input
    int i;
    int elapsed = ReadCoreTimer();
    int finishtime = elapsed + 1000;
    while (ReadCoreTimer() < finishtime); // sample for more than 200 ns
    for (i = 0; i < 3; i++) {
        AD1CHSbits.CH0SA = 15;
        AD1CON1bits.SAMP = 1;
        int elapsed = ReadCoreTimer();
        int finishtime = elapsed + SAMPLE_TIME;
        while (ReadCoreTimer() < finishtime); // sample for more than 200 ns
        AD1CON1bits.SAMP = 0; // stop sampling and start converting
        while (!AD1CON1bits.DONE); // wait for the conversion process to finish
    }
    return (ADC1BUF0);
}

void getFrontPath(struct path_mark* fp) {
    int i = 0;
    int on_count = 0;
    int max_val = 0;
    int max_phot = 0;
    //starts at 0 (front) and goes clockwise
    //marks all the starts of paths
    for (i = 0; i < SENSOR_COUNT; i++) {
        readings[i] = readSensor(i);
        sprintf(msg, " %d  %d\n", i, readings[i]);
        NU32_WriteUART1(msg);
        if (readings[i] > max_val) {
            max_val = readings[i];
            max_phot = i;
        }
    }
    //if nothing found last time, get min distance from straight ahead
    if (fp->len == 0) {
        fp->start = 7;
        fp->end = 7;
        fp->len = 1;
    }
    for (i = 0; i < SENSOR_COUNT; i++) {
        if (readings[i] > thresholds[i]) {
            marks[on_count] = i;
            //sprintf(msg, "added %d marks %d\n", on_count, marks[on_count]);
            //NU32_WriteUART1(msg);
            on_count++;
            while ((readings[i] > thresholds[i]) && (i < SENSOR_COUNT)) {
                //sprintf(msg, "read pin %d and got value %d\n", i, readings[i]);
                //NU32_WriteUART1(msg);
                i++;
            }
            i--;
            marks[on_count] = i;
            //sprintf(msg, "closed %d to marks %d \n", on_count, marks[on_count]);
            //NU32_WriteUART1(msg);
            on_count++;
        }
    }
    if (on_count == 0) {
        sprintf(msg, "nothing over threshold, but %d had largest with %d\n", max_val,max_phot);
        NU32_WriteUART1(msg);
        fp->start = max_phot;
        fp->end = max_phot;
        fp->len = -1;
        return;
    }
    for (i = 0; i < on_count; i++) {
        //sprintf(msg, "marks %d\n", marks[i]);
        //NU32_WriteUART1(msg);
    }
    //purs a -1 in the next index to show how many paths found when we iterate through this
    if (on_count < SENSOR_COUNT * 2) marks[on_count] = -1;
    i = 0;
    int min_dist = 1000;
    int min_i = 0;
    int min_size = 0;
    int temp_dist = 0;
    int temp_size = 0;
    //check all marked paths for closest distance to previous front
    //get difference between midpoint of previous mark on circle to all found marks
    //lowest difference is good!
    while ((marks[i] != -1) && (i < SENSOR_COUNT * 2)) {
        temp_size = abs(marks[i] - marks[i+1]) + 1;
            temp_dist = abs((marks[i] + marks[i + 1]) / 2 - (fp->start + fp->end) / 2);
            if (temp_dist > 8) temp_dist = 15 - temp_dist;
            sprintf(msg, "from %d to %d\n",marks[i],marks[i+1]);
            NU32_WriteUART1(msg);
            sprintf(msg, "dist: %d\n",temp_dist);
            NU32_WriteUART1(msg);
            if (temp_dist < min_dist) {
                min_dist = temp_dist;
                min_i = i;
                min_size = temp_size;
            }
        i += 2;
    }
    //sprintf(msg, "%d to %d min dist\n", marks[min_i], marks[min_i + 1]);
    //NU32_WriteUART1(msg);
    fp->len = abs(marks[min_i] - marks[min_i+1]) + 1;
    sprintf(msg, "from %d to %d\n",marks[min_i],marks[min_i+1]);
    NU32_WriteUART1(msg);
    int lwrap = 0,rwrap = 0,wrap =0;
    int lwrapval = 0, rwrapval = 0,wrapval=0;
    temp_dist = abs(marks[min_i]-fp->start);
    if (temp_dist > 8) {
        temp_dist = 15 - temp_dist;
        lwrap = 1;
        lwrapval = ((marks[min_i]+fp->start+16)/2)%15;
    }
    temp_size = abs(marks[min_i+1]-fp->start);
    if (temp_size > 8) {
        temp_size = 15 - temp_size;
        rwrap = 1;
        rwrapval = ((marks[min_i+1]+fp->start+16)/2)%15;
    }
    if(temp_dist<temp_size){
        marks[min_i+1] = marks[min_i];
        wrap = lwrap;
        wrapval = lwrapval;
    }else{
        marks[min_i] = marks[min_i+1];
        wrap = rwrap;
        wrapval = lwrapval;
    }
    if(wrap){
        marks[min_i] = wrapval;
    marks[min_i+1] = wrapval;
    }else{
    marks[min_i] = (marks[min_i] + fp->start)/2;
    marks[min_i+1] = (marks[min_i+1] + fp->start)/2;
    }
    fp->start = marks[min_i];
    fp->end = marks[min_i + 1];
    //fp->len = abs(fp->start - fp->end) + 1;
}
