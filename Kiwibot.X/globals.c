#include "globals.h"
void waitFor(int time) {
    //time in microseconds
    //40 ticks is a microsecond, I think?
    //NU32_WriteUART1("Waiting\n");
    int finishtime = ReadCoreTimer() + time * 25;
    while (ReadCoreTimer() < finishtime);
}