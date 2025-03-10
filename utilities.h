#ifndef UTILITIES_H
#define UTILITES_H

//define operating modes
typedef enum{
    IDLE, 
    PWM, 
    ITEST,
    HOLD, 
    TRACK
} OperatingMode; 

//function prototypes 
void set_mode(OperatingMode mode);
OperatingMode get_mode(void);

#endif