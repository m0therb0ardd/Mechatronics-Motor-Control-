#ifndef UTILITIES_H
#define UTILITIES_H

//define operating modes
typedef enum {

    IDLE, 
    PWM, 
    ITEST,
    HOLD, 
    TRACK
} OperatingMode; 

//function prototypes 
void set_mode(OperatingMode mode);
OperatingMode get_mode(void);

// void set_mode(enum mode_t md);
// enum mode_t get_mode(void);
#endif


// //PUSHKAR
// #ifndef UTILITIES__H__
// #define UTILITIES__H__

// enum mode_t {
//     IDLE, 
//     PWM, 
//     ITEST, 
//     HOLD, 
//     TRACK};

// void set_mode(enum mode_t md);
// enum mode_t get_mode(void);

// #endif