#include "utilities.h"

// Global variable to store the current mode
static OperatingMode current_mode = IDLE;

// Function to set the mode
void set_mode(OperatingMode mode) {
    current_mode = mode;
}

// Function to get the current mode
OperatingMode get_mode(void) {
    return current_mode;
}