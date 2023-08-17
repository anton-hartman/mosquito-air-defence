#ifndef HR8825_CONTROLLER_H
#define HR8825_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

// Constants
// #define FULL_REV 2048
#define FULL_STEP_ANGLE 0.17578125
#define MICROSTEPS 16
#define STEP_ANGLE (FULL_STEP_ANGLE / MICROSTEPS)

// Global Variables
// Inputs
extern int16_t m1_target_angle;  // Target angle of motor 1
extern int16_t m2_target_angle;
extern int16_t m1_actual_angle;  // Actual angle measured by camera detection
extern int16_t m2_actual_angle;
// Internal variables
extern int16_t m1_count_angle;  // Current angle of motor 1 by counting steps
extern int16_t m2_count_angle;
// For testing will be set to either m1_count_angle or m1_actual_angle
extern int16_t m1_current_angle;
extern int16_t m2_current_angle;

extern int8_t manual_mode;

// Function Prototypes
void turret_control();

#endif  // HR8825_CONTROLLER_H
