#pragma once

#include <ncurses.h>
#include <cstdint>
#include <utility>
#include "HR8825_driver.hpp"

namespace stepper {

// Constants
// #define FULL_REV 2048
#define FULL_STEP_ANGLE 0.17578125
#define MICROSTEPS 16
#define MICROSTEP_ANGLE (FULL_STEP_ANGLE / MICROSTEPS)
#define STEP_DELAY 3
#define MIRCOSTEP_DELAY (STEP_DELAY / MICROSTEPS)

// Global Variables
// Inputs
extern int16_t m1_target_angle;  // Target angle of motor 1
extern int16_t m2_target_angle;
extern int16_t m1_actual_angle;  // Actual angle measured by camera detection
extern int16_t m2_actual_angle;
extern int16_t m1_count_angle;  // Current angle of motor 1 by counting steps
extern int16_t m2_count_angle;

extern int8_t manual_mode;

// Function Prototypes
void single_step(uint8_t motor, uint8_t direction);
void turret_control(std::pair<int, int> actual_pos,
                    std::pair<int, int> target_pos);
void manual_control(int ch);
void auto_control(void);

}  // namespace stepper
