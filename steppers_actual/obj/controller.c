#include "controller.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include "HR8825_driver.h"
#include "homing.h"

// float 	     4 byte 	1.2E-38 to 3.4E+38 	    6 decimal places
// double 	     8 byte 	2.3E-308 to 1.7E+308 	15 decimal places
// long double 	 10 byte 	3.4E-4932 to 1.1E+4932 	19 decimal places

// Global Variables
// Inputs
int16_t m1_target_angle;
int16_t m2_target_angle;
int16_t m1_actual_angle;
int16_t m2_actual_angle;

// Internal variables
int16_t m1_count_angle;
int16_t m2_count_angle;

// For testing
int16_t m1_current_angle;
int16_t m2_current_angle;

int8_t manual_mode = 0;
init_homing();

uint32_t steps;
void turret_control() {
  if (SYSFS_GPIO_Read(HOME_BUTTON)) {
    manual_mode = manual_mode ? 0 : 1;
    // if (manual_mode) {
    //   manual_mode = 0;
    // } else {
    //   manual_mode = 1;
    // }
    printf("Manual mode: %d\n", manual_mode);
  }

  if (!manual_mode) {
    if (m1_current_angle != m1_target_angle) {
      select_motor(MOTOR1);
      steps = abs(m1_target_angle - m1_current_angle) / STEP_ANGLE;
      if (m1_current_angle < m1_target_angle) {
        turn_motor(FORWARD, steps, 3);
        stop_motor();
      } else {
        turn_motor(BACKWARD, steps, 3);
        stop_motor();
      }
      // m1_current_angle = m1_target_angle;
    }

    if (m2_current_angle != m2_target_angle) {
      select_motor(MOTOR2);
      steps = abs(m2_target_angle - m2_current_angle) / STEP_ANGLE;
      if (m2_current_angle < m2_target_angle) {
        turn_motor(FORWARD, steps, 3);
        stop_motor();
      } else {
        turn_motor(BACKWARD, steps, 3);
        stop_motor();
      }
      // m2_current_angle = m2_target_angle;
    }
  } else {
  }
}
