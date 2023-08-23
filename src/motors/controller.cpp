#include "controller.hpp"
#include <cstdlib>
#include "../utilities/utilities.hpp"

namespace stepper {

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

void single_step(uint8_t motor, uint8_t direction) {
  const uint8_t steps = 1;
  driver::select_motor(MOTOR1);
  driver::turn_motor(BACKWARD, steps, STEP_DELAY);
  // stop_motor();
}

void turret_control() {
  // uint8_t home = read_pin(HOME_BTN);
  uint8_t home = 0;
  //   LOG_DEBUG("Home: %d\n", home);
  if (home) {
    utilities::delay_ms(1000);
    manual_mode = manual_mode ? 0 : 1;
  }

  if (manual_mode) {
    manual_control();
  } else {
    auto_control();
  }
}

void manual_control() {}

uint32_t steps;
void auto_control() {
  if (m1_current_angle != m1_target_angle) {
    driver::select_motor(MOTOR1);
    steps = abs(m1_target_angle - m1_current_angle) / FULL_STEP_ANGLE;
    if (m1_current_angle < m1_target_angle) {
      driver::turn_motor(FORWARD, steps, STEP_DELAY);
      driver::stop_motor();
    } else {
      driver::turn_motor(BACKWARD, steps, STEP_DELAY);
      driver::stop_motor();
    }
    // m1_current_angle = m1_target_angle;
  }

  if (m2_current_angle != m2_target_angle) {
    driver::select_motor(MOTOR2);
    steps = abs(m2_target_angle - m2_current_angle) / FULL_STEP_ANGLE;
    if (m2_current_angle < m2_target_angle) {
      driver::turn_motor(FORWARD, steps, STEP_DELAY);
      driver::stop_motor();
    } else {
      driver::turn_motor(BACKWARD, steps, STEP_DELAY);
      driver::stop_motor();
    }
    // m2_current_angle = m2_target_angle;
  }
}

}  // namespace stepper
