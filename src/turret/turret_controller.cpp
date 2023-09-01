#include "turret_controller.hpp"
#include <cstdlib>
#include "../utilities/utils.hpp"

namespace turret {

// Global Variables
int16_t m1_target_angle;
int16_t m2_target_angle;
int16_t m1_actual_angle;
int16_t m2_actual_angle;
int16_t m1_count_angle;
int16_t m2_count_angle;

void manual_step(uint8_t motor, uint8_t direction) {
  const uint8_t steps = 50;
  driver::select_motor(motor);
  driver::turn_motor(direction, steps, STEP_DELAY);
}

void manual_control(int ch) {
  switch (ch) {
    case KEY_UP:
      // Move motor 1 forward by a predefined step
      manual_step(MOTOR1, BACKWARD);
      break;
    case KEY_DOWN:
      // Move motor 1 backward by a predefined step
      manual_step(MOTOR1, FORWARD);
      break;
    case KEY_LEFT:
      // Move motor 2 forward by a predefined step
      manual_step(MOTOR2, BACKWARD);
      break;
    case KEY_RIGHT:
      // Move motor 2 backward by a predefined step
      manual_step(MOTOR2, FORWARD);
      break;
    default:
      break;
  }
}

uint32_t steps;
void auto_control(std::pair<int, int> actual_pos,
                  std::pair<int, int> target_pos) {
  m1_actual_angle = actual_pos.first;
  m2_actual_angle = actual_pos.second;
  m1_target_angle = target_pos.first;
  m2_target_angle = target_pos.second;

  if (m1_actual_angle != m1_target_angle) {
    driver::select_motor(MOTOR1);
    steps = abs(m1_target_angle - m1_actual_angle) / MICROSTEP_ANGLE;
    if (m1_actual_angle < m1_target_angle) {
      driver::turn_motor(FORWARD, steps, STEP_DELAY);
      driver::stop_motor();
    } else {
      driver::turn_motor(BACKWARD, steps, STEP_DELAY);
      driver::stop_motor();
    }
    m1_actual_angle = m1_target_angle;
  }

  if (m2_actual_angle != m2_target_angle) {
    driver::select_motor(MOTOR2);
    steps = abs(m2_target_angle - m2_actual_angle) / MICROSTEP_ANGLE;
    if (m2_actual_angle < m2_target_angle) {
      driver::turn_motor(FORWARD, steps, STEP_DELAY);
      driver::stop_motor();
    } else {
      driver::turn_motor(BACKWARD, steps, STEP_DELAY);
      driver::stop_motor();
    }
    m2_actual_angle = m2_target_angle;
  }
}

}  // namespace turret
