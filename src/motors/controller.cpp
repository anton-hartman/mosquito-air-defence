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

int8_t manual_mode = 1;

void single_step(uint8_t motor, uint8_t direction) {
  const uint8_t steps = 50;
  driver::select_motor(motor);
  driver::turn_motor(direction, steps, STEP_DELAY);
  // utilities::microstep_delay_ms(STEP_DELAY, MICROSTEPS);
  // stop_motor();
}

void turret_control() {
  // Initialize ncurses mode
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);  // Enables arrow key detection
  timeout(10);  // Set a timeout for getch(). If no key is pressed within
                // 100ms, getch() returns ERR

  int ch = getch();  // Get the character pressed

  if (ch == 'h') {
    manual_mode = !manual_mode;
  } else if (ch == ERR && manual_mode) {
    // No key was pressed during the timeout period and we're in manual mode
    driver::stop_all_motors();
  } else if (manual_mode) {
    manual_control(ch);  // Pass the pressed key to manual control
  } else {
    auto_control();
  }

  // End ncurses mode
  endwin();
}

void manual_control(int ch) {
  switch (ch) {
    case KEY_UP:
      // Move motor 1 forward by a predefined step
      single_step(MOTOR1, BACKWARD);
      break;
    case KEY_DOWN:
      // Move motor 1 backward by a predefined step
      single_step(MOTOR1, FORWARD);
      break;
    case KEY_LEFT:
      // Move motor 2 forward by a predefined step
      single_step(MOTOR2, BACKWARD);
      break;
    case KEY_RIGHT:
      // Move motor 2 backward by a predefined step
      single_step(MOTOR2, FORWARD);
      break;
  }
}

uint32_t steps;
void auto_control() {
  if (m1_current_angle != m1_target_angle) {
    driver::select_motor(MOTOR1);
    steps = abs(m1_target_angle - m1_current_angle) / MICROSTEP_ANGLE;
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
    steps = abs(m2_target_angle - m2_current_angle) / MICROSTEP_ANGLE;
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
