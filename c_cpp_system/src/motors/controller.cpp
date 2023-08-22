namespace stepper {

#include "controller.hpp"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstdint>
#include "HR8825_driver.hpp"
#include "utilities/debug.hpp"
#include "utilities/utilities.hpp"

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

int8_t init_manual_control(void) {
  SYSFS_GPIO_Export(HOME_BTN);
  SYSFS_GPIO_Export(M1_LEFT_BTN);
  SYSFS_GPIO_Export(M1_RIGHT_BTN);
  SYSFS_GPIO_Export(M2_LEFT_BTN);
  SYSFS_GPIO_Export(M2_RIGHT_BTN);

  SYSFS_GPIO_Direction(HOME_BTN, IN);
  SYSFS_GPIO_Direction(M1_LEFT_BTN, IN);
  SYSFS_GPIO_Direction(M1_RIGHT_BTN, IN);

  return 0;
}

int8_t read_pin(uint8_t pin) {
  SYSFS_GPIO_Direction(pin, IN);
  uint8_t pin_val = SYSFS_GPIO_Read(pin);
  //   LOG_DEBUG("Pin %d: %d\n", pin, pin_val);
  SYSFS_GPIO_Direction(pin, OUT);
  SYSFS_GPIO_Write(pin, LOW);
  return pin_val;
}

void single_step(uint8_t motor, uint8_t direction) {
  const uint8_t steps = 1;
  select_motor(MOTOR1);
  turn_motor(BACKWARD, steps, STEP_DELAY);
  // stop_motor();
}

void turret_control() {
  uint8_t home = read_pin(HOME_BTN);
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

void manual_control() {
  if (read_pin(M1_LEFT_BTN)) {
    single_step(MOTOR1, BACKWARD);
  }

  if (read_pin(M1_RIGHT_BTN)) {
    single_step(MOTOR1, FORWARD);
  }

  if (read_pin(M2_LEFT_BTN)) {
    single_step(MOTOR2, BACKWARD);
  }

  if (read_pin(M2_RIGHT_BTN)) {
    single_step(MOTOR2, FORWARD);
  }
}

uint32_t steps;
void auto_control() {
  if (m1_current_angle != m1_target_angle) {
    select_motor(MOTOR1);
    steps = abs(m1_target_angle - m1_current_angle) / FULL_STEP_ANGLE;
    if (m1_current_angle < m1_target_angle) {
      turn_motor(FORWARD, steps, STEP_DELAY);
      stop_motor();
    } else {
      turn_motor(BACKWARD, steps, STEP_DELAY);
      stop_motor();
    }
    // m1_current_angle = m1_target_angle;
  }

  if (m2_current_angle != m2_target_angle) {
    select_motor(MOTOR2);
    steps = abs(m2_target_angle - m2_current_angle) / FULL_STEP_ANGLE;
    if (m2_current_angle < m2_target_angle) {
      turn_motor(FORWARD, steps, STEP_DELAY);
      stop_motor();
    } else {
      turn_motor(BACKWARD, steps, STEP_DELAY);
      stop_motor();
    }
    // m2_current_angle = m2_target_angle;
  }
}

}  // namespace stepper
