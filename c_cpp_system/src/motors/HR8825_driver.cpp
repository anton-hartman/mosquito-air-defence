namespace driver {

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "HR8825_driver.h"
#include "controller.h"
#include "utilities/debug.hpp"
#include "utilities/utilities.hpp"

uint8_t init_driver_pins(void) {
  SYSFS_GPIO_Export(M1_ENABLE_PIN);
  SYSFS_GPIO_Export(M1_DIR_PIN);
  SYSFS_GPIO_Export(M1_STEP_PIN);
  SYSFS_GPIO_Write(M1_DIR_PIN, 1);
  SYSFS_GPIO_Direction(M1_ENABLE_PIN, OUT);
  SYSFS_GPIO_Direction(M1_DIR_PIN, OUT);
  SYSFS_GPIO_Direction(M1_STEP_PIN, OUT);
  SYSFS_GPIO_Export(M2_ENABLE_PIN);
  SYSFS_GPIO_Export(M2_DIR_PIN);
  SYSFS_GPIO_Export(M2_STEP_PIN);
  SYSFS_GPIO_Write(M2_DIR_PIN, 1);
  SYSFS_GPIO_Direction(M2_ENABLE_PIN, OUT);
  SYSFS_GPIO_Direction(M2_DIR_PIN, OUT);
  SYSFS_GPIO_Direction(M2_STEP_PIN, OUT);
  return 0;
}

void driver_exit(void) {}

MOTOR Motor;

void select_motor(uint8_t name) {
  Motor.name = name;
  if (name == MOTOR1) {
    Motor.enable_pin = M1_ENABLE_PIN;
    Motor.direction_pin = M1_DIR_PIN;
    Motor.step_pin = M1_STEP_PIN;
  } else if (name == MOTOR2) {
    Motor.enable_pin = M2_ENABLE_PIN;
    Motor.direction_pin = M2_DIR_PIN;
    Motor.step_pin = M2_STEP_PIN;
  } else {
    LOG_DEBUG("please set motor: MOTOR1 or MOTOR2\\r\\n");
  }
}

static void enable_motor(void) {
  SYSFS_GPIO_Write(Motor.enable_pin, 1);
}

void stop_motor(void) {
  SYSFS_GPIO_Write(Motor.enable_pin, 0);
}

void turn_motor(uint8_t direction, uint16_t steps, uint16_t stepdelay) {
  Motor.direction = direction;
  if (direction == FORWARD) {
    // LOG_DEBUG("motor %d formward\\r\\n", Motor.name);
    enable_motor();
    SYSFS_GPIO_Write(Motor.direction_pin, 0);
  } else if (direction == BACKWARD) {
    // LOG_DEBUG("motor %d backmward\\r\\n", Motor.name);
    enable_motor();
    SYSFS_GPIO_Write(Motor.direction_pin, 1);
  } else {
    stop_motor();
  }

  if (steps == 0)
    return;

  uint32_t microsteps = steps * MICROSTEPS;
  //   LOG_DEBUG("Turn %d steps = %d microsteps\\r\\n", steps, microsteps);
  for (uint32_t i = 0; i < steps; i++) {
    SYSFS_GPIO_Write(Motor.step_pin, 1);
    utilities::microstep_delay_ms(stepdelay, MICROSTEPS);
    SYSFS_GPIO_Write(Motor.step_pin, 0);
    utilities::microstep_delay_ms(stepdelay, MICROSTEPS);
  }
}

}  // namespace driver
