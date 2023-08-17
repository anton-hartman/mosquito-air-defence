#include <stdio.h>   //printf()
#include <stdlib.h>  //exit()
#include <string.h>  //strcmp()

#include "Debug.h"  //DEBUG()
#include "HR8825_driver.h"
#include "controller.h"

/**
 * Initialises all the pins used by the driver.
 *
 * Example:
 * if(init_pins())
 *   exit(0);
 */
uint8_t init_pins(void) {
  SYSFS_GPIO_Export(M1_ENABLE_PIN);
  SYSFS_GPIO_Export(M1_DIR_PIN);
  SYSFS_GPIO_Export(M1_STEP_PIN);
  // SYSFS_GPIO_Export(M1_M0_PIN);
  // SYSFS_GPIO_Export(M1_M1_PIN);
  // SYSFS_GPIO_Export(M1_M2_PIN);

  SYSFS_GPIO_Write(M2_DIR_PIN, 1);
  SYSFS_GPIO_Write(M1_DIR_PIN, 1);

  SYSFS_GPIO_Direction(M1_ENABLE_PIN, OUT);
  SYSFS_GPIO_Direction(M1_DIR_PIN, OUT);
  SYSFS_GPIO_Direction(M1_STEP_PIN, OUT);
  // SYSFS_GPIO_Direction(M1_M0_PIN, OUT);
  // SYSFS_GPIO_Direction(M1_M1_PIN, OUT);
  // SYSFS_GPIO_Direction(M1_M2_PIN, OUT);

  SYSFS_GPIO_Export(M2_ENABLE_PIN);
  SYSFS_GPIO_Export(M2_DIR_PIN);
  SYSFS_GPIO_Export(M2_STEP_PIN);
  // SYSFS_GPIO_Export(M2_M0_PIN);
  // SYSFS_GPIO_Export(M2_M1_PIN);
  // SYSFS_GPIO_Export(M2_M2_PIN);

  SYSFS_GPIO_Direction(M2_ENABLE_PIN, OUT);
  SYSFS_GPIO_Direction(M2_DIR_PIN, OUT);
  SYSFS_GPIO_Direction(M2_STEP_PIN, OUT);
  // SYSFS_GPIO_Direction(M2_M0_PIN, OUT);
  // SYSFS_GPIO_Direction(M2_M1_PIN, OUT);
  // SYSFS_GPIO_Direction(M2_M2_PIN, OUT);

  return 0;
}

void driver_exit(void) {}

void microstep_delay_ms(uint32_t ms, uint8_t microsteps) {
  int basedelay = 50000 / microsteps;
  for (int j = ms; j > 0; j--)
    for (int i = basedelay; i > 0; i--)
      ;
}

/**
 * Millisecond delay.
 * @param millis: time in milliseconds.
 */
void delay_ms(uint32_t ms) {
  microstep_delay_ms(ms, 1);
}

/**
 * Microsecond delay.
 * @param us: time in microseconds.
 */
void delay_us(uint32_t us) {
  int j;
  for (j = us; j > 0; j--)
    ;
}

MOTOR Motor;

/**
 * Select motor
 * @param name: motor.
 *
 * Example:
 * select_motor(MOTOR1);
 * or: select_motor(MOTOR2);
 */
void select_motor(uint8_t name) {
  Motor.name = name;
  if (name == MOTOR1) {
    Motor.enable_pin = M1_ENABLE_PIN;
    Motor.direction_pin = M1_DIR_PIN;
    Motor.step_pin = M1_STEP_PIN;
    // Motor.M0_pin = M1_M0_PIN;
    // Motor.M1_pin = M1_M1_PIN;
    // Motor.M2_pin = M1_M2_PIN;
  } else if (name == MOTOR2) {
    Motor.enable_pin = M2_ENABLE_PIN;
    Motor.direction_pin = M2_DIR_PIN;
    Motor.step_pin = M2_STEP_PIN;
    // Motor.M0_pin = M2_M0_PIN;
    // Motor.M1_pin = M2_M1_PIN;
    // Motor.M2_pin = M2_M2_PIN;
  } else {
    DEBUG("please set motor: MOTOR1 or MOTOR2\r\n");
  }
}

static void enable_motor(void) {
  SYSFS_GPIO_Write(Motor.enable_pin, 1);
}

/**
 * The motor stops rotating and the driver chip is disabled.
 */
void stop_motor(void) {
  SYSFS_GPIO_Write(Motor.enable_pin, 0);
}

/**
 * Turn the motor.
 * @param direction: direction.
 * @param steps: Step count.
 * @param stepdelay: step delay.
 */
void turn_motor(uint8_t direction, uint16_t steps, uint16_t stepdelay) {
  Motor.direction = direction;
  if (direction == FORWARD) {
    DEBUG("motor %d formward\r\n", Motor.name);
    enable_motor();
    SYSFS_GPIO_Write(Motor.direction_pin, 0);
  } else if (direction == BACKWARD) {
    DEBUG("motor %d backmward\r\n", Motor.name);
    enable_motor();
    SYSFS_GPIO_Write(Motor.direction_pin, 1);
  } else {
    stop_motor();
  }

  if (steps == 0)
    return;

  uint16_t i = 0;
  DEBUG("turn %d steps\r\n", steps);
  for (i = 0; i < steps; i++) {
    SYSFS_GPIO_Write(Motor.step_pin, 1);
    microstep_delay_ms(stepdelay, MICROSTEPS);
    SYSFS_GPIO_Write(Motor.step_pin, 0);
    microstep_delay_ms(stepdelay, MICROSTEPS);
  }
}
