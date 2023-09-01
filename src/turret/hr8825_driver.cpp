#include "hr8825_driver.hpp"
#include <JetsonGPIO.h>
#include "../utilities/debug.hpp"
#include "../utilities/utils.hpp"
#include "turret_controller.hpp"

namespace driver {

uint8_t init_driver_pins(void) {
  GPIO::setmode(GPIO::BOARD);

  GPIO::setup(M1_ENABLE_PIN, GPIO::OUT, GPIO::LOW);
  GPIO::setup(M1_DIR_PIN, GPIO::OUT, GPIO::HIGH);
  GPIO::setup(M1_STEP_PIN, GPIO::OUT, GPIO::LOW);

  GPIO::setup(M2_ENABLE_PIN, GPIO::OUT, GPIO::LOW);
  GPIO::setup(M2_DIR_PIN, GPIO::OUT, GPIO::HIGH);
  GPIO::setup(M2_STEP_PIN, GPIO::OUT, GPIO::LOW);
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
  GPIO::output(Motor.enable_pin, GPIO::HIGH);
}

void stop_motor(void) {
  GPIO::output(Motor.enable_pin, GPIO::LOW);
}

void stop_all_motors(void) {
  select_motor(MOTOR1);
  stop_motor();
  select_motor(MOTOR2);
  stop_motor();
}

/**
 * @brief Turn the motor a certain number of steps in a certain direction.
 * For now delay compensates for microstepping inside turn_motor().
 */
void turn_motor(uint8_t direction, uint16_t steps, uint16_t stepdelay) {
  Motor.direction = direction;
  if (direction == FORWARD) {
    enable_motor();
    GPIO::output(Motor.direction_pin, GPIO::LOW);
  } else if (direction == BACKWARD) {
    enable_motor();
    GPIO::output(Motor.direction_pin, GPIO::HIGH);
  } else {
    stop_motor();
  }

  if (steps == 0)
    return;

  for (uint32_t i = 0; i < steps; i++) {
    GPIO::output(Motor.step_pin, GPIO::HIGH);
    utils::microstep_delay_ms(stepdelay, MICROSTEPS);
    GPIO::output(Motor.step_pin, GPIO::LOW);
    utils::microstep_delay_ms(stepdelay, MICROSTEPS);
  }
}

}  // namespace driver
