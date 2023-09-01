
#include "turret_controller.hpp"
#include <JetsonGPIO.h>
#include <cstdlib>
#include "../utilities/utils.hpp"

namespace turret {

#define M1_ENABLE_PIN 32
#define M1_DIR_PIN 33
#define M1_STEP_PIN 35

#define M2_ENABLE_PIN 7
#define M2_DIR_PIN 18
#define M2_STEP_PIN 12

const int FULL_STEP_ANGLE = 0.17578125;
const int MICROSTEPS = 16;
const int MICROSTEP_ANGLE = FULL_STEP_ANGLE / MICROSTEPS;
const int STEP_DELAY = 3;
const int MIRCOSTEP_DELAY = STEP_DELAY / MICROSTEPS;

typedef struct {
  uint8_t enable_pin;
  uint8_t direction_pin;
  uint8_t step_pin;
  int step_count;
} Stepper;

static Stepper x_stepper;
static Stepper y_stepper;

static void init(void) {
  GPIO::setmode(GPIO::BOARD);

  GPIO::setup(M1_ENABLE_PIN, GPIO::OUT, GPIO::LOW);
  GPIO::setup(M1_DIR_PIN, GPIO::OUT, GPIO::HIGH);
  GPIO::setup(M1_STEP_PIN, GPIO::OUT, GPIO::LOW);
  x_stepper.enable_pin = M1_ENABLE_PIN;
  x_stepper.direction_pin = M1_DIR_PIN;
  x_stepper.step_pin = M1_STEP_PIN;

  GPIO::setup(M2_ENABLE_PIN, GPIO::OUT, GPIO::LOW);
  GPIO::setup(M2_DIR_PIN, GPIO::OUT, GPIO::HIGH);
  GPIO::setup(M2_STEP_PIN, GPIO::OUT, GPIO::LOW);
  y_stepper.enable_pin = M2_ENABLE_PIN;
  y_stepper.direction_pin = M2_DIR_PIN;
  y_stepper.step_pin = M2_STEP_PIN;
}

static void enable_motor(Stepper& stepper) {
  GPIO::output(stepper.enable_pin, GPIO::HIGH);
}

static void stop_motor(Stepper& stepper) {
  GPIO::output(stepper.enable_pin, GPIO::LOW);
}

static void stop_all_motors(void) {
  stop_motor(x_stepper);
  stop_motor(y_stepper);
}

static void turn_motor(Stepper& stepper, int steps, int step_delay) {
  if (steps == 0)
    return;

  GPIO::output(stepper.enable_pin, GPIO::HIGH);
  if (steps > 0) {
    GPIO::output(stepper.direction_pin, GPIO::LOW);
  } else {
    GPIO::output(stepper.direction_pin, GPIO::HIGH);
  }

  for (uint32_t i = 0; i < abs(steps); i++) {
    GPIO::output(stepper.step_pin, GPIO::HIGH);
    utils::microstep_delay_ms(step_delay, MICROSTEPS);
    GPIO::output(stepper.step_pin, GPIO::LOW);
    utils::microstep_delay_ms(step_delay, MICROSTEPS);
  }

  stepper.step_count += steps;
}

void manual_control(int ch) {
  const int steps = 100;
  switch (ch) {
    case KEY_UP:
      turn_motor(y_stepper, steps, STEP_DELAY);
      break;
    case KEY_DOWN:
      turn_motor(y_stepper, -steps, STEP_DELAY);
      break;
    case KEY_LEFT:
      turn_motor(x_stepper, steps, STEP_DELAY);
      break;
    case KEY_RIGHT:
      turn_motor(x_stepper, -steps, STEP_DELAY);
      break;
    default:
      break;
  }
}

void auto_control(std::pair<int, int> detected_angle,
                  std::pair<int, int> target_angle) {
  if (detected_angle.first == target_angle.first and
      detected_angle.second == target_angle.second) {
    return;
  }

  turn_motor(x_stepper,
             ((detected_angle.first - target_angle.first) / MICROSTEP_ANGLE),
             STEP_DELAY);
  stop_motor(x_stepper);

  turn_motor(y_stepper,
             ((detected_angle.second - target_angle.second) / MICROSTEP_ANGLE),
             STEP_DELAY);
  stop_motor(y_stepper);
}

void home_steppers(void) {
  x_stepper.step_count = 0;
  y_stepper.step_count = 0;
}

}  // namespace turret