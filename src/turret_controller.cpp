
#include "../include/turret_controller.hpp"
#include <JetsonGPIO.h>
#include <ncurses.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "../include/utils.hpp"

namespace turret {

#define M1_ENABLE_PIN 32
#define M1_DIR_PIN 33
#define M1_STEP_PIN 35

#define M2_ENABLE_PIN 7
#define M2_DIR_PIN 18
#define M2_STEP_PIN 12

const float FULL_STEP_ANGLE = 0.17578125;
const int MICROSTEPS = 16;
const double MICROSTEP_ANGLE = FULL_STEP_ANGLE / MICROSTEPS;
const float STEP_DELAY = 3;
const double MIRCOSTEP_DELAY = STEP_DELAY / MICROSTEPS;

typedef struct {
  uint8_t enable_pin;
  uint8_t direction_pin;
  uint8_t step_pin;
  int step_count = 0;
  int pos_step_limit = 1000000;
  int neg_step_limit = -1000000;
} Stepper;

static Stepper x_stepper;
static Stepper y_stepper;

void init(void) {
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

void stop_all_motors(void) {
  stop_motor(x_stepper);
  stop_motor(y_stepper);
}

static void turn_motor(Stepper& stepper,
                       int steps,
                       int step_delay,
                       bool manual = false) {
  if (steps == 0)
    return;

  // Check if the desired movement will surpass the positive limit
  if (!manual) {
    if (stepper.step_count + steps > stepper.pos_step_limit) {
      steps = stepper.pos_step_limit - stepper.step_count;
      std::cout << "Positive stepper limit reached" << std::endl;
    }
    // Check if the desired movement will surpass the negative limit
    else if (stepper.step_count + steps < stepper.neg_step_limit) {
      steps = stepper.neg_step_limit - stepper.step_count;
      std::cout << "Negative stepper limit reached" << std::endl;
    }
  }

  GPIO::output(stepper.enable_pin, GPIO::HIGH);
  if (steps > 0) {
    GPIO::output(stepper.direction_pin, GPIO::LOW);
  } else {
    GPIO::output(stepper.direction_pin, GPIO::HIGH);
  }

  for (uint32_t i = 0; i < abs(steps); i++) {
    GPIO::output(stepper.step_pin, GPIO::HIGH);
    std::this_thread::sleep_for(std::chrono::microseconds((step_delay)));
    usleep(step_delay);
    // utils::microstep_delay_ms(step_delay, MICROSTEPS);
    GPIO::output(stepper.step_pin, GPIO::LOW);
    utils::microstep_delay_ms(step_delay, MICROSTEPS);
  }

  stepper.step_count += steps;
}

void manual_control(int ch) {
  const int steps = 100;
  switch (ch) {
    case KEY_UP:
      turn_motor(y_stepper, -steps, STEP_DELAY, true);
      break;
    case KEY_DOWN:
      turn_motor(y_stepper, steps, STEP_DELAY, true);
      break;
    case KEY_LEFT:
      turn_motor(x_stepper, -steps, STEP_DELAY, true);
      break;
    case KEY_RIGHT:
      turn_motor(x_stepper, steps, STEP_DELAY, true);
      break;
    case 'w':
      y_stepper.neg_step_limit = y_stepper.step_count;
      std::cout << "y_stepper.neg_step_limit: " << y_stepper.neg_step_limit
                << std::endl;
      break;
    case 's':
      y_stepper.pos_step_limit = y_stepper.step_count;
      std::cout << "y_stepper.pos_step_limit: " << y_stepper.pos_step_limit
                << std::endl;
      break;
    case 'a':
      x_stepper.neg_step_limit = x_stepper.step_count;
      std::cout << "x_stepper.neg_step_limit: " << x_stepper.neg_step_limit
                << std::endl;
      break;
    case 'd':
      x_stepper.pos_step_limit = x_stepper.step_count;
      std::cout << "x_stepper.pos_step_limit: " << x_stepper.pos_step_limit
                << std::endl;
      break;
    default:
      break;
  }
}

void auto_control(std::pair<int, int> detected_angle,
                  std::pair<int, int> target_angle) {
  if (detected_angle.first == target_angle.first and
      detected_angle.second == target_angle.second) {
    std::cout << "Auto" << detected_angle.first << " " << target_angle.second
              << std::endl;
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