
#include "../include/turret_controller.hpp"
#include <JetsonGPIO.h>
#include <ncurses.h>
#undef OK  // ncurses and opencv have a macro conflict
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

const int8_t CLOCKWISE = 1;
const int8_t ANTI_CLOCKWISE = -1;
const float FULL_STEP_ANGLE = 0.17578125;
const uint8_t MICROSTEPS = 16;
const double MICROSTEP_ANGLE = FULL_STEP_ANGLE / MICROSTEPS;
const uint32_t STEP_DELAY_US = 300'000;
const uint32_t MICROSTEP_DELAY_US = STEP_DELAY_US / MICROSTEPS;
const uint16_t BELIEF_REGION_UNCERTAINTY = 100;

std::atomic<bool> new_target_flag(false);
std::atomic<bool> new_belief_flag(false);
std::atomic<bool> run_flag(true);

Stepper::Stepper(std::string name,
                 uint8_t enable_pin,
                 uint8_t direction_pin,
                 uint8_t step_pin)
    : name(name),
      enable_pin(enable_pin),
      direction_pin(direction_pin),
      step_pin(step_pin) {}

Stepper x_stepper("x_stepper", M1_ENABLE_PIN, M1_DIR_PIN, M1_STEP_PIN);
Stepper y_stepper("y_stepper", M2_ENABLE_PIN, M2_DIR_PIN, M2_STEP_PIN);

void init(void) {
  GPIO::setmode(GPIO::BOARD);

  GPIO::setup(M1_ENABLE_PIN, GPIO::OUT, GPIO::LOW);
  GPIO::setup(M1_DIR_PIN, GPIO::OUT, GPIO::HIGH);
  GPIO::setup(M1_STEP_PIN, GPIO::OUT, GPIO::LOW);

  GPIO::setup(M2_ENABLE_PIN, GPIO::OUT, GPIO::LOW);
  GPIO::setup(M2_DIR_PIN, GPIO::OUT, GPIO::HIGH);
  GPIO::setup(M2_STEP_PIN, GPIO::OUT, GPIO::LOW);
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

void home_steppers(void) {
  x_stepper.step_count.store(0);
  y_stepper.step_count.store(0);
}

void update_target(const std::pair<uint16_t, uint16_t>& target_pixels) {
  std::pair<float, float> mm = utils::pixel_to_mm(
      target_pixels.first, target_pixels.second, utils::CAMERA_DEPTH);

  auto correct_step_count = [](Stepper& stepper, float mm) -> void {
    float angle = utils::mm_to_angle(mm, utils::TURRET_DEPTH);
    int32_t step_error =
        angle / MICROSTEP_ANGLE - stepper.prev_step_count.load();
    stepper.step_count.fetch_add(step_error);
    stepper.prev_step_count.store(stepper.step_count);
  };

  correct_step_count(x_stepper, mm.first);
  correct_step_count(y_stepper, mm.second);
}

/**
 * @return The a circle with centre the pixel co-ordinates of the laser and
 * radius the uncertainty.
 */
utils::Circle get_laser_belief_region(void) {
  float x_angle = x_stepper.step_count.load() * MICROSTEP_ANGLE;
  float y_angle = y_stepper.step_count.load() * MICROSTEP_ANGLE;

  double X = utils::angle_to_mm(x_angle, utils::TURRET_DEPTH);
  double Y = utils::angle_to_mm(y_angle, utils::TURRET_DEPTH);

  utils::Circle belief_region(utils::mm_to_pixel(X, Y, utils::CAMERA_DEPTH));
  belief_region.radius = 100;
  return belief_region;
}

// void correct_laser_belief(const utils::Point<uint16_t>& laser_detected_px) {
void correct_laser_belief(
    const std::pair<uint16_t, uint16_t>& laser_detected_px) {
  std::pair<float, float> mm = utils::pixel_to_mm(
      laser_detected_px.first, laser_detected_px.second, utils::CAMERA_DEPTH);

  auto correct_step_count = [](Stepper& stepper, float mm) -> void {
    float angle = utils::mm_to_angle(mm, utils::TURRET_DEPTH);
    int32_t step_error =
        angle / MICROSTEP_ANGLE - stepper.prev_step_count.load();
    stepper.step_count.fetch_add(step_error);
    stepper.prev_step_count.store(stepper.step_count);
  };

  correct_step_count(x_stepper, mm.first);
  correct_step_count(y_stepper, mm.second);
}

static uint32_t get_steps_and_set_direction(Stepper& stepper) {
  int32_t steps = stepper.target_step_count.load() - stepper.step_count.load();
  // std::cout << "[" << stepper.name
  //           << ": target_steps = " << stepper.target_step_count
  //           << ", step_count = " << stepper.step_count << "]" << std::endl;
  if (steps > 0) {
    GPIO::output(stepper.direction_pin, GPIO::LOW);
    stepper.direction = CLOCKWISE;
  } else {
    GPIO::output(stepper.direction_pin, GPIO::HIGH);
    stepper.direction = ANTI_CLOCKWISE;
  }
  return abs(steps);
}

void run_stepper(Stepper& stepper) {
  uint32_t steps;
  uint16_t delay_us = 300;
  enable_motor(stepper);
  while (run_flag.load()) {
    steps = get_steps_and_set_direction(stepper);
    for (uint32_t i = 0; i < steps; i++) {
      GPIO::output(stepper.step_pin, GPIO::HIGH);
      std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
      GPIO::output(stepper.step_pin, GPIO::LOW);
      // if (new_target_flag.load() or new_belief_flag.load()) {
      //   new_target_flag.store(false);
      //   new_belief_flag.store(false);
      //   break;
      // }
      std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
    }
    if (stepper.direction == CLOCKWISE) {
      stepper.step_count.fetch_add(steps);
    } else {
      stepper.step_count.fetch_sub(steps);
    }
  }
}

static void update_target_steps(Stepper& stepper, int16_t steps) {
  std::cout << "[" << stepper.name << ": new target step count = "
            << stepper.target_step_count.fetch_add(steps) << "]" << std::endl;
  new_target_flag.store(true);
}

void keyboard_manual(int ch) {
  const int steps = 100;
  switch (ch) {
    case KEY_UP:
      update_target_steps(y_stepper, -steps);
      break;
    case KEY_DOWN:
      update_target_steps(y_stepper, steps);
      break;
    case KEY_LEFT:
      update_target_steps(x_stepper, -steps);
      break;
    case KEY_RIGHT:
      update_target_steps(x_stepper, steps);
      break;
    /*case 'w':
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
      break;*/
    default:
      break;
  }
}

void keyboard_auto(int ch) {
  const int steps = 100;
  switch (ch) {
    case KEY_UP:
      update_target(std::pair<uint16_t, uint16_t>(0, -200));
      break;
    case KEY_DOWN:
      update_target(std::pair<uint16_t, uint16_t>(0, 200));
      break;
    case KEY_LEFT:
      update_target(std::pair<uint16_t, uint16_t>(-200, 0));
      break;
    case KEY_RIGHT:
      update_target(std::pair<uint16_t, uint16_t>(200, 0));
      break;
    default:
      break;
  }
}

}  // namespace turret