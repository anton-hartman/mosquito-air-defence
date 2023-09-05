
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
const uint8_t MICROSTEPS = 16;
const double MICROSTEP_ANGLE = FULL_STEP_ANGLE / MICROSTEPS;
const uint16_t STEP_DELAY_US = 3000;
const uint16_t MICROSTEP_DELAY_US = STEP_DELAY_US / MICROSTEPS;
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
// x_stepper.name = "x_stepper";
Stepper y_stepper("y_stepper", M2_ENABLE_PIN, M2_DIR_PIN, M2_STEP_PIN);
// Stepper::y_stepper.name = "y_stepper";

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

// void update_target(const utils::Point target_px) {}

void update_target_steps(Stepper& stepper, int16_t steps) {
  std::puts("update_target_steps");
  std::cout << stepper.name << ": new target step count = "
            << stepper.target_step_count.fetch_add(steps) << std::endl;
  new_target_flag.store(true);
}

utils::Circle get_laser_belief_region(void) {
  std::pair<uint16_t, uint16_t> pixel_point =
      utils::angle_to_pixel({x_stepper.step_count.load() * MICROSTEP_ANGLE,
                             y_stepper.step_count.load() * MICROSTEP_ANGLE});
  utils::Circle belief_region;
  belief_region.x = pixel_point.first;
  belief_region.y = pixel_point.second;
  belief_region.radius = BELIEF_REGION_UNCERTAINTY;
  new_target_flag.store(true);
  return belief_region;
}

// void correct_laser_belief_region(const utils::Point& laser_detected_px) {}

static uint32_t get_steps_and_set_direction(Stepper& stepper) {
  int32_t steps = stepper.target_step_count.load() - stepper.step_count.load();
  std::cout << "get_steps_and_dir - " << stepper.name
            << ": target_steps = " << stepper.target_step_count
            << ", steps = " << steps << std::endl;
  if (steps > 0) {
    GPIO::output(stepper.direction_pin, GPIO::LOW);
    stepper.direction = 0;
  } else {
    GPIO::output(stepper.direction_pin, GPIO::HIGH);
    stepper.direction = 1;
  }
  return abs(steps);
}

void run_stepper(Stepper& stepper) {
  uint32_t steps;
  while (run_flag.load()) {
    steps = get_steps_and_set_direction(stepper);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    for (uint32_t i = 0; i < steps; i++) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      GPIO::output(stepper.step_pin, GPIO::HIGH);
      std::this_thread::sleep_for(
          std::chrono::microseconds(MICROSTEP_DELAY_US));
      GPIO::output(stepper.step_pin, GPIO::LOW);
      if (new_target_flag.load() or new_belief_flag.load()) {
        new_target_flag.store(false);
        new_belief_flag.store(false);
        if (stepper.direction) {
          stepper.target_step_count.fetch_sub(i);
        } else {
          stepper.target_step_count.fetch_add(i);
        }
        break;
      }
      std::this_thread::sleep_for(
          std::chrono::microseconds(MICROSTEP_DELAY_US));
    }
  }
}

void manual_control(int ch) {
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

}  // namespace turret