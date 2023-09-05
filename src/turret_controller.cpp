
#include "../include/turret_controller.hpp"
#include <JetsonGPIO.h>
#include <ncurses.h>
#include <unistd.h>
#include <atomic>
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
const float STEP_DELAY = 3;
// const float MIRCOSTEP_DELAY = STEP_DELAY / MICROSTEPS;

std::atomic<bool> new_target_flag(false);
std::atomic<bool> run_flag(true);

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

void update_target(const utils::Point target_px) {}

void update_target_steps(const int x_steps, const int y_steps) {
  x_stepper.target_step_count += x_steps;
  y_stepper.target_step_count += y_steps;
  new_target_flag.store(true);
}

utils::Circle get_laser_belief_region(void) {
  std::pair<uint16_t, uint16_t> pixel_point =
      utils::angle_to_pixel({x_stepper.step_count * MICROSTEP_ANGLE,
                             y_stepper.step_count * MICROSTEP_ANGLE});
  utils::Circle belief_region;
  belief_region.x = pixel_point.first;
  belief_region.y = pixel_point.second;
  belief_region.radius = 100;
  new_target_flag.store(true);
  return belief_region;
}

void correct_laser_belief_region(const utils::Point& laser_detected_px) {}

static uint32_t get_steps_and_set_direction(Stepper& stepper) {
  uint32_t steps = stepper.target_step_count - stepper.step_count;
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
  const uint16_t DELAY_US = 3000 / MICROSTEPS;

  while (run_flag.load()) {
    for (uint32_t i = 0; i < get_steps_and_set_direction(stepper); i++) {
      GPIO::output(stepper.step_pin, GPIO::HIGH);
      std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
      GPIO::output(stepper.step_pin, GPIO::LOW);

      std::chrono::high_resolution_clock::time_point loop_start_time =
          std::chrono::high_resolution_clock::now();
      if (stepper.direction) {
        stepper.step_count += 1;
      } else {
        stepper.step_count -= 1;
      }
      if (new_target_flag.load()) {
        new_target_flag.store(false);
        break;
      }
      std::chrono::high_resolution_clock::time_point loop_end_time =
          std::chrono::high_resolution_clock::now();
      uint32_t loop_duration =
          std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time -
                                                                loop_start_time)
              .count();
      std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
    }
  }
}

void home_steppers(void) {
  x_stepper.step_count = 0;
  y_stepper.step_count = 0;
}

}  // namespace turret