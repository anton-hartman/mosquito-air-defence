
#include "../include/turret_controller.hpp"
#include <JetsonGPIO.h>
#include <chrono>
#include <iostream>
#include <thread>

namespace turret {

#define M1_ENABLE_PIN 32
#define M1_DIR_PIN 33
#define M1_STEP_PIN 35

#define M2_ENABLE_PIN 7
#define M2_DIR_PIN 18
#define M2_STEP_PIN 12

const double PI = 3.14159265359;

const int8_t CLOCKWISE = 1;
const int8_t ANTI_CLOCKWISE = -1;
const float FULL_STEP_ANGLE = 0.17578125;
const uint8_t MICROSTEPS = 16;
const double MICROSTEP_ANGLE = FULL_STEP_ANGLE / MICROSTEPS;
const uint32_t STEP_DELAY_US = 300'000;
const uint32_t MICROSTEP_DELAY_US = STEP_DELAY_US / MICROSTEPS;

const uint16_t TANK_DEPTH = 318;                 // mm
const uint16_t TURRET_DEPTH = 550 + TANK_DEPTH;  // mm
const uint16_t CAMERA_DEPTH = 765 + TANK_DEPTH;  // mm
const uint16_t BELIEF_REGION_UNCERTAINTY = 100;

const double f_x = 647.0756309728268;
const double f_y = 861.7363873209705;
const double c_x = 304.4404590127848;
const double c_y = 257.5858878142162;

std::atomic<bool> run_flag(true);

Stepper::Stepper(std::string name,
                 uint8_t enable_pin,
                 uint8_t direction_pin,
                 uint8_t step_pin,
                 double c,
                 double f)
    : name(name),
      enable_pin(enable_pin),
      direction_pin(direction_pin),
      step_pin(step_pin),
      principal_point(c),
      focal_length(f),
      pos_step_limit(1'000'000),
      neg_step_limit(-1'000'000),
      setpoint_px(0),
      detected_laser_px(0),
      new_setpoint(false),
      new_feedback(false),
      direction(CLOCKWISE),
      prev_step_count(0),
      step_count(0),
      target_step_count(0) {}

float Stepper::get_belief_angle(void) const {
  return step_count.load() * MICROSTEP_ANGLE;
}

Stepper x_stepper("x_stepper",
                  M1_ENABLE_PIN,
                  M1_DIR_PIN,
                  M1_STEP_PIN,
                  c_x,
                  f_x);
Stepper y_stepper("y_stepper",
                  M2_ENABLE_PIN,
                  M2_DIR_PIN,
                  M2_STEP_PIN,
                  c_y,
                  f_y);

static void enable_motor(Stepper& stepper) {
  GPIO::output(stepper.enable_pin, GPIO::HIGH);
}

static void stop_motor(Stepper& stepper) {
  GPIO::output(stepper.enable_pin, GPIO::LOW);
}

static float pixel_to_angle(const Stepper& stepper, const uint16_t& px) {
  float mm = utils::pixel_to_mm(stepper, px, CAMERA_DEPTH);
  return utils::mm_to_angle(mm, TURRET_DEPTH) * 360.0 / PI;
}

static uint16_t belief_angle_to_pixel(const Stepper& stepper) {
  float mm =
      utils::angle_to_mm(stepper.get_belief_angle() * PI / 360.0, TURRET_DEPTH);
  return utils::mm_to_pixel(stepper, mm, CAMERA_DEPTH);
}

static void correct_belief(Stepper& stepper) {
  float angle = pixel_to_angle(stepper, stepper.detected_laser_px);
  int32_t step_error =
      (angle / MICROSTEP_ANGLE) - stepper.prev_step_count.load();
  stepper.step_count.fetch_add(step_error);
  stepper.prev_step_count.store(stepper.step_count);
}

static void setpoint_to_steps(Stepper& stepper) {
  float angle = pixel_to_angle(stepper, stepper.setpoint_px);
  stepper.target_step_count.store(angle / MICROSTEP_ANGLE);
}

static uint32_t get_steps_and_set_direction(Stepper& stepper) {
  int32_t steps = stepper.target_step_count.load() - stepper.step_count.load();
  if (steps > 0) {
    GPIO::output(stepper.direction_pin, GPIO::LOW);
    stepper.direction.store(CLOCKWISE);
  } else {
    GPIO::output(stepper.direction_pin, GPIO::HIGH);
    stepper.direction.store(ANTI_CLOCKWISE);
  }
  return abs(steps);
}

// Should only be used for testing
static void increment_setpoint_in_steps(Stepper& stepper, int16_t steps) {
  stepper.target_step_count.fetch_add(steps);
  std::cout << "[" << stepper.name
            << ": new target step count = " << stepper.target_step_count.load()
            << "]" << std::endl;
}

void init(void) {
  GPIO::setmode(GPIO::BOARD);

  GPIO::setup(M1_ENABLE_PIN, GPIO::OUT, GPIO::LOW);
  GPIO::setup(M1_DIR_PIN, GPIO::OUT, GPIO::HIGH);
  GPIO::setup(M1_STEP_PIN, GPIO::OUT, GPIO::LOW);

  GPIO::setup(M2_ENABLE_PIN, GPIO::OUT, GPIO::LOW);
  GPIO::setup(M2_DIR_PIN, GPIO::OUT, GPIO::HIGH);
  GPIO::setup(M2_STEP_PIN, GPIO::OUT, GPIO::LOW);
}

void stop_all_motors(void) {
  stop_motor(x_stepper);
  stop_motor(y_stepper);
}

/**
 * @return The a circle with centre the pixel co-ordinates of the laser and
 * radius the uncertainty.
 */
utils::Circle get_belief_region(void) {
  uint16_t x_px = belief_angle_to_pixel(x_stepper);
  uint16_t y_px = belief_angle_to_pixel(y_stepper);

  return utils::Circle(x_px, y_px, BELIEF_REGION_UNCERTAINTY);
}

std::pair<uint16_t, uint16_t> get_belief_px(void) {
  return std::pair<uint16_t, uint16_t>(belief_angle_to_pixel(x_stepper),
                                       belief_angle_to_pixel(y_stepper));
}

std::pair<uint16_t, uint16_t> get_setpoint_px(void) {
  return std::pair<uint16_t, uint16_t>(x_stepper.setpoint_px.load(),
                                       y_stepper.setpoint_px.load());
}

std::pair<uint16_t, uint16_t> get_target_px(void) {
  return std::pair<uint16_t, uint16_t>(
      pixel_to_angle(x_stepper, x_stepper.target_step_count.load()),
      pixel_to_angle(y_stepper, y_stepper.target_step_count.load()));
}

void update_setpoint(const std::pair<uint16_t, uint16_t> setpoint_px) {
  x_stepper.setpoint_px.store(setpoint_px.first);
  y_stepper.setpoint_px.store(setpoint_px.second);
  x_stepper.new_setpoint.store(true);
  y_stepper.new_setpoint.store(true);
}

void update_belief(const std::pair<uint16_t, uint16_t> detected_laser_px) {
  x_stepper.detected_laser_px.store(detected_laser_px.first);
  y_stepper.detected_laser_px.store(detected_laser_px.second);
  x_stepper.new_feedback.store(true);
  y_stepper.new_feedback.store(true);
}

void run_stepper(Stepper& stepper) {
  uint32_t steps;
  uint32_t i;
  uint16_t delay_us = 300;
  enable_motor(stepper);
  while (run_flag.load() and !utils::exit_flag.load()) {
    steps = get_steps_and_set_direction(stepper);
    for (i = 0; i < steps; i++) {
      GPIO::output(stepper.step_pin, GPIO::HIGH);
      std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
      GPIO::output(stepper.step_pin, GPIO::LOW);
      if (stepper.new_setpoint.load() or stepper.new_feedback.load()) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
    }
    if (stepper.direction == CLOCKWISE) {
      stepper.step_count.fetch_add(i);
    } else {
      stepper.step_count.fetch_sub(i);
    }

    if (stepper.new_feedback.load()) {
      correct_belief(stepper);
      stepper.new_feedback.store(false);
    }
    if (stepper.new_setpoint.load()) {
      setpoint_to_steps(stepper);
      stepper.new_setpoint.store(false);
    }
  }
  stop_motor(stepper);
}

void keyboard_auto(int ch) {
  const unsigned int px = 100;
  switch (ch) {
    case 'w':
      turret::update_setpoint(std::pair<uint16_t, uint16_t>(
          x_stepper.setpoint_px.load(), y_stepper.setpoint_px.load() - px));
      break;
    case 's':
      turret::update_setpoint(std::pair<uint16_t, uint16_t>(
          x_stepper.setpoint_px.load(), y_stepper.setpoint_px.load() + px));
      break;
    case 'a':
      turret::update_setpoint(std::pair<uint16_t, uint16_t>(
          x_stepper.setpoint_px.load() - px, y_stepper.setpoint_px.load()));
      break;
    case 'd':
      turret::update_setpoint(std::pair<uint16_t, uint16_t>(
          x_stepper.setpoint_px.load() + px, y_stepper.setpoint_px.load()));
      break;
    default:
      break;
  }
}

void keyboard_manual(int ch) {
  const int steps = 1000;
  switch (ch) {
    case 'w':
      turret::increment_setpoint_in_steps(turret::y_stepper, -steps);
      break;
    case 's':
      turret::increment_setpoint_in_steps(turret::y_stepper, steps);
      break;
    case 'a':
      turret::increment_setpoint_in_steps(turret::x_stepper, -steps);
      break;
    case 'd':
      turret::increment_setpoint_in_steps(turret::x_stepper, steps);
      break;
    default:
      break;
  }
}

}  // namespace turret