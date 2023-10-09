#include "../include/stepper.hpp"
#include <JetsonGPIO.h>
#include <chrono>
#include <thread>
#include "../include/frame.hpp"
#include "../include/turret.hpp"
#include "../include/utils.hpp"

const int8_t CLOCKWISE = 1;
const int8_t ANTI_CLOCKWISE = -1;

Stepper::Stepper(std::string name,
                 uint8_t enable_pin,
                 uint8_t direction_pin,
                 uint8_t step_pin,
                 uint8_t gpio_clockwise,
                 uint8_t gpio_anticlockwise,
                 float depth,
                 uint16_t origin_px,
                 double c,
                 double f)
    : name(name),
      enable_pin(enable_pin),
      direction_pin(direction_pin),
      step_pin(step_pin),
      gpio_clockwise(gpio_clockwise),
      gpio_anticlockwise(gpio_anticlockwise),
      depth(depth),
      origin_px(origin_px),
      principal_point(c),
      focal_length(f),
      pos_step_limit(1'000'000),
      neg_step_limit(-1'000'000),
      target_px(150),
      detected_laser_px(0),
      new_setpoint(false),
      new_feedback(false),
      direction(CLOCKWISE),
      current_steps(0),
      target_steps(0),
      previous_error(0),
      integral(0) {}

void Stepper::home(void) {
  current_steps.store(0);
  target_steps.store(0);
}

void Stepper::enable_stepper(void) {
  GPIO::output(enable_pin, GPIO::HIGH);
}

void Stepper::stop_stepper(void) {
  GPIO::output(enable_pin, GPIO::LOW);
}

int32_t Stepper::pixel_to_steps(const uint16_t& px) const {
  double mm = (px - origin_px) * Turret::CAMERA_DEPTH / focal_length;
  return std::atan2(mm, depth) / MICROSTEP_ANGLE_RAD;
}

uint16_t Stepper::steps_to_pixel(const int32_t& steps) const {
  double mm = depth * std::tan(steps * MICROSTEP_ANGLE_RAD);
  return (mm * focal_length / Turret::CAMERA_DEPTH) + origin_px;
}

void Stepper::save_steps() {
  steps_at_detection.store(current_steps.load());
}

void Stepper::correct_belief() {
  int32_t step_error =
      (pixel_to_steps(detected_laser_px.load()) - steps_at_detection.load());
  current_steps.fetch_add(step_error);
}

void Stepper::update_target_steps() {
  target_steps.store(pixel_to_steps(target_px.load()));
}

void Stepper::step_manually(const int32_t steps) {
  if (steps > 0) {
    GPIO::output(direction_pin, gpio_clockwise);
    direction.store(CLOCKWISE);
  } else {
    GPIO::output(direction_pin, gpio_anticlockwise);
    direction.store(ANTI_CLOCKWISE);
  }
  uint32_t delay_us = 1000 / MICROSTEPS;
  for (uint32_t i = 0; i < abs(steps); i++) {
    GPIO::output(step_pin, GPIO::HIGH);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
    GPIO::output(step_pin, GPIO::LOW);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
  }
}

uint32_t Stepper::get_pid_error_and_set_direction() {
  int32_t error = target_steps.load() - current_steps.load();

  integral += error;
  int32_t derivative = error - previous_error;
  previous_error = error;

  int32_t output = K_P * error + K_I * integral + K_D * derivative;

  if (output > 0) {
    GPIO::output(direction_pin, gpio_clockwise);
    direction.store(CLOCKWISE);
  } else {
    GPIO::output(direction_pin, gpio_anticlockwise);
    direction.store(ANTI_CLOCKWISE);
  }

  return abs(output);
}

void Stepper::run_stepper() {
  uint32_t steps;
  uint32_t i;
  uint32_t auto_delay_us = 3000 / MICROSTEPS;
  while (!utils::exit_flag.load()) {
    if (!utils::manual_mode.load()) {
      steps = get_pid_error_and_set_direction();

      for (i = 0; i < steps; i++) {
        GPIO::output(step_pin, GPIO::HIGH);
        std::this_thread::sleep_for(std::chrono::microseconds(auto_delay_us));
        GPIO::output(step_pin, GPIO::LOW);
        if (new_setpoint.load() or new_feedback.load() or
            !utils::run_flag.load() or utils::exit_flag.load()) {
          break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(auto_delay_us));
      }
      if (direction == CLOCKWISE) {
        current_steps.fetch_add(i);
      } else {
        current_steps.fetch_sub(i);
      }

      if (new_feedback.load()) {
        correct_belief();
        new_feedback.store(false);
      }
      if (new_setpoint.load()) {
        update_target_steps();
        new_setpoint.store(false);
      }
    }
  }
  stop_stepper();
}

void Stepper::set_origin_px(const uint16_t px) {
  origin_px.store(px);
}

void Stepper::set_target_px(const uint16_t px) {
  target_px.store(px);
  new_setpoint.store(true);
}

void Stepper::set_detected_laser_px(const uint16_t px) {
  detected_laser_px.store(px);
  new_feedback.store(true);
}

uint16_t Stepper::get_origin_px(void) const {
  return origin_px.load();
}

uint16_t Stepper::get_target_px(void) const {
  return target_px.load();
}

uint16_t Stepper::get_detected_laser_px(void) const {
  return detected_laser_px.load();
}

uint16_t Stepper::get_current_px(void) const {
  return steps_to_pixel(current_steps.load());
}

int32_t Stepper::get_current_steps(void) const {
  return current_steps.load();
}

int32_t Stepper::get_target_steps(void) const {
  return target_steps.load();
}