#include "../include/stepper.hpp"
#include <JetsonGPIO.h>
#include <chrono>
#include <thread>
#include "../include/turret.hpp"
#include "../include/utils.hpp"

const int8_t CLOCKWISE = 1;
const int8_t ANTI_CLOCKWISE = -1;

std::atomic<bool> run_flag(true);

Stepper::Stepper(std::string name,
                 uint8_t enable_pin,
                 uint8_t direction_pin,
                 uint8_t step_pin,
                 float depth,
                 uint16_t origin_px,
                 double c,
                 double f)
    : name(name),
      enable_pin(enable_pin),
      direction_pin(direction_pin),
      step_pin(step_pin),
      depth(depth),
      origin_px(origin_px),
      principal_point(c),
      focal_length(f),
      pos_step_limit(1'000'000),
      neg_step_limit(-1'000'000),
      target_px(0),
      detected_laser_px(0),
      new_setpoint(false),
      new_feedback(false),
      direction(CLOCKWISE),
      current_steps(0),
      target_steps(0),
      manual(false) {}

void Stepper::enable_stepper(void) {
  GPIO::output(enable_pin, GPIO::HIGH);
}

void Stepper::stop_stepper(void) {
  GPIO::output(enable_pin, GPIO::LOW);
}

int32_t Stepper::pixel_to_steps(const uint16_t& px) const {
  // double mm = utils::pixel_to_mm(principal_point, focal_length, px);
  double mm = (px - origin_px) * Turret::CAMERA_DEPTH / focal_length;
  return std::atan2(mm, depth) / MICROSTEP_ANGLE_RAD;
}

uint16_t Stepper::steps_to_pixel(const int32_t& steps) const {
  double mm = depth * std::tan(steps * MICROSTEP_ANGLE_RAD);
  // return utils::mm_to_pixel(principal_point, focal_length, mm);
  return (mm * focal_length / Turret::CAMERA_DEPTH) + origin_px;
}

void Stepper::correct_belief() {
  int32_t step_error =
      (pixel_to_steps(detected_laser_px.load()) - current_steps.load());
  current_steps.fetch_add(step_error);
}

void Stepper::update_target_steps() {
  target_steps.store(pixel_to_steps(target_px.load()));
}

void Stepper::increment_setpoint_in_steps(const int32_t steps) {
  target_px.fetch_add(steps_to_pixel(steps));
  target_steps.fetch_add(steps);
}

uint32_t Stepper::get_steps_and_set_direction() {
  int32_t steps = target_steps.load() - current_steps.load();
  if (steps > 0) {
    GPIO::output(direction_pin, GPIO::LOW);
    direction.store(CLOCKWISE);
  } else {
    GPIO::output(direction_pin, GPIO::HIGH);
    direction.store(ANTI_CLOCKWISE);
  }
  return abs(steps);
}

void Stepper::run_stepper() {
  uint32_t steps;
  uint32_t i;
  uint32_t auto_delay = 100000;
  uint32_t manual_delay = 1000;
  uint32_t delay_us = auto_delay / MICROSTEPS;
  enable_stepper();
  while (run_flag.load() and !utils::exit_flag.load()) {
    steps = get_steps_and_set_direction();
    for (i = 0; i < steps; i++) {
      GPIO::output(step_pin, GPIO::HIGH);
      std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
      GPIO::output(step_pin, GPIO::LOW);
      if (new_setpoint.load() or new_feedback.load()) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
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

    if (manual.load()) {
      delay_us = manual_delay / MICROSTEPS;
    } else {
      delay_us = auto_delay / MICROSTEPS;
    }
  }
  stop_stepper();
}

void Stepper::set_manual(const bool manual) {
  this->manual.store(manual);
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