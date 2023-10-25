#include "../include/stepper.hpp"
#include <JetsonGPIO.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "../include/frame.hpp"
#include "../include/mads.hpp"
#include "../include/turret.hpp"

const int8_t CLOCKWISE = 1;
const int8_t ANTI_CLOCKWISE = -1;

Stepper::Stepper(std::string name,
                 uint8_t enable_pin,
                 uint8_t direction_pin,
                 uint8_t step_pin,
                 uint8_t forward,
                 uint8_t backward,
                 float depth,
                 uint16_t turret_origin_px,
                 double f)
    : name(name),
      enable_pin(enable_pin),
      direction_pin(direction_pin),
      step_pin(step_pin),
      forward(forward),
      backward(backward),
      depth(depth),
      turret_origin_px(turret_origin_px),
      focal_length(f),
      target_px(150),
      detected_laser_px(0),
      new_setpoint(false),
      new_feedback(false),
      current_steps(0),
      previous_steps(0),
      target_steps(0),
      previous_error(0),
      integral(0),
      centered(false) {}

void Stepper::home(void) {
  current_steps.store(0);
  target_steps.store(0);
}

void Stepper::enable_stepper(void) {
  GPIO::output(enable_pin, GPIO::HIGH);
  integral = 0;
  previous_error = 0;
}

void Stepper::stop_stepper(void) {
  GPIO::output(enable_pin, GPIO::LOW);
}

int32_t Stepper::pixel_to_steps(const uint16_t& px) const {
  double mm = (px - turret_origin_px) * Turret::CAMERA_DEPTH / focal_length;
  return std::atan2(mm, depth) / MICROSTEP_ANGLE_RAD;
}

uint16_t Stepper::steps_to_pixel(const int32_t& steps) const {
  double mm = depth * std::tan(steps * MICROSTEP_ANGLE_RAD);
  return (mm * focal_length / Turret::CAMERA_DEPTH) + turret_origin_px;
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
    GPIO::output(direction_pin, forward);
  } else {
    GPIO::output(direction_pin, backward);
  }
  uint32_t delay_us = 100 / MICROSTEPS;
  for (uint32_t i = 0; i < abs(steps); i++) {
    GPIO::output(step_pin, GPIO::HIGH);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
    GPIO::output(step_pin, GPIO::LOW);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
  }
}

uint32_t Stepper::get_pid_error_and_set_direction(
    const double& elapsed_time_ms) {
  double error = target_steps.load() - current_steps.load();
  double derivative = 0;

  if (mads::control.load() != Control::MANUAL and mads::feedback.load() and
      !mads::turret_stopped.load()) {
    integral += error * elapsed_time_ms;
    derivative =
        -((current_steps.load() - previous_steps.load()) / elapsed_time_ms);
    previous_steps.store(current_steps.load());
    // derivative = (error - previous_error) / elapsed_time_ms;
    previous_error = error;
  } else {
    integral = 0;
    previous_error = 0;
  }

  double steps = K_P * error + K_I * integral + K_D * derivative;
  if (steps > 0) {
    GPIO::output(direction_pin, forward);
  } else {
    GPIO::output(direction_pin, backward);
  }

  return (abs(steps) < 250) ? abs(steps) : 250;
}

bool Stepper::step(const uint32_t& steps) {
  uint32_t auto_delay_us = 1000 / MICROSTEPS;
  for (uint32_t i = 0; i < steps; i++) {
    GPIO::output(step_pin, GPIO::HIGH);
    std::this_thread::sleep_for(std::chrono::microseconds(auto_delay_us));
    GPIO::output(step_pin, GPIO::LOW);
    if (new_setpoint.load() or new_feedback.load() or mads::exit_flag.load()) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(auto_delay_us));
  }
  return true;
}

void Stepper::run_stepper() {
  int lost_count = 0;
  auto start_time = std::chrono::high_resolution_clock::now();
  while (!mads::exit_flag.load()) {
    if (mads::control.load() != Control::MANUAL) {
      auto end_time = std::chrono::high_resolution_clock::now();
      double elapsed_time_ms =
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
              end_time - start_time)
              .count();
      uint32_t steps = get_pid_error_and_set_direction(elapsed_time_ms);
      start_time = end_time;

      step(steps);

      if (mads::laser_lost.load()) {
        if (lost_count++ < 10) {
          step(1);
          std::cout << this->name << " lost_cout = " << lost_count << std::endl;
          std::this_thread::sleep_for(std::chrono::milliseconds(300));
        } else {
          uint32_t auto_delay_us = 50000 / MICROSTEPS;
          bool fwrd = false;

          if (this->name == "x_stepper") {
            mads::x_centered.store(false);
          } else {
            mads::y_centered.store(false);
          }

          if (current_steps.load() > 0) {
            GPIO::output(direction_pin, backward);
            fwrd = false;
          } else {
            GPIO::output(direction_pin, forward);
            fwrd = true;
          }

          int l_steps = 0;
          while (!new_feedback.load() and std::abs(current_steps.load()) > 5 and
                 !mads::exit_flag.load() and
                 mads::control.load() != Control::MANUAL) {
            GPIO::output(step_pin, GPIO::HIGH);
            std::this_thread::sleep_for(
                std::chrono::microseconds(auto_delay_us));
            GPIO::output(step_pin, GPIO::LOW);
            std::this_thread::sleep_for(
                std::chrono::microseconds(auto_delay_us));
            if (fwrd) {
              current_steps.fetch_add(2);
            } else {
              current_steps.fetch_add(-2);
            }
            if (l_steps++ > 20) {
              std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
          }

          if (this->name == "x_stepper") {
            mads::x_centered.store(true);
          } else {
            mads::y_centered.store(true);
          }
          while (!mads::both_centered() and !mads::exit_flag.load() and
                 mads::control.load() != Control::MANUAL) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
          }
          std::cout << "current_steps = " << current_steps.load() << std::endl;
          std::cout << this->name << std::endl;
        }
      }

      if (new_feedback.load()) {
        lost_count = 0;
        correct_belief();
        new_feedback.store(false);
      } else {
        current_steps.store(target_steps.load());
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
  turret_origin_px.store(px);
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
  return turret_origin_px.load();
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