#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include "turret_controller.hpp"

// class Turret;

class Stepper {
 private:
  const std::string name;
  const uint8_t enable_pin;
  const uint8_t direction_pin;
  const uint8_t step_pin;

  const double depth;  // mm

  // Camera intrinsic parameters
  const double principal_point;  // principal point (usually the image center).
  const double focal_length;     // focal lengths in pixel units.

  int32_t pos_step_limit;
  int32_t neg_step_limit;

  std::atomic<uint16_t> target_px;
  std::atomic<uint16_t> detected_laser_px;
  std::atomic<bool> new_setpoint;
  std::atomic<bool> new_feedback;

  std::atomic<int8_t> direction;
  std::atomic<int32_t> current_steps;
  std::atomic<int32_t> target_steps;

  void enable_motor();
  void stop_motor();

  int32_t pixel_to_steps(const uint16_t& px) const;
  uint16_t steps_to_pixel(const int32_t& steps) const;

  /**
   * @return The absolute value of the steps to take and sets the direction
   */
  uint32_t get_steps_and_set_direction();

 public:
  Stepper(std::string name,
          uint8_t enable_pin,
          uint8_t direction_pin,
          uint8_t step_pin,
          float depth,
          double c,
          double f);

  void correct_belief();
  void setpoint_to_steps();

  /**
   * @note Should only be used for testing
   */
  void increment_setpoint_in_steps(const int32_t steps);

  void run_stepper(void);

  void set_target_px(const uint16_t px);
  void set_detected_laser_px(const uint16_t px);

  uint16_t get_target_px(void);
  uint16_t get_detected_laser_px(void);
  int32_t get_current_steps(void);
  int32_t get_target_steps(void);

  friend void Turret::stop_all_motors(void);
};