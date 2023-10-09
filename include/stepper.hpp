#pragma once

#include <atomic>
#include <cmath>  // For M_PI
#include <cstdint>
#include <string>

const double FULL_STEP_ANGLE_DEG = 0.17578125;
const uint8_t MICROSTEPS = 32;
const double MICROSTEP_ANGLE_DEG = FULL_STEP_ANGLE_DEG / MICROSTEPS;
const double MICROSTEP_ANGLE_RAD = MICROSTEP_ANGLE_DEG * M_PI / 180;

class Stepper {
 private:
  const std::string name;
  const uint8_t enable_pin;
  const uint8_t direction_pin;
  const uint8_t step_pin;

  const uint8_t gpio_clockwise;
  const uint8_t gpio_anticlockwise;

  const double depth;               // mm
  std::atomic<uint16_t> origin_px;  // The origin of the turret in pixels

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
  std::atomic<int32_t> steps_at_detection;
  std::atomic<int32_t> current_steps;
  std::atomic<int32_t> target_steps;

  // For PID control
  int32_t previous_error;
  int32_t integral;

  int32_t pixel_to_steps(const uint16_t& px) const;
  uint16_t steps_to_pixel(const int32_t& steps) const;

  uint32_t get_pid_error_and_set_direction();

  void correct_belief();
  void update_target_steps();

 public:
  Stepper(std::string name,
          uint8_t enable_pin,
          uint8_t direction_pin,
          uint8_t step_pin,
          uint8_t gpio_clockwise,
          uint8_t gpio_anticlockwise,
          float depth,
          uint16_t origin_px,
          double c,
          double f);

  void run_stepper(void);
  void stop_stepper(void);
  void enable_stepper(void);
  void save_steps();

  void step_manually(const int32_t steps);

  void home(void);
  void set_origin_px(const uint16_t px);
  void set_target_px(const uint16_t px);
  void set_detected_laser_px(const uint16_t px);

  uint16_t get_origin_px(void) const;
  uint16_t get_target_px(void) const;
  uint16_t get_detected_laser_px(void) const;
  uint16_t get_current_px(void) const;

  int32_t get_current_steps(void) const;
  int32_t get_target_steps(void) const;
};