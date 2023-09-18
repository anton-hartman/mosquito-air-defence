#pragma once

#include <atomic>
#include <cmath>  // For M_PI
#include <cstdint>
#include <string>

const double FULL_STEP_ANGLE_DEG = 0.17578125;
const uint8_t MICROSTEPS = 16;
const double MICROSTEP_ANGLE_DEG = FULL_STEP_ANGLE_DEG / MICROSTEPS;
const double MICROSTEP_ANGLE_RAD = MICROSTEP_ANGLE_DEG * M_PI / 180;

class Stepper {
 private:
  const std::string name;
  const uint8_t enable_pin;
  const uint8_t direction_pin;
  const uint8_t step_pin;

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
  std::atomic<bool> manual;

  std::atomic<int8_t> direction;
  std::atomic<int32_t> steps_at_detection;
  std::atomic<int32_t> current_steps;
  std::atomic<int32_t> target_steps;

  int32_t pixel_to_steps(const uint16_t& px) const;
  uint16_t steps_to_pixel(const int32_t& steps) const;

  /**
   * @return The absolute value of the steps to take and sets the direction
   */
  uint32_t get_steps_and_set_direction();

  void correct_belief();
  void update_target_steps();

 public:
  Stepper(std::string name,
          uint8_t enable_pin,
          uint8_t direction_pin,
          uint8_t step_pin,
          float depth,
          uint16_t origin_px,
          double c,
          double f);

  void run_stepper(void);
  void stop_stepper(void);
  void enable_stepper(void);
  void save_steps();

  /**
   * @note Should only be used for testing
   */
  void increment_setpoint_in_steps(const int32_t steps);

  void set_manual(const bool manual);
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