#pragma once

#include <atomic>
#include <cmath>  // For M_PI
#include <cstdint>
#include <string>

class Stepper {
 private:
  const std::string name;
  const uint8_t enable_pin;
  const uint8_t direction_pin;
  const uint8_t step_pin;

  const uint8_t gpio_clockwise;
  const uint8_t gpio_anticlockwise;

  const double depth;                      // mm
  std::atomic<uint16_t> turret_origin_px;  // The origin of the turret in pixels

  // Camera intrinsic parameters
  const double principal_point;  // principal point (usually the image center).
  const double focal_length;     // focal lengths in pixel units.

  std::atomic<uint16_t> target_px;
  std::atomic<uint16_t> detected_laser_px;
  std::atomic<bool> new_setpoint;
  std::atomic<bool> new_feedback;

  std::atomic<int32_t> steps_at_detection;
  std::atomic<int32_t> current_steps;
  std::atomic<int32_t> previous_steps;
  std::atomic<int32_t> target_steps;

  // For PID control
  double previous_error;
  double integral;

  /**
   * Convert a pixel coordinate to a real-world coordinate in millimeters.
   *
   * @param px Pixel coordinate with origin at top-left of the image.
   *
   * @return A float containing the distance from the priciple point in
   * millimeters.
   *
   * @note - The input pixel coordinate is based on an origin at the top-left
   * corner of the image.
   * @note - The returned real-world coordinate has its origin at the optical
   * center of the camera. This optical center is defined by the intrinsic
   * parameters (c_x, c_y) from the camera matrix. Positive X values extend to
   * the right of the optical center, negative X values extend to the left.
   * Positive Y values extend downward from the optical center, and negative Y
   * values extend upward.
   */
  int32_t pixel_to_steps(const uint16_t& px) const;

  /**
   * Convert real-world a coordinate in millimeters to a pixel coordinate. All
   * parameters must be in millimeters
   *
   * @param mm Real-world coordinate with origin at the optical center of the
   * camera.
   *
   * @return A uint16_t containing the coordinate in pixels.
   *
   * @note - The input real-world coordinate is based on an origin at the
   * optical center of the camera.
   * @note - The returned pixel coordinate has its origin at the top-left corner
   * of the image.
   */
  uint16_t steps_to_pixel(const int32_t& steps) const;

  uint32_t get_pid_error_and_set_direction(const double& elapsed_time_ms);

  void correct_belief();
  void update_target_steps();
  bool step(const uint32_t& steps);

 public:
  Stepper(std::string name,
          uint8_t enable_pin,
          uint8_t direction_pin,
          uint8_t step_pin,
          uint8_t gpio_clockwise,
          uint8_t gpio_anticlockwise,
          float depth,
          uint16_t turret_origin_px,
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