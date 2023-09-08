
#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include "utils.hpp"

namespace turret {

extern std::atomic<bool> run_flag;

struct Stepper {
  const std::string name;
  const uint8_t enable_pin;
  const uint8_t direction_pin;
  const uint8_t step_pin;

  // Camera intrinsic parameters
  const double principal_point;  // principal point (usually the image center).
  const double focal_length;     // focal lengths in pixel units.

  int32_t pos_step_limit;
  int32_t neg_step_limit;

  std::atomic<uint16_t> setpoint_px;
  std::atomic<uint16_t> detected_laser_px;
  std::atomic<bool> new_setpoint;
  std::atomic<bool> new_feedback;

  std::atomic<int8_t> direction;
  std::atomic<int32_t> prev_step_count;
  std::atomic<int32_t> step_count;
  std::atomic<int32_t> target_step_count;

  Stepper(std::string name,
          uint8_t enable_pin,
          uint8_t direction_pin,
          uint8_t step_pin,
          double c,
          double f);

  float get_belief_angle(void);
};

extern Stepper x_stepper;
extern Stepper y_stepper;

void init(void);
void run_stepper(Stepper& stepper);
void stop_all_motors(void);
utils::Circle get_turret_belief_region(void);
void keyboard_manual(int ch);
void keyboard_auto(int ch);

}  // namespace turret
