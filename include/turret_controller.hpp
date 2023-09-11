
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

  float get_belief_angle(void) const;
};

extern Stepper x_stepper;
extern Stepper y_stepper;

void init(void);
std::pair<uint16_t, uint16_t> get_belief_px(void);
std::pair<uint16_t, uint16_t> get_setpoint_px(void);
std::pair<uint16_t, uint16_t> get_target_px(void);
void update_belief(const std::pair<uint16_t, uint16_t> detected_laser_px);
void update_setpoint(const std::pair<uint16_t, uint16_t> setpoint_px);
void run_stepper(Stepper& stepper);
void stop_all_motors(void);
utils::Circle get_belief_region(void);
void keyboard_auto(int ch);
void keyboard_manual(int ch);

}  // namespace turret
