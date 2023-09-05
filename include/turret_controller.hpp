
#pragma once

#include <cstdint>
#include <mutex>
#include "utils.hpp"

namespace turret {

struct Stepper {
  uint8_t enable_pin;
  uint8_t direction_pin;
  uint8_t step_pin;
  uint8_t direction = 0;
  int32_t step_count = 0;
  int32_t target_step_count = 0;
  int32_t pos_step_limit = 1'000'000;
  int32_t neg_step_limit = -1'000'000;
};

Stepper x_stepper;
Stepper y_stepper;

void init(void);
utils::Circle get_laser_belief_region(void);
void correct_laser_belief_region(const utils::Point& laser_detected_px);
void update_target(const utils::Point& target_px);
void update_target_steps(const int x_steps, const int y_steps);
void run_stepper(Stepper& stepper);
void home_steppers(void);
void stop_all_motors(void);

}  // namespace turret
