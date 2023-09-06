
#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include "utils.hpp"

namespace turret {

struct Stepper {
  std::string name;
  const uint8_t enable_pin;
  const uint8_t direction_pin;
  const uint8_t step_pin;
  int8_t direction = 1;
  int32_t pos_step_limit = 1'000'000;
  int32_t neg_step_limit = -1'000'000;
  std::atomic<int32_t> prev_step_count{0};
  std::atomic<int32_t> step_count{0};
  std::atomic<int32_t> target_step_count{0};

  Stepper(std::string name,
          uint8_t enable_pin,
          uint8_t direction_pin,
          uint8_t step_pin);
};

extern Stepper x_stepper;
extern Stepper y_stepper;

void init(void);
void home_steppers(void);
void run_stepper(Stepper& stepper);
void stop_all_motors(void);
utils::Circle get_laser_belief_region(void);
void correct_laser_belief(
    const std::pair<uint16_t, uint16_t>& laser_detected_px);
void update_target(const std::pair<uint16_t, uint16_t>& target_px);
void keyboard_manual(int ch);
void keyboard_auto(int ch);

}  // namespace turret
