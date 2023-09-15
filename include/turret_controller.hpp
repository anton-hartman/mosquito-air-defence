
#pragma once

#include <atomic>
#include <cstdint>
#include <utility>
// #include "stepper.hpp"

class Stepper;
class Turret {
 private:
  std::atomic<bool> run_flag;

  Stepper x_stepper;
  Stepper y_stepper;

 public:
  Turret(void);

  std::pair<uint16_t, uint16_t> get_belief_px(void);
  std::pair<uint16_t, uint16_t> get_setpoint_px(void);
  std::pair<int32_t, int32_t> get_belief_steps(void);
  std::pair<int32_t, int32_t> get_setpoint_steps(void);

  void update_belief(const std::pair<uint16_t, uint16_t> detected_laser_px);
  void update_setpoint(const std::pair<uint16_t, uint16_t> setpoint_px);

  void run_x_stepper(void);
  void run_y_stepper(void);
  void stop_all_motors(void);

  void keyboard_auto(int ch);
  void keyboard_manual(int ch, int steps);
};
