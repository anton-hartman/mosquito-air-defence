
#pragma once

#include <atomic>
#include <cstdint>
#include <utility>
#include "stepper.hpp"

class Turret {
 private:
  std::atomic<bool> run_flag;

  Stepper x_stepper;
  Stepper y_stepper;

 public:
  Turret(void);

  void set_origin(const std::pair<uint16_t, uint16_t> turret_origin_px);

  std::pair<uint16_t, uint16_t> get_origin_px(void) const;
  std::pair<uint16_t, uint16_t> get_belief_px(void) const;
  std::pair<uint16_t, uint16_t> get_setpoint_px(void) const;
  std::pair<int32_t, int32_t> get_belief_steps(void) const;
  std::pair<int32_t, int32_t> get_setpoint_steps(void) const;

  void update_belief(const std::pair<uint16_t, uint16_t> detected_laser_px);
  void update_setpoint(const std::pair<uint16_t, uint16_t> setpoint_px);

  void run_x_stepper(void);
  void run_y_stepper(void);
  void stop_turret(void);

  void keyboard_auto(int ch, int px);
  void keyboard_manual(int ch, int steps);
};
