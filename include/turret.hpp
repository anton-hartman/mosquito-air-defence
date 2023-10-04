
#pragma once

#include <atomic>
#include <cstdint>
#include <utility>
#include "stepper.hpp"
#include "two_pass_algorithm.hpp"

class Turret {
 public:
  static const uint16_t TANK_DEPTH = 318;
  // static const int TANK_DEPTH = -2;
  static const uint16_t CAMERA_DEPTH = 793 + TANK_DEPTH;  // mm
  static const int TURRET_DEPTH = 575 + TANK_DEPTH;
  static const int VERTICAL_DISTANCE_BETWEEN_MIRRORS = 10;
  static const int Y_STEPPER_DEPTH = TURRET_DEPTH;
  static const int X_STEPPER_DEPTH =
      TURRET_DEPTH + VERTICAL_DISTANCE_BETWEEN_MIRRORS;

  // For PID control
  static constexpr double K_P = 0.5;
  static constexpr double K_I = 0.001;
  static constexpr double K_D = 0.0;

 private:
  std::atomic<bool> run_flag;

  Stepper x_stepper;
  Stepper y_stepper;

 public:
  Turret(void);

  void set_manual_mode(const bool manual_mode);
  void save_steps_at_frame();
  void set_origin(const std::pair<uint16_t, uint16_t> turret_origin_px);

  std::pair<uint16_t, uint16_t> get_origin_px(void) const;
  std::pair<uint16_t, uint16_t> get_belief_px(void) const;
  std::pair<uint16_t, uint16_t> get_setpoint_px(void) const;
  std::pair<int32_t, int32_t> get_belief_steps(void) const;
  std::pair<int32_t, int32_t> get_setpoint_steps(void) const;

  void update_belief(const std::pair<int32_t, int32_t> detected_laser_px);
  void update_setpoint(const std::pair<uint16_t, uint16_t> setpoint_px);

  void run_x_stepper(void);
  void run_y_stepper(void);
  void stop_turret(void);
  void start_turret(void);

  void keyboard_auto(int ch, int px);
  void keyboard_manual(int ch, int steps);
};
