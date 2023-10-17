
#pragma once

#include "pt.hpp"
#include "stepper.hpp"

// constexpr double FULL_STEP_ANGLE_DEG = 0.17578125; // For cheapie
constexpr double FULL_STEP_ANGLE_DEG = 1.8;
extern int MICROSTEPS;
extern double MICROSTEP_ANGLE_DEG;
extern double MICROSTEP_ANGLE_RAD;

void set_microsteps(int microsteps);

extern double K_P;
extern double K_I;
extern double K_D;

class Turret {
 public:
  static const int TANK_DEPTH = 318;
  static const int CAMERA_DEPTH = 793 + TANK_DEPTH;  // mm
  static const int TURRET_DEPTH = 395 + TANK_DEPTH;
  static const int VERTICAL_DISTANCE_BETWEEN_MIRRORS = 14;
  static const int Y_STEPPER_DEPTH = TURRET_DEPTH;
  static const int X_STEPPER_DEPTH =
      TURRET_DEPTH + VERTICAL_DISTANCE_BETWEEN_MIRRORS;

 private:
  Stepper x_stepper;
  Stepper y_stepper;

 public:
  Turret(void);

  void save_steps_at_frame(void);
  void set_origin(const Pt turret_origin_px);

  Pt get_origin_px(void) const;
  Pt get_belief_px(void) const;
  Pt get_setpoint_px(void) const;
  Pt get_belief_steps(void) const;
  Pt get_setpoint_steps(void) const;

  void home(const Pt detected_laser_px);
  void update_belief(const Pt detected_laser_px);
  void update_setpoint(const Pt setpoint_px);

  void run_x_stepper(void);
  void run_y_stepper(void);
  void stop_turret(void);
  void start_turret(void);

  void keyboard_auto(int ch, int px);
  void keyboard_manual(int ch, int steps);
};
