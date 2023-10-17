#include "../include/turret.hpp"
#include <JetsonGPIO.h>
#include "../include/frame.hpp"
#include "../include/mads.hpp"
#include "../include/stepper.hpp"

#define M1_ENABLE_PIN 32
#define M1_DIR_PIN 33
#define M1_STEP_PIN 35

#define M2_ENABLE_PIN 7
#define M2_DIR_PIN 18
#define M2_STEP_PIN 12

int MICROSTEPS;
double MICROSTEP_ANGLE_DEG;
double MICROSTEP_ANGLE_RAD;

void set_microsteps(int microsteps) {
  MICROSTEPS = microsteps;
  MICROSTEP_ANGLE_DEG = FULL_STEP_ANGLE_DEG / MICROSTEPS;
  MICROSTEP_ANGLE_RAD = MICROSTEP_ANGLE_DEG * M_PI / 180;
}

double K_P = 0.05;
double K_I = 0.0;
double K_D = 0.0;

Turret::Turret(void)
    : x_stepper("x_stepper",
                M1_ENABLE_PIN,
                M1_DIR_PIN,
                M1_STEP_PIN,
                1,
                0,
                X_STEPPER_DEPTH,
                mads::TURRET_X_ORIGIN_PX,
                mads::C_X_DOUBLE,
                mads::F_X),
      y_stepper("y_stepper",
                M2_ENABLE_PIN,
                M2_DIR_PIN,
                M2_STEP_PIN,
                0,
                1,
                Y_STEPPER_DEPTH,
                mads::TURRET_Y_ORIGIN_PX,
                mads::C_Y_DOUBLE,
                mads::F_Y) {
  GPIO::setmode(GPIO::BOARD);
  GPIO::setup(M1_ENABLE_PIN, GPIO::OUT, GPIO::LOW);
  GPIO::setup(M1_DIR_PIN, GPIO::OUT, GPIO::HIGH);
  GPIO::setup(M1_STEP_PIN, GPIO::OUT, GPIO::LOW);
  GPIO::setup(M2_ENABLE_PIN, GPIO::OUT, GPIO::LOW);
  GPIO::setup(M2_DIR_PIN, GPIO::OUT, GPIO::HIGH);
  GPIO::setup(M2_STEP_PIN, GPIO::OUT, GPIO::LOW);
}

void Turret::save_steps_at_frame(void) {
  x_stepper.save_steps();
  y_stepper.save_steps();
}

void Turret::set_origin(const Pt turret_origin_px) {
  x_stepper.set_origin_px(turret_origin_px.x);
  y_stepper.set_origin_px(turret_origin_px.y);
  update_belief(turret_origin_px);
  update_setpoint(turret_origin_px);
}

void Turret::run_x_stepper(void) {
  x_stepper.run_stepper();
}

void Turret::run_y_stepper(void) {
  y_stepper.run_stepper();
}

void Turret::stop_turret(void) {
  x_stepper.stop_stepper();
  y_stepper.stop_stepper();
}

void Turret::start_turret(void) {
  x_stepper.enable_stepper();
  y_stepper.enable_stepper();
}

Pt Turret::get_origin_px(void) const {
  return Pt{x_stepper.get_origin_px(), y_stepper.get_origin_px()};
}

Pt Turret::get_belief_px(void) const {
  return Pt{x_stepper.get_current_px(), y_stepper.get_current_px()};
}

Pt Turret::get_setpoint_px(void) const {
  return Pt{x_stepper.get_target_px(), y_stepper.get_target_px()};
}

Pt Turret::get_belief_steps(void) const {
  return Pt{x_stepper.get_current_steps(), y_stepper.get_current_steps()};
}

Pt Turret::get_setpoint_steps(void) const {
  return Pt{x_stepper.get_target_steps(), y_stepper.get_target_steps()};
}

void Turret::home(const Pt detected_laser_px) {
  x_stepper.set_target_px(detected_laser_px.x);
  y_stepper.set_target_px(detected_laser_px.y);
  x_stepper.set_detected_laser_px(detected_laser_px.x);
  y_stepper.set_detected_laser_px(detected_laser_px.y);
  x_stepper.home();
  y_stepper.home();
}

void Turret::update_setpoint(const Pt setpoint_px) {
  if (setpoint_px.x < 0 or setpoint_px.y < 0 or setpoint_px.x > COLS or
      setpoint_px.y > ROWS) {
    return;
  }
  x_stepper.set_target_px(setpoint_px.x);
  y_stepper.set_target_px(setpoint_px.y);
}

void Turret::update_belief(const Pt detected_laser_px) {
  if (detected_laser_px.x < 0 or detected_laser_px.y < 0 or
      detected_laser_px.x > COLS or detected_laser_px.y > ROWS) {
    return;
  }
  x_stepper.set_detected_laser_px(detected_laser_px.x);
  y_stepper.set_detected_laser_px(detected_laser_px.y);
}

void Turret::keyboard_auto(int ch, int px) {
  switch (ch) {
    case 'w':
      update_setpoint(
          Pt{x_stepper.get_target_px(), y_stepper.get_target_px() - px});
      break;
    case 's':
      update_setpoint(
          Pt{x_stepper.get_target_px(), y_stepper.get_target_px() + px});
      break;
    case 'a':
      update_setpoint(
          Pt{x_stepper.get_target_px() - px, y_stepper.get_target_px()});
      break;
    case 'd':
      update_setpoint(
          Pt{x_stepper.get_target_px() + px, y_stepper.get_target_px()});
      break;
    default:
      break;
  }
}

void Turret::keyboard_manual(int ch, int steps) {
  switch (ch) {
    case 'w':
      y_stepper.step_manually(-steps);
      break;
    case 's':
      y_stepper.step_manually(steps);
      break;
    case 'a':
      x_stepper.step_manually(-steps);
      break;
    case 'd':
      x_stepper.step_manually(steps);
      break;
    default:
      break;
  }
}