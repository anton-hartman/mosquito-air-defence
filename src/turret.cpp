
#include "../include/turret.hpp"
#include <JetsonGPIO.h>
#include "../include/stepper.hpp"
#include "../include/utils.hpp"

#define M1_ENABLE_PIN 32
#define M1_DIR_PIN 33
#define M1_STEP_PIN 35

#define M2_ENABLE_PIN 7
#define M2_DIR_PIN 18
#define M2_STEP_PIN 12

const double F_X = 647.0756309728268;
const double F_Y = 861.7363873209705;
const double C_X = 304.4404590127848;
const double C_Y = 257.5858878142162;

const int TURRET_DEPTH = 550;                      // mm
const int VERTICAL_DISTANCE_BETWEEN_MIRRORS = 50;  // mm
const int X_STEPPER_DEPTH =
    TURRET_DEPTH + VERTICAL_DISTANCE_BETWEEN_MIRRORS + utils::TANK_DEPTH;  // mm
const int Y_STEPPER_DEPTH = TURRET_DEPTH + utils::TANK_DEPTH;              // mm

Turret::Turret(void)
    : run_flag(true),
      x_stepper("x_stepper",
                M1_ENABLE_PIN,
                M1_DIR_PIN,
                M1_STEP_PIN,
                X_STEPPER_DEPTH,
                C_X,
                F_X),
      y_stepper("y_stepper",
                M2_ENABLE_PIN,
                M2_DIR_PIN,
                M2_STEP_PIN,
                Y_STEPPER_DEPTH,
                C_Y,
                F_Y) {
  GPIO::setmode(GPIO::BOARD);
  GPIO::setup(M1_ENABLE_PIN, GPIO::OUT, GPIO::LOW);
  GPIO::setup(M1_DIR_PIN, GPIO::OUT, GPIO::HIGH);
  GPIO::setup(M1_STEP_PIN, GPIO::OUT, GPIO::LOW);
  GPIO::setup(M2_ENABLE_PIN, GPIO::OUT, GPIO::LOW);
  GPIO::setup(M2_DIR_PIN, GPIO::OUT, GPIO::HIGH);
  GPIO::setup(M2_STEP_PIN, GPIO::OUT, GPIO::LOW);
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

std::pair<uint16_t, uint16_t> Turret::get_belief_px(void) {
  return std::pair<uint16_t, uint16_t>(x_stepper.get_current_px(),
                                       y_stepper.get_current_px());
}

std::pair<uint16_t, uint16_t> Turret::get_setpoint_px(void) {
  return std::pair<uint16_t, uint16_t>(x_stepper.get_target_px(),
                                       y_stepper.get_target_px());
}

std::pair<int32_t, int32_t> Turret::get_belief_steps(void) {
  return std::pair<int32_t, int32_t>(x_stepper.get_current_steps(),
                                     y_stepper.get_current_steps());
}

std::pair<int32_t, int32_t> Turret::get_setpoint_steps(void) {
  return std::pair<int32_t, int32_t>(x_stepper.get_target_steps(),
                                     y_stepper.get_target_steps());
}

void Turret::update_setpoint(const std::pair<uint16_t, uint16_t> setpoint_px) {
  x_stepper.set_target_px(setpoint_px.first);
  y_stepper.set_target_px(setpoint_px.second);
}

void Turret::update_belief(
    const std::pair<uint16_t, uint16_t> detected_laser_px) {
  x_stepper.set_detected_laser_px(detected_laser_px.first);
  y_stepper.set_detected_laser_px(detected_laser_px.second);
}

void Turret::keyboard_auto(int ch, int px) {
  switch (ch) {
    case 'w':
      update_setpoint(std::pair<uint16_t, uint16_t>(
          x_stepper.get_target_px(), y_stepper.get_target_px() - px));
      break;
    case 's':
      update_setpoint(std::pair<uint16_t, uint16_t>(
          x_stepper.get_target_px(), y_stepper.get_target_px() + px));
      break;
    case 'a':
      update_setpoint(std::pair<uint16_t, uint16_t>(
          x_stepper.get_target_px() - px, y_stepper.get_target_px()));
      break;
    case 'd':
      update_setpoint(std::pair<uint16_t, uint16_t>(
          x_stepper.get_target_px() + px, y_stepper.get_target_px()));
      break;
    default:
      break;
  }
}

void Turret::keyboard_manual(int ch, int steps) {
  switch (ch) {
    case 'w':
      y_stepper.increment_setpoint_in_steps(-steps);
      break;
    case 's':
      y_stepper.increment_setpoint_in_steps(steps);
      break;
    case 'a':
      x_stepper.increment_setpoint_in_steps(-steps);
      break;
    case 'd':
      x_stepper.increment_setpoint_in_steps(steps);
      break;
    default:
      break;
  }
}