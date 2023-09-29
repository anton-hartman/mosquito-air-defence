
#include "../include/turret.hpp"
#include <JetsonGPIO.h>
#include "../include/frame.hpp"
#include "../include/stepper.hpp"
#include "../include/utils.hpp"

#define M1_ENABLE_PIN 32
#define M1_DIR_PIN 33
#define M1_STEP_PIN 35

#define M2_ENABLE_PIN 7
#define M2_DIR_PIN 18
#define M2_STEP_PIN 12

Turret::Turret(void)
    : run_flag(true),
      x_stepper("x_stepper",
                M1_ENABLE_PIN,
                M1_DIR_PIN,
                M1_STEP_PIN,
                X_STEPPER_DEPTH,
                X_ORIGIN_PX,
                C_X,
                F_X),
      y_stepper("y_stepper",
                M2_ENABLE_PIN,
                M2_DIR_PIN,
                M2_STEP_PIN,
                Y_STEPPER_DEPTH,
                Y_ORIGIN_PX,
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

void Turret::set_manual_mode(const bool manual_mode) {
  x_stepper.set_manual(manual_mode);
  y_stepper.set_manual(manual_mode);
}

void Turret::save_steps_at_frame() {
  x_stepper.save_steps();
  y_stepper.save_steps();
}

void Turret::set_origin(const std::pair<uint16_t, uint16_t> turret_origin_px) {
  x_stepper.set_origin_px(turret_origin_px.first);
  y_stepper.set_origin_px(turret_origin_px.second);
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

std::pair<uint16_t, uint16_t> Turret::get_origin_px(void) const {
  return std::pair<uint16_t, uint16_t>(x_stepper.get_origin_px(),
                                       y_stepper.get_origin_px());
}

std::pair<uint16_t, uint16_t> Turret::get_belief_px(void) const {
  return std::pair<uint16_t, uint16_t>(x_stepper.get_current_px(),
                                       y_stepper.get_current_px());
}

std::pair<uint16_t, uint16_t> Turret::get_setpoint_px(void) const {
  return std::pair<uint16_t, uint16_t>(x_stepper.get_target_px(),
                                       y_stepper.get_target_px());
}

std::pair<int32_t, int32_t> Turret::get_belief_steps(void) const {
  return std::pair<int32_t, int32_t>(x_stepper.get_current_steps(),
                                     y_stepper.get_current_steps());
}

std::pair<int32_t, int32_t> Turret::get_setpoint_steps(void) const {
  return std::pair<int32_t, int32_t>(x_stepper.get_target_steps(),
                                     y_stepper.get_target_steps());
}

void Turret::update_setpoint(const std::pair<uint16_t, uint16_t> setpoint_px) {
  x_stepper.set_target_px(setpoint_px.first);
  y_stepper.set_target_px(setpoint_px.second);
}

// void Turret::update_belief(BoundingBoxMap bounding_boxes) {
//   for (const std::pair<int, Rectangle>& element : bounding_boxes) {
//     int label = element.first;
//     const Rectangle& rectangle = element.second;
//     const Point& min_point = rectangle.first;
//     const Point& max_point = rectangle.second;
//     std::cout << "Label: " << label << " Bounding Rectangle: {{"
//               << min_point.first << ", " << min_point.second << "}, {"
//               << max_point.first << ", " << max_point.second << "}}\n";
//   }
// }

void Turret::update_belief(
    const std::pair<int32_t, int32_t> detected_laser_px) {
  if (detected_laser_px.first < 0 && detected_laser_px.second < 0) {
    return;
  }
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