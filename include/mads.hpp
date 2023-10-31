#pragma once

#include <JetsonGPIO.h>
#include <atomic>

enum class Control { MANUAL, FULL_AUTO, KEYBOARD_AUTO };
enum Display {
  OFF = 0,
  LASER_DETECTION = 1 << 0,
  MOSQUITO_DETECTION = 1 << 1,
  ALL_DETECTIONS = LASER_DETECTION | MOSQUITO_DETECTION,
  TRACKING = 1 << 2,
  ALL = ALL_DETECTIONS | TRACKING
};

namespace Debug {
enum DebugEnum {
  OFF = 0,
  BG_SUB = 1 << 0,
  MORPH = 1 << 1,
  TRACKING = 1 << 2,
  ALL = BG_SUB | MORPH | TRACKING
};
}

class mads {
 private:
  static std::atomic<bool> laser;

 public:
  static constexpr int LASER_PIN = 11;
  static constexpr double F_X = 1279.13149855341;
  static constexpr double F_Y = 1246.965909876756;
  static constexpr double C_X_DOUBLE = 457.9588295305912;
  static constexpr double C_Y_DOUBLE = 240.0948537167988;
  static const int C_X;
  static const int C_Y;
  static constexpr int TURRET_X_ORIGIN_PX = 551;
  static constexpr int TURRET_Y_ORIGIN_PX = 211;

  static std::atomic<bool> exit_flag;
  static std::atomic<Control> control;
  static std::atomic<bool> feedback;
  static std::atomic<bool> turret_stopped;
  static std::atomic<Display> display;
  static std::atomic<Debug::DebugEnum> debug;
  static std::atomic<bool> laser_lost;
  static std::atomic<bool> x_centered;
  static std::atomic<bool> y_centered;

  static void set_laser(bool on) {
    if (on) {
      GPIO::output(LASER_PIN, GPIO::HIGH);
      laser.store(true);
    } else {
      GPIO::output(LASER_PIN, GPIO::LOW);
      laser.store(false);
    }
  }

  static bool both_centered() {
    if (x_centered.load() and y_centered.load()) {
      return true;
    } else {
      return false;
    }
  }

  static bool get_laser_state(void) { return laser.load(); }
};