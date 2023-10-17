#pragma once

#include <atomic>
#include "turret.hpp"

class sys {
 private:
  static std::atomic<bool> laser_state;

 public:
  static std::atomic<bool> exit_flag;
  static std::atomic<bool> run_flag;
  static std::atomic<bool> keyboard_manual_mode;

  static void set_laser(bool on) {
    if (on) {
      Turret::enable_laser();
      laser_state.store(true);
    } else {
      Turret::disable_laser();
      laser_state.store(false);
    }
  }

  static bool get_laser_state(void) { return laser_state.load(); }
};