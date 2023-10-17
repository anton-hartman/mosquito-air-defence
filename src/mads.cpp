#include "../include/mads.hpp"
#include <cmath>

const int mads::C_X = std::round(C_X_DOUBLE);
const int mads::C_Y = std::round(C_Y_DOUBLE);

std::atomic<bool> mads::exit_flag(false);
std::atomic<bool> mads::feedback(false);
std::atomic<bool> mads::laser(false);
std::atomic<bool> mads::turret_stopped(true);
std::atomic<Control> mads::control(Control::MANUAL);