#pragma once

#include <atomic>

namespace sys {

extern std::atomic<bool> exit_flag;
extern std::atomic<bool> run_flag;
extern std::atomic<bool> keyboard_manual_mode;

}  // namespace sys