
#pragma once

#include <ncurses.h>
#include <cstdint>
#include <utility>

namespace turret {

void manual_control(int ch);
void auto_control(std::pair<int, int> actual_pos,
                  std::pair<int, int> target_pos);
void home_steppers(void);

}  // namespace turret
