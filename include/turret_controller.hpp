
#pragma once

#include <utility>  // For std::pair<int, int>

namespace turret {

void init(void);
void manual_control(int ch);
void auto_control(std::pair<int, int> detected_angle,
                  std::pair<int, int> target_angle);
void home_steppers(void);
void stop_all_motors(void);

}  // namespace turret
