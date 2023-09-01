#include "utils.hpp"

namespace utils {

void microstep_delay_ms(uint32_t ms, uint8_t microsteps) {
  int basedelay = 50000 / microsteps;
  for (int j = ms; j > 0; j--)
    for (int i = basedelay; i > 0; i--)
      ;
}

void delay_ms(uint32_t ms) {
  microstep_delay_ms(ms, 1);
}

void delay_us(uint32_t us) {
  int j;
  for (j = us; j > 0; j--)
    ;
}

}  // namespace utils
