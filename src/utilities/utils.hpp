#pragma once

#include <cstdint>

namespace utils {

/**
 * @brief Delays for the specified milliseconds considering microsteps.
 *
 * @param ms Milliseconds to delay.
 * @param microsteps Number of microsteps.
 */
void microstep_delay_ms(uint32_t ms, uint8_t microsteps);

/**
 * @brief Delays for the specified milliseconds.
 *
 * @param ms Milliseconds to delay.
 */
void delay_ms(uint32_t ms);

/**
 * @brief Delays for the specified microseconds.
 *
 * @param us Microseconds to delay.
 */
void delay_us(uint32_t us);

}  // namespace utils
