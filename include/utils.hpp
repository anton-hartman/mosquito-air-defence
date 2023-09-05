#pragma once

#include <cstdint>
#include <utility>
#include <vector>

namespace utils {

/**
 * @param x uint16_t
 * @param y uint16_t
 */
struct Point {
  uint16_t x;
  uint16_t y;
};

/**
 * @param x uint16_t
 * @param y uint16_t
 * @param radius uint16_t
 */
struct Circle {
  uint16_t x;
  uint16_t y;
  uint16_t radius;
};

/**
 * @brief Converts a pixel co-ordinates to a pair of x and y co-ordinates in mm.
 */
std::pair<float, float> pixel_to_mm(const Point& pixel_point,
                                    const std::vector<float> camera_matrix,
                                    const float camera_depth);

/**
 * @brief Converts a pair of stepper motor angles to pixels co-ordinates.
 */
std::pair<uint16_t, uint16_t> angle_to_pixel(
    const std::pair<float, float>& angle);

}  // namespace utils
