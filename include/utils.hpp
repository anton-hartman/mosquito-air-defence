#pragma once

#include <cstdint>
#include <utility>

// Forward declaration to break the circular dependency
namespace turret {
struct Stepper;
}

namespace utils {

/**
 * @param x uint16_t
 * @param y uint16_t
 * @param radius uint16_t
 */
struct Circle {
  uint16_t x;
  uint16_t y;
  uint16_t radius;

  Circle() {}

  Circle(uint16_t x, uint16_t y, uint16_t radius)
      : x(x), y(y), radius(radius) {}

  Circle(std::pair<uint16_t, uint16_t> center)
      : x(center.first), y(center.second) {}
};

/**
 * @return depth * tan(theta)
 */
float angle_to_mm(float theta, float depth);

/**
 * @return arctan(mm / depth)
 */
float mm_to_angle(float mm, float depth);

/**
 * Convert a pixel coordinate to a real-world coordinate in millimeters.
 *
 * @param px Pixel coordinate with origin at top-left of the image.
 * @param depth Known distance from the camera to the plane in millimeters.
 *
 * @return A float containing the distance from the priciple point in
 * millimeters.
 *
 * @note - The input pixel coordinate is based on an origin at the top-left
 * corner of the image.
 * @note - The returned real-world coordinate has its origin at the optical
 * center of the camera. This optical center is defined by the intrinsic
 * parameters (c_x, c_y) from the camera matrix. Positive X values extend to the
 * right of the optical center, negative X values extend to the left. Positive Y
 * values extend downward from the optical center, and negative Y values extend
 * upward.
 */
float pixel_to_mm(const turret::Stepper& stepper,
                  const uint16_t px,
                  const uint16_t depth);

/**
 * Convert real-world a coordinate in millimeters to a pixel coordinate. All
 * parameters must be in millimeters
 *
 * @param mm Real-world coordinate with origin at the optical center of the
 * camera.
 * @param depth Known distance from the camera to the plane in millimeters.
 *
 * @return A uint16_t containing the coordinate in pixels.
 *
 * @note - The input real-world coordinate is based on an origin at the optical
 * center of the camera.
 * @note - The returned pixel coordinate has its origin at the top-left corner
 * of the image.
 */
uint16_t mm_to_pixel(const turret::Stepper& stepper,
                     const uint16_t mm,
                     const uint16_t depth);

}  // namespace utils
