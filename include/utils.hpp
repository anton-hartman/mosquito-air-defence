#pragma once

#include <atomic>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <utility>

namespace utils {

extern std::atomic<bool> exit_flag;
extern std::atomic<bool> run_flag;
extern std::atomic<bool> manual_mode;

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

void draw_target(cv::Mat& frame,
                 const std::pair<uint16_t, uint16_t>& target,
                 const cv::Scalar& colour);

/**
 * @brief Puts a text label on an image.
 *
 * This function places a given text label on a specified image at the given
 * origin point. The label is typically used for marking and identification
 * purposes on image displays.
 *
 * @param img The image on which the label will be placed.
 * @param label The text string to be placed on the image.
 * @param origin The top-left corner of the text string in the image.
 *               For example, a point (10,30) means the text starts 10 pixels
 * from the left and 30 pixels from the top of the image.
 */
void put_label(cv::Mat& img,
               const std::string& label,
               const std::pair<uint16_t, uint16_t>& origin,
               const double& font_scale = 1);

/**
 * Convert a pixel coordinate to a real-world coordinate in millimeters.
 *
 * @param px Pixel coordinate with origin at top-left of the image.
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
double pixel_to_mm(const double principal_point,
                   const double focal_length,
                   const uint16_t& px);

/**
 * Convert real-world a coordinate in millimeters to a pixel coordinate. All
 * parameters must be in millimeters
 *
 * @param mm Real-world coordinate with origin at the optical center of the
 * camera.
 *
 * @return A uint16_t containing the coordinate in pixels.
 *
 * @note - The input real-world coordinate is based on an origin at the optical
 * center of the camera.
 * @note - The returned pixel coordinate has its origin at the top-left corner
 * of the image.
 */
uint16_t mm_to_pixel(const double principal_point,
                     const double focal_length,
                     const double& mm);

}  // namespace utils
