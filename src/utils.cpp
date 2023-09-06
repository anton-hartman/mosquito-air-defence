#include "../include/utils.hpp"
#include <cmath>

namespace utils {

void set_camera_intrinsics(cv::Mat& camera_matrix) {
  f_x = camera_matrix.at<double>(0, 0);
  f_y = camera_matrix.at<double>(1, 1);
  c_x = camera_matrix.at<double>(0, 2);
  c_y = camera_matrix.at<double>(1, 2);
}

/**
 * @return depth * tan(theta)
 */
float angle_to_mm(float theta, float depth) {
  return depth * std::tan(theta);
}

/**
 * @return arctan(mm / depth)
 */
float mm_to_angle(float mm, float depth) {
  return std::atan2(mm, depth);
}

/**
 * Convert real-world coordinates in millimeters to pixel coordinates. All
 * parameters must be in millimeters
 *
 * @param X Real-world coordinate in the horizontal direction (origin at the
 * optical center of the camera).
 * @param Y Real-world coordinate in the vertical direction (origin at the
 * optical center of the camera).
 * @param Z Known distance from the camera to the plane in millimeters.
 *
 * @return A std::pair<uint16_t, uint16_t> containing the u and v coordinates in
 * pixels.
 *
 * @note - The input real-world coordinates (X, Y) are based on an origin at the
 * optical center of the camera.
 * @note - The returned pixel coordinates (u, v) have their origin at the
 * top-left corner of the image.
 */
std::pair<uint16_t, uint16_t> mm_to_pixel(const float& X,
                                          const float& Y,
                                          const uint16_t& Z) {
  double u = (X * f_x / Z) + c_x;
  double v = (Y * f_y / Z) + c_y;
  return {u, v};
}

/**
 * Convert pixel coordinates to real-world coordinates in millimeters.
 *
 * @param u Pixel coordinate in the horizontal direction (origin at top-left of
 * the image).
 * @param v Pixel coordinate in the vertical direction (origin at top-left of
 * the image).
 * @param Z Known distance from the camera to the plane in millimeters.
 *
 * @return A std::pair<float, float> containing the X and Y coordinates in
 * millimeters.
 *
 * @note - The input pixel coordinates (u, v) are based on an origin at the
 * top-left corner of the image.
 * @note - The returned real-world coordinates (X, Y) have their origin at the
 * optical center of the camera. This optical center is defined by the intrinsic
 * parameters (c_x, c_y) from the camera matrix. Positive X values extend to the
 * right of the optical center, negative X values extend to the left. Positive Y
 * values extend downward from the optical center, and negative Y values extend
 * upward.
 */
std::pair<float, float> pixel_to_mm(const uint16_t& u,
                                    const uint16_t& v,
                                    const uint16_t& Z) {
  double X = (u - c_x) * Z / f_x;
  double Y = (v - c_y) * Z / f_y;
  return {X, Y};
}

}  // namespace utils
