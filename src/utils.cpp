#include "../include/utils.hpp"
#include <cmath>
#include "../include/turret_controller.hpp"

namespace utils {

// void set_camera_intrinsics(cv::Mat& camera_matrix) {
//   f_x = camera_matrix.at<double>(0, 0);
//   f_y = camera_matrix.at<double>(1, 1);
//   c_x = camera_matrix.at<double>(0, 2);
//   c_y = camera_matrix.at<double>(1, 2);
// }

float angle_to_mm(const float& theta, const float& depth) {
  return depth * std::tan(theta);
}

float mm_to_angle(const float& mm, const float& depth) {
  return std::atan2(mm, depth);
}

float pixel_to_mm(const turret::Stepper& stepper,
                  const uint16_t& px,
                  const uint16_t& depth) {
  return (px - stepper.principal_point) * depth / stepper.focal_length;
}

uint16_t mm_to_pixel(const turret::Stepper& stepper,
                     const uint16_t& mm,
                     const uint16_t& depth) {
  return (mm * stepper.focal_length / depth) + stepper.principal_point;
}

}  // namespace utils
