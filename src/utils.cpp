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

void draw_target(cv::Mat& frame,
                 const std::pair<uint16_t, uint16_t>& target,
                 const cv::Scalar& colour) {
  // Length of the perpendicular lines for target and setpoint
  int line_length = 50;
  // Draw a horizontal line passing through the target point
  cv::line(frame, cv::Point(target.first - line_length, target.second),
           cv::Point(target.first + line_length, target.second), colour, 2);
  // Draw a vertical line passing through the target point
  cv::line(frame, cv::Point(target.first, target.second - line_length),
           cv::Point(target.first, target.second + line_length), colour, 2);
}

void put_label(cv::Mat& img,
               const std::string& label,
               const std::pair<uint16_t, uint16_t>& origin,
               const double& font_scale) {
  int font_face = cv::FONT_HERSHEY_DUPLEX;
  int thickness = 2;
  cv::putText(img, label, cv::Point(origin.first, origin.second), font_face,
              font_scale, cv::Scalar(0, 255, 255), thickness);
}

float turret_angle_to_mm(const float& theta_deg) {
  return TURRET_DEPTH * std::tan(theta_deg * M_PI / 180);
}

float mm_to_turret_angle(const float& mm) {
  return std::atan2(mm, TURRET_DEPTH) * 180 / M_PI;
}

float pixel_to_mm(const turret::Stepper& stepper, const uint16_t& px) {
  return (px - stepper.principal_point) * CAMERA_DEPTH / stepper.focal_length;
}

uint16_t mm_to_pixel(const turret::Stepper& stepper, const uint16_t& mm) {
  return (mm * stepper.focal_length / CAMERA_DEPTH) + stepper.principal_point;
}

}  // namespace utils
