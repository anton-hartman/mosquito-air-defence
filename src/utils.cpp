#include "../include/utils.hpp"

namespace utils {

void draw_target(cv::Mat& frame, const Pt& target, const cv::Scalar& colour) {
  // Length of the perpendicular lines for target and setpoint
  int line_length = 50;
  // Draw a horizontal line passing through the target point
  cv::line(frame, cv::Point(target.x - line_length, target.y),
           cv::Point(target.x + line_length, target.y), colour, 2);
  // Draw a vertical line passing through the target point
  cv::line(frame, cv::Point(target.x, target.y - line_length),
           cv::Point(target.x, target.y + line_length), colour, 2);
}

void put_label(cv::Mat& img,
               const std::string& label,
               const Pt& origin,
               const double& font_scale) {
  int font_face = cv::FONT_HERSHEY_DUPLEX;
  int thickness = 2;
  cv::putText(img, label, cv::Point(origin.x, origin.y), font_face, font_scale,
              cv::Scalar(0, 255, 255), thickness);
}

}  // namespace utils
