#include "../include/utils.hpp"

namespace utils {

std::pair<float, float> pixel_to_mm(const Point& pixel_point,
                                    const std::vector<float> camera_matrix,
                                    const float camera_depth) {
  float fx = camera_matrix[0];  // 0, 0
  float fy = camera_matrix[4];  // 1, 1
  float cx = camera_matrix[2];  // 0, 2
  float cy = camera_matrix[5];  // 1, 2

  float X = (pixel_point.x - cx) * camera_depth / fx;
  float Y = (pixel_point.y - cy) * camera_depth / fy;

  return {X, Y};
}

std::pair<uint16_t, uint16_t> angle_to_pixel(
    const std::pair<float, float>& angle) {
  return {69, 69};
}

}  // namespace utils
