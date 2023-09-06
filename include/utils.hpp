#pragma once

#include <cstdint>
#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>

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

  Circle(std::pair<uint16_t, uint16_t> center)
      : x(center.first), y(center.second) {}
};

static double f_x;
static double f_y;
static double c_x;
static double c_y;

static const uint16_t TANK_DEPTH = 318;  // mm
const uint16_t TURRET_DEPTH = 575;       // mm
const uint16_t CAMERA_DEPTH = 780;       // mm

void set_camera_intrinsics(cv::Mat& camera_matrix);

float angle_to_mm(float theta, float depth);
float mm_to_angle(float mm, float depth);

std::pair<float, float> pixel_to_mm(const uint16_t& u,
                                    const uint16_t& v,
                                    const uint16_t& Z);
std::pair<uint16_t, uint16_t> mm_to_pixel(const float& X,
                                          const float& Y,
                                          const uint16_t& Z);

// std::pair<float, float> pixel_to_angle(std::pair<uint16_t, uint16_t>
// pixel_coordinates);

}  // namespace utils
