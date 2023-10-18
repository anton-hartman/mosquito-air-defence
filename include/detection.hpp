#pragma once

#include <atomic>
#include <opencv2/opencv.hpp>
#include "pt.hpp"

namespace detection {

enum class StructElemType {
  MOS_OPENING,
  MOS_CLOSING,
  LASER_OPENING,
  LASER_CLOSING
};

extern Pt ignore_region_top_left;
extern Pt ignore_region_bottom_right;
std::atomic<float> bg_learning_rate = 0.0;

void set_ignore_region(Pt top_left, Pt bottom_right);

std::vector<Pt> detect_lasers(cv::Mat red_frame, uint8_t threshold);
Pt distinguish_lasers(const std::vector<Pt>& pts);
std::vector<Pt> detect_mosquitoes(cv::Mat red_frame,
                                  uint8_t threshold,
                                  bool bg_sub = true);
void set_background(const cv::Mat& frame);

}  // namespace detection