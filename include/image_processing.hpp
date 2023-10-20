#pragma once

#include <cuda_runtime.h>
#include <atomic>
#include <opencv2/opencv.hpp>
#include "../include/frame.hpp"
#include "pt.hpp"

enum class StructElemType {
  MOS_OPENING,
  MOS_CLOSING,
  LASER_OPENING,
  LASER_CLOSING
};

namespace gpu {

void init_gpu();
void free_gpu();

/**
 * @brief Create a disk-shaped structuring element, with diameter = 2*radius+1.
 */
void set_struct_elem(int struct_elem_radius, StructElemType type);

}  // namespace gpu

namespace detection {

extern Pt ignore_region_top_left;
extern Pt ignore_region_bottom_right;
extern std::atomic<float> bg_learning_rate;

void set_ignore_region(Pt top_left, Pt bottom_right);
void set_background(const cv::Mat& frame);

std::vector<Pt> detect_lasers(cv::Mat red_frame, const uint8_t threshold);
Pt distinguish_lasers(const std::vector<Pt>& pts);
std::vector<Pt> detect_mosquitoes(cv::Mat red_frame,
                                  const uint8_t threshold,
                                  const bool bg_sub = true);
void remove_lasers_from_mos(const std::vector<Pt>& laser_pts,
                            std::vector<Pt>& mos_pts,
                            const int remove_radius);
}  // namespace detection