#pragma once

#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>
#include "pt.hpp"

namespace gpu {

extern Pt ignore_region_top_left;
extern Pt ignore_region_bottom_right;

__constant__ int d_COLS;
__constant__ int d_ROWS;

extern const dim3 block_size;
extern const dim3 grid_size;
extern const size_t frame_size;

void init_gpu();
void free_gpu();

void set_ignore_region(Pt top_left, Pt bottom_right);

std::vector<Pt> detect_laser(cv::Mat red_frame, uint8_t threshold);
Pt distinguish_laser_only_2(const std::vector<Pt>& pts);
std::vector<Pt> detect_mosquitoes(cv::Mat red_frame,
                                  uint8_t threshold,
                                  bool bg_sub = true);
void set_background(const cv::Mat& frame);
void set_bg_learning_rate(const float& bg_learning_rate);

}  // namespace gpu