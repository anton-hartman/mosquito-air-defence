#pragma once

#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>
#include "frame.hpp"

namespace gpu {

extern std::pair<uint16_t, uint16_t> ignore_region_top_left;
extern std::pair<uint16_t, uint16_t> ignore_region_bottom_right;

__constant__ int d_COLS;
__constant__ int d_ROWS;

extern const dim3 block_size;
extern const dim3 grid_size;
extern const size_t frame_size;

void init_gpu();
void free_gpu();

void set_ignore_region(std::pair<uint16_t, uint16_t> top_left,
                       std::pair<uint16_t, uint16_t> bottom_right);

std::pair<int32_t, int32_t> detect_laser(cv::Mat red_frame, uint8_t threshold);
std::vector<Pt> detect_mosquitoes(cv::Mat red_frame,
                                  uint8_t threshold,
                                  bool bg_sub = true);
void set_background(const cv::Mat& frame);
void set_learning_rate(const float& learning_rate);

}  // namespace gpu