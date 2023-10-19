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

void set_struct_elem(int struct_elem_size, StructElemType type);

// __global__ void gaussian_smoothing(uint8_t* input,
//                                    uint8_t* output,
//                                    int kernel_size,
//                                    float sigma);

// __global__ void binarise_gt(uint8_t* frame, uint8_t threshold);
// __global__ void binarise_lt(uint8_t* frame, uint8_t threshold);
// __global__ void subtract_and_update_background(uint8_t* frame,
//                                                uint8_t* bg_frame,
//                                                float learning_rate);
// /**
//  * @brief Erosion is the minimum value of the pixels covered by the
//  structuring
//  * element.
//  */
// __global__ void erosion(uint8_t* input,
//                         uint8_t* output,
//                         uint8_t* struct_elem,
//                         int struct_elem_size);
// /**
//  * @brief Dilation is the maximum value of the pixels covered by the
//  * structuring element.
//  */
// __global__ void dilation(uint8_t* input,
//                          uint8_t* output,
//                          uint8_t* struct_elem,
//                          int struct_elem_size);
// /**
//  * @brief Opening is erosion followed by dilation with the same structuring
//  * element.
//  */
// void opening(uint8_t* input_and_output,
//              uint8_t* temp,
//              uint8_t* struct_elem,
//              int struct_elem_size);

// /**
//  * @brief Closing is dilation followed by erosion with the same structuring
//  * element.
//  */
// void closing(uint8_t* input_and_output,
//              uint8_t* temp,
//              uint8_t* struct_elem,
//              int struct_elem_size);

}  // namespace gpu

namespace detection {

extern Pt ignore_region_top_left;
extern Pt ignore_region_bottom_right;
extern std::atomic<float> bg_learning_rate;

void set_ignore_region(Pt top_left, Pt bottom_right);
void set_background(const cv::Mat& frame);

std::vector<Pt> detect_lasers(cv::Mat red_frame, uint8_t threshold);
Pt distinguish_lasers(const std::vector<Pt>& pts);
std::vector<Pt> detect_mosquitoes(cv::Mat red_frame,
                                  uint8_t threshold,
                                  bool bg_sub = true);

}  // namespace detection