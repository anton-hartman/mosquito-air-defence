#pragma once

#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>
#include "frame.hpp"

namespace gpu {

void undistort(cv::Mat& input_frame,
               cv::Mat& output_frame,
               cv::Mat& camera_matrix,
               cv::Mat& dist_coeffs);

__global__ void binarizeKernel(uint8_t* device_red_frame,
                               int width,
                               int height,
                               uint8_t threshold);

uint32_t binarise(uint8_t* red_frame, uint8_t threshold);

}  // namespace gpu