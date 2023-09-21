#pragma once

#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>
#include "frame.hpp"

namespace gpu {

__constant__ int d_WIDTH;
__constant__ int d_HEIGHT;

extern const dim3 block_size;
extern const dim3 grid_size;

extern const size_t frame_size;
extern uint8_t* device_frame;
extern uint8_t* device_temp_frame;
extern uint8_t* d_struct_elem;
extern int struct_elem_size;

void init_gpu();

void undistort(cv::Mat& input_frame,
               cv::Mat& output_frame,
               cv::Mat& camera_matrix,
               cv::Mat& dist_coeffs);

__global__ void binarise(uint8_t* device_frame, uint8_t threshold);
__global__ void erosion(uint8_t* input,
                        uint8_t* output,
                        uint8_t* struct_elem,
                        uint8_t struct_elem_size);
__global__ void dilation(uint8_t* input,
                         uint8_t* output,
                         uint8_t* struct_elem,
                         uint8_t struct_elem_size);

uint32_t detect_laser(uint8_t* red_frame, uint8_t threshold);
void opening();

}  // namespace gpu