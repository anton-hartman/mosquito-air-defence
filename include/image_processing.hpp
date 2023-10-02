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

void init_gpu();
void free_gpu();

__global__ void binarise(uint8_t* device_frame, uint8_t threshold);
__global__ void erosion(uint8_t* input, uint8_t* output);
__global__ void dilation(uint8_t* input, uint8_t* output);

void opening();
void closing();
void open_and_close();
void close_and_open();

std::pair<int32_t, int32_t> detect_laser(uint8_t* red_frame, uint8_t threshold);
std::pair<int32_t, int32_t> detect_laser(cv::Mat red_frame, uint8_t threshold);

}  // namespace gpu