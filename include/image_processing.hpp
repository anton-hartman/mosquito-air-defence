#pragma once

#include <cuda_runtime.h>
#include "../include/frame.hpp"

namespace gpu {

__constant__ int d_COLS;
__constant__ int d_ROWS;

extern const dim3 block_size;
extern const dim3 grid_size;
extern const size_t frame_size;

uint8_t* d_frame_1;
uint8_t* d_frame_2;
uint8_t* mos_d_frame_1;
uint8_t* mos_d_frame_2;

uint8_t* d_background;

uint8_t* d_mos_opening_struct_elem;
uint8_t* d_mos_closing_struct_elem;
uint8_t* d_laser_opening_struct_elem;
uint8_t* d_laser_closing_struct_elem;

void init_gpu();
void free_gpu();

__global__ void gaussian_smoothing(uint8_t* input,
                                   uint8_t* output,
                                   int kernel_size,
                                   float sigma);

__global__ void binarise_gt(uint8_t* frame, uint8_t threshold);
__global__ void binarise_lt(uint8_t* frame, uint8_t threshold);
__global__ void subtract_and_update_background(uint8_t* frame,
                                               uint8_t* bg_frame,
                                               int learning_rate);
/**
 * @brief Erosion is the minimum value of the pixels covered by the structuring
 * element.
 */
__global__ void erosion(uint8_t* input,
                        uint8_t* output,
                        uint8_t* struct_elem,
                        int struct_elem_size);
/**
 * @brief Dilation is the maximum value of the pixels covered by the
 * structuring element.
 */
__global__ void dilation(uint8_t* input,
                         uint8_t* output,
                         uint8_t* struct_elem,
                         int struct_elem_size);
/**
 * @brief Opening is erosion followed by dilation with the same structuring
 * element.
 */
void opening(uint8_t* input_and_output, uint8_t* temp);

/**
 * @brief Closing is dilation followed by erosion with the same structuring
 * element.
 */
void closing(uint8_t* input_and_output, uint8_t* temp);

}  // namespace gpu