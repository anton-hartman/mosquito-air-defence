#include <chrono>
#include <iostream>
#include <thread>

#include "../include/image_processing.hpp"

namespace gpu {

dim3 const block_size(16, 8);
dim3 const grid_size((COLS + block_size.x - 1) / block_size.x,
                     (ROWS + block_size.y - 1) / block_size.y);

const size_t frame_size = COLS * ROWS * sizeof(uint8_t);

void init_gpu() {
  cudaMemcpyToSymbol(d_COLS, &COLS, sizeof(int));
  cudaMemcpyToSymbol(d_ROWS, &ROWS, sizeof(int));
  cudaMemcpyToSymbol(d_bg_learning_rate, 0, sizeof(float));

  cudaMalloc((void**)&d_frame_1, frame_size);
  cudaMalloc((void**)&d_frame_2, frame_size);
  cudaMalloc((void**)&mos_d_frame_1, frame_size);
  cudaMalloc((void**)&mos_d_frame_2, frame_size);

  int mos_opening_size = 1;
  int mos_closing_size = 3;
  int laser_opening_size = 1;
  int laser_closing_size = 3;

  auto diameter = [](int x) -> int { return x * 2 + 1; };

  cudaMalloc((void**)&d_mos_opening_struct_elem,
             std::pow(diameter(mos_opening_size), 2) * sizeof(uint8_t));
  cudaMalloc((void**)&d_mos_closing_struct_elem,
             std::pow(diameter(mos_closing_size), 2) * sizeof(uint8_t));
  cudaMalloc((void**)&d_laser_opening_struct_elem,
             std::pow(diameter(laser_opening_size), 2) * sizeof(uint8_t));
  cudaMalloc((void**)&d_laser_closing_struct_elem,
             std::pow(diameter(laser_closing_size), 2) * sizeof(uint8_t));

  set_struct_elem(mos_opening_size, StructElemType::MOS_OPENING);
  set_struct_elem(mos_closing_size, StructElemType::MOS_CLOSING);
  set_struct_elem(laser_opening_size, StructElemType::LASER_OPENING);
  set_struct_elem(laser_closing_size, StructElemType::LASER_CLOSING);

  cudaMalloc((void**)&d_background, frame_size);
}

void free_gpu() {
  cudaFree(d_frame_1);
  cudaFree(d_frame_2);
  cudaFree(mos_d_frame_1);
  cudaFree(mos_d_frame_2);
  cudaFree(d_background);
  cudaFree(d_mos_opening_struct_elem);
  cudaFree(d_mos_closing_struct_elem);
  cudaFree(d_laser_opening_struct_elem);
  cudaFree(d_laser_closing_struct_elem);
}

__global__ void gaussian_smoothing(uint8_t* input,
                                   uint8_t* output,
                                   int kernel_size,
                                   float sigma) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_COLS && y < d_ROWS) {
    float sum = 0.0f;
    float total_weight = 0.0f;
    int half_kernel_size = kernel_size / 2;

    for (int i = -half_kernel_size; i <= half_kernel_size; ++i) {
      for (int j = -half_kernel_size; j <= half_kernel_size; ++j) {
        int current_x = x + j;
        int current_y = y + i;

        if (current_x >= 0 && current_x < d_COLS && current_y >= 0 &&
            current_y < d_ROWS) {
          float weight = exp(-(i * i + j * j) / (2.0f * sigma * sigma));
          sum += input[current_y * d_COLS + current_x] * weight;
          total_weight += weight;
        }
      }
    }

    output[y * d_COLS + x] = static_cast<uint8_t>(sum / total_weight);
  }
}

__global__ void binarise_gt(uint8_t* frame, uint8_t threshold) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_COLS && y < d_ROWS) {
    frame[y * d_COLS + x] = frame[y * d_COLS + x] > threshold ? 255 : 0;
  }
}

__global__ void binarise_lt(uint8_t* frame, uint8_t threshold) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_COLS && y < d_ROWS) {
    frame[y * d_COLS + x] = frame[y * d_COLS + x] < threshold ? 255 : 0;
  }
}

__global__ void subtract_and_update_background(uint8_t* frame,
                                               uint8_t* bg_frame,
                                               float learning_rate) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_COLS && y < d_ROWS) {
    bg_frame[y * d_COLS + x] = learning_rate * frame[y * d_COLS + x] +
                               (1 - learning_rate) * bg_frame[y * d_COLS + x];
    frame[y * d_COLS + x] =
        abs(bg_frame[y * d_COLS + x] - frame[y * d_COLS + x]);
  }
}

__global__ void erosion(uint8_t* input,
                        uint8_t* output,
                        uint8_t* struct_elem,
                        int struct_elem_size) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_COLS && y < d_ROWS) {
    int min_val = 255;
    for (int i = -struct_elem_size; i <= struct_elem_size; i++) {
      for (int j = -struct_elem_size; j <= struct_elem_size; j++) {
        if (y + i >= 0 && y + i < d_ROWS && x + j >= 0 && x + j < d_COLS) {
          if (struct_elem[(i + struct_elem_size) * (2 * struct_elem_size + 1) +
                          j + struct_elem_size] == 1) {
            int idx = (y + i) * d_COLS + (x + j);
            min_val = min(min_val, (int)input[idx]);
          }
        }
      }
    }
    output[y * d_COLS + x] = min_val;
  }
}

__global__ void dilation(uint8_t* input,
                         uint8_t* output,
                         uint8_t* struct_elem,
                         int struct_elem_size) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_COLS && y < d_ROWS) {
    int max_val = 0;
    for (int i = -struct_elem_size; i <= struct_elem_size; i++) {
      for (int j = -struct_elem_size; j <= struct_elem_size; j++) {
        if (y + i >= 0 && y + i < d_ROWS && x + j >= 0 && x + j < d_COLS) {
          if (struct_elem[(i + struct_elem_size) * (2 * struct_elem_size + 1) +
                          j + struct_elem_size] == 1) {
            int idx = (y + i) * d_COLS + (x + j);
            max_val = max(max_val, (int)input[idx]);
          }
        }
      }
    }
    output[y * d_COLS + x] = max_val;
  }
}

void opening(uint8_t* input_and_output, uint8_t* temp) {
  erosion<<<grid_size, block_size>>>(mos_d_frame_2, mos_d_frame_1);
  dilation<<<grid_size, block_size>>>(mos_d_frame_1, mos_d_frame_2);
}

void closing(uint8_t* input_and_output, uint8_t* temp) {
  dilation<<<grid_size, block_size>>>(input_and_output, temp);
  erosion<<<grid_size, block_size>>>(temp, input_and_output);
}

}  // namespace gpu
