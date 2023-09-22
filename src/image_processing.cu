#include <iostream>
#include "../include/image_processing.hpp"

namespace gpu {

dim3 const block_size(16, 8);
dim3 const grid_size((WIDTH + block_size.x - 1) / block_size.x,
                     (HEIGHT + block_size.y - 1) / block_size.y);

const size_t frame_size = WIDTH * HEIGHT * sizeof(uint8_t);
uint8_t* device_frame;
uint8_t* device_temp_frame;

const int struct_elem_size = 2;
const int diameter = 2 * struct_elem_size + 1;
__constant__ uint8_t d_structuring_element[diameter * diameter];
__constant__ int d_struct_elem_size;

void create_structuring_element(uint8_t* struct_elem, int struct_elem_size) {
  int diameter = 2 * struct_elem_size + 1;

  for (int i = 0; i < diameter; i++) {
    for (int j = 0; j < diameter; j++) {
      int y = i - struct_elem_size;  // y-coordinate relative to the center
      int x = j - struct_elem_size;  // x-coordinate relative to the center
      struct_elem[i * diameter + j] =
          (x * x + y * y <= struct_elem_size * struct_elem_size) ? 1 : 0;
    }
  }
}

void initialize_struct_elem() {
  uint8_t host_struct_elem[diameter * diameter];
  create_structuring_element(host_struct_elem, struct_elem_size);

  // Copy the host array to the GPU constant memory
  cudaMemcpyToSymbol(d_structuring_element, host_struct_elem,
                     diameter * diameter * sizeof(uint8_t));
  cudaMemcpyToSymbol(d_struct_elem_size, &struct_elem_size, sizeof(int));
}

void init_gpu() {
  cudaMemcpyToSymbol(d_WIDTH, &WIDTH, sizeof(int));
  cudaMemcpyToSymbol(d_HEIGHT, &HEIGHT, sizeof(int));

  cudaMalloc((void**)&device_frame, frame_size);
  cudaMalloc((void**)&device_temp_frame, frame_size);

  // init_structuring_element();
  initialize_struct_elem();
}

void free_gpu() {
  cudaFree(device_frame);
  cudaFree(device_temp_frame);
}

__global__ void binarise(uint8_t* device_frame, uint8_t threshold) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_WIDTH && y < d_HEIGHT) {
    device_frame[y * d_WIDTH + x] =
        device_frame[y * d_WIDTH + x] >= threshold ? 255 : 0;
  }
}

__global__ void erosion(uint8_t* input, uint8_t* output) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_WIDTH && y < d_HEIGHT) {
    int min_val = 255;
    for (int i = -d_struct_elem_size; i <= d_struct_elem_size; i++) {
      for (int j = -d_struct_elem_size; j <= d_struct_elem_size; j++) {
        if (y + i >= 0 && y + i < d_HEIGHT && x + j >= 0 && x + j < d_WIDTH) {
          if (d_structuring_element[(i + d_struct_elem_size) *
                                        (2 * d_struct_elem_size + 1) +
                                    j + d_struct_elem_size] == 1) {
            int idx = (y + i) * d_WIDTH + (x + j);
            min_val = min(min_val, (int)input[idx]);
          }
        }
      }
    }
    output[y * d_WIDTH + x] = min_val;
  }
}

__global__ void dilation(uint8_t* input, uint8_t* output) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_WIDTH && y < d_HEIGHT) {
    int maxVal = 0;
    for (int i = -d_struct_elem_size; i <= d_struct_elem_size; i++) {
      for (int j = -d_struct_elem_size; j <= d_struct_elem_size; j++) {
        if (y + i >= 0 && y + i < d_HEIGHT && x + j >= 0 && x + j < d_WIDTH) {
          if (d_structuring_element[(i + d_struct_elem_size) *
                                        (2 * d_struct_elem_size + 1) +
                                    j + d_struct_elem_size] == 1) {
            int idx = (y + i) * d_WIDTH + (x + j);
            maxVal = max(maxVal, (int)input[idx]);
          }
        }
      }
    }
    output[y * d_WIDTH + x] = maxVal;
  }
}

uint32_t detect_laser(uint8_t* red_frame, uint8_t threshold) {
  cudaError_t err;

  cudaMemcpy(device_frame, red_frame, frame_size, cudaMemcpyHostToDevice);

  binarise<<<grid_size, block_size>>>(device_frame, threshold);
  cudaDeviceSynchronize();
  err = cudaMemcpy(red_frame, device_frame, frame_size, cudaMemcpyDeviceToHost);
  if (err != cudaSuccess) {
    printf("CUDA error binarise: %s\n", cudaGetErrorString(err));
  }
  cv::Mat bin_mat(HEIGHT, WIDTH, CV_8UC1, red_frame);
  cv::imshow("binarise", bin_mat);
  cv::waitKey(1);

  // open_and_close();
  close_and_open();
  cudaDeviceSynchronize();

  err = cudaMemcpy(red_frame, device_frame, frame_size, cudaMemcpyDeviceToHost);
  if (err != cudaSuccess) {
    printf("CUDA error first: %s\n", cudaGetErrorString(err));
  }
  cv::Mat oepning_mat(HEIGHT, WIDTH, CV_8UC1, red_frame);
  cv::imshow("first", oepning_mat);
  cv::waitKey(1);

  // open_and_close();
  close_and_open();
  close_and_open();
  close_and_open();
  cudaDeviceSynchronize();

  err = cudaMemcpy(red_frame, device_frame, frame_size, cudaMemcpyDeviceToHost);
  if (err != cudaSuccess) {
    printf("CUDA error second: %s\n", cudaGetErrorString(err));
  }
  cv::Mat closing_mat(HEIGHT, WIDTH, CV_8UC1, red_frame);
  cv::imshow("second", closing_mat);
  cv::waitKey(1);

  return 0;
}

void opening() {
  erosion<<<grid_size, block_size>>>(device_frame, device_temp_frame);
  dilation<<<grid_size, block_size>>>(device_temp_frame, device_frame);
}

void closing() {
  dilation<<<grid_size, block_size>>>(device_frame, device_temp_frame);
  erosion<<<grid_size, block_size>>>(device_temp_frame, device_frame);
}

void open_and_close() {
  opening();
  closing();
}

void close_and_open() {
  closing();
  opening();
}

}  // namespace gpu
