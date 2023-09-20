#include "../include/image_processing.hpp"
#include <cuda_runtime.h>

namespace gpu {

void undistort(cv::Mat& input_frame,
               cv::Mat& output_frame,
               cv::Mat& camera_matrix,
               cv::Mat& dist_coeffs) {
  cv::undistort(input_frame, output_frame, camera_matrix, dist_coeffs);
}

void binarise(uint8_t* red_frame, uint8_t threshold) {
  uint8_t* device_red_frame;
  size_t size = WIDTH * HEIGHT * sizeof(uint8_t);

  // Allocate memory on GPU
  cudaMalloc((void**)&device_red_frame, size);

  // Transfer data from host to device
  cudaMemcpy(device_red_frame, red_frame, size, cudaMemcpyHostToDevice);

  // Define block and grid sizes
  dim3 blockSize(16, 16);  // You can adjust these values based on your needs
  dim3 gridSize((WIDTH + blockSize.x - 1) / blockSize.x,
                (HEIGHT + blockSize.y - 1) / blockSize.y);

  // Launch the kernel
  binarizeKernel<<<gridSize, blockSize>>>(device_red_frame, WIDTH, HEIGHT,
                                          threshold);

  // Transfer data back from device to host
  cudaMemcpy(red_frame, device_red_frame, size, cudaMemcpyDeviceToHost);

  // Free the allocated memory on GPU
  cudaFree(device_red_frame);
}

__global__ void binarizeKernel(uint8_t* device_red_frame,
                               int width,
                               int height,
                               uint8_t threshold) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < width && y < height) {
    device_red_frame[y * width + x] =
        device_red_frame[y * width + x] > threshold ? 255 : 0;
  }
}

}  // namespace gpu
