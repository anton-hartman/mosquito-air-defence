#include "../include/image_processing.hpp"

namespace gpu {

dim3 const block_size(16, 8);
dim3 const grid_size((WIDTH + block_size.x - 1) / block_size.x,
                     (HEIGHT + block_size.y - 1) / block_size.y);

const size_t frame_size = WIDTH * HEIGHT * sizeof(uint8_t);
uint8_t* device_frame;
uint8_t* device_temp_frame;
uint8_t* d_struct_elem;
int struct_elem_size = 1;

uint8_t* create_struct_elem(int size) {
  int diameter = 2 * size + 1;
  uint8_t* struct_elem = new uint8_t[diameter * diameter];

  for (int i = 0; i < diameter; i++) {
    for (int j = 0; j < diameter; j++) {
      int y = i - size;  // y-coordinate relative to the center
      int x = j - size;  // x-coordinate relative to the center
      struct_elem[i * diameter + j] = (x * x + y * y <= size * size) ? 1 : 0;
    }
  }
  return struct_elem;
}

size_t get_sizeof_elem(int size) {
  int diameter = 2 * size + 1;
  return (diameter * diameter) * sizeof(uint8_t);
}

void init_gpu() {
  cudaMemcpyToSymbol(d_WIDTH, &WIDTH, sizeof(int));
  cudaMemcpyToSymbol(d_HEIGHT, &HEIGHT, sizeof(int));

  cudaMalloc((void**)&device_frame, frame_size);
  cudaMalloc((void**)&device_temp_frame, frame_size);

  std::cout << "Struct elem: " << create_struct_elem(struct_elem_size)
            << std::endl;
  cudaMalloc((void**)&d_struct_elem, get_sizeof_elem(struct_elem_size));
  cudaMemcpy(d_struct_elem, create_struct_elem(struct_elem_size),
             get_sizeof_elem(struct_elem_size), cudaMemcpyHostToDevice);
}

void free_gpu() {
  cudaFree(device_frame);
  cudaFree(device_temp_frame);
}

void undistort(cv::Mat& input_frame,
               cv::Mat& output_frame,
               cv::Mat& camera_matrix,
               cv::Mat& dist_coeffs) {
  cv::undistort(input_frame, output_frame, camera_matrix, dist_coeffs);
}

__global__ void binarise(uint8_t* device_frame, uint8_t threshold) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_WIDTH && y < d_HEIGHT) {
    device_frame[y * d_WIDTH + x] =
        device_frame[y * d_WIDTH + x] >= threshold ? 255 : 0;
  }
}

/**
 * @brief
 *
 * @param input
 * @param output
 * @param struct_elem Must be square
 * @param struct_elem_size
 * @return __global__
 */
__global__ void erosion(uint8_t* input,
                        uint8_t* output,
                        uint8_t* struct_elem,
                        uint8_t struct_elem_size) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_WIDTH && y < d_HEIGHT) {
    int min_val = 255;
    for (int i = -struct_elem_size; i <= struct_elem_size; i++) {
      for (int j = -struct_elem_size; j <= struct_elem_size; j++) {
        if (y + i >= 0 && y + i < d_HEIGHT && x + j >= 0 && x + j < d_WIDTH) {
          if (struct_elem[(i + struct_elem_size) * (2 * struct_elem_size + 1) +
                          j + struct_elem_size] == 1) {
            int idx = (y + i) * d_WIDTH + (x + j);
            min_val = min(min_val, (int)input[idx]);
          }
        }
      }
    }
    output[y * d_WIDTH + x] = min_val;
  }
}

__global__ void dilation(uint8_t* input,
                         uint8_t* output,
                         int* struct_elem,
                         int struct_elem_size) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_WIDTH && y < d_HEIGHT) {
    int maxVal = 0;
    for (int i = -struct_elem_size; i <= struct_elem_size; i++) {
      for (int j = -struct_elem_size; j <= struct_elem_size; j++) {
        if (y + i >= 0 && y + i < d_HEIGHT && x + j >= 0 && x + j < d_WIDTH) {
          if (struct_elem[(i + struct_elem_size) * (2 * struct_elem_size + 1) +
                          j + struct_elem_size] == 1) {
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

  erosion<<<grid_size, block_size>>>(device_frame, device_temp_frame,
                                     d_struct_elem, struct_elem_size);
  cudaDeviceSynchronize();

  err = cudaMemcpy(red_frame, device_temp_frame, frame_size,
                   cudaMemcpyDeviceToHost);
  if (err != cudaSuccess) {
    printf("CUDA error erosion: %s\n", cudaGetErrorString(err));
  }
  cv::Mat erode_mat(HEIGHT, WIDTH, CV_8UC1, red_frame);
  cv::imshow("erosion", erode_mat);
  cv::waitKey(1);

  return 0;
}

// void opening() {
//   // Define structuring element and its size
//   int struct_elem_size = 1;  // For a 3x3 structuring element
//   // Example for a 3x3 square structuring element
//   int struct_elem[] = {1, 1, 1, 1, 1, 1, 1, 1, 1};

//   // Perform Opening
//   erosion<<<grid_size, block_size>>>(device_frame, device_temp_frame,
//                                      struct_elem, struct_elem_size);
//   // dilation<<<grid_size, block_size>>>(device_temp_frame, device_frame,
//   //                                     struct_elem, struct_elem_size);
// }

}  // namespace gpu
