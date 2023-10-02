#include <fstream>
#include <iostream>
#include "../include/image_processing.hpp"
// #include "../include/labeling_algorithms.h"
// #include "../include/labels_solver.h"
#include "../include/two_pass_algorithm.hpp"

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

std::pair<int32_t, int32_t> laser_position;

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

__global__ void gaussian_smoothing(uint8_t* input,
                                   uint8_t* output,
                                   int kernel_size,
                                   float sigma) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_WIDTH && y < d_HEIGHT) {
    float sum = 0.0f;
    float total_weight = 0.0f;
    int half_kernel_size = kernel_size / 2;

    for (int i = -half_kernel_size; i <= half_kernel_size; ++i) {
      for (int j = -half_kernel_size; j <= half_kernel_size; ++j) {
        int current_x = x + j;
        int current_y = y + i;

        if (current_x >= 0 && current_x < d_WIDTH && current_y >= 0 &&
            current_y < d_HEIGHT) {
          float weight = exp(-(i * i + j * j) / (2.0f * sigma * sigma));
          sum += input[current_y * d_WIDTH + current_x] * weight;
          total_weight += weight;
        }
      }
    }

    output[y * d_WIDTH + x] = static_cast<uint8_t>(sum / total_weight);
  }
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

bool is_blob_in_ignore_region(
    const std::pair<uint16_t, uint16_t>& blob,
    const std::pair<uint16_t, uint16_t>& ignore_region_top_left,
    const std::pair<uint16_t, uint16_t>& ignore_region_bottom_right) {
  uint16_t x = blob.first;
  uint16_t y = blob.second;
  return x >= ignore_region_top_left.first &&
         x <= ignore_region_bottom_right.first &&
         y >= ignore_region_top_left.second &&
         y <= ignore_region_bottom_right.second;
}

std::pair<int32_t, int32_t> distinguish_laser(
    std::vector<std::pair<uint16_t, uint16_t>> blobs,
    std::pair<uint16_t, uint16_t> camera_origin,
    std::pair<uint16_t, uint16_t> ignore_region_top_left,
    std::pair<uint16_t, uint16_t> ignore_region_bottom_right) {
  // std::cout << "All Blobs:" << std::endl;
  // for (size_t i = 0; i < blobs.size(); ++i) {
  //   std::cout << "Blob " << i + 1 << ": (" << blobs[i].first << ", "
  //             << blobs[i].second << ")" << std::endl;
  // }

  if (blobs.size() == 1)
    return blobs.at(0);

  std::pair<int32_t, int32_t> result = std::make_pair(-1, -1);
  double minDist = std::numeric_limits<double>::infinity();
  double maxDist = -1;

  uint16_t ox = camera_origin.first;
  uint16_t oy = camera_origin.second;

  for (size_t i = 0; i < blobs.size(); i++) {
    std::pair<uint16_t, uint16_t> blob = blobs[i];
    uint16_t x = blob.first;
    uint16_t y = blob.second;

    if (is_blob_in_ignore_region(blob, ignore_region_top_left,
                                 ignore_region_bottom_right)) {
      std::cout << "blob in ignore region" << std::endl;
      continue;  // Skip blobs in the ignore region
    }

    double dist = std::hypot(x - ox, y - oy);

    if (y <= oy && dist < minDist) {
      minDist = dist;
      result = std::make_pair(x, y);
    }

    if (y >= oy && dist > maxDist) {
      maxDist = dist;
      result = std::make_pair(x, y);
    }

    if (y > oy && result.first == -1)
      result = std::make_pair(x, y);
  }

  // std::cout << "result = (" << result.first << ", " << result.second << ")"
  //           << std::endl;
  return result;
}

std::pair<int32_t, int32_t> distinguish_laser(
    std::vector<std::pair<uint16_t, uint16_t>> blobs,
    std::pair<uint16_t, uint16_t> camera_origin) {
  if (blobs.size() < 1)
    return blobs.at(0);

  std::pair<uint16_t, uint16_t> blob1 = blobs[0];
  std::pair<uint16_t, uint16_t> blob2 = blobs[1];

  uint16_t ox = camera_origin.first;
  uint16_t oy = camera_origin.second;

  uint16_t x1 = blob1.first;
  uint16_t y1 = blob1.second;
  uint16_t x2 = blob2.first;
  uint16_t y2 = blob2.second;

  // When both blobs are at or below the camera origin the closest blob is the
  // laser
  if (y1 <= oy && y2 <= oy) {
    double dist1 = std::hypot(x1 - ox, y1 - oy);
    double dist2 = std::hypot(x2 - ox, y2 - oy);

    if (dist1 < dist2)
      return std::make_pair(x1, y1);
    else
      return std::make_pair(x2, y2);
  }

  // When both blobs are at or above the camera origin the furthest blob is the
  // laser
  if (y1 >= oy && y2 >= oy) {
    double dist1 = std::hypot(x1 - ox, y1 - oy);
    double dist2 = std::hypot(x2 - ox, y2 - oy);

    if (dist1 > dist2)
      return std::make_pair(x1, y1);
    else
      return std::make_pair(x2, y2);
  }

  // When blobs are on either side of the camera origin the laser is the blob
  // that is above the camera origin
  if (y1 > oy)
    return std::make_pair(x1, y1);
  else
    return std::make_pair(x2, y2);
}

void save_frame_to_text_file(const uint8_t* frame,
                             size_t size,
                             const std::string& filename) {
  std::ofstream out_file(filename);

  out_file << "{";
  if (!out_file) {
    std::cerr << "Could not open file for writing: " << filename << std::endl;
    return;
  }

  for (size_t i = 0; i < size; ++i) {
    out_file << static_cast<unsigned int>(
        frame[i]);  // Cast to unsigned int for proper text output
    if (i < size - 1) {
      out_file << ", ";  // Separate each byte by space
    }
  }
  out_file << "}";

  out_file.close();
  std::cout << "Saved frame to: " << filename << std::endl;
}

std::pair<int32_t, int32_t> detect_laser(uint8_t* red_frame,
                                         uint8_t threshold) {
  cudaError_t err;

  err = cudaMemcpy(device_frame, red_frame, frame_size, cudaMemcpyHostToDevice);
  if (err != cudaSuccess) {
    printf("CUDA error 1: %s\n", cudaGetErrorString(err));
  }

  gaussian_smoothing<<<grid_size, block_size>>>(device_frame, device_temp_frame,
                                                5, 6.0f);
  err = cudaMemcpy(device_frame, device_temp_frame, frame_size,
                   cudaMemcpyDeviceToDevice);
  if (err != cudaSuccess) {
    printf("CUDA error 2: %s\n", cudaGetErrorString(err));
  }
  // cudaDeviceSynchronize();

  binarise<<<grid_size, block_size>>>(device_frame, threshold);
  close_and_open();

  err = cudaMemcpy(red_frame, device_frame, frame_size, cudaMemcpyDeviceToHost);
  if (err != cudaSuccess) {
    printf("CUDA error 3: %s\n", cudaGetErrorString(err));
  }
  // save_frame_to_text_file(red_frame, WIDTH * HEIGHT, "gpu_frame.txt");

  std::chrono::high_resolution_clock::time_point start_time =
      std::chrono::high_resolution_clock::now();

  laser_position = distinguish_laser(
      find_blobs(red_frame), std::make_pair(X_ORIGIN_PX, Y_ORIGIN_PX),
      std::make_pair(0, 0), std::make_pair(0, 0));

  std::chrono::high_resolution_clock::time_point end_time =
      std::chrono::high_resolution_clock::now();
  uint32_t duration = std::chrono::duration_cast<std::chrono::microseconds>(
                          end_time - start_time)
                          .count();
  // std::cout << "GPU processing time = " << duration << " us" << std::endl;

  cv::Mat mat(HEIGHT, WIDTH, CV_8UC1, red_frame);
  cv::putText(mat,
              "laser pos = (" + std::to_string(laser_position.first) + ", " +
                  std::to_string(laser_position.second) + ")",
              cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
              cv::Scalar(255, 255, 255), 2);
  cv::imshow("pre-processed frame", mat);
  cv::waitKey(1);
  // std::cout << "laser pos = (" << laser_position.first << ", "
  //           << laser_position.second << ")" << std::endl;
  return laser_position;
  // return std::make_pair(-2, -2);
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
