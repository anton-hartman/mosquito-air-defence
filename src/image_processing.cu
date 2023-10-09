#include <chrono>
#include <iostream>
#include <thread>
#include "../include/image_processing.hpp"

namespace gpu {

std::pair<uint16_t, uint16_t> ignore_region_top_left = {523, 293};
std::pair<uint16_t, uint16_t> ignore_region_bottom_right = {553, 316};

void set_ignore_region(std::pair<uint16_t, uint16_t> top_left,
                       std::pair<uint16_t, uint16_t> bottom_right) {
  ignore_region_top_left = top_left;
  ignore_region_bottom_right = bottom_right;
}

dim3 const block_size(16, 8);
dim3 const grid_size((COLS + block_size.x - 1) / block_size.x,
                     (ROWS + block_size.y - 1) / block_size.y);

const size_t frame_size = COLS * ROWS * sizeof(uint8_t);
uint8_t* d_frame_1;
uint8_t* d_frame_2;
uint8_t* mos_d_frame_1;
uint8_t* mos_d_frame_2;

const int struct_elem_size = 2;
const int diameter = 2 * struct_elem_size + 1;
__constant__ uint8_t d_structuring_element[diameter * diameter];
__constant__ int d_struct_elem_size;

std::pair<int32_t, int32_t> laser_position = std::make_pair(-2, -2);

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
  cudaMemcpyToSymbol(d_COLS, &COLS, sizeof(int));
  cudaMemcpyToSymbol(d_ROWS, &ROWS, sizeof(int));

  cudaMalloc((void**)&d_frame_1, frame_size);
  cudaMalloc((void**)&d_frame_2, frame_size);
  cudaMalloc((void**)&mos_d_frame_1, frame_size);
  cudaMalloc((void**)&mos_d_frame_2, frame_size);

  initialize_struct_elem();

  cudaMemcpyToSymbol(d_learning_rate, &learning_rate, sizeof(float));
  cudaMalloc((void**)&d_background, frame_size);
}

void free_gpu() {
  cudaFree(d_frame_1);
  cudaFree(d_frame_2);
  cudaFree(mos_d_frame_1);
  cudaFree(mos_d_frame_2);
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

__global__ void binarise_gt(uint8_t* device_frame, uint8_t threshold) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_COLS && y < d_ROWS) {
    device_frame[y * d_COLS + x] =
        device_frame[y * d_COLS + x] >= threshold ? 255 : 0;
  }
}

__global__ void binarise_lt(uint8_t* device_frame, uint8_t threshold) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_COLS && y < d_ROWS) {
    device_frame[y * d_COLS + x] =
        device_frame[y * d_COLS + x] <= threshold ? 255 : 0;
  }
}

__global__ void subtract_and_update_background(uint8_t* device_frame) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_COLS && y < d_ROWS) {
    d_background[y * d_COLS + x] =
        d_learning_rate * device_frame[y * d_COLS + x] +
        (1 - d_learning_rate) * d_background[y * d_COLS + x];
    device_frame[y * d_COLS + x] =
        abs(device_frame[y * d_COLS + x] - d_background[y * d_COLS + x]);
  }
}

__global__ void erosion(uint8_t* input, uint8_t* output) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_COLS && y < d_ROWS) {
    int min_val = 255;
    for (int i = -d_struct_elem_size; i <= d_struct_elem_size; i++) {
      for (int j = -d_struct_elem_size; j <= d_struct_elem_size; j++) {
        if (y + i >= 0 && y + i < d_ROWS && x + j >= 0 && x + j < d_COLS) {
          if (d_structuring_element[(i + d_struct_elem_size) *
                                        (2 * d_struct_elem_size + 1) +
                                    j + d_struct_elem_size] == 1) {
            int idx = (y + i) * d_COLS + (x + j);
            min_val = min(min_val, (int)input[idx]);
          }
        }
      }
    }
    output[y * d_COLS + x] = min_val;
  }
}

__global__ void dilation(uint8_t* input, uint8_t* output) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_COLS && y < d_ROWS) {
    int maxVal = 0;
    for (int i = -d_struct_elem_size; i <= d_struct_elem_size; i++) {
      for (int j = -d_struct_elem_size; j <= d_struct_elem_size; j++) {
        if (y + i >= 0 && y + i < d_ROWS && x + j >= 0 && x + j < d_COLS) {
          if (d_structuring_element[(i + d_struct_elem_size) *
                                        (2 * d_struct_elem_size + 1) +
                                    j + d_struct_elem_size] == 1) {
            int idx = (y + i) * d_COLS + (x + j);
            maxVal = max(maxVal, (int)input[idx]);
          }
        }
      }
    }
    output[y * d_COLS + x] = maxVal;
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

std::pair<uint16_t, uint16_t> distinguish_laser(
    const std::vector<Blob>& blobs) {
  if (blobs.size() == 1)
    return std::make_pair(blobs.at(0).cen_x, blobs.at(0).cen_y);

  std::pair<int32_t, int32_t> result = std::make_pair(-1, -1);
  double minDist = std::numeric_limits<double>::infinity();
  double maxDist = -1;

  for (size_t i = 0; i < blobs.size(); i++) {
    uint16_t x = blobs[i].cen_x;
    uint16_t y = blobs[i].cen_y;

    if (is_blob_in_ignore_region(std::make_pair(x, y), ignore_region_top_left,
                                 ignore_region_bottom_right)) {
      // std::cout << "blob in ignore region" << std::endl;
      continue;  // Skip blobs in the ignore region
    }

    double dist = std::hypot(x - C_X, y - C_Y);

    if (y <= C_Y && dist < minDist) {
      minDist = dist;
      result = std::make_pair(x, y);
    }

    if (y >= C_Y && dist > maxDist) {
      maxDist = dist;
      result = std::make_pair(x, y);
    }

    if (y > C_Y && result.first == -1)
      result = std::make_pair(x, y);
  }

  return result;
}

std::pair<uint16_t, uint16_t> distinguish_laser_only_2(
    const std::vector<Blob>& blobs) {
  if (blobs.size() == 1) {
    return std::make_pair(blobs.at(0).cen_x, blobs.at(0).cen_y);
  } else if (blobs.size() < 1) {
    return std::make_pair(-1, -1);
  }

  std::vector<Blob> two_blobs;
  for (size_t i = 0; i < blobs.size(); i++) {
    if (is_blob_in_ignore_region({blobs[i].cen_x, blobs[i].cen_y},
                                 ignore_region_top_left,
                                 ignore_region_bottom_right)) {
      continue;
    } else {
      two_blobs.push_back(blobs[i]);
    }
  }

  if (two_blobs.size() == 1) {
    return std::make_pair(two_blobs[0].cen_x, two_blobs[0].cen_y);
  } else if (two_blobs.size() < 1) {
    return std::make_pair(-2, -2);
  }

  if (two_blobs.size() > 2) {
    std::cout << "More than two blobs outside of ignore region: "
              << std::to_string(two_blobs.size()) << std::endl;
  }

  uint16_t x1 = two_blobs[0].cen_x;
  uint16_t y1 = two_blobs[0].cen_y;
  uint16_t x2 = two_blobs[1].cen_x;
  uint16_t y2 = two_blobs[1].cen_y;

  // When both blobs are at or below the camera origin, then the one closer to
  // the origin of the camera is the laser.
  if (y1 >= C_Y && y2 >= C_Y) {
    if (y1 < y2) {
      // y1 is closer to the camera origin
      return std::make_pair(x1, y1);
    } else {
      return std::make_pair(x2, y2);
    }
  }

  // When both blobs are at or above the camera origin, then the one farther
  // from the origin of the camera is the laser.
  if (y1 <= C_Y && y2 <= C_Y) {
    if (y1 < y2) {
      // y1 is further from the camera origin
      return std::make_pair(x1, y1);
    } else {
      return std::make_pair(x2, y2);
    }
  }

  // When blobs are on either side of the camera origin
  if (y1 < C_Y)
    return std::make_pair(x1, y1);
  else
    return std::make_pair(x2, y2);
}

int get_blobs(cv::Mat frame, std::vector<Blob>& blobs) {
  int i, j, k, l, r = frame.rows, c = frame.cols, id = 1;
  // Stores ID of a pixel; -1 means unvisited
  std::vector<std::vector<int>> pixel_ID(r, std::vector<int>(c, -1));
  std::queue<Pt> open_list;  // Breadth-First-Search hence queue of points

  for (i = 1; i < r - 1; i++) {
    for (j = 1; j < c - 1; j++) {
      if (i >= r || j >= c || frame.at<uint8_t>(i, j) == 0 ||
          pixel_ID[i][j] > -1) {
        continue;
      }
      Pt start = {j, i};
      open_list.push(start);
      int sum_x = 0, sum_y = 0, n_pixels = 0, max_x = 0, max_y = 0;
      int min_x = c + 1, min_y = r + 1;
      // Dequeue the element at the head of the queue
      while (!open_list.empty()) {
        Pt top = open_list.front();
        open_list.pop();
        if (top.y >= r || top.x >= c) {
          continue;
        }

        pixel_ID[top.y][top.x] = id;
        n_pixels++;
        // To obtain the bounding box of the blob w.r.t the original image
        min_x = (top.x < min_x) ? top.x : min_x;
        min_y = (top.y < min_y) ? top.y : min_y;
        max_x = (top.x > max_x) ? top.x : max_x;
        max_y = (top.y > max_y) ? top.y : max_y;
        sum_y += top.y;
        sum_x += top.x;

        // Add the 8 - connected neighbours that are yet to be visited, to the
        // queue
        for (k = top.y - 1; k <= top.y + 1; k++) {
          for (l = top.x - 1; l <= top.x + 1; l++) {
            if (k < 0 || l < 0 || k >= r || l >= c ||
                frame.at<uint8_t>(k, l) == 0 || pixel_ID[k][l] > -1) {
              continue;
            }
            Pt next = {l, k};
            pixel_ID[k][l] = id;
            open_list.push(next);
          }
        }
      }

      if (n_pixels < 20) {  // At least 20 pixels
        continue;
      }

      Blob nextcentre = {
          min_x,    max_x, min_y, max_y, sum_x / n_pixels, sum_y / n_pixels,
          n_pixels, id};
      blobs.push_back(nextcentre);
      id++;
    }
  }
  return blobs.size();
}

std::pair<int32_t, int32_t> detect_laser(cv::Mat red_frame, uint8_t threshold) {
  cudaError_t err = cudaMemcpy(d_frame_1, red_frame.ptr(), frame_size,
                               cudaMemcpyHostToDevice);
  (err != cudaSuccess) ? printf("CUDA err: %s\n", cudaGetErrorString(err)) : 0;

  gaussian_smoothing<<<grid_size, block_size>>>(d_frame_1, d_frame_2, 5, 6.0f);
  binarise_gt<<<grid_size, block_size>>>(d_frame_2, threshold);
  dilation<<<grid_size, block_size>>>(d_frame_2, d_frame_1);
  erosion<<<grid_size, block_size>>>(d_frame_1, d_frame_2);
  erosion<<<grid_size, block_size>>>(d_frame_2, d_frame_1);
  dilation<<<grid_size, block_size>>>(d_frame_1, d_frame_2);

  err = cudaMemcpy(red_frame.ptr(), d_frame_2, frame_size,
                   cudaMemcpyDeviceToHost);
  (err != cudaSuccess) ? printf("CUDA err: %s\n", cudaGetErrorString(err)) : 0;

  int num_blobs = -2;
  std::vector<Blob> blobs;
  num_blobs = get_blobs(red_frame, blobs);
  laser_position = distinguish_laser_only_2(blobs);

  for (size_t i = 0; i < blobs.size(); i++) {
    cv::circle(red_frame, cv::Point(blobs[i].cen_x, blobs[i].cen_y), 20,
               cv::Scalar(150, 255, 255), 2);
    cv::putText(red_frame, std::to_string(i),
                cv::Point(blobs[i].cen_x + 10, blobs[i].cen_y + 10),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1);
  }
  cv::putText(red_frame,
              "laser pos = (" + std::to_string(laser_position.first) + ", " +
                  std::to_string(laser_position.second) +
                  ")  num blobs = " + std::to_string(num_blobs),
              cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
              cv::Scalar(255, 255, 255), 2);
  cv::imshow("cv::Mat processed", red_frame);
  cv::waitKey(1);
  return laser_position;
}

std::vector<Pt> detect_mosquitoes(cv::Mat red_frame,
                                  uint8_t threshold,
                                  bool bg_sub) {
  cudaError_t err = cudaMemcpy(mos_d_frame_1, red_frame.ptr(), frame_size,
                               cudaMemcpyHostToDevice);
  (err != cudaSuccess) ? printf("CUDA err: %s\n", cudaGetErrorString(err)) : 0;

  gaussian_smoothing<<<grid_size, block_size>>>(mos_d_frame_1, mos_d_frame_2, 5,
                                                6.0f);
  if (bg_sub) {
    subtract_and_update_background<<<grid_size, block_size>>>(mos_d_frame_2);
    binarise_gt<<<grid_size, block_size>>>(mos_d_frame_2, threshold);
  } else {
    binarise_lt<<<grid_size, block_size>>>(mos_d_frame_2, threshold);
  }
  dilation<<<grid_size, block_size>>>(mos_d_frame_2, mos_d_frame_1);
  erosion<<<grid_size, block_size>>>(mos_d_frame_1, mos_d_frame_2);
  erosion<<<grid_size, block_size>>>(mos_d_frame_2, mos_d_frame_1);
  dilation<<<grid_size, block_size>>>(mos_d_frame_1, mos_d_frame_2);

  err = cudaMemcpy(red_frame.ptr(), mos_d_frame_2, frame_size,
                   cudaMemcpyDeviceToHost);
  (err != cudaSuccess) ? printf("CUDA err: %s\n", cudaGetErrorString(err)) : 0;

  std::vector<Blob> blobs;
  int num_blobs = -2;
  num_blobs = get_blobs(red_frame, blobs);
  std::vector<Pt> blob_centres;
  for (size_t i = 0; i < blobs.size(); i++) {
    blob_centres.push_back({blobs[i].cen_x, blobs[i].cen_y});
  }

  if (blob_centres.size() == 0) {
    blob_centres.push_back({-1, -1});
  }

  for (size_t i = 0; i < blobs.size(); i++) {
    cv::circle(red_frame, cv::Point(blobs[i].cen_x, blobs[i].cen_y), 20,
               cv::Scalar(150, 255, 255), 2);
    cv::putText(red_frame, std::to_string(i),
                cv::Point(blobs[i].cen_x + 10, blobs[i].cen_y + 10),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1);
  }

  cv::putText(red_frame, "num blobs = " + std::to_string(num_blobs),
              cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
              cv::Scalar(255, 255, 255), 2);
  cv::imshow("mosquitoes", red_frame);
  cv::waitKey(1);
  return blob_centres;
}

}  // namespace gpu
