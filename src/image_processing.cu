#include <chrono>
#include <iostream>
#include <thread>
#include "../include/frame.hpp"
#include "../include/image_processing.hpp"
#include "../include/mads.hpp"

namespace gpu {

Pt ignore_region_top_left = Pt{525, 268};
Pt ignore_region_bottom_right = Pt{578, 305};

dim3 const block_size(16, 8);
dim3 const grid_size((COLS + block_size.x - 1) / block_size.x,
                     (ROWS + block_size.y - 1) / block_size.y);

const size_t frame_size = COLS * ROWS * sizeof(uint8_t);
uint8_t* d_frame_1;
uint8_t* d_frame_2;
uint8_t* mos_d_frame_1;
uint8_t* mos_d_frame_2;
__constant__ float d_learning_rate;
uint8_t* d_background;

const int struct_elem_size = 1;
const int diameter = 2 * struct_elem_size + 1;
__constant__ uint8_t d_structuring_element[diameter * diameter];
__constant__ int d_struct_elem_size;

void set_ignore_region(Pt top_left, Pt bottom_right) {
  ignore_region_top_left = top_left;
  ignore_region_bottom_right = bottom_right;
}

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
        device_frame[y * d_COLS + x] > threshold ? 255 : 0;
  }
}

__global__ void binarise_lt(uint8_t* device_frame, uint8_t threshold) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_COLS && y < d_ROWS) {
    device_frame[y * d_COLS + x] =
        device_frame[y * d_COLS + x] < threshold ? 255 : 0;
  }
}

__global__ void subtract_and_update_background(uint8_t* device_frame,
                                               uint8_t* bg_frame) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_COLS && y < d_ROWS) {
    bg_frame[y * d_COLS + x] = d_learning_rate * device_frame[y * d_COLS + x] +
                               (1 - d_learning_rate) * bg_frame[y * d_COLS + x];
    device_frame[y * d_COLS + x] =
        abs(bg_frame[y * d_COLS + x] - device_frame[y * d_COLS + x]);
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

bool is_pt_in_ignore_region(const Pt& pt,
                            const Pt& ignore_region_top_left,
                            const Pt& ignore_region_bottom_right) {
  int x = pt.x;
  int y = pt.y;
  return (x >= ignore_region_top_left.x && x <= ignore_region_bottom_right.x &&
          y >= ignore_region_top_left.y && y <= ignore_region_bottom_right.y);
}

Pt distinguish_laser_only_2(const std::vector<Pt>& pts) {
  if (pts.size() == 1) {
    return {pts.at(0).x, pts.at(0).y};
  } else if (pts.size() < 1) {
    return {-1, -1};
  }

  std::vector<Pt> two_pts;
  for (size_t i = 0; i < pts.size(); i++) {
    if (is_pt_in_ignore_region({pts.at(i).x, pts.at(i).y},
                               ignore_region_top_left,
                               ignore_region_bottom_right)) {
      continue;
    } else {
      two_pts.push_back(pts[i]);
    }
  }

  if (two_pts.size() == 1) {
    return {two_pts.at(0).x, two_pts.at(0).y};
  } else if (two_pts.size() < 1) {
    return {-2, -2};
  }

  // put a thrid level of debugging
  // if (debug.load() == level3) {
  // if (two_pts.size() > 2) {
  //   std::cout << std::to_string(two_pts.size())
  //             << " lasers outside of ignore region (max = 2)." << std::endl;
  // }
  // }

  uint16_t x1 = two_pts.at(0).x;
  uint16_t y1 = two_pts.at(0).y;
  uint16_t x2 = two_pts.at(1).x;
  uint16_t y2 = two_pts.at(1).y;

  // When both lasers are at or below the camera origin, then the one closer to
  // the origin of the camera is the laser.
  if (y1 >= mads::C_Y && y2 >= mads::C_Y) {
    if (y1 < y2) {
      // y1 is closer to the camera origin
      return {x1, y1};
    } else {
      return {x2, y2};
    }
  }

  // When both lasers are at or above the camera origin, then the one farther
  // from the origin of the camera is the laser.
  if (y1 <= mads::C_Y && y2 <= mads::C_Y) {
    if (y1 < y2) {
      // y1 is further from the camera origin
      return {x1, y1};
    } else {
      return {x2, y2};
    }
  }

  // When lasers are on either side of the camera origin
  if (y1 < mads::C_Y)
    return {x1, y1};
  else
    return {x2, y2};
}

int get_blobs(cv::Mat frame, std::vector<Pt>& blobs) {
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

      if (n_pixels < 5) {  // At least 20 pixels
        continue;
      }

      Pt nextcentre = {sum_x / n_pixels, sum_y / n_pixels};
      blobs.push_back(nextcentre);
      id++;
    }
  }
  return blobs.size();
}

std::vector<Pt> detect_laser(cv::Mat red_frame, uint8_t threshold) {
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

  int num_laser_pts = -2;
  std::vector<Pt> laser_pts;
  num_laser_pts = get_blobs(red_frame, laser_pts);
  // laser_position = distinguish_laser_only_2(blobs);

  for (size_t i = 0; i < laser_pts.size(); i++) {
    cv::circle(red_frame, laser_pts.at(i).cv_pt(), 20,
               cv::Scalar(150, 255, 255), 2);
    cv::putText(red_frame, std::to_string(i),
                cv::Point(laser_pts.at(i).x + 10, laser_pts.at(i).y + 10),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1);
  }
  cv::putText(red_frame, "num lasers = " + std::to_string(num_laser_pts),
              cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
              cv::Scalar(255, 255, 255), 2);
  cv::imshow("laser detection", red_frame);
  cv::waitKey(1);
  return laser_pts;
}

void set_background(const cv::Mat& frame) {
  cudaMemcpy(d_background, frame.ptr(), frame_size, cudaMemcpyHostToDevice);
}

void set_bg_learning_rate(const float& bg_learning_rate) {
  cudaMemcpyToSymbol(d_learning_rate, &bg_learning_rate, sizeof(float));
}

std::vector<Pt> detect_mosquitoes(cv::Mat red_frame,
                                  uint8_t threshold,
                                  bool bg_sub) {
  // cudaError_t err = cudaMemcpy(mos_d_frame_1, red_frame.ptr(), frame_size,
  //                              cudaMemcpyHostToDevice);
  cudaError_t err = cudaMemcpy(mos_d_frame_2, red_frame.ptr(), frame_size,
                               cudaMemcpyHostToDevice);
  (err != cudaSuccess) ? printf("CUDA err: %s\n", cudaGetErrorString(err)) : 0;

  // gaussian_smoothing<<<grid_size, block_size>>>(mos_d_frame_1, mos_d_frame_2,
  // 5, 6.0f);
  if (bg_sub) {
    subtract_and_update_background<<<grid_size, block_size>>>(mos_d_frame_2,
                                                              d_background);
    cv::Mat temp(ROWS, COLS, CV_8UC1);
    cudaMemcpy(temp.ptr(), d_background, frame_size, cudaMemcpyDeviceToHost);
    cv::imshow("background", temp);
    cv::waitKey(1);

    cudaMemcpy(temp.ptr(), mos_d_frame_2, frame_size, cudaMemcpyDeviceToHost);
    cv::imshow("subtracted", temp);
    cv::waitKey(1);

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

  std::vector<Pt> mos_pts;
  int num_mos_pts = -2;
  num_mos_pts = get_blobs(red_frame, mos_pts);

  if (mos_pts.size() == 0) {
    mos_pts.push_back(Pt{-1, -1});
  }

  for (size_t i = 0; i < mos_pts.size(); i++) {
    cv::circle(red_frame, mos_pts.at(i).cv_pt(), 20, cv::Scalar(150, 255, 255),
               2);
    cv::putText(red_frame, std::to_string(i),
                cv::Point(mos_pts.at(i).x + 10, mos_pts.at(i).y + 10),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1);
  }

  cv::putText(red_frame, "num mos_pts = " + std::to_string(num_mos_pts),
              cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
              cv::Scalar(255, 255, 255), 2);
  cv::imshow("mosquitoes", red_frame);
  cv::waitKey(1);
  return mos_pts;
}

}  // namespace gpu
