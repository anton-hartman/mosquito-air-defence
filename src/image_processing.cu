#include <chrono>
#include <iostream>
#include <thread>
#include "../include/frame.hpp"
#include "../include/image_processing.hpp"
#include "../include/mads.hpp"
#include "../include/tracking.hpp"

const dim3 block_size(16, 8);
const dim3 grid_size((COLS + block_size.x - 1) / block_size.x,
                     (ROWS + block_size.y - 1) / block_size.y);
const size_t frame_size = COLS * ROWS * sizeof(uint8_t);
uint8_t* d_laser_tmp;
uint8_t* d_laser_frame;
uint8_t* d_mos_tmp;
uint8_t* d_mos_frame;
uint8_t* d_background;
uint8_t* d_mos_opening_struct_elem;
uint8_t* d_mos_closing_struct_elem;
uint8_t* d_laser_opening_struct_elem;
uint8_t* d_laser_closing_struct_elem;
int mos_opening_radius;
int mos_closing_radius;
int laser_opening_radius;
int laser_closing_radius;

namespace gpu {

__constant__ int d_COLS;
__constant__ int d_ROWS;

void init_gpu() {
  cudaMemcpyToSymbol(d_COLS, &COLS, sizeof(int));
  cudaMemcpyToSymbol(d_ROWS, &ROWS, sizeof(int));

  cudaMalloc((void**)&d_laser_tmp, frame_size);
  cudaMalloc((void**)&d_laser_frame, frame_size);
  cudaMalloc((void**)&d_mos_tmp, frame_size);
  cudaMalloc((void**)&d_mos_frame, frame_size);
  cudaMalloc((void**)&d_background, frame_size);

  mos_opening_radius = 1;
  mos_closing_radius = 3;
  laser_opening_radius = 1;
  laser_closing_radius = 3;

  auto diameter = [](int x) -> int { return x * 2 + 1; };

  cudaMalloc((void**)&d_mos_opening_struct_elem,
             std::pow(diameter(mos_opening_radius), 2) * sizeof(uint8_t));
  cudaMalloc((void**)&d_mos_closing_struct_elem,
             std::pow(diameter(mos_closing_radius), 2) * sizeof(uint8_t));
  cudaMalloc((void**)&d_laser_opening_struct_elem,
             std::pow(diameter(laser_opening_radius), 2) * sizeof(uint8_t));
  cudaMalloc((void**)&d_laser_closing_struct_elem,
             std::pow(diameter(laser_closing_radius), 2) * sizeof(uint8_t));

  set_struct_elem(mos_opening_radius, StructElemType::MOS_OPENING);
  set_struct_elem(mos_closing_radius, StructElemType::MOS_CLOSING);
  set_struct_elem(laser_opening_radius, StructElemType::LASER_OPENING);
  set_struct_elem(laser_closing_radius, StructElemType::LASER_CLOSING);
}

void free_gpu() {
  cudaFree(d_laser_tmp);
  cudaFree(d_laser_frame);
  cudaFree(d_mos_tmp);
  cudaFree(d_mos_frame);
  cudaFree(d_background);
  cudaFree(d_mos_opening_struct_elem);
  cudaFree(d_mos_closing_struct_elem);
  cudaFree(d_laser_opening_struct_elem);
  cudaFree(d_laser_closing_struct_elem);
}

void set_struct_elem(int struct_elem_radius, StructElemType type) {
  int diameter = 2 * struct_elem_radius + 1;
  uint8_t* host_struct_elem = new uint8_t[diameter * diameter];

  for (int i = 0; i < diameter; i++) {
    for (int j = 0; j < diameter; j++) {
      int y = i - struct_elem_radius;  // y-coordinate relative to the center
      int x = j - struct_elem_radius;  // x-coordinate relative to the center
      host_struct_elem[i * diameter + j] =
          (x * x + y * y <= struct_elem_radius * struct_elem_radius) ? 1 : 0;
    }
  }

  int elem_memory_size = diameter * diameter * sizeof(uint8_t);
  switch (type) {
    case StructElemType::MOS_OPENING:
      cudaFree(d_mos_opening_struct_elem);
      cudaMalloc((void**)&d_mos_opening_struct_elem, elem_memory_size);
      cudaMemcpy(d_mos_opening_struct_elem, host_struct_elem, elem_memory_size,
                 cudaMemcpyHostToDevice);
      mos_opening_radius = struct_elem_radius;
      break;
    case StructElemType::MOS_CLOSING:
      cudaFree(d_mos_closing_struct_elem);
      cudaMalloc((void**)&d_mos_closing_struct_elem, elem_memory_size);
      cudaMemcpy(d_mos_closing_struct_elem, host_struct_elem, elem_memory_size,
                 cudaMemcpyHostToDevice);
      mos_closing_radius = struct_elem_radius;
      break;
    case StructElemType::LASER_OPENING:
      cudaFree(d_laser_opening_struct_elem);
      cudaMalloc((void**)&d_laser_opening_struct_elem, elem_memory_size);
      cudaMemcpy(d_laser_opening_struct_elem, host_struct_elem,
                 elem_memory_size, cudaMemcpyHostToDevice);
      laser_opening_radius = struct_elem_radius;
      break;
    case StructElemType::LASER_CLOSING:
      cudaFree(d_laser_closing_struct_elem);
      cudaMalloc((void**)&d_laser_closing_struct_elem, elem_memory_size);
      cudaMemcpy(d_laser_closing_struct_elem, host_struct_elem,
                 elem_memory_size, cudaMemcpyHostToDevice);
      laser_closing_radius = struct_elem_radius;
      break;
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

/**
 * @brief Erosion is the minimum value of the pixels covered by the
 structuring
 * element.
 */
__global__ void erosion(uint8_t* input,
                        uint8_t* output,
                        uint8_t* struct_elem,
                        int struct_elem_radius) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_COLS && y < d_ROWS) {
    int min_val = 255;
    for (int i = -struct_elem_radius; i <= struct_elem_radius; i++) {
      for (int j = -struct_elem_radius; j <= struct_elem_radius; j++) {
        if (y + i >= 0 && y + i < d_ROWS && x + j >= 0 && x + j < d_COLS) {
          if (struct_elem[(i + struct_elem_radius) *
                              (2 * struct_elem_radius + 1) +
                          j + struct_elem_radius] == 1) {
            int idx = (y + i) * d_COLS + (x + j);
            min_val = min(min_val, (int)input[idx]);
          }
        }
      }
    }
    output[y * d_COLS + x] = min_val;
  }
}

/**
 * @brief Dilation is the maximum value of the pixels covered by the
 * structuring element.
 */
__global__ void dilation(uint8_t* input,
                         uint8_t* output,
                         uint8_t* struct_elem,
                         int struct_elem_radius) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_COLS && y < d_ROWS) {
    int max_val = 0;
    for (int i = -struct_elem_radius; i <= struct_elem_radius; i++) {
      for (int j = -struct_elem_radius; j <= struct_elem_radius; j++) {
        if (y + i >= 0 && y + i < d_ROWS && x + j >= 0 && x + j < d_COLS) {
          if (struct_elem[(i + struct_elem_radius) *
                              (2 * struct_elem_radius + 1) +
                          j + struct_elem_radius] == 1) {
            int idx = (y + i) * d_COLS + (x + j);
            max_val = max(max_val, (int)input[idx]);
          }
        }
      }
    }
    output[y * d_COLS + x] = max_val;
  }
}

/**
 * @brief Opening is erosion followed by dilation with the same structuring
 * element.
 */
void opening(uint8_t* input_and_output,
             uint8_t* temp,
             uint8_t* struct_elem,
             int struct_elem_radius) {
  erosion<<<grid_size, block_size>>>(input_and_output, temp, struct_elem,
                                     struct_elem_radius);
  dilation<<<grid_size, block_size>>>(temp, input_and_output, struct_elem,
                                      struct_elem_radius);
}

/**
 * @brief Closing is dilation followed by erosion with the same structuring
 * element.
 */
void closing(uint8_t* input_and_output,
             uint8_t* temp,
             uint8_t* struct_elem,
             int struct_elem_radius) {
  dilation<<<grid_size, block_size>>>(input_and_output, temp, struct_elem,
                                      struct_elem_radius);
  erosion<<<grid_size, block_size>>>(temp, input_and_output, struct_elem,
                                     struct_elem_radius);
}

}  // namespace gpu

namespace detection {

Pt ignore_region_top_left = Pt{538, 288};
Pt ignore_region_bottom_right = Pt{556, 302};
std::atomic<float> bg_learning_rate(0.0);

void set_ignore_region(Pt top_left, Pt bottom_right) {
  ignore_region_top_left = top_left;
  ignore_region_bottom_right = bottom_right;
}

bool is_pt_in_ignore_region(const Pt& pt,
                            const Pt& ignore_region_top_left,
                            const Pt& ignore_region_bottom_right) {
  return (pt.x >= ignore_region_top_left.x and
          pt.x <= ignore_region_bottom_right.x and
          pt.y >= ignore_region_top_left.y and
          pt.y <= ignore_region_bottom_right.y);
}

void set_background(const cv::Mat& frame) {
  cudaMemcpy(d_background, frame.ptr(), frame_size, cudaMemcpyHostToDevice);
}

Pt distinguish_with_x(const std::vector<Pt>& two_pts) {
  int x0 = two_pts.at(0).x;
  int x1 = two_pts.at(1).x;

  // When both lasers are at or below the camera origin, then the one closer to
  // the origin of the camera is the laser.
  if (x0 >= mads::C_X and x1 >= mads::C_X) {
    if (x0 < x1) {
      // x0 is closer to the camera origin
      return two_pts.at(0);
    } else {
      return two_pts.at(1);
    }
  }

  // When both lasers are at or above the camera origin, then the one farther
  // from the origin of the camera is the laser.
  if (x0 <= mads::C_X and x1 <= mads::C_X) {
    if (x0 < x1) {
      // x0 is further from the camera origin
      return two_pts.at(0);
    } else {
      return two_pts.at(1);
    }
  }

  // When lasers are on either side of the camera origin then the one above the
  // origin is the laser
  if (x0 < mads::C_X) {
    return two_pts.at(0);
  } else {
    return two_pts.at(1);
  }
}

Pt distinguish_with_y(const std::vector<Pt>& two_pts) {
  int y0 = two_pts.at(0).y;
  int y1 = two_pts.at(1).y;

  // When both lasers are at or below the camera origin, then the one closer to
  // the origin of the camera is the laser.
  if (y0 >= mads::C_Y and y1 >= mads::C_Y) {
    if (y0 < y1) {
      // y0 is closer to the camera origin
      return two_pts.at(0);
    } else {
      return two_pts.at(1);
    }
  }

  // When both lasers are at or above the camera origin, then the one farther
  // from the origin of the camera is the laser.
  if (y0 <= mads::C_Y and y1 <= mads::C_Y) {
    if (y0 < y1) {
      // y0 is further from the camera origin
      return two_pts.at(0);
    } else {
      return two_pts.at(1);
    }
  }

  // When lasers are on either side of the camera origin then the one above the
  // origin is the laser
  if (y0 < mads::C_Y) {
    return two_pts.at(0);
  } else {
    return two_pts.at(1);
  }
}

Pt distinguish_lasers(const std::vector<Pt>& pts) {
  if (pts.size() == 0) {
    return {-1, -1};
  }

  std::vector<Pt> two_pts;
  for (size_t i = 0; i < pts.size(); i++) {
    if (is_pt_in_ignore_region(pts.at(i), ignore_region_top_left,
                               ignore_region_bottom_right)) {
      continue;
    } else {
      two_pts.push_back(pts.at(i));
    }
  }

  if (two_pts.size() == 1) {
    return two_pts.at(0);
  } else if (two_pts.size() == 0) {
    return {-2, -2};
  } else if (two_pts.size() > 2) {
    return {-3, -3};
  }

  // if (std::abs(two_pts.at(0).x - two_pts.at(1).x) >=
  //     std::abs(two_pts.at(0).y - two_pts.at(1).y)) {
  //   return distinguish_with_x(two_pts);
  // } else {
  //   return distinguish_with_y(two_pts);
  // }
  return distinguish_with_y(two_pts);
}

int get_blobs(const cv::Mat& frame, std::vector<Pt>& blobs) {
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

        // Add the 8 connected neighbours that are yet to be visited, to the
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

      // if (n_pixels < 2) {  // At least x pixels
      //   continue;
      // }

      Pt nextcentre = {sum_x / n_pixels, sum_y / n_pixels};
      blobs.push_back(nextcentre);
      id++;
    }
  }
  return blobs.size();
}

std::vector<Pt> detect_lasers(cv::Mat red_frame, const uint8_t threshold) {
  cudaError_t err = cudaMemcpy(d_laser_frame, red_frame.ptr(), frame_size,
                               cudaMemcpyHostToDevice);
  (err != cudaSuccess) ? printf("CUDA err: %s\n", cudaGetErrorString(err)) : 0;

  gpu::binarise_gt<<<grid_size, block_size>>>(d_laser_frame, threshold);
  gpu::opening(d_laser_frame, d_laser_tmp, d_laser_opening_struct_elem,
               laser_opening_radius);
  if (mads::display.load() & Display::LASER_DETECTION and
      mads::debug == Debug::DEEP) {
    cv::Mat temp(ROWS, COLS, CV_8UC1);
    cudaMemcpy(temp.ptr(), d_laser_frame, frame_size, cudaMemcpyDeviceToHost);
    cv::putText(
        temp, "laser opening radius = " + std::to_string(laser_opening_radius),
        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
        cv::Scalar(255, 255, 255), 2);
    cv::circle(temp, cv::Point(50, 50), laser_opening_radius,
               cv::Scalar(150, 0, 0), -1, 4);
    cv::imshow("laser opened", temp);
    cv::waitKey(1);
  }

  gpu::closing(d_laser_frame, d_laser_tmp, d_laser_closing_struct_elem,
               laser_closing_radius);
  if (mads::display.load() & Display::LASER_DETECTION and
      mads::debug == Debug::DEEP) {
    cv::Mat temp(ROWS, COLS, CV_8UC1);
    cudaMemcpy(temp.ptr(), d_laser_frame, frame_size, cudaMemcpyDeviceToHost);
    cv::putText(
        temp, "laser closing radius = " + std::to_string(laser_closing_radius),
        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
        cv::Scalar(255, 255, 255), 2);
    cv::circle(temp, cv::Point(50, 50), laser_closing_radius,
               cv::Scalar(150, 0, 0), -1, 4);
    cv::imshow("laser closed", temp);
    cv::waitKey(1);
  }

  err = cudaMemcpy(red_frame.ptr(), d_laser_frame, frame_size,
                   cudaMemcpyDeviceToHost);
  (err != cudaSuccess) ? printf("CUDA err: %s\n", cudaGetErrorString(err)) : 0;

  int num_laser_pts = -2;
  std::vector<Pt> laser_pts;
  num_laser_pts = get_blobs(red_frame, laser_pts);

  if (mads::display.load() & Display::LASER_DETECTION) {
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
    cv::imshow("laser detection (not distinguished)", red_frame);
    cv::waitKey(1);
  }
  return laser_pts;
}

std::vector<Pt> detect_mosquitoes(cv::Mat red_frame,
                                  const uint8_t threshold,
                                  const bool bg_sub) {
  cudaError_t err = cudaMemcpy(d_mos_frame, red_frame.ptr(), frame_size,
                               cudaMemcpyHostToDevice);
  (err != cudaSuccess) ? printf("CUDA err: %s\n", cudaGetErrorString(err)) : 0;

  if (bg_sub) {
    gpu::subtract_and_update_background<<<grid_size, block_size>>>(
        d_mos_frame, d_background, bg_learning_rate);
    if (mads::display.load() & Display::MOSQUITO_DETECTION and
        mads::debug == Debug::ON) {
      cv::Mat temp(ROWS, COLS, CV_8UC1);
      cudaMemcpy(temp.ptr(), d_background, frame_size, cudaMemcpyDeviceToHost);
      cv::imshow("background", temp);
      cv::waitKey(1);

      cudaMemcpy(temp.ptr(), d_mos_frame, frame_size, cudaMemcpyDeviceToHost);
      cv::imshow("subtracted", temp);
      cv::waitKey(1);
    }

    gpu::binarise_gt<<<grid_size, block_size>>>(d_mos_frame, threshold);
  } else {
    gpu::binarise_lt<<<grid_size, block_size>>>(d_mos_frame, threshold);
  }

  gpu::opening(d_mos_frame, d_mos_tmp, d_mos_opening_struct_elem,
               mos_opening_radius);
  if (mads::display.load() & Display::MOSQUITO_DETECTION and
      mads::debug == Debug::DEEP) {
    cv::Mat temp(ROWS, COLS, CV_8UC1);
    cudaMemcpy(temp.ptr(), d_mos_frame, frame_size, cudaMemcpyDeviceToHost);
    cv::putText(temp,
                "mos opening radius = " + std::to_string(mos_opening_radius),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
                cv::Scalar(255, 255, 255), 2);
    cv::circle(temp, cv::Point(50, 50), mos_opening_radius,
               cv::Scalar(150, 0, 0), -1, 4);
    cv::imshow("mos opened", temp);
    cv::waitKey(1);
  }

  gpu::closing(d_mos_frame, d_mos_tmp, d_mos_closing_struct_elem,
               mos_closing_radius);
  if (mads::display.load() & Display::MOSQUITO_DETECTION and
      mads::debug == Debug::DEEP) {
    cv::Mat temp(ROWS, COLS, CV_8UC1);
    cudaMemcpy(temp.ptr(), d_mos_frame, frame_size, cudaMemcpyDeviceToHost);
    cv::putText(temp,
                "mos closing radius = " + std::to_string(mos_closing_radius),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
                cv::Scalar(255, 255, 255), 2);
    cv::circle(temp, cv::Point(50, 50), mos_closing_radius,
               cv::Scalar(150, 0, 0), -1, 4);
    cv::imshow("mos closed", temp);
    cv::waitKey(1);
  }

  err = cudaMemcpy(red_frame.ptr(), d_mos_frame, frame_size,
                   cudaMemcpyDeviceToHost);
  (err != cudaSuccess) ? printf("CUDA err: %s\n", cudaGetErrorString(err)) : 0;

  std::vector<Pt> mos_pts;
  int num_mos_pts = -2;
  num_mos_pts = get_blobs(red_frame, mos_pts);

  if (mads::display.load() & Display::MOSQUITO_DETECTION) {
    for (int i = 0; i < mos_pts.size(); ++i) {
      cv::circle(red_frame, mos_pts.at(i).cv_pt(), 20,
                 cv::Scalar(150, 255, 255), 2);
      cv::putText(red_frame, std::to_string(i),
                  cv::Point(mos_pts.at(i).x + 10, mos_pts.at(i).y + 10),
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1);
    }

    cv::putText(red_frame, "num mos_pts = " + std::to_string(num_mos_pts),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
                cv::Scalar(255, 255, 255), 2);
    cv::imshow("mosquitoes", red_frame);
    cv::waitKey(1);
  }
  return mos_pts;
}

void remove_lasers_from_mos(const std::vector<Pt>& laser_pts,
                            std::vector<Pt>& mos_pts,
                            const int remove_radius) {
  auto euclidean_dist = [](Pt pt1, Pt pt2) {
    return std::sqrt(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2));
  };

  for (std::vector<Pt>::iterator it = mos_pts.begin(); it != mos_pts.end();) {
    bool remove = false;
    for (const Pt& laser_pt : laser_pts) {
      if (euclidean_dist(*it, laser_pt) < remove_radius) {
        remove = true;
        break;
      }
    }
    if (remove) {
      it = mos_pts.erase(it);
    } else {
      ++it;
    }
  }

  if (mads::display.load() & Display::MOSQUITO_DETECTION) {
    cv::Mat black_image(ROWS, COLS, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::circle(black_image, cv::Point(300, 50), remove_radius,
               cv::Scalar(150, 0, 0), -1, 4);
    for (size_t i = 0; i < mos_pts.size(); ++i) {
      cv::circle(black_image, mos_pts.at(i).cv_pt(), 15,
                 cv::Scalar(150, 255, 255), 2);
      cv::putText(black_image, std::to_string(i),
                  cv::Point(mos_pts.at(i).x + 10, mos_pts.at(i).y + 10),
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1);
    }
    cv::imshow("laser removed mossies", black_image);
    cv::waitKey(1);
  }
}

}  // namespace detection

namespace tracking {
// Tracking tracker;

// void track_mosquitoes(const std::vector<Pt>& blob_centres) {
//   tracker.associate_and_update_tracks(blob_centres);
//   // tracker.predict_centres();
// }

// std::vector<Pt> get_tracked_mosquitoes(const std::vector<Pt>& blob_centres) {
//   track_mosquitoes(blob_centres);
//   // return tracker.get_predicted_centres();
//   return std::vector<Pt>();
// }

// Pt get_tracked_mosquito(const std::vector<Pt>& blob_centres) {
//   track_mosquitoes(blob_centres);
//   return tracker.get_predicted_centre(-1);
// }

}  // namespace tracking
