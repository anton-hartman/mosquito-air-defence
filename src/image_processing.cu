#include <iostream>
#include "../include/image_processing.hpp"

namespace gpu {

typedef struct pt_ {
  int x, y;
} pt;

typedef struct blob_ {
  int min_x, max_x;
  int min_y, max_y;
  int cen_x, cen_y;
  int n_pixels;
  int ID;
} blob;

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

std::pair<uint16_t, uint16_t> distinguish_laser(
    const std::vector<blob>& blobs,
    const std::pair<uint16_t, uint16_t> camera_origin,
    const std::pair<uint16_t, uint16_t> ignore_region_top_left,
    const std::pair<uint16_t, uint16_t> ignore_region_bottom_right) {
  if (blobs.size() == 1)
    return std::make_pair(blobs.at(0).cen_x, blobs.at(0).cen_y);

  std::pair<int32_t, int32_t> result = std::make_pair(-1, -1);
  double minDist = std::numeric_limits<double>::infinity();
  double maxDist = -1;

  uint16_t ox = camera_origin.first;
  uint16_t oy = camera_origin.second;

  for (size_t i = 0; i < blobs.size(); i++) {
    uint16_t x = blobs[i].cen_x;
    uint16_t y = blobs[i].cen_y;

    if (is_blob_in_ignore_region(std::make_pair(x, y), ignore_region_top_left,
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

  return result;
}

void get_blobs(uint8_t* frame, std::vector<blob>& blobs) {
  // int i, j, k, l, r = img.rows, c = img.cols, id = 1;
  int i, j, k, l, r = HEIGHT, c = WIDTH, id = 1;
  std::vector<std::vector<int>> pixel_ID(r, std::vector<int>(c, -1));
  // Stores ID of a pixel; -1 means unvisited
  std::queue<pt> open_list;
  // Breadth-First-Search hence queue of points
  for (i = 1; i < r - 1; i++) {
    for (j = 1; j < c - 1; j++) {
      if (frame[i * WIDTH + j] == 0 || pixel_ID[i][j] > -1) {
        continue;
      }
      pt start = {j, i};
      open_list.push(start);
      int sum_x = 0, sum_y = 0, n_pixels = 0, max_x = 0, max_y = 0;
      int min_x = c + 1, min_y = r + 1;
      while (!open_list.empty()) {  // Dequeue the element at the head of the
                                    // queue
        pt top = open_list.front();
        open_list.pop();
        pixel_ID[top.y][top.x] = id;
        n_pixels++;  // To obtain the bounding box of the blob w.r.t the
                     // original image
        min_x = (top.x < min_x) ? top.x : min_x;
        min_y = (top.y < min_y) ? top.y : min_y;
        max_x = (top.x > max_x) ? top.x : max_x;
        max_y = (top.y > max_y) ? top.y : max_y;
        sum_y += top.y;
        sum_x += top.x;  // Add the 8-connected neighbours that are yet to be
                         // visited, to the queue
        for (k = top.y - 1; k <= top.y + 1; k++) {
          for (l = top.x - 1; l <= top.x + 1; l++) {
            if (frame[k * WIDTH + l] == 0 || pixel_ID[k][l] > -1) {
              continue;
            }
            pt next = {l, k};
            pixel_ID[k][l] = id;
            open_list.push(next);
          }
        }
      }

      if (n_pixels < 20) {  // At least 20 pixels
        continue;
      }

      blob nextcentre = {
          min_x,    max_x, min_y, max_y, sum_x / n_pixels, sum_y / n_pixels,
          n_pixels, id};
      blobs.push_back(nextcentre);
      id++;
    }
  }
  std::cout
      << blobs.size();  // To test correctness; can use the vector as desired
}

void get_blobs(cv::Mat frame, std::vector<blob>& blobs) {
  // int i, j, k, l, r = img.rows, c = img.cols, id = 1;
  int i, j, k, l, r = HEIGHT, c = WIDTH, id = 1;
  std::vector<std::vector<int>> pixel_ID(r, std::vector<int>(c, -1));
  // Stores ID of a pixel; -1 means unvisited
  std::queue<pt> open_list;
  // Breadth-First-Search hence queue of points
  for (i = 1; i < r - 1; i++) {
    for (j = 1; j < c - 1; j++) {
      if (frame.at<uint8_t>(i, j) == 0 || pixel_ID[i][j] > -1) {
        // if (frame[i * WIDTH + j] == 0 || pixel_ID[i][j] > -1) {
        continue;
      }
      pt start = {j, i};
      open_list.push(start);
      int sum_x = 0, sum_y = 0, n_pixels = 0, max_x = 0, max_y = 0;
      int min_x = c + 1, min_y = r + 1;
      while (!open_list.empty()) {  // Dequeue the element at the head of the
                                    // queue
        pt top = open_list.front();
        open_list.pop();
        pixel_ID[top.y][top.x] = id;
        n_pixels++;  // To obtain the bounding box of the blob w.r.t the
                     // original image
        min_x = (top.x < min_x) ? top.x : min_x;
        min_y = (top.y < min_y) ? top.y : min_y;
        max_x = (top.x > max_x) ? top.x : max_x;
        max_y = (top.y > max_y) ? top.y : max_y;
        sum_y += top.y;
        sum_x += top.x;  // Add the 8-connected neighbours that are yet to be
                         // visited, to the queue
        for (k = top.y - 1; k <= top.y + 1; k++) {
          for (l = top.x - 1; l <= top.x + 1; l++) {
            if (frame.at<uint8_t>(k, l) == 0 || pixel_ID[k][l] > -1) {
              // if (frame[k * WIDTH + l] == 0 || pixel_ID[k][l] > -1) {
              continue;
            }
            pt next = {l, k};
            pixel_ID[k][l] = id;
            open_list.push(next);
          }
        }
      }

      if (n_pixels < 20) {  // At least 20 pixels
        continue;
      }

      blob nextcentre = {
          min_x,    max_x, min_y, max_y, sum_x / n_pixels, sum_y / n_pixels,
          n_pixels, id};
      blobs.push_back(nextcentre);
      id++;
    }
  }
  std::cout
      << blobs.size();  // To test correctness; can use the vector as desired
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

  std::vector<blob> blobs;
  get_blobs(red_frame, blobs);
  laser_position =
      distinguish_laser(blobs, std::make_pair(X_ORIGIN_PX, Y_ORIGIN_PX),
                        std::make_pair(0, 0), std::make_pair(0, 0));

  cv::Mat mat(HEIGHT, WIDTH, CV_8UC1, red_frame);
  cv::putText(mat,
              "laser pos = (" + std::to_string(laser_position.first) + ", " +
                  std::to_string(laser_position.second) + ")",
              cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
              cv::Scalar(255, 255, 255), 2);
  cv::imshow("pre-processed frame", mat);
  cv::waitKey(1);
  return laser_position;
}

std::pair<int32_t, int32_t> detect_laser(cv::Mat red_frame, uint8_t threshold) {
  // cudaError_t err;

  // Upload the CPU input to the GPU
  cv::cuda::GpuMat input_gpu_mat;
  input_gpu_mat.upload(red_frame);
  // Create an output GpuMat
  cv::cuda::GpuMat output_gpu_mat(red_frame.size(), red_frame.type());

  gaussian_smoothing<<<grid_size, block_size>>>(
      input_gpu_mat.ptr<uint8_t>(), output_gpu_mat.ptr<uint8_t>(), 5, 6.0f);
  // err = cudaMemcpy(device_frame, device_temp_frame, frame_size,
  //                  cudaMemcpyDeviceToDevice);
  // if (err != cudaSuccess) {
  //   printf("CUDA error 2: %s\n", cudaGetErrorString(err));
  // }
  // cudaDeviceSynchronize();

  binarise<<<grid_size, block_size>>>(output_gpu_mat.ptr<uint8_t>(), threshold);
  close_and_open();

  // err = cudaMemcpy(red_frame, device_frame, frame_size,
  // cudaMemcpyDeviceToHost); if (err != cudaSuccess) {
  //   printf("CUDA error 3: %s\n", cudaGetErrorString(err));
  // }

  // Download the result back to CPU
  output_gpu_mat.download(red_frame);
  std::vector<blob> blobs;
  get_blobs(red_frame, blobs);
  laser_position =
      distinguish_laser(blobs, std::make_pair(X_ORIGIN_PX, Y_ORIGIN_PX),
                        std::make_pair(0, 0), std::make_pair(0, 0));

  // cv::Mat mat(HEIGHT, WIDTH, CV_8UC1, red_frame);
  cv::putText(red_frame,
              "laser pos = (" + std::to_string(laser_position.first) + ", " +
                  std::to_string(laser_position.second) + ")",
              cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
              cv::Scalar(255, 255, 255), 2);
  cv::imshow("pre-processed frame", red_frame);
  cv::waitKey(1);
  return laser_position;
}

void opening() {
  erosion<<<grid_size, block_size>>>(device_frame, device_temp_frame);
  dilation<<<grid_size, block_size>>>(device_temp_frame, device_frame);
}

void opening(uint8_t* input, uint8_t* output) {
  erosion<<<grid_size, block_size>>>(input, output);
  dilation<<<grid_size, block_size>>>(output, input);
}

void closing() {
  dilation<<<grid_size, block_size>>>(device_frame, device_temp_frame);
  erosion<<<grid_size, block_size>>>(device_temp_frame, device_frame);
}

void closing(uint8_t* input, uint8_t* output) {
  dilation<<<grid_size, block_size>>>(output, input);
  erosion<<<grid_size, block_size>>>(input, output);
}

void open_and_close() {
  opening();
  closing();
}

void close_and_open() {
  closing();
  opening();
}

std::vector<blob> detect_mosquitoes(cv::Mat red_frame, uint8_t threshold) {
  cv::cuda::GpuMat input_gpu_mat;
  input_gpu_mat.upload(red_frame);
  cv::cuda::GpuMat output_gpu_mat(red_frame.size(), red_frame.type());

  gaussian_smoothing<<<grid_size, block_size>>>(
      input_gpu_mat.ptr<uint8_t>(), output_gpu_mat.ptr<uint8_t>(), 5, 6.0f);
  binarise<<<grid_size, block_size>>>(output_gpu_mat.ptr<uint8_t>(), threshold);
  closing(output_gpu_mat.ptr<uint8_t>(), input_gpu_mat.ptr<uint8_t>());
  opening(input_gpu_mat.ptr<uint8_t>(), output_gpu_mat.ptr<uint8_t>());

  output_gpu_mat.download(red_frame);
  std::vector<blob> blobs;
  get_blobs(red_frame, blobs);

  cv::imshow("mosquitoes pre-processed", red_frame);
  cv::waitKey(1);
  return blobs;
}

__global__ void subtract_background(uint8_t* device_frame) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < d_WIDTH && y < d_HEIGHT) {
    background[y * d_WIDTH + x] =
        learning_rate * device_frame[y * d_WIDTH + x] +
        (1 - learning_rate) * background[y * d_WIDTH + x];
    device_frame[y * d_WIDTH + x] =
        device_frame[y * d_WIDTH + x] - background[y * d_WIDTH + x];
  }
}

class Subtractor {
 private:
  __constant__ cv::cuda::GpuMat background;
  __constant__ float learning_rate;

 public:
  Subtractor(cv::Mat backgroud, float learning_rate)
      : learning_rate(learning_rate) {
    this->background.upload(backgroud);
  }

  std::vector<blob> detect_mosquitoes(cv::Mat red_frame) {
    cv::cuda::GpuMat input_gpu_mat;
    input_gpu_mat.upload(red_frame);
    cv::cuda::GpuMat output_gpu_mat(red_frame.size(), red_frame.type());

    gaussian_smoothing<<<grid_size, block_size>>>(
        input_gpu_mat.ptr<uint8_t>(), output_gpu_mat.ptr<uint8_t>(), 5, 6.0f);
    // binarise<<<grid_size, block_size>>>(output_gpu_mat.ptr<uint8_t>(),
    // threshold);
    subtract_background<<<grid_size, block_size>>>(
        output_gpu_mat.ptr<uint8_t>());
    closing(output_gpu_mat.ptr<uint8_t>(), input_gpu_mat.ptr<uint8_t>());
    opening(input_gpu_mat.ptr<uint8_t>(), output_gpu_mat.ptr<uint8_t>());

    output_gpu_mat.download(red_frame);
    std::vector<blob> blobs;
    get_blobs(red_frame, blobs);

    cv::imshow("mosquitoes pre-processed", red_frame);
    cv::waitKey(1);
    return blobs;
  }
}

}  // namespace gpu
