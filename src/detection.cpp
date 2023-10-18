#include "../include/detection.hpp"

namespace detection {

Pt ignore_region_top_left = Pt{525, 268};
Pt ignore_region_bottom_right = Pt{578, 305};

void set_ignore_region(Pt top_left, Pt bottom_right) {
  ignore_region_top_left = top_left;
  ignore_region_bottom_right = bottom_right;
}

void set_struct_elem(int size_elem_size, StructElemType type) {
  int diameter = 2 * struct_elem_size + 1;
  uint8_t* host_struct_elem = new uint8_t[diameter * diameter];

  for (int i = 0; i < diameter; i++) {
    for (int j = 0; j < diameter; j++) {
      int y = i - struct_elem_size;  // y-coordinate relative to the center
      int x = j - struct_elem_size;  // x-coordinate relative to the center
      host_struct_elem[i * diameter + j] =
          (x * x + y * y <= struct_elem_size * struct_elem_size) ? 1 : 0;
    }
  }

  int elem_mem_size = diameter * diameter * sizeof(uint8_t);
  switch (type) {
    case StructElemType::MOS_OPENING:
      cudaMemcpy(gpu::d_mos_opening_struct_elem, host_struct_elem,
                 elem_mem_size, cudaMemcpyHostToDevice);
      break;
    case StructElemType::MOS_CLOSING:
      cudaMemcpy(gpu::d_mos_closing_struct_elem, host_struct_elem,
                 elem_mem_size, cudaMemcpyHostToDevice);
      break;
    case StructElemType::LASER_OPENING:
      cudaMemcpy(gpu::d_laser_opening_struct_elem, host_struct_elem,
                 elem_mem_size, cudaMemcpyHostToDevice);
      break;
    case StructElemType::LASER_CLOSING:
      cudaMemcpy(gpu::d_laser_closing_struct_elem, host_struct_elem,
                 elem_mem_size, cudaMemcpyHostToDevice);
      break;
  }
}

void set_background(const cv::Mat& frame) {
  cudaMemcpy(gpu::d_background, frame.ptr(), gpu::frame_size,
             cudaMemcpyHostToDevice);
}

bool is_pt_in_ignore_region(const Pt& pt,
                            const Pt& ignore_region_top_left,
                            const Pt& ignore_region_bottom_right) {
  return (pt.x >= ignore_region_top_left.x and
          pt.x <= ignore_region_bottom_right.x and
          pt.y >= ignore_region_top_left.y and
          pt.y <= ignore_region_bottom_right.y);
}

/**
 * @brief Assumes x-axis of camera and turret origin are aligned.
 * Probably wrong and should use the same method as distinguish_with_y
 */
Pt distinguish_with_x(const std::vector<Pt>& two_pts) {
  int x0 = two_pts.at(0).x;
  int x1 = two_pts.at(1).x;

  if (std::abs(x0 - mads::C_X) > std::abs(x1 - mads::C_X)) {
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

  if (std::abs(two_pts.at(0).x - two_pts.at(1).x) >=
      std::abs(two_pts.at(0).y - two_pts.at(1).y)) {
    return distinguish_with_x(two_pts);
  } else {
    return distinguish_with_y(two_pts);
  }
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

std::vector<Pt> detect_lasers(cv::Mat red_frame, uint8_t threshold) {
  // cudaError_t err = cudaMemcpy(d_frame_1, red_frame.ptr(), frame_size,
  //  cudaMemcpyHostToDevice);
  cudaError_t err = cudaMemcpy(d_frame_2, red_frame.ptr(), frame_size,
                               cudaMemcpyHostToDevice);
  (err != cudaSuccess) ? printf("CUDA err: %s\n", cudaGetErrorString(err)) : 0;

  // gpu::gaussian_smoothing<<<grid_size, block_size>>>(d_frame_1, d_frame_2,
  // 5, 6.0f);
  gpu::binarise_gt<<<grid_size, block_size>>>(d_frame_2, threshold);
  gpu::dilation<<<grid_size, block_size>>>(d_frame_2, d_frame_1);
  gpu::erosion<<<grid_size, block_size>>>(d_frame_1, d_frame_2);
  gpu::erosion<<<grid_size, block_size>>>(d_frame_2, d_frame_1);
  gpu::dilation<<<grid_size, block_size>>>(d_frame_1, d_frame_2);

  err = cudaMemcpy(red_frame.ptr(), d_frame_2, frame_size,
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
    cv::imshow("laser detection", red_frame);
    cv::waitKey(1);
  }
  return laser_pts;
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
  // 5,
  //                                               6.0f);
  // if (mads::display.load() & Display::MOSQUITO_DETECTION and
  //     mads::debug == Debug::DEEP) {
  //   cv::Mat temp(ROWS, COLS, CV_8UC1);
  //   cudaMemcpy(temp.ptr(), mos_d_frame_2, frame_size,
  //   cudaMemcpyDeviceToHost); cv::imshow("smoothed", temp); cv::waitKey(1);
  // }
  if (bg_sub) {
    subtract_and_update_background<<<grid_size, block_size>>>(
        mos_d_frame_2, d_background, bg_learning_rate);
    if (mads::display.load() & Display::MOSQUITO_DETECTION and
        mads::debug == Debug::ON) {
      cv::Mat temp(ROWS, COLS, CV_8UC1);
      cudaMemcpy(temp.ptr(), d_background, frame_size, cudaMemcpyDeviceToHost);
      cv::imshow("background", temp);
      cv::waitKey(1);

      cudaMemcpy(temp.ptr(), mos_d_frame_2, frame_size, cudaMemcpyDeviceToHost);
      cv::imshow("subtracted", temp);
      cv::waitKey(1);
    }

    binarise_gt<<<grid_size, block_size>>>(mos_d_frame_2, threshold);
  } else {
    binarise_lt<<<grid_size, block_size>>>(mos_d_frame_2, threshold);
  }
  // closing == dilation followed by erosion
  closing(mos_d_frame_2, mos_d_frame_1);
  if (mads::display.load() & Display::MOSQUITO_DETECTION and
      mads::debug == Debug::DEEP) {
    cv::Mat temp(ROWS, COLS, CV_8UC1);
    cudaMemcpy(temp.ptr(), mos_d_frame_2, frame_size, cudaMemcpyDeviceToHost);
    cv::imshow("mos closed", temp);
    cv::waitKey(1);
  }

  // opening == erosion followed by dilation
  opening(mos_d_frame_2, mos_d_frame_1);
  if (mads::display.load() & Display::MOSQUITO_DETECTION and
      mads::debug == Debug::DEEP) {
    cv::Mat temp(ROWS, COLS, CV_8UC1);
    cudaMemcpy(temp.ptr(), mos_d_frame_2, frame_size, cudaMemcpyDeviceToHost);
    cv::imshow("mos opened", temp);
    cv::waitKey(1);
  }

  // dilation<<<grid_size, block_size>>>(mos_d_frame_2, mos_d_frame_1);
  // erosion<<<grid_size, block_size>>>(mos_d_frame_1, mos_d_frame_2);
  // erosion<<<grid_size, block_size>>>(mos_d_frame_2, mos_d_frame_1);
  // dilation<<<grid_size, block_size>>>(mos_d_frame_1, mos_d_frame_2);

  err = cudaMemcpy(red_frame.ptr(), mos_d_frame_2, frame_size,
                   cudaMemcpyDeviceToHost);
  (err != cudaSuccess) ? printf("CUDA err: %s\n", cudaGetErrorString(err)) : 0;

  std::vector<Pt> mos_pts;
  int num_mos_pts = -2;
  num_mos_pts = get_blobs(red_frame, mos_pts);

  if (mads::display.load() & Display::MOSQUITO_DETECTION) {
    for (const Pt& mos_pt : mos_pts) {
      cv::circle(red_frame, mos_pt.cv_pt(), 20, cv::Scalar(150, 255, 255), 2);
      cv::putText(red_frame, std::to_string(i),
                  cv::Point(mos_pt.x + 10, mos_pts.y + 10),
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

}  // namespace detection