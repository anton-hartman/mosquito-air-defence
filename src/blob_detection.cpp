#include <iostream>
#include <opencv2/opencv.hpp>
#include <queue>
#include <string>
// #include "../include/frame.hpp"

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

int WIDTH;
int HEIGHT;

void get_blobs(uint8_t* img, std::vector<blob>& blobs) {
  // int i, j, k, l, r = img.rows, c = img.cols, id = 1;
  int i, j, k, l, r = HEIGHT, c = WIDTH, id = 1;
  std::vector<std::vector<int> > pixel_ID(r, std::vector<int>(c, -1));
  // Stores ID of a pixel; -1 means unvisited
  std::queue<pt> open_list;
  // Breadth-First-Search hence queue of points
  for (i = 1; i < r - 1; i++) {
    for (j = 1; j < c - 1; j++) {
      //   if (img.at<uchar>(i, j) == 0 || pixel_ID[i][j] > -1)
      if (img[i * WIDTH + j] == 0 || pixel_ID[i][j] > -1) {
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
            // if (img.at<uchar>(k, l) == 0 || pixel_ID[k][l] > -1)
            if (img[k * WIDTH + l] == 0 || pixel_ID[k][l] > -1) {
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

void convert_to_red_frame(const cv::Mat& frame, uint8_t* red_frame) {
  // Check if the input Mat is of the expected size and type
  if (frame.rows != HEIGHT || frame.cols != WIDTH) {
    std::cerr << "Invalid input Mat dimensions!" << std::endl;
    throw std::runtime_error("Invalid input Mat");
  }
  if (frame.type() != CV_8UC3) {
    std::cerr << "Invalid input Mat type!" << std::endl;
  }

  for (int i = 0; i < HEIGHT; ++i) {
    for (int j = 0; j < WIDTH; ++j) {
      cv::Vec3b pixel = frame.at<cv::Vec3b>(i, j);
      // Extracting the red channel (assuming BGR format)
      red_frame[i * WIDTH + j] = pixel[2];
    }
  }
}

int main() {
  std::string fname;
  std::cout << "Enter name of file :";
  std::cin >> fname;
  cv::Mat image = cv::imread(fname, 0);
  cv::Mat output_image;
  WIDTH = image.cols;
  HEIGHT = image.rows;
  // Convert to CV_8UC3
  cv::cvtColor(image, output_image, cv::COLOR_GRAY2BGR);
  uint8_t* red_frame = new uint8_t[HEIGHT * WIDTH];
  convert_to_red_frame(output_image, red_frame);
  std::vector<blob> blobs;
  get_blobs(red_frame, blobs);
  return 0;
}