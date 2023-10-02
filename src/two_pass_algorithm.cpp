#include "../include/two_pass_algorithm.hpp"

#include <algorithm>
#include <chrono>
#include <climits>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <thread>
#include <vector>
#include "../include/frame.hpp"

using namespace std;

// const int WIDTH = 300;
// const int HEIGHT = 300;

// Function to perform the Two-Pass Algorithm
BoundingBoxMap find_connected_components(const uint8_t* frame) {
  vector<vector<int>> label_image(HEIGHT, vector<int>(WIDTH, 0));
  int next_label = 1;
  map<int, int> equivalences;
  BoundingBoxMap bounding_boxes;

  // First Pass
  for (int y = 0; y < HEIGHT; ++y) {
    for (int x = 0; x < WIDTH; ++x) {
      if (frame[y * WIDTH + x] == 255) {  // Foreground pixel
        vector<int> neighbors;
        if (x > 0) {
          neighbors.push_back(label_image[y][x - 1]);  // Left
        }
        if (y > 0) {
          neighbors.push_back(label_image[y - 1][x]);  // Above
        }
        if (x > 0 && y > 0) {
          neighbors.push_back(label_image[y - 1][x - 1]);  // Above Left
        }
        if (x < WIDTH - 1 && y > 0) {
          neighbors.push_back(label_image[y - 1][x + 1]);  // Above Right
        }

        neighbors.erase(remove(neighbors.begin(), neighbors.end(), 0),
                        neighbors.end());  // Remove zeros

        if (neighbors.empty()) {
          label_image[y][x] = next_label;
          equivalences[next_label] = next_label;
          bounding_boxes[next_label] = {{y, x}, {y, x}};
          ++next_label;
          // std::cout << "empty neighbors" << std::endl;
        } else {
          int min_neighbor = *min_element(neighbors.begin(), neighbors.end());
          label_image[y][x] = min_neighbor;

          pair<Point, Point>& bounding_box = bounding_boxes[min_neighbor];
          Point& top_left = bounding_box.first;
          Point& bottom_right = bounding_box.second;

          top_left = {min(top_left.first, y), min(top_left.second, x)};
          bottom_right = {max(bottom_right.first, y),
                          max(bottom_right.second, x)};

          // std::cout << "not empty 1" << std::endl;
          for (const int& neighbor : neighbors) {
            int root = neighbor;
            // Initialize if not already present
            if (equivalences.find(root) == equivalences.end()) {
              equivalences[root] = root;
            }

            std::cout << "Equivalences" << std::endl;
            for (const pair<int, int>& element : equivalences) {
              int label = element.first;
              int root = element.second;
              cout << "Label: " << label << " Root: " << root << "\n";
            }
            // Resolve to root label
            int count = 0;
            while (equivalences[root] != root and count++ < 15) {
              if (equivalences.find(root) ==
                  equivalences.end()) {  // Additional safety check
                equivalences[root] = root;
              }
              root = equivalences[root];
              std::cout << "Count = " << count << std::endl;
            }
            equivalences[root] = min_neighbor;
          }
          // std::cout << "not empty neighbors" << std::endl;
        }
      }
    }
  }

  // std::cout << "first pass done" << std::endl;
  // Second Pass
  BoundingBoxMap final_bounding_boxes;
  for (int y = 0; y < HEIGHT; ++y) {
    for (int x = 0; x < WIDTH; ++x) {
      if (label_image[y][x] > 0) {
        int root_label = label_image[y][x];
        while (equivalences[root_label] != root_label) {
          root_label = equivalences[root_label];
        }

        if (final_bounding_boxes.find(root_label) ==
            final_bounding_boxes.end()) {
          final_bounding_boxes[root_label] = {{y, x}, {y, x}};
        } else {
          pair<Point, Point>& bounding_box = final_bounding_boxes[root_label];
          Point& top_left = bounding_box.first;
          Point& bottom_right = bounding_box.second;
          top_left = {min(top_left.first, y), min(top_left.second, x)};
          bottom_right = {max(bottom_right.first, y),
                          max(bottom_right.second, x)};
        }
      }
    }
  }

  return final_bounding_boxes;
  // return {};
}

std::vector<std::pair<uint16_t, uint16_t>> find_blobs(const uint8_t* frame) {
  std::chrono::high_resolution_clock::time_point start_time =
      std::chrono::high_resolution_clock::now();

  BoundingBoxMap bounding_boxes = find_connected_components(frame);
  // std::cout << "components connectd" << std::endl;

  int c_x = 0;
  int c_y = 0;
  std::vector<std::pair<uint16_t, uint16_t>> blobs;
  for (const pair<int, Rectangle>& element : bounding_boxes) {
    int label = element.first;
    const Rectangle& rectangle = element.second;
    const Point& min_point = rectangle.first;
    const Point& max_point = rectangle.second;
    cout << "Label: " << label << " Bounding Rectangle: {{" << min_point.first
         << ", " << min_point.second << "}, {" << max_point.first << ", "
         << max_point.second << "}}\n";
    c_x = (min_point.second + max_point.second) / 2;
    c_y = (min_point.first + max_point.first) / 2;
    blobs.push_back({c_x, c_y});
  }

  std::chrono::high_resolution_clock::time_point end_time =
      std::chrono::high_resolution_clock::now();
  uint32_t duration = std::chrono::duration_cast<std::chrono::microseconds>(
                          end_time - start_time)
                          .count();
  std::cout << "Connected components processing time = " << duration << " us"
            << std::endl;

  return blobs;
}

// uint8_t* readFrameFromTextFile(const std::string& filename, size_t size) {
//   std::ifstream infile(filename);
//   if (!infile) {
//     std::cerr << "Could not open file for reading: " << filename <<
//     std::endl; return nullptr;
//   }

//   std::string line;
//   std::getline(infile, line);

//   // Remove the opening and closing braces
//   line = line.substr(1, line.length() - 2);

//   // Use a stringstream to split the string by commas
//   std::stringstream ss(line);
//   std::string token;
//   std::vector<uint8_t> values;
//   while (std::getline(ss, token, ',')) {
//     values.push_back(static_cast<uint8_t>(std::stoi(token)));
//   }

//   // Create a new uint8_t array and copy the values
//   size = values.size();
//   uint8_t* frame = new uint8_t[size];
//   for (size_t i = 0; i < size; ++i) {
//     frame[i] = values[i];
//   }

//   return frame;
// }

// int main() {
//   // uint8_t frame[WIDTH * HEIGHT] = {
//   //     0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0,
//   // };

//   BoundingBoxMap bounding_boxes = find_connected_components(
//       readFrameFromTextFile("gpu_frame.txt", WIDTH * HEIGHT));
//   // BoundingBoxMap bounding_boxes = find_connected_components(frame);

//   // Output the bounding rectangles
//   for (const pair<int, Rectangle>& element : bounding_boxes) {
//     int label = element.first;
//     const Rectangle& rectangle = element.second;
//     const Point& min_point = rectangle.first;
//     const Point& max_point = rectangle.second;
//     cout << "Label: " << label << " Bounding Rectangle: {{" <<
//     min_point.first
//          << ", " << min_point.second << "}, {" << max_point.first << ", "
//          << max_point.second << "}}\n";
//   }

//   return 0;
// }