#include "../include/two_pass_algorithm.hpp"

#include <algorithm>
#include <chrono>
#include <climits>
#include <iostream>
#include <map>
#include <thread>
#include <vector>
#include "../include/frame.hpp"

using namespace std;

// Function to perform the Two-Pass Algorithm
BoundingBoxMap find_connected_components(const uint8_t* frame) {
  vector<vector<int>> label_image(HEIGHT, vector<int>(WIDTH, 0));
  int next_label = 1;
  map<int, int> equivalences;
  BoundingBoxMap bounding_boxes;

  // First Pass
  for (int y = 0; y < HEIGHT; ++y) {
    for (int x = 0; x < WIDTH; ++x) {
      if (frame[y * WIDTH + x] == 1) {  // Foreground pixel
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
        } else {
          int min_neighbor = *min_element(neighbors.begin(), neighbors.end());
          label_image[y][x] = min_neighbor;

          pair<Point, Point>& bounding_box = bounding_boxes[min_neighbor];
          Point& top_left = bounding_box.first;
          Point& bottom_right = bounding_box.second;

          top_left = {min(top_left.first, y), min(top_left.second, x)};
          bottom_right = {max(bottom_right.first, y),
                          max(bottom_right.second, x)};

          for (const int& neighbor : neighbors) {
            int root = neighbor;
            while (equivalences[root] != root) {
              root = equivalences[root];
            }
            equivalences[root] = min_neighbor;
          }
        }
      }
    }
  }

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
}

std::pair<uint16_t, uint16_t> find_laser_pos(const uint8_t* frame) {
  BoundingBoxMap bounding_boxes = find_connected_components(frame);

  int c_x = 0;
  int c_y = 0;
  for (const pair<int, Rectangle>& element : bounding_boxes) {
    int label = element.first;
    const Rectangle& rectangle = element.second;
    const Point& min_point = rectangle.first;
    const Point& max_point = rectangle.second;
    cout << "Label: " << label << " Bounding Rectangle: {{" << min_point.first
         << ", " << min_point.second << "}, {" << max_point.first << ", "
         << max_point.second << "}}\n";
    c_x += (min_point.second + max_point.second) / 2;
    c_y += (min_point.first + max_point.first) / 2;
    return {c_x, c_y};
  }
}

// int main() {
//   const int WIDTH = 5;
//   const int HEIGHT = 4;

//   uint8_t frame[WIDTH * HEIGHT] = {
//       0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0,
//   };

//   BoundingBoxMap bounding_boxes =
//       find_connected_components(frame, WIDTH, HEIGHT);

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
