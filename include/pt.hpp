#pragma once

#include <cmath>
#include <opencv2/core.hpp>

class Pt {
 public:
  int x, y;

  Pt() {
    x = -1;
    y = -1;
  }

  Pt(int x, int y) {
    this->x = x;
    this->y = y;
  }

  Pt(double x, double y) {
    this->x = static_cast<int>(std::round(x));
    this->y = static_cast<int>(std::round(y));
  }

  Pt(const Pt& other) {
    x = other.x;
    y = other.y;
  }

  Pt& operator=(const Pt& other) {
    this->x = other.x;
    this->y = other.y;
    return *this;
  }

  bool operator==(const Pt& other) const {
    return (x == other.x) and (y == other.y);
  }

  /**
   * @brief Compares the x coordinate of the point with the given value.
   */
  bool operator==(const int x) const { return (this->x == x); }

  cv::Point cv_pt() const { return cv::Point(x, y); }
};
