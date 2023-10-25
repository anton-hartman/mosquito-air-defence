#pragma once

#include <cmath>
#include <opencv2/core.hpp>
#include <string>

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

  bool operator!=(const Pt& other) const {
    return (x != other.x) or (y != other.y);
  }

  /**
   * @brief Compares the x coordinate of the point with the given value.
   */
  bool operator==(const int x) const { return (this->x == x); }

  cv::Point cv_pt() const { return cv::Point(x, y); }

  cv::Point cv_pt(const int offset) const {
    return cv::Point(x + offset, y + offset);
  }

  std::string to_string() const {
    return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
  }

  bool in_radius(const Pt& other, const int radius) const {
    auto euclidean_dist = [](Pt pt1, Pt pt2) {
      return std::sqrt(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2));
    };

    return euclidean_dist(*this, other) <= radius;
  }
};
