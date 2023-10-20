#pragma once

#include <cmath>
#include <opencv2/core.hpp>

class Pt {
 public:
  int x, y;
  // int id;

  Pt() {
    x = -1;
    y = -1;
  }

  // Pt(int id) {
  //   x = -1;
  //   y = -1;
  //   this->id = id;
  // }

  Pt(int x, int y) {
    this->x = x;
    this->y = y;
  }

  // Pt(int x, int y, int id) {
  //   this->x = x;
  //   this->y = y;
  //   this->id = id;
  // }

  Pt(double x, double y) {
    this->x = static_cast<int>(std::round(x));
    this->y = static_cast<int>(std::round(y));
  }

  // Pt(double x, double y, int id) {
  //   this->x = static_cast<int>(std::round(x));
  //   this->y = static_cast<int>(std::round(y));
  //   this->id = id;
  // }

  Pt(const Pt& other) {
    x = other.x;
    y = other.y;
  }

  // Pt(const Pt& other) {
  //   x = other.x;
  //   y = other.y;
  //   id = other.id;
  // }

  // Pt& operator=(const Pt& other) {
  //   x = other.x;
  //   y = other.y;
  //   id = other.id;
  //   return *this;
  // }

  Pt& operator=(const Pt& other) {
    this->x = other.x;
    this->y = other.y;
    return *this;
  }

  bool operator==(const Pt& other) const {
    return (x == other.x) and (y == other.y);
  }

  bool operator==(const int x) const { return (this->x == x); }

  cv::Point cv_pt() const { return cv::Point(x, y); }
};
