#pragma once

#include <cmath>
#include <opencv2/core.hpp>

struct Pt_d {
  double x, y;
  int id = -1;

  Pt_d() {
    x = -1;
    y = -1;
  };

  Pt_d(double x, double y) {
    this->x = x;
    this->y = y;
  };

  Pt_d(double x, double y, int id) {
    this->x = x;
    this->y = y;
    this->id = id;
  };

  Pt_d(const Pt_d& other) {
    x = other.x;
    y = other.y;
    id = other.id;
  };

  Pt_d(const Pt_d& other, const int& id) {
    x = other.x;
    y = other.y;
    this->id = id;
  };

  Pt_d& operator=(const Pt_d& other) {
    x = other.x;
    y = other.y;
    id = other.id;
    return *this;
  };

  bool operator==(const Pt_d& other) const {
    return (x == other.x) && (y == other.y) && (id == other.id);
  };
};

class Pt {
 private:
  static int id_counter;

 public:
  int x, y;
  int id = -1;

  Pt() {
    x = -1;
    y = -1;
  };

  Pt(int x, int y) {
    this->x = x;
    this->y = y;
  };

  Pt(int x, int y, int id) {
    this->x = x;
    this->y = y;
    this->id = id;
  };

  Pt(const Pt& other) {
    x = other.x;
    y = other.y;
    id = other.id;
  };

  Pt& operator=(const Pt& other) {
    x = other.x;
    y = other.y;
    id = other.id;
    return *this;
  };

  bool operator==(const Pt& other) const {
    return (x == other.x) && (y == other.y) && (id == other.id);
  };

  Pt(const Pt_d& other) {
    x = static_cast<int>(std::round(other.x));
    y = static_cast<int>(std::round(other.y));
  };

  cv::Point cv_pt() const { return cv::Point(x, y); };
};
