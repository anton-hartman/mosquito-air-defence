#pragma once

#include <vector>
#include "pt.hpp"

using Matrix = std::vector<std::vector<double>>;

class Track {
 public:
  int id;
  int age = 0;
  Pt pt;
  Pt detected_pt;

  int max_hist = 20;
  std::deque<Pt> pt_hist;
  std::deque<Pt> detected_pt_hist;

  Track(int id, Pt pt) : id(id), pt(pt) {}
  Track() : id(-1), pt({-1, -1}), detected_pt({-1, -1}) {}

  Track& operator=(const Track& other) {
    if (this != &other) {
      this->id = other.id;
      this->age = other.age;
      this->pt = other.pt;
      this->detected_pt = other.detected_pt;
    }
    return *this;
  }

  bool operator==(const int id) const { return this->id == id; }
};

class Kalman {
 private:
  static int id_counter;

  // State vector [x, y, vx, vy]
  std::vector<double> x;  // = {0, 0, 0, 0};
  // Control input [ux, uy] = [ax, ay]
  std::vector<double> u;  // = {0, 0};

  double dt, std_acc, std_meas_x, std_meas_y;
  Matrix A, B, H, Q, R, P;

 public:
  Track track;

  Kalman(Pt detected_pt,
         double dt_,
         double a_x_,
         double a_y_,
         double sigma_a,
         double sigma_z_x,
         double sigma_z_y);

  Track predict();
  Track update(Pt pt);
};
