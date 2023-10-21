#pragma once

#include <deque>
#include <vector>
#include "frame.hpp"
#include "pt.hpp"

using Matrix = std::vector<std::vector<double>>;

class Track {
 public:
  const int max_hist = 50;
  int id;
  int age;
  Pt predicted_pt;
  Pt updated_pt;
  Pt detected_pt;

  std::deque<Pt> predicted_pt_hist;
  std::deque<Pt> updated_pt_hist;
  std::deque<Pt> detected_pt_hist;

  Track(int id, Pt detected_pt)
      : id(id),
        age(0),
        detected_pt(detected_pt),
        predicted_pt(detected_pt),
        updated_pt(detected_pt) {}

  Track()
      : id(-2),
        age(-2),
        predicted_pt({ROWS - 50, COLS - 50}),
        updated_pt({ROWS - 50, COLS - 50}),
        detected_pt({ROWS - 50, COLS - 50}) {}

  Track& operator=(const Track& other) {
    if (this != &other) {
      this->id = other.id;
      this->age = other.age;
      this->predicted_pt = other.predicted_pt;
      this->updated_pt = other.updated_pt;
      this->detected_pt = other.detected_pt;
      this->predicted_pt_hist = other.predicted_pt_hist;
      this->updated_pt_hist = other.updated_pt_hist;
      this->detected_pt_hist = other.detected_pt_hist;
    }
    return *this;
  }

  Track(const Track& other) {
    this->id = other.id;
    this->age = other.age;
    this->predicted_pt = other.predicted_pt;
    this->updated_pt = other.updated_pt;
    this->detected_pt = other.detected_pt;
    this->predicted_pt_hist = other.predicted_pt_hist;
    this->updated_pt_hist = other.updated_pt_hist;
    this->detected_pt_hist = other.detected_pt_hist;
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
