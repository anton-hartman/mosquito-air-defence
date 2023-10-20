#pragma once

#include <vector>
#include "pt.hpp"

using Matrix = std::vector<std::vector<double>>;

class Kalman {
 private:
  static int id_counter;
  int id;
  Pt detected_pt;

  std::vector<double> x = {0, 0, 0, 0};  // State vector [x, y, vx, vy]
  std::vector<double> u = {0, 0};        // Control input [ux, uy]

  double dt, std_acc, std_meas_x, std_meas_y;
  Matrix A, B, H, Q, R, P;

 public:
  Kalman(double dt_,
         double u_x_,
         double u_y_,
         double sigma_a,
         double sigma_z_x,
         double sigma_z_y);

  int age = 0;
  std::vector<double> predict();
  std::vector<double> update(Pt pt);
  //   Pt predict_without_updating_states() const;

  // Getters
  int get_id() const { return id; };
  Pt_d pt_d() const { return Pt_d{x[0], x[1]}; };
};
