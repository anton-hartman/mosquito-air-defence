
#pragma once

#include <opencv2/opencv.hpp>
#include "frame.hpp"

// Function declarations from kalman.hpp without Eigen dependencies
class Kalman {
 public:
  Kalman(double dt_,
         double u_x_,
         double u_y_,
         double sigma_a,
         double sigma_z_x,
         double sigma_z_y);
  void predict();
  void update(Pt pt);
  // Other Eigen-independent member functions if needed
};
