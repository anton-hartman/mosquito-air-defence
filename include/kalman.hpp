/* Discrete Kalman filter implementation for object tracking
 * Reference:
 * https://machinelearningspace.com/2d-object-tracking-using-kalman-filter/
 */
#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <opencv2/core.hpp>

class Kalman {
 private:
  double dt;          // Time for one cycle
  Eigen::Vector2d u;  // Control input
  Eigen::Vector4d x;  // Initial State (position and velocity)

  // Standard deviations of acceleration(sigma_a) and measurement(sigma_z)
  double std_acc = 0, std_meas_x = 0, std_meas_y = 0;

  // State estimation matrix
  Eigen::Matrix4d A;

  // Control input matrix
  Eigen::Matrix<double, 4, 2> B;

  // State-to-measurement domain transformation matrix
  Eigen::Matrix<double, 2, 4> H;

  // Process Noise Covariance
  double c1, c2, c3;
  double sig_a_2;
  Eigen::Matrix4d Q;

  // Measurement Noise Covariance
  Eigen::Matrix2d R;

  Eigen::Matrix4d P = Eigen::Matrix4d::Identity();  // Error Covariance

 public:
  Kalman(double dt_,
         double u_x_,
         double u_y_,
         double sigma_a,
         double sigma_z_x,
         double sigma_z_y);

  // Time update equations:
  Eigen::Vector2d predict(void);

  // Measurement update equations:
  Eigen::Vector2d update(cv::Point pt);

  // Getters:
  Eigen::Matrix4d get_A();

  Eigen::Matrix<double, 4, 2> get_B();
  Eigen::Matrix<double, 2, 4> get_H();

  Eigen::Matrix4d get_Q();

  Eigen::Matrix2d get_R();
};