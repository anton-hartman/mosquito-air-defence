/* Discrete Kalman filter implementation for object tracking
 * Reference:
 * https://machinelearningspace.com/2d-object-tracking-using-kalman-filter/
 */
#pragma once

#include <cmath>
// #include <iostream>
#include <math.h>
#include <opencv2/core.hpp>

#include <Eigen/Dense>

class KalmanFilter2D {
 private:
  double dt = 0;                  // Time for one cycle
  Eigen::Vector2d u{0, 0};        // Control input
  Eigen::Vector4d x{0, 0, 0, 0};  // Initial State (position and velocity)

  // Standard deviations of acceleration(sigma_a) and measurement(sigma_z)
  double std_acc = 0, std_meas_x = 0, std_meas_y = 0;

  // State estimation matrix
  Eigen::Matrix4d A{{1, 0, dt, 0}, {0, 1, 0, dt}, {0, 0, 1, 0}, {0, 0, 0, 1}};

  // Control input matrix
  Eigen::Matrix<double, 4, 2> B{{(dt * dt) / 2, 0},
                                {0, (dt * dt) / 2},
                                {dt, 0},
                                {0, dt}};

  // State-to-measurement domain transformation matrix
  Eigen::Matrix<double, 2, 4> H{{1, 0, 0, 0}, {0, 1, 0, 0}};

  // Process Noise Covariance
  double c1 = (pow(dt, 4) / 4), c2 = (pow(dt, 3) / 2), c3 = (dt * dt);
  double sig_a_2 = pow(std_acc, 2);
  Eigen::Matrix4d Q{{sig_a_2 * c1, 0, sig_a_2* c2, 0},
                    {0, sig_a_2* c1, 0, sig_a_2* c2},
                    {sig_a_2 * c2, 0, sig_a_2* c3, 0},
                    {0, sig_a_2* c2, 0, sig_a_2* c3}};

  // Measurement Noise Covariance
  Eigen::Matrix2d R{{std_meas_x * std_meas_x, 0}, {0, std_meas_y* std_meas_y}};

  Eigen::Matrix4d P = Eigen::Matrix4d::Identity();  // Error Covariance

 public:
  KalmanFilter2D(double dt_,
                 double u_x_,
                 double u_y_,
                 double sigma_a,
                 double sigma_z_x,
                 double sigma_z_y)
      : dt(dt_),
        std_acc(sigma_a),
        std_meas_x(sigma_z_x),
        std_meas_y(sigma_z_y) {
    u[0] = u_x_;
    u[1] = u_y_;
  };

  // Time update equations:
  Eigen::Vector2d predict() {
    // Update time state:
    x = A * x + B * u;

    // Calculate error covariance:
    // P = (A * P * A') + Q
    P = A * P * A.transpose() + Q;

    return x.head(2);  // (Eigen::Vector2d)x(Eigen::seq(0, 2));
  }

  // Measurement update equations:
  Eigen::Vector2d update(cv::Point pt) {
    // Calculating Kalman gain (K):
    Eigen::Matrix2d S = (H * P * H.transpose()) + R;
    Eigen::Matrix<double, 4, 2> K = (P * H.transpose()) * S.inverse();

    Eigen::Vector2d z0{pt.x, pt.y};
    x += K * (z0 - (H * x));
    x(0) = round(x(0));
    x(1) = round(x(1));

    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();

    P = (I - (K * H)) * P;

    return x.head(2);  // (Eigen::Vector2d)x(Eigen::seq(0, 2));
  }

  // Getters:
  Eigen::Matrix4d get_A() { return A; }

  Eigen::Matrix<double, 4, 2> get_B() { return B; }

  Eigen::Matrix<double, 2, 4> get_H() { return H; }

  Eigen::Matrix4d get_Q() { return Q; }

  Eigen::Matrix2d get_R() { return R; }
};