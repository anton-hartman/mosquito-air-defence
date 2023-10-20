#include "../include/kalman.hpp"

int Kalman::id_counter = 0;

Kalman::Kalman(double dt_,
               double u_x_,
               double u_y_,
               double sigma_a,
               double sigma_z_x,
               double sigma_z_y)
    : dt(dt_),
      std_acc(sigma_a),
      std_meas_x(sigma_z_x),
      std_meas_y(sigma_z_y),
      id(id_counter++) {
  u[0] = u_x_;
  u[1] = u_y_;

  // clang-format off

  // Initial state:
  x << 0, 0, 0, 0;

  // State transition matrix:
  A << 1, 0, dt,  0,
       0, 1,  0, dt, 
       0, 0,  1,  0, 
       0, 0,  0,  1;

  // Control input matrix:
  B << (dt * dt) / 2,             0,
                   0, (dt * dt) / 2,
                  dt,             0,
                   0,            dt;

  // Measurement matrix:
  H << 1, 0, 0, 0,
       0, 1, 0, 0;

  // Process noise covariance:
  c1 = (dt * dt * dt * dt) / 4;
  c2 = (dt * dt * dt) / 2;
  c3 = (dt * dt);
  sig_a_2 = std_acc * std_acc;

  Q << c1 * sig_a_2,            0, c2 * sig_a_2,            0,
                  0, c1 * sig_a_2,            0, c2 * sig_a_2,
       c2 * sig_a_2,            0, c3 * sig_a_2,            0,
                  0, c2 * sig_a_2,            0, c3 * sig_a_2;

  // Measurement noise covariance:
  R << std_meas_x * std_meas_x,                       0,
                             0, std_meas_y * std_meas_y;

  // clang-format on
};

// Time update equations:
Eigen::Vector2d Kalman::predict(void) {
  // Update time state:
  x = A * x + B * u;

  // Calculate error covariance:
  // P = (A * P * A') + Q
  P = A * P * A.transpose() + Q;

  predicted_centre = eigen_to_pt_d(x.head(2));
  return x.head(2);  // (Eigen::Vector2d)x(Eigen::seq(0, 2));
}

// Measurement update equations:
Eigen::Vector2d Kalman::update(Pt pt) {
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
Eigen::Matrix4d Kalman::get_A() {
  return A;
}

Eigen::Matrix<double, 4, 2> Kalman::get_B() {
  return B;
}

Eigen::Matrix<double, 2, 4> Kalman::get_H() {
  return H;
}

Eigen::Matrix4d Kalman::get_Q() {
  return Q;
}

Eigen::Matrix2d Kalman::get_R() {
  return R;
}

Pt_d Kalman::get_predicted_centre() const {
  return Pt_d(predicted_centre, id);
}