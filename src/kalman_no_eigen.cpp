#include "../include/kalman_no_eigen.hpp"

#pragma region Matrix Operations

// Function for Matrix multiplication
Matrix multiply(const Matrix& A, const Matrix& B) {
  int rowsA = A.size(), colsA = A[0].size();
  int rowsB = B.size(), colsB = B[0].size();
  Matrix C(rowsA, std::vector<double>(colsB, 0));

  for (int i = 0; i < rowsA; i++) {
    for (int j = 0; j < colsB; j++) {
      for (int k = 0; k < colsA; k++) {
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  return C;
}

// Function for Matrix to Vector multiplication
std::vector<double> multiply(const Matrix& A, const std::vector<double>& B) {
  int rowsA = A.size(), colsA = A[0].size();
  std::vector<double> C(rowsA, 0);

  for (int i = 0; i < rowsA; i++) {
    for (int j = 0; j < colsA; j++) {
      C[i] += A[i][j] * B[j];
    }
  }
  return C;
}

// Function for Matrix transpose
Matrix transpose(const Matrix& A) {
  int rowsA = A.size(), colsA = A[0].size();
  Matrix B(colsA, std::vector<double>(rowsA, 0));
  for (int i = 0; i < rowsA; i++) {
    for (int j = 0; j < colsA; j++) {
      B[j][i] = A[i][j];
    }
  }
  return B;
}

// Function for Matrix inverse (only for 2x2 matrices)
Matrix inverse_2x2(const Matrix& A) {
  double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
  Matrix B(2, std::vector<double>(2, 0));
  B[0][0] = A[1][1] / det;
  B[0][1] = -A[0][1] / det;
  B[1][0] = -A[1][0] / det;
  B[1][1] = A[0][0] / det;
  return B;
}

// Function for Vector to Vector subtraction
std::vector<double> subtract(const std::vector<double>& A,
                             const std::vector<double>& B) {
  int size = A.size();
  std::vector<double> C(size);
  for (int i = 0; i < size; i++) {
    C[i] = A[i] - B[i];
  }
  return C;
}

// Function for identity matrix
Matrix identity(int size) {
  Matrix A(size, std::vector<double>(size, 0));
  for (int i = 0; i < size; i++) {
    A[i][i] = 1;
  }
  return A;
}

#pragma endregion Matrix Operations

int Kalman::id_counter = 0;

Kalman::Kalman(Pt detected_pt,
               double dt_,
               double a_x_,
               double a_y_,
               double sigma_a,
               double sigma_z_x,
               double sigma_z_y)
    : dt(dt_),
      std_acc(sigma_a),
      std_meas_x(sigma_z_x),
      std_meas_y(sigma_z_y),
      track(id_counter++, detected_pt) {
  // Initialize matrices and vectors here
  x = {static_cast<double>(detected_pt.x), static_cast<double>(detected_pt.y),
       0, 0};
  u = {a_x_, a_y_};

  // clang-format off

  // State transition matrix A
  A = {{1, 0, dt,  0},
       {0, 1,  0, dt},
       {0, 0,  1,  0}, 
       {0, 0,  0,  1}};

  // Control input matrix B
  B = {{dt * dt / 2,           0},
       {          0, dt * dt / 2}, 
       {         dt,           0},
       {          0,          dt}};

  // Measurement matrix H
  H = {{1, 0, 0, 0},
       {0, 1, 0, 0}};

  // Process noise covariance Q
  double c1 = pow(dt, 4) / 4;
  double c2 = pow(dt, 3) / 2;
  double c3 = pow(dt, 2);
  double sig_a_2 = std_acc * std_acc;
  Q = {{c1 * sig_a_2,            0, c2 * sig_a_2,            0},
       {           0, c1 * sig_a_2,            0, c2 * sig_a_2},
       {c2 * sig_a_2,            0, c3 * sig_a_2,            0},
       {           0, c2 * sig_a_2,            0, c3 * sig_a_2}};

  // Measurement noise covariance R
  R = {{std_meas_x * std_meas_x,                       0}, 
       {                      0, std_meas_y * std_meas_y}};

  // clang-format on

  // Error covariance P
  P = identity(4);
}

Track Kalman::predict() {
  // x = A * x + B * u
  x = multiply(A, x);
  std::vector<double> Bu = multiply(B, u);
  for (size_t i = 0; i < x.size(); ++i) {
    x[i] += Bu[i];
  }

  // P = A * P * A^T + Q
  Matrix AP = multiply(A, P);
  Matrix APAT = multiply(AP, transpose(A));
  for (size_t i = 0; i < 4; ++i) {
    for (size_t j = 0; j < 4; ++j) {
      P[i][j] = APAT[i][j] + Q[i][j];
    }
  }

  track.pt = {x[0], x[1]};
  pt_hist.push_back(track.pt);
  if (pt_hist.size() > track.max_hist) {
    pt_hist.pop_front();
  }
  return track;
}

// std::vector<double> Kalman::predict_without_updating_states(
//     const std::vector<double>& x_t0) const {
//   // x_t1 = A * x_t0 + B * u
//   x_t1 = multiply(A, x_t0);
//   std::vector<double> Bu = multiply(B, u);
//   for (size_t i = 0; i < x.size(); ++i) {
//     x_t1[i] += Bu[i];
//   }

//   // P = A * P * A^T + Q
//   Matrix AP = multiply(A, P);
//   Matrix APAT = multiply(AP, transpose(A));
//   for (size_t i = 0; i < 4; ++i) {
//     for (size_t j = 0; j < 4; ++j) {
//       P[i][j] = APAT[i][j] + Q[i][j];
//     }
//   }

//   return x;
// }

Track Kalman::update(Pt detected_pt) {
  // Measurement update
  std::vector<double> z = {
      static_cast<double>(detected_pt.x),
      static_cast<double>(detected_pt.y)};              // Measurement vector
  std::vector<double> y = subtract(z, multiply(H, x));  // Innovation vector

  // Compute Kalman gain: K = P * H^T * (H * P * H^T + R)^-1
  Matrix PHT = multiply(P, transpose(H));
  Matrix HPHT = multiply(H, PHT);
  Matrix S = {{HPHT[0][0] + R[0][0], HPHT[0][1] + R[0][1]},
              {HPHT[1][0] + R[1][0], HPHT[1][1] + R[1][1]}};
  Matrix K = multiply(PHT, inverse_2x2(S));

  // Update state: x = x + K * y
  std::vector<double> Ky = multiply(K, y);
  for (size_t i = 0; i < x.size(); ++i) {
    x[i] += Ky[i];
  }

  // Update error covariance: P = (I - K * H) * P
  Matrix KH = multiply(K, H);
  Matrix I = identity(4);
  Matrix I_KH(4, std::vector<double>(4, 0));
  for (size_t i = 0; i < 4; ++i) {
    for (size_t j = 0; j < 4; ++j) {
      I_KH[i][j] = I[i][j] - KH[i][j];
    }
  }
  P = multiply(I_KH, P);

  track.pt = {x[0], x[1]};
  track.detected_pt = detected_pt;
    detected_pt_hist.push_back(track.pt);
  if (detected_pt_hist.size() > track.max_hist) {
    detected_pt_hist.pop_front();
  }
  track.age = 0;
  return track;
}