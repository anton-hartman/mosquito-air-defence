#include "../include/tracking.hpp"
#include <iostream>

void Tracking::add_kalman() {
  kalman_trackers.push_back(Kalman(0.035, 1.0, 1.0, 1.0, 0.1, 0.1));
}

Pt Tracking::track_mosquito(Pt blob_centre) {
  if (num_trackers == 0) {
    add_kalman();
    num_trackers++;
  }

  Eigen::Vector2d updated_vals =
      kalman_trackers.at(0).update(cv::Point(blob_centre.x, blob_centre.y));
  Eigen::Vector2d predicted_vals = kalman_trackers.at(0).predict();

  std::cout << "blob centre: (" << blob_centre.x << ", " << blob_centre.y << ")"
            << std::endl;
  std::cout << "updated: " << updated_vals << std::endl;
  std::cout << "predicted: " << predicted_vals << std::endl;

  return {static_cast<int>(std::round(predicted_vals(0))),
          static_cast<int>(std::round(predicted_vals(1)))};
}

// Pt Tracking::track_mosquitos(std::vector<Pt> blob_centres) {}

void Tracking::associate_and_update(const std::vector<Pt>& blob_centres) {
  std::cout << "hello";
}
