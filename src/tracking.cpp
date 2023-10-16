#include "../include/tracking.hpp"
#include <iostream>

void Tracking::add_kalman() {
  kalman_trackers.push_back(Kalman(0.035, 1.0, 1.0, 1.0, 0.1, 0.1));
}

double Tracking::euclidean_distance(const Pt_d& predicted, const Pt& detected) {
  return std::sqrt(std::pow(detected.x - predicted.x, 2) +
                   std::pow(detected.y - predicted.y, 2));
}

std::vector<std::vector<double>> Tracking::get_cost_matrix(
    const std::vector<Pt>& blob_centres) {
  std::vector<std::vector<double>> cost_matrix;
  for (int i = 0; i < kalman_trackers.size(); i++) {
    for (int j = 0; j < blob_centres.size(); j++) {
      cost_matrix.at(i).at(j) = euclidean_distance(
          kalman_trackers.at(i).get_predicted_centre(), blob_centres.at(j));
    }
  }
}

void Tracking::associate_and_update(const std::vector<Pt>& blob_centres) {
  if (blob_centres.size() == 0) {
    return;
  }

  if (kalman_trackers.size() != 0) {
    std::vector<std::vector<double>> cost_matrix =
        get_cost_matrix(blob_centres);

    std::cout << "cost matrix: " << std::endl;
    for (int i = 0; i < cost_matrix.size(); i++) {
      for (int j = 0; j < cost_matrix.at(i).size(); j++) {
        std::cout << cost_matrix.at(i).at(j) << " ";
      }
      std::cout << std::endl;
    }

    vector<int> assignment;
    double cost = hungarian.Solve(cost_matrix, assignment);

    // int x = 0;
    // for (int i = 0; i < assignment.size(); i++) {
    //   // Remove trackers that have no associated blobs
    //   if (assignment.at(i) == -1) {
    //     kalman_trackers.erase(kalman_trackers.begin() + i);
    //     continue;
    //   }
    //   kalman_trackers.at(x).update(blob_centres.at(assignment.at(i)));
    //   x++;
    // }

    // // Add new trackers for blobs that have no associated trackers (new
    // // mosquitos)
    // if (blob_centres.size() > kalman_trackers.size()) {
    //   std::vector<Pt> unassigned_blobs = blob_centres;  // Deep copy
    //   for (int i = 0; i < assignment.size(); i++) {
    //     unassigned_blobs.at(assignment.at(i)) = {-9, -9};
    //   }
    //   unassigned_blobs.erase(std::remove(unassigned_blobs.begin(),
    //                                      unassigned_blobs.end(), Pt{-9, -9}),
    //                          unassigned_blobs.end());

    //   for (int i = 0; i < unassigned_blobs.size(); i++) {
    //     add_kalman();
    //     kalman_trackers.at(kalman_trackers.size() - 1)
    //         .update(unassigned_blobs.at(i));
    //   }
    // }
  } else {
    for (int i = 0; i < blob_centres.size(); i++) {
      add_kalman();
      // kalman_trackers.at(kalman_trackers.size() -
      // 1).update(blob_centres.at(i));
    }
  }
}

void Tracking::predict_centres() {
  for (int i = 0; i < kalman_trackers.size(); i++) {
    kalman_trackers.at(i).predict();
  }
}

std::vector<Pt> Tracking::get_predicted_centres() const {
  std::vector<Pt> predicted_centres;
  for (int i = 0; i < kalman_trackers.size(); i++) {
    predicted_centres.push_back(kalman_trackers.at(i).get_predicted_centre());
  }
  return predicted_centres;
}

Pt Tracking::get_predicted_centre(const int& id) const {
  if (kalman_trackers.size() == 0) {
    return {-1, -1};
  }
  if (id == -1) {
    return kalman_trackers.at(0).get_predicted_centre();
  }
  for (int i = 0; i < kalman_trackers.size(); i++) {
    if (kalman_trackers.at(i).get_id() == id) {
      return kalman_trackers.at(i).get_predicted_centre();
    }
  }
}
