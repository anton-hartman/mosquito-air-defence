#include "../include/tracking.hpp"
#include <iostream>
#include "../include/hungarian.hpp"
#include "../include/kalman_no_eigen.hpp"

namespace tracking {

using Matrix = std::vector<std::vector<double>>;

HungarianAlgorithm hungarian;
std::vector<Kalman> tracks;
int max_age = 5;

Matrix get_cost_matrix(const std::vector<Pt>& blobs) {
  auto euclidean_dist = [](Pt_d predicted, Pt detected) {
    return std::sqrt(std::pow(predicted.x - detected.x, 2) +
                     std::pow(predicted.y - detected.y, 2));
  };

  Matrix cost_matrix;
  for (int i = 0; i < tracks.size(); i++) {
    for (int j = 0; j < blobs.size(); j++) {
      cost_matrix.at(i).at(j) =
          euclidean_dist(tracks.at(i).pt_d(), blobs.at(j));
    }
  }
}

void associate_and_update_tracks(const std::vector<Pt>& blobs) {
  if (blobs.size() == 0) {
    return;
  }

  if (tracks.size() != 0) {
    std::vector<std::vector<double>> cost_matrix = get_cost_matrix(blobs);

    std::cout << "cost matrix: " << std::endl;
    for (int i = 0; i < cost_matrix.size(); i++) {
      for (int j = 0; j < cost_matrix.at(i).size(); j++) {
        std::cout << cost_matrix.at(i).at(j) << " ";
      }
      std::cout << std::endl;
    }

    vector<int> assignment;
    double cost = hungarian.Solve(cost_matrix, assignment);

    // Using two indicies because elements can be removed from the vector.
    int x = 0;
    for (int i = 0; i < assignment.size(); ++i) {
      if (assignment.at(i) == -1) {
        if (tracks.at(i).age > max_age) {
          tracks.erase(tracks.begin() + i);
          continue;
        } else {
          ++tracks.at(i).age;
        }
      } else {
        tracks.at(x).update(blobs.at(assignment.at(i)));
        ++x;
      }
    }

    // Add new trackers for blobs that have no associated trackers (new
    // mosquitos)
    if (blobs.size() > tracks.size()) {
      std::vector<Pt> unassigned_blobs = blobs;  // Deep copy
      for (int i = 0; i < assignment.size(); i++) {
        unassigned_blobs.at(assignment.at(i)) = {-9, -9};
      }
      unassigned_blobs.erase(std::remove(unassigned_blobs.begin(),
                                         unassigned_blobs.end(), Pt{-9, -9}),
                             unassigned_blobs.end());

      for (int i = 0; i < unassigned_blobs.size(); i++) {
        tracks.push_back(Kalman(0.035, 1.0, 1.0, 1.0, 0.1, 0.1));
        tracks.end()->update(unassigned_blobs.at(i));
      }
    }
  } else {
    for (int i = 0; i < blobs.size(); i++) {
      tracks.push_back(Kalman(0.035, 1.0, 1.0, 1.0, 0.1, 0.1));
      tracks.end()->update(blobs.at(i));
    }
  }
}

// void Tracking::predict_centres() {
//   for (int i = 0; i < tracks.size(); i++) {
//     tracks.at(i).predict();
//   }
// }

// std::vector<Pt> Tracking::get_predicted_centres() const {
//   std::vector<Pt> predicted_centres;
//   for (int i = 0; i < tracks.size(); i++) {
//     predicted_centres.push_back(tracks.at(i).get_predicted_centre());
//   }
//   return predicted_centres;
// }

// Pt Tracking::get_predicted_centre(const int& id) const {
//   if (tracks.size() == 0) {
//     return {-1, -1};
//   }
//   if (id == -1) {
//     return tracks.at(0).get_predicted_centre();
//   }
//   for (int i = 0; i < tracks.size(); i++) {
//     if (tracks.at(i).get_id() == id) {
//       return tracks.at(i).get_predicted_centre();
//     }
//   }
// }

}  // namespace tracking