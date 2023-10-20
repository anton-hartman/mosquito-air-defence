#include "../include/tracking.hpp"
#include <iostream>
#include "../include/hungarian.hpp"
#include "../include/mads.hpp"

using Matrix = std::vector<std::vector<double>>;

namespace tracking {

HungarianAlgorithm hungarian;
int max_age = 5;

Matrix get_cost_matrix(const std::vector<Pt>& blobs) {
  auto euclidean_dist = [](Pt predicted, Pt detected) {
    return std::sqrt(std::pow(predicted.x - detected.x, 2) +
                     std::pow(predicted.y - detected.y, 2));
  };

  Matrix cost_matrix;
  for (int i = 0; i < kalmans.size(); i++) {
    for (int j = 0; j < blobs.size(); j++) {
      cost_matrix.at(i).at(j) =
          euclidean_dist(kalmans.at(i).track.pt, blobs.at(j));
    }
  }
}

void associate_and_update_tracks(const std::vector<Pt>& blobs, const int dt) {
  if (blobs.size() == 0) {
    return;
  }

  if (kalmans.size() != 0) {
    Matrix cost_matrix = get_cost_matrix(blobs);

    if (mads::debug.load() & Debug::TRACKING) {
      std ::cout << "cost matrix: " << std::endl;
      for (int i = 0; i < cost_matrix.size(); i++) {
        for (int j = 0; j < cost_matrix.at(i).size(); j++) {
          std::cout << cost_matrix.at(i).at(j) << " ";
        }
        std::cout << std::endl;
      }
    }

    vector<int> assignment;
    double cost = hungarian.Solve(cost_matrix, assignment);

    // Using two indicies because elements can be removed from the vector.
    int x = 0;
    for (int i = 0; i < assignment.size(); ++i) {
      if (assignment.at(i) == -1) {
        if (kalmans.at(i).age > max_age) {
          kalmans.erase(kalmans.begin() + i);
          continue;
        } else {
          ++kalmans.at(i).track.age;
        }
      } else {
        kalmans.at(x).update(blobs.at(assignment.at(i)));
        ++x;
      }
    }

    // Add new trackers for blobs that have no associated trackers (new
    // mosquitos)
    if (blobs.size() > kalmans.size()) {
      std::vector<Pt> unassigned_blobs = blobs;  // Deep copy
      for (int i = 0; i < assignment.size(); i++) {
        unassigned_blobs.at(assignment.at(i)) = {-9, -9};
      }
      unassigned_blobs.erase(std::remove(unassigned_blobs.begin(),
                                         unassigned_blobs.end(), Pt{-9, -9}),
                             unassigned_blobs.end());

      for (const Pt& blob : unassigned_blobs) {
        kalmans.push_back(Kalman(blob, dt, 1.0, 1.0, 1.0, 0.1, 0.1));
      }
    }
  } else {
    for (const Pt& blob : blobs) {
      kalmans.push_back(Kalman(blob, dt, 1.0, 1.0, 1.0, 0.1, 0.1));
    }
  }
}

Track track_closest_to_laser(const Pt& laser_pt) {
  if (kalmans.size() == 0) {
    return Track();
  }

  double min_dist = 1000000;
  int min_dist_index = -1;
  for (int i = 0; i < kalmans.size(); i++) {
    double dist = std::sqrt(std::pow(kalmans.at(i).track.pt.x - laser_pt.x, 2) +
                            std::pow(kalmans.at(i).track.py.y - laser_pt.y, 2));
    if (dist < min_dist) {
      min_dist = dist;
      min_dist_index = i;
    }
  }
  current_track_id = kalmans.at(min_dist_index).track.id;
  return kalmans.at(min_dist_index).track;
}

Track get_current_track_preditction(const Pt& laser_pt) {
  if (kalmans.size() == 0) {
    return {-1, -1};
  }

  Track track;
  for (Kalman& kalman : kalmans) {
    kalman.predict();
    if (kalman.track.id == current_track_id) {
      track = kalman.track;
    }
  }

  if (track == nullptr) {
    return get_closest_to_laser(laser_pt);
  } else {
    return track;
  }
}

}  // namespace tracking