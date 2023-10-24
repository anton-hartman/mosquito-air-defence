#include "../include/tracking.hpp"
#include <iostream>
#include "../include/hungarian.hpp"
#include "../include/mads.hpp"

using Matrix = std::vector<std::vector<double>>;

namespace tracking {

std::atomic<int> acc_sigma(1);
std::atomic<float> kalman_dt(1);
int max_age = 20;
int current_track_id = -1;
std::vector<Kalman> kalmans;
HungarianAlgorithm hungarian;

void add_kalman(const Pt& pt, const int dt) {
  kalmans.push_back(Kalman(pt, dt, 0, 0, acc_sigma.load(), 1, 1));
}

Matrix get_cost_matrix(const std::vector<Pt>& blobs) {
  auto euclidean_dist = [](Pt predicted, Pt detected) {
    return std::sqrt(std::pow(predicted.x - detected.x, 2) +
                     std::pow(predicted.y - detected.y, 2));
  };

  Matrix cost_matrix(kalmans.size(), std::vector<double>(blobs.size(), 0));
  for (int i = 0; i < kalmans.size(); i++) {
    for (int j = 0; j < blobs.size(); j++) {
      // cost_matrix.at(i).at(j) =
      //     euclidean_dist(kalmans.at(i).track.detected_pt, blobs.at(j));
      cost_matrix.at(i).at(j) =
          euclidean_dist(kalmans.at(i).track.predicted_pt, blobs.at(j));
    }
  }
  return cost_matrix;
}

void associate_and_update_tracks(const std::vector<Pt>& blobs, const int dt) {
  if (kalmans.size() != 0) {
    std::vector<int> assignment;
    if (blobs.size() != 0) {
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

      double cost = hungarian.Solve(cost_matrix, assignment);
    } else {
      assignment = std::vector<int>(kalmans.size(), -1);
    }

    // Using two indicies because elements can be removed from the vector.
    int x = 0;
    if (assignment.size() != kalmans.size()) {
      std::cout << "ERROR -> assignment size: " << assignment.size()
                << " kalmans size: " << kalmans.size() << std::endl;
    }
    for (int i = 0; i < assignment.size(); ++i) {
      if (assignment.at(i) == -1) {
        if (kalmans.at(x).track.age > max_age) {
          kalmans.erase(kalmans.begin() + i);
          continue;
        } else {
          ++kalmans.at(x).track.age;
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
        add_kalman(blob, dt);
      }
    }

  } else {
    for (const Pt& blob : blobs) {
      add_kalman(blob, dt);
    }
  }
}

Track get_closest_track(const Pt& pt) {
  if (kalmans.size() == 0) {
    return Track();
  }

  if (pt == -1) {
    Track();
  }

  double min_dist = 1000000;
  int min_dist_index = -1;
  for (int i = 0; i < kalmans.size(); i++) {
    // double dist =
    //     std::sqrt(std::pow(kalmans.at(i).track.detected_pt.x - pt.x, 2) +
    //               std::pow(kalmans.at(i).track.detected_pt.y - pt.y, 2));
    double dist =
        std::sqrt(std::pow(kalmans.at(i).track.predicted_pt.x - pt.x, 2) +
                  std::pow(kalmans.at(i).track.predicted_pt.y - pt.y, 2));
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
    return Track();
  }

  Track track;
  for (Kalman& kalman : kalmans) {
    kalman.predict();
    if (kalman.track.id == current_track_id) {
      track = kalman.track;
    }
  }

  if (track == -1) {
    return get_closest_track(laser_pt);
  } else {
    return track;
  }
}

}  // namespace tracking