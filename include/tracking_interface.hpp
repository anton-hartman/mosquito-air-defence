
#pragma once

#include <vector>
#include "frame.hpp"
#include "kalman_interface.hpp"

// Function declarations from tracking.hpp without Eigen dependencies
class Tracking {
 public:
  int num_trackers = 0;
  std::vector<Kalman> kalman_trackers;
  void add_kalman();
  Pt track_mosquito(Pt blob_centre);
  //   Pt track_mosquitos(std::vector<Pt> blob_centres);

  /**
   * @brief Associate detections with predictions and update trackers
   */
  void associate_and_update(const std::vector<Pt>& blob_centres);
};
