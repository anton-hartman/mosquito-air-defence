#pragma once

#include <vector>
#include "frame.hpp"
#include "kalman_filter_2d.hpp"

class Tracking {
 private:
  int num_trackers = 0;
  std::vector<KalmanFilter2D> kalman_trackers;

  void init_kalman();
  Pt track_mosquito(Pt blob_centre);

 public:
  Pt track_mosquitos(std::vector<Pt> blob_centres);
};