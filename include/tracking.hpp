#pragma once

#include <vector>
#include "frame.hpp"
#include "kalman.hpp"

class Tracking {
 private:
  int num_trackers = 0;
  std::vector<Kalman> kalman_trackers;

  void init_kalman();
  Pt track_mosquito(Pt blob_centre);

 public:
  Pt track_mosquitos(std::vector<Pt> blob_centres);
};