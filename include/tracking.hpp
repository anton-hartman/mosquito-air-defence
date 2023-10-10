#pragma once

#include <vector>
#include "frame.hpp"

class Tracking {
 private:
  int num_tracks = 0;

  void init_kalman();
  Pt track_mosquito(Pt blob_centre);

 public:
  Pt track_mosquitos(std::vector<Pt> blob_centres);
};