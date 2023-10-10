#include "../include/tracking.hpp"

void Tracking::init_kalman() {}

Pt Tracking::track_mosquito(Pt blob_centre) {}

Pt Tracking::track_mosquitos(std::vector<Pt> blob_centres) {
  if (blob_centres.size() > num_tracks) {
    init_kalman();
  }
}
