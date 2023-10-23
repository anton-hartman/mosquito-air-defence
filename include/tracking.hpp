#pragma once

#include <atomic>
#include <vector>
#include "kalman_no_eigen.hpp"
#include "pt.hpp"

namespace tracking {

extern int current_track_id;
extern std::vector<Kalman> kalmans;
extern std::atomic<int> acc_sigma;
extern std::atomic<float> kalman_dt;

/**
 * @brief Associate detections with predictions and update trackers
 *
 * @param blobs The detections at time t.
 * @param dt The time interval between t and t-1.
 */
void associate_and_update_tracks(const std::vector<Pt>& blobs, const int dt);
Track get_closest_track(const Pt& pt);
Track get_current_track_preditction(const Pt& laser_pt);

}  // namespace tracking