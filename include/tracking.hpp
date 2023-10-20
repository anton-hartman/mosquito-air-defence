#pragma once

#include <vector>
#include "hungarian.hpp"
#include "kalman.hpp"
#include "pt.hpp"

// #include "mos.hpp"

class Tracking {
 private:
  //   std::vector<Mos> mos_trackers;
  HungarianAlgorithm hungarian;

  void add_kalman();
  std::vector<std::vector<double>> get_cost_matrix(
      const std::vector<Pt>& blob_centres);
  double euclidean_distance(const Pt_d& predicted, const Pt& detected);

 public:
  /**
   * @brief Associate detections with predictions and update trackers
   */
  void associate_and_update(const std::vector<Pt>& blob_centres);
  void predict_centres();
  std::vector<Pt> get_predicted_centres() const;
  Pt get_predicted_centre(const int& id) const;
};