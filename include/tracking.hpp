#pragma once

#include <vector>
#include "pt.hpp"

namespace tracking {

/**
 * @brief Associate detections with predictions and update trackers
 */
void associate_and_update_tracks(const std::vector<Pt>& blob);
void predict_centres();
std::vector<Pt> get_predicted_centres();
Pt get_predicted_centre(const int& id);

}  // namespace tracking