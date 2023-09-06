#include "../include/image_processing.hpp"

namespace imgs {

void undistort(cv::Mat& input_frame,
               cv::Mat& output_frame,
               cv::Mat& camera_matrix,
               cv::Mat& dist_coeffs) {
  cv::undistort(input_frame, output_frame, camera_matrix, dist_coeffs);
}

}  // namespace imgs