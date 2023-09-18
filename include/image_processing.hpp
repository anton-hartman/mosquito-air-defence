#include <opencv2/opencv.hpp>

namespace imgs {

void undistort(cv::Mat& input_frame,
               cv::Mat& output_frame,
               cv::Mat& camera_matrix,
               cv::Mat& dist_coeffs);

}