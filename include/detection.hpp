#include <opencv2/opencv.hpp>
#include "utils.hpp"

class Detection {
 private:
  cv::Mat background;  // The current background model.
  float alpha;         // Weight for updating the background model.

  cv::Mat subtract(const cv::Mat& frame);

 public:
  // Constructor: Initializes the bg_subtractor with the first frame.
  Detection(const cv::Mat& first_frame, float alpha = 0.01);

  // Detects blobs in the given binarized image and returns their bounding
  // boxes.
  std::vector<std::vector<int>> detect_mosquitoes(const cv::Mat& frame);
};
