#include "background_subtractor.hpp"

/**
 * @brief The ObjectDetector class uses background subtraction to detect
 * objects in a frame.
 */
class ObjectDetector {
 private:
  BackgroundSubtractor bg_subtractor;  // Background subtractor instance.

 public:
  // Constructor: Initializes the bg_subtractor with the first frame.
  ObjectDetector(const cv::Mat& first_frame, float alpha = 0.01);

  // Detects blobs (objects) in the given binarized image and returns their
  // bounding boxes.
  std::vector<std::vector<int>> detectBlobs(const cv::Mat& binarized_image);

  // Detects objects in the given frame by first subtracting the background.
  std::vector<std::vector<int>> detectObjects(const cv::Mat& frame);

  // Detects the laser point in the given frame and returns its position (x, y).
  std::pair<int, int> detectLaser(const cv::Mat& frame,
                                  int hue_lower,
                                  int hue_upper,
                                  int min_intensity);
};