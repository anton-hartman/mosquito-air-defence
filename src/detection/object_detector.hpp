#include "background_subtractor.hpp"

/**
 * @brief The ObjectDetector class uses background subtraction to detect
 * objects in a frame.
 */
class ObjectDetector {
 private:
  BackgroundSubtractor bg_subtractor;  // Background subtractor instance.

  // Thresholding values for red and white
  int hLowRed, sLowRed, vLowRed;
  int hHighRed, sHighRed, vHighRed;
  int hLowWhite, sLowWhite, vLowWhite;
  int hHighWhite, sHighWhite, vHighWhite;

 public:
  // Constructor: Initializes the bg_subtractor with the first frame.
  ObjectDetector(const cv::Mat& first_frame, float alpha = 0.01);

  // Detects blobs (objects) in the given binarized image and returns their
  // bounding boxes.
  std::vector<std::vector<int>> detectBlobs(const cv::Mat& binarized_image);

  // Detects objects in the given frame by first subtracting the background.
  std::vector<std::vector<int>> detectObjects(const cv::Mat& frame);

  std::pair<int, int> detectLaserWit(const cv::Mat& frame,
                                     cv::Scalar lower_threshold);

  std::pair<int, int> detectLaser(const cv::Mat& frame);

  // Setters for threshold values (optional, you can also initialize in the
  // constructor)
  void setRedThresholds(int hl, int sl, int vl, int hh, int sh, int vh);
  void setWhiteThresholds(int hl, int sl, int vl, int hh, int sh, int vh);

  // Additional function for creating trackbars
  void createThresholdTrackbars();
  // Callbacks for trackbars
  static void onLowHRedChange(int, void* ptr);
  static void onHighHRedChange(int, void* ptr);
  static void onLowSRedChange(int, void* ptr);
  static void onHighSRedChange(int, void* ptr);
  static void onLowVRedChange(int, void* ptr);
  static void onHighVRedChange(int, void* ptr);
  static void onLowHWhiteChange(int, void* ptr);
  static void onHighHWhiteChange(int, void* ptr);
  static void onLowSWhiteChange(int, void* ptr);
  static void onHighSWhiteChange(int, void* ptr);
  static void onLowVWhiteChange(int, void* ptr);
  static void onHighVWhiteChange(int, void* ptr);
};
