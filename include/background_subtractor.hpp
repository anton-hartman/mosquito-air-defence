#pragma once

#include <opencv2/opencv.hpp>

/**
 * @brief The BackgroundSubtractor class is responsible for subtracting the
 * background from a given frame to detect changes or objects.
 */
class BackgroundSubtractor {
 private:
  cv::Mat background;        // The current background model.
  float alpha;               // Weight for updating the background model.
  cv::Mat subtracted_frame;  // The frame obtained after background subtraction.

 public:
  // Constructor: Initializes the background with the first frame in grayscale.
  BackgroundSubtractor(const cv::Mat& first_frame, float alpha = 0.01);

  // Subtracts the background from the given frame.
  cv::Mat subtract(const cv::Mat& frame);

  // Getters and Setters
  cv::Mat getBackground() const;
  cv::Mat getSubtractedFrame() const;
  void setSubtractedFrame(const cv::Mat& frame);
};
