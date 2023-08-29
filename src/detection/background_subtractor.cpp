#include "background_subtractor.hpp"

BackgroundSubtractor::BackgroundSubtractor(const cv::Mat& first_frame,
                                           float alpha)
    : alpha(alpha) {
  cv::cvtColor(first_frame, background, cv::COLOR_BGR2GRAY);
  background.convertTo(background, CV_32F);
  subtracted_frame = background.clone();
}

cv::Mat BackgroundSubtractor::subtract(const cv::Mat& frame) {
  // Convert frame to grayscale
  cv::Mat gray_frame;
  cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
  gray_frame.convertTo(gray_frame, CV_32F);

  // Update the background model the background is already in grayscale
  background = alpha * gray_frame + (1 - alpha) * background;

  // Calculate the absolute difference between the frame and background
  cv::Mat diff = cv::abs(gray_frame - background);

  // Binarize the difference
  cv::Mat thresholded_diff;
  cv::threshold(diff, thresholded_diff, 30, 255, cv::THRESH_BINARY);

  return thresholded_diff;
}

cv::Mat BackgroundSubtractor::getBackground() const {
  return background;
}
cv::Mat BackgroundSubtractor::getSubtractedFrame() const {
  return subtracted_frame;
}
void BackgroundSubtractor::setSubtractedFrame(const cv::Mat& frame) {
  subtracted_frame = frame;
}
