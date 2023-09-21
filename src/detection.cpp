#include "../include/detection.hpp"
#include "../include/frame.hpp"
#include "../include/utils.hpp"

Detection::Detection(const cv::Mat& first_frame, float alpha) {
  this->alpha = alpha;
  cv::cvtColor(first_frame, background, cv::COLOR_BGR2GRAY);
  background.convertTo(background, CV_32F);
}

/**
 * @brief Perfroms background subtraction and updates the background model
 * according to alpha.
 */
cv::Mat Detection::subtract(const cv::Mat& frame) {
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

std::vector<std::vector<int>> Detection::detect_mosquitoes(
    const cv::Mat& frame) {
  cv::Mat bin_image = subtract(frame);
  bin_image.convertTo(bin_image, CV_8U);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(bin_image, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<int>> bounding_boxes;
  for (const auto& cnt : contours) {
    if (cv::contourArea(cnt) > 1) {
      cv::Moments moments = cv::moments(cnt);
      int cx = static_cast<int>(moments.m10 / moments.m00);
      int cy = static_cast<int>(moments.m01 / moments.m00);
      cv::Rect rect = cv::boundingRect(cnt);
      std::vector<int> bbox = {rect.x, rect.y, rect.x + rect.width,
                               rect.y + rect.height, 1};
      bounding_boxes.push_back(bbox);
    }
  }
  return bounding_boxes;
}