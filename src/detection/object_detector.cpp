#include "object_detector.hpp"

ObjectDetector::ObjectDetector(const cv::Mat& first_frame, float alpha)
    : bg_subtractor(first_frame, alpha) {}

std::vector<std::vector<int>> ObjectDetector::detectBlobs(
    const cv::Mat& binarized_image) {
  cv::Mat bin_image = binarized_image.clone();
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

std::vector<std::vector<int>> ObjectDetector::detectObjects(
    const cv::Mat& frame) {
  cv::Mat subtracted_frame = bg_subtractor.subtract(frame);
  return detectBlobs(subtracted_frame);
}