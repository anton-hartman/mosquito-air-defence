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

std::pair<int, int> ObjectDetector::detectLaser(const cv::Mat& frame,
                                                int hue_lower,
                                                int hue_upper,
                                                int min_intensity) {
  // Convert the frame to HSV
  cv::Mat hsv;
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

  // Threshold based on hue and intensity
  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(hue_lower, 50, min_intensity),
              cv::Scalar(hue_upper, 255, 255), mask);

  // Display the mask
  cv::imshow("Mask", mask);

  // Find blobs in the mask
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // Analyze blobs to find the laser point
  for (const auto& contour : contours) {
    if (cv::contourArea(contour) <= 10) {  // Assuming laser point is small
      cv::Moments moments = cv::moments(contour);
      int cx = static_cast<int>(moments.m10 / moments.m00);
      int cy = static_cast<int>(moments.m01 / moments.m00);
      return {cx, cy};  // Return the centroid of the laser point
    }
  }
  return {-1, -1};  // Return an invalid position if laser not found
}

std::pair<int, int> ObjectDetector::detectLaserWit(const cv::Mat& frame,
                                                   cv::Scalar lower_threshold) {
  cv::Mat mask;
  cv::inRange(frame, lower_threshold, cv::Scalar(255, 255, 255), mask);

  // // Display the mask
  cv::imshow("Mask", mask);

  // Find blobs in the mask
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // Analyze blobs to find the laser point
  for (const auto& contour : contours) {
    if (cv::contourArea(contour) <= 10) {  // Assuming laser point is small
      cv::Moments moments = cv::moments(contour);
      int cx = static_cast<int>(moments.m10 / moments.m00);
      int cy = static_cast<int>(moments.m01 / moments.m00);
      return {cx, cy};  // Return the centroid of the laser point
    }
  }
  return {-1, -1};  // Return an invalid position if laser not found
}
