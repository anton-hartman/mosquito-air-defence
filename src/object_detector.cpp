#include "../include/object_detector.hpp"
#include "../include/utils.hpp"

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

// Old detection method, not used anymore
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

/**
 * @brief Puts a text label on an image.
 *
 * This function places a given text label on a specified image at the given
 * origin point. The label is typically used for marking and identification
 * purposes on image displays.
 *
 * @param img The image on which the label will be placed.
 * @param label The text string to be placed on the image.
 * @param origin The top-left corner of the text string in the image.
 *               For example, a point (10,30) means the text starts 10 pixels
 * from the left and 30 pixels from the top of the image.
 */
void putLabel(cv::Mat& img, const std::string& label, const cv::Point& origin) {
  int fontFace = cv::QT_FONT_NORMAL;
  double fontScale = 1;
  int thickness = 2;
  cv::putText(img, label, origin, fontFace, fontScale, cv::Scalar(0, 255, 255),
              thickness);
}

#pragma region HSV Slider Functions
void ObjectDetector::setRedThresholds(int hl,
                                      int sl,
                                      int vl,
                                      int hh,
                                      int sh,
                                      int vh) {
  hLowRed = hl;
  sLowRed = sl;
  vLowRed = vl;
  hHighRed = hh;
  sHighRed = sh;
  vHighRed = vh;
}

void ObjectDetector::setWhiteThresholds(int hl,
                                        int sl,
                                        int vl,
                                        int hh,
                                        int sh,
                                        int vh) {
  hLowWhite = hl;
  sLowWhite = sl;
  vLowWhite = vl;
  hHighWhite = hh;
  sHighWhite = sh;
  vHighWhite = vh;
}

void ObjectDetector::createThresholdTrackbars() {
  cv::namedWindow("Red Thresholds", cv::WINDOW_NORMAL);
  cv::namedWindow("White Thresholds", cv::WINDOW_NORMAL);

  // Red Mask
  cv::createTrackbar("Low H", "Red Thresholds", &hLowRed, 179, onLowHRedChange,
                     this);
  cv::createTrackbar("High H", "Red Thresholds", &hHighRed, 180,
                     onHighHRedChange, this);
  cv::createTrackbar("Low S", "Red Thresholds", &sLowRed, 255, onLowSRedChange,
                     this);
  cv::createTrackbar("High S", "Red Thresholds", &sHighRed, 255,
                     onHighSRedChange, this);
  cv::createTrackbar("Low V", "Red Thresholds", &vLowRed, 255, onLowVRedChange,
                     this);
  cv::createTrackbar("High V", "Red Thresholds", &vHighRed, 255,
                     onHighVRedChange, this);

  // White Mask
  cv::createTrackbar("Low H", "White Thresholds", &hLowWhite, 179,
                     onLowHWhiteChange, this);
  cv::createTrackbar("High H", "White Thresholds", &hHighWhite, 180,
                     onHighHWhiteChange, this);
  cv::createTrackbar("Low S", "White Thresholds", &sLowWhite, 255,
                     onLowSWhiteChange, this);
  cv::createTrackbar("High S", "White Thresholds", &sHighWhite, 255,
                     onHighSWhiteChange, this);
  cv::createTrackbar("Low V", "White Thresholds", &vLowWhite, 255,
                     onLowVWhiteChange, this);
  cv::createTrackbar("High V", "White Thresholds", &vHighWhite, 255,
                     onHighVWhiteChange, this);
}

void ObjectDetector::onLowHRedChange(int value, void* ptr) {
  ObjectDetector* objDet = static_cast<ObjectDetector*>(ptr);
  objDet->hLowRed = value;
}

void ObjectDetector::onHighHRedChange(int value, void* ptr) {
  ObjectDetector* objDet = static_cast<ObjectDetector*>(ptr);
  objDet->hHighRed = value;
}

// Callback functions for Red Mask
void ObjectDetector::onLowSRedChange(int value, void* ptr) {
  ObjectDetector* objDet = static_cast<ObjectDetector*>(ptr);
  objDet->sLowRed = value;
}

void ObjectDetector::onHighSRedChange(int value, void* ptr) {
  ObjectDetector* objDet = static_cast<ObjectDetector*>(ptr);
  objDet->sHighRed = value;
}

void ObjectDetector::onLowVRedChange(int value, void* ptr) {
  ObjectDetector* objDet = static_cast<ObjectDetector*>(ptr);
  objDet->vLowRed = value;
}

void ObjectDetector::onHighVRedChange(int value, void* ptr) {
  ObjectDetector* objDet = static_cast<ObjectDetector*>(ptr);
  objDet->vHighRed = value;
}

// Callback functions for White Mask
void ObjectDetector::onLowHWhiteChange(int value, void* ptr) {
  ObjectDetector* objDet = static_cast<ObjectDetector*>(ptr);
  objDet->hLowWhite = value;
}

void ObjectDetector::onHighHWhiteChange(int value, void* ptr) {
  ObjectDetector* objDet = static_cast<ObjectDetector*>(ptr);
  objDet->hHighWhite = value;
}

void ObjectDetector::onLowSWhiteChange(int value, void* ptr) {
  ObjectDetector* objDet = static_cast<ObjectDetector*>(ptr);
  objDet->sLowWhite = value;
}

void ObjectDetector::onHighSWhiteChange(int value, void* ptr) {
  ObjectDetector* objDet = static_cast<ObjectDetector*>(ptr);
  objDet->sHighWhite = value;
}

void ObjectDetector::onLowVWhiteChange(int value, void* ptr) {
  ObjectDetector* objDet = static_cast<ObjectDetector*>(ptr);
  objDet->vLowWhite = value;
}

void ObjectDetector::onHighVWhiteChange(int value, void* ptr) {
  ObjectDetector* objDet = static_cast<ObjectDetector*>(ptr);
  objDet->vHighWhite = value;
}
#pragma endregion HSV Slider Functions

std::pair<int, int> ObjectDetector::detectLaser(const cv::Mat& frame) {
  // Convert to HSV
  cv::Mat hsv;
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

  // Threshold for red using the class members
  cv::Mat maskRed1, maskRed2, maskRed;
  cv::inRange(hsv, cv::Scalar(hLowRed, sLowRed, vLowRed),
              cv::Scalar(hHighRed, sHighRed, vHighRed), maskRed1);
  cv::inRange(hsv, cv::Scalar(170, sLowRed, vLowRed),
              cv::Scalar(180, sHighRed, vHighRed), maskRed2);
  maskRed = maskRed1 | maskRed2;

  // Threshold for white using the class members
  cv::Mat maskWhite;
  cv::inRange(hsv, cv::Scalar(hLowWhite, sLowWhite, vLowWhite),
              cv::Scalar(hHighWhite, sHighWhite, vHighWhite), maskWhite);

  // Find contours
  std::vector<std::vector<cv::Point>> contoursRed, contoursWhite;
  cv::findContours(maskRed, contoursRed, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  cv::findContours(maskWhite, contoursWhite, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  // Black image
  cv::Mat cntrsImg(frame.size(), CV_8UC3, cv::Scalar(0, 0, 0));
  // Draw red contours in red
  cv::drawContours(cntrsImg, contoursRed, -1, cv::Scalar(0, 0, 255), 2);
  // Draw white contours in white
  cv::drawContours(cntrsImg, contoursWhite, -1, cv::Scalar(255, 255, 255), 2);

  cv::Mat displayFrame = frame.clone();
  int proximityDistance = 10;

  for (std::vector<cv::Point>& contourWhite : contoursWhite) {
    cv::Rect rectWhite = cv::boundingRect(contourWhite);

    if (rectWhite.width >= 5 && rectWhite.width <= 50 &&
        rectWhite.height >= 5 && rectWhite.height <= 50) {
      cv::rectangle(displayFrame, rectWhite, cv::Scalar(0, 255, 0), 2);

      bool isNearRed = false;

      for (cv::Point whitePt : contourWhite) {
        for (std::vector<cv::Point>& contourRed : contoursRed) {
          for (cv::Point redPt : contourRed) {
            if (cv::norm(whitePt - redPt) < proximityDistance) {
              isNearRed = true;
              break;
            }
          }
          if (isNearRed)
            break;
        }
        if (isNearRed)
          break;
      }

      if (isNearRed) {
        cv::Point centerWhite(rectWhite.x + rectWhite.width / 2,
                              rectWhite.y + rectWhite.height / 2);
        cv::circle(displayFrame, centerWhite, 4, cv::Scalar(255, 0, 0), -1);
      }
    }
  }

  // Prepare for display
  cv::resize(maskRed, maskRed, frame.size());
  cv::resize(maskWhite, maskWhite, frame.size());
  cv::cvtColor(maskRed, maskRed, cv::COLOR_GRAY2BGR);
  cv::cvtColor(maskWhite, maskWhite, cv::COLOR_GRAY2BGR);

  // Add Labels
  putLabel(displayFrame, "Original", cv::Point(10, 30));
  putLabel(maskRed, "Red Mask", cv::Point(10, 30));
  putLabel(maskWhite, "White Mask", cv::Point(10, 30));
  putLabel(cntrsImg, "Contours", cv::Point(10, 30));

  cv::Mat concatenatedOutput;
  cv::hconcat(maskWhite, maskRed, concatenatedOutput);
  cv::hconcat(cntrsImg, displayFrame, displayFrame);
  cv::vconcat(concatenatedOutput, displayFrame, concatenatedOutput);

  // Resize the concatenated output to fit the screen if necessary
  const int screenWidth = 1500;  // Modify this as per your screen resolution
  if (concatenatedOutput.cols > screenWidth) {
    float scale = (float)screenWidth / concatenatedOutput.cols;
    cv::resize(concatenatedOutput, concatenatedOutput, cv::Size(), scale,
               scale);
  }

  cv::imshow("Combined Display", concatenatedOutput);
  char key = static_cast<char>(cv::waitKey(1));
  if (key == 'q') {
    exit(0);
  }

  return {0, 0};  // Return appropriate values, placeholder for now
}
