#include "../include/detection.hpp"
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

static void laser_display(cv::Mat& maskRed,
                          cv::Mat& maskWhite,
                          cv::Mat& cntrsImg,
                          cv::Mat& displayFrame) {
  // Prepare for display
  // cv::resize(maskRed, maskRed, frame.size());
  // cv::resize(maskWhite, maskWhite, frame.size());
  cv::cvtColor(maskRed, maskRed, cv::COLOR_GRAY2BGR);
  cv::cvtColor(maskWhite, maskWhite, cv::COLOR_GRAY2BGR);

  // Add Labels
  utils::put_label(displayFrame, "Original",
                   std::pair<uint16_t, uint16_t>(10, 30));
  utils::put_label(maskRed, "Red Mask", std::pair<uint16_t, uint16_t>(10, 30));
  utils::put_label(maskWhite, "White Mask",
                   std::pair<uint16_t, uint16_t>(10, 30));
  utils::put_label(cntrsImg, "Contours", std::pair<uint16_t, uint16_t>(10, 30));

  cv::Mat concatenatedOutput;
  cv::hconcat(maskWhite, maskRed, concatenatedOutput);
  cv::hconcat(cntrsImg, displayFrame, displayFrame);
  cv::vconcat(concatenatedOutput, displayFrame, concatenatedOutput);

  // Resize the concatenated output to fit the screen if necessary
  const int screenWidth = 1200;  // Modify this as per your screen resolution
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
}

std::pair<uint16_t, uint16_t> Detection::detect_laser(
    const cv::Mat& frame,
    const utils::Circle& laser_belief_region_px) {
  std::pair<uint16_t, uint16_t> laser_pos;

  // Convert to HSV
  cv::Mat hsv;
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

  // Threshold for red using the class members
  cv::Mat maskRed1, maskRed2, maskRed;
  cv::inRange(hsv, cv::Scalar(h_red_low_1, s_red_low, v_red_low),
              cv::Scalar(h_red_high_1, s_red_high, v_red_high), maskRed1);
  cv::inRange(hsv, cv::Scalar(h_red_low_2, s_red_low, v_red_low),
              cv::Scalar(h_red_low_2, s_red_high, v_red_high), maskRed2);
  maskRed = maskRed1 | maskRed2;

  // Threshold for white using the class members
  cv::Mat maskWhite;
  cv::inRange(hsv, cv::Scalar(h_white_low, s_white_low, v_white_low),
              cv::Scalar(h_white_high, s_white_high, v_white_high), maskWhite);

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
        laser_pos.first = centerWhite.x;
        laser_pos.second = centerWhite.y;
      }
    }
  }

  laser_display(maskRed, maskWhite, cntrsImg, displayFrame);

  return laser_pos;  // Return appropriate values, placeholder for now
}

#pragma region HSV Slider Functions
void Detection::set_red_thresholds(int hl_1,
                                   int hl_2,
                                   int sl,
                                   int vl,
                                   int hh_1,
                                   int hh_2,
                                   int sh,
                                   int vh) {
  h_red_low_1 = hl_1;
  h_red_low_2 = hl_2;
  s_red_low = sl;
  v_red_low = vl;
  h_red_high_1 = hh_1;
  h_red_high_2 = hh_2;
  s_red_high = sh;
  v_red_high = vh;
}

void Detection::set_white_thresholds(int hl,
                                     int sl,
                                     int vl,
                                     int hh,
                                     int sh,
                                     int vh) {
  h_white_low = hl;
  s_white_low = sl;
  v_white_low = vl;
  h_white_high = hh;
  s_white_high = sh;
  v_white_high = vh;
}

void Detection::create_threshold_trackbars() {
  cv::namedWindow("Red Thresholds", cv::WINDOW_NORMAL);
  cv::namedWindow("White Thresholds", cv::WINDOW_NORMAL);

  // Red Mask
  cv::createTrackbar("Low H1", "Red Thresholds", &h_red_low_1, 179,
                     on_h_1_red_low_change, this);
  cv::createTrackbar("High H1", "Red Thresholds", &h_red_high_1, 180,
                     on_h_1_red_high_change, this);
  cv::createTrackbar("Low H2", "Red Thresholds", &h_red_low_2, 179,
                     on_h_2_red_low_change, this);
  cv::createTrackbar("High H2", "Red Thresholds", &h_red_high_2, 180,
                     on_h_2_red_high_change, this);
  cv::createTrackbar("Low S", "Red Thresholds", &s_red_low, 255,
                     on_s_red_low_change, this);
  cv::createTrackbar("High S", "Red Thresholds", &s_red_high, 255,
                     on_s_red_high_change, this);
  cv::createTrackbar("Low V", "Red Thresholds", &v_red_low, 255,
                     on_v_red_low_change, this);
  cv::createTrackbar("High V", "Red Thresholds", &v_red_high, 255,
                     on_v_red_high_change, this);

  // White Mask
  cv::createTrackbar("Low H", "White Thresholds", &h_white_low, 179,
                     on_h_white_low_change, this);
  cv::createTrackbar("High H", "White Thresholds", &h_white_high, 180,
                     on_h_white_high_change, this);
  cv::createTrackbar("Low S", "White Thresholds", &s_white_low, 255,
                     on_s_white_low_change, this);
  cv::createTrackbar("High S", "White Thresholds", &s_white_high, 255,
                     on_s_white_high_change, this);
  cv::createTrackbar("Low V", "White Thresholds", &v_white_low, 255,
                     on_v_white_low_change, this);
  cv::createTrackbar("High V", "White Thresholds", &v_white_high, 255,
                     on_v_white_high_change, this);
}

void Detection::on_h_1_red_low_change(int value, void* ptr) {
  Detection* objDet = static_cast<Detection*>(ptr);
  objDet->h_red_low_1 = value;
}

void Detection::on_h_1_red_high_change(int value, void* ptr) {
  Detection* objDet = static_cast<Detection*>(ptr);
  objDet->h_red_high_1 = value;
}

void Detection::on_h_2_red_low_change(int value, void* ptr) {
  Detection* objDet = static_cast<Detection*>(ptr);
  objDet->h_red_low_2 = value;
}

void Detection::on_h_2_red_high_change(int value, void* ptr) {
  Detection* objDet = static_cast<Detection*>(ptr);
  objDet->h_red_high_2 = value;
}

// Callback functions for Red Mask
void Detection::on_s_red_low_change(int value, void* ptr) {
  Detection* objDet = static_cast<Detection*>(ptr);
  objDet->s_red_low = value;
}

void Detection::on_s_red_high_change(int value, void* ptr) {
  Detection* objDet = static_cast<Detection*>(ptr);
  objDet->s_red_high = value;
}

void Detection::on_v_red_low_change(int value, void* ptr) {
  Detection* objDet = static_cast<Detection*>(ptr);
  objDet->v_red_low = value;
}

void Detection::on_v_red_high_change(int value, void* ptr) {
  Detection* objDet = static_cast<Detection*>(ptr);
  objDet->v_red_high = value;
}

// Callback functions for White Mask
void Detection::on_h_white_low_change(int value, void* ptr) {
  Detection* objDet = static_cast<Detection*>(ptr);
  objDet->h_white_low = value;
}

void Detection::on_h_white_high_change(int value, void* ptr) {
  Detection* objDet = static_cast<Detection*>(ptr);
  objDet->h_white_high = value;
}

void Detection::on_s_white_low_change(int value, void* ptr) {
  Detection* objDet = static_cast<Detection*>(ptr);
  objDet->s_white_low = value;
}

void Detection::on_s_white_high_change(int value, void* ptr) {
  Detection* objDet = static_cast<Detection*>(ptr);
  objDet->s_white_high = value;
}

void Detection::on_v_white_low_change(int value, void* ptr) {
  Detection* objDet = static_cast<Detection*>(ptr);
  objDet->v_white_low = value;
}

void Detection::on_v_white_high_change(int value, void* ptr) {
  Detection* objDet = static_cast<Detection*>(ptr);
  objDet->v_white_high = value;
}
#pragma endregion HSV Slider Functions
