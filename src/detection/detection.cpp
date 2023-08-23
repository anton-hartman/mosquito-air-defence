#include <opencv2/opencv.hpp>
#include <vector>

class ObjectDetector {
 private:
  class BackgroundSubtractor {
   public:
    cv::Mat background;
    float alpha;
    cv::Mat subtracted_frame;

    BackgroundSubtractor(const cv::Mat& first_frame, float alpha)
        : alpha(alpha) {
      cv::cvtColor(first_frame, background, cv::COLOR_BGR2GRAY);
      background.convertTo(background, CV_32F);
      // convertBlackToWhite(background);
      subtracted_frame = background.clone();
    }

    /**
     * @brief Used to speed up the background subtraction process in sample
     * footages where the background is pure white.
     */
    void convertBlackToWhite(cv::Mat& frame) {
      for (int i = 0; i < frame.rows; i++) {
        for (int j = 0; j < frame.cols; j++) {
          if (frame.at<float>(i, j) <= 220) {
            frame.at<float>(i, j) = 255.0;
          }
        }
      }
    }

    cv::Mat subtract(const cv::Mat& frame) {
      cv::Mat gray_frame;
      cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
      gray_frame.convertTo(gray_frame, CV_32F);
      background = alpha * gray_frame + (1 - alpha) * background;
      cv::Mat diff = cv::abs(gray_frame - background);
      cv::Mat thresholded_diff;
      cv::threshold(diff, thresholded_diff, 30, 255, cv::THRESH_BINARY);
      return thresholded_diff;
    }
  };

  BackgroundSubtractor bg_subtractor;

 public:
  ObjectDetector(const cv::Mat& first_frame, float alpha = 0.01)
      : bg_subtractor(first_frame, alpha) {}

  std::vector<std::vector<int>> detectBlobs(const cv::Mat& binarized_image) {
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

  std::vector<std::vector<int>> detect_objects(const cv::Mat& frame) {
    cv::Mat subtracted_frame = bg_subtractor.subtract(frame);
    return detectBlobs(subtracted_frame);
  }

  cv::Mat getBackground() const { return bg_subtractor.background; }

  cv::Mat getSubtractedFrame() const { return bg_subtractor.subtracted_frame; }

  void setSubtractedFrame(const cv::Mat& frame) {
    bg_subtractor.subtracted_frame = frame;
  }
};

// int main() {
//   // Example usage
//   cv::Mat first_frame = cv::imread("path_to_image.jpg");
//   ObjectDetector detector(first_frame);
//   cv::Mat frame = cv::imread("path_to_another_image.jpg");
//   auto detected_objects = detector.detectObjects(frame);
//   // ... further processing
//   return 0;
// }
