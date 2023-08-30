#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "detection/object_detector.hpp"

// Mosquito class
class Mosquito {
 public:
  int id;
  cv::Point2f centroid;
  int frames_missing;
  bool matched;

  Mosquito(int mosquito_id, cv::Point2f centroid)
      : id(mosquito_id), centroid(centroid), frames_missing(0), matched(true) {}

  void update(cv::Point2f newCentroid) {
    centroid = newCentroid;
    frames_missing = 0;
    matched = true;
  }

  friend std::ostream& operator<<(std::ostream& os, const Mosquito& mosquito) {
    os << "Mosquito " << mosquito.id << " at (" << mosquito.centroid.x << ", "
       << mosquito.centroid.y << ")";
    return os;
  }
};

class VideoInterface {
 private:
  double frame_resize_factor;
  bool darkmode;
  bool comparison;

 public:
  VideoInterface(double frame_resize_factor = 1.0,
                 bool darkmode = false,
                 bool comparison = false)
      : frame_resize_factor(frame_resize_factor),
        darkmode(darkmode),
        comparison(comparison) {}

  //   void mosquito_based(cv::Mat& frame,
  //                       const std::vector<Mosquito>& tracked_mosquitoes) {
  //     for (const auto& mosquito : tracked_mosquitoes) {
  //       cv::Point2f centroid = mosquito.centroid;
  //       cv::Scalar color =
  //           mosquito.matched ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
  //       cv::circle(frame, centroid, 5, color, 2);
  //       cv::putText(frame, std::to_string(mosquito.id),
  //                   centroid - cv::Point2f(0, 10), cv::FONT_HERSHEY_SIMPLEX,
  //                   0.5, color, 2);
  //     }
  //   }

  //   void bbox_based(cv::Mat& frame,
  //                   const std::vector<cv::Rect2i>& track_bbs_ids) {
  //     cv::Scalar green(0, 255, 0);
  //     for (const auto& bb : track_bbs_ids) {
  //       cv::rectangle(frame, bb.tl(), bb.br(), green, 2);
  //       // Assuming you have an ID or similar property in your cv::Rect2i or
  //       // another data structure cv::putText(frame, std::to_string(bb.id),
  //       // bb.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, green, 2);
  //     }
  //   }

  void start_feed(const std::string& video_path) {
    // cv::VideoCapture cap(video_path);
    cv::VideoCapture cap(
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, "
        "format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv "
        "flip-method=2 ! video/x-raw, width=1280, height=720, "
        "format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR "
        "! appsink");

    if (!cap.isOpened()) {
      // std::cerr << "Failed to read the video file." << std::endl;
      return;
    }

    int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    int fps = static_cast<int>(cap.get(cv::CAP_PROP_FPS));
    std::cout << "FPS: " << fps << std::endl;

    cv::Mat frame;
    int frame_index = 0;

    // Assuming ObjectDetector is a C++ class similar to the Python version
    cap >> frame;
    cv::resize(frame, frame, cv::Size(), frame_resize_factor,
               frame_resize_factor);
    ObjectDetector obj_detector(frame, 0.01);

    while (true) {
      cap >> frame;
      if (frame.empty()) {
        std::cout << "End of video file." << std::endl;
        break;
      }
      cv::resize(frame, frame, cv::Size(), frame_resize_factor,
                 frame_resize_factor);

      // Assuming detectObjects returns bounding boxes for now
      auto bounding_boxes = obj_detector.detectObjects(frame);

      // Display the results
      //   mosquito_based(frame, tracked_mosquitoes);
      //   bbox_based(frame, track_bbs_ids);

      // ... (rest of the processing, similar to the Python code but translated
      // to C++ syntax)

      cv::imshow("Video", frame);
      char key = static_cast<char>(
          cv::waitKey(1));  // Wait for a keystroke in the window
      if (key == 'q') {
        break;
      }

      frame_index++;
    }

    cap.release();
    cv::destroyAllWindows();
  }
};
