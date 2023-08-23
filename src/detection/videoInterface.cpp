#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

// Mosquito class
class Mosquito {
 private:
  int id;
  cv::Point2f centroid;
  int frames_missing;
  bool matched;

 public:
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

// VideoInterface class
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

  // ... (other methods like mosquito_based, bbox_based, etc.)

  void start_feed(std::string video_path) {
    cv::VideoCapture cap(video_path);
    int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    int fps = static_cast<int>(cap.get(cv::CAP_PROP_FPS));
    std::cout << "FPS: " << fps << std::endl;

    // ... (rest of the method, similar to the Python code but translated to C++
    // syntax)

    // Note: The Python specific parts like list comprehensions, numpy
    // operations, etc. will need their corresponding C++/OpenCV translations.
  }
};

