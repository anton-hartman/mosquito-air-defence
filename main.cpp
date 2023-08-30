#include <signal.h>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include "src/detection/object_detector.hpp"
#include "src/motors/HR8825_driver.hpp"
#include "src/motors/controller.hpp"

const float SCALING_FACTOR = 1.0;
const float ALPHA = 0.01;

// System Exit
void Handler(int signo) {
  printf("\r\nHandler:Motor Stop\r\n");
  driver::stop_all_motors();
  driver::driver_exit();
  cv::destroyAllWindows();
  exit(0);
}

int main(void) {
  // Exception handling: ctrl + c
  signal(SIGINT, Handler);

  // 1.System Initialization
  // if (driver::init_driver_pins())
  //   exit(0);

  cv::VideoCapture cap(
      "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, "
      "format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv "
      "flip-method=2 ! video/x-raw, width=1280, height=720, "
      "format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR "
      "! appsink");

  if (!cap.isOpened()) {
    std::cerr << "Error opening cv video capture" << std::endl;
    return -1;
  }

  cv::Mat frame;
  cap >> frame;
  cv::resize(frame, frame, cv::Size(), SCALING_FACTOR, SCALING_FACTOR);
  ObjectDetector obj_detector(frame, ALPHA);

  std::pair<int, int> laser_pos;
  std::pair<int, int> target_pos = {0, 0};

  int min = 254;
  cv::Scalar lower_threshold = cv::Scalar(min, min, min);
  while (1) {
    cap >> frame;
    if (frame.empty()) {
      std::cout << "End of video file." << std::endl;
      break;
    }
    cv::resize(frame, frame, cv::Size(), SCALING_FACTOR, SCALING_FACTOR);

    // Assuming detectObjects returns bounding boxes for now
    // auto bounding_boxes = obj_detector.detectObjects(frame);
    // laser_pos = obj_detector.detectLaser(frame, 0, 10, 50);
    laser_pos = obj_detector.detectLaserWit(frame, lower_threshold);
    printf("Laser pos: %d, %d\n", laser_pos.first, laser_pos.second);

    cv::imshow("Video", frame);
    char key = static_cast<char>(
        cv::waitKey(1));  // Wait for a keystroke in the window
    if (key == 'q') {
      break;
    }

    // stepper::turret_control(laser_pos, target_pos);
    // delay_ms(3000);
  }

  // 3.System Exit

  return 0;
}
