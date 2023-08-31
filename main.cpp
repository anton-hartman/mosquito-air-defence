#include <signal.h>
#include <sys/select.h>
#include <unistd.h>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include "src/detection/object_detector.hpp"
#include "src/motors/HR8825_driver.hpp"
#include "src/motors/turret_controller.hpp"

const float SCALING_FACTOR = 1.0;
const float ALPHA = 0.01;

cv::VideoCapture cap;
int manual_mode = 1;

void exit_handler(int signo) {
  printf("\r\nSystem exit\r\n");
  // refresh();
  // endwin();  // End ncurses mode
  driver::stop_all_motors();
  driver::driver_exit();
  cv::destroyAllWindows();
  if (cap.isOpened()) {
    cap.release();
  }
  exit(0);
}

cv::VideoCapture init_system(void) {
  // initscr();  // Initialize ncurses mode
  // cbreak();
  // noecho();
  // keypad(stdscr, true);   // Enables arrow key detection
  // nodelay(stdscr, true);  // TRUE = non-blocking, FALSE = blocking

  cv::VideoCapture cap(
      "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, "
      "format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv "
      "flip-method=2 ! video/x-raw, width=1280, height=720, "
      "format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR "
      "! appsink");

  if (!cap.isOpened()) {
    std::cerr << "Error opening cv video capture" << std::endl;
    exit(-1);
  }

  signal(SIGINT, exit_handler);

  if (driver::init_driver_pins())
    exit(0);

  return cap;
}

std::pair<int, int> process_frame(cv::VideoCapture& cap,
                                  ObjectDetector& obj_detector,
                                  cv::Scalar& lower_threshold) {
  cv::Mat frame;
  cap >> frame;
  if (frame.empty()) {
    std::cout << "End of video file." << std::endl;
    exit(0);
  }
  cv::resize(frame, frame, cv::Size(), SCALING_FACTOR, SCALING_FACTOR);

  std::pair<int, int> laser_pos =
      obj_detector.detectLaserWit(frame, lower_threshold);
  // printf("Laser pos: %d, %d\n", laser_pos.first, laser_pos.second);

  cv::imshow("Video", frame);
  char key = static_cast<char>(cv::waitKey(1));
  if (key == 'q') {
    exit(0);
  }

  return laser_pos;
}

bool is_key_pressed() {
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(STDIN_FILENO, &readfds);

  timeval timeout;
  timeout.tv_sec = 0;   // 0 seconds
  timeout.tv_usec = 0;  // 0 microseconds

  int activity = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout);

  return (activity > 0 && FD_ISSET(STDIN_FILENO, &readfds));
}

int main(void) {
  cv::VideoCapture cap = init_system();

  cv::Mat initial_frame;
  cap >> initial_frame;
  cv::resize(initial_frame, initial_frame, cv::Size(), SCALING_FACTOR,
             SCALING_FACTOR);
  ObjectDetector obj_detector(initial_frame, ALPHA);

  std::pair<int, int> target_pos = {0, 0};
  int min = 254;
  cv::Scalar lower_threshold = cv::Scalar(min, min, min);

  char ch;
  while (true) {
    std::pair<int, int> laser_pos =
        process_frame(cap, obj_detector, lower_threshold);

    // ch = getch();
    if (is_key_pressed()) {
      std::cin >> ch;  // Read the pressed key
      std::cout << "Key Pressed: " << ch << std::endl;
      if (ch == 'h' or ch == 'H') {
        manual_mode = !manual_mode;
        std::cout << "Manual mode: " << manual_mode << std::flush;
      }

      if (manual_mode) {
        turret::manual_control(ch);
      } else {
        turret::auto_control(laser_pos, {0, 0});
      }
    }
  }
  return 0;
}
