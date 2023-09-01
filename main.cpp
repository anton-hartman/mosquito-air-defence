

#include <ncurses.h>
#undef OK  // ncurses and opencv have a macro conflict
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include "include/detection.hpp"
#include "include/turret_controller.hpp"
#include "include/utils.hpp"

const float SCALING_FACTOR = 1.0;
const float ALPHA = 0.01;

cv::VideoCapture cap;
int manual_mode = 1;

void exit_handler(int signo) {
  printf("\r\nSystem exit\r\n");
  refresh();  // Also something with ncurses
  endwin();   // End ncurses mode
  turret::stop_all_motors();
  cv::destroyAllWindows();
  if (cap.isOpened()) {
    cap.release();
  }
  exit(0);
}

void init_ncurses(void) {
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);    // Enables arrow key detection
  nodelay(stdscr, FALSE);  // TRUE = non-blocking, FALSE = blocking
  timeout(10);             // Set a timeout for getch().
}

cv::VideoCapture init_system(void) {
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
  turret::init();
  return cap;
}

void process_video(cv::VideoCapture& cap, Detection& detector) {
  cv::Mat frame;
  std::pair<int, int> laser_pos;

  while (true) {
    cap >> frame;
    if (frame.empty()) {
      std::cout << "frame is empty" << std::endl;
      exit(0);
    }
    cv::resize(frame, frame, cv::Size(), SCALING_FACTOR, SCALING_FACTOR);
    laser_pos = detector.detect_laser(frame);

    // Add delay here to adjust frame rate if necessary
    // std::this_thread::sleep_for(std::chrono::milliseconds(delayTime));
  }
}

void turret_control(std::pair<int, int>& laser_pos,
                    std::pair<int, int>& target_pos) {
  int ch;
  std::string state = manual_mode ? "Manual" : "Auto";
  init_ncurses();
  while (true) {
    ch = getch();
    if (ch == 'm') {
      manual_mode = !manual_mode;
      if (manual_mode) {
        std::cout << "Manual" << std::endl;
      } else {
        std::cout << "Auto" << std::endl;
      }
    } else if (ch == 'h') {
      turret::home_steppers();
      std::cout << "Stepper homed" << std::endl;
    } else if (manual_mode and ch == -1) {
      // -1 is returned when noting is pressed before the timeout period.
      turret::stop_all_motors();
    }

    if (manual_mode) {
      turret::manual_control(ch);
    } else {
      // Assuming laser_pos and target_pos are global/shared variables,
      // ensure safe access using a mutex or other synchronization mechanisms.
      turret::auto_control(laser_pos, target_pos);
    }
  }
}

int main(void) {
  cv::VideoCapture cap = init_system();
  cv::Mat initial_frame;
  cap >> initial_frame;
  cv::resize(initial_frame, initial_frame, cv::Size(), SCALING_FACTOR,
             SCALING_FACTOR);
  Detection detector(initial_frame, ALPHA);
  detector.set_red_thresholds(0, 50, 190, 10, 255, 255);
  detector.set_white_thresholds(0, 0, 245, 180, 20, 255);
  detector.create_threshold_trackbars();

  std::pair<int, int> target_pos = {500, 300};
  std::pair<int, int> laser_pos;
  cv::Mat frame;

  // Launch the threads
  std::thread video_thread(process_video, std::ref(cap), std::ref(detector));
  std::thread turret_thread(turret_control, std::ref(laser_pos),
                            std::ref(target_pos));

  // Join the threads (or use detach based on requirements)
  video_thread.join();
  turret_thread.join();

  return 0;
}
