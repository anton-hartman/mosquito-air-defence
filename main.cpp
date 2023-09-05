#include <ncurses.h>
#undef OK  // ncurses and opencv have a macro conflict
#include <signal.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <thread>
#include <utility>
#include <vector>
#include "camera_calibration.cpp"
#include "include/detection.hpp"
#include "include/turret_controller.hpp"
#include "include/utils.hpp"

std::mutex mtx;

const float SCALING_FACTOR = 1.0;
const float ALPHA = 0.01;
const float FRAME_TIME_MS = 1000 / 30.0;
const std::vector<float> CAMERA_MATRIX = {647.0756309728268,
                                          0,
                                          304.4404590127848,
                                          0,
                                          861.7363873209705,
                                          257.5858878142162,
                                          0,
                                          0,
                                          1};
const float CAMERA_DEPTH = 1104;  // mm

cv::VideoCapture cap;
int manual_mode = 1;
// std::pair<int, int> target_angle = {500, 300};
// std::pair<int, int> laser_angle;

// Data shared accross threads (use mutex to ensure safe access)
// Laser co-ordinates plus uncertainty in pixels
utils::Circle laser_belief_region_px;
// Laser co-ordinates detected by camera in pixels
utils::Point laser_detected_px;
// Target co-ordinates in pixels
utils::Point target_px;

// struct SharedData {
//   // Updated by turret thread, read by detection thread
//   std::pair<int, int> counted_laser_position;
//   // Updated by detection thread, read by turret thread
//   std::pair<int, int> detected_laser_position;
//   // Updated by detection thread, read by turret thread
//   std::pair<int, int> target_position;
//   // Flag to indicate when a new frame is ready for processing
//   bool new_frame_ready = false;
// };

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

  while (true) {
    std::chrono::high_resolution_clock::time_point loop_start_time =
        std::chrono::high_resolution_clock::now();

    cap >> frame;
    {
      std::lock_guard<std::mutex> lock(mtx);
      laser_belief_region_px = turret::get_laser_belief_region();
    }

    if (frame.empty()) {
      std::cout << "frame is empty" << std::endl;
      exit(0);
    }
    cv::resize(frame, frame, cv::Size(), SCALING_FACTOR, SCALING_FACTOR);

    {
      std::lock_guard<std::mutex> lock(mtx);
      laser_detected_px = detector.detect_laser(frame, laser_belief_region_px);
      turret::correct_laser_belief_region(laser_detected_px);
    }

    std::chrono::high_resolution_clock::time_point loop_end_time =
        std::chrono::high_resolution_clock::now();
    uint32_t loop_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time -
                                                              loop_start_time)
            .count();

    if (loop_duration < FRAME_TIME_MS) {
      // You have additional time before the next frame is expected.
      // This can be used as idle time or for other tasks.

      // std::this_thread::sleep_for(std::chrono::milliseconds(
      //     static_cast<int>(FRAME_TIME_MS - loop_duration)));
    } else {
      std::cout << "Processing took longer than expected for a frame."
                << std::endl;
      // Processing took longer than expected for a frame.
      // Consider dropping frames or optimizing your processing.
    }
  }
}

void turret_control(void) {
  int ch;
  std::string state = manual_mode ? "Manual" : "Auto";
  init_ncurses();
  while (true) {
    // std::lock_guard<std::mutex> lock(mtx);
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
    } else {
      if (manual_mode) {
        // turret::manual_control(ch);
      } else {
        switch (ch) {
          case KEY_UP:
            target_px.y -= 100;
            break;
          case KEY_DOWN:
            target_px.y += 100;
            break;
          case KEY_LEFT:
            target_px.x -= 100;
            break;
          case KEY_RIGHT:
            target_px.x += 100;
            break;
          default:
            break;
        }
        // target_px =
        //     pixel_to_mm(target_px, CAMERA_MATRIX, CAMERA_DEPTH);
        // target_px = {std::atan2(target_px.x, CAMERA_DEPTH),
        //                       std::atan2(target_px.y,
        //                       CAMERA_DEPTH)};
        // Assuming laser_angle and target_angle are global/shared variables,
        // ensure safe access using a mutex or other synchronization
        // mechanisms.
        // turret::auto_control();
      }
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

  // Launch the threads
  std::thread video_thread(process_video, std::ref(cap), std::ref(detector));
  std::thread turret_thread(turret_control);

  // Join the threads (or use detach based on requirements)
  video_thread.join();
  turret_thread.join();

  // calibrate_cam();

  return 0;
}
