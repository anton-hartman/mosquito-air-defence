#include <ncurses.h>
#undef OK  // ncurses and opencv have a macro conflict
#include <signal.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <thread>
#include <utility>
#include <vector>
// #include "camera_calibration.cpp"
#include "include/detection.hpp"
#include "include/turret_controller.hpp"
#include "include/utils.hpp"

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

// Laser co-ordinates plus uncertainty in pixels
utils::Circle laser_belief_region_px;
// // Laser co-ordinates detected by camera in pixels
// std::pair<uint16_t, uint16_t> laser_detected_px;
// // Target co-ordinates in pixels
// std::pair<uint16_t, uint16_t> target_px;

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
  // initscr();
  // cbreak();
  // noecho();
  // keypad(stdscr, TRUE);    // Enables arrow key detection
  // nodelay(stdscr, FALSE);  // TRUE = non-blocking, FALSE = blocking
  // timeout(10);             // Set a timeout for getch().

  initscr();             // Initialize the ncurses mode
  cbreak();              // Disable line buffering
  noecho();              // Don't display the pressed key
  keypad(stdscr, TRUE);  // Enable arrow keys
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
    // Directly after capturing a new frame so that it is the belief state at
    // the instance of capturing the frame. Esure frame size remains constant
    // otherwise the belief state will be for the wrong frame size.
    // laser_belief_region_px = turret::get_turret_belief_region();

    if (frame.empty()) {
      std::cout << "frame is empty" << std::endl;
      exit(0);
    }
    cv::resize(frame, frame, cv::Size(), SCALING_FACTOR, SCALING_FACTOR);

    // turret::detected_laser_px.store(
    // detector.detect_laser(frame, laser_belief_region_px));
    // turret::new_feedback.store(true);

    // turret::correct_laser_belief(turret::detected_laser_px);

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
      std::cout << "Processing took longer than expected for a frame. ("
                << loop_duration << "ms)" << std::endl;
      cap >> frame;  // Drop a frame
    }
  }
}

void user_input(void) {
  int ch;
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
    } else if (manual_mode and ch == -1) {
      // -1 is returned when noting is pressed before the timeout period.
      turret::stop_all_motors();
    } else if (manual_mode) {
      turret::keyboard_manual(ch);
    } else {
      turret::keyboard_auto(ch);
    }
  }
}

void turret_horizontal(void) {
  turret::run_stepper(turret::x_stepper);
}

void turret_vertical(void) {
  turret::run_stepper(turret::y_stepper);
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
  std::thread user_input_thread(user_input);
  std::thread video_thread(process_video, std::ref(cap), std::ref(detector));
  std::thread turret_horizontal_thread(turret_horizontal);
  std::thread turret_vertical_thread(turret_vertical);

  // Join the threads (or use detach based on requirements)
  user_input_thread.join();
  video_thread.detach();
  turret_horizontal_thread.detach();
  turret_vertical_thread.detach();

  // calibrate_cam();

  return 0;
}
