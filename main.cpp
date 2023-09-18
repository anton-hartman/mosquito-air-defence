#include <signal.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <thread>
#include <utility>
#include <vector>
// #include "camera_calibration.cpp"
#include <atomic>
#include "include/detection.hpp"
#include "include/turret.hpp"
#include "include/utils.hpp"

Turret turret;

std::atomic<bool> utils::exit_flag(false);
std::atomic<bool> enable_feedback_flag(false);

const float SCALING_FACTOR = 1.0;
const float ALPHA = 0.01;
const float FRAME_TIME_MS = 1000 / 24.0;
// const std::vector<float> CAMERA_MATRIX = {647.0756309728268,
//                                           0,
//                                           304.4404590127848,
//                                           0,
//                                           861.7363873209705,
//                                           257.5858878142162,
//                                           0,
//                                           0,
//                                           1};
// const float CAMERA_DEPTH = 1104;  // mm

cv::VideoCapture cap;
int manual_mode = 1;

// Laser co-ordinates plus uncertainty in pixels
utils::Circle laser_belief_region_px;
std::pair<uint16_t, uint16_t> laser_pos;

void exit_handler(int signo) {
  printf("\r\nSystem exit\r\n");
  turret.stop_turret();
  cv::destroyAllWindows();
  if (cap.isOpened()) {
    cap.release();
  }
  utils::exit_flag.store(true);
}

cv::VideoCapture init_system(void) {
  cv::VideoCapture cap(
      "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, "
      "format=(string)NV12, framerate=(fraction)24/1 ! nvvidconv "
      "flip-method=2 ! video/x-raw, width=1280, height=720, "
      "format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR "
      "! appsink");

  if (!cap.isOpened()) {
    std::cerr << "Error opening cv video capture" << std::endl;
    exit(-1);
  }

  signal(SIGINT, exit_handler);
  return cap;
}

void process_video(cv::VideoCapture& cap, Detection& detector) {
  cv::Mat frame;

  while (true) {
    std::chrono::high_resolution_clock::time_point loop_start_time =
        std::chrono::high_resolution_clock::now();

    cap >> frame;
    turret.save_steps_at_frame();
    // Directly after capturing a new frame so that it is the belief state at
    // the instance of capturing the frame. Esure frame size remains constant
    // otherwise the belief state will be for the wrong frame size.
    // laser_belief_region_px = turret.get_belief_region();

    if (frame.empty()) {
      std::cout << "Frame is empty, exiting." << std::endl;
      exit(0);
    }
    // cv::resize(frame, frame, cv::Size(), SCALING_FACTOR, SCALING_FACTOR);
    laser_pos = detector.detect_laser(frame, laser_belief_region_px);
    if (enable_feedback_flag.load()) {
      turret.update_belief(laser_pos);
      // enable_feedback_flag.store(false);
      // std::cout << "Feedback off" << std::endl;
    }

    utils::draw_target(frame, turret.get_origin_px(), cv::Scalar(0, 255, 0));
    utils::put_label(frame, "Origin", turret.get_origin_px(), 0.5);
    utils::draw_target(frame, laser_pos, cv::Scalar(0, 0, 255));
    utils::put_label(frame, "Detected laser", laser_pos, 0.5);
    utils::draw_target(frame, turret.get_setpoint_px(), cv::Scalar(255, 0, 0));
    utils::put_label(frame, "Setpoint", turret.get_setpoint_px(), 0.5);
    utils::draw_target(frame, turret.get_belief_px(), cv::Scalar(255, 0, 255));
    utils::put_label(frame, "Belief", turret.get_belief_px(), 0.5);
    std::pair<int32_t, int32_t> current_steps = turret.get_belief_steps();
    utils::put_label(frame,
                     "Belief steps (" + std::to_string(current_steps.first) +
                         ", " + std::to_string(current_steps.second) + ")",
                     std::pair<uint16_t, uint16_t>(10, 30), 0.5);
    std::pair<int32_t, int32_t> target_steps = turret.get_setpoint_steps();
    utils::put_label(frame,
                     "Target steps (" + std::to_string(target_steps.first) +
                         ", " + std::to_string(target_steps.second) + ")",
                     std::pair<uint16_t, uint16_t>(10, 60), 0.5);

    std::chrono::high_resolution_clock::time_point loop_end_time =
        std::chrono::high_resolution_clock::now();
    uint32_t loop_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time -
                                                              loop_start_time)
            .count();

    cv::imshow("frame", frame);
    char key = static_cast<char>(cv::waitKey(1));
    if (key == 'q') {
      utils::exit_flag.store(true);
    }

    if (loop_duration < FRAME_TIME_MS) {
      // You have additional time before the next frame is expected.
      // This can be used as idle time or for other tasks.

      // std::this_thread::sleep_for(std::chrono::milliseconds(
      //     static_cast<int>(FRAME_TIME_MS - loop_duration)));
    } else {
      // std::cout << "Processing took longer than expected for a frame. ("
      //           << loop_duration << "ms)" << std::endl;
      cap >> frame;  // Drop a frame
    }
  }
}

void user_input(void) {
  bool turret_stopped = false;
  bool adjust_size = false;
  int steps = 105;
  int px = 110;
  char ch;
  while (!utils::exit_flag.load()) {
    std::cout << "Enter a character: ";
    std::cin >> ch;  // Read a character from standard input

    if (ch == 'e') {
      if (turret_stopped) {
        turret_stopped = false;
        turret.start_turret();
        std::cout << "Turret started" << std::endl;
      } else {
        turret_stopped = true;
        turret.stop_turret();
        std::cout << "Turret stopped" << std::endl;
      }
    } else if (ch == 'o') {
      turret.set_origin(laser_pos);
      std::cout << "Origin set to: " << laser_pos.first << ", "
                << laser_pos.second << std::endl;
    } else if (ch == 'm') {
      manual_mode = !manual_mode;
      if (manual_mode) {
        turret.set_manual_mode(true);
        std::cout << "Manual" << std::endl;
        enable_feedback_flag.store(false);
        std::cout << "Feedback off" << std::endl;

      } else {
        turret.set_manual_mode(false);
        std::cout << "Auto" << std::endl;
      }
    } else if (ch == 'f') {
      enable_feedback_flag.store(!enable_feedback_flag.load());
      if (enable_feedback_flag.load()) {
        std::cout << "Feedback on" << std::endl;
      } else {
        std::cout << "Feedback off" << std::endl;
      }
    } else if (ch == 'c') {
      adjust_size = !adjust_size;
    } else if (manual_mode and adjust_size) {
      if (ch == 'w') {
        steps += 20;
      } else if (ch == 's') {
        steps -= 20;
      }
      std::cout << "Steps per click: " << steps << std::endl;
    } else if (!manual_mode and adjust_size) {
      if (ch == 'w') {
        px += 100;
      } else if (ch == 's') {
        px -= 100;
      }
      std::cout << "Pixels per click: " << steps << std::endl;
    } else if (manual_mode) {
      turret.keyboard_manual(ch, steps);
    } else {
      turret.keyboard_auto(ch, px);
    }
  }
}

void turret_horizontal(void) {
  turret.run_x_stepper();
}

void turret_vertical(void) {
  turret.run_y_stepper();
}

int main(void) {
  cv::VideoCapture cap = init_system();
  cv::Mat initial_frame;
  cap >> initial_frame;
  // cv::resize(initial_frame, initial_frame, cv::Size(), SCALING_FACTOR,
  //            SCALING_FACTOR);
  Detection detector(initial_frame, ALPHA);
  detector.set_red_thresholds(0, 170, 50, 190, 10, 180, 255, 255);
  detector.set_white_thresholds(0, 0, 245, 180, 20, 255);
  detector.create_threshold_trackbars();

  // Launch the threads
  std::thread user_input_thread(user_input);
  std::thread video_thread(process_video, std::ref(cap), std::ref(detector));
  std::thread turret_horizontal_thread(turret_horizontal);
  std::thread turret_vertical_thread(turret_vertical);

  // Join the threads (or use detach based on requirements)
  video_thread.detach();
  turret_horizontal_thread.join();
  turret_vertical_thread.join();
  user_input_thread.detach();

  // calibrate_cam();

  exit(0);

  return 0;
}
