#include <cuda_runtime.h>
#include <signal.h>
#include <atomic>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include "include/detection.hpp"
#include "include/frame.hpp"
#include "include/image_processing.hpp"
#include "include/turret.hpp"
#include "include/utils.hpp"

Turret turret;

std::atomic<bool> utils::run_flag(true);
std::atomic<bool> utils::exit_flag(false);
std::atomic<bool> enable_feedback_flag(false);
std::atomic<bool> mos_detection_flag(false);

cv::VideoCapture cap;
cv::Mat frame;

int manual_mode = 1;
bool turret_stopped = true;

// Laser co-ordinates plus uncertainty in pixels
utils::Circle laser_belief_region_px;
std::pair<int32_t, int32_t> laser_pos_px;
std::vector<Pt> mosquitoes_px;

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
  std::cout << "Using pipeline: \n\t" << pipeline << std::endl;
  cv::VideoCapture cap(pipeline);

  if (!cap.isOpened()) {
    std::cerr << "Error opening cv video capture" << std::endl;
    exit(-1);
  }

  signal(SIGINT, exit_handler);
  return cap;
}

bool save_frame_as_jpeg(const cv::Mat& frame, int& counter) {
  if (frame.empty()) {
    std::cerr << "The frame is empty, cannot save it." << std::endl;
    return false;
  }
  std::string file_name = "../cal_imgs/img" + std::to_string(counter) + ".jpg";
  if (!cv::imwrite(file_name, frame)) {
    std::cerr << "Failed to save the frame." << std::endl;
    return false;
  }
  std::cout << "Saved the frame to " << file_name << std::endl;
  counter++;
  return true;
}

// Global variables
bool drawing = false;
cv::Point top_left_pt, bottom_right_pt;

// Mouse callback function
void draw_rectangle(int event, int x, int y, int flags, void* param) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    drawing = true;
    top_left_pt = cv::Point(x, y);
  } else if (event == cv::EVENT_LBUTTONUP) {
    drawing = false;
    bottom_right_pt = cv::Point(x, y);
    cv::rectangle(*static_cast<cv::Mat*>(param), top_left_pt, bottom_right_pt,
                  cv::Scalar(0, 255, 0), 2);
  }
}

std::pair<cv::Point, cv::Point> get_bounding_box() {
  // Make a deep copy of the global frame
  cv::Mat local_frame = frame.clone();

  cv::namedWindow("Draw a rectangle");

  // Assign draw_rectangle function to mouse callback
  cv::setMouseCallback("Draw a rectangle", draw_rectangle, &local_frame);

  while (true) {
    cv::imshow("Draw a rectangle", local_frame);

    // Exit when 'Esc' is pressed
    if (cv::waitKey(1) == 27)
      break;
  }

  cv::destroyAllWindows();

  return std::make_pair(top_left_pt, bottom_right_pt);
}

void markup_frame() {
  cv::rectangle(frame,
                cv::Point(gpu::ignore_region_top_left.first,
                          gpu::ignore_region_top_left.second),
                cv::Point(gpu::ignore_region_bottom_right.first,
                          gpu::ignore_region_bottom_right.second),
                cv::Scalar(0, 255, 0), 2);
  utils::draw_target(frame, {C_X_DOUBLE, C_Y_DOUBLE}, cv::Scalar(0, 255, 255));
  utils::put_label(frame, "Camera Origin", {C_X_DOUBLE, C_Y_DOUBLE}, 0.5);
  utils::draw_target(frame, turret.get_origin_px(), cv::Scalar(0, 255, 0));
  utils::put_label(frame, "Turret Origin", turret.get_origin_px(), 0.5);
  utils::draw_target(frame, laser_pos_px, cv::Scalar(0, 0, 255));
  utils::put_label(frame, "Detected laser", laser_pos_px, 0.5);
  utils::draw_target(frame, turret.get_belief_px(), cv::Scalar(255, 0, 255));
  utils::put_label(frame, "Belief", turret.get_belief_px(), 0.5);
  utils::draw_target(frame, turret.get_setpoint_px(), cv::Scalar(255, 0, 0));
  std::pair<uint16_t, uint16_t> set_pt = turret.get_setpoint_px();
  utils::put_label(frame,
                   "Setpoint (" + std::to_string(set_pt.first) + ", " +
                       std::to_string(set_pt.second) + ")",
                   std::pair<uint16_t, uint16_t>(10, 90), 0.5);
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

  if (turret_stopped) {
    cv::putText(frame, "Turret Disabled", cv::Point(COLS - 150, 50),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
                cv::LINE_AA);
  } else {
    cv::putText(frame, "Turret Enabled", cv::Point(COLS - 150, 50),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1,
                cv::LINE_AA);
  }
  if (!mos_detection_flag.load()) {
    if (manual_mode) {
      cv::putText(frame, "Keyboard = Manual", cv::Point(COLS - 150, 80),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
                  cv::LINE_AA);
    } else {
      cv::putText(frame, "Keyboard = Auto", cv::Point(COLS - 150, 80),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1,
                  cv::LINE_AA);
    }
  } else {
    cv::putText(frame, "Keyboard = OFF", cv::Point(COLS - 150, 80),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 100, 100), 1,
                cv::LINE_AA);
  }
  if (enable_feedback_flag.load()) {
    cv::putText(frame, "Feedback Enabled", cv::Point(COLS - 150, 110),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1,
                cv::LINE_AA);
  } else {
    cv::putText(frame, "Feedback Disabled", cv::Point(COLS - 150, 110),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
                cv::LINE_AA);
  }
  if (mos_detection_flag.load()) {
    cv::putText(frame, "Mos Detection = ON", cv::Point(COLS - 150, 130),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1,
                cv::LINE_AA);
  } else {
    cv::putText(frame, "Mos Detection = OFF", cv::Point(COLS - 150, 130),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1,
                cv::LINE_AA);
  }
}

void process_video(cv::VideoCapture& cap, Detection& detector) {
  std::vector<cv::Mat> channels;
  cv::Mat red_channel;

  gpu::init_gpu();

  std::cout << "ROWS: " << ROWS << std::endl;
  std::cout << "COLS: " << COLS << std::endl;

  bool save_img = false;
  int save_counter = 0;

  while (!utils::exit_flag.load()) {
    auto start_time = std::chrono::high_resolution_clock::now();

    cap >> frame;
    turret.save_steps_at_frame();
    if (frame.empty()) {
      std::cout << "Frame is empty, exiting." << std::endl;
      exit(0);
    }

    if (save_img) {
      save_frame_as_jpeg(frame, save_counter);
      save_img = false;
    }

    cv::split(frame, channels);
    red_channel = channels[2];

    // cv::Mat undistorted_frame;
    // cv::undistort(frame, undistorted_frame, CAMERA_MATRIX, DIST_COEFFS);
    // Look into converting from steps to pixels for laser belief region. How
    // must distorition be accounted for?

    laser_pos_px = gpu::detect_laser(red_channel, 230);
    if (enable_feedback_flag.load()) {
      turret.update_belief(laser_pos_px);
    }
    if (mos_detection_flag.load()) {
      mosquitoes_px = gpu::detect_mosquitoes(red_channel, 100);
      turret.update_setpoint({mosquitoes_px.at(0).x, mosquitoes_px.at(0).y});
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                        end_time - start_time)
                        .count();

    cv::putText(frame,
                "FPS: " + std::to_string(1000 / duration) + " (" +
                    std::to_string(duration) + " ms)",
                cv::Point(COLS - 150, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

    markup_frame();

    cv::imshow("frame", frame);
    char key = static_cast<char>(cv::waitKey(1));
    if (key == 'q') {
      utils::exit_flag.store(true);
    } else if (key == 'b') {
      std::pair<cv::Point, cv::Point> points = get_bounding_box();
      gpu::set_ignore_region({points.first.x, points.first.y},
                             {points.second.x, points.second.y});

      std::cout << "Top left point: (" << points.first.x << ", "
                << points.first.y << ")\n";
      std::cout << "Bottom right point: (" << points.second.x << ", "
                << points.second.y << ")\n";
    } else if (key == 's') {
      save_img = true;
    }
  }

  std::cout << "Video processing out of main loop" << std::endl;
  gpu::free_gpu();
}

void user_input(void) {
  std::cout << "Turret disabled" << std::endl;

  bool adjust_size = false;
  int steps = 105;
  int px = 110;
  char ch;
  while (!utils::exit_flag.load()) {
    std::cout << "Enter a character: ";
    std::cin >> ch;  // Read a character from standard input

    if (ch == 'q') {
      std::cout << "Exit with keypress = q" << std::endl;
      utils::exit_flag.store(true);
      break;
    }

    if (ch == 'k') {
      mos_detection_flag.store(!mos_detection_flag.load());
      std::cout << "Mosquito detection: " << mos_detection_flag.load()
                << std::endl;
    } else if (ch == 'h') {
      turret.update_belief(laser_pos_px);
      turret.update_setpoint(laser_pos_px);
    } else if (turret_stopped and (ch == 'e' or ch == 'w' or ch == 'a' or
                                   ch == 's' or ch == 'd')) {
      turret_stopped = false;
      turret.start_turret();
      std::cout << "Turret started" << std::endl;
    } else if (!turret_stopped and ch == 'e') {
      turret_stopped = true;
      turret.stop_turret();
      std::cout << "Turret stopped" << std::endl;
    } else if (ch == 'o') {
      turret.set_origin(laser_pos_px);
      std::cout << "Origin set to: " << laser_pos_px.first << ", "
                << laser_pos_px.second << std::endl;
    } else if (ch == 'm') {
      utils::run_flag.store(false);
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
      utils::run_flag.store(true);
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
    } else if (!mos_detection_flag.load()) {
      if (manual_mode) {
        turret.keyboard_manual(ch, steps);
      } else {
        turret.keyboard_auto(ch, px);
      }
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
  float alpha = 0.1;
  Detection detector(initial_frame, alpha);

  // Launch the threads
  std::thread user_input_thread(user_input);
  std::thread video_thread(process_video, std::ref(cap), std::ref(detector));
  std::thread turret_horizontal_thread(turret_horizontal);
  std::thread turret_vertical_thread(turret_vertical);

  std::cout << "Threads launched" << std::endl;
  // Join the threads (or use detach based on requirements)
  video_thread.join();
  std::cout << "Video thread joined" << std::endl;
  turret_horizontal_thread.join();
  std::cout << "Turret horizontal thread joined" << std::endl;
  turret_vertical_thread.join();
  std::cout << "Turret vertical thread joined" << std::endl;
  user_input_thread.detach();

  cudaDeviceReset();
  exit(0);

  return 0;
}
