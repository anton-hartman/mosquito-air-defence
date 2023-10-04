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

cv::VideoCapture cap;
cv::Mat frame;
int manual_mode = 1;

// Laser co-ordinates plus uncertainty in pixels
utils::Circle laser_belief_region_px;
std::pair<int32_t, int32_t> laser_pos;

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

// void convert_to_red_frame(const cv::Mat& frame, uint8_t* red_frame) {
//   // Check if the input Mat is of the expected size and type
//   if (frame.rows != ROWS || frame.cols != COLS || frame.type() != CV_8UC3) {
//     std::cerr << "Invalid input Mat dimensions or type!" << std::endl;
//     throw std::runtime_error("Invalid input Mat");
//   }

//   for (int i = 0; i < ROWS; ++i) {
//     for (int j = 0; j < COLS; ++j) {
//       // cv::Vec3b pixel = frame.at<cv::Vec3b>(i, j);
//       // // Extracting the red channel (assuming BGR format)
//       // red_frame[i * COLS + j] = pixel[2];

//       // cv::Vec3b pixel = frame.at<cv::Vec3b>(i, j);
//       // Extracting the red channel (assuming BGR format)
//       red_frame[i * COLS + j] = frame.at<cv::Vec3b>(i, j)[2];
//     }
//   }
// }

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

void process_video(cv::VideoCapture& cap, Detection& detector) {
  std::vector<cv::Mat> channels;
  cv::Mat red_channel;

  // uint8_t* red_frame = new uint8_t[COLS * ROWS];
  double total_duration = 0;
  double avg_duration = 0;
  uint32_t loop_count = 0;

  gpu::init_gpu();

  std::cout << "ROWS: " << ROWS << std::endl;
  std::cout << "COLS: " << COLS << std::endl;

  while (!utils::exit_flag.load()) {
    // loop_count++;
    auto start_time = std::chrono::high_resolution_clock::now();

    cap >> frame;
    turret.save_steps_at_frame();
    if (frame.empty()) {
      std::cout << "Frame is empty, exiting." << std::endl;
      exit(0);
    }
    cv::split(frame, channels);
    red_channel = channels[2];
    // std::cout << "Frame size: " << red_channel.size() << std::endl;

    // cv::Mat undistorted_frame;
    // cv::undistort(frame, undistorted_frame, CAMERA_MATRIX, DIST_COEFFS);
    // Look into converting from steps to pixels for laser belief region. How
    // must distorition be accounted for?

    laser_pos = gpu::detect_laser(red_channel, 230);
    // convert_to_red_frame(frame, red_frame);
    // laser_pos = gpu::detect_laser(red_frame, 230);
    if (enable_feedback_flag.load()) {
      turret.update_belief(laser_pos);
    }

    utils::draw_target(frame, {X_ORIGIN_PX, Y_ORIGIN_PX},
                       cv::Scalar(0, 255, 255));
    utils::put_label(frame, "Camera Origin", {X_ORIGIN_PX, Y_ORIGIN_PX}, 0.5);
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

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                        end_time - start_time)
                        .count();

    cv::putText(frame, "FPS: " + std::to_string(1000 / duration),
                cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

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
    }
  }

  std::cout << "Video processing out of main loop" << std::endl;

  // delete[] red_frame;
  gpu::free_gpu();

  avg_duration = total_duration / loop_count;
  std::cout << "Average duration of kernel = " << avg_duration << "us"
            << std::endl;
}

void user_input(void) {
  bool turret_stopped = true;

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

    // if (ch == 'b') {
    //   std::pair<cv::Point, cv::Point> points = get_bounding_box();
    //   gpu::set_ignore_region({points.first.x, points.first.y},
    //                          {points.second.x, points.second.y});

    //   std::cout << "Top left point: (" << points.first.x << ", "
    //             << points.first.y << ")\n";
    //   std::cout << "Bottom right point: (" << points.second.x << ", "
    //             << points.second.y << ")\n";
    // } else
    if (turret_stopped and
        (ch == 'e' or ch == 'w' or ch == 'a' or ch == 's' or ch == 'd')) {
      turret_stopped = false;
      turret.start_turret();
      std::cout << "Turret started" << std::endl;
    } else if (!turret_stopped and ch == 'e') {
      turret_stopped = true;
      turret.stop_turret();
      std::cout << "Turret stopped" << std::endl;
    } else if (ch == 'o') {
      turret.set_origin(laser_pos);
      std::cout << "Origin set to: " << laser_pos.first << ", "
                << laser_pos.second << std::endl;
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
