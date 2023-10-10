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

std::atomic<bool> utils::manual_mode(true);
std::atomic<bool> utils::run_flag(true);
std::atomic<bool> utils::exit_flag(false);
std::atomic<bool> enable_feedback_flag(false);
std::atomic<bool> mos_detection_flag(false);

cv::VideoCapture cap;
cv::Mat frame;
cv::Mat red_channel;
float learning_rate = 0.1;

bool turret_stopped = true;

int laser_threshold = 230;
bool mos_bg_sub = true;
int mos_threshold = 30;

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
  cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

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
  cv::putText(frame, "Camera Origin", cv::Point(C_X_DOUBLE, C_Y_DOUBLE),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1,
              cv::LINE_AA);
  utils::draw_target(frame, turret.get_origin_px(), cv::Scalar(0, 255, 0));
  cv::putText(
      frame, "Turret Origin",
      cv::Point(turret.get_origin_px().first, turret.get_origin_px().second),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  utils::draw_target(frame, laser_pos_px, cv::Scalar(0, 0, 255));
  cv::putText(frame, "Detected Laser",
              cv::Point(laser_pos_px.first, laser_pos_px.second + 20),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1,
              cv::LINE_AA);
  utils::draw_target(frame, turret.get_belief_px(), cv::Scalar(255, 0, 255));
  cv::putText(
      frame, "Belief",
      cv::Point(turret.get_belief_px().first, turret.get_belief_px().second),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  utils::draw_target(frame, turret.get_setpoint_px(), cv::Scalar(255, 0, 0));

  std::pair<int32_t, int32_t> current_steps = turret.get_belief_steps();
  cv::putText(frame,
              "Belief steps (" + std::to_string(current_steps.first) + ", " +
                  std::to_string(current_steps.second) + ")",
              cv::Point(COLS - 230, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  std::pair<int32_t, int32_t> target_steps = turret.get_setpoint_steps();
  cv::putText(frame,
              "Target steps (" + std::to_string(target_steps.first) + ", " +
                  std::to_string(target_steps.second) + ")",
              cv::Point(COLS - 230, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  std::pair<uint16_t, uint16_t> set_pt = turret.get_setpoint_px();
  cv::putText(frame,
              "Setpoint (" + std::to_string(set_pt.first) + ", " +
                  std::to_string(set_pt.second) + ")",
              cv::Point(COLS - 230, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

  if (turret_stopped) {
    cv::putText(frame, "Turret Disabled", cv::Point(10, 50),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
                cv::LINE_AA);
  } else {
    cv::putText(frame, "Turret Enabled", cv::Point(10, 50),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1,
                cv::LINE_AA);
  }
  if (!mos_detection_flag.load()) {
    if (utils::manual_mode.load()) {
      cv::putText(frame, "Keyboard = Manual", cv::Point(10, 70),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
                  cv::LINE_AA);
    } else {
      cv::putText(frame, "Keyboard = Auto", cv::Point(10, 70),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1,
                  cv::LINE_AA);
    }
  } else {
    cv::putText(frame, "Keyboard = OFF", cv::Point(10, 70),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50, 50, 50), 1,
                cv::LINE_AA);
  }
  if (enable_feedback_flag.load()) {
    cv::putText(frame, "Feedback Enabled", cv::Point(10, 90),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1,
                cv::LINE_AA);
  } else {
    cv::putText(frame, "Feedback Disabled", cv::Point(10, 90),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
                cv::LINE_AA);
  }
  if (mos_detection_flag.load()) {
    cv::putText(frame, "Mos Detection = ON", cv::Point(10, 110),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1,
                cv::LINE_AA);
  } else {
    cv::putText(frame, "Mos Detection = OFF", cv::Point(10, 110),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
                cv::LINE_AA);
  }

  cv::putText(frame, "Laser Threshold = " + std::to_string(laser_threshold),
              cv::Point(10, 130), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  cv::putText(frame,
              "Mos Threshold = " + std::to_string(mos_threshold) +
                  "Bg Sub = " + std::to_string(mos_bg_sub),
              cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  cv::putText(frame,
              "PID = (" + std::to_string(K_P) + ", " + std::to_string(K_I) +
                  ", " + std::to_string(K_D) + ")",
              cv::Point(10, 170), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

void test_framerate(cv::VideoCapture& cap) {
  if (!cap.isOpened()) {
    std::cerr << "Error: Could not open video stream." << std::endl;
    return;
  }

  cv::Mat frame;
  int frame_count = 0;
  auto start_time = std::chrono::high_resolution_clock::now();

  while (!utils::exit_flag.load()) {
    if (!cap.read(frame)) {
      std::cerr << "Error: Could not read frame." << std::endl;
      break;
    }

    frame_count++;

    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                            end_time - start_time)
                            .count();

    if (elapsed_time >= 1000) {  // Calculate FPS every second
      float fps = static_cast<float>(frame_count) / (elapsed_time / 1000.0f);
      std::cout << "FPS: " << fps << std::endl;

      // Reset frame count and elapsed time
      frame_count = 0;
      start_time = std::chrono::high_resolution_clock::now();
    }

    // Do something with the frame (e.g., display it)
    cv::imshow("Frame", frame);
    if (cv::waitKey(1) == 27) {
      break;  // Exit if the 'ESC' key is pressed
    }
  }
}

void process_video(cv::VideoCapture& cap) {
  std::vector<cv::Mat> channels;

  gpu::init_gpu();

  std::cout << "ROWS: " << ROWS << std::endl;
  std::cout << "COLS: " << COLS << std::endl;

  bool save_img = false;
  int save_counter = 0;

  gpu::set_learning_rate(learning_rate);

  auto start_time = std::chrono::high_resolution_clock::now();
  while (!utils::exit_flag.load()) {
    // auto start_time = std::chrono::high_resolution_clock::now();

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

    if (mos_detection_flag.load()) {
      mosquitoes_px = gpu::detect_mosquitoes(red_channel.clone(), mos_threshold,
                                             mos_bg_sub);
      turret.update_setpoint({mosquitoes_px.at(0).x, mosquitoes_px.at(0).y});
    }
    laser_pos_px = gpu::detect_laser(red_channel, laser_threshold);
    if (enable_feedback_flag.load()) {
      turret.update_belief(laser_pos_px);
    }

    markup_frame();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                        end_time - start_time)
                        .count();

    start_time = std::chrono::high_resolution_clock::now();

    // std::cout << "FPS: " + std::to_string(1000 / duration) + " (" +
    //                  std::to_string(duration) + " ms)"
    //           << std::endl;
    cv::putText(frame,
                "FPS: " + std::to_string(1000 / duration) + " (" +
                    std::to_string(duration) + " ms)",
                cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

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

  bool threshold_mode = false;
  bool mos_threshold_mode = false;
  bool kp_mode = false;
  bool ki_mode = false;
  bool kd_mode = false;

  bool adjust_size = false;
  int steps = 31 * MICROSTEPS;
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

    if (ch == 'p') {
      learning_rate += 0.01;
      gpu::set_learning_rate(learning_rate);
      std::cout << "Learning rate: " << learning_rate << std::endl;
    } else if (ch == 'l') {
      learning_rate -= 0.01;
      gpu::set_learning_rate(learning_rate);
      std::cout << "Learning rate: " << learning_rate << std::endl;
    } else if (ch == 'g') {
      gpu::set_background(red_channel);
      std::cout << "Background set to curret frame" << std::endl;
    } else if (ch == 't') {
      threshold_mode = !threshold_mode;
      if (threshold_mode) {
        std::cout << "Threshold mode on" << std::endl;
      } else {
        mos_threshold_mode = false;
        kp_mode = false;
        ki_mode = false;
        kd_mode = false;
        std::cout << "Threshold mode off" << std::endl;
      }
    } else if (mos_threshold_mode) {
      if (ch == 'w') {
        mos_threshold += 10;
      } else if (ch == 's') {
        mos_threshold -= 10;
      }
    } else if (kp_mode) {
      if (ch == 'w') {
        K_P += 0.01;
      } else if (ch == 's') {
        K_P -= 0.01;
      }
    } else if (ki_mode) {
      if (ch == 'w') {
        K_I += 0.00001;
      } else if (ch == 's') {
        K_I -= 0.00001;
      }
    } else if (kd_mode) {
      if (ch == 'w') {
        K_D += 0.01;
      } else if (ch == 's') {
        K_D -= 0.01;
      }
    } else if (threshold_mode) {
      if (ch == 'm') {
        mos_threshold_mode = !mos_threshold_mode;
        if (mos_threshold_mode) {
          std::cout << "Mosquito threshold mode on" << std::endl;
        } else {
          std::cout << "Mosquito threshold mode off" << std::endl;
        }
      } else if (ch == 'p') {
        kp_mode = !kp_mode;
        if (kp_mode) {
          std::cout << "Kp mode on" << std::endl;
        } else {
          std::cout << "Kp mode off" << std::endl;
        }
      } else if (ch == 'i') {
        ki_mode = !ki_mode;
        if (ki_mode) {
          std::cout << "Ki mode on" << std::endl;
        } else {
          std::cout << "Ki mode off" << std::endl;
        }
      } else if (ch == 'd') {
        kd_mode = !kd_mode;
        if (kd_mode) {
          std::cout << "Kd mode on" << std::endl;
        } else {
          std::cout << "Kd mode off" << std::endl;
        }
      }
    } else if (ch == 'k') {
      utils::manual_mode.store(false);
      mos_detection_flag.store(!mos_detection_flag.load());
      std::cout << "Mosquito detection: " << mos_detection_flag.load()
                << std::endl;
    } else if (ch == 'h') {
      turret.home(laser_pos_px);
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
      utils::manual_mode.store(!utils::manual_mode.load());
      if (utils::manual_mode.load()) {
        std::cout << "Manual" << std::endl;
        enable_feedback_flag.store(false);
        std::cout << "Feedback off" << std::endl;
      } else {
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
    } else if (utils::manual_mode.load() and adjust_size) {
      if (ch == 'w') {
        steps += 2 * MICROSTEPS;
      } else if (ch == 's') {
        steps -= 2 * MICROSTEPS;
      }
      std::cout << "Steps per click: " << steps << std::endl;
    } else if (!utils::manual_mode.load() and adjust_size) {
      if (ch == 'w') {
        px += 100;
      } else if (ch == 's') {
        px -= 100;
      }
      std::cout << "Pixels per click: " << steps << std::endl;
    } else if (!mos_detection_flag.load()) {
      if (utils::manual_mode.load()) {
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
  // gpu::set_background(initial_frame);

  // Launch the threads
  std::thread user_input_thread(user_input);
  std::thread video_thread(process_video, std::ref(cap));
  // std::thread video_thread(test_framerate, std::ref(cap));
  std::thread turret_horizontal_thread(turret_horizontal);
  std::thread turret_vertical_thread(turret_vertical);

  std::cout << "Threads launched" << std::endl;
  video_thread.join();
  std::cout << "Video thread joined" << std::endl;
  turret_horizontal_thread.join();
  std::cout << "Turret horizontal thread joined" << std::endl;
  turret_vertical_thread.join();
  std::cout << "Turret vertical thread joined" << std::endl;
  user_input_thread.detach();
  std::cout << "User input thread detched" << std::endl;

  cudaDeviceReset();
  exit(0);

  return 0;
}
