#include <cuda_runtime.h>
#include <signal.h>
#include <atomic>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>
#include "include/camera.hpp"
#include "include/frame.hpp"
#include "include/image_processing.hpp"
#include "include/pt.hpp"
#include "include/sys_flags.hpp"
#include "include/turret.hpp"

const double F_X = 1279.13149855341;
const double F_Y = 1246.965909876756;
const double C_X_DOUBLE = 457.9588295305912;
const double C_Y_DOUBLE = 240.0948537167988;
const int C_X = std::round(C_X_DOUBLE);
const int C_Y = std::round(C_Y_DOUBLE);
const int TURRET_X_ORIGIN_PX = 550;
const int TURRET_Y_ORIGIN_PX = 334;

std::atomic<bool> sys::keyboard_manual_mode(true);
std::atomic<bool> sys::run_flag(true);
std::atomic<bool> sys::exit_flag(false);
std::atomic<bool> enable_feedback_flag(false);
std::atomic<bool> mos_detection_flag(false);

Turret turret;
cv::VideoCapture cap;
cv::Mat frame;
cv::Mat red_channel;

float bg_learning_rate = 0.0;
bool turret_stopped = true;
bool mos_bg_sub = true;
int laser_threshold = 200;
int mos_threshold = 30;
Pt laser_pt_px;
std::vector<Pt> mosquitoe_pts_px;

void exit_handler(int signo) {
  printf("\r\nSystem exit\r\n");
  turret.stop_turret();
  cv::destroyAllWindows();
  if (cap.isOpened()) {
    cap.release();
  }
  sys::exit_flag.store(true);
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

void draw_target(cv::Mat& frame, const Pt& target, const cv::Scalar& colour) {
  // Length of the perpendicular lines for target and setpoint
  int line_length = 50;
  // Draw a horizontal line passing through the target point
  cv::line(frame, cv::Point(target.x - line_length, target.y),
           cv::Point(target.x + line_length, target.y), colour, 2);
  // Draw a vertical line passing through the target point
  cv::line(frame, cv::Point(target.x, target.y - line_length),
           cv::Point(target.x, target.y + line_length), colour, 2);
}

void markup_frame() {
  cv::rectangle(
      frame,
      cv::Point(gpu::ignore_region_top_left.x, gpu::ignore_region_top_left.y),
      cv::Point(gpu::ignore_region_bottom_right.x,
                gpu::ignore_region_bottom_right.y),
      cv::Scalar(0, 255, 0), 2);

  draw_target(frame,
              Pt{static_cast<int>((C_X_DOUBLE)), static_cast<int>(C_Y_DOUBLE)},
              cv::Scalar(0, 255, 255));
  cv::putText(frame, "Camera Origin", cv::Point(C_X_DOUBLE, C_Y_DOUBLE),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1,
              cv::LINE_AA);
  draw_target(frame, turret.get_origin_px(), cv::Scalar(0, 255, 0));
  cv::putText(frame, "Turret Origin",
              cv::Point(turret.get_origin_px().x, turret.get_origin_px().y),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1,
              cv::LINE_AA);
  draw_target(frame, laser_pt_px, cv::Scalar(0, 0, 255));
  cv::putText(
      frame, "Detected Laser", cv::Point(laser_pt_px.x, laser_pt_px.y + 20),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  draw_target(frame, turret.get_belief_px(), cv::Scalar(255, 0, 255));
  cv::putText(frame, "Belief",
              cv::Point(turret.get_belief_px().x, turret.get_belief_px().y),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1,
              cv::LINE_AA);
  draw_target(frame, turret.get_setpoint_px(), cv::Scalar(255, 0, 0));

  Pt current_steps = turret.get_belief_steps();
  cv::putText(frame,
              "Belief steps (" + std::to_string(current_steps.x) + ", " +
                  std::to_string(current_steps.y) + ")",
              cv::Point(COLS - 230, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

  Pt target_steps = turret.get_setpoint_steps();
  cv::putText(frame,
              "Target steps (" + std::to_string(target_steps.x) + ", " +
                  std::to_string(target_steps.y) + ")",
              cv::Point(COLS - 230, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

  Pt setpoint = turret.get_setpoint_px();
  cv::putText(frame,
              "Setpoint (" + std::to_string(setpoint.x) + ", " +
                  std::to_string(setpoint.y) + ")",
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
    if (sys::keyboard_manual_mode.load()) {
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
                  " Bg Sub = (" + std::to_string(mos_bg_sub) + ")",
              cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  cv::putText(frame,
              "PID = (" + std::to_string(K_P) + ", " + std::to_string(K_I) +
                  ", " + std::to_string(K_D) + ")",
              cv::Point(10, 170), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  cv::putText(frame, "Learning rate = " + std::to_string(bg_learning_rate),
              cv::Point(10, 190), cv::FONT_HERSHEY_SIMPLEX, 0.5,
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

  while (!sys::exit_flag.load()) {
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
  gpu::set_learning_rate(bg_learning_rate);
  std::cout << "ROWS: " << ROWS << std::endl;
  std::cout << "COLS: " << COLS << std::endl;

  bool save_img = false;
  int save_counter = 0;

  auto start_time = std::chrono::high_resolution_clock::now();
  while (!sys::exit_flag.load()) {
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
      mosquitoe_pts_px = gpu::detect_mosquitoes(red_channel.clone(),
                                                mos_threshold, mos_bg_sub);
      std::vector<Pt> laser_pts =
          gpu::detect_laser(red_channel, laser_threshold);
      laser_pt_px = gpu::distinguish_laser_only_2(laser_pts);
      turret.update_belief(laser_pt_px);
      turret.update_setpoint(
          {mosquitoe_pts_px.at(0).x, mosquitoe_pts_px.at(0).y});
    } else {
      std::vector<Pt> laser_pts =
          gpu::detect_laser(red_channel, laser_threshold);
      laser_pt_px = gpu::distinguish_laser_only_2(laser_pts);
      if (enable_feedback_flag.load()) {
        turret.update_belief(laser_pt_px);
      }
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
      sys::exit_flag.store(true);
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
  bool edit_mode = false;
  int steps = 1 * MICROSTEPS;
  int px = 110;
  char ch;
  std::string str_input;

  while (!sys::exit_flag.load()) {
    if (edit_mode) {
      std::cout << "Edit mode (? = info, e = exit):" << std::endl;
    } else {
      if (!mos_detection_flag.load()) {
        std::cout << "Manual control mode (? = info, e = edit mode): ";
      } else {
        std::cout << "Auto control mode (? = info, e = edit mode): ";
      }
    }

    std::cin >> ch;
    std::cin.ignore();  // Discard the newline character from the input stream

    if (ch == 'q') {
      std::cout << "Exit with keypress = q" << std::endl;
      sys::exit_flag.store(true);
      break;
    }

    try {
      if (ch == 'e') {
        edit_mode = !edit_mode;
        if (edit_mode) {
          std::cout << "Edit mode on" << std::endl;
        } else {
          std::cout << "Edit mode off" << std::endl;
        }
      } else if (edit_mode) {
        if (ch == '?') {
          std::cout << "l = laser threshold" << std::endl;
          std::cout << "m = mosquito threshold" << std::endl;
          std::cout << "p = K_P gain" << std::endl;
          std::cout << "i = K_I gain" << std::endl;
          std::cout << "d = K_D gain" << std::endl;
          std::cout << "n = microsteps" << std::endl;
          std::cout << "b = background learning rate" << std::endl;
        } else if (ch == 'l') {
          std::cout << "Enter laser threshold: ";
          std::getline(std::cin, str_input);
          laser_threshold = std::stoi(str_input);
        } else if (ch == 'm') {
          std::cout << "Enter mosquito threshold: ";
          std::getline(std::cin, str_input);
          mos_threshold = std::stoi(str_input);
        } else if (ch == 'p') {
          std::cout << "Enter P: ";
          std::getline(std::cin, str_input);
          K_P = std::stof(str_input);
        } else if (ch == 'i') {
          std::cout << "Enter I: ";
          std::getline(std::cin, str_input);
          K_I = std::stof(str_input);
        } else if (ch == 'd') {
          std::cout << "Enter D: ";
          std::getline(std::cin, str_input);
          K_D = std::stof(str_input);
        } else if (ch == 'n') {
          std::cout << "Enter microsteps: ";
          std::getline(std::cin, str_input);
          int microsteps = std::stoi(str_input);
          set_microsteps(microsteps);
          std::cout << "Microsteps: " << str_input << std::endl;
        } else if (ch == 'b') {
          std::cout << "Enter background learning rate: ";
          std::getline(std::cin, str_input);
          bg_learning_rate = std::stof(str_input);
          gpu::set_learning_rate(bg_learning_rate);
          std::cout << "Learning rate: " << std::to_string(bg_learning_rate)
                    << std::endl;
        }
      } else if (ch == '?') {
        std::cout << "g = set background" << std::endl;
        std::cout << "k = toggle kill mosquitoes (disables keyboard control)"
                  << std::endl;
        std::cout << "h = home turret" << std::endl;
        std::cout << "z = enable/disable turret" << std::endl;
        std::cout << "o = set turret origin" << std::endl;
        std::cout << "m = toggle manual/auto keyboard mode" << std::endl;
        std::cout << "f = toggle turret feedback" << std::endl;
        std::cout << "c = set step size" << std::endl;
        std::cout << "w, a, s, d = move turret" << std::endl;
      } else if (ch == 'g') {
        gpu::set_background(red_channel);
        std::cout << "Background set to curret frame" << std::endl;
      } else if (ch == 'h') {
        turret.home(laser_pt_px);
      } else if (turret_stopped and (ch == 'e' or ch == 'w' or ch == 'a' or
                                     ch == 's' or ch == 'd')) {
        turret_stopped = false;
        turret.start_turret();
        std::cout << "Turret started" << std::endl;
      } else if (ch == 'z') {
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
        turret.set_origin(laser_pt_px);
        std::cout << "Turrt origin set to: " << laser_pt_px.x << ", "
                  << laser_pt_px.y << std::endl;
      } else if (ch == 'f') {
        enable_feedback_flag.store(!enable_feedback_flag.load());
        if (enable_feedback_flag.load()) {
          std::cout << "Turret feedback on" << std::endl;
        } else {
          std::cout << "Turret feedback off" << std::endl;
        }
      } else if (ch == 'c') {
        if (sys::keyboard_manual_mode.load()) {
          std::cout << "Enter step size: ";
          std::getline(std::cin, str_input);
          steps = std::stoi(str_input);
        } else {
          std::cout << "Enter pixel step size: ";
          std::getline(std::cin, str_input);
          px = std::stoi(str_input);
        }
      } else if (ch == 'm') {
        sys::keyboard_manual_mode.store(!sys::keyboard_manual_mode.load());
        if (sys::keyboard_manual_mode.load()) {
          std::cout << "Keyboard manual mode" << std::endl;
          enable_feedback_flag.store(false);
          std::cout << "Turret feedback off" << std::endl;
        } else {
          std::cout << "Keyboard auto mode" << std::endl;
        }
      } else if (ch == 'k') {
        if (!mos_detection_flag.load()) {
          sys::keyboard_manual_mode.store(false);
          std::vector<cv::Mat> initial_channels;
          cv::split(frame, initial_channels);
          gpu::set_background(initial_channels[2]);
          mos_detection_flag.store(true);
          std::cout << "Mosquito kill mode on " << mos_detection_flag.load()
                    << std::endl;
        } else {
          mos_detection_flag.store(false);
          std::cout << "Mosquito kill mode off " << mos_detection_flag.load()
                    << std::endl;
        }
      } else if (ch == 'w' or ch == 'a' or ch == 's' or ch == 'd') {
        if (!mos_detection_flag.load()) {
          if (sys::keyboard_manual_mode.load()) {
            turret.keyboard_manual(ch, steps);
          } else {
            turret.keyboard_auto(ch, px);
          }
        } else {
          std::cout
              << "Cannot manually move turret while mosquito detection is on"
              << std::endl;
        }
      } else {
        std::cout << "Invalid input" << std::endl;
      }
    } catch (std::invalid_argument& e) {
      std::cout << "Invalid argument" << std::endl;
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
  set_microsteps(32);
  cv::VideoCapture cap = init_system();
  cv::Mat initial_frame;
  cap >> initial_frame;

  std::vector<cv::Mat> initial_channels;
  cv::split(initial_frame, initial_channels);
  gpu::set_background(initial_channels[2]);

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
