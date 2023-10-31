#include <cuda_runtime.h>
#include <signal.h>
#include <atomic>
#include <cmath>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>
#include "include/frame.hpp"
#include "include/image_processing.hpp"
#include "include/mads.hpp"
#include "include/pt.hpp"
#include "include/tracking.hpp"
#include "include/turret.hpp"

Turret turret;
cv::VideoCapture cap;
cv::Mat frame;
cv::Mat red_channel;

bool mos_bg_sub = false;
int laser_threshold = 213;
int mos_threshold = 110;
int laser_remove_radius = 5;
Pt laser_pt_px;
std::vector<Pt> mos_pts_px;

bool show_laser = false;
bool show_setpoint = false;
bool show_belief = false;

enum class Quali { START, STOP };
Quali quali = Quali::STOP;
enum class QualiTest { NONE, LASER, MOSQUITO, TRACKING, MAIN };
QualiTest quali_test = QualiTest::NONE;
std::ofstream outfile;

void exit_handler(int signo) {
  printf("\r\nSystem exit\r\n");
  turret.stop_turret();
  cv::destroyAllWindows();
  if (cap.isOpened()) {
    cap.release();
  }
  mads::exit_flag.store(true);
}

cv::VideoCapture init_system(void) {
  std::cout << "Using pipeline: \n\t" << pipeline << std::endl;
  cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

  if (!cap.isOpened()) {
    std::cerr << "Error opening cv video capture" << std::endl;
    exit(-1);
  }
  GPIO::setup(mads::LASER_PIN, GPIO::OUT, GPIO::LOW);
  signal(SIGINT, exit_handler);
  return cap;
}

void mouse_click_callback(int event, int x, int y, int flags, void* userdata) {
  Pt* point = reinterpret_cast<Pt*>(userdata);
  if (event == cv::EVENT_LBUTTONDOWN) {
    point->x = x;
    point->y = y;
    std::cout << "Mouse clicked at (" << x << ", " << y << ")" << std::endl;
  }
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
  int line_length = 30;
  // Draw a horizontal line passing through the target point
  cv::line(frame, cv::Point(target.x - line_length, target.y),
           cv::Point(target.x + line_length, target.y), colour, 1);
  // Draw a vertical line passing through the target point
  cv::line(frame, cv::Point(target.x, target.y - line_length),
           cv::Point(target.x, target.y + line_length), colour, 1);
}

void markup_frame() {
  cv::rectangle(frame, detection::ignore_region_top_left.cv_pt(),
                detection::ignore_region_bottom_right.cv_pt(),
                cv::Scalar(0, 255, 0), 2);

  draw_target(frame, Pt{mads::C_X, mads::C_Y}, cv::Scalar(0, 255, 255));
  cv::putText(frame, "Camera Origin", cv::Point(mads::C_X, mads::C_Y),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1,
              cv::LINE_AA);
  draw_target(frame, turret.get_origin_px(), cv::Scalar(0, 255, 0));
  cv::putText(frame, "Turret Origin", turret.get_origin_px().cv_pt(),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1,
              cv::LINE_AA);
  if (show_laser) {
    draw_target(frame, laser_pt_px, cv::Scalar(0, 0, 255));
    cv::putText(frame, "Detected Laser",
                cv::Point(laser_pt_px.x, laser_pt_px.y + 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1,
                cv::LINE_AA);
  }
  if (show_belief) {
    draw_target(frame, turret.get_belief_px(), cv::Scalar(255, 0, 255));
    cv::putText(frame, "Belief", turret.get_belief_px().cv_pt(),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1,
                cv::LINE_AA);
  }
  if (show_setpoint) {
    draw_target(frame, turret.get_setpoint_px(), cv::Scalar(255, 0, 0));
    cv::putText(frame, "Setpoint", turret.get_setpoint_px().cv_pt(),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1,
                cv::LINE_AA);
  }

  cv::putText(frame, "Belief steps " + turret.get_belief_steps().to_string(),
              cv::Point(COLS - 230, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  cv::putText(frame, "Target steps " + turret.get_setpoint_steps().to_string(),
              cv::Point(COLS - 230, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  cv::putText(frame, "Setpoint " + turret.get_setpoint_px().to_string(),
              cv::Point(COLS - 230, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  cv::putText(frame,
              "Detected laser " + turret.get_detected_laser_px().to_string(),
              cv::Point(COLS - 230, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

  if (mads::turret_stopped.load()) {
    cv::putText(frame, "Turret Disabled", cv::Point(10, 50),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
                cv::LINE_AA);
  } else {
    cv::putText(frame, "Turret Enabled", cv::Point(10, 50),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1,
                cv::LINE_AA);
  }

  if (mads::control.load() == Control::MANUAL) {
    cv::putText(frame, "Control = MANUAL", cv::Point(10, 70),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50, 50, 50), 1,
                cv::LINE_AA);
  } else if (mads::control.load() == Control::KEYBOARD_AUTO) {
    cv::putText(frame, "Control = KEYBOARD_AUTO", cv::Point(10, 70),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
                cv::LINE_AA);
  } else {
    cv::putText(frame, "Control = FULL_AUTO", cv::Point(10, 70),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1,
                cv::LINE_AA);
  }

  if (mads::feedback.load()) {
    cv::putText(frame, "Feedback Enabled", cv::Point(10, 90),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1,
                cv::LINE_AA);
  } else {
    cv::putText(frame, "Feedback Disabled", cv::Point(10, 90),
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
  cv::putText(
      frame,
      "Learning rate = " + std::to_string(detection::bg_learning_rate.load()),
      cv::Point(10, 190), cv::FONT_HERSHEY_SIMPLEX, 0.5,
      cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  cv::putText(frame,
              "Laser = (" + std::to_string(mads::get_laser_state()) + ")",
              cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  cv::putText(frame, "Num Tracks = " + std::to_string(tracking::kalmans.size()),
              cv::Point(10, 230), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  if (mads::laser_lost.load()) {
    cv::putText(frame, "LASER LOST", cv::Point(10, 250),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
                cv::LINE_AA);
  }
}

void test_framerate(cv::VideoCapture& cap) {
  if (!cap.isOpened()) {
    std::cerr << "Error: Could not open video stream." << std::endl;
    return;
  }

  cv::Mat frame;
  int frame_count = 0;
  auto start_time = std::chrono::high_resolution_clock::now();

  while (!mads::exit_flag.load()) {
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

// void laser_toggle_frame(cv::VideoCapture& cap) {
//   while (!mads::exit_flag.load()) {
//     cap >> frame;
//     if (frame.empty()) {
//       std::cout << "Frame is empty, exiting." << std::endl;
//       exit(0);
//     }
//     if (mads::get_laser_state()) {
//       cv::imshow("with laser", frame);
//       cv::waitKey(1);
//       mads::set_laser(false);
//     } else {
//       cv::imshow("without laser", frame);
//       cv::waitKey(1);
//       sys::set_laser(true);
//     }
//   }
// }

void process_video(cv::VideoCapture& cap) {
  std::vector<cv::Mat> channels;
  gpu::init_gpu();
  std::cout << "ROWS: " << ROWS << std::endl;
  std::cout << "COLS: " << COLS << std::endl;
  Pt prev_clicked_pt;
  Pt clicked_point;
  cv::namedWindow("frame", 1);
  cv::setMouseCallback("frame", mouse_click_callback, &clicked_point);

  std::vector<Pt> prev_mossies;

  auto start_time = std::chrono::high_resolution_clock::now();
  while (!mads::exit_flag.load()) {
    cap >> frame;
    turret.save_steps_at_frame();
    if (frame.empty()) {
      std::cout << "Frame is empty, exiting." << std::endl;
      exit(0);
    }

    cv::split(frame, channels);
    red_channel = channels[2];

    if (mads::control.load() == Control::FULL_AUTO) {
      mos_pts_px = detection::detect_mosquitoes(red_channel.clone(),
                                                mos_threshold, mos_bg_sub);
      std::vector<Pt> laser_pts =
          detection::detect_lasers(red_channel, laser_threshold);
      // if (mos_bg_sub) {
      detection::remove_lasers_from_mos(laser_pts, mos_pts_px,
                                        laser_remove_radius);
      // }
      laser_pt_px = detection::distinguish_lasers(laser_pts);
      if (laser_pt_px == Pt{-3, -3}) {
        cv::putText(frame,
                    "MORE THAN TWO LASERS OUTSIDE IGNORE REGION (ALL DECTIONS "
                    "DISCARDED)",
                    cv::Point(200, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
      } else if (laser_pt_px == Pt{-1, -1}) {
        cv::putText(frame, "NO LASERS DETECTED OUTSIDE OF IGNORE REGION",
                    cv::Point(200, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
      }
      turret.update_belief(laser_pt_px);

      if (mos_pts_px.size() < prev_mossies.size()) {
        bool curr_mos_not_in_radius = true;
        for (const Pt& mos : mos_pts_px) {
          if (mos.in_radius(laser_pt_px, laser_remove_radius)) {
            curr_mos_not_in_radius = false;
          }
        }

        if (curr_mos_not_in_radius) {
          for (const Pt& prev_mos : prev_mossies) {
            if (prev_mos.in_radius(laser_pt_px, laser_remove_radius)) {
              mos_pts_px.push_back(prev_mos);
            }
          }
        }
      }
      prev_mossies = mos_pts_px;
      tracking::associate_and_update_tracks(mos_pts_px,
                                            tracking::kalman_dt.load());
      Track current_track;
      if (clicked_point != prev_clicked_pt) {
        current_track = tracking::get_closest_track(clicked_point);
        prev_clicked_pt = clicked_point;
      } else {
        current_track = tracking::get_current_track_preditction(laser_pt_px);
      }
      // turret.update_setpoint(current_track.detected_pt);
      turret.update_setpoint(current_track.predicted_pt);

      if (mads::display.load() & Display::TRACKING) {
        for (const Kalman& kalman : tracking::kalmans) {
          cv::rectangle(frame, kalman.track.predicted_pt.cv_pt(-15),
                        kalman.track.predicted_pt.cv_pt(15),
                        cv::Scalar(0, 0, 255), 2);
          cv::putText(frame, std::to_string(kalman.track.id),
                      kalman.track.predicted_pt.cv_pt(10),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255),
                      1);

          cv::circle(frame, kalman.track.updated_pt.cv_pt(), 5,
                     cv::Scalar(255, 0, 0), 2);
          cv::putText(frame, std::to_string(kalman.track.id),
                      kalman.track.updated_pt.cv_pt(10),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255),
                      1);
          cv::circle(frame, kalman.track.detected_pt.cv_pt(), 12,
                     cv::Scalar(0, 255, 0), 2);
          cv::putText(frame, std::to_string(kalman.track.id),
                      kalman.track.detected_pt.cv_pt(10),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255),
                      1);
          cv::putText(frame, std::to_string(kalman.track.age),
                      kalman.track.detected_pt.cv_pt(-10),
                      cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255),
                      1);

          // Draw the tracks
          for (int i = 1; i < kalman.track.detected_pt_hist.size(); ++i) {
            cv::line(frame, kalman.track.detected_pt_hist[i - 1].cv_pt(),
                     kalman.track.detected_pt_hist[i].cv_pt(),
                     cv::Scalar(0, 255, 0), 2);
          }
          for (int i = 1; i < kalman.track.updated_pt_hist.size(); ++i) {
            cv::line(frame, kalman.track.updated_pt_hist[i - 1].cv_pt(),
                     kalman.track.updated_pt_hist[i].cv_pt(),
                     cv::Scalar(255, 0, 0), 2);
          }
          for (int i = 1; i < kalman.track.predicted_pt_hist.size(); ++i) {
            cv::line(frame, kalman.track.predicted_pt_hist[i - 1].cv_pt(),
                     kalman.track.predicted_pt_hist[i].cv_pt(),
                     cv::Scalar(0, 0, 255), 2);
          }
        }
      }
    } else {
      std::vector<Pt> laser_pts =
          detection::detect_lasers(red_channel, laser_threshold);
      laser_pt_px = detection::distinguish_lasers(laser_pts);
      if (mads::feedback.load()) {
        turret.update_belief(laser_pt_px);
      }

      if (quali_test == QualiTest::LASER and quali == Quali::START) {
        auto now_ms =
            std::chrono::time_point_cast<std::chrono::milliseconds>(start_time);
        auto value = now_ms.time_since_epoch();
        long duration = value.count();
        outfile << duration << "," << laser_pt_px.to_string() << ";";
      }
    }

    markup_frame();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                        end_time - start_time)
                        .count();
    start_time = end_time;

    // std::cout << "FPS: " + std::to_string(1000 / duration) + " (" +
    //                  std::to_string(duration) + " ms)"
    //           << std::endl;
    cv::putText(frame,
                "FPS: " + std::to_string(1000 / duration) + " (" +
                    std::to_string(duration) + " ms)",
                cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    cv::putText(frame, "Clicked Pt = " + clicked_point.to_string(),
                cv::Point(10, 300), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    cv::imshow("frame", frame);
    char key = static_cast<char>(cv::waitKey(1));
    if (key == 'q') {
      mads::exit_flag.store(true);
    } else if (key == 'b') {
      std::pair<cv::Point, cv::Point> points = get_bounding_box();
      detection::set_ignore_region({points.first.x, points.first.y},
                                   {points.second.x, points.second.y});

      std::cout << "Top left point: (" << points.first.x << ", "
                << points.first.y << ")\n";
      std::cout << "Bottom right point: (" << points.second.x << ", "
                << points.second.y << ")\n";
    }
  }

  std::cout << "Video processing out of main loop" << std::endl;
  gpu::free_gpu();
}

void user_input(void) {
  bool edit_mode = false;
  bool test_mode = false;
  bool turret_origin_mode = false;
  int steps = 1 * MICROSTEPS;
  int px = 110;
  std::string input;

  bool next_laser = false;

  while (!mads::exit_flag.load()) {
    if (quali_test == QualiTest::LASER) {
      std::cout << "____________________________________" << std::endl;
      std::cout << "LASER TEST MODE (? = info, q = exit): " << std::endl;
    } else if (test_mode) {
      std::cout << "____________________________________________" << std::endl;
      std::cout << "QAULIFICATION TEST MODE (? = info, q = exit): "
                << std::endl;
    } else if (edit_mode) {
      std::cout << "______________________________" << std::endl;
      std::cout << "EDIT MODE (? = info, q = exit): " << std::endl;
    } else {
      if (mads::control.load() == Control::MANUAL) {
        std::cout << "_________________________" << std::endl;
        std::cout << "MANUAL CONTROL (? = info): ";
      } else if (mads::control.load() == Control::KEYBOARD_AUTO) {
        std::cout << "________________________________" << std::endl;
        std::cout << "KEYBOARD AUTO CONTROL (? = info): ";
      } else {
        std::cout << "____________________________" << std::endl;
        std::cout << "FULL AUTO CONTROL (? = info): ";
      }
    }

    std::getline(std::cin, input);
    try {
      if (input == "q") {
        edit_mode = false;
        test_mode = false;
        quali_test = QualiTest::NONE;
      } else if (input == "e") {
        edit_mode = true;
      } else if (input == "t") {
        test_mode = true;
      } else if (test_mode) {
        if (input == "?") {
          std::cout << "l = laser test" << std::endl;
          std::cout << "Inside a test mode: options = 'start' and 'stop'"
                    << std::endl;
        } else if (quali_test == QualiTest::LASER) {
          if (input == "first") {
            outfile.open("/home/anton/mosquito-air-defence/laser_quali.txt",
                         std::ios_base::out);
            outfile << turret.get_setpoint_px().to_string() << " = set point"
                    << std::endl;
            outfile.close();
          } else if (input == "start") {
            outfile.open("/home/anton/mosquito-air-defence/laser_quali.txt",
                         std::ios_base::app);
            mads::feedback.store(true);
            turret.start_turret();
            mads::turret_stopped.store(false);
            mads::control.store(Control::KEYBOARD_AUTO);
            quali = Quali::START;
            std::cout << "Laser quali test started" << std::endl;
          } else if (input == "s") {
            quali = Quali::STOP;
            std::cout << "Laser quali test stop" << std::endl;
            outfile << std::endl;
            outfile.close();
          } else {
            std::cout << "Invalid input" << std::endl;
          }
        } else if (input == "l") {
          quali_test = QualiTest::LASER;
          mads::control.store(Control::MANUAL);
          turret.home_with_belief(laser_pt_px);
        } else {
          std::cout << "Invalid input" << std::endl;
        }
      } else if (edit_mode) {
        if (input == "?") {
          std::cout << "l = laser threshold" << std::endl;
          std::cout << "m = mosquito threshold" << std::endl;
          std::cout << "p = K_P gain" << std::endl;
          std::cout << "i = K_I gain" << std::endl;
          std::cout << "d = K_D gain" << std::endl;
          std::cout << "n = microsteps" << std::endl;
          std::cout << "b = background learning rate" << std::endl;
          std::cout << "lo = laser opening radius" << std::endl;
          std::cout << "lc = laser closing radius" << std::endl;
          std::cout << "mo = mosquito opening radius" << std::endl;
          std::cout << "mc = mosqutio closing radius" << std::endl;
          std::cout << "r = remove lasers from mos radius" << std::endl;
          std::cout << "a = acceleration sigma" << std::endl;
          std::cout << "dt = kalman dt" << std::endl;
        } else if (input == "a") {
          std::cout << "Enter acceleration sigma: ";
          std::getline(std::cin, input);
          tracking::acc_sigma.store(std::stoi(input));
        } else if (input == "dt") {
          std::cout << "Enter kalman dt: ";
          std::getline(std::cin, input);
          tracking::kalman_dt.store(std::stof(input));
        } else if (input == "r") {
          std::cout << "Enter radius: ";
          std::getline(std::cin, input);
          laser_remove_radius = std::stoi(input);
        } else if (input == "lo") {
          std::cout << "Enter laser opening radius: ";
          std::getline(std::cin, input);
          gpu::set_struct_elem(std::stoi(input), StructElemType::LASER_OPENING);
        } else if (input == "lc") {
          std::cout << "Enter laser closing radius: ";
          std::getline(std::cin, input);
          gpu::set_struct_elem(std::stoi(input), StructElemType::LASER_CLOSING);
        } else if (input == "mo") {
          std::cout << "Enter mosquito opening radius: ";
          std::getline(std::cin, input);
          gpu::set_struct_elem(std::stoi(input), StructElemType::MOS_OPENING);
        } else if (input == "mc") {
          std::cout << "Enter mosquito closing radius: ";
          std::getline(std::cin, input);
          gpu::set_struct_elem(std::stoi(input), StructElemType::MOS_CLOSING);
        } else if (input == "l") {
          std::cout << "Enter laser threshold: ";
          std::getline(std::cin, input);
          laser_threshold = std::stoi(input);
        } else if (input == "m") {
          std::cout << "Enter mosquito threshold: ";
          std::getline(std::cin, input);
          mos_threshold = std::stoi(input);
        } else if (input == "p") {
          std::cout << "Enter P: ";
          std::getline(std::cin, input);
          K_P = std::stof(input);
        } else if (input == "i") {
          std::cout << "Enter I: ";
          std::getline(std::cin, input);
          K_I = std::stof(input);
        } else if (input == "d") {
          std::cout << "Enter D: ";
          std::getline(std::cin, input);
          K_D = std::stof(input);
        } else if (input == "n") {
          std::cout << "Enter microsteps: ";
          std::getline(std::cin, input);
          int microsteps = std::stoi(input);
          set_microsteps(microsteps);
          std::cout << "Microsteps: " << input << std::endl;
        } else if (input == "b") {
          std::cout << "Enter background learning rate: ";
          std::getline(std::cin, input);
          detection::bg_learning_rate.store(std::stof(input));
          std::cout << "Learning rate: "
                    << std::to_string(detection::bg_learning_rate.load())
                    << std::endl;
        } else {
          std::cout << "Invalid input" << std::endl;
        }
      } else if (input == "?") {
        std::cout << "t = qualification test mode" << std::endl;
        std::cout << "e = edit mode" << std::endl;
        std::cout << "display ? = display modes" << std::endl;
        std::cout << "debug ? = debug modes" << std::endl;
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
        std::cout << "sl = show laser" << std::endl;
        std::cout << "ss = show setpoint" << std::endl;
        std::cout << "sb = show belief" << std::endl;
        std::cout << "bg sub = toggle mosquito bg subtraction" << std::endl;
      } else if (input == "display ?") {
        std::cout << "display off" << std::endl;
        std::cout << "display laser" << std::endl;
        std::cout << "display mos" << std::endl;
        std::cout << "display all" << std::endl;
        std::cout << "display tracking" << std::endl;
      } else if (input == "debug ?") {
        std::cout << "debug off" << std::endl;
        std::cout << "debug bg sub" << std::endl;
        std::cout << "debug morph" << std::endl;
        std::cout << "debug tracking" << std::endl;
      } else if (input == "debug off") {
        mads::debug.store(Debug::OFF);
      } else if (input == "debug bg sub") {
        mads::debug.store(Debug::BG_SUB);
      } else if (input == "debug morph") {
        mads::debug.store(Debug::MORPH);
      } else if (input == "debug tracking") {
        mads::debug.store(Debug::TRACKING);
      } else if (input == "display off") {
        mads::display.store(Display::OFF);
      } else if (input == "display laser") {
        mads::display.store(Display::LASER_DETECTION);
      } else if (input == "display mos") {
        mads::display.store(Display::MOSQUITO_DETECTION);
      } else if (input == "display all") {
        mads::display.store(Display::ALL);
      } else if (input == "display tracking") {
        mads::display.store(Display::TRACKING);
      } else if (input == "bg sub") {
        mos_bg_sub = !mos_bg_sub;
      } else if (input == "sl") {
        show_laser = !show_laser;
      } else if (input == "ss") {
        show_setpoint = !show_setpoint;
      } else if (input == "sb") {
        show_belief = !show_belief;
      } else if (input == "g") {
        detection::set_background(red_channel);
        std::cout << "Background set to curret frame" << std::endl;
      } else if (input == "h") {
        mads::control.store(Control::MANUAL);
        turret.home(laser_pt_px);
      } else if (mads::turret_stopped.load() and
                 mads::control.load() == Control::MANUAL and
                 (input == "e" or input == "w" or input == "a" or
                  input == "s" or input == "d")) {
        mads::turret_stopped.store(false);
        turret.start_turret();
        std::cout << "Turret started" << std::endl;
      } else if (input == "z") {
        if (mads::turret_stopped.load()) {
          mads::turret_stopped.store(false);
          turret.start_turret();
          std::cout << "Turret started" << std::endl;
        } else {
          mads::turret_stopped.store(true);
          turret.stop_turret();
          std::cout << "Turret stopped" << std::endl;
        }
      } else if (input == "l") {
        mads::set_laser(!mads::get_laser_state());
        if (mads::get_laser_state()) {
          std::cout << "Laser on" << std::endl;
        } else {
          std::cout << "Laser off" << std::endl;
        }
      } else if (input == "o") {
        turret_origin_mode = !turret_origin_mode;
        if (turret_origin_mode) {
          std::cout << "Turret origin mode = ON" << std::endl;
        } else {
          std::cout << "Turret origin mode = OFF" << std::endl;
        }
      } else if (input == "f") {
        mads::feedback.store(!mads::feedback.load());
        if (mads::feedback.load()) {
          std::cout << "Turret feedback on" << std::endl;
        } else {
          std::cout << "Turret feedback off" << std::endl;
        }
      } else if (input == "c") {
        if (mads::control.load() == Control::MANUAL) {
          std::cout << "Enter step size: ";
          std::getline(std::cin, input);
          steps = std::stoi(input);
        } else {
          std::cout << "Enter pixel step size: ";
          std::getline(std::cin, input);
          px = std::stoi(input);
        }
      } else if (input == "m") {
        if (mads::control.load() != Control::MANUAL) {
          mads::control.store(Control::MANUAL);
          mads::feedback.store(false);
          std::cout << "MANUAL control = Turret feedback OFF" << std::endl;
        } else {
          mads::control.store(Control::KEYBOARD_AUTO);
        }
      } else if (input == "k") {
        if (mads::control.load() != Control::FULL_AUTO) {
          // std::vector<cv::Mat> initial_channels;
          // cv::split(frame, initial_channels);
          // gpu::set_background(initial_channels[2]);
          mads::control.store(Control::FULL_AUTO);
        } else {
          mads::control.store(Control::MANUAL);
        }
      } else if (input == "w" or input == "a" or input == "s" or input == "d") {
        if (turret_origin_mode) {
          turret.step_origin(input[0], px);
          std::cout << "Turrt origin set to: "
                    << turret.get_origin_px().to_string() << std::endl;
        } else if (mads::control.load() == Control::MANUAL) {
          turret.keyboard_manual(input[0], steps);
        } else if (mads::control.load() == Control::KEYBOARD_AUTO) {
          turret.keyboard_auto(input[0], px);
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
  cv::VideoCapture cap = init_system();
  set_microsteps(32);
  cv::Mat initial_frame;
  cap >> initial_frame;

  std::vector<cv::Mat> initial_channels;
  cv::split(initial_frame, initial_channels);
  detection::set_background(initial_channels[2]);
  mads::set_laser(false);

  std::thread user_input_thread(user_input);
  std::thread video_thread(process_video, std::ref(cap));
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
