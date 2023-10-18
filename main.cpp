#include <cuda_runtime.h>
#include <signal.h>
#include <atomic>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>
#include "include/detection.hpp"
#include "include/frame.hpp"
// #include "include/image_processing.hpp"
#include "include/mads.hpp"
#include "include/pt.hpp"
#include "include/turret.hpp"

Turret turret;
cv::VideoCapture cap;
cv::Mat frame;
cv::Mat red_channel;

// float bg_learning_rate = 0.0;
bool mos_bg_sub = true;
int laser_threshold = 200;
int mos_threshold = 30;
Pt laser_pt_px;
std::vector<Pt> mos_pts_px;

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
  cv::rectangle(frame, gpu::ignore_region_top_left.cv_pt(),
                gpu::ignore_region_bottom_right.cv_pt(), cv::Scalar(0, 255, 0),
                2);

  draw_target(frame, Pt{mads::C_X, mads::C_Y}, cv::Scalar(0, 255, 255));
  cv::putText(frame, "Camera Origin", cv::Point(mads::C_X, mads::C_Y),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1,
              cv::LINE_AA);
  draw_target(frame, turret.get_origin_px(), cv::Scalar(0, 255, 0));
  cv::putText(frame, "Turret Origin", turret.get_origin_px().cv_pt(),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1,
              cv::LINE_AA);
  // draw_target(frame, laser_pt_px, cv::Scalar(0, 0, 255));
  cv::putText(
      frame, "Detected Laser", cv::Point(laser_pt_px.x, laser_pt_px.y + 20),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  draw_target(frame, turret.get_belief_px(), cv::Scalar(255, 0, 255));
  cv::putText(frame, "Belief", turret.get_belief_px().cv_pt(),
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

float euclidean_dist(Pt pt1, Pt pt2) {
  return std::sqrt(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2));
}

void remove_laser_pts(const std::vector<Pt>& laser_pts,
                      std::vector<Pt>& mos_pts) {
  for (std::vector<Pt>::iterator it = mos_pts.begin(); it != mos_pts.end();) {
    bool remove = false;
    for (const Pt& laser_pt : laser_pts) {
      if (euclidean_dist(*it, laser_pt) < 20) {
        remove = true;
        break;
      }
    }
    if (remove) {
      it = mos_pts.erase(it);
    } else {
      ++it;
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

  bool save_img = false;
  int save_counter = 0;

  auto start_time = std::chrono::high_resolution_clock::now();
  while (!mads::exit_flag.load()) {
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

    if (mads::control.load() == Control::FULL_AUTO) {
      mos_pts_px = detection::detect_mosquitoes(red_channel.clone(),
                                                mos_threshold, mos_bg_sub);
      std::vector<Pt> laser_pts =
          detection::detect_lasers(red_channel, laser_threshold);
      remove_laser_pts(laser_pts, mos_pts_px);

      if ((mads::display.load() & Display::ALL_DECTIONS) ==
          Display::ALL_DECTIONS) {
        cv::Mat black_image(ROWS, COLS, CV_8UC3, cv::Scalar(0, 0, 0));
        for (size_t i = 0; i < mos_pts_px.size(); ++i) {
          cv::circle(black_image, mos_pts_px.at(i).cv_pt(), 20,
                     cv::Scalar(150, 255, 255), 2);
          cv::putText(
              black_image, std::to_string(i),
              cv::Point(mos_pts_px.at(i).x + 10, mos_pts_px.at(i).y + 10),
              cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1);
        }
        cv::imshow("laser removed mossies", black_image);
        cv::waitKey(1);
      }

      laser_pt_px = detection::distinguish_lasers(laser_pts);
      if (laser_pt_px == Pt{-3, -3}) {
        cv::putText(frame,
                    "MORE THAN TWO LASERS OUTSIDE IGNORE REGION (ALL DECTIONS "
                    "DISCARDED)",
                    cv::Point(200, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
      } else {
        turret.update_belief(laser_pt_px);
      }
      if (mos_pts_px.size() > 0) {
        turret.update_setpoint(mos_pts_px.at(0));
      }
    } else {
      std::vector<Pt> laser_pts =
          gpu::detect_lasers(red_channel, laser_threshold);
      laser_pt_px = gpu::distinguish_laser_only_2(laser_pts);
      if (mads::feedback.load()) {
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
      mads::exit_flag.store(true);
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

  while (!mads::exit_flag.load()) {
    if (edit_mode) {
      std::cout << "Edit mode (? = info, e = exit):" << std::endl;
    } else {
      if (mads::control.load() == Control::MANUAL) {
        std::cout << "MANUAL control mode (? = info, e = edit mode): ";
      } else if (mads::control.load() == Control::KEYBOARD_AUTO) {
        std::cout << "KEYBOARD_AUTO control mode (? = info, e = edit mode): ";
      } else {
        std::cout << "FULL_AUTO control mode (? = info, e = edit mode): ";
      }
    }

    std::cin >> ch;
    std::cin.ignore();  // Discard the newline character from the input stream

    if (ch == 'q') {
      std::cout << "Exit with keypress = q" << std::endl;
      mads::exit_flag.store(true);
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
          detection::bg_learning_rate.store(std::stof(str_input));
          std::cout << "Learning rate: "
                    << std::to_string(detection::bg_learning_rate.load())
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
        // Control prev_control = mads::control.load();
        mads::control.store(Control::MANUAL);
        turret.home(laser_pt_px);
        // mads::control.store(prev_control);
      } else if (mads::turret_stopped.load() and
                 (ch == 'e' or ch == 'w' or ch == 'a' or ch == 's' or
                  ch == 'd')) {
        mads::turret_stopped.store(false);
        turret.start_turret();
        std::cout << "Turret started" << std::endl;
      } else if (ch == 'z') {
        if (mads::turret_stopped.load()) {
          mads::turret_stopped.store(false);
          turret.start_turret();
          std::cout << "Turret started" << std::endl;
        } else {
          mads::turret_stopped.store(true);
          turret.stop_turret();
          std::cout << "Turret stopped" << std::endl;
        }
      } else if (ch == 'l') {
        mads::set_laser(!mads::get_laser_state());
        if (mads::get_laser_state()) {
          std::cout << "Laser on" << std::endl;
        } else {
          std::cout << "Laser off" << std::endl;
        }
      } else if (ch == 'o') {
        turret.set_origin(laser_pt_px);
        std::cout << "Turrt origin set to: " << laser_pt_px.x << ", "
                  << laser_pt_px.y << std::endl;
      } else if (ch == 'f') {
        mads::feedback.store(!mads::feedback.load());
        if (mads::feedback.load()) {
          std::cout << "Turret feedback on" << std::endl;
        } else {
          std::cout << "Turret feedback off" << std::endl;
        }
      } else if (ch == 'c') {
        if (mads::control.load() == Control::MANUAL) {
          std::cout << "Enter step size: ";
          std::getline(std::cin, str_input);
          steps = std::stoi(str_input);
        } else {
          std::cout << "Enter pixel step size: ";
          std::getline(std::cin, str_input);
          px = std::stoi(str_input);
        }
      } else if (ch == 'm') {
        if (mads::control.load() != Control::MANUAL) {
          mads::control.store(Control::MANUAL);
          mads::feedback.store(false);
          std::cout << "MANUAL control = Turret feedback OFF" << std::endl;
        } else {
          mads::control.store(Control::KEYBOARD_AUTO);
        }
      } else if (ch == 'k') {
        if (mads::control.load() != Control::FULL_AUTO) {
          // std::vector<cv::Mat> initial_channels;
          // cv::split(frame, initial_channels);
          // gpu::set_background(initial_channels[2]);
          mads::control.store(Control::FULL_AUTO);
        } else {
          mads::control.store(Control::MANUAL);
        }
      } else if (ch == 'w' or ch == 'a' or ch == 's' or ch == 'd') {
        if (mads::control.load() == Control::MANUAL) {
          turret.keyboard_manual(ch, steps);
        } else if (mads::control.load() == Control::KEYBOARD_AUTO) {
          turret.keyboard_auto(ch, px);
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
  gpu::set_background(initial_channels[2]);
  mads::set_laser(true);

  std::thread user_input_thread(user_input);
  // std::thread video_thread(laser_toggle_frame, std::ref(cap));
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
