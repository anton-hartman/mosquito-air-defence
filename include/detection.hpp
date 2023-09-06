#include <opencv2/opencv.hpp>
#include "utils.hpp"

class Detection {
 private:
  cv::Mat background;  // The current background model.
  float alpha;         // Weight for updating the background model.

  // Thresholding values for red and white masks for laser detection.
  int h_red_low, s_red_low, v_red_low;
  int h_red_high, s_red_high, v_red_high;
  int h_white_low, s_white_low, v_white_low;
  int h_white_high, s_white_high, v_white_high;

  cv::Mat subtract(const cv::Mat& frame);
  void put_label(cv::Mat& img,
                 const std::string& label,
                 const cv::Point& origin);

 public:
  // Constructor: Initializes the bg_subtractor with the first frame.
  Detection(const cv::Mat& first_frame, float alpha = 0.01);

  // Detects blobs in the given binarized image and returns their bounding
  // boxes.
  std::vector<std::vector<int>> detect_mosquitoes(const cv::Mat& frame);

  std::pair<uint16_t, uint16_t> detect_laser(
      const cv::Mat& frame,
      const utils::Circle& laser_belief_region_px);

  void set_red_thresholds(int hl, int sl, int vl, int hh, int sh, int vh);
  void set_white_thresholds(int hl, int sl, int vl, int hh, int sh, int vh);

  void create_threshold_trackbars();
  // Callbacks for trackbars
  static void on_h_red_low_change(int, void* ptr);
  static void on_h_red_high_change(int, void* ptr);
  static void on_s_red_low_change(int, void* ptr);
  static void on_s_red_high_change(int, void* ptr);
  static void on_v_red_low_change(int, void* ptr);
  static void on_v_red_high_change(int, void* ptr);
  static void on_h_white_low_change(int, void* ptr);
  static void on_h_white_high_change(int, void* ptr);
  static void on_s_white_low_change(int, void* ptr);
  static void on_s_white_high_change(int, void* ptr);
  static void on_v_white_low_change(int, void* ptr);
  static void on_v_white_high_change(int, void* ptr);
};
