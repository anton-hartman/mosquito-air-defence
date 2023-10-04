#include "../include/frame.hpp"

// const std::string width = "1640";
// const std::string height = "1232";

// const std::string left = "40";
// const std::string top = "280";  // top = bottom because of flip
// const std::string right = "1560";
// const std::string bottom = "920";

// // const std::string left = "0";
// // const std::string top = "0";
// // const std::string right = "1640";
// // const std::string bottom = "1232";

const std::string width = "1280";
const std::string height = "720";

// const std::string left = "0";
// const std::string top = "0";  // top = bottom because of flip
// const std::string right = "1280";
// const std::string bottom = "720";

const std::string left = "100";
const std::string top = "200";  // top = bottom because of flip
const std::string right = "1160";
const std::string bottom = "540";

const std::string croppped_width =
    std::to_string(std::stoi(right) - std::stoi(left));
const std::string cropped_height =
    std::to_string(std::stoi(bottom) - std::stoi(top));

const int COLS = std::stoi(croppped_width);
const int ROWS = std::stoi(cropped_height);

const std::string pipeline =
    "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=" + width +
    ", height=" + height + ", format=(string)NV12 ! nvvidconv left=" + left +
    " top=" + top + " right=" + right + " bottom=" + bottom +
    " flip-method=2 ! video/x-raw, format=(string)BGRx, width=" +
    croppped_width + ", height=" + cropped_height +
    " ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

const double F_X = 647.0756309728268;
const double F_Y = 861.7363873209705;
const double C_X = 304.4404590127848;
const double C_Y = 257.5858878142162;

const uint16_t X_ORIGIN_PX = 592;
const uint16_t Y_ORIGIN_PX = 591;