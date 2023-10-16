#include "../include/frame.hpp"
#include <cmath>

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

const std::string left = "120";
const std::string top = "200";  // top = bottom because of flip
const std::string right = "1160";
const std::string bottom = "540";

const std::string croppped_width =
    std::to_string(std::stoi(right) - std::stoi(left));
const std::string cropped_height =
    std::to_string(std::stoi(bottom) - std::stoi(top));

const int COLS = std::stoi(croppped_width);
const int ROWS = std::stoi(cropped_height);

// "gainrange=\"1 16\" ispdigitalgainrange=\"1 8\" tnr-strength=-1 tnr-mode=0 "
// "ee-mode=0 ee-strength=-1 aelock=true awblock=true "

const std::string pipeline =
    // "gainrange=\"3 4\" ispdigitalgainrange=\"6 7\" awblock=false "
    "nvarguscamerasrc sensor-id=0 tnr-mode=1 tnr-strength=0 "
    // "nvarguscamerasrc sensor-id=0 "
    "exposuretimerange=\"100000000 100000001\" "
    "! video/x-raw(memory:NVMM), width=" +
    width + ", height=" + height +
    ", format=(string)NV12, framerate=30/1 ! nvvidconv left=" + left +
    " top=" + top + " right=" + right + " bottom=" + bottom +
    " flip-method=2 ! video/x-raw, format=(string)BGRx, width=" +
    croppped_width + ", height=" + cropped_height +
    " ! videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=true "
    "max-buffers=1";

// const std::string pipeline =
//     // " exposuretimerange=\"10000000 10000001\" "
//     // "gainrange=\"3 4\" ispdigitalgainrange=\"6 7\" awblock=false "
//     "nvarguscamerasrc sensor-id=0 ! "
//     "video/x-raw(memory:NVMM), width=" +
//     width + ", height=" + height +
//     ", format=(string)NV12, framerate=60/1 ! nvvidconv left=" + left +
//     " top=" + top + " right=" + right + " bottom=" + bottom +
//     " flip-method=2 ! video/x-raw, format=(string)BGRx, width=" +
//     croppped_width + ", height=" + cropped_height +
//     " ! videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=true "
//     "max-buffers=1";

// distCoeffs : [0.06150274950265684, -0.8341596977696566, 0.005983206836437576,
// -0.008859874706440122, 5.750238644767881]

const double F_X = 1279.13149855341;
const double F_Y = 1246.965909876756;
const double C_X_DOUBLE = 457.9588295305912;
const double C_Y_DOUBLE = 240.0948537167988;
const uint16_t C_X = std::round(C_X_DOUBLE);
const uint16_t C_Y = std::round(C_Y_DOUBLE);

const uint16_t TURRET_X_ORIGIN_PX = 533;
const uint16_t TURRET_Y_ORIGIN_PX = 332;

// double K_P = 0.05;
// double K_I = 0.0;
// double K_D = 0.2;

const float learning_rate = 0.001;