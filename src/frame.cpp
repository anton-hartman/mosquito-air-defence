#include "../include/frame.hpp"
#include <cmath>

const std::string width = "1280";
const std::string height = "720";

// const std::string left = "0";
// const std::string top = "0";  // top = bottom because of flip
// const std::string right = "1280";
// const std::string bottom = "720";

const std::string left = "90";     // right
const std::string top = "200";     // top = bottom because of flip
const std::string right = "1190";  // left
const std::string bottom = "600";  // bottom = top because of flip

const std::string croppped_width =
    std::to_string(std::stoi(right) - std::stoi(left));
const std::string cropped_height =
    std::to_string(std::stoi(bottom) - std::stoi(top));

const int COLS = std::stoi(croppped_width);
const int ROWS = std::stoi(cropped_height);

const std::string pipeline =
    "nvarguscamerasrc sensor-id=0 tnr-mode=1 tnr-strength=0 "
    "exposuretimerange=\"3000000 3000001\" "
    "! video/x-raw(memory:NVMM), width=" +
    width + ", height=" + height +
    ", format=(string)NV12, framerate=30/1 ! nvvidconv left=" + left +
    " top=" + top + " right=" + right + " bottom=" + bottom +
    " flip-method=2 ! video/x-raw, format=(string)BGRx, width=" +
    croppped_width + ", height=" + cropped_height +
    " ! videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=true "
    "max-buffers=1";
