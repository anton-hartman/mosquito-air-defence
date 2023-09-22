#include "../include/frame.hpp"

const std::string width = "1640";
const std::string height = "1232";
const std::string left = "40";
const std::string top = "280";  // top = bottom because of flip
const std::string right = "1560";
const std::string bottom = "920";
// const std::string left = "0";
// const std::string top = "0";
// const std::string right = "1640";
// const std::string bottom = "1232";
const std::string croppped_width =
    std::to_string(std::stoi(right) - std::stoi(left));
const std::string cropped_height =
    std::to_string(std::stoi(bottom) - std::stoi(top));

const int WIDTH = std::stoi(croppped_width);
const int HEIGHT = std::stoi(cropped_height);

const std::string pipeline =
    "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=" + width +
    ", height=" + height + ", format=(string)NV12 ! nvvidconv left=" + left +
    " top=" + top + " right=" + right + " bottom=" + bottom +
    " flip-method=2 ! video/x-raw, format=(string)BGRx, width=" +
    croppped_width + ", height=" + cropped_height +
    " ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
