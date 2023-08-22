#include "gpio.hpp"
#include <stdexcept>  // for std::runtime_error

namespace gpio {

GpioPin::GpioPin(unsigned int chip_num,
                 unsigned int pin_num,
                 const std::string& consumer) {
  // Open the GPIO chip
  chip_ = gpiod_chip_open_by_number(chip_num);
  if (!chip_) {
    throw std::runtime_error("Failed to open GPIO chip");
  }

  // Get the GPIO line
  line_ = gpiod_chip_get_line(chip_, pin_num);
  if (!line_) {
    gpiod_chip_close(chip_);
    throw std::runtime_error("Failed to get GPIO line");
  }

  // Reserve the line as input by default
  if (gpiod_line_request_input(line_, consumer.c_str()) < 0) {
    gpiod_chip_close(chip_);
    throw std::runtime_error("Failed to request GPIO line as input");
  }
}

GpioPin::~GpioPin() {
  if (isValid()) {
    gpiod_line_release(line_);
    gpiod_chip_close(chip_);
  }
}

void GpioPin::setDirection(bool output, int initial_val) {
  if (isValid()) {
    if (output) {
      if (gpiod_line_request_output(line_, "mosquito-air-defence",
                                    initial_val) < 0) {
        throw std::runtime_error("Failed to set GPIO line as output");
      }
    } else {
      if (gpiod_line_request_input(line_, "mosquito-air-defence") < 0) {
        throw std::runtime_error("Failed to set GPIO line as input");
      }
    }
  }
}

void GpioPin::write(int value) {
  if (isValid()) {
    gpiod_line_set_value(line_, value);
  }
}

int GpioPin::read() {
  if (isValid()) {
    return gpiod_line_get_value(line_);
  }
  return -1;  // Indicate error
}

bool GpioPin::isValid() const {
  return chip_ && line_;
}

}  // namespace gpio
