#pragma once

#include <gpiod.h>
#include <string>

namespace gpio {

class GpioPin {
 public:
  // Constructor: Takes the GPIO chip number, pin number, and an optional
  // consumer name
  GpioPin(unsigned int chip_num,
          unsigned int pin_num,
          const std::string& consumer = "mosquito-air-defence");

  // Destructor: Ensures the pin is released when the object is destroyed
  ~GpioPin();

  // Set the direction (input or output) and initial value
  void setDirection(bool output, int initial_val = 0);

  // Write a value to the GPIO pin
  void write(int value);

  // Read a value from the GPIO pin
  int read();

 private:
  struct gpiod_chip* chip_;  // The GPIO chip
  struct gpiod_line* line_;  // The GPIO line (pin)

  // Helper to check if the line is valid
  bool isValid() const;
};

}  // namespace gpio
