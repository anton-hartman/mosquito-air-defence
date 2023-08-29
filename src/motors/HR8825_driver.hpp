#pragma once

#include <cstdint>

namespace driver {

#define M1_ENABLE_PIN 32
#define M1_DIR_PIN 33
#define M1_STEP_PIN 35

#define M2_ENABLE_PIN 7
#define M2_DIR_PIN 18
#define M2_STEP_PIN 12

// HR8825
#define MOTOR1 1
#define MOTOR2 2

// Motor Dir
#define FORWARD 0
#define BACKWARD 1

typedef struct {
  uint8_t name;
  uint8_t direction;
  uint8_t enable_pin;
  uint8_t direction_pin;
  uint8_t step_pin;
} MOTOR;

uint8_t init_driver_pins(void);
void driver_exit(void);

void select_motor(uint8_t name);
void stop_motor(void);
void stop_all_motors(void);
void turn_motor(uint8_t direction, uint16_t steps, uint16_t stepdelay);

}  // namespace driver