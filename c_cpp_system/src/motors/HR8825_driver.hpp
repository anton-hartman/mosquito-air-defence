#pragma once
namespace driver {

#include <stdint.h>
#include "../sysfs_gpio.h"

/* GPIO */
// #define M1_ENABLE_PIN GPIO12
// #define M1_DIR_PIN GPIO13
// #define M1_STEP_PIN GPIO19
// #define M1_M0_PIN GPIO16
// #define M1_M1_PIN GPIO17
// #define M1_M2_PIN GPIO20

// #define M2_ENABLE_PIN GPIO4
// #define M2_DIR_PIN GPIO24
// #define M2_STEP_PIN GPIO18
// #define M2_M0_PIN GPIO21
// #define M2_M1_PIN GPIO22
// #define M2_M2_PIN GPIO27

#define M1_ENABLE_PIN PIN32
#define M1_DIR_PIN PIN33
#define M1_STEP_PIN PIN35
// #define M1_M0_PIN PIN36
// #define M1_M1_PIN PIN37
// #define M1_M2_PIN PIN38

#define M2_ENABLE_PIN PIN7
#define M2_DIR_PIN PIN18
#define M2_STEP_PIN PIN12
// #define M2_M0_PIN PIN40
// #define M2_M1_PIN PIN15
// #define M2_M2_PIN PIN13

// HR8825
#define MOTOR1 1
#define MOTOR2 2

// Motor Dir
#define FORWARD 0
#define BACKWARD 1

typedef struct {
  uint8_t name;
  char* microstep;
  uint8_t direction;
  uint8_t enable_pin;
  uint8_t direction_pin;
  uint8_t step_pin;
  uint8_t M0_pin;
  uint8_t M1_pin;
  uint8_t M2_pin;
} MOTOR;

uint8_t init_driver_pins(void);  // DEV_ModuleInit(void)
void driver_exit(void);          // DEV_ModuleExit(void)

void select_motor(uint8_t name);
void stop_motor(void);
void turn_motor(uint8_t direction, uint16_t steps, uint16_t stepdelay);

}  // namespace driver