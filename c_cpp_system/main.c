#include <signal.h>
#include <stdio.h>   //printf()
#include <stdlib.h>  //exit()
#include "steppers/HR8825_driver.h"
#include "steppers/controller.h"

void Handler(int signo) {
  // System Exit
  printf("\r\nHandler:Motor Stop\r\n");
  select_motor(MOTOR1);
  stop_motor();
  select_motor(MOTOR2);
  stop_motor();
  driver_exit();

  exit(0);
}

int main(void) {
  // 1.System Initialization
  init_driver_pins();
  init_manual_control();
  // if (init_driver_pins())
  //   exit(0);

  // Exception handling:ctrl + c
  signal(SIGINT, Handler);

  // For testing
  m1_current_angle = 45;
  m2_current_angle = 45;
  m1_target_angle = 15;
  m2_target_angle = 50;
  while (1) {
    turret_control();
    // delay_ms(3000);
  }

  // 3.System Exit
  driver_exit();
  return 0;
}