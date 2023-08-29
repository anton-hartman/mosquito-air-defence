#include <signal.h>
#include <cstdlib>
#include "src/detection/videoInterface.cpp"
#include "src/motors/HR8825_driver.hpp"
#include "src/motors/controller.hpp"

void Handler(int signo) {
  // System Exit
  // printf("\r\nHandler:Motor Stop\r\n");
  driver::select_motor(MOTOR1);
  driver::stop_motor();
  driver::select_motor(MOTOR2);
  driver::stop_motor();
  driver::driver_exit();

  exit(0);
}

int main(void) {
  // 1.System Initialization
  driver::init_driver_pins();
  // init_manual_control();
  // if (init_driver_pins())
  //   exit(0);

  // Exception handling:ctrl + c
  signal(SIGINT, Handler);

  // For testing
  stepper::m1_current_angle = 45;
  stepper::m2_current_angle = 45;
  stepper::m1_target_angle = 15;
  stepper::m2_target_angle = 50;
  while (1) {
    stepper::turret_control();
    // delay_ms(3000);
  }

  // 3.System Exit
  driver::driver_exit();
  return 0;
}

// // Main execution
// int main() {
//   VideoInterface vi(0.4, false, true);
//   vi.start_feed("../test_footage/min-30fps.mp4");
//   return 0;
// }
