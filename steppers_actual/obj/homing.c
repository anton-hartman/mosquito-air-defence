#include "homing.h"

uint8_t init_homing() {
  SYSFS_GPIO_Export(LEFT_BUTTON);
  SYSFS_GPIO_Export(HOME_BUTTON);
  SYSFS_GPIO_Export(RIGHT_BUTTON);

  SYSFS_GPIO_Direction(LEFT_BUTTON, IN);
  SYSFS_GPIO_Direction(HOME_BUTTON, IN);
  SYSFS_GPIO_Direction(RIGHT_BUTTON, IN);

  return 0;
}