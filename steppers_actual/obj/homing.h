#include <stdint.h>
#include <stdio.h>   //printf()
#include <stdlib.h>  //exit()
#include <string.h>  //strcmp()

#include "Debug.h"
#include "sysfs_gpio.h"

#define LEFT_BUTTON PIN11
#define HOME_BUTTON PIN13
#define RIGHT_BUTTON PIN15

uint8_t init_homing();