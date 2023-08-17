/*****************************************************************************
* | File        :   sysfs_gpio.h
* | Author      :   Waveshare team
* | Function    :   Drive SC16IS752 GPIO
* | Info        :   Read and write /sys/class/gpio
*----------------
* |	This version:   V1.0
* | Date        :   2019-06-04
* | Info        :   Basic version
*
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#ifndef __SYSFS_GPIO_
#define __SYSFS_GPIO_

#include <stdio.h>

#define IN 0
#define OUT 1

#define LOW 0
#define HIGH 1

#define NUM_MAXBUF 4
#define DIR_MAXSIZ 60

#define SYSFS_GPIO_DEBUG 1
#if SYSFS_GPIO_DEBUG
#define SYSFS_GPIO_Debug(__info, ...) printf("Debug: " __info, ##__VA_ARGS__)
#else
#define SYSFS_GPIO_Debug(__info, ...)
#endif

// BCM GPIO for Jetson nano
#define GPIO4 216     // Pin 7,  BCM 4
#define GPIO17 50     // Pin 11, BCM 17
#define GPIO18 79     // Pin 12, BCM 18
#define GPIO27 14     // Pin 13, BCM 27
#define GPIO22 194    // Pin 15, BCM 22
#define GPIO23 232    // Pin 16, BCM 23
#define GPIO24 15     // Pin 18, BCM 24
#define SPI0_MOSI 16  // Pin 19, BCM 10
#define SPI0_MISO 17  // Pin 21, BCM 9
#define GPIO25 13     // Pin 22, BCM 25
#define SPI0_SCK 18   // Pin 23, BCM 11
#define SPI0_CS0 19   // Pin 24, BCM 8
#define SPI0_CS1 20   // Pin 26, BCM 7
#define GPIO5 149     // Pin 29, BCM 5
#define GPIO6 200     // Pin 31, BCM 6
#define GPIO12 168    // Pin 32, BCM 12
#define GPIO13 38     // Pin 33, BCM 13
#define GPIO19 76     // Pin 35, BCM 19
#define GPIO16 51     // Pin 36, BCM 16
#define GPIO26 12     // Pin 37, BCM 26
#define GPIO20 77     // Pin 38, BCM 20
#define GPIO21 78     // Pin 40, BCM 21
// 22PIN + 2PIN UART0 + 2PIN I2C0 + 2PIN I2C
// + 2PIN 3V3 + 2PIN 5V + 8PIN GND  = 40PIN

// GPIO in terms of physical pins, there are 22 GPIO pins
#define PIN7 216   // BCM 4,  Linux 216
#define PIN11 50   // BCM 17, Linux 50
#define PIN12 79   // BCM 18, Linux 79
#define PIN13 14   // BCM 27, Linux 14
#define PIN15 194  // BCM 22, Linux 194
#define PIN16 232  // BCM 23, Linux 232
#define PIN18 15   // BCM 24, Linux 15
#define PIN19 16   // BCM 10, Linux 16
#define PIN21 17   // BCM 9,  Linux 17
#define PIN22 13   // BCM 25, Linux 13
#define PIN23 18   // BCM 11, Linux 18
#define PIN24 19   // BCM 8,  Linux 19
#define PIN26 20   // BCM 7,  Linux 20
#define PIN29 149  // BCM 5,  Linux 149
#define PIN31 200  // BCM 6,  Linux 200
#define PIN32 168  // BCM 12, Linux 168
#define PIN33 38   // BCM 13, Linux 38
#define PIN35 76   // BCM 19, Linux 76
#define PIN36 51   // BCM 16, Linux 51
#define PIN37 12   // BCM 26, Linux 12
#define PIN38 77   // BCM 20, Linux 77
#define PIN40 78   // BCM 21, Linux 78

int SYSFS_GPIO_Export(int Pin);
int SYSFS_GPIO_Unexport(int Pin);
int SYSFS_GPIO_Direction(int Pin, int Dir);
int SYSFS_GPIO_Read(int Pin);
int SYSFS_GPIO_Write(int Pin, int value);

#endif