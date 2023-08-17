/*****************************************************************************
* | File        :   HR8825.h
* | Author      :   Waveshare team
* | Function    :   Drive HR8825
* | Info        :
*                The HR8825 provides an integrated motor driver solution for
*                printers, scanners, and other automated equipment applications.
*                The device has two H-bridge drivers and a microstepping indexer,
*                and is intended to drive a bipolar stepper motor.
*----------------
* |	This version:   V1.0
* | Date        :   2022-6-2
* | Info        :   Basic version
*
******************************************************************************/
#ifndef __PCA9685_H_
#define __PCA9685_H_

#include "DEV_Config.h"

#define MOTOR1  1
#define MOTOR2  2

//Motor Dir
#define FORWARD 0
#define BACKWARD 1

//Control Mode
#define HARDWARD 0
#define SOFTWARD 1

typedef struct {
    uint8_t Name;
    char *MicroStep;
    uint8_t Dir;
    uint8_t EnablePin;
    uint8_t DirPin;
    uint8_t StepPin;
    uint8_t M0Pin;
    uint8_t M1Pin;
    uint8_t M2Pin;
} MOTOR;

void HR8825_SelectMotor(uint8_t name);
void HR8825_Stop(void);
void HR8825_SetMicroStep(char mode, const char *stepformat);
void setMicroStep(uint8_t micro_steps);
void HR8825_TurnStep(uint8_t dir, uint16_t steps, uint16_t stepdelay);
#endif