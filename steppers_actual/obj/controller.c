#include <stdio.h>     
#include <stdlib.h>    
#include <signal.h>
#include "DEV_Config.h"
#include "HR8825.h"

// #define FULL_REV 2048
#define FULL_STEP_ANGLE 0.17578125 // 360/(32*64) = 0.17578125

// Global Variables
int16_t m1_current_angle; // Current angle of motor 1
int16_t m2_current_angle;

void Handler(int signo) {
    //System Exit
    printf("\r\nHandler:Motor Stop\r\n");
    HR8825_SelectMotor(MOTOR1);
    HR8825_Stop();
    HR8825_SelectMotor(MOTOR2);
    HR8825_Stop();
    DEV_ModuleExit();

    exit(0);
}

int main(void) {
    //1.System Initialization
    if(DEV_ModuleInit()) // If init is successful it returns 0
        exit(0);
    
    // Exception handling:ctrl + c
    signal(SIGINT, Handler);

    while(1) {

        setMicroStep(1);
    
        HR8825_SelectMotor(MOTOR1);
        HR8825_TurnStep(BACKWARD, FULL_REV, 3);
        HR8825_Stop();
        DEV_Delay_ms(5000);
       
        HR8825_SelectMotor(MOTOR2);
        HR8825_TurnStep(BACKWARD, FULL_REV, 3);
        HR8825_Stop();
        DEV_Delay_ms(5000);
    }
    
    //3.System Exit
    DEV_ModuleExit();
    return 0;
}

