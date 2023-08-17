#include <stdio.h>      //printf()
#include <stdlib.h>     //exit()
#include <signal.h>
#include "DEV_Config.h"
#include "HR8825.h"
#include "controller.h"


void  Handler(int signo)
{
    //System Exit
    printf("\r\nHandler:Motor Stop\r\n");
    HR8825_SelectMotor(MOTOR1);
    HR8825_Stop();
    HR8825_SelectMotor(MOTOR2);
    HR8825_Stop();
    DEV_ModuleExit();

    exit(0);
}

int main(void)
{
    //1.System Initialization
    if(DEV_ModuleInit())
        exit(0);
    
    // Exception handling:ctrl + c
    signal(SIGINT, Handler);

    m1_current_angle = 45;
    m2_current_angle = 45;

    m1_target_angle = 15;
    m2_target_angle = 50;
    while (1) {
        turret_control();
        // print test
        printf("m1_current_angle: %d\n", m1_current_angle);
    }
    
    //3.System Exit
    DEV_ModuleExit();
    return 0;
}

