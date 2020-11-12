/*
 * dev_drive.cpp
 *
 *  Created on: 2020年11月11日
 *      Author: fanyi
 */
#include"sc_ftm.h"
#include "dev_drive.h"
float Motor_L = 10.50;
float Motor_R = 10.50;
float Servo = 5.60;
void MOTOR_PWM(void)
{
    SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_0,20000U,Motor_L);
    SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_1,20000U,0);
    SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_2,20000U,Motor_R);
    SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_3,20000U,0);
}
void SERVO_PWM(void)
{
    SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50U,Servo);
}


