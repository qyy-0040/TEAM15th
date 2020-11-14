/*
 * dev_drive.cpp
 *
 *  Created on: 2020年11月11日
 *      Author: fanyi
 */
#include"sc_ftm.h"
#include "dev_drive.h"
#include "lib_pidctrl.h"
float Motor_L_Output;
float Motor_R_Output;
float Motor_L;
float Motor_R;
float Servo;
float Servo_kp;
float Servo_kd;
float Servo_ki;
extern float Servo_Output;

void MOTOR_PWM(void* userData)
{
    SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_0,20000U,Motor_L_Output);
    SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_1,20000U,0);
    SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_2,20000U,Motor_R_Output);
    SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_3,20000U,0);
}
void SERVO_PWM(void* userData)
{
    SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50U,Servo_Output);
}


