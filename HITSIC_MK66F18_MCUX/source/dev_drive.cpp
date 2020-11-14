/*
 * dev_drive.cpp
 *
 *  Created on: 2020年11月11日
 *      Author: fanyi
 */
#include "dev_drive.h"

float Motor_L_Output;
float Motor_R_Output;
float Motor_L;
float Motor_R;
float Servo;
float Servo_kp;
float Servo_kd;
float Servo_ki;
extern float Servo_Output;

void Update_Servo_Output(void)
{
    pidCtrl_t*Servo_pid = PIDCTRL_Construct(Servo_kp, Servo_ki, Servo_kd);
    Servo_err = Servo_err/10;
    Servo_Output = Servo + PIDCTRL_UpdateAndCalcPID(Servo_pid, Servo_err);
}

void Update_MotorR_Output(void)
{
    Motor_L_Output = Motor_L;
    Motor_R_Output = Motor_R;
}

void Update_MotorL_Output(void)
{
    Motor_L_Output = Motor_L;
    Motor_R_Output = Motor_R;
}

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


