/*
 * dev_drive.h
 *
 *  Created on: 2020年11月11日
 *      Author: fanyi
 */

#ifndef DEV_DRIVE_H_
#define DEV_DRIVE_H_

/**头文件引用**/
#include"sc_ftm.h"
#include "lib_pidctrl.h"

/**函数原型**/
void MOTOR_PWM(void* userData);
void SERVO_PWM(void* userData);
void Update_Servo_Output(float Servo_err);
void Update_Motor_Output(float Motor_err);

#endif /* DEV_DRIVE_H_ */
