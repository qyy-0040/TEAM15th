#ifndef DEV_CONTROL_HPP_
#define DEV_CONTROL_HPP_
#include "hitsic_common.h"
#include "inc_stdlib.hpp"

#include "sys_pitmgr.hpp"
#include "sys_extint.hpp"
#include "drv_imu_invensense.hpp"
#include "lib_pidctrl.h"

#include "app_menu.hpp"
#include "sc_ftm.h"

#include "dev_Image.h"

#define CTRL_SPD_CTRL_MS    (5U)
#define CTRL_DIR_CTRL_MS    (20U)

#define CTRL_1G             (9.80f)
//#define CTRL_ASIN(x)        (arm_arcsin_f32(x))
#define CTRL_ASIN(x)        (asin(x))

#define CTRL_PI             (3.1415926f)

#define CTRL_DEG2RAD(x)     (x * (CTRL_PI / 180.0f))
#define CTRL_RAD2DEG(x)     (x * (180.0f / CTRL_PI))

#define CTRL_ENCO_SPD_COEFF (270.689 / ((float)CTRL_SPD_CTRL_MS))

extern inv::mpu6050_t imu_6050;

void CTRL_Init(void);



void CTRL_MenuInit(menu_list_t *menuList);

/* ******************** 速度环 ******************** */
extern int32_t ctrl_spdCtrlEn[3];
extern float ctrl_spdSet;
extern pidCtrl_t ctrl_spdPid;
extern float ctrl_spdL, ctrl_spdR;
extern float ctrl_spdAvg;
extern float ctrl_spdPidOutput;

void CTRL_SpdCtrl(void *userData);

/* *********************************************** */


/* ******************** 转向环 ******************** */
extern int32_t ctrl_dirCtrlEn[3];
extern pidCtrl_t ctrl_dirPid;
extern float ctrl_dirPidOutput;

void CTRL_DirCtrl(void *userData);

/* *********************************************** */



void CTRL_MotorUpdate(float motorL, float motorR);


#endif /* DEV_CONTROL_HPP_ */
