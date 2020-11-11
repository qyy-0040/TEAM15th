#ifndef _IMAGE_H
#define _IMAGE_H
#include "hitsic_common.h"
#include "sys_extint.hpp"

/** HITSIC_Module_DRV */
#include "drv_disp_ssd1306.hpp"
#include "drv_dmadvp.hpp"
#include "drv_cam_zf9v034.hpp"
#include "drv_cam_zf9v034_test.hpp"

#include "cm_backtrace.h"
#include "app_menu_def.hpp"
#include "app_menu.hpp"



#define MISS 255
#define CAMERA_H  120                            //图片高度
#define CAMERA_W  188                            //图片宽度
#define FAR_LINE 1//图像处理上边界
#define NEAR_LINE 113//图像处理下边界
#define LEFT_SIDE 0//图像处理左边界
#define RIGHT_SIDE 187//图像处理右边界
#define MISS 255
#define white_num_MAX 10//每行最多允许白条数

/////////////////////////////
#define black 0
#define white 1
#define blue  2
#define green 3
#define red   4
#define gray  5
#define purple 6
///////////////////////////

extern uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
extern uint8_t image_Buffer_0[CAMERA_H][CAMERA_W];
extern uint32_t imageTH;//指向灰度图的首地址

void head_clear(void);
void THRE(void);
int find_f(int a);
void search_white_range();
void find_all_connect();
void find_road();
uint8_t find_continue(uint8_t i_start, uint8_t j_start);
void ordinary_two_line(void);
void image_main();
void get_mid_line(void);
void my_memset(uint8_t* ptr, uint8_t num, uint8_t size);
void Cam_Test_Over(void);
void Cam_Test(void);
void Cam_Test_1(menu_keyOp_t *_op);
void Cam_Init(void);

int find_crossLeft();
int find_crossRight();
void ols_generateData(uint8_t range[], uint8_t side[], uint8_t xi[], uint8_t yi[]);
void ols_fitImg(uint8_t range[], uint8_t side[], uint8_t xi[], uint8_t yi[]);
void ols_road();
void check();
#endif //
