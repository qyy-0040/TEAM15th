/*
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Copyright 2018 - 2020 HITSIC
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "hitsic_common.h"

/** HITSIC_Module_DRV */
#include "drv_ftfx_flash.hpp"
#include "drv_disp_ssd1306.hpp"
#include "drv_imu_invensense.hpp"
#include "drv_dmadvp.hpp"
#include "drv_cam_zf9v034.hpp"

/** HITSIC_Module_SYS */
#include "sys_pitmgr.hpp"
#include "sys_extint.hpp"
#include "sys_uartmgr.hpp"
#include "cm_backtrace.h"
//#include "easyflash.h"

/** HITSIC_Module_LIB */
#include "lib_graphic.hpp"

/** HITSIC_Module_APP */
#include "app_menu.hpp"
#include "app_svbmp.hpp"

/** FATFS */
#include "ff.h"
#include "sdmmc_config.h"
FATFS fatfs;                                   //逻辑驱动器的工作区

#include "sc_adc.h"
#include "sc_ftm.h"

/** HITSIC_Module_TEST */
#include "drv_cam_zf9v034_test.hpp"
#include "app_menu_test.hpp"
#include "drv_imu_invensense_test.hpp"
#include "sys_fatfs_test.hpp"
#include "sys_fatfs_diskioTest.hpp"

/** SCLIB_TEST */
#include "sc_test.hpp"

/**TEAM 15th Dev**/
#include "dev_Image.h"
#include "dev_drive.h"

void MENU_DataSetUp(void);

inv::i2cInterface_t imu_i2c(nullptr, IMU_INV_I2cRxBlocking, IMU_INV_I2cTxBlocking);
inv::mpu6050_t imu_6050(imu_i2c);


disp_ssd1306_frameBuffer_t dispBuffer;
graphic::bufPrint0608_t<disp_ssd1306_frameBuffer_t> bufPrinter(dispBuffer);
extern float Motor_L;
extern float Motor_R;
extern float Motor_L_Output;
extern float Motor_R_Output;
extern float Servo;
extern float Servo_kp;
extern float Servo_kd;
extern float Servo_ki;
extern float Servo_Output;
extern uint32_t threshold;
extern uint32_t preview;

void main(void)
{
    /** 初始化阶段，关闭总中断 */
    HAL_EnterCritical();

    /** BSP（板级支持包）初始化 */
    RTECLK_HsRun_180MHz();
    RTEPIN_Basic();
    RTEPIN_Digital();
    RTEPIN_Analog();
    RTEPIN_LPUART0_DBG();
    RTEPIN_UART0_WLAN();
    RTEPIP_Basic();
    RTEPIP_Device();

    /** 初始化调试组件 */
    DbgConsole_Init(0U, 921600U, kSerialPort_Uart, CLOCK_GetFreq(kCLOCK_CoreSysClk));
    PRINTF("Welcome to HITSIC !\n");
    PRINTF("GCC %d.%d.%d\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
    cm_backtrace_init("HITSIC_MK66F18", "2020-v3.0", "v4.1.1");

    /** 初始化OLED屏幕 */
    DISP_SSD1306_Init();
    extern const uint8_t DISP_image_100thAnniversary[8][128];
    DISP_SSD1306_BufferUpload((uint8_t*) DISP_image_100thAnniversary);
    DISP_SSD1306_delay_ms(500);
    /** 初始化ftfx_Flash */
    FLASH_SimpleInit();
    /** 初始化PIT中断管理器 */
    pitMgr_t::init();
    /** 初始化I/O中断管理器 */
    extInt_t::init();
    /** 初始化菜单 */
    MENU_Init();
    MENU_Data_NvmReadRegionConfig();
    MENU_Data_NvmRead(menu_currRegionNum);
    /** 菜单挂起 */
    MENU_Suspend();
    /** 初始化摄像头 */
    Cam_Init();
    /** 初始化IMU */
    //TODO: 在这里初始化IMU（MPU6050）
    /** 菜单就绪 */
    MENU_Resume();
    /** 控制环初始化 */
    pitMgr_t::insert(5U, 4U, MOTOR_PWM, pitMgr_t::enable);
    pitMgr_t::insert(20U, 5U, SERVO_PWM, pitMgr_t::enable);
    /**外部中断初始化**/
    PORT_SetPinInterruptConfig(PORTA, 9U, kPORT_InterruptRisingEdge);
    extInt_t::insert(PORTA, 9U,MENU_Suspend);
    PORT_SetPinInterruptConfig(PORTA, 9U, kPORT_InterruptFallingEdge);
    extInt_t::insert(PORTA, 9U,MENU_Resume);
    PORT_SetPinInterruptConfig(PORTA, 11U, kPORT_InterruptLogicZero);
    extInt_t::insert(PORTA, 11U,Update_Servo_Error);
    PORT_SetPinInterruptConfig(PORTA, 13U, kPORT_InterruptLogicZero);
    extInt_t::insert(PORTA, 13U,Cam_Init);
    /** 初始化结束，开启总中断 */
    HAL_ExitCritical();
    Servo_Output = Servo;
    Motor_L_Output = Motor_L;
    Motor_R_Output = Motor_R;
    while(true)
    {

    }
}
void MENU_DataSetUp(void)
{
    MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(nullType, NULL, "", 0, 0));

    /** 子菜单指针初始化 */
    menu_list_t *myList_1 = MENU_ListConstruct(
            "Para_Ctrl",     ///> 菜单标题，在菜单列表中的第一行显示，最大12字符。
            10,             ///> 菜单列表的大小，须预留1位用于返回上一级的[back]。
            menu_menuRoot   ///> 该菜单的上级菜单指针。注意：该指针仅用于返回上级菜单，并不会将子菜单插入上级菜单。
        );
    /** 检查内存分配是否成功 */
        assert(myList_1);
    /** 将子菜单的跳转入口插入其上级菜单 */
        MENU_ListInsert(
            menu_menuRoot,  ///> 要插入的上级菜单。
            MENU_ItemConstruct(
            menuType,   ///> 类型标识，指明这是一个菜单跳转类型的菜单项。
            myList_1,   ///> 数据指针，这里指向要跳转到的菜单列表。
            "Control", ///> 菜单项名称，在菜单列表中显示。
            0,          ///> 数据的保存位置，对于非数据类型填0即可。
            0           ///> 属性Flag，无任何属性填0。
        ));
        {   //这里加这组括号只是为了缩进方便，其内部的语句用于向myList_1插入菜单项。
            MENU_ListInsert(myList_1, MENU_ItemConstruct(
                    varfType,  ///> 类型标识，指明这是一个整数类型的菜单项
                &Motor_L,  ///> 数据指针，这里指向要操作的整数。必须是int32_t类型。
                "Motor_L",   ///> 菜单项名称，在菜单列表中显示。
                10,        ///> 数据的保存地址，不能重复且尽可能连续，步长为1。
                           ///> 全局数据区0~9的地址为保留地址，不能使用。
                menuItem_data_global
                           ///> 属性flag。此flag表示该变量存储于全局数据区
            ));
            MENU_ListInsert(myList_1, MENU_ItemConstruct(
                    varfType,  ///> 类型标识，指明这是一个整数类型的菜单项
                    &Motor_R,  ///> 数据指针，这里指向要操作的整数。必须是int32_t类型。
                    "Motor_R",   ///> 菜单项名称，在菜单列表中显示。
                    11,        ///> 数据的保存地址，不能重复且尽可能连续，步长为1。
                                ///> 全局数据区0~9的地址为保留地址，不能使用。
                    menuItem_data_global
                                ///> 属性flag。此flag表示该变量存储于全局数据区.
            ));
            MENU_ListInsert(myList_1, MENU_ItemConstruct(
                    varfType,  ///> 类型标识，指明这是一个整数类型的菜单项
                    &Servo,  ///> 数据指针，这里指向要操作的整数。必须是int32_t类型。
                    "Servo",   ///> 菜单项名称，在菜单列表中显示。
                    12,        ///> 数据的保存地址，不能重复且尽可能连续，步长为1。
                    ///> 全局数据区0~9的地址为保留地址，不能使用。
                    menuItem_data_global
                    ///> 属性flag。此flag表示该变量存储于全局数据区.
            ));
            MENU_ListInsert(myList_1, MENU_ItemConstruct(
                    varfType,  ///> 类型标识，指明这是一个整数类型的菜单项
                    &Servo_kp,  ///> 数据指针，这里指向要操作的整数。必须是int32_t类型。
                    "Servo_kp",   ///> 菜单项名称，在菜单列表中显示。
                    13,        ///> 数据的保存地址，不能重复且尽可能连续，步长为1。
                    ///> 全局数据区0~9的地址为保留地址，不能使用。
                    menuItem_data_global
                    ///> 属性flag。此flag表示该变量存储于全局数据区.
            ));
            MENU_ListInsert(myList_1, MENU_ItemConstruct(
                    varfType,  ///> 类型标识，指明这是一个整数类型的菜单项
                    &Servo_kd,  ///> 数据指针，这里指向要操作的整数。必须是int32_t类型。
                    "Servo_kd",   ///> 菜单项名称，在菜单列表中显示。
                    14,        ///> 数据的保存地址，不能重复且尽可能连续，步长为1。
                    ///> 全局数据区0~9的地址为保留地址，不能使用。
                    menuItem_data_global
                    ///> 属性flag。此flag表示该变量存储于全局数据区.
            ));
            MENU_ListInsert(myList_1, MENU_ItemConstruct(
                    varfType,  ///> 类型标识，指明这是一个整数类型的菜单项
                    &Servo_ki,  ///> 数据指针，这里指向要操作的整数。必须是int32_t类型。
                    "Servo_ki",   ///> 菜单项名称，在菜单列表中显示。
                    15,        ///> 数据的保存地址，不能重复且尽可能连续，步长为1。
                    ///> 全局数据区0~9的地址为保留地址，不能使用。
                    menuItem_data_global
                    ///> 属性flag。此flag表示该变量存储于全局数据区.
            ));
        }
        menu_list_t *myList_2 = MENU_ListConstruct(
                    "Image",     ///> 菜单标题，在菜单列表中的第一行显示，最大12字符。
                    10,             ///> 菜单列表的大小，须预留1位用于返回上一级的[back]。
                    menu_menuRoot   ///> 该菜单的上级菜单指针。注意：该指针仅用于返回上级菜单，并不会将子菜单插入上级菜单。
                );
        /** 检查内存分配是否成功 */
        assert(myList_2);
        /** 将子菜单的跳转入口插入其上级菜单 */
        MENU_ListInsert(
                menu_menuRoot,  ///> 要插入的上级菜单。
                MENU_ItemConstruct(
                        menuType,   ///> 类型标识，指明这是一个菜单跳转类型的菜单项。
                        myList_2,   ///> 数据指针，这里指向要跳转到的菜单列表。
                        "Image", ///> 菜单项名称，在菜单列表中显示。
                        0,          ///> 数据的保存位置，对于非数据类型填0即可。
                        0           ///> 属性Flag，无任何属性填0。
                ));
        {
            MENU_ListInsert(myList_2, MENU_ItemConstruct(
                    variType,  ///> 类型标识，指明这是一个整数类型的菜单项
                    &threshold,  ///> 数据指针，这里指向要操作的整数。必须是int32_t类型。
                    "threshold",   ///> 菜单项名称，在菜单列表中显示。
                    16,        ///> 数据的保存地址，不能重复且尽可能连续，步长为1。
                    ///> 全局数据区0~9的地址为保留地址，不能使用。
                    menuItem_data_global
                    ///> 属性flag。此flag表示该变量存储于全局数据区.
            ));
            MENU_ListInsert(myList_2, MENU_ItemConstruct(
                    variType,  ///> 类型标识，指明这是一个整数类型的菜单项
                    &preview,  ///> 数据指针，这里指向要操作的整数。必须是int32_t类型。
                    "preview",   ///> 菜单项名称，在菜单列表中显示。
                    17,        ///> 数据的保存地址，不能重复且尽可能连续，步长为1。
                    ///> 全局数据区0~9的地址为保留地址，不能使用。
                    menuItem_data_global
                    ///> 属性flag。此flag表示该变量存储于全局数据区.
            ));
            MENU_ListInsert(myList_2, MENU_ItemConstruct(
                    procType,  ///> 类型标识，指明这是一个浮点类型的菜单项
                    Cam_Test,///> 数据指针，这里指向要操作的整数。必须是float类型。
                    "Cam_Test", ///> 菜单项名称，在菜单列表中显示。
                    0,         ///> 数据的保存地址，不能重复且尽可能连续，步长为1。
                    menuItem_proc_uiDisplay|menuItem_proc_runOnce
                    ///> 属性flag。此flag表示该该程序运行一次就退出。
            ));

        }
        menu_list_t *myList_3 = MENU_ListConstruct(
                "RUN",     ///> 菜单标题，在菜单列表中的第一行显示，最大12字符。
                5,             ///> 菜单列表的大小，须预留1位用于返回上一级的[back]。
                menu_menuRoot   ///> 该菜单的上级菜单指针。注意：该指针仅用于返回上级菜单，并不会将子菜单插入上级菜单。
        );
        /** 检查内存分配是否成功 */
        assert(myList_3);
        /** 将子菜单的跳转入口插入其上级菜单 */
        MENU_ListInsert(
                menu_menuRoot,  ///> 要插入的上级菜单。
                MENU_ItemConstruct(
                        menuType,   ///> 类型标识，指明这是一个菜单跳转类型的菜单项。
                        myList_3,   ///> 数据指针，这里指向要跳转到的菜单列表。
                        "RUN", ///> 菜单项名称，在菜单列表中显示。
                        0,          ///> 数据的保存位置，对于非数据类型填0即可。
                        0           ///> 属性Flag，无任何属性填0。
                ));
        {
            MENU_ListInsert(myList_3, MENU_ItemConstruct(
                    procType,  ///> 类型标识，指明这是一个浮点类型的菜单项
                    Update_Servo_Error,///> 数据指针，这里指向要操作的整数。必须是float类型。
                    "GOGOGO", ///> 菜单项名称，在菜单列表中显示。
                    0,         ///> 数据的保存地址，不能重复且尽可能连续，步长为1。
                    menuItem_proc_uiDisplay
                    ///> 属性flag。此flag表示该该程序运行一次就退出。
            ));
        }

}

/**
 * 『灯千结的碎碎念』 Tips by C.M. :
 * 1. 浮点数计算有时（例如除零时）会产生“nan”，即“非数（Not-a-Number）”。
 *      要检测一个变量是否为“nan”，只需判断这个变量是否和自身相等。如果该
 *      变量与自身不相等（表达式“var == var”的值为假），则可判定该浮点数
 *      的值是nan，需要进行车模保护动作。
 * 2. 由于车模震动等因素，IMU可能会断开连接。一旦发现IMU读取失败，应执行车
 *      模保护动作。另外，IMU在单片机复位的瞬间可能正在进行传输，导致时序
 *      紊乱，初始化失败。因此装有IMU的车模复位时必须全车断电。
 * 3. 正常情况下图像帧率为50FPS，即20ms一帧。若摄像头时序紊乱，会导致控制周
 *      期混乱。因而有必要在每次图像采集完成时测量距离上次图像采集完成的时
 *      间间隔，如果明显偏离20ms，须执行车模保护动作。
 * 4. 直立车需特别注意：有时控制输出会使两个电机向相反方向旋转，这在正常运行
 *      中是十分危险的，可能造成车模进入“原地陀螺旋转”的状态，极易损坏车模或
 *      导致人员受伤。在设置电机占空比时务必做好异常保护。
 */
