#include "dev_control.hpp"

/**控制环PITMGR任务句柄**/
pitMgr_t* ctrl_spdCtrlHandle = nullptr;
pitMgr_t* ctrl_dirCtrlHandle = nullptr;

/**控制环初始化**/
void CTRL_Init(void)
{
    ctrl_dirCtrlHandle = pitMgr_t::insert(CTRL_DIR_CTRL_MS, 2U, CTRL_DirCtrl, pitMgr_t::enable);
    assert(ctrl_dirCtrlHandle);
    ctrl_spdCtrlHandle = pitMgr_t::insert(CTRL_SPD_CTRL_MS, 3U, CTRL_SpdCtrl, pitMgr_t::enable);
    assert(ctrl_spdCtrlHandle);
}

/**菜单项初始化**/
void CTRL_MenuInit(menu_list_t *menuList)
{
    static menu_list_t *ctrlMenuList =
            MENU_ListConstruct("Control", 32, menuList);
    assert(ctrlMenuList);
    MENU_ListInsert(menuList, MENU_ItemConstruct(menuType, ctrlMenuList, "Control", 0, 0));

    {
        MENU_ListInsert(ctrlMenuList, MENU_ItemConstruct(nullType, NULL, "ENB", 0, 0));

        MENU_ListInsert(ctrlMenuList, MENU_ItemConstruct(variType, &ctrl_spdCtrlEn[0], "spd.en", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad | menuItem_dataExt_HasMinMax));
        MENU_ListInsert(ctrlMenuList, MENU_ItemConstruct(variType, &ctrl_dirCtrlEn[0], "dir.en", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad | menuItem_dataExt_HasMinMax));

        MENU_ListInsert(ctrlMenuList, MENU_ItemConstruct(nullType, NULL, "SPD", 0, 0));

        MENU_ListInsert(ctrlMenuList, MENU_ItemConstruct(varfType, &ctrl_motorL, "MotorL", 11U,
                menuItem_data_region));
        MENU_ListInsert(ctrlMenuList, MENU_ItemConstruct(varfType, &ctrl_motorR, "MotorR", 12U,
                menuItem_data_region)); //速度闭环后就删掉
        /*MENU_ListInsert(ctrlMenuList, MENU_ItemConstruct(varfType, &ctrl_spdSet, "spdSet", 11U,
                menuItem_data_region));
        MENU_ListInsert(ctrlMenuList, MENU_ItemConstruct(varfType, &ctrl_spdPid.kp, "spd.kp", 13U,
                menuItem_data_region));
        MENU_ListInsert(ctrlMenuList, MENU_ItemConstruct(varfType, &ctrl_spdPid.ki, "spd.ki", 14U,
                menuItem_data_region));
        MENU_ListInsert(ctrlMenuList, MENU_ItemConstruct(varfType, &ctrl_spdPid.kd, "spd.kd", 15U,
                menuItem_data_region));
        MENU_ListInsert(ctrlMenuList, MENU_ItemConstruct(varfType, &ctrl_spdPidOutput, "spd.out", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));*/

        MENU_ListInsert(ctrlMenuList, MENU_ItemConstruct(nullType, NULL, "DIR", 0, 0));

        MENU_ListInsert(ctrlMenuList, MENU_ItemConstruct(varfType, &ctrl_dirPid.kp, "dir.kp", 20U,
                menuItem_data_region));
        MENU_ListInsert(ctrlMenuList, MENU_ItemConstruct(varfType, &ctrl_dirPid.ki, "dir.ki", 21U,
                menuItem_data_region));
        MENU_ListInsert(ctrlMenuList, MENU_ItemConstruct(varfType, &ctrl_dirPid.kd, "dir.kd", 22U,
                menuItem_data_region));
        MENU_ListInsert(ctrlMenuList, MENU_ItemConstruct(varfType, &ctrl_dirPidOutput, "dir.out", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
    }

    static menu_list_t *imageMenuList =
                MENU_ListConstruct("Image", 10, menuList);
    assert(imageMenuList);
    MENU_ListInsert(menuList, MENU_ItemConstruct(menuType, imageMenuList, "Image", 0, 0));
    {
        MENU_ListInsert(ctrlMenuList, MENU_ItemConstruct(variType, &ctrl_imgCtrlEn[0], "img.en", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad | menuItem_dataExt_HasMinMax));
        MENU_ListInsert(imageMenuList, MENU_ItemConstruct(variType, &threshold, "threshold", 25U,
                menuItem_data_global));
        MENU_ListInsert(imageMenuList, MENU_ItemConstruct(variType, &preview, "preview", 26U,
                menuItem_data_global));
        MENU_ListInsert(imageMenuList, MENU_ItemConstruct(varfType, &mid_line[preview], "cur", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
    }
    static menu_list_t *paraMenuList =
                MENU_ListConstruct("Parameter", 10, menuList);
    assert(paraMenuList);
    MENU_ListInsert(menuList, MENU_ItemConstruct(menuType, paraMenuList, "Parameter", 0, 0));
    {
        MENU_ListInsert(paraMenuList, MENU_ItemConstruct(varfType, &ctrl_dirPidOutput, "dir.out", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(paraMenuList, MENU_ItemConstruct(varfType, &mid_line[preview], "cur", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
    }
}


/* ******************** 速度环 ******************** */
/*int32_t ctrl_spdCtrlEn[3] = {0, 0, 1}; ///< 速度环使能

float ctrl_spdSet = 0.0f; ///< 速度设置

pidCtrl_t ctrl_spdPid =
{
    .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
    .errCurr = 0.0f, .errIntg = 0.0f, .errDiff = 0.0f, .errPrev = 0.0f,
};

float ctrl_spdL = 0.0f, ctrl_spdR = 0.0f;
float ctrl_spdAvg = 0.0f;

float ctrl_spdPidOutput = 0.0f; ///< 速度环输出

void CTRL_SpdCtrl(void *userData)
{
    ctrl_spdL = ((float)SCFTM_GetSpeed(ENCO_L_PERIPHERAL)) * CTRL_ENCO_SPD_COEFF;
    SCFTM_ClearSpeed(ENCO_L_PERIPHERAL);
    ctrl_spdR = ((float)SCFTM_GetSpeed(ENCO_R_PERIPHERAL)) * CTRL_ENCO_SPD_COEFF;
    SCFTM_ClearSpeed(ENCO_R_PERIPHERAL);
    ctrl_spdAvg = (ctrl_spdL + ctrl_spdR) / 2.0f;
    if(1 == ctrl_spdCtrlEn[0])
    {
        PIDCTRL_ErrUpdate(&ctrl_spdPid, ctrl_spdAvg - ctrl_spdSet);
        ctrl_spdPidOutput = PIDCTRL_CalcPIDGain(&ctrl_spdPid);
    }
    else
    {
        ctrl_spdPidOutput = 0.0f;
    }
}*/

/* *********************************************** */

/* ******************** 转向环 ******************** */
int32_t ctrl_dirCtrlEn[3] = {0, 0, 1}; ///< 转向环使能

pidCtrl_t ctrl_dirPid =
{
    .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
    .errCurr = 0.0f, .errIntg = 0.0f, .errDiff = 0.0f, .errPrev = 0.0f,
};

float ctrl_sevromid;            ///<舵机中值
float ctrl_dirPidOutput = 0.0f; ///< 转向环输出

void CTRL_DirCtrl(float *err)
{
    if(1 == ctrl_dirCtrlEn[0])
    {
        PIDCTRL_ErrUpdate(&ctrl_dirPid, (*err));
        ctrl_dirPidOutput = ctrl_sevromid + PIDCTRL_CalcPIDGain(&ctrl_dirPid);
    }
    else
    {
        ctrl_dirPidOutput = ctrl_sevromid;
    }
    SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50U,Servo_Output);
}

/* *********************************************** */
//速度闭环后再启用
float ctrl_motorL = 0.0f, ctrl_motorR = 0.0f;

void CTRL_MotorUpdate(float motorL, float motorR)
{
    /** 左电机满载 **/
    if(motorL > 100.0f)
    {
        motorR -= (motorL - 100.0f);
        motorL = 100.0f;
    }
    if(motorL < -100.0f)
    {
        motorR -= (motorL + 100.0f);
        motorL = -100.0f;
    }
    /** 右电机满载 **/
    if(motorR > 100.0f)
    {
        motorL -= (motorL - 100.0f);
        motorR = 100.0f;
    }
    if(motorR < -100.0f)
    {
        motorL -= (motorL + 100.0f);
        motorR = -100.0f;
    }
    /** 反转保护 **/
    if(motorL < 0.0f && motorR > 0.0f)
    {
        motorL = 0.0f;
    }
    if(motorL > 0.0f && motorR < 0.0f)
    {
        motorR = 0.0f;
    }

    if(ctrl_motorLQ > 0)
    {
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_0, 20000U, ctrl_motorL);
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_1, 20000U, 0.0f);
    }
    else
    {
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_0, 20000U, 0.0f);
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_1, 20000U, -ctrl_motorL);
    }

    if(ctrl_motorRQ > 0)
    {
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_2, 20000U, ctrl_motorR);
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_3, 20000U, 0.0f);
    }
    else
    {
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_2, 20000U, 0.0f);
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_3, 20000U, -ctrl_motorR);
    }
}


