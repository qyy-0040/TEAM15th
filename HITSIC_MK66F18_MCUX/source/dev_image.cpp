/*
 * dev_Image.cpp
 *
 *  Created on: 2020年11月10日
 *      Author: fanyi
 */
/**TEAM 15th Dev**/
#include "dev_Image.h"
uint8_t *imageBuffer0 = new uint8_t[DMADVP0->imgSize];
uint8_t *imageBuffer1 = new uint8_t[DMADVP0->imgSize];
uint8_t *fullBuffer = NULL;
cam_zf9v034_configPacket_t cameraCfg;
dmadvp_config_t dmadvpCfg;
dmadvp_handle_t dmadvpHandle;
int f[10 * CAMERA_H];//考察连通域联通性
//每个白条子属性
typedef struct {
    uint8_t   left;//左边界
    uint8_t   right;//右边界
    int   connect_num;//连通标记
}range;

//每行的所有白条子
typedef struct {
    uint8_t   num;//每行白条数量
    range   area[white_num_MAX];//该行各白条区域
}all_range;

//属于赛道的每个白条子属性
typedef struct {
    uint8_t   left;//左边界
    uint8_t   right;//右边界
    uint8_t   width;//宽度
}road_range;

//每行属于赛道的每个白条子
typedef struct {
    uint8_t   white_num;
    road_range   connected[white_num_MAX];
}road;

all_range white_range[CAMERA_H];//所有白条子
road my_road[CAMERA_H];//赛道
uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
uint8_t left_line[CAMERA_H], right_line[CAMERA_H];//赛道的左右边界
uint8_t mid_line[CAMERA_H];
int all_connect_num = 0;//所有白条子数
uint8_t top_road;//赛道最高处所在行数
uint8_t threshold = 160;//阈值

/////////////////////////////
//新加变量
uint8_t left_cross[CAMERA_H];   //十字路口的左边界
uint8_t right_cross[CAMERA_H];  //十字路口的右边界
uint8_t cross[CAMERA_H] = { -1 };       //十字路口所在行
uint8_t cross_left[2] = { 0,119 };                  //十字路口左右两边的上下边界所在列数
uint8_t cross_right[2] = { 0,119 };             //考虑到特殊情况 上边界设置为0 下边界设置为119
uint8_t ols_leftXi[CAMERA_H];
uint8_t ols_leftYi[CAMERA_H] = { 0 };
uint8_t ols_rightXi[CAMERA_H];
uint8_t ols_rightYi[CAMERA_H] = { 0 };

////////////////////////////////////////////
//功能：二值化
//输入：灰度图片
//输出：二值化图片
//备注：
///////////////////////////////////////////
void THRE()
{
    uint8_t* map;
    uint8_t* my_map;
    map = fullBuffer;
    for (int i = 0; i < 120; i++)
    {
        my_map = &IMG[i][0];
        for (int j = 0; j < 188; j++)
        {
            if ((*map) > threshold)
                (*my_map) = 1;
            else (*my_map) = 0;
            map++;
            my_map++;
        }
    }
}

////////////////////////////////////////////
//功能：粗犷的清车头
//输入：
//输出：
//备注：要根据自己车头的大小进行修改
///////////////////////////////////////////
void head_clear(void)
{
    uint8_t* my_map;
    for (int i = 119; i >= 84; i--)
    {
        my_map = &IMG[i][0];
        for (int j = 40; j <= 135; j++)
        {
            *(my_map+j) = white;
        }
    }
}

////////////////////////////////////////////
//功能：查找父节点
//输入：节点编号
//输出：最老祖先
//备注：含路径压缩
///////////////////////////////////////////
int find_f(int node)
{
    if (f[node] == node)return node;//找到最古老祖先，return
    f[node] = find_f(f[node]);//向上寻找自己的父节点
    return f[node];
}

////////////////////////////////////////////
//功能：提取跳变沿 并对全部白条子标号
//输入：IMG[120][188]
//输出：white_range[120]
//备注：指针提速
///////////////////////////////////////////
void search_white_range()
{
    uint8_t i, j;
    int istart = NEAR_LINE;//处理起始行
    int iend = FAR_LINE;//处理终止行
    int tnum = 0;//当前行白条数
    all_connect_num = 0;//白条编号初始化
    uint8_t* map = NULL;
    for (i = istart; i >= iend; i--)
    {
        map = &IMG[i][LEFT_SIDE];//指针行走加快访问速度
        tnum = 0;
        for (j = LEFT_SIDE; j <= RIGHT_SIDE; j++, map++)
        {
            if ((*map))//遇白条左边界
            {
                tnum++;
                if (tnum >= white_num_MAX)break;
                range* now_white = &white_range[i].area[tnum];
                now_white->left = j;

                //开始向后一个一个像素点找这个白条右边界
                map++;
                j++;

                while ((*map) && j <= RIGHT_SIDE)
                {
                    map++;
                    j++;
                }
                now_white->right = j - 1;
                now_white->connect_num = ++all_connect_num;//白条数加一，给这个白条编号
            }
        }
        white_range[i].num = tnum;
    }
}

////////////////////////////////////////////
//功能：寻找白条子连通性，将全部联通白条子的节点编号刷成最古老祖先的节点编号
//输入：
//输出：
//备注：
///////////////////////////////////////////
void find_all_connect()
{
    //f数组初始化
    for (int i = 1; i <= all_connect_num; i++)
        f[i] = i;

    //u为up d为down 即为当前处理的这两行中的上面那行和下面那行
    //u_num：上面行白条数
    //u_left：上面行当前白条左边界
    //u_right：上面行当前白条右边界
    //i_u：当前处理的这个白条是当前这行（上面行）白条中的第i_u个
    int u_num, i_u, u_left, u_right;
    int d_num, i_d, d_left, d_right;
    all_range* u_white = NULL;
    all_range* d_white = NULL;
    for (int i = NEAR_LINE; i > FAR_LINE; i--)//因为每两行每两行比较 所以循环到FAR_LINE+1
    {
        u_num = white_range[i - 1].num;
        d_num = white_range[i].num;
        u_white = &white_range[i - 1];
        d_white = &white_range[i];
        i_u = 1; i_d = 1;

        //循环到当前行或上面行白条子数耗尽为止
        while (i_u <= u_num && i_d <= d_num)
        {
            //变量先保存，避免下面访问写的冗杂且访问效率低
            u_left = u_white->area[i_u].left;
            u_right = u_white->area[i_u].right;
            d_left = d_white->area[i_d].left;
            d_right = d_white->area[i_d].right;

            if (u_left <= d_right && u_right >= d_left)//如果两个白条联通
                f[find_f(u_white->area[i_u].connect_num)] = find_f(d_white->area[i_d].connect_num);//父节点连起来

            //当前算法规则，手推一下你就知道为啥这样了
            if (d_right > u_right)i_u++;
            if (d_right < u_right)i_d++;
            if (d_right == u_right) { i_u++; i_d++; }
        }
    }
}

////////////////////////////////////////////
//功能：寻找赛道
//输入：
//输出：
//备注：
///////////////////////////////////////////
void find_road()
{
    uint8_t istart = NEAR_LINE;
    uint8_t iend = FAR_LINE;
    top_road = NEAR_LINE;//赛道最高处所在行数，先初始化话为最低处
    int road_f = -1;//赛道所在连通域父节点编号，先初始化为-1，以判断是否找到赛道
    int while_range_num = 0, roud_while_range_num = 0;
    all_range* twhite_range = NULL;
    road* tmy_road = NULL;
    //寻找赛道所在连通域
    // 寻找最中心的白条子
    for (int i = 1; i <= white_range[istart].num; i++)
        if (white_range[istart].area[i].left <= CAMERA_W / 2
            && white_range[istart].area[i].right >= CAMERA_W / 2 && (white_range[istart].area[i].right - white_range[istart].area[i].left) >= 90)
            road_f = find_f(white_range[istart].area[i].connect_num);

    if (road_f == -1)//若赛道没在中间，在113行选一行最长的认为这就是赛道
    {
        int widthmax = 0, jselect = 1;
        for (int i = 1; i <= white_range[istart].num; i++)
            if (white_range[istart].area[i].right - white_range[istart].area[i].left > widthmax)
            {
                widthmax = white_range[istart].area[i].right - white_range[istart].area[i].left;
                jselect = i;
            }
        road_f = find_f(white_range[istart].area[jselect].connect_num);
    }

    //现在我们已经得到了赛道所在连通域父节点编号，接下来把所有父节点编号是road_f的所有白条子扔进赛道数组就行了
    for (int i = istart; i >= iend; i--)
    {
        //变量保存，避免之后写的冗杂且低效
        twhite_range = &white_range[i];
        tmy_road = &my_road[i];
        while_range_num = twhite_range->num;
        tmy_road->white_num = 0;
        roud_while_range_num = 0;
        for (int j = 1; j <= while_range_num; j++)
        {
            if (find_f(twhite_range->area[j].connect_num) == road_f)
            {
                top_road = i;
                tmy_road->white_num++; roud_while_range_num++;
                tmy_road->connected[roud_while_range_num].left = twhite_range->area[j].left;
                tmy_road->connected[roud_while_range_num].right = twhite_range->area[j].right;
                tmy_road->connected[roud_while_range_num].width = twhite_range->area[j].right - twhite_range->area[j].left;

            }
        }
    }
}

////////////////////////////////////////////
//功能：返回相连下一行白条子编号
//输入：i_start起始行  j_start白条标号
//输出：白条标号
//备注：认为下一行与本行赛道重叠部分对多的白条为选定赛道
///////////////////////////////////////////
uint8_t find_continue(uint8_t i_start, uint8_t j_start)
{
    uint8_t j_return;
    uint8_t j;
    uint8_t width_max = 0;
    uint8_t width_new = 0;
    uint8_t left = 0;
    uint8_t right = 0;
    uint8_t dright, dleft, uright, uleft;
    j_return = MISS;//如果没找到，输出255
    if (j_start > my_road[i_start].white_num)
        return MISS;
    //选一个重叠最大的
    for (j = 1; j <= my_road[i_start - 1].white_num; j++)
    {
        dleft = my_road[i_start].connected[j_start].left;
        dright = my_road[i_start].connected[j_start].right;
        uleft = my_road[i_start - 1].connected[j].left;
        uright = my_road[i_start - 1].connected[j].right;
        if (//相连
            dleft < uright
            &&
            dright > uleft
            )
        {
            //计算重叠大小
            if (dleft < uleft) left = uleft;
            else left = dleft;

            if (dright > uright) right = uright;
            else right = dright;

            width_new = right - left + 1;

            if (width_new > width_max)
            {
                width_max = width_new;
                j_return = j;
            }
        }

    }
    return j_return;
}

////////////////////////////////////////////
//功能：通用决定双边
//输入：
//输出：
//备注：
///////////////////////////////////////////
void ordinary_two_line(void)
{
    uint8_t i;
    uint8_t j;
    uint8_t j_continue[CAMERA_H];//第一条连通路径
    uint8_t i_start;
    uint8_t i_end;
    uint8_t j_start = MISS;
    int width_max;

    //寻找起始行最宽的白条子
    i_start = NEAR_LINE;
    i_end = FAR_LINE;
    width_max = 0;
    for (j = 1; j <= my_road[i_start].white_num; j++)
    {
        if (my_road[i_start].connected[j].width > width_max)
        {
            width_max = my_road[i_start].connected[j].width;
            j_start = j;
        }
    }
    j_continue[i_start] = j_start;

    //记录连贯区域编号
    for (i = i_start; i > i_end; i--)
    {
        //如果相连编号大于该行白条数，非正常，从此之后都MISS
        if (j_continue[i] > my_road[i].white_num)
        {
            j_continue[i - 1] = MISS;
        }
        else
        {
            j_continue[i - 1] = find_continue(i, j_continue[i]);
        }

    }

    //全部初始化为MISS
    my_memset(left_line, MISS, CAMERA_H);
    my_memset(right_line, MISS, CAMERA_H);


    for (i = i_start; i > i_end; i--)
    {
        if (j_continue[i] <= my_road[i].white_num)
        {
            left_line[i] = my_road[i].connected[j_continue[i]].left;
            right_line[i] = my_road[i].connected[j_continue[i]].right;
            IMG[i][left_line[i]] = blue;
            IMG[i][right_line[i]] = red;
        }
        else
        {
            left_line[i] = MISS;
            right_line[i] = MISS;
        }
    }
}

////////////////////////////////////////////
//功能：数组初始化
//输入：uint8_t* ptr 数组首地址, uint8_t num初始化的值, uint8_t size数组大小
//输出：
//备注：因为k66库中认为memset函数不安全，所以无法使用；因此需要自己写一个my_memset
///////////////////////////////////////////
void my_memset(uint8_t* ptr, uint8_t num, uint8_t size)
{
    uint8_t* p = ptr;
    uint8_t my_num = num;
    uint8_t Size = size;
    for (int i = 0; i < Size; i++, p++)
    {
        *p = my_num;
    }
}
////////////////////////////////////////////
//功能：中线合成
//输入：左右边界
//输出：中线
//备注：
///////////////////////////////////////////
void get_mid_line(void)
{
    my_memset(mid_line, MISS, CAMERA_H);
    for(int i = NEAR_LINE;i >= FAR_LINE;i--)
        if (left_line[i] != MISS)
        {
            mid_line[i] = (left_line[i] + right_line[i]) / 2;
        }
        else
        {
            mid_line[i] = MISS;
        }

}
////////////////////////////////////////////
//功能：图像处理主程序
//输入：
//输出：
//备注：
///////////////////////////////////////////
void image_main()
{
    search_white_range();
    find_all_connect();
    find_road();
    /*到此处为止，我们已经得到了属于赛道的结构体数组my_road[CAMERA_H]*/
    ordinary_two_line();
    get_mid_line();

    for (int i = NEAR_LINE; i >= FAR_LINE; i--)
        if (mid_line[i] != MISS)
            IMG[i][mid_line[i]] = red;
}
bool cam_test_status = true;
void Cam_Test_Over(void)
{
    DISP_SSD1306_Fill(0);
    cam_test_status = false;
}
void Cam_Test(void)
{
    MENU_Suspend();
    //初始化部分：
    cam_zf9v034_configPacket_t cameraCfg;
    CAM_ZF9V034_GetDefaultConfig(&cameraCfg);                                   //设置摄像头配置
    CAM_ZF9V034_CfgWrite(&cameraCfg);                                   //写入配置
    dmadvp_config_t dmadvpCfg;
    CAM_ZF9V034_GetReceiverConfig(&dmadvpCfg, &cameraCfg);    //生成对应接收器的配置数据，使用此数据初始化接受器并接收图像数据。
    DMADVP_Init(DMADVP0, &dmadvpCfg);
    dmadvp_handle_t dmadvpHandle;
    DMADVP_TransferCreateHandle(&dmadvpHandle, DMADVP0, CAM_ZF9V034_UnitTestDmaCallback);
    uint8_t *imageBuffer0 = new uint8_t[DMADVP0->imgSize];
    uint8_t *imageBuffer1 = new uint8_t[DMADVP0->imgSize];
    uint8_t *fullBuffer = NULL;
    disp_ssd1306_frameBuffer_t *dispBuffer = new disp_ssd1306_frameBuffer_t;
    DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, imageBuffer0);
    DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, imageBuffer1);
    DMADVP_TransferStart(DMADVP0, &dmadvpHandle);


    while(true)
    {

        while (kStatus_Success != DMADVP_TransferGetFullBuffer(DMADVP0, &dmadvpHandle, &fullBuffer));
        dispBuffer->Clear();
        const uint8_t imageTH = 100;
        for (int i = 0; i < cameraCfg.imageRow; i += 2)
        {
            int16_t imageRow = i >> 1;//除以2 为了加速;
            int16_t dispRow = (imageRow / 8) + 1, dispShift = (imageRow % 8);
            for (int j = 0; j < cameraCfg.imageCol; j += 2)
            {
                int16_t dispCol = j >> 1;
                if (fullBuffer[i * cameraCfg.imageCol + j] > imageTH)
                {
                    dispBuffer->SetPixelColor(dispCol, imageRow, 1);
                }
            }
        }
        DISP_SSD1306_BufferUpload((uint8_t*) dispBuffer);
        DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, fullBuffer);
        //PORT_SetPinInterruptConfig(PORTE, 10U, kPORT_InterruptLogicZero);
        //extInt_t::insert(PORTE, 10U,Cam_Test_Over);
        MENU_Resume();
    }
}
void Cam_Test_1(menu_keyOp_t *_op)
{

}
void Cam_Init(void)
{

}

////////////////////////////////////////////
//新增函数

void check()
{
    printf("cross_left:%d %d\n", cross_left[0], cross_left[1]);
    printf("cross_right:%d %d\n", cross_right[0], cross_right[1]);
    printf("left_line:\n");
    for (int i = FAR_LINE; i <= NEAR_LINE; i++) {
        printf(" %d\t",  left_line[i]);
        if (i % 10 == 0)printf("\n");
    }
    printf("right_line:\n");
    for (int i = FAR_LINE; i <= NEAR_LINE; i++) {
        printf(" %d\t",  right_line[i]);
        if (i % 10 == 0)printf("\n");
    }
}
///////////////////////////////////////////////////////////////////////////
//十字路口左边界
//输入
//输出
//
///////////////////////////////////////////////////////////////////////////
int find_crossLeft() {
    int schange = 0;//是否存在突变
    for (int i = 3; i <CAMERA_H ; i++) {

        //对于左边界来说 数值突然变大 十字路口下端 突然变小 十字路口上端
        if ((left_line[i] - left_line[i - 1] > 3) && (left_line[i] - left_line[i - 2] > 3 )&& (left_line[i] - left_line[i - 3] > 3))
        {
            schange = 1;
            cross_left[1] = i;
        }
        if ((left_line[i] - left_line[i - 1] < -3) && (left_line[i] - left_line[i - 2] < -3) && (left_line[i] - left_line[i - 3] < -3))
        {
            schange = 1;
            cross_left[0] = i;
        }
    }
    //如果下边界比上边界小，令下边界为119
    if (cross_left[1] < cross_left[0]) cross_left[1] = 119;
    if (schange == 1)       return 1;//存在突变
    if (schange == 0)   return 0;   //nope
    /// //


}

///////////////////////////////////////////////////////////////////////////
//十字路口右边界
//输入
//输出
//
///////////////////////////////////////////////////////////////////////////
int find_crossRight() {
    int schange = 0;//是否存在突变
    for (int i = 3; i < CAMERA_H; i++) {

        //对于右边界来说 数值突然变大 十字路口上端 突然变小 十字路口下端
        if (right_line[i] - right_line[i - 1] > 5 && right_line[i] - right_line[i - 2] > 5 && right_line[i] - right_line[i - 3] > 5)
        {
            schange = 1;
            cross_right[0] = i;
        }
        if (right_line[i] - right_line[i - 1] < -5 && right_line[i] - right_line[i - 2] < -5 && right_line[i] - right_line[i - 3] < -5)
        {
            schange = 1;
            cross_right[1] = i;
        }
    }
    //如果下边界比上边界小，令下边界为119
    if (cross_right[1] < cross_right[0]) cross_right[1] = 119;
    if (schange == 1)       return 1;//存在突变
    if (schange == 0)   return 0;   //nope
}

///////////////////////////////////////////////////////
//生成最小二乘法的参考数组xi yi 以及需要拟合的数组cross[CAMERA_H]
//输入 (为了左右两边不用写两个函数，range为cross_left[2]/cross_right[2]；side 为 left_line/right_side
//输出
//
///////////////////////////////////////////////////////
void ols_generateData(uint8_t range[], uint8_t side[], uint8_t xi[], uint8_t yi[])
{
    //为了之后方便判断数组长度，先将xi yi 全部初始化为-1
    for (int i = 0; i < CAMERA_H; i++) {
        xi[i] = -1;
    }
    for (int i = 0; i < CAMERA_H; i++) {
        yi[i] = -1;
    }
    int j = 0;//xi的第j个元素
    //初始化xi xi的存放的数值是行数
    for (int i = range[0] - 20; i < range[0] - 5; i++) {
        if (i >= 0) {
            xi[j] = i;
            j++;
        }
    }
    for (int i = range[1] + 7; i < range[i] + 30; i++)
    {
        if (i <= 119) {
            xi[j] = i;
            j++;
        }
    }
    //初始化yi yi是xi对应位置当中那一行边界值所在列数
    for (int i = 0; xi[i] >= 0 && i <= 119; i++) {
        yi[i] = side[xi[i]];
    }
    return;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////
//最小二乘法的函数实现
//输入：已知的 xi(ols_leftXi,ols_rightXi)  ;yi(ols_leftYi,ols_rightYi),
//      range 即cross_left[2]/cross_right[2],    line 即 left_line/right_line
//输出：
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ols_fitImg(uint8_t range[], uint8_t side[], uint8_t xi[], uint8_t yi[])
{
    int midvar1 = 0;
    int midvar2 = 0;//两个没有感情的中间变量
    float k;
    //k
    for (int i = 0; xi[i] != -1 && i <= 119; i++) { //求出曲线斜率
        midvar1 += 2 * xi[i] * yi[i];
        midvar2 += xi[i] * xi[i];
    }
    k = (float)midvar1 / (float)midvar2;
    //renew side
    for (int i = range[0]; i < range[1]; i++) { //拟合边界值
        side[i] = (int)(k * i);
    }
    return;
}

/////////////////////////////////////////////////////////////////////////////////
//功能：检测到十字路口开始拟合赛道
//输入：
//输出：
//
/////////////////////////////////////////////////////////////////////////////////
void ols_road()
{
    int flag1 = find_crossLeft();
    int flag2 = find_crossRight();
    if (flag1 == 1 && flag2 == 1) {//检测到十字路口 开始拟合赛道
        ols_generateData(cross_left, left_line, ols_leftXi, ols_leftYi);
        ols_generateData(cross_right, right_line, ols_rightXi, ols_rightYi);
        ols_fitImg(cross_left, left_line, ols_leftXi, ols_leftYi);
        ols_fitImg(cross_right, right_line, ols_rightXi, ols_rightYi);

    }


    return;
}
