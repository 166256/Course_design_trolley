/***************************************************
*
* @file         Road.h
* @brief        传感器数据处理文件
* @version      v1.0
* @date         2022/1/18
*
***************************************************/
#ifndef _FILTER_h
#define _FILTER_h
#include "sys.h"
//-----------------------宏-----------------------//
#define Windows 8	//连续获取10个陀螺仪数值
//--------------------全局变量---------------------//

//---------------------结构体---------------------//
struct RC_Para
{
    float temp;  //暂存值,存储RC_baro
    float value; //滤波值
    float RC;    //低通滤波参数
};
typedef struct RC_Para *RC_Filter_pt;

struct Complement2_Para
{
    float Fuse_para;            //融合系数
    float angle_from_gyro;      //陀螺仪解算的角度
    float angle_from_mag;       //电子罗盘解算的角度
    float Fuse_angle;           //数据融合后的姿态角
    float angle_from_mag_last;  //
};
typedef struct Complement2_Para * Com2_Filter_pt;
extern float test_mag;
/***************************************************************************/
float Kalmen_getAngle(float now_angle, float now_rate,float dt);
float FuseAngleCalculate_2(Com2_Filter_pt Filter,float gyro_omega,float angle_mag,float dt);
float Movingaverage_filter(float value,float Filter_buff[],int8_t Filter_num);
float RCFilter(float value,RC_Filter_pt Filter);
float limit_ab(float x, int a, int b);
#endif
