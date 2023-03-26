/***************************************************
*
* @file         Filter.c
* @brief        滤波函数
* @version      v1.0
* @date         2022/2/8
*
***************************************************/

#include "Filter.h"

//-----------------------宏-----------------------//


//--------------------全局变量---------------------//

//float Filter_buf[Filter_N + 1] = {0};	//滑动窗口数组

float Q_angle = 0;	//陀螺仪噪声协方差
float Q_bias = 0;	//陀螺仪漂移噪声协方差
float R_angle = 0;	//角度测量噪声协方差
float kal_P[2][2] = {0};
float kal_k[2] = {0};
float kal_angle;	//滤波值
float kal_rate;		//滤波值
float angle_bias;

float cpm_k = 0.5;	//二阶滤波系数
float cpm_angle;//滤波值
float test_mag;

//---------------------结构体---------------------//



//-------------------------------------------------------------------------------------------------------------------
//  @brief      卡尔曼滤波 | yaw角
//  @param      now_angle            应该由加速度计得到angle，再使用陀螺仪进行补偿，但是加速度计得不到yaw角，可能还是得使用磁力计
//  @param      now_rate             陀螺仪得到的角速度
//  @param      dt      			 采样时间
//  @return     kal_angle			 滤波值
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
float Kalmen_getAngle(float now_angle, float now_rate,float dt)
{
    //预测当前角度
    kal_angle = kal_angle - Q_bias * dt + now_rate * dt;

    //预测误差协方差
    kal_P[0][0] += Q_angle - (kal_P[0][1] -kal_P[1][0]) * dt;
    kal_P[0][1] -= kal_P[1][1] * dt;
    kal_P[1][0] -= kal_P[1][1] * dt;
    kal_P[1][1] = kal_P[1][0] + Q_bias;

    //计算卡尔曼增益
    kal_k[0] = kal_P[0][0]/(kal_P[0][0] + R_angle);
    kal_k[1] = kal_P[1][0]/(kal_P[0][0] + R_angle);

    //计算最优估计值
    kal_angle = kal_angle + kal_k[0] * (now_angle - kal_angle);
    Q_bias = Q_bias + kal_k[1] * (now_angle - kal_angle);

    //更新协方差矩阵
    kal_P[0][0] = kal_P[0][0] - kal_k[0] * kal_P[0][0];
    kal_P[0][1] = kal_P[0][1] - kal_k[0] * kal_P[0][1];
    kal_P[1][0] = kal_P[1][0] - kal_k[1] * kal_P[0][0];
    kal_P[1][0] = kal_P[1][0] - kal_k[1] * kal_P[0][1];

    return kal_angle;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      二阶互补滤波
//  @param      Filter     传进来的结构体指针
//  @param      gyro_omega 陀螺仪的角速度
//  @param      angle_mag  电子罗盘解算的yaw
//  @param      dt         积分时间
//  @return     数据融合值
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
float FuseAngleCalculate_2(Com2_Filter_pt Filter,float gyro_omega,float angle_mag,float dt)
{
    float Fuse_P,Fuse_I,Fuse_compensation;     //误差比例项 || 误差积分项 || 数据补偿
    //更新mag值
    Filter->angle_from_mag = angle_mag;
    //从左到右
    if(Filter->angle_from_mag_last < 0.5 && Filter->angle_from_mag > 359.5)
        Filter->Fuse_angle = Filter->angle_from_mag;
    //从右到左
    else if(Filter->angle_from_mag_last > 359.5 && Filter->angle_from_mag < 0.5)
        Filter->Fuse_angle = Filter->angle_from_mag;

    Fuse_P=(Filter->angle_from_mag-Filter->Fuse_angle) * Filter->Fuse_para * Filter->Fuse_para;

    Fuse_I += Fuse_P * dt;
    //误差积分限幅
    Fuse_I = limit_ab(Fuse_I,-1,1);
    Fuse_compensation = Fuse_I + 2 * Filter->Fuse_para * (Filter->angle_from_mag - Filter->Fuse_angle);
    test_mag = Fuse_compensation;
    //融合角为修正后陀螺仪值的积分
    Filter->Fuse_angle += (Fuse_compensation + gyro_omega) * dt * 57.2958f;

    //限幅
    if(Filter->Fuse_angle < 0)
        Filter->Fuse_angle = 0;
    if(Filter->Fuse_angle > 360)
        Filter->Fuse_angle = 360;
    //更新
    Filter->angle_from_mag_last = Filter->angle_from_mag;
    return Filter->Fuse_angle;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      输入一个值弹出一个值，所有值取平均 | 滑动滤波
//  @param		待滤波的值
//  @return
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------

float Movingaverage_filter(float value,float Filter_buff[],int8_t Filter_num)
{
    int8_t i = 0;//遍历
    float temp = value;
    float Filter_sum = 0;
    Filter_buff[Filter_num] = temp;

    for(i = 0; i < Filter_num; i++)
    {
        Filter_buff[i] = Filter_buff[i+1];		//数据左移
        Filter_sum += Filter_buff[i];
    }
    temp = Filter_sum / Filter_num;
    return temp;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      低通滤波
//  @param      待滤波的值
//  @return     滤波值
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------

float RCFilter(float value,RC_Filter_pt Filter)
{
    Filter->temp = value;
    Filter->value = (1 - Filter->RC) * Filter->value + Filter->RC * Filter->temp;
//	temp = RC * value + (1 - RC) * temp;
    return Filter->value;
}

float limit_ab(float x, int a, int b)
{
    if (x < a) x = a;
    if (x > b) x = b;
    return x;
}

