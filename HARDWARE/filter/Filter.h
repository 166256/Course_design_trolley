/***************************************************
*
* @file         Road.h
* @brief        ���������ݴ����ļ�
* @version      v1.0
* @date         2022/1/18
*
***************************************************/
#ifndef _FILTER_h
#define _FILTER_h
#include "sys.h"
//-----------------------��-----------------------//
#define Windows 8	//������ȡ10����������ֵ
//--------------------ȫ�ֱ���---------------------//

//---------------------�ṹ��---------------------//
struct RC_Para
{
    float temp;  //�ݴ�ֵ,�洢RC_baro
    float value; //�˲�ֵ
    float RC;    //��ͨ�˲�����
};
typedef struct RC_Para *RC_Filter_pt;

struct Complement2_Para
{
    float Fuse_para;            //�ں�ϵ��
    float angle_from_gyro;      //�����ǽ���ĽǶ�
    float angle_from_mag;       //�������̽���ĽǶ�
    float Fuse_angle;           //�����ںϺ����̬��
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
