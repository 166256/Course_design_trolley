/***************************************************
*
* @file         Filter.c
* @brief        �˲�����
* @version      v1.0
* @date         2022/2/8
*
***************************************************/

#include "Filter.h"

//-----------------------��-----------------------//


//--------------------ȫ�ֱ���---------------------//

//float Filter_buf[Filter_N + 1] = {0};	//������������

float Q_angle = 0;	//����������Э����
float Q_bias = 0;	//������Ư������Э����
float R_angle = 0;	//�ǶȲ�������Э����
float kal_P[2][2] = {0};
float kal_k[2] = {0};
float kal_angle;	//�˲�ֵ
float kal_rate;		//�˲�ֵ
float angle_bias;

float cpm_k = 0.5;	//�����˲�ϵ��
float cpm_angle;//�˲�ֵ
float test_mag;

//---------------------�ṹ��---------------------//



//-------------------------------------------------------------------------------------------------------------------
//  @brief      �������˲� | yaw��
//  @param      now_angle            Ӧ���ɼ��ٶȼƵõ�angle����ʹ�������ǽ��в��������Ǽ��ٶȼƵò���yaw�ǣ����ܻ��ǵ�ʹ�ô�����
//  @param      now_rate             �����ǵõ��Ľ��ٶ�
//  @param      dt      			 ����ʱ��
//  @return     kal_angle			 �˲�ֵ
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
float Kalmen_getAngle(float now_angle, float now_rate,float dt)
{
    //Ԥ�⵱ǰ�Ƕ�
    kal_angle = kal_angle - Q_bias * dt + now_rate * dt;

    //Ԥ�����Э����
    kal_P[0][0] += Q_angle - (kal_P[0][1] -kal_P[1][0]) * dt;
    kal_P[0][1] -= kal_P[1][1] * dt;
    kal_P[1][0] -= kal_P[1][1] * dt;
    kal_P[1][1] = kal_P[1][0] + Q_bias;

    //���㿨��������
    kal_k[0] = kal_P[0][0]/(kal_P[0][0] + R_angle);
    kal_k[1] = kal_P[1][0]/(kal_P[0][0] + R_angle);

    //�������Ź���ֵ
    kal_angle = kal_angle + kal_k[0] * (now_angle - kal_angle);
    Q_bias = Q_bias + kal_k[1] * (now_angle - kal_angle);

    //����Э�������
    kal_P[0][0] = kal_P[0][0] - kal_k[0] * kal_P[0][0];
    kal_P[0][1] = kal_P[0][1] - kal_k[0] * kal_P[0][1];
    kal_P[1][0] = kal_P[1][0] - kal_k[1] * kal_P[0][0];
    kal_P[1][0] = kal_P[1][0] - kal_k[1] * kal_P[0][1];

    return kal_angle;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���׻����˲�
//  @param      Filter     �������Ľṹ��ָ��
//  @param      gyro_omega �����ǵĽ��ٶ�
//  @param      angle_mag  �������̽����yaw
//  @param      dt         ����ʱ��
//  @return     �����ں�ֵ
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
float FuseAngleCalculate_2(Com2_Filter_pt Filter,float gyro_omega,float angle_mag,float dt)
{
    float Fuse_P,Fuse_I,Fuse_compensation;     //�������� || �������� || ���ݲ���
    //����magֵ
    Filter->angle_from_mag = angle_mag;
    //������
    if(Filter->angle_from_mag_last < 0.5 && Filter->angle_from_mag > 359.5)
        Filter->Fuse_angle = Filter->angle_from_mag;
    //���ҵ���
    else if(Filter->angle_from_mag_last > 359.5 && Filter->angle_from_mag < 0.5)
        Filter->Fuse_angle = Filter->angle_from_mag;

    Fuse_P=(Filter->angle_from_mag-Filter->Fuse_angle) * Filter->Fuse_para * Filter->Fuse_para;

    Fuse_I += Fuse_P * dt;
    //�������޷�
    Fuse_I = limit_ab(Fuse_I,-1,1);
    Fuse_compensation = Fuse_I + 2 * Filter->Fuse_para * (Filter->angle_from_mag - Filter->Fuse_angle);
    test_mag = Fuse_compensation;
    //�ںϽ�Ϊ������������ֵ�Ļ���
    Filter->Fuse_angle += (Fuse_compensation + gyro_omega) * dt * 57.2958f;

    //�޷�
    if(Filter->Fuse_angle < 0)
        Filter->Fuse_angle = 0;
    if(Filter->Fuse_angle > 360)
        Filter->Fuse_angle = 360;
    //����
    Filter->angle_from_mag_last = Filter->angle_from_mag;
    return Filter->Fuse_angle;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����һ��ֵ����һ��ֵ������ֵȡƽ�� | �����˲�
//  @param		���˲���ֵ
//  @return
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------

float Movingaverage_filter(float value,float Filter_buff[],int8_t Filter_num)
{
    int8_t i = 0;//����
    float temp = value;
    float Filter_sum = 0;
    Filter_buff[Filter_num] = temp;

    for(i = 0; i < Filter_num; i++)
    {
        Filter_buff[i] = Filter_buff[i+1];		//��������
        Filter_sum += Filter_buff[i];
    }
    temp = Filter_sum / Filter_num;
    return temp;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ͨ�˲�
//  @param      ���˲���ֵ
//  @return     �˲�ֵ
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

