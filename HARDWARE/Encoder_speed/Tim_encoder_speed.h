#ifndef	__TIM_ENCODER_SPEED__
#define	__TIM_ENCODER_SPEED__

#include "stm32f10x.h"
#include <stdio.h>

#define	Tim_time			20		// ��������Ϊ100ms
#define	Total_Resolution	1040	// 1040Ϊת��ת��һ���������������4��Ƶ��
#define	Encoder_TIM_Period	0xffff	// ��װ��ֵ�����ɴ���65535 ��ΪF103�Ķ�ʱ����16λ��

extern volatile int16_t encoderNum_R,encoderNum_L;

//��������
void Tim_EncoderL_Init(void);
void Tim_EncoderR_Init(void);
vu16 Get_Encoder_Cnt(vu16	TIMx);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
int calc_motor_Left_rotate_speed(void);
int calc_motor_Right_rotate_speed(void);
#endif
