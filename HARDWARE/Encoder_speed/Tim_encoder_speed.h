#ifndef	__TIM_ENCODER_SPEED__
#define	__TIM_ENCODER_SPEED__

#include "stm32f10x.h"
#include <stdio.h>

#define	Tim_time			100		// 测速周期为100ms
#define	Total_Resolution	1040	// 1040为转轮转过一周输出的脉冲数（4倍频后）
#define	Encoder_TIM_Period	0xffff	// 重装载值，不可大于65535 因为F103的定时器是16位的

extern volatile int16_t encoderNum_L;			
extern volatile int16_t encoderNum_R;

//函数声明
void Tim_EncoderL_Init(void);
void Tim_EncoderR_Init(void);
vu16 Get_Encoder_Cnt(vu16	TIMx);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
float calc_motor_Left_rotate_speed(void);
float calc_motor_Right_rotate_speed(void);
#endif
