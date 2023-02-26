//这是一个利用编码器进行测速的.h文件

#ifndef		TIM_ENCODER_SPEED
#define		TIM_ENCODER_SPEED

#include	"stm32f10x.h"
#include 	<stdio.h>

//#define		Tim_Encoder_GPIO_CLK1		RCC_APB2Periph_GPIOA		
//#define		Tim_Encoder_GPIO_CLK2		RCC_APB2Periph_GPIOB

//#define		Tim_Encoder1_CLK				RCC_APB1Periph_TIM2
//#define		Tim_Encoder2_CLK				RCC_APB1Periph_TIM3

//#define		Encoder_GPIO1					GPIOA
#define		Encoder_GPIO2					GPIOB
#define		Tim_E1B_E1A							TIM2
#define		Tim_E2B_E2A							TIM3
#define		Timx_timing							TIM2

#define		Tim_time								100	//测速周期为100ms

#define		Total_Resolution				1040	//1040为转轮转过一周输出的脉冲数（4倍频后）

#define		GPIO_E1B								GPIO_Pin_15	//左轮E1B，对应A相
#define		GPIO_E1A								GPIO_Pin_3	//左轮E1A，对应B相
#define		GPIO_E2B								GPIO_Pin_6	//右轮E2B，对应A相
#define		GPIO_E2A								GPIO_Pin_7	//右轮E2A，对应B相
#define		E1B_Channel							TIM_Channel_1	//E1B对应TIM2_CH1
#define		E1A_Channel							TIM_Channel_2	//E1A对应TIM2_CH2
#define		E2B_Channel							TIM_Channel_1	//E2B对应TIM3_CH1
#define		E2A_Channel							TIM_Channel_2	//E2A对应TIM3_CH2

#define		Encoder_TIM_Period			0xffff	// 重装载值，不可大于65535 因为F103的定时器是16位的

#define		Tim_E1B_E1A_IRQn				TIM2_IRQn	//定义TIM2中断源
#define		Tim_E2B_E2A_IRQn				TIM3_IRQn	//定义TIM3中断源

//函数声明
void	Tim_E1B_E1A_Config(void);
void	Tim_E2B_E2A_Config(void);
vu16	Get_Encoder_Cnt(vu16	TIMx);
void	TIM2_IRQHandler(void);
void	TIM3_IRQHandler(void);
void	calc_motor_Left_rotate_speed(void);
void	calc_motor_Right_rotate_speed(void);
//void SysTick_Init(void);
//void SysTick_Handler(void);
//void	TIMx_Timing(void);
//void delay_ms(u16 nms);
//void delay_us(u16 nus);
#endif
