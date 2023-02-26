//����һ�����ñ��������в��ٵ�.h�ļ�

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

#define		Tim_time								100	//��������Ϊ100ms

#define		Total_Resolution				1040	//1040Ϊת��ת��һ���������������4��Ƶ��

#define		GPIO_E1B								GPIO_Pin_15	//����E1B����ӦA��
#define		GPIO_E1A								GPIO_Pin_3	//����E1A����ӦB��
#define		GPIO_E2B								GPIO_Pin_6	//����E2B����ӦA��
#define		GPIO_E2A								GPIO_Pin_7	//����E2A����ӦB��
#define		E1B_Channel							TIM_Channel_1	//E1B��ӦTIM2_CH1
#define		E1A_Channel							TIM_Channel_2	//E1A��ӦTIM2_CH2
#define		E2B_Channel							TIM_Channel_1	//E2B��ӦTIM3_CH1
#define		E2A_Channel							TIM_Channel_2	//E2A��ӦTIM3_CH2

#define		Encoder_TIM_Period			0xffff	// ��װ��ֵ�����ɴ���65535 ��ΪF103�Ķ�ʱ����16λ��

#define		Tim_E1B_E1A_IRQn				TIM2_IRQn	//����TIM2�ж�Դ
#define		Tim_E2B_E2A_IRQn				TIM3_IRQn	//����TIM3�ж�Դ

//��������
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
