//这是一个利用Tim产生PWM信号程序的头文件

#ifndef		TIM_PWM
#define		TIM_PWM

#include "stm32f10x.h"

//#define	Tim_GPIO_CLK	RCC_APB2Periph_GPIOA
//#define	Tim_CLK			RCC_APB2Periph_TIM1	

//#define GPIOx			GPIOA
//#define	GPIO_Pin1		GPIO_Pin_8
//#define	GPIO_Pin2		GPIO_Pin_11

//#define	GPIO_Pin_AIN1	GPIO_Pin_0
//#define	GPIO_Pin_AIN2	GPIO_Pin_1
//#define	GPIO_Pin_BIN1	GPIO_Pin_4
//#define	GPIO_Pin_BIN2	GPIO_Pin_5

//函数声明
void Tim_gpio_config(void);
void PWM_Start(TIM_TypeDef* timer,uint8_t tim_channel,uint8_t freq,uint8_t duty);
void Steering_Control_Forward(void);
void Steering_Control_Reversal(void);
void Steering_Control_Stop(void);

#endif
