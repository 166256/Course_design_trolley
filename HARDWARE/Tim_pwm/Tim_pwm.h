//����һ������Tim����PWM�źų����ͷ�ļ�

#ifndef		TIM_PWM
#define		TIM_PWM

#include "stm32f10x.h"

extern unsigned char tim1_num1;
extern unsigned char tim1_num2;
extern unsigned int tim1_num3;		// �����ж�S��
extern unsigned char tim1_flag;
extern unsigned char judge_S;

//��������
void tim4_gpio_config(void);
void PWM_Start(TIM_TypeDef* timer,uint8_t tim_channel,uint8_t freq,uint8_t duty);
void tim1_init(u16 psc,u16 arr);

#endif
