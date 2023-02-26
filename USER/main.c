#include "stm32f10x.h" 
#include "sys.h"
#include "delay.h" 
#include "led.h"
#include "oled.h"
#include "usart.h"

#include "Tim_encoder_speed.h"
#include "Tim_pwm.h"

int main (void)
{
	delay_init();				// ��ʱ������ʼ��
	RCC_Configuration();		// ϵͳʱ�ӳ�ʼ��
	LED_Init();					// LED��ʼ��
	OLED_Init();				// OLED��ʼ��
	delay_ms(1000);				// �ȴ������豸��ʼ������
//	uart1_init(115200);
//	Tim_gpio_config();
//	PWM_Start(TIM1,1,10,20);	//����
//	PWM_Start(TIM1,4,10,20);	//����
//	Steering_Control_Forward();	//����ת��Ϊ��
//	Tim_E2B_E2A_Config();		//���ֱ�����
//	Tim_E1B_E1A_Config();		//���ֱ�����
/**********************************************/

	OLED_ShowString(0,0,"Hellow Word");
	while(1)
	{
//		USART_SendData(USART1,0x05);
		delay_ms(500);	
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);
		delay_ms(500);	
		GPIO_SetBits(GPIOA,GPIO_Pin_4);

	}	
}	


