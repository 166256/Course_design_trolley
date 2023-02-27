#include "stm32f10x.h" 
#include "sys.h"
#include "delay.h" 
#include "led.h"
#include "oled.h"
#include "usart.h"
#include "tcrt500l.h"

#include "Tim_encoder_speed.h"
#include "Tim_pwm.h"

char L2=1,L1=1,R1=1,R2=1;

int main (void)
{
	delay_init();				// ��ʱ������ʼ��
	RCC_Configuration();		// ϵͳʱ�ӳ�ʼ��
	LED_Init();					// LED��ʼ��
	OLED_Init();				// OLED��ʼ��
	delay_ms(1000);				// �ȴ������豸��ʼ������
	tcrt500l_init();
//	uart1_init(115200);
	Tim_gpio_config();
	PWM_Start(TIM4,1,7,50);		//����
	PWM_Start(TIM4,2,7,0);	//����
	PWM_Start(TIM4,3,7,50);	//����
	PWM_Start(TIM4,4,7,0);	//����
//	Steering_Control_Forward();	//����ת��Ϊ��
//	Tim_E2B_E2A_Config();		//���ֱ�����
//	Tim_E1B_E1A_Config();		//���ֱ�����
/**********************************************/

	OLED_ShowString(0,0,"TCRT_L2:");
	OLED_ShowString(0,2,"TCRT_L1:");
	OLED_ShowString(0,4,"TCRT_R1:");
	OLED_ShowString(0,6,"TCRT_R2:");
	
	TIM_SetCompare1(TIM4,2000);
	TIM_SetCompare2(TIM4,0);
	TIM_SetCompare3(TIM4,2000);
	TIM_SetCompare4(TIM4,0);
	while(1)
	{
//		USART_SendData(USART1,0x05);
		L2 = TCRT_L2;
		L1 = TCRT_L1;
		R1 = TCRT_R1;
		R2 = TCRT_R2;
		
		OLED_ShowNum(8*8,0,L2,1,1);
		OLED_ShowNum(8*8,2,L1,1,1);
		OLED_ShowNum(8*8,4,R1,1,1);
		OLED_ShowNum(8*8,6,R2,1,1);
		
		delay_ms(500);	
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);
		delay_ms(500);	
		GPIO_SetBits(GPIOA,GPIO_Pin_4);

	}	
}	


