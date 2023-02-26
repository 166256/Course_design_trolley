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
	delay_init();				// 延时函数初始化
	RCC_Configuration();		// 系统时钟初始化
	LED_Init();					// LED初始化
	OLED_Init();				// OLED初始化
	delay_ms(1000);				// 等待其他设备初始化就绪
//	uart1_init(115200);
//	Tim_gpio_config();
//	PWM_Start(TIM1,1,10,20);	//左轮
//	PWM_Start(TIM1,4,10,20);	//右轮
//	Steering_Control_Forward();	//设置转向为正
//	Tim_E2B_E2A_Config();		//右轮编码器
//	Tim_E1B_E1A_Config();		//左轮编码器
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


