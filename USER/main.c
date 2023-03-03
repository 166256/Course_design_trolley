#include "stm32f10x.h" 
#include "sys.h"
#include "delay.h" 
#include "led.h"
#include "oled.h"
#include "usart.h"
#include "tcrt500l.h"
#include "Tim_encoder_speed.h"
#include "Tim_pwm.h"

#include "control.h"
#include "mpu9250.h"

//#include "adc.h"

char L2=1,L1=1,M0=1,R1=1,R2=1;

int main (void)
{
	delay_init();				// 延时函数初始化
	RCC_Configuration();		// 系统时钟初始化
	LED_Init();					// LED初始化
	OLED_Init();				// OLED初始化
	delay_ms(1000);				// 等待其他设备初始化就绪
	tcrt500l_init();
	uart1_init(9600);
	tim1_init(999,7199);
	tim4_gpio_config();
	PWM_Start(TIM4,1,11,0);		//左轮
	PWM_Start(TIM4,2,11,30);	//左轮
	PWM_Start(TIM4,3,11,0);		//右轮
	PWM_Start(TIM4,4,11,30);	//右轮
	
	TIM_SetCompare1(TIM4,0);
	TIM_SetCompare2(TIM4,2100);
	TIM_SetCompare3(TIM4,0);
	TIM_SetCompare4(TIM4,2000);

	Tim_EncoderR_Init();		//右轮编码器
	Tim_EncoderL_Init();
//	ADC_Configuration();
	
/**********************************************/

	OLED_ShowString(0,0,"TCRT_L2:");
	OLED_ShowString(0,2,"TCRT_L1:");
	OLED_ShowString(0,4,"TCRT_R1:");
	OLED_ShowString(0,6,"TCRT_R2:");
	
	while(1)
	{
		USART_SendData(USART1,0x05);
		L2 = TCRT_L2;
		L1 = TCRT_L1;
		M0 = TCRT_M0;
		R1 = TCRT_R1;
		R2 = TCRT_R2;
		
		OLED_ShowNum(8*8,0,L2,1,1);
		OLED_ShowNum(8*8,2,L1,1,1);
		OLED_ShowNum(8*8,4,R1,1,1);
		OLED_ShowNum(8*8,6,R2,1,1);
		
//		delay_ms(500);	
//		GPIO_ResetBits(GPIOA,GPIO_Pin_4);
//		delay_ms(500);	
//		GPIO_SetBits(GPIOA,GPIO_Pin_4);
		delay_ms(100);
		calc_motor_Right_rotate_speed();
		calc_motor_Left_rotate_speed();
	}	
}	


