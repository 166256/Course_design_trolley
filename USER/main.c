#include "stm32f10x.h" 
#include "sys.h"
#include "delay.h" 
#include "led.h"
#include "oled.h"
#include "usart.h"
#include "tcrt500l.h"
#include "Tim_encoder_speed.h"
#include "Tim_pwm.h"

#include "protocol.h"
#include "gpio.h"
#include "control.h"
#include "mpu9250.h"

//#include "adc.h"

extern volatile int16_t encoderNum_R,encoderNum_L;

char L2=1,L1=1,M0=1,R1=1,R2=1;

int main (void)
{
	delay_init();				// 延时函数初始化
	RCC_Configuration();		// 系统时钟初始化
	LED_Init();					// LED初始化
	OLED_Init();				// OLED初始化
	tcrt500l_init();
	uart1_init(9600);
	
//	key_gpio_config();
//	while(KEY_SCAN() == 1)
//		delay_ms(20);
	
	delay_ms(1000);				// 等待其他设备初始化就绪
	
	tim1_init(99,7199);			// 10ms中断一次
	tim4_gpio_config();
	
	motor_init();
	PID_Init();
	Tim_EncoderR_Init();		//右轮编码器
	Tim_EncoderL_Init();
//	ADC_Configuration();

	MPU9250_Init();
	
/**********************************************/

	OLED_ShowString(0,0,"TCRT_L2:");
	OLED_ShowString(0,2,"TCRT_L1:");
	OLED_ShowString(0,4,"TCRT_R1:");
	OLED_ShowString(0,6,"TCRT_R2:");
	
	while(1)
	{		
//		OLED_ShowNum(8*8,0,L2,1,1);
//		OLED_ShowNum(8*8,2,L1,1,1);
//		OLED_ShowNum(8*8,4,R1,1,1);
//		OLED_ShowNum(8*8,6,R2,1,1);
		
		if(tim1_flag == 1)
		{
			L2 = TCRT_L2;
			L1 = TCRT_L1;
			M0 = TCRT_M0;
			R1 = TCRT_R1;
			R2 = TCRT_R2;

			if(offset1 != 0 || offset2 != 0 || offset3 != 0)
			{
				offset_modify();
			}
//			moter_control();		
			read_status();
//			pid_L.target_val = v_basic;
//			pid_R.target_val = v_basic;
		}
		if(start_flag && tim1_num1 >= 2)
		{
			calc_motor_Right_rotate_speed();
			calc_motor_Left_rotate_speed();
			AutoReloadCallbackR();
			AutoReloadCallbackL();
			tim1_num1 = 0;
			
			// 调试电机用
			motor_buffer[5] = (unsigned char)(error + 3);
			motor_buffer[6] = (encoderNum_L & 0x00FF);
			motor_buffer[7] = (encoderNum_L & 0xFF00) >> 8;
			motor_buffer[8] = (encoderNum_R & 0x00FF);
			motor_buffer[9] = (encoderNum_R & 0xFF00) >> 8;
			motor_buffer[10] = (v_basic & 0x00FF);
			motor_buffer[11] = (v_basic & 0xFF00) >> 8;
			motor_buffer[12] = (res_pwm_L & 0x00FF);
			motor_buffer[13] = (res_pwm_L & 0xFF00) >> 8;
			motor_buffer[14] = (res_pwm_R & 0x00FF);
			motor_buffer[15] = (res_pwm_R & 0xFF00) >> 8;
			packet_bluedata(motor_buffer);
		}
		else if(start_flag == 0)
		{
			motor_stop();
		}
	}	
}	


