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
#include "track.h"

//#include "adc.h"

extern volatile int16_t encoderNum_R,encoderNum_L;

char L2=1,L1=1,M0=1,R1=1,R2=1;
short gyro_angle;
int mag_x,mag_y,mag_z;

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
	motor_init();
	PID_Init();
	Tim_EncoderR_Init();		//右轮编码器
	Tim_EncoderL_Init();
//	ADC_Configuration();

	MPU9250_Init();
	IMU_OffsetInit();
	
/**********************************************/

	OLED_ShowString(0,0,"angle:");
	OLED_ShowString(0,2,"error:");
	
	while(1)
	{		
		OLED_ShowNum(8*8,0,gyro_angle/100,1,1);
		OLED_ShowNum(8*9,0,(gyro_angle/10)%10,1,1);
		OLED_ShowNum(8*10,0,gyro_angle%10,1,1);
		OLED_ShowNum(8*8,2,error,1,1);
		
		if(tim1_flag == 1)
		{
			L2 = TCRT_L2;
			L1 = TCRT_L1;
			M0 = TCRT_M0;
			R1 = TCRT_R1;
			R2 = TCRT_R2;

			if(offset1 != 0 || offset2 != 0 || offset3 != 0)
				offset_modify();		
			read_status();
			IMU_9250_GetValues();
			get_angle_IMU();
			gyro_angle = (short) gyro_angle_dir;
			
			tim1_flag = 0;
		}
		if(tim1_num2 >= 5)
		{
			tim1_num2 = 0;
//			mag_x = IMU_9250.mag_x * 100;
//			mag_y = IMU_9250.mag_y * 100;
//			mag_z = IMU_9250.mag_z * 100;
//			INT_SPLIT_CHAR(0,mag_x);
//			INT_SPLIT_CHAR(4,mag_y);
//			INT_SPLIT_CHAR(8,mag_z);
//			packet_bluedata(motor_buffer);
			motor_buffer[0] = (encoderNum_R & 0x00FF);
			motor_buffer[1] = (encoderNum_R & 0xFF00) >> 8;
			motor_buffer[2] = (v_basic & 0x00FF);
			motor_buffer[3] = (v_basic & 0xFF00) >> 8;
			motor_buffer[4] = (east_distance_cm & 0x000000FF);
			motor_buffer[5] = (east_distance_cm & 0x0000FF00) >> 8;
			motor_buffer[6] = (east_distance_cm & 0x00FF0000) >> 16;
			motor_buffer[7] = (east_distance_cm & 0xFF000000) >> 24;
			motor_buffer[8] = (north_distance_cm & 0x000000FF);
			motor_buffer[9] = (north_distance_cm & 0x0000FF00) >> 8;
			motor_buffer[10] = (north_distance_cm & 0x00FF0000) >> 16;
			motor_buffer[11] = (north_distance_cm & 0xFF000000) >> 24;
			motor_buffer[12] = (unsigned char)error;
			motor_buffer[13] = (offset_R & 0x00FF);
			motor_buffer[14] = (offset_R & 0xFF00) >> 8;
			packet_bluedata(motor_buffer);
		}
		if(start_flag && tim1_num1 >= 2)
//		if(tim1_num1 >= 2)
		{
			calc_motor_Right_rotate_speed();
			calc_motor_Left_rotate_speed();
			AutoReloadCallbackR();
			AutoReloadCallbackL();
			tim1_num1 = 0;
			
			track_construction(gyro_angle_dir,20);
//			DEBUG_printf("%f,%f,%f,%f,%f,%f\n",mag_angle_dir, gyro_angle_dir, east_distance, IMU_9250.mag_x, IMU_9250.mag_y, IMU_9250.mag_z);			
			gyro_angle = (short) gyro_angle_dir;
			

		}
		else if(start_flag == 0)
		{
			motor_stop();
		}
		
	}	
}	


