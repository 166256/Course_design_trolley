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

extern volatile int16_t encoderNum_R,encoderNum_L;

unsigned char motor_buffer[SENT_DATA - 3];
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
	tim1_init(49,7199);			// 5ms中断一次
	motor_init();
	PID_Init();
	Tim_EncoderR_Init();		// 右轮编码器
	Tim_EncoderL_Init();
	MPU9250_Init();
	IMU_OffsetInit();
	
/**********************************************/

	OLED_ShowString(0,0,"angle:");
	OLED_ShowString(0,2,"error:");
	OLED_ShowString(0,4,"judge_S:");
	
	while(1)
	{		
		OLED_ShowNum(8*8,0,gyro_angle/100,1,1);
		OLED_ShowNum(8*9,0,(gyro_angle/10)%10,1,1);
		OLED_ShowNum(8*10,0,gyro_angle%10,1,1);
		OLED_ShowNum(8*8,2,error,1,1);
		OLED_ShowNum(8*8,4,judge_S,1,1);
		
		if(offset1 != 0 || offset2 != 0 || offset3 != 0)
			offset_modify();
		
		if(start_flag && tim1_num1 >= 4)
//		if(tim1_num1 >= 4)
		{
			tim1_num1 = 0;
	
			Position_error = get_error(5); // 每读一次传感器需要10ms
			
			calc_motor_Right_rotate_speed();
			calc_motor_Left_rotate_speed();
			AutoReloadCallbackR(Position_error); // 参数为Position_error时要改方向环的PID，改的很小，建议为原本参数除以20
			AutoReloadCallbackL(Position_error);
		
			track_construction(gyro_angle_dir,20); // 轨迹重构
//			DEBUG_printf("%f,%f,%f,%f,%f,%f\n",mag_angle_dir, gyro_angle_dir, east_distance, IMU_9250.mag_x, IMU_9250.mag_y, IMU_9250.mag_z);			
			gyro_angle = (short) gyro_angle_dir;
		}
		else if(start_flag == 0)
			motor_stop();
		if(tim1_flag == 1)
		{			
			tim1_flag = 0;
			
			L2 = TCRT_L2;
			L1 = TCRT_L1;
			M0 = TCRT_M0;
			R1 = TCRT_R1;
			R2 = TCRT_R2;
		
			read_status();
			IMU_9250_GetValues();
			get_angle_IMU();
			gyro_angle = (short) gyro_angle_dir;
		}
		if(tim1_num2 >= 8 && start_flag == 1)
		{
			tim1_num2 = 0;
			
			SHORT_SPLIT_CHAR(0,encoderNum_R);
			SHORT_SPLIT_CHAR(2,v_basic);
			INT_SPLIT_CHAR(4,-east_distance_cm);
			INT_SPLIT_CHAR(8,north_distance_cm);
			motor_buffer[12] = (unsigned char)Position_error;
			SHORT_SPLIT_CHAR(13,offset_R);

			packet_bluedata(motor_buffer);
		}
//		if(tim1_num3 >= 900)
//		{
//			tim1_num3 = 0;
//			judge_S = 0;
//			Position_KP = Position_KP * 0.6;
//		}
	}	
}	


