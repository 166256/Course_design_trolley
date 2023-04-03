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
#include "kalman.h"

extern volatile int16_t encoderNum_R,encoderNum_L;
extern unsigned char path_status;

unsigned char motor_buffer[SENT_DATA - 3];
char L2=1,L1=1,M0=1,R1=1,R2=1;
short gyro_angle;
int mag_x,mag_y,mag_z;
float kalman_fusion_angle = 0;

int main (void)
{
	delay_init();				// ��ʱ������ʼ��
	RCC_Configuration();		// ϵͳʱ�ӳ�ʼ��
	LED_Init();					// LED��ʼ��
	OLED_Init();				// OLED��ʼ��
	tcrt500l_init();
	uart1_init(115200);
	
//	DEBUG_printf("AT+BAUD=115200\n");
//	while(1)
//		delay_ms(50000);																							
	
//	key_gpio_config();
//	while(KEY_SCAN() == 1)
//		delay_ms(20);
	
	delay_ms(1000);				// �ȴ������豸��ʼ������
	tim1_init(49,7199);			// 5ms�ж�һ��
	motor_init();
	PID_Init();
	Tim_EncoderR_Init();		// ���ֱ�����
	Tim_EncoderL_Init();
	MPU9250_Init();
	IMU_OffsetInit();
//	kalman_config_angle(&kalman_angle);
	
/**********************************************/

	OLED_ShowString(0,0,"angle:");
	OLED_ShowString(0,2,"error:");
	OLED_ShowString(0,4,"status:");
	
	while(1)
	{		
		if(start_flag && tim1_num1 >= 4)
//		if(tim1_num1 >= 4)
		{
			tim1_num1 = 0;
	
			Position_error = get_error(5); // ÿ��һ�δ�������Ҫ10ms
			
			calc_motor_Right_rotate_speed(); // ��ȡ��������ֵ
			calc_motor_Left_rotate_speed();
			AutoReloadCallbackR(Position_error); // ����ΪPosition_errorʱҪ�ķ��򻷵�PID���ĵĺ�С������Ϊԭ����������20
			AutoReloadCallbackL(Position_error);
		
			track_construction(gyro_angle_dir,20); // �켣�ع�		
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
		
			read_error();	// �õ�ƫ������ֵ
			read_status();	// ״̬��
			
			IMU_9250_GetValues();
			get_angle_IMU();
//			Calibrated_Mag(); // �������
//			kalman_fusion_angle = kalman_update(&kalman_angle,mag_angle_dir,Mov_9250.gyro_z,0.005); // �������ں�		
			gyro_angle = (short) gyro_angle_dir;
			
			OLED_ShowNum(8*8,0,gyro_angle/100,1,1);
			OLED_ShowNum(8*9,0,(gyro_angle/10)%10,1,1);
			OLED_ShowNum(8*10,0,gyro_angle%10,1,1);
			OLED_ShowNum(8*8,2,error,1,1);
			OLED_ShowNum(8*8,4,path_status,1,1);
		}
		if(tim1_num2 >= 8 && start_flag == 1)
		{
			tim1_num2 = 0;
			
			SHORT_SPLIT_CHAR(0,encoderNum_R);
			SHORT_SPLIT_CHAR(2,v_basic);
			INT_SPLIT_CHAR(4,-east_distance_cm);
			INT_SPLIT_CHAR(8,north_distance_cm);
			motor_buffer[12] = (unsigned char)Position_error;
			motor_buffer[13] = path_status;
			motor_buffer[14] = offset_R*2;
			SHORT_SPLIT_CHAR(15,encoderNum_L);

			packet_bluedata(motor_buffer);
		}

	}	
}	


