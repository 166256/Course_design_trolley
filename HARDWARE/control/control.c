#include "control.h"
#include "Tim_encoder_speed.h"
#include "usart.h"
#include "protocol.h"
#include "Tim_pwm.h"

extern char L2,L1,M0,R1,R2;
extern volatile int16_t encoderNum_R,encoderNum_L;

unsigned char status = 0;
unsigned char v_offset1 = 3,v_offset2 = 7,v_offset3 = 13;
short v_basic = 80;
PID pid_L,pid_R;

unsigned char motor_buffer[SENT_DATA - 3];

void moter_control()
{
	if(M0 == 1 && (L2 || R2) == 0)
	{	
		status = 2;
		// �ָ�
		pid_L.target_val = v_basic;
		pid_R.target_val = v_basic;
	}
	else if(L1 == 1 && (M0 || L2 || R1 || R2) == 0)
	{
		status = 1;
		// �����ҿ�
		pid_L.target_val = v_basic - v_offset1;
		pid_R.target_val = v_basic + v_offset1;
	}
	else if(L2 == 1 && (L1 || M0 || R1 || R2) == 0)
	{
		status = 0;
		// �����ҿ�
		pid_L.target_val = v_basic - v_offset2;
		pid_R.target_val = v_basic + v_offset2;
	}
	else if(R1 == 1 && (L1 || L2 || M0 || R2) == 0)
	{
		status = 3;
		// �������
		pid_L.target_val = v_basic + v_offset1;
		pid_R.target_val = v_basic - v_offset1;
	}
	else if(R2 == 1 && (L1 || L2 || R1 || M0) == 0)
	{
		status = 4;
		// �������
		pid_L.target_val = v_basic + v_offset2;
		pid_R.target_val = v_basic - v_offset2;
	}
	
	if(M0 == 0 && L1 == 0 && L2 == 0 && R1 == 0 && R2 ==0)
	{
		if(status == 4) // ��
		{
			pid_L.target_val = v_basic + 1.2 * v_offset3;
			pid_R.target_val = v_basic - 1.2 * v_offset3;
		}
		else if(status == 0) // ��
		{
			pid_L.target_val = v_basic - 1.2 * v_offset3;
			pid_R.target_val = v_basic + 1.2 * v_offset3;		
		}
	}
	
	// ����״̬����λ��
	motor_buffer[0] = L2;
	motor_buffer[1] = L1;
	motor_buffer[2] = M0;
	motor_buffer[3] = R1;
	motor_buffer[4] = R2;
	motor_buffer[5] = status;
	packet_bluedata(motor_buffer);
}

//void read_status()
//{
//	if(M0 == 1 && (L2 || R2) == 0)
//	{	
//		status = 2;
//		error = 0;
//	}
//	else if(L1 == 1 && (M0 || L2 || R1 || R2) == 0)
//	{
//		status = 1;
//		error = -1;
//	}
//	else if(L2 == 1 && (L1 || M0 || R1 || R2) == 0)
//	{
//		status = 0;
//		error = -2
//	}
//	else if(R1 == 1 && (L1 || L2 || M0 || R2) == 0)
//	{
//		status = 3;
//		error = 1;
//	}
//	else if(R2 == 1 && (L1 || L2 || R1 || M0) == 0)
//	{
//		status = 4;
//		error = 2;
//	}
//	if(M0 == 0 && L1 == 0 && L2 == 0 && R1 == 0 && R2 ==0)
//	{
//		if(status == 4) // ��
//			error = 3;
//		else if(status == 0) // ��
//			error = -3;
//	}
//}

//int PID_dir(int error,int Target)   //����PID(λ��ʽ)//Target=0;
//{  
//	int Output = 0;
//	float Position_KP = 0,Position_KI = 0,Position_KD = 0;
//	static float Bias,Integral_bias,Last_Bias;
//	Bias = error - Target;                                  //����ƫ��
//	Integral_bias += Bias;                                  //���ƫ��Ļ���
//	Output = Position_KP * Bias + 
//		     Position_KI * Integral_bias + 
//		     Position_KD * (Bias - Last_Bias);       //λ��ʽPID������
//	Last_Bias = Bias;                                       //������һ��ƫ�� 
//	return Output;                                           //�������ֵ
//}

void PID_Init()
{	
	pid_R.Kp = 1.6;
	pid_R.Ki = 0.14;
	pid_R.Kd = 0.05;
	pid_L.Kp = 1.6;
	pid_L.Ki = 0.14;
	pid_L.Kd = 0.05;
	
	pid_R.target_val = v_basic;
	pid_L.target_val = v_basic;
}

void offset_modify()
{
	v_basic = Speed;
//	v_offset1 = offset1;
//	v_offset2 = offset2;
//	v_offset3 = offset3;
	
	// ���Ե����
	pid_L.Kp = (float)offset1 / 10;
	pid_L.Ki = (float)offset2 / 100;
	pid_L.Kd = (float)offset3 / 100;
	
	pid_R.Kp = (float)offset1 / 10;
	pid_R.Ki = (float)offset2 / 100;
	pid_R.Kd = (float)offset3 / 100;
}

int PID_speed(int actual_val,PID pid)
{
	/*����Ŀ��ֵ��ʵ��ֵ�����*/
	pid.err = pid.target_val - actual_val;
	
	/*������*/
	pid.integral += pid.err;

	/*PID�㷨ʵ��*/
	pid.output_val = pid.Kp * pid.err + 
				     pid.Ki * pid.integral + 
				     pid.Kd * (pid.err - pid.err_last);

	/*����*/
	pid.err_last = pid.err;

	/*���ص�ǰʵ��ֵ*/
	return pid.output_val;
}

int res_pwm_R = 0,res_pwm_L = 0; /*PWMֵ��PID�����*/
int old_pwm_R = 0,old_pwm_L = 0;
//���ڶ�ʱ���Ļص�����
void AutoReloadCallbackR()
{
	int sum = 0;/*������ֵ��PID���룩*/
	
    /* ��ȡ�������������ٶ�ֵ */
	sum = encoderNum_R;
    /*����PID���㣬�õ�PWM���ֵ*/
    res_pwm_R = PID_speed(sum, pid_R);
	res_pwm_R += old_pwm_R;
	old_pwm_R = res_pwm_R;
	
	if(res_pwm_R > 3500)
		res_pwm_R = 3500;
	else if(res_pwm_R < 1500)
		res_pwm_R = 1500;
	/*����PWMֵ���Ƶ��ת��*/
	TIM_SetCompare4(TIM4,res_pwm_R);
	
    /*����λ��ͨ��1����ʵ��ֵ*/
//	packet_bluedata(sum);
}

//���ڶ�ʱ���Ļص�����
void AutoReloadCallbackL()
{
	float sum = 0;/*������ֵ��PID���룩*/
	
    /* ��ȡ�������������ٶ�ֵ */
	sum = encoderNum_L;
    /*����PID���㣬�õ�PWM���ֵ*/
    res_pwm_L = PID_speed(sum, pid_L);
	res_pwm_L += old_pwm_L;
	old_pwm_L = res_pwm_L;
	
	if(res_pwm_L > 3500)
		res_pwm_L = 3500;
	else if(res_pwm_L < 1000)
		res_pwm_L = 1000;
	/*����PWMֵ���Ƶ��ת��*/
	TIM_SetCompare2(TIM4,res_pwm_L);
	
    /*����λ��ͨ��1����ʵ��ֵ*/
//	packet_bluedata(sum);
}

void motor_init()
{
	PWM_Start(TIM4,1,11,0);		//����
	PWM_Start(TIM4,2,11,30);	//����
	PWM_Start(TIM4,3,11,0);		//����
	PWM_Start(TIM4,4,11,30);	//����
	
	TIM_SetCompare1(TIM4,0);
	TIM_SetCompare2(TIM4,0);
	TIM_SetCompare3(TIM4,0);
	TIM_SetCompare4(TIM4,0);
}

void motor_stop()
{
	TIM_SetCompare1(TIM4,0);
	TIM_SetCompare2(TIM4,0);
	TIM_SetCompare3(TIM4,0);
	TIM_SetCompare4(TIM4,0);
}
