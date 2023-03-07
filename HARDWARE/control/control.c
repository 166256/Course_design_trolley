#include "control.h"
#include "Tim_encoder_speed.h"
#include "usart.h"

extern char L2,L1,M0,R1,R2;
extern volatile int16_t encoderNum_R,encoderNum_L;

unsigned char status = 0;
unsigned char first = 2,second = 2, third = 2;	// ��ʼ״̬�����м�
unsigned char v_offset1 = 13,v_offset2 = 33,v_offset3 = 63;
short v_basic = 400;
PID pid_L,pid_R;
int num_L,num_M,num_R;

unsigned char motor_buffer[5];

void moter_control()
{
	if(M0 == 1 && (L1 || L2 || R1 || R2) == 0)
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
	
	motor_buffer[0] = L2;
	motor_buffer[1] = L1;
	motor_buffer[2] = M0;
	motor_buffer[3] = R1;
	motor_buffer[4] = R2;
	motor_buffer[5] = status;
	packet_bluedata(motor_buffer);
}

void PID_Init()
{	
	pid_R.Kp = 3;
//	pid_R.Ki = 0.1;
	pid_L.Kp = 3;
	pid_L.target_val = v_basic;
	pid_R.target_val = v_basic;
}

void offset_modify()
{
	v_offset1 = offset1;
	v_offset2 = offset2;
	v_offset3 = offset3;
	v_basic = Speed;
}

int PID_realize(int actual_val,PID pid)
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
    res_pwm_R = PID_realize(sum, pid_R);
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
    res_pwm_L = PID_realize(sum, pid_L);
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
