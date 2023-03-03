#include "control.h"
#include "Tim_encoder_speed.h"

extern char L2,L1,M0,R1,R2;

unsigned char status = 0;
unsigned int K = 1;
PID pid;

void moter_control()
{
	switch(status)
	{
		case 0:
			if(L1 == 0)
			{
				status = 1;
				// �����ҿ�
				TIM_SetCompare2(TIM4,INITIAL_SPEED - OFFSET * K); // ����
				TIM_SetCompare4(TIM4,INITIAL_SPEED + OFFSET * K); // ����
			}
			else if(R1 == 0)
			{
				status = 1;
				// �������
				TIM_SetCompare2(TIM4,INITIAL_SPEED + OFFSET * K);
				TIM_SetCompare4(TIM4,INITIAL_SPEED - OFFSET * K);
			}
			break;
		case 1:
			if(M0 == 0)
			{
				status = 0;
				// �ָ�
				TIM_SetCompare2(TIM4,INITIAL_SPEED);
				TIM_SetCompare4(TIM4,INITIAL_SPEED);
			}
			break;
		default:break;
	}
}

void PID_Init()
{
	pid.Kp = 0;
	pid.Ki = 0;
	pid.Kd = 0;
}

float PID_realize(float actual_val)
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

//���ڶ�ʱ���Ļص�����
void AutoReloadCallbackR()
{
	float sum = 0;/*������ֵ��PID���룩*/
	int res_pwm = 0;/*PWMֵ��PID�����*/
	
    /* ��ȡ�������������ٶ�ֵ */
	sum = calc_motor_Right_rotate_speed();
    /*����PID���㣬�õ�PWM���ֵ*/
    res_pwm = PID_realize(sum);
	
	if(res_pwm > 3500)
		res_pwm = 3500;
	else if(res_pwm < 1000)
		res_pwm = 1000;
	/*����PWMֵ���Ƶ��ת��*/
	TIM_SetCompare4(TIM4,res_pwm);
	
    /*����λ��ͨ��1����ʵ��ֵ*/
//	set_computer_value(SEND_FACT_CMD, CURVES_CH1, &sum, 1); 
}

//���ڶ�ʱ���Ļص�����
void AutoReloadCallbackL()
{
	float sum = 0;/*������ֵ��PID���룩*/
	int res_pwm = 0;/*PWMֵ��PID�����*/
	
    /* ��ȡ�������������ٶ�ֵ */
	sum = calc_motor_Left_rotate_speed();
    /*����PID���㣬�õ�PWM���ֵ*/
    res_pwm = PID_realize(sum);
	
	if(res_pwm > 3500)
		res_pwm = 3500;
	else if(res_pwm < 1000)
		res_pwm = 1000;
	/*����PWMֵ���Ƶ��ת��*/
	TIM_SetCompare2(TIM4,res_pwm);
	
    /*����λ��ͨ��1����ʵ��ֵ*/
//	set_computer_value(SEND_FACT_CMD, CURVES_CH1, &sum, 1); 
}
