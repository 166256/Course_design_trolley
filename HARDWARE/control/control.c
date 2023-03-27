#include "control.h"
#include "Tim_encoder_speed.h"
#include "usart.h"
#include "protocol.h"
#include "Tim_pwm.h"

extern char L2,L1,M0,R1,R2;	// ��������״̬
extern volatile int16_t encoderNum_R,encoderNum_L; // ����������

int Position_error = 0;	// ����PID��error�����Ʒ���2
char error_sensor[7] = {0};	// ��¼��һ�����������ڣ�ÿ������������⵽���Σ������Ǵ����������ĵڼ����������������������⴫������
unsigned char status = 2;	// ��¼�����Ǵ����������ĵڼ�����������⵽����
int error = 0;				// ����PID��error�����Ʒ���1
short v_basic = 95;			// ��������ٶ�
PID pid_L,pid_R;			// �ٶȻ���PID�ṹ�����

float Position_KP = 0.60,Position_KI = 0,Position_KD = 0;	// ����PID����

void read_status()
{
	if(M0 == 1 && (L2 || R2) == 0)
	{	
		error_sensor[3]++;
		status = 2;
		error = 0;
	}
	else if(L1 == 1 && (M0 || L2 || R1 || R2) == 0)
	{
		error_sensor[2]++;
		status = 1;
		error = -1;
	}
	else if(L2 == 1 && (L1 || M0 || R1 || R2) == 0)
	{
		error_sensor[1]++;
		status = 0;
		error = -2;
	}
	else if(R1 == 1 && (L1 || L2 || M0 || R2) == 0)
	{
		error_sensor[4]++;
		status = 3;
		error = 1;
	}
	else if(R2 == 1 && (L1 || L2 || R1 || M0) == 0)
	{
		error_sensor[5]++;
		status = 4;
		error = 2;
	}
	if(M0 == 0 && L1 == 0 && L2 == 0 && R1 == 0 && R2 ==0)
	{
		if(status == 4) // ��
		{
			error_sensor[6]++;
			error = 4;
		}
		else if(status == 0) // ��
		{
			error_sensor[0]++;
			error = -4;
		}
	}
}

int get_error(unsigned char ms)
{
	int position_error = 0;
	for(char i = 0; i<7; i++)
	{
		position_error += error_sensor[i] * (i - 3) * ms;
		error_sensor[i] = 0;
	}
	return position_error;
}

void PID_Init()
{	
	pid_R.Kp = 7.5;
	pid_R.Ki = 0.35;
	pid_R.Kd = 0;
	pid_L.Kp = 7.5;
	pid_L.Ki = 0.35;
	pid_L.Kd = 0;
	
	pid_R.target_val = v_basic;
	pid_L.target_val = v_basic;
}

void offset_modify()
{
	v_basic = Speed;
	pid_R.target_val = v_basic;
	pid_L.target_val = v_basic;
//	Position_KP = (float)offset1 / 10;
	Position_KP= (float)offset1 / 100;
//	Position_KI= (float)offset2 / 100;
	Position_KD= (float)offset3 / 10;
	
	// ���Ե����
	pid_L.Kp = (float)offset2 / 10;
//	pid_L.Ki = (float)offset2 / 100;
//	pid_L.Kd = (float)offset3 / 100;
	
	pid_R.Kp = (float)offset2 / 10;
//	pid_R.Ki = (float)offset2 / 100;
//	pid_R.Kd = (float)offset3 / 100;
}

int PID_dir(int Error,int Target)   //����PID(λ��ʽ)//Target=0;
{  
	int Output = 0;
	
	static float Bias,Integral_bias,Last_Bias;
	Bias = Error - Target;                                  //����ƫ��
	Integral_bias += Bias;                                  //���ƫ��Ļ���
	Output = Position_KP * Bias + 
		     Position_KI * Integral_bias + 
		     Position_KD * (Bias - Last_Bias);       //λ��ʽPID������
	Last_Bias = Bias;                                       //������һ��ƫ�� 
	return Output;                                           //�������ֵ
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

volatile short res_pwm_R = 0,res_pwm_L = 0; /*PWMֵ��PID�����*/
volatile short old_pwm_R = 0,old_pwm_L = 0;
int offset_R = 0;
//���ڶ�ʱ���Ļص�����
void AutoReloadCallbackR(int Error)
{
	offset_R = PID_dir(Error,0);
	
#ifdef PID_DIR_ADD
	pid_R.target_val -= offset_R;
#endif

#ifdef PID_DIR
	pid_R.target_val = v_basic - offset_R;
#endif
	
	int sum = 0;/*������ֵ��PID���룩*/
	
    /* ��ȡ�������������ٶ�ֵ */
	sum = encoderNum_R;
    /*����PID���㣬�õ�PWM���ֵ*/
    res_pwm_R = PID_speed(sum, pid_R);
	res_pwm_R += old_pwm_R;
	old_pwm_R = res_pwm_R;
	
	if(res_pwm_R > 7100)
		res_pwm_R = 7100;
	else if(res_pwm_R < 1500)
		res_pwm_R = 1500;
	/*����PWMֵ���Ƶ��ת��*/
	TIM_SetCompare4(TIM4,res_pwm_R);
	
    /*����λ��ͨ��1����ʵ��ֵ*/
//	packet_bluedata(sum);
}

int offset_L  = 0;
//���ڶ�ʱ���Ļص�����
void AutoReloadCallbackL(int Error)
{
	offset_L = PID_dir(Error,0);
#ifdef PID_DIR_ADD
	pid_L.target_val += offset_L;
#endif

#ifdef PID_DIR
	pid_L.target_val = v_basic + offset_L;
#endif
	float sum = 0;/*������ֵ��PID���룩*/
	
    /* ��ȡ�������������ٶ�ֵ */
	sum = encoderNum_L;
    /*����PID���㣬�õ�PWM���ֵ*/
    res_pwm_L = PID_speed(sum, pid_L);
	res_pwm_L += old_pwm_L;
	old_pwm_L = res_pwm_L;
	
	if(res_pwm_L > 7100)
		res_pwm_L = 7100;
	else if(res_pwm_L < 1000)
		res_pwm_L = 1000;
	/*����PWMֵ���Ƶ��ת��*/
	TIM_SetCompare2(TIM4,res_pwm_L);
	
    /*����λ��ͨ��1����ʵ��ֵ*/
//	packet_bluedata(sum);
}

void motor_init()
{
	tim4_gpio_config();
	PWM_Start(TIM4,1,15,0);		//����
	PWM_Start(TIM4,2,15,30);	//����
	PWM_Start(TIM4,3,15,0);		//����
	PWM_Start(TIM4,4,15,30);	//����
	
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
