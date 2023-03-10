#include "control.h"
#include "Tim_encoder_speed.h"
#include "usart.h"
#include "protocol.h"
#include "Tim_pwm.h"

extern char L2,L1,M0,R1,R2;
extern volatile int16_t encoderNum_R,encoderNum_L;

unsigned char status = 2;
unsigned char v_offset1 = 3,v_offset2 = 7,v_offset3 = 13;
short v_basic = 120;
PID pid_L,pid_R;

int error = 0;
float Position_KP = 6.9,Position_KI = 0,Position_KD = 0;

unsigned char motor_buffer[SENT_DATA - 3];

void read_status()
{
	if(M0 == 1 && (L2 || R2) == 0)
	{	
		status = 2;
		error = 0;
	}
	else if(L1 == 1 && (M0 || L2 || R1 || R2) == 0)
	{
		status = 1;
		error = -1;
	}
	else if(L2 == 1 && (L1 || M0 || R1 || R2) == 0)
	{
		status = 0;
		error = -2;
	}
	else if(R1 == 1 && (L1 || L2 || M0 || R2) == 0)
	{
		status = 3;
		error = 1;
	}
	else if(R2 == 1 && (L1 || L2 || R1 || M0) == 0)
	{
		status = 4;
		error = 2;
	}
	if(M0 == 0 && L1 == 0 && L2 == 0 && R1 == 0 && R2 ==0)
	{
		if(status == 4) // 右
			error = 4;
		else if(status == 0) // 左
			error = -4;
	}
}

void PID_Init()
{	
	pid_R.Kp = 7.5;
	pid_R.Ki = 0.35;
	pid_R.Kd = 0;
	pid_L.Kp = 7.5;
	pid_L.Ki = 0.35;
	pid_L.Kd = 0.05;
	
	pid_R.target_val = v_basic;
	pid_L.target_val = v_basic;
}

void offset_modify()
{
	v_basic = Speed;
//	Position_KP = (float)offset1 / 10;
//	Position_KD = (float)offset3 / 100;
//	v_offset1 = offset1;
//	v_offset2 = offset2;
//	v_offset3 = offset3;
	
	// 调试电机用
	pid_L.Kp = (float)offset1 / 10;
	pid_L.Ki = (float)offset2 / 100;
	pid_L.Kd = (float)offset3 / 100;
	
	pid_R.Kp = (float)offset1 / 10;
	pid_R.Ki = (float)offset2 / 100;
	pid_R.Kd = (float)offset3 / 100;
}

int PID_dir(int Error,int Target)   //方向PID(位置式)//Target=0;
{  
	int Output = 0;
	
	static float Bias,Integral_bias,Last_Bias;
	Bias = Error - Target;                                  //计算偏差
	Integral_bias += Bias;                                  //求出偏差的积分
	Output = Position_KP * Bias + 
		     Position_KI * Integral_bias + 
		     Position_KD * (Bias - Last_Bias);       //位置式PID控制器
	Last_Bias = Bias;                                       //保存上一次偏差 
	return Output;                                           //返回输出值
}

int PID_speed(int actual_val,PID pid)
{
	/*计算目标值与实际值的误差*/
	pid.err = pid.target_val - actual_val;
	
	/*积分项*/
	pid.integral += pid.err;

	/*PID算法实现*/
	pid.output_val = pid.Kp * pid.err + 
				     pid.Ki * pid.integral + 
				     pid.Kd * (pid.err - pid.err_last);

	/*误差传递*/
	pid.err_last = pid.err;

	/*返回当前实际值*/
	return pid.output_val;
}

volatile short res_pwm_R = 0,res_pwm_L = 0; /*PWM值（PID输出）*/
volatile short old_pwm_R = 0,old_pwm_L = 0;

// 周期定时器的回调函数
void AutoReloadCallbackR()
{
	int offset = 0,sum = 0;;
	
	/* 方向环，得到后轮差速 */
	offset = PID_dir(error,0);
	
	/* 计算右轮电机的目标速度 */
	pid_R.target_val = v_basic - offset;
	
    /* 读取编码器测量的速度值 */
	sum = encoderNum_R;
	
    /* 速度环，得到PWM输出值 */
    res_pwm_R = PID_speed(sum, pid_R);
	res_pwm_R += old_pwm_R;
	old_pwm_R = res_pwm_R;
	
	/* 限制PWM值在合理范围 */
	if(res_pwm_R > 7100)
		res_pwm_R = 7100;
	else if(res_pwm_R < 1500)
		res_pwm_R = 1500;
	
	/*根据PWM值控制电机转动*/
	TIM_SetCompare4(TIM4,res_pwm_R);
}

//周期定时器的回调函数
void AutoReloadCallbackL()
{
	int offset  = 0;
	offset = PID_dir(error,0);
	pid_L.target_val = v_basic + offset; // 
	
	float sum = 0;/*编码器值（PID输入）*/
	
    /* 读取编码器测量的速度值 */
	sum = encoderNum_L;
    /*进行PID运算，得到PWM输出值*/
    res_pwm_L = PID_speed(sum, pid_L);
	res_pwm_L += old_pwm_L;
	old_pwm_L = res_pwm_L;
	
	if(res_pwm_L > 7100)
		res_pwm_L = 7100;
	else if(res_pwm_L < 1000)
		res_pwm_L = 1000;
	/*根据PWM值控制电机转动*/
	TIM_SetCompare2(TIM4,res_pwm_L);
	
    /*给上位机通道1发送实际值*/
//	packet_bluedata(sum);
}

void motor_init()
{
	PWM_Start(TIM4,1,15,0);		//左轮
	PWM_Start(TIM4,2,15,30);	//左轮
	PWM_Start(TIM4,3,15,0);		//右轮
	PWM_Start(TIM4,4,15,30);	//右轮
	
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
