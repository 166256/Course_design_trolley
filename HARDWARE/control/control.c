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
		// 恢复
		pid_L.target_val = v_basic;
		pid_R.target_val = v_basic;
	}
	else if(L1 == 1 && (M0 || L2 || R1 || R2) == 0)
	{
		status = 1;
		// 左慢右快
		pid_L.target_val = v_basic - v_offset1;
		pid_R.target_val = v_basic + v_offset1;
	}
	else if(L2 == 1 && (L1 || M0 || R1 || R2) == 0)
	{
		status = 0;
		// 左慢右快
		pid_L.target_val = v_basic - v_offset2;
		pid_R.target_val = v_basic + v_offset2;
	}
	else if(R1 == 1 && (L1 || L2 || M0 || R2) == 0)
	{
		status = 3;
		// 右慢左快
		pid_L.target_val = v_basic + v_offset1;
		pid_R.target_val = v_basic - v_offset1;
	}
	else if(R2 == 1 && (L1 || L2 || R1 || M0) == 0)
	{
		status = 4;
		// 右慢左快
		pid_L.target_val = v_basic + v_offset2;
		pid_R.target_val = v_basic - v_offset2;
	}
	
	if(M0 == 0 && L1 == 0 && L2 == 0 && R1 == 0 && R2 ==0)
	{
		if(status == 4) // 右
		{
			pid_L.target_val = v_basic + 1.2 * v_offset3;
			pid_R.target_val = v_basic - 1.2 * v_offset3;
		}
		else if(status == 0) // 左
		{
			pid_L.target_val = v_basic - 1.2 * v_offset3;
			pid_R.target_val = v_basic + 1.2 * v_offset3;		
		}
	}
	
	// 发送状态到上位机
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
//		if(status == 4) // 右
//			error = 3;
//		else if(status == 0) // 左
//			error = -3;
//	}
//}

//int PID_dir(int error,int Target)   //方向PID(位置式)//Target=0;
//{  
//	int Output = 0;
//	float Position_KP = 0,Position_KI = 0,Position_KD = 0;
//	static float Bias,Integral_bias,Last_Bias;
//	Bias = error - Target;                                  //计算偏差
//	Integral_bias += Bias;                                  //求出偏差的积分
//	Output = Position_KP * Bias + 
//		     Position_KI * Integral_bias + 
//		     Position_KD * (Bias - Last_Bias);       //位置式PID控制器
//	Last_Bias = Bias;                                       //保存上一次偏差 
//	return Output;                                           //返回输出值
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
	
	// 调试电机用
	pid_L.Kp = (float)offset1 / 10;
	pid_L.Ki = (float)offset2 / 100;
	pid_L.Kd = (float)offset3 / 100;
	
	pid_R.Kp = (float)offset1 / 10;
	pid_R.Ki = (float)offset2 / 100;
	pid_R.Kd = (float)offset3 / 100;
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

int res_pwm_R = 0,res_pwm_L = 0; /*PWM值（PID输出）*/
int old_pwm_R = 0,old_pwm_L = 0;
//周期定时器的回调函数
void AutoReloadCallbackR()
{
	int sum = 0;/*编码器值（PID输入）*/
	
    /* 读取编码器测量的速度值 */
	sum = encoderNum_R;
    /*进行PID运算，得到PWM输出值*/
    res_pwm_R = PID_speed(sum, pid_R);
	res_pwm_R += old_pwm_R;
	old_pwm_R = res_pwm_R;
	
	if(res_pwm_R > 3500)
		res_pwm_R = 3500;
	else if(res_pwm_R < 1500)
		res_pwm_R = 1500;
	/*根据PWM值控制电机转动*/
	TIM_SetCompare4(TIM4,res_pwm_R);
	
    /*给上位机通道1发送实际值*/
//	packet_bluedata(sum);
}

//周期定时器的回调函数
void AutoReloadCallbackL()
{
	float sum = 0;/*编码器值（PID输入）*/
	
    /* 读取编码器测量的速度值 */
	sum = encoderNum_L;
    /*进行PID运算，得到PWM输出值*/
    res_pwm_L = PID_speed(sum, pid_L);
	res_pwm_L += old_pwm_L;
	old_pwm_L = res_pwm_L;
	
	if(res_pwm_L > 3500)
		res_pwm_L = 3500;
	else if(res_pwm_L < 1000)
		res_pwm_L = 1000;
	/*根据PWM值控制电机转动*/
	TIM_SetCompare2(TIM4,res_pwm_L);
	
    /*给上位机通道1发送实际值*/
//	packet_bluedata(sum);
}

void motor_init()
{
	PWM_Start(TIM4,1,11,0);		//左轮
	PWM_Start(TIM4,2,11,30);	//左轮
	PWM_Start(TIM4,3,11,0);		//右轮
	PWM_Start(TIM4,4,11,30);	//右轮
	
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
