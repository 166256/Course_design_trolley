#include "control.h"
#include "Tim_encoder_speed.h"
#include "usart.h"

extern char L2,L1,M0,R1,R2;
extern unsigned int Kp,Ki,Kd,Speed;
extern volatile int16_t encoderNum_R,encoderNum_L;

unsigned char status = 0;
unsigned char v_offset = 40;
short v_basic = 300;
PID pid_L,pid_R;
int num_L,num_M,num_R;

void moter_control()
{
	if(L1 == 1)
	{
		status = 1;
		// 左慢右快
		pid_L.target_val = v_basic - v_offset;
		pid_R.target_val = v_basic + v_offset;
		num_L++;
//		printf("L:%d, M:%d, R:%d",num_L,num_M,num_R);
	}
	if(R1 == 1)
	{
		status = 1;
		// 右慢左快
		pid_L.target_val = v_basic + v_offset;
		pid_R.target_val = v_basic - v_offset;
		num_R++;
//		printf("L:%d, M:%d, R:%d",num_L,num_M,num_R);
	}
	if(M0 == 1)
	{
		status = 0;
		// 恢复
		pid_L.target_val = v_basic;
		pid_R.target_val = v_basic;
		num_M++;
//		printf("L:%d, M:%d, R:%d",num_L,num_M,num_R);		
	}
//	switch(status)
//	{
//		case 0:
//			if(L1 == 0)
//			{
//				status = 1;
//				// 左慢右快
//				pid_L.target_val = v_basic - v_offset;
//				pid_R.target_val = v_basic + v_offset;
//			}
//			else if(R1 == 0)
//			{
//				status = 1;
//				// 右慢左快
//				pid_L.target_val = v_basic + v_offset;
//				pid_R.target_val = v_basic - v_offset;
//			}
//			break;
//		case 1:
//			if(M0 == 0)
//			{
//				status = 0;
//				// 恢复
//				pid_L.target_val = v_basic;
//				pid_R.target_val = v_basic;
//			}
//			break;
//		default:break;
//	}

}

void PID_Init()
{
	pid_R.Kp = 2;
//	pid.Ki = 0;
//	pid.Kd = 0;
	
	pid_L.Kp = 2;
//	pid_L.Ki = Ki;
//	pid_L.Kd = Kd;
	
//	v_offset = Speed;
}

int PID_realize(int actual_val,PID pid)
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
    res_pwm_R = PID_realize(sum, pid_R);
	res_pwm_R += old_pwm_R;
	old_pwm_R = res_pwm_R;
	
	if(res_pwm_R > 3500)
		res_pwm_R = 3500;
	else if(res_pwm_R < 1000)
		res_pwm_R = 1000;
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
    res_pwm_L = PID_realize(sum, pid_L);
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
