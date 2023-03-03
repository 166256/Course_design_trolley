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
				// 左慢右快
				TIM_SetCompare2(TIM4,INITIAL_SPEED - OFFSET * K); // 左轮
				TIM_SetCompare4(TIM4,INITIAL_SPEED + OFFSET * K); // 右轮
			}
			else if(R1 == 0)
			{
				status = 1;
				// 右慢左快
				TIM_SetCompare2(TIM4,INITIAL_SPEED + OFFSET * K);
				TIM_SetCompare4(TIM4,INITIAL_SPEED - OFFSET * K);
			}
			break;
		case 1:
			if(M0 == 0)
			{
				status = 0;
				// 恢复
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

//周期定时器的回调函数
void AutoReloadCallbackR()
{
	float sum = 0;/*编码器值（PID输入）*/
	int res_pwm = 0;/*PWM值（PID输出）*/
	
    /* 读取编码器测量的速度值 */
	sum = calc_motor_Right_rotate_speed();
    /*进行PID运算，得到PWM输出值*/
    res_pwm = PID_realize(sum);
	
	if(res_pwm > 3500)
		res_pwm = 3500;
	else if(res_pwm < 1000)
		res_pwm = 1000;
	/*根据PWM值控制电机转动*/
	TIM_SetCompare4(TIM4,res_pwm);
	
    /*给上位机通道1发送实际值*/
//	set_computer_value(SEND_FACT_CMD, CURVES_CH1, &sum, 1); 
}

//周期定时器的回调函数
void AutoReloadCallbackL()
{
	float sum = 0;/*编码器值（PID输入）*/
	int res_pwm = 0;/*PWM值（PID输出）*/
	
    /* 读取编码器测量的速度值 */
	sum = calc_motor_Left_rotate_speed();
    /*进行PID运算，得到PWM输出值*/
    res_pwm = PID_realize(sum);
	
	if(res_pwm > 3500)
		res_pwm = 3500;
	else if(res_pwm < 1000)
		res_pwm = 1000;
	/*根据PWM值控制电机转动*/
	TIM_SetCompare2(TIM4,res_pwm);
	
    /*给上位机通道1发送实际值*/
//	set_computer_value(SEND_FACT_CMD, CURVES_CH1, &sum, 1); 
}
