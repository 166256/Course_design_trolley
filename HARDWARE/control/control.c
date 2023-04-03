#include "control.h"
#include "Tim_encoder_speed.h"
#include "usart.h"
#include "protocol.h"
#include "Tim_pwm.h"
#include "mpu9250.h"

extern char L2,L1,M0,R1,R2;	// 传感器的状态
extern volatile int16_t encoderNum_R,encoderNum_L; // 编码器读数

unsigned char path_status = 0;	// 记录现在是在哪一段赛道
unsigned char status = 2;	// 记录现在是从左往右数的第几个传感器检测到黑线
int Position_error = 0;		// 方向环PID的error，控制方法2
char error_sensor[7] = {0};	// 记录在一个控制周期内，每个传感器各检测到几次，索引是从左往右数的第几个传感器（包括两个虚拟传感器）
int error = 0;				// 方向环PID的error，控制方法1
short v_basic = 120;			// 电机基础速度
short v_target = 0;			// 引入弯道降速后，存储目标的
PID pid_L,pid_R;			// 速度环的PID结构体变量

float Position_KP = 0.75,Position_KI = 0,Position_KD = 0.4;	// 方向环PID参数
unsigned char lock = 0;
void read_status()
{
	if((M0 && L1 && R1 && (R2 || L2)) == 1 && (lock == 0)) // 所有灯都灭了
	{
		lock = 1;
		path_status = (path_status + 1) % 3;
	}
	if(lock == 1 && (M0 && L1 && R1 && (R2 || L2)) == 0)
	{
		lock =0;
	}
	switch(path_status)
	{
		case 0:
			v_basic = 120; // 120 // 130
			Position_KP = 0.09; // 0.09 // 0.085 sudu80
			Position_KD = 0.2; // 0.2 // 0.4
			break;
		case 1:
			v_basic = 120;
			Position_KP = 0.08;
			Position_KD = 0;
			break;
		case 2:
			v_basic = 95;
			Position_KP = 0.08;
			Position_KD = 0;
			break;
		default:break;
	}
}

void read_error()
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
		if(status == 4) // 右
		{
			error_sensor[6]++;
			error = 4;
		}
		else if(status == 0) // 左
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
#if WEIGNT_LOWER==0
		position_error += error_sensor[i] * (i - 3) * ms;
#elif WEIGNT_LOWER==1
		if(i==6)
			position_error += (int)(error_sensor[i] * 4);
		else if(i==0)
			position_error += (int)(error_sensor[i] * -4);
		else if(i==5)
			position_error += (int)(error_sensor[i] * 3);
		else if(i==1)
			position_error += (int)(error_sensor[i] * -3);
		else if(i>3)
			position_error += (int)(error_sensor[i] * 2);
		else if(i<3)
			position_error += (int)(error_sensor[i] * -2);
#endif
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
	
	v_target = v_basic;
}

void offset_modify()
{
	v_basic = Speed;
	v_target = v_basic;
	pid_R.target_val = v_basic;
	pid_L.target_val = v_basic;
//	Position_KP = (float)offset1 / 10;
	Position_KP = (float)offset1 / 1000;
//	Position_KI = (float)offset2 / 100;
	Position_KD = (float)offset3 / 10;
	
	// 调试电机用
	pid_L.Kp = (float)offset2 / 10;
//	pid_L.Ki = (float)offset2 / 100;
//	pid_L.Kd = (float)offset3 / 100;
	
	pid_R.Kp = (float)offset2 / 10;
//	pid_R.Ki = (float)offset2 / 100;
//	pid_R.Kd = (float)offset3 / 100;
}
float Bias,Integral_bias,Last_Bias,Bias_2;
int PID_dir(int Error,int Target)   //方向PID(位置式)//Target=0;
{  
	int Output = 0;
	
	Bias = (float)(Error - Target);                                  //计算偏差
	Integral_bias += Bias;                                  //求出偏差的积分
	if(Integral_bias < POS_I_MIN)
		Integral_bias = POS_I_MIN;
	else if(Integral_bias > POS_I_MAX)
		Integral_bias = POS_I_MAX;
	
	Bias_2 = (Bias > 0) ? (Bias * Bias) : -(Bias * Bias);
#if PID_2==0
	Output = (int)(Position_KP * Bias +
#elif PID_2==1
	Output = (int)(Position_KP * Bias_2 +
#endif
		     Position_KI * Integral_bias + 
		     Position_KD * (Bias - Last_Bias));       //位置式PID控制器
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

void v_change()
{
//	v_basic = (short)((float)v_target * (1 - (float)fabs((double)Mov_9250.gyro_z)/383*90)); // 最大值为4000 * 3.14159 / 16.4f / 180

	v_basic = (short)((float)v_target * (1 - (float)((float)fabs((float)Position_error)/240)));
}

volatile short res_pwm_R = 0,res_pwm_L = 0; /*PWM值（PID输出）*/
volatile short old_pwm_R = 0,old_pwm_L = 0;
int offset_R = 0;
//周期定时器的回调函数
void AutoReloadCallbackR(int Error)
{
	offset_R = PID_dir(Error,0);
	
#if PID_DIR_ADD==1
	pid_R.target_val -= offset_R;
#elif PID_DIR_ADD==0
	pid_R.target_val = v_basic - offset_R;
#endif
	
	int sum = 0;/*编码器值（PID输入）*/
	
    /* 读取编码器测量的速度值 */
	sum = encoderNum_R;
    /*进行PID运算，得到PWM输出值*/
    res_pwm_R = PID_speed(sum, pid_R);
	res_pwm_R += old_pwm_R;
	old_pwm_R = res_pwm_R;
	
	if(res_pwm_R > 7100)
		res_pwm_R = 7100;
	else if(res_pwm_R < 1500)
		res_pwm_R = 1500;
	/*根据PWM值控制电机转动*/
	TIM_SetCompare4(TIM4,res_pwm_R);
	
    /*给上位机通道1发送实际值*/
//	packet_bluedata(sum);
}

int offset_L  = 0;
//周期定时器的回调函数
void AutoReloadCallbackL(int Error)
{
	offset_L = PID_dir(Error,0);
#if PID_DIR_ADD==1
	pid_L.target_val += offset_L;
#elif PID_DIR_ADD==0
	pid_L.target_val = v_basic + offset_L;
#endif
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
	tim4_gpio_config();
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
