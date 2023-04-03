#ifndef __CONTROL_H__
#define __CONTROL_H__	 
#include "sys.h"
#include "protocol.h"

typedef struct
{
	int target_val;   // 目标值
	int err;          // 偏差值
	int err_last;     // 上一个偏差值
	int integral;     // 积分值
	int output_val;   // 输出值
	float Kp,Ki,Kd;     // 比例、积分、微分系数
}PID;

extern short v_basic;
extern PID pid_L,pid_R;
extern volatile short res_pwm_R,res_pwm_L;
extern int error;
extern int offset_R,offset_L;
extern int Position_error;
extern float Position_KP,Position_KI,Position_KD;	// 方向环PID参数

#define PID_DIR_ADD 	0	// 方向环PID改成增量
#define WEIGNT_LOWER	1	// 降低旁边传感器的权重
#define PID_2			0	// 将PID控制改成 y = k * x^2 ,用这种的时候，p得很小，0.0几
#define POS_I_MAX		1000
#define POS_I_MIN		(-1000)

void moter_control(void);
void PID_Init(void);
void offset_modify(void);
void AutoReloadCallbackR(int Error);
void AutoReloadCallbackL(int Error);
void motor_init(void);
void motor_stop(void);
void read_error(void);
void read_status(void);
int get_error(unsigned char ms);
void v_change(void);

#endif
