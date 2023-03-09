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
extern unsigned char motor_buffer[SENT_DATA - 3];
extern volatile short res_pwm_R,res_pwm_L;
extern int error;

void moter_control(void);
void PID_Init(void);
void offset_modify(void);
void AutoReloadCallbackR(void);
void AutoReloadCallbackL(void);
void motor_init(void);
void motor_stop(void);
void read_status(void);

#endif
