#ifndef __CONTROL_H__
#define __CONTROL_H__	 
#include "sys.h"

#define INITIAL_SPEED	2000
#define OFFSET			50

typedef struct
{
	float target_val;   // 目标值
	float err;          // 偏差值
	float err_last;     // 上一个偏差值
	float Kp,Ki,Kd;     // 比例、积分、微分系数
	float integral;     // 积分值
	float output_val;   // 输出值
}PID;

void moter_control(void);
void PID_Init(void);
void AutoReloadCallbackR(void);
void AutoReloadCallbackL(void);

#endif
