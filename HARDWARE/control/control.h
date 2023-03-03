#ifndef __CONTROL_H__
#define __CONTROL_H__	 
#include "sys.h"

#define INITIAL_SPEED	2000
#define OFFSET			50

typedef struct
{
	int target_val;   // 目标值
	int err;          // 偏差值
	int err_last;     // 上一个偏差值
	int Kp,Ki,Kd;     // 比例、积分、微分系数
	int integral;     // 积分值
	int output_val;   // 输出值
}PID;

void moter_control(void);
void PID_Init(void);
void AutoReloadCallbackR(void);
void AutoReloadCallbackL(void);

#endif
