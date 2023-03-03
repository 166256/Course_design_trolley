#ifndef __CONTROL_H__
#define __CONTROL_H__	 
#include "sys.h"

#define INITIAL_SPEED	2000
#define OFFSET			50

typedef struct
{
	float target_val;   // Ŀ��ֵ
	float err;          // ƫ��ֵ
	float err_last;     // ��һ��ƫ��ֵ
	float Kp,Ki,Kd;     // ���������֡�΢��ϵ��
	float integral;     // ����ֵ
	float output_val;   // ���ֵ
}PID;

void moter_control(void);
void PID_Init(void);
void AutoReloadCallbackR(void);
void AutoReloadCallbackL(void);

#endif
