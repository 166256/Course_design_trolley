#ifndef __CONTROL_H__
#define __CONTROL_H__	 
#include "sys.h"

#define INITIAL_SPEED	2000
#define OFFSET			50

typedef struct
{
	int target_val;   // Ŀ��ֵ
	int err;          // ƫ��ֵ
	int err_last;     // ��һ��ƫ��ֵ
	int Kp,Ki,Kd;     // ���������֡�΢��ϵ��
	int integral;     // ����ֵ
	int output_val;   // ���ֵ
}PID;

extern short v_basic;
extern PID pid_L,pid_R;

void moter_control(void);
void PID_Init(void);
void offset_modify(void);
void AutoReloadCallbackR(void);
void AutoReloadCallbackL(void);

#endif
