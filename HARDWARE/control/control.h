#ifndef __CONTROL_H__
#define __CONTROL_H__	 
#include "sys.h"
#include "protocol.h"

typedef struct
{
	int target_val;   // Ŀ��ֵ
	int err;          // ƫ��ֵ
	int err_last;     // ��һ��ƫ��ֵ
	int integral;     // ����ֵ
	int output_val;   // ���ֵ
	float Kp,Ki,Kd;     // ���������֡�΢��ϵ��
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
