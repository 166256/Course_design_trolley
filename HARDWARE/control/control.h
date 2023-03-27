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
extern volatile short res_pwm_R,res_pwm_L;
extern int error;
extern int offset_R,offset_L;
extern int Position_error;
extern float Position_KP,Position_KI,Position_KD;	// ����PID����

//#define PID_DIR_ADD 0
#define PID_DIR 	1

void moter_control(void);
void PID_Init(void);
void offset_modify(void);
void AutoReloadCallbackR(int Error);
void AutoReloadCallbackL(int Error);
void motor_init(void);
void motor_stop(void);
void read_status(void);
int get_error(unsigned char ms);

#endif
