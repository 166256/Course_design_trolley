#include "control.h"

extern char L2,L1,M0,R1,R2;

unsigned char status = 0;
unsigned int K = 1;

void moter_control()
{
	switch(status)
	{
		case 0:
			if(L1 == 0)
			{
				status = 1;
				// �����ҿ�
				TIM_SetCompare2(TIM4,INITIAL_SPEED - OFFSET * K); // ����
				TIM_SetCompare4(TIM4,INITIAL_SPEED + OFFSET * K); // ����
			}
			else if(R1 == 0)
			{
				status = 1;
				// �������
				TIM_SetCompare2(TIM4,INITIAL_SPEED + OFFSET * K);
				TIM_SetCompare4(TIM4,INITIAL_SPEED - OFFSET * K);
			}
			break;
		case 1:
			if(M0 == 0)
			{
				status = 0;
				// �ָ�
				TIM_SetCompare2(TIM4,INITIAL_SPEED);
				TIM_SetCompare4(TIM4,INITIAL_SPEED);
			}
			break;
		default:break;
	}
}
