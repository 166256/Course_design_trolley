#ifndef __TCRT500L_H__
#define __TCRT500L_H__	 
#include "sys.h"

void tcrt500l_init(void);

#define	TCRT_L2	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2)
#define TCRT_L1	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)
#define TCRT_M0	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)
#define TCRT_R1	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)
#define TCRT_R2	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)
		 				    
#endif
