#ifndef __GPIO_H__
#define __GPIO_H__	 
#include "sys.h"

#define KEY_SCAN()	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)	// ¶ÁÈ¡°´¼ü1(KEY1)

void key_gpio_config(void);
	
#endif
