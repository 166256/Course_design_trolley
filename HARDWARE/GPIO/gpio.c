#include "gpio.h"

void key_gpio_config()
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 			 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	    		 
	GPIO_Init(GPIOA, &GPIO_InitStructure);	  		
}
