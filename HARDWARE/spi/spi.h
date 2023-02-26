#ifndef __SPI_H__
#define __SPI_H__	 
#include "sys.h"

#define SPI2_NSS_PIN	GPIO_Pin_3
#define SPI2_SCK_PIN	GPIO_Pin_5
#define SPI2_MISO_PIN	GPIO_Pin_15
#define SPI2_MOSI_PIN	GPIO_Pin_4

void OLED_GPIO_Init(void);

#endif
