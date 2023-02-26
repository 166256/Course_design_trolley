#include "spi.h"

/*
	PB12 SPI2_NSS		PA15	MISO
	PB13 SPI2_SCK		PB3		NSS
	PB14 SPI2_MISO		PB4		MOSI
	PB15 SPI2_MOSI		PB5		SCK

*/

void OLED_GPIO_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure; 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	// 重映射需要开启AFIO时钟

	// c8t6的PB3、PB4、PA15引脚默认配置为JTAG功能，关闭JTAG功能，让其可以充当普通GPIO口来进行使用
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	// 引脚重映射函数
	
	GPIO_InitStructure.GPIO_Pin = SPI2_NSS_PIN; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	// NSS引脚初始化为普通IO推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = SPI2_SCK_PIN; 
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = SPI2_MISO_PIN; 
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = SPI2_MOSI_PIN; 
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
}

