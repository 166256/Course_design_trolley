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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	// ��ӳ����Ҫ����AFIOʱ��

	// c8t6��PB3��PB4��PA15����Ĭ������ΪJTAG���ܣ��ر�JTAG���ܣ�������Գ䵱��ͨGPIO��������ʹ��
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	// ������ӳ�亯��
	
	GPIO_InitStructure.GPIO_Pin = SPI2_NSS_PIN; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	// NSS���ų�ʼ��Ϊ��ͨIO�������
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = SPI2_SCK_PIN; 
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = SPI2_MISO_PIN; 
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = SPI2_MOSI_PIN; 
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
}

