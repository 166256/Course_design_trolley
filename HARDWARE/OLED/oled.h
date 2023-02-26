
#ifndef __OLED_H__
#define __OLED_H__
			  	 
//========移植的时候，根据所选芯片引入相应的头文件==========，在main.h里面查看
#include "stm32f10x.h"
 	
//OLED模式设置
//0:4线串行模式
//1:并行8080模式
#define OLED_MODE 0
#define SIZE 16
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	  
  						  
//========移植的时候，根据引脚分配进行修改==========

//使用4线串行接口时使用 
#define OLED_RST_GPIO			GPIOB
#define OLED_RST_GPIO_PIN		GPIO_Pin_3	// NSS
#define OLED_DC_GPIO			GPIOA
#define OLED_DC_GPIO_PIN		GPIO_Pin_15 // MISO
#define OLED_SCLK_GPIO			GPIOB
#define OLED_SCLK_GPIO_PIN		GPIO_Pin_5	// SCK
#define OLED_SDIN_GPIO			GPIOB
#define OLED_SDIN_GPIO_PIN		GPIO_Pin_4	// MOSI


//CS
#define OLED_CS_Clr()  //NOT USE
#define OLED_CS_Set()  //NOT USE

//RES
#define OLED_RST_Clr() GPIO_WriteBit(OLED_RST_GPIO, OLED_RST_GPIO_PIN, Bit_RESET)	//RES RES => 接RES（RST）引脚
#define OLED_RST_Set() GPIO_WriteBit(OLED_RST_GPIO, OLED_RST_GPIO_PIN, Bit_SET)

//DC
#define OLED_DC_Clr() GPIO_WriteBit(OLED_DC_GPIO, OLED_DC_GPIO_PIN, Bit_RESET)
#define OLED_DC_Set() GPIO_WriteBit(OLED_DC_GPIO, OLED_DC_GPIO_PIN, Bit_SET)

//SCLK,D0
#define OLED_SCLK_Clr() GPIO_WriteBit(OLED_SCLK_GPIO, OLED_SCLK_GPIO_PIN, Bit_RESET)
#define OLED_SCLK_Set() GPIO_WriteBit(OLED_SCLK_GPIO, OLED_SCLK_GPIO_PIN, Bit_SET)

//SDIN,D1
#define OLED_SDIN_Clr() GPIO_WriteBit(OLED_SDIN_GPIO, OLED_SDIN_GPIO_PIN, Bit_RESET)
#define OLED_SDIN_Set() GPIO_WriteBit(OLED_SDIN_GPIO, OLED_SDIN_GPIO_PIN, Bit_SET)

 		     
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据


//OLED控制用函数
void OLED_WR_Byte(unsigned char dat,unsigned char cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);	   							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(unsigned char x,unsigned char y,unsigned char t);
void OLED_Fill(unsigned char x1,unsigned char y1,unsigned char x2,unsigned char y2,unsigned char dot);
void OLED_ShowChar(unsigned char x,unsigned char y,unsigned char chr);
void OLED_ShowNum(unsigned char x,unsigned char y,unsigned long num,unsigned char len,unsigned char size);
void OLED_ShowString(unsigned char x,unsigned char y, unsigned char *p);	 
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(unsigned char x,unsigned char y,unsigned char no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);

void OLED_Show_Variable(unsigned char x,unsigned char y,unsigned long number,unsigned char size);

#endif  
	 



