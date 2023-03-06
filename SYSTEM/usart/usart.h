#ifndef __USART_H__
#define __USART_H__
#include "stdio.h"	
#include "sys.h" 

#define RECEIVE_NUM	8
#define SENT_DATA	5

extern unsigned int offset1,offset2,K,Speed;

void uart1_init(u32 bound);
void usart1_sendbyte(uint8_t data);
void packet_bluedata(int speed);

#endif
