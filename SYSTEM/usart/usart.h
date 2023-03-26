#ifndef __USART_H__
#define __USART_H__
#include "stdio.h"	
#include "sys.h" 

void uart1_init(u32 bound);
void usart1_sendbyte(uint8_t data);
void DEBUG_printf(char* fmt,...) ;

#endif
