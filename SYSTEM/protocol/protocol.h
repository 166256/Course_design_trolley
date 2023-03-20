#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__
#include "sys.h" 

#define RECEIVE_NUM	11
#define SENT_DATA	19

extern unsigned int offset1,offset2,offset3,start_flag,Speed;

void packet_bluedata(unsigned char buffer[SENT_DATA - 3]);
void decode_bluedata(unsigned char data);
void UART_Send_Message(u8 *Data,u8 lenth);
void short2char(unsigned char b[2],int a,int c);

#endif
