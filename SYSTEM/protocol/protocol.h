#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__
#include "sys.h" 

#define SHORT_SPLIT_CHAR(x, y)  motor_buffer[x] = (y & 0x00FF);\
                                motor_buffer[(x)+1] = (y & 0xFF00) >> 8;

#define INT_SPLIT_CHAR(x, y)    motor_buffer[x] = (y & 0x000000FF);\
								motor_buffer[(x)+1] = (y & 0x0000FF00) >> 8;\
								motor_buffer[(x)+2] = (y & 0x00FF0000) >> 16;\
								motor_buffer[(x)+3] = (y & 0xFF000000) >> 24;

#define RECEIVE_NUM	11
#define SENT_DATA	20

extern char offset1,offset2,offset3,start_flag,Speed;

void packet_bluedata(unsigned char buffer[SENT_DATA - 3]);
void decode_bluedata(unsigned char data);
void UART_Send_Message(u8 *Data,u8 lenth);
void short2char(unsigned char b[2],int a,int c);

#endif
