#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__
#include "sys.h" 

#define RECEIVE_NUM	9
#define SENT_DATA	15

extern unsigned int offset1,offset2,offset3,start_flag,Speed;

void packet_bluedata(unsigned char buffer[SENT_DATA - 3]);
void decode_bluedata(unsigned char data);

#endif
