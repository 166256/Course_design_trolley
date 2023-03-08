#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__
#include "sys.h" 

#define DECODE_TO_INT(x, y)     (x) = (int) (((int) decode_data[y] <<24) | \
                                ((int)decode_data[y+1] <<16) |\
                                ((int)decode_data[y+2])<<8 |\
                                ((int)decode_data[y+3]));

#endif
