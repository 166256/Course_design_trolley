#include "protocol.h"
#include "usart.h"

unsigned int offset1,offset2,offset3,start_flag,Speed;
unsigned char checksum = 0;
unsigned char usart1_status = 0;
unsigned char decode_data[RECEIVE_NUM] = {0};
unsigned char num = 0;
unsigned char packet_data[SENT_DATA] = {0};

/*
*********************************************************************************************************
*	函 数 名: decode_bluedata                    
*	功能说明: 使用状态机接收蓝牙上位机发送的数据包，并进行解包
*	形    参: data 串口1接收的数据
*	返 回 值: 无    
*********************************************************************************************************
*/
void packet_bluedata(unsigned char buffer[SENT_DATA - 3])
{
	// 包头包尾
	packet_data[0] = 0xA5;
	packet_data[SENT_DATA - 1] = 0x5A;
	
	// 填装
	for(unsigned char i = 0; i < (SENT_DATA - 3); i++)
		packet_data[i+1] = buffer[i];
	
	// 校验位
	for(unsigned char i = 1; i < (SENT_DATA - 2); i++)
		packet_data[SENT_DATA - 2] += packet_data[i];
	
	// 发送
	for(unsigned char i = 0; i < SENT_DATA; i++)
		usart1_sendbyte(packet_data[i]);

	packet_data[SENT_DATA - 2] = 0;
}

/*
*********************************************************************************************************
*	函 数 名: decode_bluedata                    
*	功能说明: 使用状态机接收蓝牙上位机发送的数据包，并进行解包
*	形    参: data 串口1接收的数据
*	返 回 值: 无    
*********************************************************************************************************
*/
void decode_bluedata(unsigned char data)
{
	switch(usart1_status)
	{
		case 0:	// 检测包头
			if(data == 0xA5)
			{
				usart1_status = 1;
				decode_data[0] = data;
				num = 1;
			}
			break;
		case 1: // 采集数据
			if(num == RECEIVE_NUM ) // 采集完了一个数据包
			{ 
				if(decode_data[0] == 0xA5 && decode_data[RECEIVE_NUM -1] == 0x5A) // 包头包尾正确
				{
					for(unsigned char i = 1;i < RECEIVE_NUM - 2;i++) // 计算校验和
						checksum += decode_data[i];
					if(checksum == decode_data[RECEIVE_NUM -2]) // 校验正确则开始解包
					{
						offset1 = decode_data[1];
						offset2 = decode_data[2];
						offset3 = decode_data[3];
						start_flag = decode_data[4];
						Speed = decode_data[5] | decode_data[6] << 8 ;
					}
					checksum = 0;
				}
				usart1_status = 0;
			} 
			else
			{
				decode_data[num] = data;
				num++;
			}
			break;
		default:break;
	}
}

