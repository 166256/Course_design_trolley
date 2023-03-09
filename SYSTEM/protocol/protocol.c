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
*	�� �� ��: decode_bluedata                    
*	����˵��: ʹ��״̬������������λ�����͵����ݰ��������н��
*	��    ��: data ����1���յ�����
*	�� �� ֵ: ��    
*********************************************************************************************************
*/
void packet_bluedata(unsigned char buffer[SENT_DATA - 3])
{
	// ��ͷ��β
	packet_data[0] = 0xA5;
	packet_data[SENT_DATA - 1] = 0x5A;
	
	// ��װ
	for(unsigned char i = 0; i < (SENT_DATA - 3); i++)
		packet_data[i+1] = buffer[i];
	
	// У��λ
	for(unsigned char i = 1; i < (SENT_DATA - 2); i++)
		packet_data[SENT_DATA - 2] += packet_data[i];
	
	// ����
	for(unsigned char i = 0; i < SENT_DATA; i++)
		usart1_sendbyte(packet_data[i]);

	packet_data[SENT_DATA - 2] = 0;
}

/*
*********************************************************************************************************
*	�� �� ��: decode_bluedata                    
*	����˵��: ʹ��״̬������������λ�����͵����ݰ��������н��
*	��    ��: data ����1���յ�����
*	�� �� ֵ: ��    
*********************************************************************************************************
*/
void decode_bluedata(unsigned char data)
{
	switch(usart1_status)
	{
		case 0:	// ����ͷ
			if(data == 0xA5)
			{
				usart1_status = 1;
				decode_data[0] = data;
				num = 1;
			}
			break;
		case 1: // �ɼ�����
			if(num == RECEIVE_NUM ) // �ɼ�����һ�����ݰ�
			{ 
				if(decode_data[0] == 0xA5 && decode_data[RECEIVE_NUM -1] == 0x5A) // ��ͷ��β��ȷ
				{
					for(unsigned char i = 1;i < RECEIVE_NUM - 2;i++) // ����У���
						checksum += decode_data[i];
					if(checksum == decode_data[RECEIVE_NUM -2]) // У����ȷ��ʼ���
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

