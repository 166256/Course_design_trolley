#include "sys.h"
#include "usart.h"	

extern unsigned int K;
unsigned int offset1,offset2,offset3,Speed;
unsigned char checksum = 0;
unsigned char usart1_status = 0;
unsigned char decode_data[RECEIVE_NUM] = {0};
unsigned char num = 0;
unsigned char packet_data[SENT_DATA] = {0};

void uart1_init(u32 bound){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		// USART1_TX  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	// �����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				// USART1_RX	  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	// ��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;// ��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		// �����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			// IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	
  
	USART_InitStructure.USART_BaudRate = bound;										// ���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// �ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;								// ����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	// ��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// �շ�ģʽ

	USART_Init(USART1, &USART_InitStructure); 		// ��ʼ������1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	// ʹ�ܴ��ڽ����ж�
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);	// ʹ�ܴ��ڿ����ж�
	USART_Cmd(USART1, ENABLE);                    	// ʹ�ܴ���1 
}

void usart1_sendbyte(uint8_t data)
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET); //�ȴ����ͼĴ���Ϊ�ղ��ܷ�����һ���ַ�	
	USART1->DR = data;
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC) != SET);		//�ȴ��������
}

void packet_bluedata(int speed)
{
	// ��ͷ��β
	packet_data[0] = 0xA5;
	packet_data[SENT_DATA - 1] = 0x5A;
	
	// Ҫ���͵�����
	packet_data[1] = speed & 0x0F;
	packet_data[2] = speed & 0xF0;
	
	// У��λ
	packet_data[SENT_DATA - 2] = packet_data[1] + packet_data[2];
	
	// ����
	for(unsigned char i = 0; i < SENT_DATA; i++)
	{
		usart1_sendbyte(packet_data[i]);
	}
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
					if(checksum == decode_data[RECEIVE_NUM -2]) // У����ȷ
					{
						offset1 = decode_data[1];
						offset2 = decode_data[2];
						offset3 = decode_data[3];
						Speed = decode_data[4] | decode_data[5] << 8 ;
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
/*
*********************************************************************************************************
*	�� �� ��: USART1_IRQHandler                    
*	����˵��: ����1���պ���������������λ�����͵����ݰ�   
*	��    ��: ��
*	�� �� ֵ: ��    
*********************************************************************************************************
*/
void USART1_IRQHandler(void)                	
{      
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  
	{
		USART_ClearFlag(USART1, USART_FLAG_RXNE);
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		unsigned char recvbyte = USART_ReceiveData(USART1);	
		decode_bluedata(recvbyte);
	}
	if(USART_GetITStatus(USART1,USART_IT_IDLE)!=RESET)
	{	
		USART_ReceiveData(USART1);	//���Ĳο��ֲ� ������������־λ����
	}
} 

