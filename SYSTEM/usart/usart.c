#include "sys.h"
#include "usart.h"	  

volatile unsigned char usart_flag;          //�������յ�������
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

extern unsigned int K;
unsigned int Kp,Ki,Kd,speed;
unsigned char checksum = 0;
unsigned char usart1_status = 0;
unsigned char Data[MAX_NUM] = {0};
unsigned char num = 0;
void deal_bluedata(unsigned char data)
{
	switch(usart1_status)
	{
		case 0:	// ����ͷ
			if(data == 0xA5)
			{
				usart1_status = 1;
				Data[0] = data;
				num = 1;
			}
			break;
		case 1: // �ɼ�����
			if(num == MAX_NUM ) // �ɼ�����һ�����ݰ�
			{ 
				if(Data[0] == 0xA5 && Data[MAX_NUM -1] == 0x5A) // ��ͷ��β��ȷ
				{
					for(unsigned char i = 1;i < MAX_NUM - 2;i++) // ����У���
						checksum += Data[i];
					if(checksum == Data[MAX_NUM -2]) // У����ȷ
					{
						Kp = Data[1];
						Ki = Data[2];
						Kd = Data[3];
						speed = Data[4] | Data[5] << 8 ;
					}
					checksum = 0;
				}
				usart1_status = 0;
			} 
			else
			{
				Data[num] = data;
				num++;
			}
			break;
		default:break;
	}
}

void USART1_IRQHandler(void)                	
{      
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  
	{
		USART_ClearFlag(USART1, USART_FLAG_RXNE);
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		unsigned char recvbyte = USART_ReceiveData(USART1);	
		deal_bluedata(recvbyte);
	}
	
	if(USART_GetITStatus(USART1,USART_IT_IDLE)!=RESET)
	{	
		USART_ReceiveData(USART1);	        //���Ĳο��ֲ� ������������־λ����
	}

} 


