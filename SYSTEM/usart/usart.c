#include "sys.h"
#include "usart.h"	



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
