#include "sys.h"
#include "usart.h"	  

volatile unsigned char usart_flag;          //蓝牙接收到的数据
void uart1_init(u32 bound){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		// USART1_TX  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	// 复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);
   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				// USART1_RX	  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	// 浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;// 抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		// 子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			// IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	
  
	USART_InitStructure.USART_BaudRate = bound;										// 串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								// 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	// 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 收发模式

	USART_Init(USART1, &USART_InitStructure); 		// 初始化串口1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	// 使能串口接受中断
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);	// 使能串口空闲中断
	USART_Cmd(USART1, ENABLE);                    	// 使能串口1 
}

void usart1_sendbyte(uint8_t data)
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET); //等待发送寄存器为空才能发送下一个字符	
	USART1->DR = data;
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC) != SET);		//等待发送完成
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
		case 0:	// 检测包头
			if(data == 0xA5)
			{
				usart1_status = 1;
				Data[0] = data;
				num = 1;
			}
			break;
		case 1: // 采集数据
			if(num == MAX_NUM ) // 采集完了一个数据包
			{ 
				if(Data[0] == 0xA5 && Data[MAX_NUM -1] == 0x5A) // 包头包尾正确
				{
					for(unsigned char i = 1;i < MAX_NUM - 2;i++) // 计算校验和
						checksum += Data[i];
					if(checksum == Data[MAX_NUM -2]) // 校验正确
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
		USART_ReceiveData(USART1);	        //查阅参考手册 软件序列清除标志位流程
	}

} 


