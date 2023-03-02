#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "sys.h" 
 /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/   		   
//IO��������
#define MPU_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}   //�˿����ø߼Ĵ���������PB11���룬��������
#define MPU_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}   //�������

//IO��������	 
#define IIC_SCL    PBout(10) 		//SCL
#define IIC_SDA    PBout(11) 		//SDA	 
#define MPU_READ_SDA   PBin(11) 		//����SDA 

//IIC���в�������
void IIC_Delay(void);				//MPU IIC��ʱ����
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�
#endif
















