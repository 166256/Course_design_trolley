#include "adc.h"

#define NUM 1	//ʹ��ADC·��

vu16 ADC_DMA_ConvertedValues[NUM]; //ADC��ֵ��ŵı���

void ADC_DMA_Init(void)
{ //DMA��ʼ������
	DMA_InitTypeDef DMA_InitStructure;//����DMA��ʼ���ṹ��
	DMA_DeInit(DMA1_Channel1);//��λDMAͨ��1
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address; //���� DMAͨ���������ַ=ADC1_DR_Address
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_DMA_ConvertedValues; //����DMAͨ��ADC���ݴ洢��������������ֱ�Ӷ��˱�������ADCֵ��
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//ָ������ΪԴ��ַ
	/*��������Ϊ�ĸ��ɼ�ͨ��*/
	DMA_InitStructure.DMA_BufferSize = NUM;//����DMA��������С������ADC�ɼ�ͨ�������޸ģ�
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//��ǰ����Ĵ�����ַ����
	/*������ö�ͨ���ɼ�*/
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//��ǰ�洢����ַ��Disable���䣬Enable���������ڶ�ͨ���ɼ���
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//�����������ݿ��16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //����洢�����ݿ��16λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMAͨ������ģʽλ���λ���ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMAͨ�����ȼ���
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//��ֹDMAͨ���洢�����洢������
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);//��ʼ��DMAͨ��1
	DMA_Cmd(DMA1_Channel1, ENABLE); //ʹ��DMAͨ��1
}
void ADC_GPIO_Init(void)
{ //GPIO��ʼ������
	GPIO_InitTypeDef  GPIO_InitStructure; 	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);     //ʹ��GPIOA��B��C��ʱ��  
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//ʹ��DMAʱ�ӣ�����ADC�����ݴ��ͣ�
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//ʹ��ADC1ʱ��
	/*��ͨ���ɼ�*/
    GPIO_InitStructure.GPIO_Pin = ADC_CH1 ; //ѡ��˿�                        
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //ѡ��IO�ӿڹ�����ʽΪģ������
	GPIO_Init(ADCPORT_1, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = ADC_CH3 | ADC_CH4; //ѡ��˿�                        
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //ѡ��IO�ӿڹ�����ʽΪģ������
//	GPIO_Init(ADCPORT_2, &GPIO_InitStructure);
}
void ADC_Configuration(void)
{ //��ʼ������
	ADC_InitTypeDef ADC_InitStructure;//����ADC��ʼ���ṹ�����
	ADC_GPIO_Init();//GPIO��ʼ������
	ADC_DMA_Init();//DMA��ʼ������
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//ADC1��ADC2����Ϊ����ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; //ʹ��ɨ��
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//ADCת������������ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//���������ת��
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ת�������Ҷ���
	/*�����Ƕ�ͨ���ɼ�*/
	ADC_InitStructure.ADC_NbrOfChannel = NUM;//˳����й���ת����ADCͨ������Ŀ������ADC�ɼ�ͨ�������޸ģ�
	ADC_Init(ADC1, &ADC_InitStructure); //����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���
	//����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ��
	//ADC1,ADCͨ��x,�������˳��ֵΪy,����ʱ��Ϊ28����

	/*������ͨ����ȡADC*/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_28Cycles5);//ADC1ѡ���ŵ�x,����˳��y,����ʱ��n������
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 2, ADC_SampleTime_28Cycles5);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_28Cycles5);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_28Cycles5);
	ADC_DMACmd(ADC1, ENABLE);// ����ADC��DMA֧�֣�Ҫʵ��DMA���ܣ������������DMAͨ���Ȳ�����
	ADC_Cmd(ADC1, ENABLE);//ʹ��ADC1
	ADC_ResetCalibration(ADC1); //����ADC1У׼�Ĵ���
	while(ADC_GetResetCalibrationStatus(ADC1));//�ȴ�ADC1У׼�������
	ADC_StartCalibration(ADC1);//��ʼADC1У׼
	while(ADC_GetCalibrationStatus(ADC1));//�ȴ�ADC1У׼���
	ADC_SoftwareStartConvCmd(ADC1, ENABLE); //ʹ��ADC1�����ʼת��
}

#if 0
//�������˲���ʼ��
void Kalman_Init(KalmanStr *p)
{
	//����ʵ�ֵĵõ��˲������ADC���ݣ�����Ϊһά�˲�
	/*
		what's Q&R ?
		״̬����x_k = Ax_(k-1)+Bu_(k-1)+w_(k-1)     w~p(0,Q)	wΪ�������(����) ���ϸ�˹�ֲ�  QΪ�䷽��
		��������z_k = Hx_k+v_k		v~p(0,R)	vΪ�������  ͬ�����ϸ�˹�ֲ�  RΪ�䷽��
	Ԥ�⣺
		����ֵ��^x_k = A^x_(k-1)+B^u_(k-1)	
		�������Э���p_k- = Ap_(k-1)A_T+Q
	
	У����
		����������ϵ����K_k=(p_k-)*H_T/(H(p_k-)+R)
		������ƣ�x_k = ^x_k+K_k(z_k-H^x_k)
		�������Э���p_k = (1-K_kH)(p_k-)
	*/
	p->A = 1;	//һά״̬����
    p->H = 1;	//һά��������
    p->P = 0.1;		//Ԥ�����
    p->Q = 0.05;//����(״̬)����
    p->R = 0.1;//��������
    p->B = 0.1;		//����ϵ������
    p->u = 0;	//ϵͳ����
    p->filterValue = 0;		//�˲��������

}

//һά�˲�����
vu16 Kalman_Filter(KalmanStr *p,vu16 ADC_CvValue)
{
	//int i;	//��������
	//for(i = 0;i < NUM;i++)
	//{
	float predictValue = p->A*p->filterValue+p->B*p->u;//����Ԥ��ֵ
    p->P = p->A*p->A*p->P + p->Q;//��Э����
    p->KalmanGain = p->P * p->H /(p->P * p->H * p->H + p->R);//���㿨��������
    p->filterValue = predictValue + (ADC_CvValue - predictValue)*p->KalmanGain;//���������ֵ
    p->P = (1 - p->KalmanGain* p->H)*p->P;//����Э����
    return p->filterValue;

	//}
}

//һ���˲��㷨
vu16 First_Order_Filter(vu16 ADC_CvValue)
{
	//int i;	//��������
	//vu16 baro[NUM];
	//vu16 *b;
	//b = ADC_CvValue;
	
//	for(i = 0;i < NUM;i++)
//	{
//		if(i > 0)
//			baro[i] = a*b[i] + (1.0f - a) * baro[i-1];
//		else
//			baro[i] = a*b[i] + (1.0f - a) * 0;
//	}
	Baro = a*ADC_CvValue + (1 - a) * Baro;
	return Baro;
}

#endif






























