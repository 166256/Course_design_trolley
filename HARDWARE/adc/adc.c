#include "adc.h"

#define NUM 1	//使用ADC路数

vu16 ADC_DMA_ConvertedValues[NUM]; //ADC数值存放的变量

void ADC_DMA_Init(void)
{ //DMA初始化设置
	DMA_InitTypeDef DMA_InitStructure;//定义DMA初始化结构体
	DMA_DeInit(DMA1_Channel1);//复位DMA通道1
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address; //定义 DMA通道外设基地址=ADC1_DR_Address
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_DMA_ConvertedValues; //定义DMA通道ADC数据存储器（其他函数可直接读此变量即是ADC值）
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//指定外设为源地址
	/*这里设置为四个采集通道*/
	DMA_InitStructure.DMA_BufferSize = NUM;//定义DMA缓冲区大小（根据ADC采集通道数量修改）
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//当前外设寄存器地址不变
	/*这里采用多通道采集*/
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//当前存储器地址：Disable不变，Enable递增（用于多通道采集）
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//定义外设数据宽度16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //定义存储器数据宽度16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA通道操作模式位环形缓冲模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA通道优先级高
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//禁止DMA通道存储器到存储器传输
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);//初始化DMA通道1
	DMA_Cmd(DMA1_Channel1, ENABLE); //使能DMA通道1
}
void ADC_GPIO_Init(void)
{ //GPIO初始化设置
	GPIO_InitTypeDef  GPIO_InitStructure; 	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);     //使能GPIOA、B、C口时钟  
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//使能DMA时钟（用于ADC的数据传送）
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//使能ADC1时钟
	/*多通道采集*/
    GPIO_InitStructure.GPIO_Pin = ADC_CH1 ; //选择端口                        
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //选择IO接口工作方式为模拟输入
	GPIO_Init(ADCPORT_1, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = ADC_CH3 | ADC_CH4; //选择端口                        
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //选择IO接口工作方式为模拟输入
//	GPIO_Init(ADCPORT_2, &GPIO_InitStructure);
}
void ADC_Configuration(void)
{ //初始化设置
	ADC_InitTypeDef ADC_InitStructure;//定义ADC初始化结构体变量
	ADC_GPIO_Init();//GPIO初始化设置
	ADC_DMA_Init();//DMA初始化设置
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//ADC1和ADC2工作为独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; //使能扫描
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//ADC转换工作在连续模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//有软件控制转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//转换数据右对齐
	/*这里是多通道采集*/
	ADC_InitStructure.ADC_NbrOfChannel = NUM;//顺序进行规则转换的ADC通道的数目（根据ADC采集通道数量修改）
	ADC_Init(ADC1, &ADC_InitStructure); //根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器
	//设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
	//ADC1,ADC通道x,规则采样顺序值为y,采样时间为28周期

	/*开启四通道读取ADC*/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_28Cycles5);//ADC1选择信道x,采样顺序y,采样时间n个周期
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 2, ADC_SampleTime_28Cycles5);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_28Cycles5);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_28Cycles5);
	ADC_DMACmd(ADC1, ENABLE);// 开启ADC的DMA支持（要实现DMA功能，还需独立配置DMA通道等参数）
	ADC_Cmd(ADC1, ENABLE);//使能ADC1
	ADC_ResetCalibration(ADC1); //重置ADC1校准寄存器
	while(ADC_GetResetCalibrationStatus(ADC1));//等待ADC1校准重置完成
	ADC_StartCalibration(ADC1);//开始ADC1校准
	while(ADC_GetCalibrationStatus(ADC1));//等待ADC1校准完成
	ADC_SoftwareStartConvCmd(ADC1, ENABLE); //使能ADC1软件开始转换
}

#if 0
//卡尔曼滤波初始化
void Kalman_Init(KalmanStr *p)
{
	//我们实现的得到滤波过后的ADC数据，所以为一维滤波
	/*
		what's Q&R ?
		状态量：x_k = Ax_(k-1)+Bu_(k-1)+w_(k-1)     w~p(0,Q)	w为过程误差(噪声) 符合高斯分布  Q为其方差
		测量量：z_k = Hx_k+v_k		v~p(0,R)	v为测量误差  同样符合高斯分布  R为其方差
	预测：
		先验值：^x_k = A^x_(k-1)+B^u_(k-1)	
		先验误差协方差：p_k- = Ap_(k-1)A_T+Q
	
	校正：
		卡尔曼增益系数：K_k=(p_k-)*H_T/(H(p_k-)+R)
		后验估计：x_k = ^x_k+K_k(z_k-H^x_k)
		更新误差协方差：p_k = (1-K_kH)(p_k-)
	*/
	p->A = 1;	//一维状态矩阵
    p->H = 1;	//一维测量矩阵
    p->P = 0.1;		//预测误差
    p->Q = 0.05;//过程(状态)误差方差
    p->R = 0.1;//测量误差方差
    p->B = 0.1;		//输入系数矩阵
    p->u = 0;	//系统输入
    p->filterValue = 0;		//滤波后的数据

}

//一维滤波函数
vu16 Kalman_Filter(KalmanStr *p,vu16 ADC_CvValue)
{
	//int i;	//遍历次数
	//for(i = 0;i < NUM;i++)
	//{
	float predictValue = p->A*p->filterValue+p->B*p->u;//计算预测值
    p->P = p->A*p->A*p->P + p->Q;//求协方差
    p->KalmanGain = p->P * p->H /(p->P * p->H * p->H + p->R);//计算卡尔曼增益
    p->filterValue = predictValue + (ADC_CvValue - predictValue)*p->KalmanGain;//计算输出的值
    p->P = (1 - p->KalmanGain* p->H)*p->P;//更新协方差
    return p->filterValue;

	//}
}

//一阶滤波算法
vu16 First_Order_Filter(vu16 ADC_CvValue)
{
	//int i;	//遍历次数
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






























