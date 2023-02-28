//这是一个利用编码器进行测速的.c文件

#include	"Tim_encoder_speed.h"

volatile int Encoder_Timer_Overflow_L = 0;		//全局变量，记录左编码器溢出次数
volatile int	Encoder_Timer_Overflow_R = 0;		//全局变量，记录右编码器溢出次数

volatile int16_t encoderNum_L = 0;			
volatile int16_t encoderNum_R = 0;
volatile float rotateSpeed_L = 0;
volatile float rotateSpeed_R = 0;

/*
*********************************************************************************************************
*	函 数 名: tim_encoderL_init                    
*	功能说明: 对左轮编码器所对应的定时器进行初始化     
*	形    参: 无
*	返 回 值: 无    
*	说    明：左轮编码器A相-->PA0-->TIM2_CH1,    B相-->PA1-->TIM2_CH2
*********************************************************************************************************
*/
void Tim_EncoderL_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;	
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 

//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	
//	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2,ENABLE);
	
//	TIM_DeInit(TIM2);
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);		
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;    // 向上计数
	TIM_TimeBaseInitStructure.TIM_Period = Encoder_TIM_Period;         // 重装载值，不可大于65535 因为F103的定时器是16位的
	TIM_TimeBaseInitStructure.TIM_Prescaler =1-1;                      // 分频系数1，即不分频
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);

	// 编码器配置
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); // 使用编码器模式3,进行4倍频		
	TIM_ICStructInit(&TIM_ICInitStructure); 
	TIM_ICInitStructure.TIM_ICFilter = 0;		// 输入通道的滤波参数
	TIM_ICInit(TIM2, &TIM_ICInitStructure);		// 输入通道初始化		
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);		// 清除TIM的更新标志位，更新标志是由于CNT计数到arr产生的
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);	// 使能定时器中断	
	TIM_SetCounter(TIM2,0);						// 设定计数器的初值为0	
	TIM_Cmd(TIM2,ENABLE);						// 使能控制定时器，计数器开始计数
}

/*
*********************************************************************************************************
*	函 数 名: tim_encoderR_init                    
*	功能说明: 对右轮编码器所对应的定时器进行初始化     
*	形    参: 无
*	返 回 值: 无    
*	说    明：右轮编码器A相-->PA6<-->TIM3_CH1,B相-->PA7<-->TIM3_CH2
*********************************************************************************************************
*/
void Tim_EncoderR_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;	
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure); 

	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);		
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;    // 向上计数
	TIM_TimeBaseInitStructure.TIM_Period = Encoder_TIM_Period;         // 重装载值，不可大于65535 因为F103的定时器是16位的
	TIM_TimeBaseInitStructure.TIM_Prescaler =1-1;                      // 分频系数1，即不分频
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);

	// 编码器配置
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //使用编码器模式3,进行4倍频		
	TIM_ICStructInit(&TIM_ICInitStructure); 
	TIM_ICInitStructure.TIM_ICFilter = 0;   	//输入通道的滤波参数
	TIM_ICInit(TIM3, &TIM_ICInitStructure);		//输入通道初始化	
	
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);		//清除TIM的更新标志位，更新标志是由于CNT计数到arr产生的
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);	//使能定时器中断	
	TIM_SetCounter(TIM3,0);						//设定计数器的初值为0	
	TIM_Cmd(TIM3,ENABLE);						//使能控制定时器，计数器开始计数
}

/*
*********************************************************************************************************
*	函 数 名: Get_Encoder_Cnt                    
*	功能说明: 读取编码器的计数值    
*	形    参: TIMx，取值为2和3，表示定时器2和3
*	返 回 值: 返回读取到相应定时器的CNT值    
*	说    明：这里通过将CNT的uint32类型的计数值， 转为int16类型，就可以利用正负来区分转动方向，
			  正数表示正转，负数表示反转。读完后将定时器的计数值清零
*********************************************************************************************************
*/
vu16 Get_Encoder_Cnt(vu16 TIMx)
{
	vu16 Encoder_cnt = 0;

	switch(TIMx)
	{
		case 2://当输入的是定时器2时
			Encoder_cnt = TIM2->CNT;	//采集编码器的计数值并保存。
			TIM_SetCounter(TIM2,0);			//读完后将定时器的计数值清零
			break;
		case 3://当输入的是定时器3时
			Encoder_cnt = TIM3->CNT;	//采集编码器的计数值并保存
			TIM_SetCounter(TIM3,0);			//读完将定时器的计数值清零
			break;
		default: 
			Encoder_cnt = 0;
			break;
	}
	return Encoder_cnt;
}

/*
*********************************************************************************************************
*	函 数 名: TIM2_IRQHandler                    
*	功能说明: 在一个定时中断里读取左轮编码器捕获到的脉冲数    
*	形    参: 无
*	返 回 值: 无   
*********************************************************************************************************
*/
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)	//如果TIM2的CNT计数计数到arr，将产生更新中断
	{
		Encoder_Timer_Overflow_L++;	//溢出次数
		TIM_SetCounter(TIM2,0);		//当发生溢出时，对CNT清0
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);		//清除中断标志位
}

/*
*********************************************************************************************************
*	函 数 名: TIM3_IRQHandler                    
*	功能说明: 在一个定时中断里读取右轮编码器捕获到的脉冲数    
*	形    参: 无
*	返 回 值: 无   
*********************************************************************************************************
*/
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET)	//如果TIM2的CNT计数计数到arr，将产生更新中断
	{
		Encoder_Timer_Overflow_R++;	//溢出次数
		TIM_SetCounter(TIM3,0);			//当发生溢出时，对CNT清0
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);		//清除中断标志位
}

/*
*********************************************************************************************************
*	函 数 名: calc_motor_Left_rotate_speed                    
*	功能说明: 利用编码器进行左轮测速   
*	形    参: 无
*	返 回 值: 无   
*	说    明：当全局变量time==Tim_time时调用一次该函数
*********************************************************************************************************
*/
void calc_motor_Left_rotate_speed(void)
{	
	/*读取编码器的值，正负代表旋转方向，溢出次数要加上去*/
	encoderNum_L = Get_Encoder_Cnt(2)+(65535*Encoder_Timer_Overflow_L);
	/* 转速(1秒钟转多少圈)=单位时间内(100ms)的计数值/总分辨率*时间系数,总分辨率即车轮转过一圈的脉冲数，时间系数是1000ms/定时时长 */
	rotateSpeed_L = (float)encoderNum_L/Total_Resolution*(1000/Tim_time);
}

/*
*********************************************************************************************************
*	函 数 名: calc_motor_Right_rotate_speed                    
*	功能说明: 利用编码器进行右边轮测速   
*	形    参: 无
*	返 回 值: 无   
*	说    明：当全局变量time==Tim_time时调用一次该函数
			  该车辆右轮存在一定的问题，导致需要使用630减掉计数值才能得到转速
*********************************************************************************************************
*/
void calc_motor_Right_rotate_speed(void)
{
	// 读取编码器的值，正负代表旋转方向，溢出次数要加上去
	encoderNum_R = Get_Encoder_Cnt(3)+(65535*Encoder_Timer_Overflow_R);
	// 转速(1秒钟转多少圈)=单位时间内(100ms)的计数值/总分辨率*时间系数,总分辨率即车轮转过一圈的脉冲数，时间系数是1000ms/定时时长
	//rotateSpeed_R = (float)encoderNum_R/Total_Resolution*(1000/Tim_time);
	rotateSpeed_R = (float)encoderNum_R/Total_Resolution*(1000/Tim_time);
}
