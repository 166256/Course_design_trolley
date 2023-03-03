#include "Tim_pwm.h"
#include "Tim_encoder_speed.h"

/*
*********************************************************************************************************
*	函 数 名: Tim_gpio_config                    
*	功能说明: 配置Tim的GPIO	        
*	形    参: 无
*	返 回 值: 无        
*	说    明：采用PA8和PA11输出PWM信号，分别对应TIM1_CH1和TIM1_CH4
*********************************************************************************************************
*/
void tim4_gpio_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOB,&GPIO_InitStructure);

}
/*
*********************************************************************************************************
*	函 数 名: PWM_Start                    
*	功能说明: 配置定时器为PWM模式     
*	形    参: timer 		定时器
			  tim_channel	通道
			  freq			PWM频率（KHz）
			  duty			PWM占空比   
*	返 回 值: 无        
*	说    明：(arr + 1)*(psc + 1)/72M = Tout ; freq即Tout
			  CCR / (arr + 1) = duty ; CCR即TIM_Pulse
*********************************************************************************************************
*/
void PWM_Start(TIM_TypeDef* timer,uint8_t tim_channel,uint8_t freq,uint8_t duty)
{
	uint16_t arr = 36000/freq-1; // 设置自动重装载寄存器的值，用于设置PWM周期
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_TimeBaseStructure.TIM_Prescaler = 1; 					// 设置分频因子，2分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
	TIM_TimeBaseStructure.TIM_Period = arr; 					// 自动重装载寄存器的值，即是PWM方波的周期
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseInit(timer,&TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;				// TIM的PWM1模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	// 使能输出比较
	TIM_OCInitStructure.TIM_Pulse = (arr+1)*duty/100;				// 设置占空比
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		// TIM输出比较极性为高
	
	switch(tim_channel)
	{
		case 1:
			TIM_OC1Init(timer,&TIM_OCInitStructure);			//TIMx的第1个通道
			TIM_OC1PreloadConfig(timer,TIM_OCPreload_Enable); 	//使能定时器TIMx在CCR1上的预装载寄存器
			break;
		case 2:
			TIM_OC2Init(timer,&TIM_OCInitStructure);			//TIMx的第2个通道
			TIM_OC2PreloadConfig(timer,TIM_OCPreload_Enable);	//使能定时器TIMx在CCR2上的预装载寄存器
			break;
		case 3:
			TIM_OC3Init(timer,&TIM_OCInitStructure);			//TIMx的第3个通道
			TIM_OC3PreloadConfig(timer,TIM_OCPreload_Enable);	//使能定时器TIMx在CCR3上的预装载寄存器
			break;
		case 4:
			TIM_OC4Init(timer,&TIM_OCInitStructure);			//TIMx的第4个通道
			TIM_OC4PreloadConfig(timer,TIM_OCPreload_Enable);	//使能定时器TIMx在CCR4上的预装载寄存器
			break;
	}
//	TIM_CtrlPWMOutputs(timer,ENABLE);	//设置TIM1的PWM输出为使能
	TIM_ARRPreloadConfig(timer,ENABLE); //使能定时器TIMx在ARR上的预装载寄存器
	TIM_Cmd(timer,ENABLE);				//使能定时器TIMx
}
/*
*********************************************************************************************************
*	函 数 名: tim2_init                    
*	功能说明: 配置定时器2，使其以100ms为周期进入中断	        
*	形    参: 无
*	返 回 值: 无        
*********************************************************************************************************
*/
void tim1_init(u16 psc,u16 arr)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 

	TIM_TimeBaseStructure.TIM_Period = arr;						// 自动重装载值	
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					// 预分频系数
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 			//根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE ); 

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);  

	TIM_Cmd(TIM1, ENABLE);  				 
}
/*
*********************************************************************************************************
*	函 数 名: TIM2_IRQHandler                    
*	功能说明: 周期为100ms的定时器2中断函数，读取右轮编码器值并测速	        
*	形    参: 无
*	返 回 值: 无        
*********************************************************************************************************
*/
void TIM1_UP_IRQHandler(void)	
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET)  // 中断标志位置1
	{
		calc_motor_Right_rotate_speed();
	}
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);		//清除中断标志位
}
