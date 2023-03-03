#include "Tim_pwm.h"
#include "Tim_encoder_speed.h"

/*
*********************************************************************************************************
*	�� �� ��: Tim_gpio_config                    
*	����˵��: ����Tim��GPIO	        
*	��    ��: ��
*	�� �� ֵ: ��        
*	˵    ��������PA8��PA11���PWM�źţ��ֱ��ӦTIM1_CH1��TIM1_CH4
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
*	�� �� ��: PWM_Start                    
*	����˵��: ���ö�ʱ��ΪPWMģʽ     
*	��    ��: timer 		��ʱ��
			  tim_channel	ͨ��
			  freq			PWMƵ�ʣ�KHz��
			  duty			PWMռ�ձ�   
*	�� �� ֵ: ��        
*	˵    ����(arr + 1)*(psc + 1)/72M = Tout ; freq��Tout
			  CCR / (arr + 1) = duty ; CCR��TIM_Pulse
*********************************************************************************************************
*/
void PWM_Start(TIM_TypeDef* timer,uint8_t tim_channel,uint8_t freq,uint8_t duty)
{
	uint16_t arr = 36000/freq-1; // �����Զ���װ�ؼĴ�����ֵ����������PWM����
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_TimeBaseStructure.TIM_Prescaler = 1; 					// ���÷�Ƶ���ӣ�2��Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // ���ϼ���
	TIM_TimeBaseStructure.TIM_Period = arr; 					// �Զ���װ�ؼĴ�����ֵ������PWM����������
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseInit(timer,&TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;				// TIM��PWM1ģʽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	// ʹ������Ƚ�
	TIM_OCInitStructure.TIM_Pulse = (arr+1)*duty/100;				// ����ռ�ձ�
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		// TIM����Ƚϼ���Ϊ��
	
	switch(tim_channel)
	{
		case 1:
			TIM_OC1Init(timer,&TIM_OCInitStructure);			//TIMx�ĵ�1��ͨ��
			TIM_OC1PreloadConfig(timer,TIM_OCPreload_Enable); 	//ʹ�ܶ�ʱ��TIMx��CCR1�ϵ�Ԥװ�ؼĴ���
			break;
		case 2:
			TIM_OC2Init(timer,&TIM_OCInitStructure);			//TIMx�ĵ�2��ͨ��
			TIM_OC2PreloadConfig(timer,TIM_OCPreload_Enable);	//ʹ�ܶ�ʱ��TIMx��CCR2�ϵ�Ԥװ�ؼĴ���
			break;
		case 3:
			TIM_OC3Init(timer,&TIM_OCInitStructure);			//TIMx�ĵ�3��ͨ��
			TIM_OC3PreloadConfig(timer,TIM_OCPreload_Enable);	//ʹ�ܶ�ʱ��TIMx��CCR3�ϵ�Ԥװ�ؼĴ���
			break;
		case 4:
			TIM_OC4Init(timer,&TIM_OCInitStructure);			//TIMx�ĵ�4��ͨ��
			TIM_OC4PreloadConfig(timer,TIM_OCPreload_Enable);	//ʹ�ܶ�ʱ��TIMx��CCR4�ϵ�Ԥװ�ؼĴ���
			break;
	}
//	TIM_CtrlPWMOutputs(timer,ENABLE);	//����TIM1��PWM���Ϊʹ��
	TIM_ARRPreloadConfig(timer,ENABLE); //ʹ�ܶ�ʱ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
	TIM_Cmd(timer,ENABLE);				//ʹ�ܶ�ʱ��TIMx
}
/*
*********************************************************************************************************
*	�� �� ��: tim2_init                    
*	����˵��: ���ö�ʱ��2��ʹ����100msΪ���ڽ����ж�	        
*	��    ��: ��
*	�� �� ֵ: ��        
*********************************************************************************************************
*/
void tim1_init(u16 psc,u16 arr)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 

	TIM_TimeBaseStructure.TIM_Period = arr;						// �Զ���װ��ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					// Ԥ��Ƶϵ��
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 			//����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
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
*	�� �� ��: TIM2_IRQHandler                    
*	����˵��: ����Ϊ100ms�Ķ�ʱ��2�жϺ�������ȡ���ֱ�����ֵ������	        
*	��    ��: ��
*	�� �� ֵ: ��        
*********************************************************************************************************
*/
void TIM1_UP_IRQHandler(void)	
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET)  // �жϱ�־λ��1
	{
		calc_motor_Right_rotate_speed();
	}
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);		//����жϱ�־λ
}
