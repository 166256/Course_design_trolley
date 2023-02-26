//����һ�����ñ��������в��ٵ�.c�ļ�

#include	"Tim_encoder_speed.h"

int Encoder_Timer_Overflow_L = 0;		//ȫ�ֱ�������¼��������������
int	Encoder_Timer_Overflow_R = 0;		//ȫ�ֱ�������¼�ұ������������

int16_t encoderNum_L = 0;			
int16_t encoderNum_R = 0;
float rotateSpeed_L = 0;
float rotateSpeed_R = 0;


/*
*********************************************************************************************************
*	�� �� ��: Tim_E1B_E1A_Config                    
*	����˵��: �����ֱ���������Ӧ�Ķ�ʱ�����г�ʼ��     
*	��    ��: ��
*	�� �� ֵ: ��    
*	˵    �������ֱ�����A��-->PA15<-->TIM2_CH1,    B��-->PB3<-->TIM2_CH2
*********************************************************************************************************
*/
void Tim_E1B_E1A_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;	
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	// ��ӳ����Ҫ����AFIOʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	// c8t6��PB3��PB4��PA15����Ĭ������ΪJTAG���ܣ��ر�JTAG���ܣ�������Գ䵱��ͨGPIO��������ʹ��
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	// ������ӳ�亯��
	
	// ������ӳ��ʱ��Ĭ��TIM2�ĸ���IO����PA0��PA1��PA2��PA3 
	// ��ӳ���ʹ��PA15��PB3��PA2��PA3����ΪTIM2��IO��	
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2,ENABLE);	
	
	// PA15ΪE1B
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
	// PB3ΪE1A
	GPIO_InitStructure.GPIO_Pin = GPIO_E1A;//��������
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��GPIO

	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);		
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;    // ���ϼ���
	TIM_TimeBaseInitStructure.TIM_Period = Encoder_TIM_Period;         // ��װ��ֵ�����ɴ���65535 ��ΪF103�Ķ�ʱ����16λ��
	TIM_TimeBaseInitStructure.TIM_Prescaler =1-1;                      // ��Ƶϵ��1��������Ƶ
	TIM_TimeBaseInit(Tim_E1B_E1A,&TIM_TimeBaseInitStructure);

	// ����������
	TIM_EncoderInterfaceConfig(Tim_E1B_E1A, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); // ʹ�ñ�����ģʽ3,����4��Ƶ		
	TIM_ICStructInit(&TIM_ICInitStructure); 
	TIM_ICInitStructure.TIM_ICFilter = 0;				// ����ͨ�����˲�����
	TIM_ICInit(Tim_E1B_E1A, &TIM_ICInitStructure);		// ����ͨ����ʼ��		
	TIM_ClearFlag(Tim_E1B_E1A, TIM_FLAG_Update);		// ���TIM�ĸ��±�־λ�����±�־������CNT������arr������
	TIM_ITConfig(Tim_E1B_E1A, TIM_IT_Update, ENABLE);	// ʹ�ܶ�ʱ���ж�	
	TIM_SetCounter(Tim_E1B_E1A,0);						// �趨�������ĳ�ֵΪ0	
	TIM_Cmd(Tim_E1B_E1A,ENABLE);						// ʹ�ܿ��ƶ�ʱ������������ʼ����
}

/*
*********************************************************************************************************
*	�� �� ��: Tim_E2B_E2A_Config                    
*	����˵��: �����ֱ���������Ӧ�Ķ�ʱ�����г�ʼ��     
*	��    ��: ��
*	�� �� ֵ: ��    
*	˵    �������ֱ�����A��-->PA6<-->TIM3_CH1,B��-->PA7<-->TIM3_CH2
*********************************************************************************************************
*/
void Tim_E2B_E2A_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;	
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);

	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);		
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;    // ���ϼ���
	TIM_TimeBaseInitStructure.TIM_Period = Encoder_TIM_Period;         // ��װ��ֵ�����ɴ���65535 ��ΪF103�Ķ�ʱ����16λ��
	TIM_TimeBaseInitStructure.TIM_Prescaler =1-1;                      // ��Ƶϵ��1��������Ƶ
	TIM_TimeBaseInit(Tim_E2B_E2A,&TIM_TimeBaseInitStructure);

	// ����������
	TIM_EncoderInterfaceConfig(Tim_E2B_E2A, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //ʹ�ñ�����ģʽ3,����4��Ƶ		
	TIM_ICStructInit(&TIM_ICInitStructure); 
	TIM_ICInitStructure.TIM_ICFilter = 0;   			//����ͨ�����˲�����
	TIM_ICInit(Tim_E2B_E2A, &TIM_ICInitStructure);		//����ͨ����ʼ��		
	TIM_ClearFlag(Tim_E2B_E2A, TIM_FLAG_Update);		//���TIM�ĸ��±�־λ�����±�־������CNT������arr������
	TIM_ITConfig(Tim_E2B_E2A, TIM_IT_Update, ENABLE);	//ʹ�ܶ�ʱ���ж�	
	TIM_SetCounter(Tim_E2B_E2A,0);						//�趨�������ĳ�ֵΪ0	
	TIM_Cmd(Tim_E2B_E2A,ENABLE);						//ʹ�ܿ��ƶ�ʱ������������ʼ����
}

/*
*********************************************************************************************************
*	�� �� ��: Get_Encoder_Cnt                    
*	����˵��: ��ȡ�������ļ���ֵ    
*	��    ��: TIMx��ȡֵΪ2��3����ʾ��ʱ��2��3
*	�� �� ֵ: ���ض�ȡ����Ӧ��ʱ����CNTֵ    
*	˵    ��������ͨ����CNT��uint32���͵ļ���ֵ�� תΪint16���ͣ��Ϳ�����������������ת������
			  ������ʾ��ת��������ʾ��ת������󽫶�ʱ���ļ���ֵ����
*********************************************************************************************************
*/
vu16 Get_Encoder_Cnt(vu16 TIMx)
{
	vu16 Encoder_cnt = 0;

	switch(TIMx)
	{
		case 2://��������Ƕ�ʱ��2ʱ
			Encoder_cnt = TIM2->CNT;	//�ɼ��������ļ���ֵ�����档
			TIM_SetCounter(TIM2,0);			//����󽫶�ʱ���ļ���ֵ����
			break;
		case 3://��������Ƕ�ʱ��3ʱ
			Encoder_cnt = TIM3->CNT;	//�ɼ��������ļ���ֵ������
			TIM_SetCounter(TIM3,0);			//���꽫��ʱ���ļ���ֵ����
			break;
		default: 
			Encoder_cnt = 0;
			break;
	}
	return Encoder_cnt;
}

/*
*********************************************************************************************************
*	�� �� ��: TIM2_IRQHandler                    
*	����˵��: ��һ����ʱ�ж����ȡ���ֱ��������񵽵�������    
*	��    ��: ��
*	�� �� ֵ: ��   
*********************************************************************************************************
*/
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)	//���TIM2��CNT����������arr�������������ж�
	{
		Encoder_Timer_Overflow_L++;	//�������
		TIM_SetCounter(TIM2,0);			//���������ʱ����CNT��0
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);		//����жϱ�־λ
}

/*
*********************************************************************************************************
*	�� �� ��: TIM3_IRQHandler                    
*	����˵��: ��һ����ʱ�ж����ȡ���ֱ��������񵽵�������    
*	��    ��: ��
*	�� �� ֵ: ��   
*********************************************************************************************************
*/
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET)	//���TIM2��CNT����������arr�������������ж�
	{
		Encoder_Timer_Overflow_R++;	//�������
		TIM_SetCounter(TIM3,0);			//���������ʱ����CNT��0
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);		//����жϱ�־λ
}

/*
*********************************************************************************************************
*	�� �� ��: calc_motor_Left_rotate_speed                    
*	����˵��: ���ñ������������ֲ���   
*	��    ��: ��
*	�� �� ֵ: ��   
*	˵    ������ȫ�ֱ���time==Tim_timeʱ����һ�θú���
*********************************************************************************************************
*/
void calc_motor_Left_rotate_speed(void)
{	
	/*��ȡ��������ֵ������������ת�����������Ҫ����ȥ*/
	encoderNum_L = Get_Encoder_Cnt(2)+(65535*Encoder_Timer_Overflow_L);
	/* ת��(1����ת����Ȧ)=��λʱ����(100ms)�ļ���ֵ/�ֱܷ���*ʱ��ϵ��,�ֱܷ��ʼ�����ת��һȦ����������ʱ��ϵ����1000ms/��ʱʱ�� */
	rotateSpeed_L = (float)encoderNum_L/Total_Resolution*(1000/Tim_time);
}

/*
*********************************************************************************************************
*	�� �� ��: calc_motor_Right_rotate_speed                    
*	����˵��: ���ñ����������ұ��ֲ���   
*	��    ��: ��
*	�� �� ֵ: ��   
*	˵    ������ȫ�ֱ���time==Tim_timeʱ����һ�θú���
			  �ó������ִ���һ�������⣬������Ҫʹ��630��������ֵ���ܵõ�ת��
*********************************************************************************************************
*/
void calc_motor_Right_rotate_speed(void)
{
	// ��ȡ��������ֵ������������ת�����������Ҫ����ȥ
	encoderNum_R = Get_Encoder_Cnt(3)+(65535*Encoder_Timer_Overflow_R);
	// ת��(1����ת����Ȧ)=��λʱ����(100ms)�ļ���ֵ/�ֱܷ���*ʱ��ϵ��,�ֱܷ��ʼ�����ת��һȦ����������ʱ��ϵ����1000ms/��ʱʱ��
	//rotateSpeed_R = (float)encoderNum_R/Total_Resolution*(1000/Tim_time);
	rotateSpeed_R = (float)encoderNum_R/Total_Resolution*(1000/Tim_time);
}
