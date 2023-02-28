#ifndef __ADC_H
#define __ADC_H 			   
#include "sys.h"


#define ADC1_DR_Address    ((uint32_t)0x4001244C) //ADC1这个外设的地址
/*ADC接口*/
#define ADCPORT_1		GPIOB	//定义ADC接口
#define ADC_CH1		GPIO_Pin_0	//PA6
#define ADC_CH2		GPIO_Pin_7	//PA7

#define ADCPORT_2		GPIOA
#define ADC_CH3		GPIO_Pin_2	//PA2
#define ADC_CH4		GPIO_Pin_3	//PA3

//typedef struct
//{
//    vu16 filterValue;//滤波后的值
//    float KalmanGain;//Kalamn增益
//    float A;//状态矩阵
//    float H;//观测矩阵
//    float Q;//状态矩阵的方差
//    float R;//观测矩阵的方差
//    float P;//预测误差
//    float B;
//    float u;
//}KalmanStr;

void ADC_DMA_Init(void);
void ADC_GPIO_Init(void);
void ADC_Configuration(void);

//vu16 Kalman_Filter(KalmanStr *p,vu16 ADC_CvValue);
//void Kalman_Init(KalmanStr *p);
//vu16 First_Order_Filter(vu16 ADC_CvValue);



#endif





























