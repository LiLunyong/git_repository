#ifndef __ADC_H
#define __ADC_H	
#include "sys.h" 



extern uint16_t ADC1_DMA_Size; 		//采样点数
extern volatile uint16_t ADC_Value_Buffer[6000]; 	// DMA目标缓冲区


void Adc_Init(void); 				//ADC通道初始化
void ADC_DMA_Init(void);

						   
//void Adc_Init(void); 				//ADC通道初始化
//u16  Get_Adc(u8 ch); 				//获得某个通道值 
//u16 Get_Adc_Average(u8 ch,u8 times);//得到某个通道给定次数采样的平均值  


#endif 






