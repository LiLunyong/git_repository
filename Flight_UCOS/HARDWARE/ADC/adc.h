#ifndef __ADC_H
#define __ADC_H	
#include "sys.h" 



extern uint16_t ADC1_DMA_Size; 		//��������
extern volatile uint16_t ADC_Value_Buffer[6000]; 	// DMAĿ�껺����


void Adc_Init(void); 				//ADCͨ����ʼ��
void ADC_DMA_Init(void);

						   
//void Adc_Init(void); 				//ADCͨ����ʼ��
//u16  Get_Adc(u8 ch); 				//���ĳ��ͨ��ֵ 
//u16 Get_Adc_Average(u8 ch,u8 times);//�õ�ĳ��ͨ����������������ƽ��ֵ  


#endif 






