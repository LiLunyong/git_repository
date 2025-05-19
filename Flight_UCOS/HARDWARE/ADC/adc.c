#include "adc.h"
#include "delay.h"		 
#include "includes.h"					//ucos ʹ��	 


uint16_t ADC1_DMA_Size = 6000; 		//��������
volatile uint16_t ADC_Value_Buffer[6000]; 	// DMAĿ�껺����



//��ʼ��ADC															   
void  Adc_Init(void)
{    
	GPIO_InitTypeDef  		GPIO_InitStructure;
	ADC_CommonInitTypeDef 	ADC_CommonInitStructure;
	ADC_InitTypeDef       	ADC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADC1ʱ��

	//�ȳ�ʼ��ADC1ͨ��5 IO��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//PA7 ͨ��7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��   

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1��λ
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//��λ����
	

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//���������׶�֮����ӳ�5��ʱ��
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1; //DMAʹ��
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz 
	ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//��ɨ��ģʽ	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;// ����ת��
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���	
	ADC_InitStructure.ADC_NbrOfConversion = 1;//1��ת���ڹ��������� Ҳ����ֻת����������1 
	ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��


	ADC_Cmd(ADC1, ENABLE);//����ADת����	
	
	ADC_DMA_Init();

}	



void ADC_DMA_Init(void)                    
{
	DMA_InitTypeDef       DMA_InitStructure;
	NVIC_InitTypeDef 	  NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2 , ENABLE );
	
    DMA_DeInit(DMA2_Stream0); // �ȸ�λ // ʹ��DMA2 Stream0��ADC1��Ӧ����
	/* DMA2 Stream0 channel0 configuration **************************************/
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ADC_Value_Buffer;       
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = ADC1_DMA_Size;                              //DMA ��������
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	
	DMA_Init( DMA2_Stream0 , &DMA_InitStructure );
	DMA_Cmd( DMA2_Stream0 , ENABLE );
	
	DMA_ClearITPendingBit( DMA2_Stream0, DMA_IT_TCIF0 );
	DMA_ITConfig( DMA2_Stream0 , DMA_IT_TC , ENABLE );		// ʹ�ܴ�������ж�
	
	
	/* Enable the DMA Stream IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);    
	
	
	ADC_DMARequestAfterLastTransferCmd( ADC1 , ENABLE );
	ADC_DMACmd(ADC1, ENABLE);	// ʹ�� ADC DMA ����
	
	//ADC1,ADCͨ��,480������,��߲���ʱ�������߾�ȷ�� �����Ѿ�����߲���ʱ���ˣ�
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_480Cycles );
	/* �����������������ģʽ�£��᲻��ת�������� DMA */
	ADC_SoftwareStartConv(ADC1);
	
}	


//DMA�жϷ�������ݲ��������ݴ����������һЩ�жϱ�־λ
void DMA2_Stream0_IRQHandler(void) {
	
	
	//�����ж�
	OSIntEnter();	  	//UCOS����ϵͳ���ж�ʱ�������**********************************************

	
    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0)) {
        /* handle half-transfer */
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);
    }
    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0)) {
        /* handle transfer-complete */
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
    } 
	
	
	//�˳��ж�
	OSIntExit();	  	//UCOS����ϵͳ���ж�ʱ�������**********************************************    	
}


////��ʼ��ADC															   
//void  Adc_Init(void)
//{    
//  GPIO_InitTypeDef  GPIO_InitStructure;
//	ADC_CommonInitTypeDef ADC_CommonInitStructure;
//	ADC_InitTypeDef       ADC_InitStructure;
//	
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOAʱ��
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADC1ʱ��

//  //�ȳ�ʼ��ADC1ͨ��5 IO��
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//PA5 ͨ��5
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
//  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��  
//	
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��  
// 
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1��λ
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//��λ����	 
// 
//	
//  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
//  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//���������׶�֮����ӳ�5��ʱ��
//  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMAʧ��
//  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz 
//  ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��
//	
//  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
//  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//��ɨ��ģʽ	
//  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//�ر�����ת��
//  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
//  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���	
//  ADC_InitStructure.ADC_NbrOfConversion = 1;//1��ת���ڹ��������� Ҳ����ֻת����������1 
//  ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��
//	
// 
//	ADC_Cmd(ADC1, ENABLE);//����ADת����	

//}				  
////���ADCֵ
////ch: @ref ADC_channels 
////ͨ��ֵ 0~16ȡֵ��ΧΪ��ADC_Channel_0~ADC_Channel_16
////����ֵ:ת�����
//u16 Get_Adc(u8 ch)   
//{
//	  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
//	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADCͨ��,480������,��߲���ʱ�������߾�ȷ��			    
//  
//	ADC_SoftwareStartConv(ADC1);		//ʹ��ָ����ADC1�����ת����������	
//	 
//	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

//	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
//}
////��ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ�� 
////ch:ͨ�����
////times:��ȡ����
////����ֵ:ͨ��ch��times��ת�����ƽ��ֵ
//u16 Get_Adc_Average(u8 ch,u8 times)
//{
//	u32 temp_val=0;
//	u8 t;
//	for(t=0;t<times;t++)
//	{
//		temp_val+=Get_Adc(ch);
//		delay_ms(5);
//	}
//	return temp_val/times;
//} 
//	 












