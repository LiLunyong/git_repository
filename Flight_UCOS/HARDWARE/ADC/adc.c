#include "adc.h"
#include "delay.h"		 
#include "includes.h"					//ucos 使用	 


uint16_t ADC1_DMA_Size = 6000; 		//采样点数
volatile uint16_t ADC_Value_Buffer[6000]; 	// DMA目标缓冲区



//初始化ADC															   
void  Adc_Init(void)
{    
	GPIO_InitTypeDef  		GPIO_InitStructure;
	ADC_CommonInitTypeDef 	ADC_CommonInitStructure;
	ADC_InitTypeDef       	ADC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟

	//先初始化ADC1通道5 IO口
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//PA7 通道7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化   

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束
	

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1; //DMA使能
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
	ADC_CommonInit(&ADC_CommonInitStructure);//初始化

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;// 连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
	ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
	ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化


	ADC_Cmd(ADC1, ENABLE);//开启AD转换器	
	
	ADC_DMA_Init();

}	



void ADC_DMA_Init(void)                    
{
	DMA_InitTypeDef       DMA_InitStructure;
	NVIC_InitTypeDef 	  NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2 , ENABLE );
	
    DMA_DeInit(DMA2_Stream0); // 先复位 // 使用DMA2 Stream0（ADC1对应流）
	/* DMA2 Stream0 channel0 configuration **************************************/
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ADC_Value_Buffer;       
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = ADC1_DMA_Size;                              //DMA 传输数量
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
	DMA_ITConfig( DMA2_Stream0 , DMA_IT_TC , ENABLE );		// 使能传输完成中断
	
	
	/* Enable the DMA Stream IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);    
	
	
	ADC_DMARequestAfterLastTransferCmd( ADC1 , ENABLE );
	ADC_DMACmd(ADC1, ENABLE);	// 使能 ADC DMA 请求
	
	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度 （这已经是最高采样时间了）
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_480Cycles );
	/* 软件触发启动：连续模式下，会不断转换并触发 DMA */
	ADC_SoftwareStartConv(ADC1);
	
}	


//DMA中断服务程序，暂不进行数据处理，就是清除一些中断标志位
void DMA2_Stream0_IRQHandler(void) {
	
	
	//进入中断
	OSIntEnter();	  	//UCOS操作系统，中断时，必须的**********************************************

	
    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0)) {
        /* handle half-transfer */
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);
    }
    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0)) {
        /* handle transfer-complete */
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
    } 
	
	
	//退出中断
	OSIntExit();	  	//UCOS操作系统，中断时，必须的**********************************************    	
}


////初始化ADC															   
//void  Adc_Init(void)
//{    
//  GPIO_InitTypeDef  GPIO_InitStructure;
//	ADC_CommonInitTypeDef ADC_CommonInitStructure;
//	ADC_InitTypeDef       ADC_InitStructure;
//	
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOA时钟
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟

//  //先初始化ADC1通道5 IO口
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//PA5 通道5
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
//  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化  
//	
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化  
// 
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	 
// 
//	
//  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
//  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
//  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
//  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
//  ADC_CommonInit(&ADC_CommonInitStructure);//初始化
//	
//  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
//  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
//  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
//  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
//  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
//  ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
//  ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
//	
// 
//	ADC_Cmd(ADC1, ENABLE);//开启AD转换器	

//}				  
////获得ADC值
////ch: @ref ADC_channels 
////通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
////返回值:转换结果
//u16 Get_Adc(u8 ch)   
//{
//	  	//设置指定ADC的规则组通道，一个序列，采样时间
//	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
//  
//	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
//	 
//	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

//	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
//}
////获取通道ch的转换值，取times次,然后平均 
////ch:通道编号
////times:获取次数
////返回值:通道ch的times次转换结果平均值
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












