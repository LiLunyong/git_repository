#include "sys.h"
#include "ELRS.h"
#include "includes.h"					//ucos 使用	  


uint8_t	SIZEBUFF_ELRS=26;
uint8_t DataA_ELRS[26];
uint32_t ELRS_ch1, ELRS_ch2, ELRS_ch3, ELRS_ch4, ELRS_ch5, ELRS_ch6, ELRS_ch7, ELRS_ch8;
uint8_t  aircraft_take_off = 0;


void Initial_UART2(unsigned long baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // 使能GPIOA时钟   							AHB1!!!!!!!!!!!!!!!!!!!!
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // 使能USART2时钟RCC_APB1PeriphClockCmd   	APB1!!!!!!!!!!!!!!!!!!!!

    // 串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); // GPIOA2复用为USART2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // GPIOA3复用为USART2

    // USART1端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // GPIOA2与GPIOA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           // 复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      // 速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         // 推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           // 上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);                 // 初始化PA2，PA3

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
	
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);		// 开启 USART2 总线空闲中断
    USART_ClearFlag(USART2, USART_FLAG_TC);
    USART_Cmd(USART2, ENABLE);
	
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	DMA_UART2_RX_init();					// 顺便初始化DMA
}


void DMA_UART2_RX_init(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	DMA_DeInit(DMA1_Stream5);   //将DMA的通道1寄存器重设为缺省值
	while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE){}

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR; //外设基地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&DataA_ELRS; //存储器基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //外设作为来源
	DMA_InitStructure.DMA_BufferSize = SIZEBUFF_ELRS;    //缓冲区大小
	DMA_InitStructure.DMA_PeripheralInc =  DMA_PeripheralInc_Disable; //外设增量模式
	DMA_InitStructure.DMA_MemoryInc =  DMA_MemoryInc_Enable; //存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize =  DMA_PeripheralDataSize_Byte;  //外设传输大小
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器传输大小
	DMA_InitStructure.DMA_Mode =  DMA_Mode_Circular; //DMA模式
	DMA_InitStructure.DMA_Priority =  DMA_Priority_Medium;  //通道优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
	DMA_Init(DMA1_Stream5,&DMA_InitStructure);

	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(DMA1_Stream5,ENABLE);    //使用usart2进行数据接收
}


void USART2_IRQHandler(void)
{
	//进入中断
	OSIntEnter();  	  		//UCOS操作系统，中断时，必须的**********************************************  
	
	
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
		//软件序列清除IDLE标志位
		DMA_Cmd(DMA1_Stream5,DISABLE);
		
		//取遥控器的摇杆值
		if((DataA_ELRS[0]== 0xC8) && (DataA_ELRS[1]== 0x18) && (DataA_ELRS[2]== 0x16)
				 && (DataA_ELRS[20]== 0x00) && (DataA_ELRS[21]== 0x00))
		{
			//第3字节开始每11位为一个通道,一共16个通道
			ELRS_ch1 = ((DataA_ELRS[3]>>0) | (DataA_ELRS[4]<<8)) & 0x07FF;
			ELRS_ch2 = ((DataA_ELRS[4]>>3) | (DataA_ELRS[5]<<5)) & 0x07FF;
			ELRS_ch3 = ((DataA_ELRS[5]>>6) | (DataA_ELRS[6]<<2) | (DataA_ELRS[7]<<10)) & 0x07FF;
			ELRS_ch4 = ((DataA_ELRS[7]>>1) | (DataA_ELRS[8]<<7)) & 0x07FF;
			ELRS_ch5 = ((DataA_ELRS[8]>>4) | (DataA_ELRS[9]<<4)) & 0x07FF;
			ELRS_ch6 = ((DataA_ELRS[9]>>7) | (DataA_ELRS[10]<<1) | (DataA_ELRS[11]<<9)) & 0x07FF;
			ELRS_ch7 = ((DataA_ELRS[11]>>2) | (DataA_ELRS[12]<<6)) & 0x07FF;
			ELRS_ch8 = ((DataA_ELRS[12]>>5) | (DataA_ELRS[13]<<3)) & 0x07FF;
		}
		
		if (ELRS_ch6 > 900){
			aircraft_take_off = 1; }	// 遥控器的右上SB拨杆
		else{
			aircraft_take_off = 0;
		}
		
		//处理完重新开启DMA接收
		DMA_SetCurrDataCounter(DMA1_Stream5,SIZEBUFF_ELRS); //重设长度想当于重置接收数组下标为0
		DMA_Cmd(DMA1_Stream5,ENABLE);

        USART_ClearITPendingBit(USART2, USART_IT_IDLE);        // 清除空闲中断
    }

	
	//退出中断
	OSIntExit();			//UCOS操作系统，中断时，必须的**********************************************	    
}


		//第0字节为device_addr(设备地址)DataA[0]//0xC8
        //第1字节为frame_size(帧大小)DataA[1]//0x18
        //第2字节对应type(类型)DataA[2]//0x16
            
        //第3字节开始每11位为一个通道,一共16个通道
//        ch1 = ((DataA_ELRS[3]>>0) | (DataA_ELRS[4]<<8)) & 0x07FF;
//        ch2 = ((DataA_ELRS[4]>>3) | (DataA_ELRS[5]<<5)) & 0x07FF;
//        ch3 = ((DataA_ELRS[5]>>6) | (DataA_ELRS[6]<<2) | (DataA_ELRS[7]<<10)) & 0x07FF;
//        ch4 = ((DataA_ELRS[7]>>1) | (DataA_ELRS[8]<<7)) & 0x07FF;
//        ch5 = ((DataA_ELRS[8]>>4) | (DataA_ELRS[9]<<4)) & 0x07FF;
//        ch6 = ((DataA_ELRS[9]>>7) | (DataA_ELRS[10]<<1) | (DataA_ELRS[11]<<9)) & 0x07FF;
//        ch7 = ((DataA_ELRS[11]>>2) | (DataA_ELRS[12]<<6)) & 0x07FF;
//        ch8 = ((DataA_ELRS[12]>>5) | (DataA_ELRS[13]<<3)) & 0x07FF;
//        ch9 = ((DataA_ELRS[13]>>2) | (DataA_ELRS[14]<<6) | (DataA_ELRS[15]<<14)) & 0x07FF;
//        ch10 = ((DataA_ELRS[15]>>1) | (DataA_ELRS[16]<<7) | (DataA_ELRS[17]<<13)) & 0x07FF;
//        ch11 = ((DataA_ELRS[16]>>4) | (DataA_ELRS[17]<<4) | (DataA_ELRS[18]<<12)) & 0x07FF;
//        ch12 = ((DataA_ELRS[17]>>7) | (DataA_ELRS[18]<<1) | (DataA_ELRS[19]<<9) | (DataA_ELRS[20]<<17)) & 0x07FF;
//        ch13 = ((DataA_ELRS[19]>>2) | (DataA_ELRS[20]<<6) | (DataA_ELRS[21]<<14)) & 0x07FF;
//        ch14 = ((DataA_ELRS[20]>>5) | (DataA_ELRS[21]<<3) | (DataA_ELRS[22]<<11)) & 0x07FF;
//        ch15 = ((DataA_ELRS[21]>>8) | (DataA_ELRS[22]<<2) | (DataA_ELRS[23]<<10)) & 0x07FF;
//        ch16 = ((DataA_ELRS[23]>>1) | (DataA_ELRS[24]<<7) | (DataA_ELRS[25]<<15)) & 0x07FF; 

		//看 DataA_ELRS 具体每个字节的数据
//		if((DataA_ELRS[0]== 0xC8) && (DataA_ELRS[1]== 0x18) && (DataA_ELRS[2]== 0x16)
//			 && (DataA_ELRS[18]== 202) && (DataA_ELRS[19]== 7) && (DataA_ELRS[20]== 0x00) && (DataA_ELRS[21]== 0x00) && (DataA_ELRS[22]== 76))
//        {
//			printf("DataA[0]=%d\tDataA[1]=%d\tDataA[2]=%d\tDataA[15]=%d\tDataA[16]=%d\tDataA[17]=%d\tDataA[18]=%d\tDataA[19]=%d\tDataA[20]=%d\tDataA[21]=%d\tDataA[22]=%d\tDataA[23]=%d\tDataA[24]=%d\tDataA[25]=%d\t\r\n", 
//				DataA_ELRS[0], DataA_ELRS[1], DataA_ELRS[2], DataA_ELRS[15], DataA_ELRS[16], DataA_ELRS[17], DataA_ELRS[18], DataA_ELRS[19], DataA_ELRS[20], DataA_ELRS[21], DataA_ELRS[22], DataA_ELRS[23], DataA_ELRS[24], DataA_ELRS[25]);
//		}
	
		//看解析后 每个通道的值
//		if((DataA_ELRS[0]== 0xC8) && (DataA_ELRS[1]== 0x18) && (DataA_ELRS[2]== 0x16)
//			 && (DataA_ELRS[20]== 0x00) && (DataA_ELRS[21]== 0x00))
//        {
//			printf("DataA[0]=%d\tDataA[1]=%d\tDataA[2]=%d\tChanel1:%d\tChanel2:%d\tChanel3:%d\tChanel4:%d\tChanel5:%d\tChanel6:%d\tChanel7:%d\tChanel8:%d\t\r\n", 
//				DataA_ELRS[0], DataA_ELRS[1], DataA_ELRS[2], ELRS_ch1, ELRS_ch2, ELRS_ch3, ELRS_ch4, ELRS_ch5, ELRS_ch6, ELRS_ch7, ELRS_ch8);
//		}

		//看解析后 每个通道的值  值都是174到1811之间
//		printf("Chanel1:%d\tChanel2:%d\tChanel3:%d\tChanel4:%d\tChanel5:%d\tChanel6:%d\tChanel7:%d\tChanel8:%d\t\r\n", 
//				ELRS_ch1, ELRS_ch2, ELRS_ch3, ELRS_ch4, ELRS_ch5, ELRS_ch6, ELRS_ch7, ELRS_ch8);
        



