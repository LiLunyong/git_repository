#include "sys.h"
#include "usart.h"
#include "led.h"
#include "GNSS.h"	
#include "NMEA0183.h"
#include "includes.h"					//ucos 使用	  



uint32_t SIZEBUFF_GNSS=2000;	//容量不够大的话，接收的数据会不全
uint8_t DataA_GNSS[2000];  		//每接收一次完整的数据，就进一次空闲中断
NMEA0183 gnss_nmea0183;


void Initial_UART4(unsigned long baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 使能GPIOA时钟   						AHB1!!!!!!!!!!!!!!!!!!!!
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); // 使能UART4时钟RCC_APB1PeriphClockCmd   	APB1!!!!!!!!!!!!!!!!!!!!

    // 串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4); // GPIOA0复用为UART4
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4); // GPIOA1复用为UART4

    // UART4端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; // GPIOA0与GPIOA1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           // 复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      // 速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         // 推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           // 上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);                 // 初始化

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &USART_InitStructure);
	
    USART_ITConfig(UART4, USART_IT_TXE, DISABLE);
    USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);		// 开启 UART4 总线空闲中断
    USART_ClearFlag(UART4, USART_FLAG_TC);
    USART_Cmd(UART4, ENABLE);
	
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	DMA_UART4_RX_init();			// 顺便初始化DMA
	GPS_SEL_Init();					// 顺便初始化单个GPIO（选择数据的传输方向）
}


void DMA_UART4_RX_init(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	DMA_DeInit(DMA1_Stream2);   //将DMA的通道1寄存器重设为缺省值
	while (DMA_GetCmdStatus(DMA1_Stream2) != DISABLE){}

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR; //外设基地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&DataA_GNSS; //存储器基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //外设作为来源
	DMA_InitStructure.DMA_BufferSize = SIZEBUFF_GNSS;    //缓冲区大小
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
	DMA_Init(DMA1_Stream2,&DMA_InitStructure);

	USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(DMA1_Stream2,ENABLE);    //使用usart4进行数据接收
}


// GPS_SEL IO初始化
void GPS_SEL_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // 使能GPIOC时钟

    // GPIO初始化设置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;         // LED_PC0
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      // 普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       // 上拉  ////////////////////这个问题，必须上拉才能接通
    GPIO_Init(GPIOC, &GPIO_InitStructure);             // 初始化GPIO

    GPIO_SetBits(GPIOC, GPIO_Pin_0); // 设置高，则IMU和GPS不连接，GPS传到主控来处理
}



void UART4_IRQHandler(void)
{

	//进入中断
	OSIntEnter();  	  		//UCOS操作系统，中断时，必须的**********************************************      

	
    if (USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
    {
		uint16_t length;
		int i=0;
		
		volatile uint32_t temp;		///////////////////////////////// 必须读取，否则没法再次进中断，就崩溃了
		temp = UART4->SR;			/////////////////////////////////
		temp = UART4->DR;			/////////////////////////////////
		(void)temp;  // 防止未使用警告   //

		//软件序列清除IDLE标志位
		DMA_Cmd(DMA1_Stream2,DISABLE);
		
		
        // 获取接收数据长度
        length = SIZEBUFF_GNSS - DMA_GetCurrDataCounter(DMA1_Stream2); 
		// 数据处理
		for(i=0; i<length; ++i)
		{
			if(nmea_decode(&gnss_nmea0183, DataA_GNSS[i]))
			{
			  ///解析代码成功  //每接收一次完整的数据，就进一次空闲中断
			}
		}
			
//		//if (DataA_GNSS[0] == 0x24 && DataA_GNSS[1] == 0x47){ } //判断是不是$G开头
//		LED_PC13 = !LED_PC13;
//		for(i=0; i<length; i++){
//			UsartSendByte(USART1, DataA_GNSS[i]);	//传到上位机看看
//		}
		
		
		
		//处理完重新开启DMA接收
		DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_FEIF1);
		DMA_SetCurrDataCounter(DMA1_Stream2, SIZEBUFF_GNSS); //重设长度想当于重置接收数组下标为0
		DMA_Cmd(DMA1_Stream2,ENABLE);
		
        USART_ClearITPendingBit(UART4, USART_IT_IDLE);        // 清除空闲中断

    }

	
	//退出中断
	OSIntExit();  	  		//UCOS操作系统，中断时，必须的**********************************************      
}






