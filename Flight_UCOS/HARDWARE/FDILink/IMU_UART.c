#include "sys.h"
#include "IMU_UART.h"
#include "usart.h"
#include "led.h"
#include <stdlib.h>
#include "includes.h"					//ucos 使用	  


static FDILink_Status_t	_FDILink;

uint32_t SIZEBUFF_IMU=256;
uint8_t DataA_IMU[256];





void Initial_UART3(unsigned long baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  // 使能GPIOA时钟   							AHB1!!!!!!!!!!!!!!!!!!!!
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // 使能USART2时钟RCC_APB1PeriphClockCmd   	APB1!!!!!!!!!!!!!!!!!!!!

    // 串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); // GPIOB10复用为USART3
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); // GPIOB11复用为USART3

    // USART1端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // GPIOB10与GPIOB11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           // 复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      // 速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         // 推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           // 上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);                 // 初始化 B10 B11

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
	
    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);		// USART3 发送中断
    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);		// 开启 USART3 总线空闲中断
    USART_ClearFlag(USART3, USART_FLAG_TC);
    USART_Cmd(USART3, ENABLE);
	
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	DMA_UART3_RX_init();					// 顺便初始化DMA
}


void DMA_UART3_RX_init(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	DMA_DeInit(DMA1_Stream1);   //将DMA的通道1寄存器重设为缺省值
	while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE){}

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR; //外设基地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&DataA_IMU; //存储器基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //外设作为来源
	DMA_InitStructure.DMA_BufferSize = SIZEBUFF_IMU;    //缓冲区大小
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
	DMA_Init(DMA1_Stream1,&DMA_InitStructure);

	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(DMA1_Stream1,ENABLE);    //使用usart3进行数据接收
}


void USART3_IRQHandler(void)
{

	//进入中断
	OSIntEnter();  	  		//UCOS操作系统，中断时，必须的**********************************************      

	
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
		uint16_t length;
//		int i=0;
		
		volatile uint32_t temp;		///////////////////////////////// 必须读取，否则没法再次进中断，就崩溃了
		temp = USART3->SR;			/////////////////////////////////
		temp = USART3->DR;			/////////////////////////////////
		(void)temp;  // 防止未使用警告   //

		//软件序列清除IDLE标志位
		DMA_Cmd(DMA1_Stream1,DISABLE);
		
		
        // 获取接收数据长度
        length = SIZEBUFF_IMU - DMA_GetCurrDataCounter(DMA1_Stream1); 
//		UsartSendByte(USART1, (uint8_t)length);
//		if((DataA_IMU[0]== 0xFC) && (DataA_IMU[1]== 0x58) && (DataA_IMU[length - 1]== 0xFD) && (length == 56))
//		{
//			LED_PC13 = !LED_PC13;
//			fdiComProtocolReceive(&_FDILink, DataA_IMU, length);
////			//如果每次中断全部打印，可能是因为波特率问题，慢，导致只显示0x58的帧，0x59的帧不显示
////			for(i=0; i<length; i++){
////				UsartSendByte(USART1, DataA_IMU[i]);	//传到上位机看看
////			}
//		}
		if((DataA_IMU[0]== 0xFC) && (DataA_IMU[length - 1]== 0xFD) && (DataA_IMU[1]== 0x40) && (length == 64))
		{
//			LED_PC13 = !LED_PC13;
			fdiComProtocolReceive(&_FDILink, DataA_IMU, length);
		}
		
		if((DataA_IMU[0]== 0xFC) && (DataA_IMU[length - 1]== 0xFD) && (DataA_IMU[1]== 0x63) && (length == 20))
		{
			fdiComProtocolReceive(&_FDILink, DataA_IMU, length);
		}
		
		
		//处理完重新开启DMA接收
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_FEIF1);
		DMA_SetCurrDataCounter(DMA1_Stream1, SIZEBUFF_IMU); //重设长度想当于重置接收数组下标为0
		DMA_Cmd(DMA1_Stream1,ENABLE);
		
        USART_ClearITPendingBit(USART3, USART_IT_IDLE);        // 清除空闲中断

	
    }
	
//	// 串口转发
//	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
//    {
//		u8 Res;
//		Res = USART_ReceiveData(USART3);	// 如果需要执行转发，那就必须先屏蔽 DMA_UART3_RX_init();
//		UsartSendByte(USART1, Res);
//		
//        USART_ClearITPendingBit(USART3, USART_IT_RXNE);        // 清除接收中断
//    }
	
	
	//退出中断
	OSIntExit();  	  		//UCOS操作系统，中断时，必须的**********************************************    

}





/***********************************配置IMU所需的函数**************************************/
/* 进入配置模式 */
int fdiComSetConfig(void)
{
	int i=0, j=0;
	char send_buff[128];
	sprintf(send_buff, "#fconfig\r\n");
	
	for(j=0; j<4; j++){
		for(i=0; i<strlen(send_buff); i++){
			UsartSendByte(USART3, send_buff[i]);
		}
		delay_ms(1000);	//LED_PC13 = !LED_PC13;
	}

	return FDI_NO_ERROR;
}


/* 读取参数，paramName为需要获取的参数名称
 *	\param[in]	paramName - Parameter name to be obtained.*/
int fdiComGetParam(char* paramName)
{
	int i=0;
	char send_buff[128];
	sprintf(send_buff, "#fparam get %s\r\n", paramName);
	
	for(i=0; i<strlen(send_buff); i++){
		UsartSendByte(USART3, send_buff[i]);	//发送到指定窗口
	}
	delay_ms(1000);	//LED_PC13 = !LED_PC13;
	delay_ms(1000);

	return FDI_NO_ERROR;
}


/*  配置发送的数据内容，msg为2位16进制数字表示数据包ID，freq为设置指定数据包的发送频率
 *	Configure the data content to be sent.
 *	\param[in]	msg - Represents the packet ID; freq - Packet transmit frequency. */
int fdiComSetConfigPacketSentMsg(char* msg, int freq)
{	
	int i=0;
	char send_buff[128];
	sprintf(send_buff, "#fmsg %s %d\r\n", msg, freq);
	
	for(i=0; i<strlen(send_buff); i++){
		UsartSendByte(USART3, send_buff[i]);	//发送到指定窗口
	}
	delay_ms(1000);	//LED_PC13 = !LED_PC13;
	delay_ms(1000);

	return FDI_NO_ERROR;
}

int fdiComSetConfigMsg(void)
{	
	int i=0;
	char send_buff[128];
	sprintf(send_buff, "#fmsg\r\n");
	
	for(i=0; i<strlen(send_buff); i++){
		UsartSendByte(USART3, send_buff[i]);	//发送到指定窗口
	}
	delay_ms(1000);	//LED_PC13 = !LED_PC13;
	delay_ms(1000);

	return FDI_NO_ERROR;
}



/* 保存已修改的配置（重要的数据更新需要保存）*/
int fdiSetSave(void)
{
	int i=0;
	char send_buff[128];
	sprintf(send_buff, "#fsave\r\n");
	
	for(i=0; i<strlen(send_buff); i++){
		UsartSendByte(USART3, send_buff[i]);	//发送到指定窗口
	}
	delay_ms(1000);	//LED_PC13 = !LED_PC13;
	delay_ms(1000);

	return FDI_NO_ERROR;
}



/* 退出配置模式 */
int fdiSetDeconfig(void)
{
	int i=0;
	char send_buff[128];
	sprintf(send_buff, "#fdeconfig\r\n");
	
	for(i=0; i<strlen(send_buff); i++){
		UsartSendByte(USART3, send_buff[i]);	//发送到指定窗口
	}
	delay_ms(1000);	//LED_PC13 = !LED_PC13;
	delay_ms(1000);

	return FDI_NO_ERROR;
}



/* 重启设备（重要数据的更新需要重启设备）
 *	Restart the device (the device needs to be restarted for the update of important data).*/
int fdiSetReboot(void)
{
	int i=0;
	char send_buff[128];
	
	sprintf(send_buff, "#freboot\r\n");
		
	for(i=0; i<strlen(send_buff); i++){
		UsartSendByte(USART3, send_buff[i]);	//发送到指定窗口
	}
	delay_ms(1000);	//LED_PC13 = !LED_PC13;
	delay_ms(1000);

	
	sprintf(send_buff, "y\r\n");
		
	for(i=0; i<strlen(send_buff); i++){
		UsartSendByte(USART3, send_buff[i]);	//发送到指定窗口
	}
	delay_ms(1000);	//LED_PC13 = !LED_PC13;


	return FDI_NO_ERROR;
	
}




//uint8_t IMU_buf1[100];
//uint8_t IMU_buf2[100];
//		int len1 = 56;
//		int len2 = 64;
		
//		if(length > 40)		// 大于指定长度才处理，不然都有问题
//		{
//			// 数据处理（用户自定义协议）
//			if((DataA_IMU[0]== 0xFC) && (DataA_IMU[length - 1]== 0xFD))
//			{
//				fdiComProtocolReceive(&_FDILink, DataA_IMU, length);
//				
//			}
//		}
		
		
//		if(length == 56)		// 大于指定长度才处理，不然都有问题
//		{
//			// 数据处理（用户自定义协议）
//			if((DataA_IMU[0]== 0xFC) && (DataA_IMU[length - 1]== 0xFD))
//			{
//				fdiComProtocolReceive(&_FDILink, DataA_IMU, length);
//				
//			}
//		}
			
//		if(length == 120)		// 等于指定长度才处理，不然都有问题
//		{
//			if((DataA_IMU[0]== 0xFC) && (DataA_IMU[57]== 0x40) && (DataA_IMU[length - 1]== 0xFD))
//			{
//				LED_PC13 = !LED_PC13;
//	//			for(i=0; i<length; i++){
//	//				UsartSendByte(USART1, DataA_IMU[i]);	//传到上位机看看
//	//			}
//				
//				memset(IMU_buf1, 0, sizeof(IMU_buf1)); // 清空数组
//				memset(IMU_buf2, 0, sizeof(IMU_buf1));
//				memcpy(IMU_buf1, DataA_IMU, sizeof(int) * len1);	// 前半段一次 memcpy
//				memcpy(IMU_buf2, DataA_IMU + len1, sizeof(int) * len2);	// 后半段一次 memcpy
//				
//				if((IMU_buf1[0]== 0xFC) && (IMU_buf1[len1 - 1]== 0xFD))
//				{
//					fdiComProtocolReceive(&_FDILink, IMU_buf1, len1);
//					
//				}
//				
//				if((IMU_buf2[0]== 0xFC) && (IMU_buf2[len2 - 1]== 0xFD))
//				{
//					fdiComProtocolReceive(&_FDILink, IMU_buf2, len2);
//					
//				}
//				
//			}
//		}
		
		//如果每次中断全部打印，可能是因为波特率问题，慢，导致只显示0x58的帧，0x59的帧不显示
//		for(i=0; i<length; i++){
//			UsartSendByte(USART1, DataA_IMU[i]);	//传到上位机看看
//		}
		
//		if((length >= 70))
//		{
//			LED_PC13 = !LED_PC13;
//			for(i=0; i<length; i++){
//					UsartSendByte(USART1, DataA_IMU[i]);	//传到上位机看看
//				}
//			
//		}
