#include "string.h"
#include "led.h"
#include "delay.h"
#include "distance.h"
#include "includes.h"					//ucos 使用	  	//UCOS操作系统，中断时，必须的**********************************************



u16 distance = 0;		 //(单位 cm)




///////////////////////////////串口方式读取
//void Initial_UART6(u32 bound)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); 	// USART6 是 APB2 

//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6); 
//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6); 

//	// USART1端口配置
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			 // 复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 // 速度50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			 // 推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			 // 上拉
//	GPIO_Init(GPIOC, &GPIO_InitStructure);					 // 初始化

//	// USART1 初始化设置
//	USART_InitStructure.USART_BaudRate = bound;										// 波特率设置
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 一个停止位
//	USART_InitStructure.USART_Parity = USART_Parity_No;								// 无奇偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 收发模式
//	USART_Init(USART6, &USART_InitStructure);										// 初始化串口1

//	USART_Cmd(USART6, ENABLE); // 使能串口
//	USART_ClearFlag(USART6, USART_FLAG_TC);
//	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE); // 开启相关中断

//	// Usart6 NVIC 配置
//	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;		  // 串口1中断通道
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  // 子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);							  // 根据指定的参数初始化NVIC寄存器
//}


//// 发送一个字节数据
//// input:byte,待发送的数据
//void USART6_send_byte(uint8_t byte)
//{
//	while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET)
//		; // 等待发送完成
//	USART6->DR = byte;
//}
//// 发送多字节数据
//void USART6_Send_bytes(uint8_t *Buffer, uint8_t Length)
//{
//	uint8_t i = 0;
//	while (i < Length)
//	{
//		USART6_send_byte(Buffer[i++]);
//	}
//}
//// 发送多字节数据+校验和
//void USART6_Send(uint8_t *Buffer, uint8_t Length)
//{
//	uint8_t i = 0;
//	while (i < Length)
//	{
//		if (i < (Length - 1))
//			Buffer[Length - 1] += Buffer[i]; // 累加Length-1前的数据
//		USART6_send_byte(Buffer[i++]);
//	}
//}

//void USART6_IRQHandler(void)
//{
//	//进入中断
//	OSIntEnter();  	  		//UCOS操作系统，中断时，必须的**********************************************  
//	
//	static uint8_t i = 0, rebuf[20] = {0};
//	uint8_t iii = 0, sum = 0;

//	if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) // 判断接收标志
//	{
//		rebuf[i++] = USART_ReceiveData(USART6); // 读取串口数据，同时清接收标志
//		if (rebuf[0] != 0x5a)					// 帧头不对
//			i = 0;
//		if ((i == 2) && (rebuf[1] != 0x5a)) // 帧头不对
//			i = 0;

//		if (i > 3) // i等于4时，已经接收到数据量字节rebuf[3]
//		{
//			if (i != (rebuf[3] + 5)) 	// 判断是否接收一帧数据完毕
//				return;					// 总共7位，接收完i++，所以数组里只能取到[6]，但i是7
//			switch (rebuf[2]) // 接收完毕后处理
//			{
//				case 0x45:
//					for(sum=0,iii=0; iii < (rebuf[3] + 4); iii++) {sum += rebuf[iii];}
//					if(sum == rebuf[iii])//校验和判断 iii这时候是6
//					{	
//						distance = (rebuf[4]<<8)|rebuf[5]; // 拷贝接收到的数据  //memcpy(Sonar_data, rebuf, 8);
//					}
//					break;
//				case 0x15:
//					break; // 原始数据接收，可模仿0x45的方式
//				case 0x35:
//					break;
//			}
//			i = 0; // 缓存清0
//		}
//	}


//	//退出中断
//	OSIntExit();			//UCOS操作系统，中断时，必须的**********************************************	
//}


//// send_com(0x56, 0x01); // 查询输出距离数据,只会输出一次
//// send_com(0x56, 0x02); // 连续输出距离数据,约 10hz（默认）
//// send_com(0x58, 0xAE); // 波特率配置：9600（默认）
//// send_com(0x58, 0xAF); // 波特率配置：115200
//// send_com(0x5A, 0x01); // 保存输出配置、IIC 地址、波特率配置
//// send_com(0x5A, 0x02); // 恢复出厂设置，恢复 IIC 地址、波特率配置
//void send_com(u8 com1, u8 com2)
//{
//	u8 bytes[4] = {0};
//	bytes[0] = 0xa5;
//	bytes[1] = com1;
//	bytes[2] = com2; // 功能字节
//	bytes[3] = bytes[0] + bytes[1] + bytes[2];
//	USART6_send_byte(bytes[0]); // 发送帧头、功能字节、校验和
//	USART6_send_byte(bytes[1]);
//	USART6_send_byte(bytes[2]);
//	USART6_send_byte(bytes[3]);
//}




///////////////////////////////脉冲方式读取
/***************
		u32 distance_ddd = 0;
		SR04_init();

		distance_ddd = SR04_get_distance();
		printf("distance = %d mm\r\n", distance_ddd);
		if(distance_ddd>0)
		{
			if(distance_ddd>=20 && distance_ddd<=4000)
			{
				printf("distance = %d mm\r\n", distance_ddd);
			}
		}
*****************/

/****************
void GY_US42_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	//输出			
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;			
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	//输入			
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;			
	GPIO_Init(GPIOC, &GPIO_InitStructure);		
	
	PCout(6) = 0;
}


u32 GY_US42_get_distance(void)
{
	u32 t,d;
	
	PCout(6) = 1;
	delay_us(20);
	PCout(6) = 0;
	
	t = 0;
	while(PCin(7) == 0)//等待高电平出现
	{
		t++;
		delay_us(1);  
		if(t>10000)
			return (unsigned int)-1;
	}
	
	t = 0;
	while(PCin(7))//测量高电平时间
	{
		t++;
		delay_us(9);   //9us == 3mm
		if(t>10000)
			return (unsigned int)-2;
	}
	
	delay_ms(10); 
	
	t = t/2;
	d = 3*t;
	return d;
}
****************/


