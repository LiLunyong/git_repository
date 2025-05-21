#include "sys.h"
#include "usart.h"	
#include "includes.h"					//ucos 使用	  


//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  

#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}

 
//初始化IO 串口1 
//baud:波特率
void usart_init(u32 baud)
{
   //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = baud;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、




	/***************************** 无线数传MD0和MD1设置 *********************************/
	/************************** MD0 = 0; MD1 = 0 半双工模式 *****************************/
	/************************** MD0 = 1; MD1 = 0 全双工模式 *****************************/
	/************************** MD0 = 0; MD1 = 1 预留模式 暂与模式0 相同  ****************/
	/************************** MD0 = 1; MD1 = 1 设置模式 *******************************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      // 普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       // 上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);             // 初始化GPIO
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_11);
	GPIO_ResetBits(GPIOA, GPIO_Pin_12);
	
	
}

#include "includes.h"					//ucos 使用	  	//UCOS操作系统，中断时，必须的**********************************************

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
//	uint8_t Res=0;

	//进入中断
	OSIntEnter();  	  		//UCOS操作系统，中断时，必须的**********************************************  

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		//接收串口数据
//		Res = USART_ReceiveData(USART1);	
		
//		// 串口转发
//		UsartSendByte(USART3, Res);	
		
		//清空串口接收中断标志位
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	} 
	
	//退出中断
	OSIntExit();			//UCOS操作系统，中断时，必须的**********************************************	
} 





struct COMM_RX_USART1 stc_COMM_RX_USART1 = {0xAA, 0xBB, 0xCC, 0xDD}; // 初始化USART1接收指令

void RX_CopeData_u8_USART1(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;

	ucRxBuffer[ucRxCnt++] = ucData; // 将收到的数据存入缓冲区中
	if (ucRxBuffer[0] != 0x55)		// 数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt = 0;
		return;
	}
	if (ucRxCnt < 6)
	{
		return;
	} // 数据不满6个，则返回
	else
	{
		switch (ucRxBuffer[1]) // 判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
		case 0x50:
			memcpy(&stc_COMM_RX_USART1, &ucRxBuffer[2], 4);
			break; // memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
		}
		ucRxCnt = 0; // 清空缓存区
	}
}

/*===================发送一个字符  ANO_TC.c中要用============*/
void UsartSendByte(USART_TypeDef *USARTx, uint8_t Data)
{
	USART_SendData(USARTx, Data);

	while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
		;
}


