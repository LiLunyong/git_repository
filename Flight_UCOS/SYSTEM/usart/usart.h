#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 


struct COMM_RX_USART1 // 串口1接收的四个字节数据指令
{
    unsigned char COMM1;
    unsigned char COMM2;
    unsigned char COMM3;
    unsigned char COMM4;
};



extern void usart_init(uint32_t baud);


void UsartSendByte(USART_TypeDef *USARTx, uint8_t Data);
void RX_CopeData_u8_USART1(unsigned char ucData);


#endif


