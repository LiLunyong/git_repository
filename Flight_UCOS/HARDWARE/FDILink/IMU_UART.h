#ifndef __IMU_UART_H
#define __IMU_UART_H

#include "string.h"
#include "sys.h"
#include "delay.h"
#include "IMU_UART.h"
#include "fdilink_decode.h"
#include "FDILink.h"



extern uint8_t DataA_IMU[256];
extern  FDILink_Status_t	_FDILink;



void Initial_UART3(unsigned long baudrate);
void DMA_UART3_RX_init(void);

int fdiComSetConfig(void);											//准备进入配置模式		
int fdiComGetParam(char* paramName);								//读取参数
int fdiComSetConfigPacketSentMsg(char* msg, int freq);				//配置发送的数据包内容及频率
int fdiComSetConfigMsg(void);
int fdiSetSave(void);												//保存配置
int fdiSetDeconfig(void);											//退出配置模式
int fdiSetReboot(void);												//重启设备

#endif


