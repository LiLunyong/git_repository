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

int fdiComSetConfig(void);											//׼����������ģʽ		
int fdiComGetParam(char* paramName);								//��ȡ����
int fdiComSetConfigPacketSentMsg(char* msg, int freq);				//���÷��͵����ݰ����ݼ�Ƶ��
int fdiComSetConfigMsg(void);
int fdiSetSave(void);												//��������
int fdiSetDeconfig(void);											//�˳�����ģʽ
int fdiSetReboot(void);												//�����豸

#endif


