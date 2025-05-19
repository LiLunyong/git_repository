#include "sys.h"
#include "IMU_UART.h"
#include "usart.h"
#include "led.h"
#include <stdlib.h>
#include "includes.h"					//ucos ʹ��	  


static FDILink_Status_t	_FDILink;

uint32_t SIZEBUFF_IMU=256;
uint8_t DataA_IMU[256];





void Initial_UART3(unsigned long baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  // ʹ��GPIOAʱ��   							AHB1!!!!!!!!!!!!!!!!!!!!
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // ʹ��USART2ʱ��RCC_APB1PeriphClockCmd   	APB1!!!!!!!!!!!!!!!!!!!!

    // ����1��Ӧ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); // GPIOB10����ΪUSART3
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); // GPIOB11����ΪUSART3

    // USART1�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // GPIOB10��GPIOB11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           // ���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      // �ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         // ���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           // ����
    GPIO_Init(GPIOB, &GPIO_InitStructure);                 // ��ʼ�� B10 B11

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
	
    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);		// USART3 �����ж�
    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);		// ���� USART3 ���߿����ж�
    USART_ClearFlag(USART3, USART_FLAG_TC);
    USART_Cmd(USART3, ENABLE);
	
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	DMA_UART3_RX_init();					// ˳���ʼ��DMA
}


void DMA_UART3_RX_init(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	DMA_DeInit(DMA1_Stream1);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE){}

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR; //�������ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&DataA_IMU; //�洢������ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //������Ϊ��Դ
	DMA_InitStructure.DMA_BufferSize = SIZEBUFF_IMU;    //��������С
	DMA_InitStructure.DMA_PeripheralInc =  DMA_PeripheralInc_Disable; //��������ģʽ
	DMA_InitStructure.DMA_MemoryInc =  DMA_MemoryInc_Enable; //�洢������ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize =  DMA_PeripheralDataSize_Byte;  //���贫���С
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�������С
	DMA_InitStructure.DMA_Mode =  DMA_Mode_Circular; //DMAģʽ
	DMA_InitStructure.DMA_Priority =  DMA_Priority_Medium;  //ͨ�����ȼ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
	DMA_Init(DMA1_Stream1,&DMA_InitStructure);

	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(DMA1_Stream1,ENABLE);    //ʹ��usart3�������ݽ���
}


void USART3_IRQHandler(void)
{

	//�����ж�
	OSIntEnter();  	  		//UCOS����ϵͳ���ж�ʱ�������**********************************************      

	
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
		uint16_t length;
//		int i=0;
		
		volatile uint32_t temp;		///////////////////////////////// �����ȡ������û���ٴν��жϣ��ͱ�����
		temp = USART3->SR;			/////////////////////////////////
		temp = USART3->DR;			/////////////////////////////////
		(void)temp;  // ��ֹδʹ�þ���   //

		//����������IDLE��־λ
		DMA_Cmd(DMA1_Stream1,DISABLE);
		
		
        // ��ȡ�������ݳ���
        length = SIZEBUFF_IMU - DMA_GetCurrDataCounter(DMA1_Stream1); 
//		UsartSendByte(USART1, (uint8_t)length);
//		if((DataA_IMU[0]== 0xFC) && (DataA_IMU[1]== 0x58) && (DataA_IMU[length - 1]== 0xFD) && (length == 56))
//		{
//			LED_PC13 = !LED_PC13;
//			fdiComProtocolReceive(&_FDILink, DataA_IMU, length);
////			//���ÿ���ж�ȫ����ӡ����������Ϊ���������⣬��������ֻ��ʾ0x58��֡��0x59��֡����ʾ
////			for(i=0; i<length; i++){
////				UsartSendByte(USART1, DataA_IMU[i]);	//������λ������
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
		
		
		//���������¿���DMA����
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_FEIF1);
		DMA_SetCurrDataCounter(DMA1_Stream1, SIZEBUFF_IMU); //���賤���뵱�����ý��������±�Ϊ0
		DMA_Cmd(DMA1_Stream1,ENABLE);
		
        USART_ClearITPendingBit(USART3, USART_IT_IDLE);        // ��������ж�

	
    }
	
//	// ����ת��
//	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
//    {
//		u8 Res;
//		Res = USART_ReceiveData(USART3);	// �����Ҫִ��ת�����Ǿͱ��������� DMA_UART3_RX_init();
//		UsartSendByte(USART1, Res);
//		
//        USART_ClearITPendingBit(USART3, USART_IT_RXNE);        // ��������ж�
//    }
	
	
	//�˳��ж�
	OSIntExit();  	  		//UCOS����ϵͳ���ж�ʱ�������**********************************************    

}





/***********************************����IMU����ĺ���**************************************/
/* ��������ģʽ */
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


/* ��ȡ������paramNameΪ��Ҫ��ȡ�Ĳ�������
 *	\param[in]	paramName - Parameter name to be obtained.*/
int fdiComGetParam(char* paramName)
{
	int i=0;
	char send_buff[128];
	sprintf(send_buff, "#fparam get %s\r\n", paramName);
	
	for(i=0; i<strlen(send_buff); i++){
		UsartSendByte(USART3, send_buff[i]);	//���͵�ָ������
	}
	delay_ms(1000);	//LED_PC13 = !LED_PC13;
	delay_ms(1000);

	return FDI_NO_ERROR;
}


/*  ���÷��͵��������ݣ�msgΪ2λ16�������ֱ�ʾ���ݰ�ID��freqΪ����ָ�����ݰ��ķ���Ƶ��
 *	Configure the data content to be sent.
 *	\param[in]	msg - Represents the packet ID; freq - Packet transmit frequency. */
int fdiComSetConfigPacketSentMsg(char* msg, int freq)
{	
	int i=0;
	char send_buff[128];
	sprintf(send_buff, "#fmsg %s %d\r\n", msg, freq);
	
	for(i=0; i<strlen(send_buff); i++){
		UsartSendByte(USART3, send_buff[i]);	//���͵�ָ������
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
		UsartSendByte(USART3, send_buff[i]);	//���͵�ָ������
	}
	delay_ms(1000);	//LED_PC13 = !LED_PC13;
	delay_ms(1000);

	return FDI_NO_ERROR;
}



/* �������޸ĵ����ã���Ҫ�����ݸ�����Ҫ���棩*/
int fdiSetSave(void)
{
	int i=0;
	char send_buff[128];
	sprintf(send_buff, "#fsave\r\n");
	
	for(i=0; i<strlen(send_buff); i++){
		UsartSendByte(USART3, send_buff[i]);	//���͵�ָ������
	}
	delay_ms(1000);	//LED_PC13 = !LED_PC13;
	delay_ms(1000);

	return FDI_NO_ERROR;
}



/* �˳�����ģʽ */
int fdiSetDeconfig(void)
{
	int i=0;
	char send_buff[128];
	sprintf(send_buff, "#fdeconfig\r\n");
	
	for(i=0; i<strlen(send_buff); i++){
		UsartSendByte(USART3, send_buff[i]);	//���͵�ָ������
	}
	delay_ms(1000);	//LED_PC13 = !LED_PC13;
	delay_ms(1000);

	return FDI_NO_ERROR;
}



/* �����豸����Ҫ���ݵĸ�����Ҫ�����豸��
 *	Restart the device (the device needs to be restarted for the update of important data).*/
int fdiSetReboot(void)
{
	int i=0;
	char send_buff[128];
	
	sprintf(send_buff, "#freboot\r\n");
		
	for(i=0; i<strlen(send_buff); i++){
		UsartSendByte(USART3, send_buff[i]);	//���͵�ָ������
	}
	delay_ms(1000);	//LED_PC13 = !LED_PC13;
	delay_ms(1000);

	
	sprintf(send_buff, "y\r\n");
		
	for(i=0; i<strlen(send_buff); i++){
		UsartSendByte(USART3, send_buff[i]);	//���͵�ָ������
	}
	delay_ms(1000);	//LED_PC13 = !LED_PC13;


	return FDI_NO_ERROR;
	
}




//uint8_t IMU_buf1[100];
//uint8_t IMU_buf2[100];
//		int len1 = 56;
//		int len2 = 64;
		
//		if(length > 40)		// ����ָ�����ȲŴ�����Ȼ��������
//		{
//			// ���ݴ����û��Զ���Э�飩
//			if((DataA_IMU[0]== 0xFC) && (DataA_IMU[length - 1]== 0xFD))
//			{
//				fdiComProtocolReceive(&_FDILink, DataA_IMU, length);
//				
//			}
//		}
		
		
//		if(length == 56)		// ����ָ�����ȲŴ�����Ȼ��������
//		{
//			// ���ݴ����û��Զ���Э�飩
//			if((DataA_IMU[0]== 0xFC) && (DataA_IMU[length - 1]== 0xFD))
//			{
//				fdiComProtocolReceive(&_FDILink, DataA_IMU, length);
//				
//			}
//		}
			
//		if(length == 120)		// ����ָ�����ȲŴ�����Ȼ��������
//		{
//			if((DataA_IMU[0]== 0xFC) && (DataA_IMU[57]== 0x40) && (DataA_IMU[length - 1]== 0xFD))
//			{
//				LED_PC13 = !LED_PC13;
//	//			for(i=0; i<length; i++){
//	//				UsartSendByte(USART1, DataA_IMU[i]);	//������λ������
//	//			}
//				
//				memset(IMU_buf1, 0, sizeof(IMU_buf1)); // �������
//				memset(IMU_buf2, 0, sizeof(IMU_buf1));
//				memcpy(IMU_buf1, DataA_IMU, sizeof(int) * len1);	// ǰ���һ�� memcpy
//				memcpy(IMU_buf2, DataA_IMU + len1, sizeof(int) * len2);	// ����һ�� memcpy
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
		
		//���ÿ���ж�ȫ����ӡ����������Ϊ���������⣬��������ֻ��ʾ0x58��֡��0x59��֡����ʾ
//		for(i=0; i<length; i++){
//			UsartSendByte(USART1, DataA_IMU[i]);	//������λ������
//		}
		
//		if((length >= 70))
//		{
//			LED_PC13 = !LED_PC13;
//			for(i=0; i<length; i++){
//					UsartSendByte(USART1, DataA_IMU[i]);	//������λ������
//				}
//			
//		}
