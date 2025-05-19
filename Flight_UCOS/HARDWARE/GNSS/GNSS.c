#include "sys.h"
#include "usart.h"
#include "led.h"
#include "GNSS.h"	
#include "NMEA0183.h"
#include "includes.h"					//ucos ʹ��	  



uint32_t SIZEBUFF_GNSS=2000;	//����������Ļ������յ����ݻ᲻ȫ
uint8_t DataA_GNSS[2000];  		//ÿ����һ�����������ݣ��ͽ�һ�ο����ж�
NMEA0183 gnss_nmea0183;


void Initial_UART4(unsigned long baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // ʹ��GPIOAʱ��   						AHB1!!!!!!!!!!!!!!!!!!!!
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); // ʹ��UART4ʱ��RCC_APB1PeriphClockCmd   	APB1!!!!!!!!!!!!!!!!!!!!

    // ����1��Ӧ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4); // GPIOA0����ΪUART4
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4); // GPIOA1����ΪUART4

    // UART4�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; // GPIOA0��GPIOA1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           // ���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      // �ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         // ���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           // ����
    GPIO_Init(GPIOA, &GPIO_InitStructure);                 // ��ʼ��

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &USART_InitStructure);
	
    USART_ITConfig(UART4, USART_IT_TXE, DISABLE);
    USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);		// ���� UART4 ���߿����ж�
    USART_ClearFlag(UART4, USART_FLAG_TC);
    USART_Cmd(UART4, ENABLE);
	
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	DMA_UART4_RX_init();			// ˳���ʼ��DMA
	GPS_SEL_Init();					// ˳���ʼ������GPIO��ѡ�����ݵĴ��䷽��
}


void DMA_UART4_RX_init(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	DMA_DeInit(DMA1_Stream2);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	while (DMA_GetCmdStatus(DMA1_Stream2) != DISABLE){}

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR; //�������ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&DataA_GNSS; //�洢������ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //������Ϊ��Դ
	DMA_InitStructure.DMA_BufferSize = SIZEBUFF_GNSS;    //��������С
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
	DMA_Init(DMA1_Stream2,&DMA_InitStructure);

	USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(DMA1_Stream2,ENABLE);    //ʹ��usart4�������ݽ���
}


// GPS_SEL IO��ʼ��
void GPS_SEL_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // ʹ��GPIOCʱ��

    // GPIO��ʼ������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;         // LED_PC0
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      // ��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // �������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       // ����  ////////////////////������⣬�����������ܽ�ͨ
    GPIO_Init(GPIOC, &GPIO_InitStructure);             // ��ʼ��GPIO

    GPIO_SetBits(GPIOC, GPIO_Pin_0); // ���øߣ���IMU��GPS�����ӣ�GPS��������������
}



void UART4_IRQHandler(void)
{

	//�����ж�
	OSIntEnter();  	  		//UCOS����ϵͳ���ж�ʱ�������**********************************************      

	
    if (USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
    {
		uint16_t length;
		int i=0;
		
		volatile uint32_t temp;		///////////////////////////////// �����ȡ������û���ٴν��жϣ��ͱ�����
		temp = UART4->SR;			/////////////////////////////////
		temp = UART4->DR;			/////////////////////////////////
		(void)temp;  // ��ֹδʹ�þ���   //

		//����������IDLE��־λ
		DMA_Cmd(DMA1_Stream2,DISABLE);
		
		
        // ��ȡ�������ݳ���
        length = SIZEBUFF_GNSS - DMA_GetCurrDataCounter(DMA1_Stream2); 
		// ���ݴ���
		for(i=0; i<length; ++i)
		{
			if(nmea_decode(&gnss_nmea0183, DataA_GNSS[i]))
			{
			  ///��������ɹ�  //ÿ����һ�����������ݣ��ͽ�һ�ο����ж�
			}
		}
			
//		//if (DataA_GNSS[0] == 0x24 && DataA_GNSS[1] == 0x47){ } //�ж��ǲ���$G��ͷ
//		LED_PC13 = !LED_PC13;
//		for(i=0; i<length; i++){
//			UsartSendByte(USART1, DataA_GNSS[i]);	//������λ������
//		}
		
		
		
		//���������¿���DMA����
		DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_FEIF1);
		DMA_SetCurrDataCounter(DMA1_Stream2, SIZEBUFF_GNSS); //���賤���뵱�����ý��������±�Ϊ0
		DMA_Cmd(DMA1_Stream2,ENABLE);
		
        USART_ClearITPendingBit(UART4, USART_IT_IDLE);        // ��������ж�

    }

	
	//�˳��ж�
	OSIntExit();  	  		//UCOS����ϵͳ���ж�ʱ�������**********************************************      
}






