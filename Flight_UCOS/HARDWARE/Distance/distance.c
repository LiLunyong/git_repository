#include "string.h"
#include "led.h"
#include "delay.h"
#include "distance.h"
#include "includes.h"					//ucos ʹ��	  	//UCOS����ϵͳ���ж�ʱ�������**********************************************



u16 distance = 0;		 //(��λ cm)




///////////////////////////////���ڷ�ʽ��ȡ
//void Initial_UART6(u32 bound)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); 	// USART6 �� APB2 

//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6); 
//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6); 

//	// USART1�˿�����
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			 // ���ù���
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 // �ٶ�50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			 // ���츴�����
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			 // ����
//	GPIO_Init(GPIOC, &GPIO_InitStructure);					 // ��ʼ��

//	// USART1 ��ʼ������
//	USART_InitStructure.USART_BaudRate = bound;										// ����������
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// �ֳ�Ϊ8λ���ݸ�ʽ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// һ��ֹͣλ
//	USART_InitStructure.USART_Parity = USART_Parity_No;								// ����żУ��λ
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// �շ�ģʽ
//	USART_Init(USART6, &USART_InitStructure);										// ��ʼ������1

//	USART_Cmd(USART6, ENABLE); // ʹ�ܴ���
//	USART_ClearFlag(USART6, USART_FLAG_TC);
//	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE); // ��������ж�

//	// Usart6 NVIC ����
//	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;		  // ����1�ж�ͨ��
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // ��ռ���ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  // �����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);							  // ����ָ���Ĳ�����ʼ��NVIC�Ĵ���
//}


//// ����һ���ֽ�����
//// input:byte,�����͵�����
//void USART6_send_byte(uint8_t byte)
//{
//	while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET)
//		; // �ȴ��������
//	USART6->DR = byte;
//}
//// ���Ͷ��ֽ�����
//void USART6_Send_bytes(uint8_t *Buffer, uint8_t Length)
//{
//	uint8_t i = 0;
//	while (i < Length)
//	{
//		USART6_send_byte(Buffer[i++]);
//	}
//}
//// ���Ͷ��ֽ�����+У���
//void USART6_Send(uint8_t *Buffer, uint8_t Length)
//{
//	uint8_t i = 0;
//	while (i < Length)
//	{
//		if (i < (Length - 1))
//			Buffer[Length - 1] += Buffer[i]; // �ۼ�Length-1ǰ������
//		USART6_send_byte(Buffer[i++]);
//	}
//}

//void USART6_IRQHandler(void)
//{
//	//�����ж�
//	OSIntEnter();  	  		//UCOS����ϵͳ���ж�ʱ�������**********************************************  
//	
//	static uint8_t i = 0, rebuf[20] = {0};
//	uint8_t iii = 0, sum = 0;

//	if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) // �жϽ��ձ�־
//	{
//		rebuf[i++] = USART_ReceiveData(USART6); // ��ȡ�������ݣ�ͬʱ����ձ�־
//		if (rebuf[0] != 0x5a)					// ֡ͷ����
//			i = 0;
//		if ((i == 2) && (rebuf[1] != 0x5a)) // ֡ͷ����
//			i = 0;

//		if (i > 3) // i����4ʱ���Ѿ����յ��������ֽ�rebuf[3]
//		{
//			if (i != (rebuf[3] + 5)) 	// �ж��Ƿ����һ֡�������
//				return;					// �ܹ�7λ��������i++������������ֻ��ȡ��[6]����i��7
//			switch (rebuf[2]) // ������Ϻ���
//			{
//				case 0x45:
//					for(sum=0,iii=0; iii < (rebuf[3] + 4); iii++) {sum += rebuf[iii];}
//					if(sum == rebuf[iii])//У����ж� iii��ʱ����6
//					{	
//						distance = (rebuf[4]<<8)|rebuf[5]; // �������յ�������  //memcpy(Sonar_data, rebuf, 8);
//					}
//					break;
//				case 0x15:
//					break; // ԭʼ���ݽ��գ���ģ��0x45�ķ�ʽ
//				case 0x35:
//					break;
//			}
//			i = 0; // ������0
//		}
//	}


//	//�˳��ж�
//	OSIntExit();			//UCOS����ϵͳ���ж�ʱ�������**********************************************	
//}


//// send_com(0x56, 0x01); // ��ѯ�����������,ֻ�����һ��
//// send_com(0x56, 0x02); // ���������������,Լ 10hz��Ĭ�ϣ�
//// send_com(0x58, 0xAE); // ���������ã�9600��Ĭ�ϣ�
//// send_com(0x58, 0xAF); // ���������ã�115200
//// send_com(0x5A, 0x01); // ����������á�IIC ��ַ������������
//// send_com(0x5A, 0x02); // �ָ��������ã��ָ� IIC ��ַ������������
//void send_com(u8 com1, u8 com2)
//{
//	u8 bytes[4] = {0};
//	bytes[0] = 0xa5;
//	bytes[1] = com1;
//	bytes[2] = com2; // �����ֽ�
//	bytes[3] = bytes[0] + bytes[1] + bytes[2];
//	USART6_send_byte(bytes[0]); // ����֡ͷ�������ֽڡ�У���
//	USART6_send_byte(bytes[1]);
//	USART6_send_byte(bytes[2]);
//	USART6_send_byte(bytes[3]);
//}




///////////////////////////////���巽ʽ��ȡ
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	//���			
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;			
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	//����			
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
	while(PCin(7) == 0)//�ȴ��ߵ�ƽ����
	{
		t++;
		delay_us(1);  
		if(t>10000)
			return (unsigned int)-1;
	}
	
	t = 0;
	while(PCin(7))//�����ߵ�ƽʱ��
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


