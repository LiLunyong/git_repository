#include "sys.h"
#include "usart.h"	
#include "includes.h"					//ucos ʹ��	  


//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  

#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}

 
//��ʼ��IO ����1 
//baud:������
void usart_init(u32 baud)
{
   //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = baud;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
	USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����




	/***************************** ��������MD0��MD1���� *********************************/
	/************************** MD0 = 0; MD1 = 0 ��˫��ģʽ *****************************/
	/************************** MD0 = 1; MD1 = 0 ȫ˫��ģʽ *****************************/
	/************************** MD0 = 0; MD1 = 1 Ԥ��ģʽ ����ģʽ0 ��ͬ  ****************/
	/************************** MD0 = 1; MD1 = 1 ����ģʽ *******************************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      // ��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // �������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       // ����
    GPIO_Init(GPIOA, &GPIO_InitStructure);             // ��ʼ��GPIO
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_11);
	GPIO_ResetBits(GPIOA, GPIO_Pin_12);
	
	
}

#include "includes.h"					//ucos ʹ��	  	//UCOS����ϵͳ���ж�ʱ�������**********************************************

void USART1_IRQHandler(void)                	//����1�жϷ������
{
//	uint8_t Res=0;

	//�����ж�
	OSIntEnter();  	  		//UCOS����ϵͳ���ж�ʱ�������**********************************************  

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		//���մ�������
//		Res = USART_ReceiveData(USART1);	
		
//		// ����ת��
//		UsartSendByte(USART3, Res);	
		
		//��մ��ڽ����жϱ�־λ
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	} 
	
	//�˳��ж�
	OSIntExit();			//UCOS����ϵͳ���ж�ʱ�������**********************************************	
} 





struct COMM_RX_USART1 stc_COMM_RX_USART1 = {0xAA, 0xBB, 0xCC, 0xDD}; // ��ʼ��USART1����ָ��

void RX_CopeData_u8_USART1(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;

	ucRxBuffer[ucRxCnt++] = ucData; // ���յ������ݴ��뻺������
	if (ucRxBuffer[0] != 0x55)		// ����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt = 0;
		return;
	}
	if (ucRxCnt < 6)
	{
		return;
	} // ���ݲ���6�����򷵻�
	else
	{
		switch (ucRxBuffer[1]) // �ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
		{
		case 0x50:
			memcpy(&stc_COMM_RX_USART1, &ucRxBuffer[2], 4);
			break; // memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
		}
		ucRxCnt = 0; // ��ջ�����
	}
}

/*===================����һ���ַ�  ANO_TC.c��Ҫ��============*/
void UsartSendByte(USART_TypeDef *USARTx, uint8_t Data)
{
	USART_SendData(USARTx, Data);

	while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
		;
}


