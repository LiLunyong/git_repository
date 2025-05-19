#include "led.h"
#include "includes.h"					//ucos ʹ��	  

int LED_PD12_state = 1;

//  LED IO��ʼ��
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOA, ENABLE); // ʹ��GPIOA��GPIOCʱ��

    // GPIO��ʼ������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;         // LED_PC13
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      // ��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // �������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       // ����
    GPIO_Init(GPIOC, &GPIO_InitStructure);             // ��ʼ��GPIO

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; // LED_PA1
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; // ����1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_SetBits(GPIOC, GPIO_Pin_13); // ���øߣ�����
    GPIO_SetBits(GPIOA, GPIO_Pin_1);
    GPIO_SetBits(GPIOD, GPIO_Pin_10); // ������
    GPIO_SetBits(GPIOD, GPIO_Pin_12);   // ������У׼
    GPIO_SetBits(GPIOD, GPIO_Pin_13);   // SD��״̬
}
// ����������
// ���ذ���ֵ
// mode:0,��֧��������;1,֧��������;
// 0��û���κΰ�������
// 1��KEY����
u8 KEY_Scan(u8 mode)
{
    static u8 key_up = 1; // �������ɿ���־
    if (mode)
        key_up = 1; // ֧������
    if (key_up && (KEY_PD11_in == 0))
    {
        delay_ms(10); // ȥ����
        key_up = 0;
        if (KEY_PD11_in == 0)
            return 1;
    }
    else if (KEY_PD11_in == 1)
        key_up = 1;
    return 0; // �ް�������
}

// �ⲿ�жϳ�ʼ������
void EXTI11_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    LED_Init();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);         // ʹ��SYSCFGʱ��
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource11); // PD11 ���ӵ��ж���11

    /* ����EXTI_Line11 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line11;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;     // �ж��¼�
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // �½��ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;               // �ж���ʹ��
    EXTI_Init(&EXTI_InitStructure);                         // ����

    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;         // �ⲿ�ж�11
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; // ��ռ���ȼ�1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;        // �����ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;              // ʹ���ⲿ�ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);                              // ����
}

// �ⲿ�ж�11�������
void EXTI15_10_IRQHandler(void)
{
	//�����ж�
	OSIntEnter();	  	//UCOS����ϵͳ���ж�ʱ�������**********************************************
	
	
    if (EXTI_GetITStatus(EXTI_Line11) != RESET)
    {
        if (KEY_Scan(0))
        {
            LED_PD12 = !LED_PD12;             // ������У׼ledָʾ  ״̬ת���� ������ʼ����1
            LED_PD12_state = !LED_PD12_state; // ledָʾ״̬
        }
        EXTI_ClearITPendingBit(EXTI_Line11); // �����־
    }
	
	
	//�˳��ж�
	OSIntExit();	  	//UCOS����ϵͳ���ж�ʱ�������**********************************************   
}
