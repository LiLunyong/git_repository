#include "led.h"
#include "includes.h"					//ucos 使用	  

int LED_PD12_state = 1;

//  LED IO初始化
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOA, ENABLE); // 使能GPIOA和GPIOC时钟

    // GPIO初始化设置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;         // LED_PC13
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      // 普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       // 上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);             // 初始化GPIO

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; // LED_PA1
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; // 按键1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_SetBits(GPIOC, GPIO_Pin_13); // 设置高，灯灭
    GPIO_SetBits(GPIOA, GPIO_Pin_1);
    GPIO_SetBits(GPIOD, GPIO_Pin_10); // 蜂鸣器
    GPIO_SetBits(GPIOD, GPIO_Pin_12);   // 磁力计校准
    GPIO_SetBits(GPIOD, GPIO_Pin_13);   // SD卡状态
}
// 按键处理函数
// 返回按键值
// mode:0,不支持连续按;1,支持连续按;
// 0，没有任何按键按下
// 1，KEY按下
u8 KEY_Scan(u8 mode)
{
    static u8 key_up = 1; // 按键按松开标志
    if (mode)
        key_up = 1; // 支持连按
    if (key_up && (KEY_PD11_in == 0))
    {
        delay_ms(10); // 去抖动
        key_up = 0;
        if (KEY_PD11_in == 0)
            return 1;
    }
    else if (KEY_PD11_in == 1)
        key_up = 1;
    return 0; // 无按键按下
}

// 外部中断初始化程序
void EXTI11_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    LED_Init();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);         // 使能SYSCFG时钟
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource11); // PD11 连接到中断线11

    /* 配置EXTI_Line11 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line11;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;     // 中断事件
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;               // 中断线使能
    EXTI_Init(&EXTI_InitStructure);                         // 配置

    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;         // 外部中断11
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; // 抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;        // 子优先级2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;              // 使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);                              // 配置
}

// 外部中断11服务程序
void EXTI15_10_IRQHandler(void)
{
	//进入中断
	OSIntEnter();	  	//UCOS操作系统，中断时，必须的**********************************************
	
	
    if (EXTI_GetITStatus(EXTI_Line11) != RESET)
    {
        if (KEY_Scan(0))
        {
            LED_PD12 = !LED_PD12;             // 磁力计校准led指示  状态转换， 这两初始都是1
            LED_PD12_state = !LED_PD12_state; // led指示状态
        }
        EXTI_ClearITPendingBit(EXTI_Line11); // 清除标志
    }
	
	
	//退出中断
	OSIntExit();	  	//UCOS操作系统，中断时，必须的**********************************************   
}
