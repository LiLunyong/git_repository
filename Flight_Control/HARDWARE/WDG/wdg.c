#include "wdg.h"
//////////////////////////////////////////////////////////////////////////////////	 
//STM32F4工程模板-库函数版本									  
////////////////////////////////////////////////////////////////////////////////// 



//初始化独立看门狗
//prer:分频数:0~7(只有低3位有效!)
//rlr:自动重装载值,0~0XFFF.
//分频因子=4*2^prer.但最大值只能是256!
//rlr:重装载寄存器值:低11位有效.
//时间计算(大概):Tout=((4*2^prer)*rlr)/32 (ms).
void IWDG_Init(u8 prer,u16 rlr)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //使能对IWDG->PR IWDG->RLR的写
	
	IWDG_SetPrescaler(prer); //设置IWDG分频系数

	IWDG_SetReload(rlr);   //设置IWDG装载值

	IWDG_ReloadCounter(); //reload
	
	IWDG_Enable();       //使能看门狗
}



//喂独立看门狗
void IWDG_Feed(void)
{
	IWDG_ReloadCounter();//reload
}

/*
if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET)
	{
		printf("iwdg reset cpu\r\n");
	}
	else
	{
		printf("nomal reset cpu\r\n");
	}
	RCC_ClearFlag();
*/
