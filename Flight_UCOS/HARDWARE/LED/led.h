#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "delay.h"

#define LED_PC13 PCout(13)
#define LED_PA1 PAout(1)//

#define BB_PD10 PDout(10)
#define KEY_PD11_in PDin(11)
#define LED_PD12 PDout(12)
#define LED_PD13 PDout(13)
//#define LED_PD14 PDout(14)


void LED_Init(void);
u8 KEY_Scan(u8 mode);
void EXTI11_Init(void);


#endif
