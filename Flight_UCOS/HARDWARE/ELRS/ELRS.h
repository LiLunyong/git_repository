#ifndef __ELRS_H
#define __ELRS_H

#include "string.h"
#include "sys.h"
#include "delay.h"
#include "ELRS.h"


extern uint8_t 	DataA_ELRS[26];
extern uint32_t ELRS_ch1, ELRS_ch2, ELRS_ch3, ELRS_ch4, ELRS_ch5, ELRS_ch6, ELRS_ch7, ELRS_ch8;
extern uint8_t  aircraft_take_off;




void Initial_UART2(unsigned long baudrate);
void DMA_UART2_RX_init(void);


#endif


