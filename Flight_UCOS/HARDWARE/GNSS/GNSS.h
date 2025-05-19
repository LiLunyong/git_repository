#ifndef __GNSS_H
#define __GNSS_H
#include "stdio.h"	
#include "sys.h"
#include "string.h"
#include "NMEA0183.h"




extern uint8_t DataA_GNSS[2000];
extern NMEA0183 gnss_nmea0183;

void Initial_UART4(unsigned long baudrate);
void DMA_UART4_RX_init(void);
void GPS_SEL_Init(void);



#endif



