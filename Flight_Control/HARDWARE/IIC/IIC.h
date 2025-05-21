#ifndef _I2C_SOFT_H
#define _I2C_SOFT_H

#include "stm32f4xx.h"
#include "sys.h"
/***************I2C GPIO定义******************/
#define IIC_GPIO GPIOB
#define SCL_PIN GPIO_Pin_6
#define SDA_PIN GPIO_Pin_7
/*********************************************/

#define IIC_SCL PBout(6) // SCL
#define IIC_SDA PBout(7) // SDA
#define SDA_read PBin(7) // 读取SDA

void I2c_delay(void);
void IIC_Init(void);
void SDA_PIN_mode(GPIOMode_TypeDef GPIO_Mode);

u8 IIC_Write_1Byte(u8 SlaveAddress, u8 REG_Address, u8 REG_data);
u8 IIC_Read_1Byte(u8 SlaveAddress, u8 REG_Address, u8 *REG_data);
u8 IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);

#endif


