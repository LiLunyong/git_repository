/***********************************************************************
 * desined by LLY
 **********************************************************************/
#include "IIC.h"
#include "delay.h"

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED 1

/******************************************************************************
 * 函数名称: I2c_delay
 * 函数功能: I2c 延时函数
 * 入口参数: 无
 ******************************************************************************/
void I2c_delay(void)
{
    delay_us(5);
    // volatile unsigned int uiCnt = 4;
    // while (uiCnt--)
    // ;
}

/******************************************************************************
 * 函数名称: I2c_Init
 * 函数功能: I2c  GPIO初始化
 * 入口参数: 无
 ******************************************************************************/
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = SCL_PIN | SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(IIC_GPIO, &GPIO_InitStructure);

    IIC_SCL = 1;
    IIC_SDA = 1;
}

void SDA_PIN_mode(GPIOMode_TypeDef GPIO_Mode)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(IIC_GPIO, &GPIO_InitStructure);
}

/******************************************************************************
 * 函数名称: I2c_Start
 * 函数功能: I2c  起始信号
 * 入口参数: 无
 ******************************************************************************/
void I2c_Start(void)
{
    SDA_PIN_mode(GPIO_Mode_OUT);
    IIC_SDA = 1;
    IIC_SCL = 1;
    I2c_delay();

    IIC_SDA = 0;
    I2c_delay();

    IIC_SCL = 0;
    I2c_delay();
}

/******************************************************************************
 * 函数名称: I2c_Stop
 * 函数功能: I2c  停止信号
 * 入口参数: 无
 ******************************************************************************/
void I2c_Stop(void)
{
    SDA_PIN_mode(GPIO_Mode_OUT);
    IIC_SCL = 1;
    IIC_SDA = 0;
    I2c_delay();

    IIC_SDA = 1;
    I2c_delay();
}

/******************************************************************************
 * 函数名称: I2c_Ack
 * 函数功能: I2c  产生应答信号
 * 入口参数: 无
 ******************************************************************************/
static void I2c_Ack(void)
{
    SDA_PIN_mode(GPIO_Mode_OUT);
    IIC_SCL = 0;
    IIC_SDA = 0;
    I2c_delay();

    IIC_SCL = 1;
    I2c_delay();

    IIC_SCL = 0;
}

/******************************************************************************
 * 函数名称: I2c_NoAck
 * 函数功能: I2c  产生NAck
 * 入口参数: 无
 ******************************************************************************/
static void I2c_NoAck(void)
{
    SDA_PIN_mode(GPIO_Mode_OUT);
    IIC_SCL = 0;
    IIC_SDA = 1;
    I2c_delay();

    IIC_SCL = 1;
    I2c_delay();

    IIC_SCL = 0;
}

/*******************************************************************************
 *函数名称:	I2c_WaitAck
 *函数功能:	等待应答信号到来
 *返回值：   1，接收应答失败
 *           0，接收应答成功
 *******************************************************************************/
static uint8_t I2c_WaitAck(void)
{
    int ack;

    // SDA_PIN_mode(GPIO_Mode_OUT);
    // IIC_SDA = 1;
    // I2c_delay();
    IIC_SCL = 1;
    I2c_delay();

    SDA_PIN_mode(GPIO_Mode_IN);
    if (SDA_read)
        ack = 1;
    else
        ack = 0;

    IIC_SCL = 0;
    I2c_delay();

    return ack;
}

/******************************************************************************
 * 函数名称: I2c_SendByte
 * 函数功能: I2c  发送一个字节数据
 * 入口参数: byte  发送的数据
 ******************************************************************************/
static void I2c_SendByte(uint8_t byte)
{
    int i;
    SDA_PIN_mode(GPIO_Mode_OUT);

    IIC_SCL = 0;
    IIC_SDA = 0;
    I2c_delay();

    for (i = 7; i >= 0; i--)
    {
        if (byte & (1 << i))
            IIC_SDA = 1;
        else
            IIC_SDA = 0;
        I2c_delay();

        IIC_SCL = 1;
        I2c_delay();

        IIC_SCL = 0;
        I2c_delay();
    }
}

/******************************************************************************
 * 函数名称: I2c_ReadByte
 * 函数功能: I2c  读取一个字节数据
 * 入口参数: 无
 * 返回值	 读取的数据
 ******************************************************************************/
static uint8_t I2c_ReadByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;
    SDA_PIN_mode(GPIO_Mode_IN);

    while (i--)
    {
        IIC_SCL = 0;
        I2c_delay();

        IIC_SCL = 1;
        I2c_delay();

        byte <<= 1;
        if (SDA_read)
        {
            byte |= 0x01;
        }
    }
    IIC_SCL = 0;
    return byte;
}

u8 IIC_Read_1Byte(u8 SlaveAddress, u8 REG_Address, u8 *REG_data)
{
    I2c_Start();
    I2c_SendByte(SlaveAddress << 1);
    if (I2c_WaitAck() == FAILED)
    {
        I2c_Stop();
        return FAILED;
    }
    I2c_SendByte(REG_Address);
    I2c_WaitAck();
    // I2c_Stop();
    I2c_Start();
    I2c_SendByte((SlaveAddress << 1) + 1);
    if (I2c_WaitAck() == FAILED)
    {
        I2c_Stop();
        return FAILED;
    }
    *REG_data = I2c_ReadByte();
    I2c_NoAck();
    I2c_Stop();
    return SUCCESS;
}

/*****************************************************************************
 *函数名称:	i2cWrite
 *函数功能:	写入指定设备 指定寄存器一个字节
 *入口参数： addr 目标设备地址
 *		     reg   寄存器地址
 *		     data 读出的数据将要存放的地址
 *******************************************************************************/
u8 IIC_Write_1Byte(u8 SlaveAddress, u8 REG_Address, u8 REG_data)
{
    I2c_Start();
    I2c_SendByte(SlaveAddress << 1);
    if (I2c_WaitAck() == FAILED)
    {
        I2c_Stop();
        return FAILED;
    }
    I2c_SendByte(REG_Address);
    I2c_WaitAck();
    I2c_SendByte(REG_data);
    I2c_WaitAck();
    I2c_Stop();
    return SUCCESS;
}

/******************************************************************************
 * 函数名称: i2cWriteBuffer
 * 函数功能: I2c       向设备的某一个地址写入固定长度的数据
 * 入口参数: addr,     设备地址
 *           reg，     寄存器地址
 *			 len，     数据长度
 *			 *data	   数据指针
 * 返回值	 1
 ******************************************************************************/
u8 IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{
    int i;
    I2c_Start();
    I2c_SendByte(SlaveAddress << 1);
    if (I2c_WaitAck() == FAILED)
    {
        I2c_Stop();
        return FAILED;
    }
    I2c_SendByte(REG_Address);
    I2c_WaitAck();
    for (i = 0; i < len; i++)
    {
        I2c_SendByte(buf[i]);
        if (I2c_WaitAck() == FAILED)
        {
            I2c_Stop();
            return FAILED;
        }
    }
    I2c_Stop();
    return SUCCESS;
}

u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{
    I2c_Start();
    I2c_SendByte(SlaveAddress << 1);
    if (I2c_WaitAck() == FAILED)
    {
        I2c_Stop();
        return FAILED;
    }
    I2c_SendByte(REG_Address);
    I2c_WaitAck();
    // I2c_Stop(); //我搞不懂了，网上这么多软件IIC代码一大堆问题，商家给的代码也是，时序都没搞明白，WTM服了
    I2c_Start();
    I2c_SendByte((SlaveAddress << 1) + 1);
    if (I2c_WaitAck() == FAILED)
    {
        I2c_Stop();
        return FAILED;
    }
    while (len)
    {
        *buf = I2c_ReadByte();
        if (len == 1)
            I2c_NoAck();
        else
            I2c_Ack();
        buf++;
        len--;
    }
    I2c_Stop();
    return SUCCESS;
}



