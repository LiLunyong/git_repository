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
 * ��������: I2c_delay
 * ��������: I2c ��ʱ����
 * ��ڲ���: ��
 ******************************************************************************/
void I2c_delay(void)
{
    delay_us(5);
    // volatile unsigned int uiCnt = 4;
    // while (uiCnt--)
    // ;
}

/******************************************************************************
 * ��������: I2c_Init
 * ��������: I2c  GPIO��ʼ��
 * ��ڲ���: ��
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
 * ��������: I2c_Start
 * ��������: I2c  ��ʼ�ź�
 * ��ڲ���: ��
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
 * ��������: I2c_Stop
 * ��������: I2c  ֹͣ�ź�
 * ��ڲ���: ��
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
 * ��������: I2c_Ack
 * ��������: I2c  ����Ӧ���ź�
 * ��ڲ���: ��
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
 * ��������: I2c_NoAck
 * ��������: I2c  ����NAck
 * ��ڲ���: ��
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
 *��������:	I2c_WaitAck
 *��������:	�ȴ�Ӧ���źŵ���
 *����ֵ��   1������Ӧ��ʧ��
 *           0������Ӧ��ɹ�
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
 * ��������: I2c_SendByte
 * ��������: I2c  ����һ���ֽ�����
 * ��ڲ���: byte  ���͵�����
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
 * ��������: I2c_ReadByte
 * ��������: I2c  ��ȡһ���ֽ�����
 * ��ڲ���: ��
 * ����ֵ	 ��ȡ������
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
 *��������:	i2cWrite
 *��������:	д��ָ���豸 ָ���Ĵ���һ���ֽ�
 *��ڲ����� addr Ŀ���豸��ַ
 *		     reg   �Ĵ�����ַ
 *		     data ���������ݽ�Ҫ��ŵĵ�ַ
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
 * ��������: i2cWriteBuffer
 * ��������: I2c       ���豸��ĳһ����ַд��̶����ȵ�����
 * ��ڲ���: addr,     �豸��ַ
 *           reg��     �Ĵ�����ַ
 *			 len��     ���ݳ���
 *			 *data	   ����ָ��
 * ����ֵ	 1
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
    // I2c_Stop(); //�Ҹ㲻���ˣ�������ô�����IIC����һ������⣬�̼Ҹ��Ĵ���Ҳ�ǣ�ʱ��û�����ף�WTM����
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



