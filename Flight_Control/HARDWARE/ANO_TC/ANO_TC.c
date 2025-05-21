#include "ANO_TC.h"

/**********为了匿名四轴上位机的协议定义的变量****************************/
// cup为小端模式存储，也就是在存储的时候，低位被存在0字节，高位在1字节
#define BYTE0(dwTemp) (*(char *)(&dwTemp))		 // 取出int型变量的低字节
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1)) //	取存储在此变量下一内存字节的内容，高字节
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))


// ANO_TC_Send01(ACCX,ACCY,ACCZ,GYROX,GYROY,GYROZ,1);//角加速度，角速度
void ANO_TC_Send01(s16 _accx, s16 _accy, s16 _accz, s16 _gyrx, s16 _gyry, s16 _gyrz, u8 sta)
{
	int i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 _cnt = 0;
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA;		 // 帧头
	BUFF[_cnt++] = 0xFF;		 // 目标地址
	BUFF[_cnt++] = 0X01;		 // 功能码
	BUFF[_cnt++] = 13;			 // 数据长度
	BUFF[_cnt++] = BYTE0(_accx); // 数据内容,小段模式，低位在前
	BUFF[_cnt++] = BYTE1(_accx); // 需要将字节进行拆分，调用上面的宏定义即可。
	BUFF[_cnt++] = BYTE0(_accy);
	BUFF[_cnt++] = BYTE1(_accy);
	BUFF[_cnt++] = BYTE0(_accz);
	BUFF[_cnt++] = BYTE1(_accz);
	BUFF[_cnt++] = BYTE0(_gyrx);
	BUFF[_cnt++] = BYTE1(_gyrx);
	BUFF[_cnt++] = BYTE0(_gyry);
	BUFF[_cnt++] = BYTE1(_gyry);
	BUFF[_cnt++] = BYTE0(_gyrz);
	BUFF[_cnt++] = BYTE1(_gyrz);
	BUFF[_cnt++] = sta;
	// SC和AC的校验直接抄最上面上面简介的即可
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // 串口逐个发送数据
}

// ANO_TC_Send02(stcMag.h[0],stcMag.h[1],stcMag.h[2],Hight*100 ,stcMag.T,1,1);//罗盘、气压、温度传感器数据
void ANO_TC_Send02(s16 MAG_X, s16 MAG_Y, s16 MAG_Z, s32 ALT_BAR, s16 TMP, u8 BAR_STA, u8 MAG_STA) // 罗盘、气压、温度传感器数据
{
	int i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 _cnt = 0;
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA;		 // 帧头
	BUFF[_cnt++] = 0xFF;		 // 目标地址
	BUFF[_cnt++] = 0X02;		 // 功能码
	BUFF[_cnt++] = 14;			 // 数据长度
	BUFF[_cnt++] = BYTE0(MAG_X); // 数据内容,小段模式，低位在前
	BUFF[_cnt++] = BYTE1(MAG_X); // 需要将字节进行拆分，调用上面的宏定义即可。
	BUFF[_cnt++] = BYTE0(MAG_Y);
	BUFF[_cnt++] = BYTE1(MAG_Y);
	BUFF[_cnt++] = BYTE0(MAG_Z);
	BUFF[_cnt++] = BYTE1(MAG_Z);
	BUFF[_cnt++] = BYTE0(ALT_BAR);
	BUFF[_cnt++] = BYTE1(ALT_BAR);
	BUFF[_cnt++] = BYTE2(ALT_BAR);
	BUFF[_cnt++] = BYTE3(ALT_BAR);
	BUFF[_cnt++] = BYTE0(TMP);
	BUFF[_cnt++] = BYTE1(TMP);
	BUFF[_cnt++] = BAR_STA;
	BUFF[_cnt++] = MAG_STA;
	// SC和AC的校验直接抄最上面上面简介的即可
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // 串口逐个发送数据
}

// ANO_TC_Send03(ROLL*100,PITCH*(-100),YAW*(-100),1);欧拉角
void ANO_TC_Send03(s16 ROL, s16 PIT, s16 YAW, u8 FUSION_STA)
{
	int i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 _cnt = 0;
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA;	   // 帧头
	BUFF[_cnt++] = 0xFF;	   // 目标地址
	BUFF[_cnt++] = 0X03;	   // 功能码
	BUFF[_cnt++] = 7;		   // 数据长度
	BUFF[_cnt++] = BYTE0(ROL); // 数据内容,小段模式，低位在前
	BUFF[_cnt++] = BYTE1(ROL); // 需要将字节进行拆分，调用上面的宏定义即可。
	BUFF[_cnt++] = BYTE0(PIT);
	BUFF[_cnt++] = BYTE1(PIT);
	BUFF[_cnt++] = BYTE0(YAW);
	BUFF[_cnt++] = BYTE1(YAW);
	BUFF[_cnt++] = FUSION_STA;
	// SC和AC的校验直接抄最上面上面简介的即可
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // 串口逐个发送数据
}

void ANO_TC_Send04(s16 V0, s16 V1, s16 V2, s16 V3, u8 FUSION_STA) // V0、V1、V2、V3：四元数，传输时扩大 10000 倍。FUSION_STA：融合状态
{
	int i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 _cnt = 0;
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA;	  // 帧头
	BUFF[_cnt++] = 0xFF;	  // 目标地址
	BUFF[_cnt++] = 0X04;	  // 功能码
	BUFF[_cnt++] = 9;		  // 数据长度
	BUFF[_cnt++] = BYTE0(V0); // 数据内容,小段模式，低位在前
	BUFF[_cnt++] = BYTE1(V0); // 需要将字节进行拆分，调用上面的宏定义即可。
	BUFF[_cnt++] = BYTE0(V1);
	BUFF[_cnt++] = BYTE1(V1);
	BUFF[_cnt++] = BYTE0(V2);
	BUFF[_cnt++] = BYTE1(V2);
	BUFF[_cnt++] = BYTE0(V3);
	BUFF[_cnt++] = BYTE1(V3);
	BUFF[_cnt++] = FUSION_STA;
	// SC和AC的校验直接抄最上面上面简介的即可
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // 串口逐个发送数据
}

// ANO_TC_Send05(Hight*100,0,1);高度    ANO_TC_Send05(44330*(1-pow(Pressure/101325,1/5.255)),0,1);//高度
void ANO_TC_Send05(u32 ALT_FU, u32 ALT_ADD, u8 FUSION_STA)
{
	int i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 _cnt = 0;
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA;		  // 帧头
	BUFF[_cnt++] = 0xFF;		  // 目标地址
	BUFF[_cnt++] = 0X05;		  // 功能码
	BUFF[_cnt++] = 9;			  // 数据长度
	BUFF[_cnt++] = BYTE0(ALT_FU); // 数据内容,小段模式，低位在前
	BUFF[_cnt++] = BYTE1(ALT_FU); // 需要将字节进行拆分，调用上面的宏定义即可。
	BUFF[_cnt++] = BYTE2(ALT_FU);
	BUFF[_cnt++] = BYTE3(ALT_FU);
	BUFF[_cnt++] = BYTE0(ALT_ADD);
	BUFF[_cnt++] = BYTE1(ALT_ADD);
	BUFF[_cnt++] = BYTE2(ALT_ADD);
	BUFF[_cnt++] = BYTE3(ALT_ADD);
	BUFF[_cnt++] = FUSION_STA;
	// SC和AC的校验直接抄最上面上面简介的即可
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // 串口逐个发送数据
}

void ANO_TC_Send07(s16 SPEED_X, s16 SPEED_Y, s16 SPEED_Z) // SPEED_XYZ：依次为 XYZ 方向上的速度，单位 cm/s。
{
	int i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 _cnt = 0;
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA;		   // 帧头
	BUFF[_cnt++] = 0xFF;		   // 目标地址
	BUFF[_cnt++] = 0X07;		   // 功能码
	BUFF[_cnt++] = 6;			   // 数据长度
	BUFF[_cnt++] = BYTE0(SPEED_X); // 数据内容,小段模式，低位在前
	BUFF[_cnt++] = BYTE1(SPEED_X); // 需要将字节进行拆分，调用上面的宏定义即可。
	BUFF[_cnt++] = BYTE0(SPEED_Y);
	BUFF[_cnt++] = BYTE1(SPEED_Y);
	BUFF[_cnt++] = BYTE0(SPEED_Z);
	BUFF[_cnt++] = BYTE1(SPEED_Z);
	// SC和AC的校验直接抄最上面上面简介的即可
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // 串口逐个发送数据
}

void ANO_TC_Send08(s32 POS_X, s32 POS_Y) // POS_XY：相比起飞点的位置偏移量，单位 cm。
{
	int i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 _cnt = 0;
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA;		 // 帧头
	BUFF[_cnt++] = 0xFF;		 // 目标地址
	BUFF[_cnt++] = 0X08;		 // 功能码
	BUFF[_cnt++] = 8;			 // 数据长度
	BUFF[_cnt++] = BYTE0(POS_X); // 数据内容,小段模式，低位在前
	BUFF[_cnt++] = BYTE1(POS_X); // 需要将字节进行拆分，调用上面的宏定义即可。
	BUFF[_cnt++] = BYTE2(POS_X);
	BUFF[_cnt++] = BYTE3(POS_X);
	BUFF[_cnt++] = BYTE0(POS_Y);
	BUFF[_cnt++] = BYTE1(POS_Y);
	BUFF[_cnt++] = BYTE2(POS_Y);
	BUFF[_cnt++] = BYTE3(POS_Y);
	// SC和AC的校验直接抄最上面上面简介的即可
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // 串口逐个发送数据
}

void ANO_TC_Send0D(s16 VOTAGE, s16 CURRENT) // VOTAGE、CURRENT：依次为电压、电流，传输时扩大 100 倍。
{
	int i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 _cnt = 0;
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA;		  // 帧头
	BUFF[_cnt++] = 0xFF;		  // 目标地址
	BUFF[_cnt++] = 0X0D;		  // 功能码
	BUFF[_cnt++] = 4;			  // 数据长度
	BUFF[_cnt++] = BYTE0(VOTAGE); // 数据内容,小段模式，低位在前
	BUFF[_cnt++] = BYTE1(VOTAGE); // 需要将字节进行拆分，调用上面的宏定义即可。
	BUFF[_cnt++] = BYTE0(CURRENT);
	BUFF[_cnt++] = BYTE1(CURRENT);
	// SC和AC的校验直接抄最上面上面简介的即可
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // 串口逐个发送数据
}

// ANO_TC_Send30(1 ,11 ,stcLonLat.lLon ,stcLonLat.lLat,(float)stcGPSV.sGPSHeight/10,9,9,9,9,9,9);
void ANO_TC_Send30(u8 FIX_STA, u8 S_NUM, s32 LNG, s32 LAT, s32 ALT_GPS, s16 N_SPE, s16 E_SPE, s16 D_SPE, u8 PDOP, u8 SACC, u8 VACC)
{
	int i;
	u8 sumcheck = 0; // FIX_STA：定位状态，UBX 协议的 FIX_STA。PDOP：定位精度，，0-20000，20000 表示 GPS 信息不可靠，传输时缩小 100 倍（0-200）SACC、VACC：依次为速度精度、高度精度，最大值 20000（mm），传输时除以 100。
	u8 addcheck = 0; // S_NUM：卫星数量  LNG、LAT：依次为经度、纬度,传输时扩大 10000000 倍变成整数传输,使用时除以 10000000 即可。
	u8 _cnt = 0;	 // ALT_GPS：GPS 模块解算出的高度。N_SPE、E_SPE、D_SPE：NED 速度（cm/s）。
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA; // 帧头
	BUFF[_cnt++] = 0xFF; // 目标地址
	BUFF[_cnt++] = 0X30; // 功能码
	BUFF[_cnt++] = 23;	 // 数据长度

	BUFF[_cnt++] = BYTE0(FIX_STA); // 数据内容,小段模式，低位在前

	BUFF[_cnt++] = BYTE0(S_NUM);

	BUFF[_cnt++] = BYTE0(LNG);
	BUFF[_cnt++] = BYTE1(LNG);
	BUFF[_cnt++] = BYTE2(LNG);
	BUFF[_cnt++] = BYTE3(LNG);

	BUFF[_cnt++] = BYTE0(LAT);
	BUFF[_cnt++] = BYTE1(LAT);
	BUFF[_cnt++] = BYTE2(LAT);
	BUFF[_cnt++] = BYTE3(LAT);

	BUFF[_cnt++] = BYTE0(ALT_GPS);
	BUFF[_cnt++] = BYTE1(ALT_GPS);
	BUFF[_cnt++] = BYTE2(ALT_GPS);
	BUFF[_cnt++] = BYTE3(ALT_GPS);

	BUFF[_cnt++] = BYTE0(N_SPE);
	BUFF[_cnt++] = BYTE1(N_SPE);

	BUFF[_cnt++] = BYTE0(E_SPE);
	BUFF[_cnt++] = BYTE1(E_SPE);

	BUFF[_cnt++] = BYTE0(D_SPE);
	BUFF[_cnt++] = BYTE1(D_SPE);

	BUFF[_cnt++] = BYTE0(PDOP);
	BUFF[_cnt++] = BYTE0(SACC);
	BUFF[_cnt++] = BYTE0(VACC);
	// SC和AC的校验直接抄最上面上面简介的即可
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // 串口逐个发送数据
}

void ANO_TC_Send32(u32 POS_X, u32 POS_Y, u32 POS_Z)
{
	int i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 _cnt = 0;
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA;		 // 帧头
	BUFF[_cnt++] = 0xFF;		 // 目标地址
	BUFF[_cnt++] = 0X32;		 // 功能码
	BUFF[_cnt++] = 12;			 // 数据长度
	BUFF[_cnt++] = BYTE0(POS_X); // 数据内容,小段模式，低位在前
	BUFF[_cnt++] = BYTE1(POS_X); // 需要将字节进行拆分，调用上面的宏定义即可。
	BUFF[_cnt++] = BYTE2(POS_X);
	BUFF[_cnt++] = BYTE3(POS_X);
	BUFF[_cnt++] = BYTE0(POS_Y);
	BUFF[_cnt++] = BYTE1(POS_Y);
	BUFF[_cnt++] = BYTE2(POS_Y);
	BUFF[_cnt++] = BYTE3(POS_Y);
	BUFF[_cnt++] = BYTE0(POS_Z);
	BUFF[_cnt++] = BYTE1(POS_Z);
	BUFF[_cnt++] = BYTE2(POS_Z);
	BUFF[_cnt++] = BYTE3(POS_Z);
	// SC和AC的校验直接抄最上面上面简介的即可
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // 串口逐个发送数据
}



/*
// static u8 AAAA = 1;
// Openlog_Save(AAAA++, 'H', 'I', 'J'); // 修改比较值，修改占空比

// void Openlog_Save(u8 AAAA, u8 _accx, u8 _accy, u8 _accz)
// {
// 	int i;
// 	u8 _cnt = 0;
// 	u8 BUFF[100];

// 	BUFF[_cnt++] = AAAA;
// 	BUFF[_cnt++] = '\t';
// 	BUFF[_cnt++] = _accx;
// 	BUFF[_cnt++] = '\t';
// 	BUFF[_cnt++] = _accy;
// 	BUFF[_cnt++] = '\t';
// 	BUFF[_cnt++] = _accz;
// 	BUFF[_cnt++] = '\n';

// 	for (i = 0; i < _cnt; i++)
// 		UsartSendByte(USART1, BUFF[i]); // 串口逐个发送数据
// }
*/

