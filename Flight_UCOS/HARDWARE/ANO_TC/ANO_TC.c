#include "ANO_TC.h"

/**********Ϊ������������λ����Э�鶨��ı���****************************/
// cupΪС��ģʽ�洢��Ҳ�����ڴ洢��ʱ�򣬵�λ������0�ֽڣ���λ��1�ֽ�
#define BYTE0(dwTemp) (*(char *)(&dwTemp))		 // ȡ��int�ͱ����ĵ��ֽ�
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1)) //	ȡ�洢�ڴ˱�����һ�ڴ��ֽڵ����ݣ����ֽ�
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))


// ANO_TC_Send01(ACCX,ACCY,ACCZ,GYROX,GYROY,GYROZ,1);//�Ǽ��ٶȣ����ٶ�
void ANO_TC_Send01(s16 _accx, s16 _accy, s16 _accz, s16 _gyrx, s16 _gyry, s16 _gyrz, u8 sta)
{
	int i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 _cnt = 0;
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA;		 // ֡ͷ
	BUFF[_cnt++] = 0xFF;		 // Ŀ���ַ
	BUFF[_cnt++] = 0X01;		 // ������
	BUFF[_cnt++] = 13;			 // ���ݳ���
	BUFF[_cnt++] = BYTE0(_accx); // ��������,С��ģʽ����λ��ǰ
	BUFF[_cnt++] = BYTE1(_accx); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
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
	// SC��AC��У��ֱ�ӳ�������������ļ���
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // ���������������
}

// ANO_TC_Send02(stcMag.h[0],stcMag.h[1],stcMag.h[2],Hight*100 ,stcMag.T,1,1);//���̡���ѹ���¶ȴ���������
void ANO_TC_Send02(s16 MAG_X, s16 MAG_Y, s16 MAG_Z, s32 ALT_BAR, s16 TMP, u8 BAR_STA, u8 MAG_STA) // ���̡���ѹ���¶ȴ���������
{
	int i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 _cnt = 0;
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA;		 // ֡ͷ
	BUFF[_cnt++] = 0xFF;		 // Ŀ���ַ
	BUFF[_cnt++] = 0X02;		 // ������
	BUFF[_cnt++] = 14;			 // ���ݳ���
	BUFF[_cnt++] = BYTE0(MAG_X); // ��������,С��ģʽ����λ��ǰ
	BUFF[_cnt++] = BYTE1(MAG_X); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
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
	// SC��AC��У��ֱ�ӳ�������������ļ���
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // ���������������
}

// ANO_TC_Send03(ROLL*100,PITCH*(-100),YAW*(-100),1);ŷ����
void ANO_TC_Send03(s16 ROL, s16 PIT, s16 YAW, u8 FUSION_STA)
{
	int i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 _cnt = 0;
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA;	   // ֡ͷ
	BUFF[_cnt++] = 0xFF;	   // Ŀ���ַ
	BUFF[_cnt++] = 0X03;	   // ������
	BUFF[_cnt++] = 7;		   // ���ݳ���
	BUFF[_cnt++] = BYTE0(ROL); // ��������,С��ģʽ����λ��ǰ
	BUFF[_cnt++] = BYTE1(ROL); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	BUFF[_cnt++] = BYTE0(PIT);
	BUFF[_cnt++] = BYTE1(PIT);
	BUFF[_cnt++] = BYTE0(YAW);
	BUFF[_cnt++] = BYTE1(YAW);
	BUFF[_cnt++] = FUSION_STA;
	// SC��AC��У��ֱ�ӳ�������������ļ���
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // ���������������
}

void ANO_TC_Send04(s16 V0, s16 V1, s16 V2, s16 V3, u8 FUSION_STA) // V0��V1��V2��V3����Ԫ��������ʱ���� 10000 ����FUSION_STA���ں�״̬
{
	int i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 _cnt = 0;
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA;	  // ֡ͷ
	BUFF[_cnt++] = 0xFF;	  // Ŀ���ַ
	BUFF[_cnt++] = 0X04;	  // ������
	BUFF[_cnt++] = 9;		  // ���ݳ���
	BUFF[_cnt++] = BYTE0(V0); // ��������,С��ģʽ����λ��ǰ
	BUFF[_cnt++] = BYTE1(V0); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	BUFF[_cnt++] = BYTE0(V1);
	BUFF[_cnt++] = BYTE1(V1);
	BUFF[_cnt++] = BYTE0(V2);
	BUFF[_cnt++] = BYTE1(V2);
	BUFF[_cnt++] = BYTE0(V3);
	BUFF[_cnt++] = BYTE1(V3);
	BUFF[_cnt++] = FUSION_STA;
	// SC��AC��У��ֱ�ӳ�������������ļ���
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // ���������������
}

// ANO_TC_Send05(Hight*100,0,1);�߶�    ANO_TC_Send05(44330*(1-pow(Pressure/101325,1/5.255)),0,1);//�߶�
void ANO_TC_Send05(u32 ALT_FU, u32 ALT_ADD, u8 FUSION_STA)
{
	int i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 _cnt = 0;
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA;		  // ֡ͷ
	BUFF[_cnt++] = 0xFF;		  // Ŀ���ַ
	BUFF[_cnt++] = 0X05;		  // ������
	BUFF[_cnt++] = 9;			  // ���ݳ���
	BUFF[_cnt++] = BYTE0(ALT_FU); // ��������,С��ģʽ����λ��ǰ
	BUFF[_cnt++] = BYTE1(ALT_FU); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	BUFF[_cnt++] = BYTE2(ALT_FU);
	BUFF[_cnt++] = BYTE3(ALT_FU);
	BUFF[_cnt++] = BYTE0(ALT_ADD);
	BUFF[_cnt++] = BYTE1(ALT_ADD);
	BUFF[_cnt++] = BYTE2(ALT_ADD);
	BUFF[_cnt++] = BYTE3(ALT_ADD);
	BUFF[_cnt++] = FUSION_STA;
	// SC��AC��У��ֱ�ӳ�������������ļ���
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // ���������������
}

void ANO_TC_Send07(s16 SPEED_X, s16 SPEED_Y, s16 SPEED_Z) // SPEED_XYZ������Ϊ XYZ �����ϵ��ٶȣ���λ cm/s��
{
	int i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 _cnt = 0;
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA;		   // ֡ͷ
	BUFF[_cnt++] = 0xFF;		   // Ŀ���ַ
	BUFF[_cnt++] = 0X07;		   // ������
	BUFF[_cnt++] = 6;			   // ���ݳ���
	BUFF[_cnt++] = BYTE0(SPEED_X); // ��������,С��ģʽ����λ��ǰ
	BUFF[_cnt++] = BYTE1(SPEED_X); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	BUFF[_cnt++] = BYTE0(SPEED_Y);
	BUFF[_cnt++] = BYTE1(SPEED_Y);
	BUFF[_cnt++] = BYTE0(SPEED_Z);
	BUFF[_cnt++] = BYTE1(SPEED_Z);
	// SC��AC��У��ֱ�ӳ�������������ļ���
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // ���������������
}

void ANO_TC_Send08(s32 POS_X, s32 POS_Y) // POS_XY�������ɵ��λ��ƫ��������λ cm��
{
	int i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 _cnt = 0;
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA;		 // ֡ͷ
	BUFF[_cnt++] = 0xFF;		 // Ŀ���ַ
	BUFF[_cnt++] = 0X08;		 // ������
	BUFF[_cnt++] = 8;			 // ���ݳ���
	BUFF[_cnt++] = BYTE0(POS_X); // ��������,С��ģʽ����λ��ǰ
	BUFF[_cnt++] = BYTE1(POS_X); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	BUFF[_cnt++] = BYTE2(POS_X);
	BUFF[_cnt++] = BYTE3(POS_X);
	BUFF[_cnt++] = BYTE0(POS_Y);
	BUFF[_cnt++] = BYTE1(POS_Y);
	BUFF[_cnt++] = BYTE2(POS_Y);
	BUFF[_cnt++] = BYTE3(POS_Y);
	// SC��AC��У��ֱ�ӳ�������������ļ���
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // ���������������
}

void ANO_TC_Send0D(s16 VOTAGE, s16 CURRENT) // VOTAGE��CURRENT������Ϊ��ѹ������������ʱ���� 100 ����
{
	int i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 _cnt = 0;
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA;		  // ֡ͷ
	BUFF[_cnt++] = 0xFF;		  // Ŀ���ַ
	BUFF[_cnt++] = 0X0D;		  // ������
	BUFF[_cnt++] = 4;			  // ���ݳ���
	BUFF[_cnt++] = BYTE0(VOTAGE); // ��������,С��ģʽ����λ��ǰ
	BUFF[_cnt++] = BYTE1(VOTAGE); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	BUFF[_cnt++] = BYTE0(CURRENT);
	BUFF[_cnt++] = BYTE1(CURRENT);
	// SC��AC��У��ֱ�ӳ�������������ļ���
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // ���������������
}

// ANO_TC_Send30(1 ,11 ,stcLonLat.lLon ,stcLonLat.lLat,(float)stcGPSV.sGPSHeight/10,9,9,9,9,9,9);
void ANO_TC_Send30(u8 FIX_STA, u8 S_NUM, s32 LNG, s32 LAT, s32 ALT_GPS, s16 N_SPE, s16 E_SPE, s16 D_SPE, u8 PDOP, u8 SACC, u8 VACC)
{
	int i;
	u8 sumcheck = 0; // FIX_STA����λ״̬��UBX Э��� FIX_STA��PDOP����λ���ȣ���0-20000��20000 ��ʾ GPS ��Ϣ���ɿ�������ʱ��С 100 ����0-200��SACC��VACC������Ϊ�ٶȾ��ȡ��߶Ⱦ��ȣ����ֵ 20000��mm��������ʱ���� 100��
	u8 addcheck = 0; // S_NUM����������  LNG��LAT������Ϊ���ȡ�γ��,����ʱ���� 10000000 �������������,ʹ��ʱ���� 10000000 ���ɡ�
	u8 _cnt = 0;	 // ALT_GPS��GPS ģ�������ĸ߶ȡ�N_SPE��E_SPE��D_SPE��NED �ٶȣ�cm/s����
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA; // ֡ͷ
	BUFF[_cnt++] = 0xFF; // Ŀ���ַ
	BUFF[_cnt++] = 0X30; // ������
	BUFF[_cnt++] = 23;	 // ���ݳ���

	BUFF[_cnt++] = BYTE0(FIX_STA); // ��������,С��ģʽ����λ��ǰ

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
	// SC��AC��У��ֱ�ӳ�������������ļ���
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // ���������������
}

void ANO_TC_Send32(u32 POS_X, u32 POS_Y, u32 POS_Z)
{
	int i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 _cnt = 0;
	u8 BUFF[100];

	BUFF[_cnt++] = 0xAA;		 // ֡ͷ
	BUFF[_cnt++] = 0xFF;		 // Ŀ���ַ
	BUFF[_cnt++] = 0X32;		 // ������
	BUFF[_cnt++] = 12;			 // ���ݳ���
	BUFF[_cnt++] = BYTE0(POS_X); // ��������,С��ģʽ����λ��ǰ
	BUFF[_cnt++] = BYTE1(POS_X); // ��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
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
	// SC��AC��У��ֱ�ӳ�������������ļ���
	for (i = 0; i < _cnt; i++)
	{
		sumcheck += BUFF[i];
		addcheck += sumcheck;
	}
	BUFF[_cnt++] = sumcheck;
	BUFF[_cnt++] = addcheck;

	for (i = 0; i < _cnt; i++)
		UsartSendByte(USART1, BUFF[i]); // ���������������
}



/*
// static u8 AAAA = 1;
// Openlog_Save(AAAA++, 'H', 'I', 'J'); // �޸ıȽ�ֵ���޸�ռ�ձ�

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
// 		UsartSendByte(USART1, BUFF[i]); // ���������������
// }
*/

