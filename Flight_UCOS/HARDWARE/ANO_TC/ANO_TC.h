#ifndef __ANO_TC_H
#define __ANO_TC_H

#include "sys.h"
#include "usart.h"

void Openlog_Save(u8 AAAA, u8 _accx, u8 _accy, u8 _accz);

void ANO_TC_Send01(s16 _accx, s16 _accy, s16 _accz, s16 _gyrx, s16 _gyry, s16 _gyrz, u8 sta);      // ACC��GYR������Ϊ���ٶȡ������Ǵ��������ݡ�SHOCK_STA����״̬��
void ANO_TC_Send02(s16 MAG_X, s16 MAG_Y, s16 MAG_Z, s32 ALT_BAR, s16 TMP, u8 BAR_STA, u8 MAG_STA); // MAG�������̴��������ݡ�TMP: �������¶ȣ��Ŵ� 10 �����䣬0.1 ���϶ȡ�ALT_BAR����ѹ�Ƹ߶ȣ���λ cm��BAR_STA��MAG_STA������Ϊ��ѹ״̬������״̬
void ANO_TC_Send03(s16 ROL, s16 PIT, s16 YAW, u8 FUSION_STA);                                      // ROL��PIT��YAW����̬�ǣ�����Ϊ��������������򣬾�ȷ�� 0.01��FUSION _STA���ں�״̬��
void ANO_TC_Send04(s16 V0, s16 V1, s16 V2, s16 V3, u8 FUSION_STA);                                 // V0��V1��V2��V3����Ԫ��������ʱ���� 10000 ����FUSION_STA���ں�״̬
void ANO_TC_Send05(u32 ALT_FU, u32 ALT_ADD, u8 FUSION_STA);                                        // ALT_FU���ںϺ�Եظ߶ȣ���λ���ס�ALT_ADD�����Ӹ߶ȴ��и߶����ݣ��糬�����������࣬��λ���ס�ALT_STA�����״̬��

void ANO_TC_Send07(s16 SPEED_X, s16 SPEED_Y, s16 SPEED_Z); // SPEED_XYZ������Ϊ XYZ �����ϵ��ٶȣ���λ cm/s��
void ANO_TC_Send08(s32 POS_X, s32 POS_Y);                  // POS_XY�������ɵ��λ��ƫ��������λ cm��

void ANO_TC_Send0D(s16 VOTAGE, s16 CURRENT); // VOTAGE��CURRENT������Ϊ��ѹ������������ʱ���� 100 ����
void ANO_TC_Send30(u8 FIX_STA, u8 S_NUM, s32 LNG, s32 LAT, s32 ALT_GPS, s16 N_SPE, s16 E_SPE, s16 D_SPE, u8 PDOP, u8 SACC, u8 VACC);
void ANO_TC_Send32(u32 POS_X, u32 POS_Y, u32 POS_Z);

#endif
