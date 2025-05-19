#ifndef __ANO_TC_H
#define __ANO_TC_H

#include "sys.h"
#include "usart.h"

void Openlog_Save(u8 AAAA, u8 _accx, u8 _accy, u8 _accz);

void ANO_TC_Send01(s16 _accx, s16 _accy, s16 _accz, s16 _gyrx, s16 _gyry, s16 _gyrz, u8 sta);      // ACC、GYR：依次为加速度、陀螺仪传感器数据。SHOCK_STA：震动状态。
void ANO_TC_Send02(s16 MAG_X, s16 MAG_Y, s16 MAG_Z, s32 ALT_BAR, s16 TMP, u8 BAR_STA, u8 MAG_STA); // MAG：磁罗盘传感器数据。TMP: 传感器温度，放大 10 倍传输，0.1 摄氏度。ALT_BAR：气压计高度，单位 cm。BAR_STA、MAG_STA：依次为气压状态、罗盘状态
void ANO_TC_Send03(s16 ROL, s16 PIT, s16 YAW, u8 FUSION_STA);                                      // ROL、PIT、YAW：姿态角，依次为横滚、俯仰、航向，精确到 0.01。FUSION _STA：融合状态。
void ANO_TC_Send04(s16 V0, s16 V1, s16 V2, s16 V3, u8 FUSION_STA);                                 // V0、V1、V2、V3：四元数，传输时扩大 10000 倍。FUSION_STA：融合状态
void ANO_TC_Send05(u32 ALT_FU, u32 ALT_ADD, u8 FUSION_STA);                                        // ALT_FU：融合后对地高度，单位厘米。ALT_ADD：附加高度传感高度数据，如超声波、激光测距，单位厘米。ALT_STA：测距状态。

void ANO_TC_Send07(s16 SPEED_X, s16 SPEED_Y, s16 SPEED_Z); // SPEED_XYZ：依次为 XYZ 方向上的速度，单位 cm/s。
void ANO_TC_Send08(s32 POS_X, s32 POS_Y);                  // POS_XY：相比起飞点的位置偏移量，单位 cm。

void ANO_TC_Send0D(s16 VOTAGE, s16 CURRENT); // VOTAGE、CURRENT：依次为电压、电流，传输时扩大 100 倍。
void ANO_TC_Send30(u8 FIX_STA, u8 S_NUM, s32 LNG, s32 LAT, s32 ALT_GPS, s16 N_SPE, s16 E_SPE, s16 D_SPE, u8 PDOP, u8 SACC, u8 VACC);
void ANO_TC_Send32(u32 POS_X, u32 POS_Y, u32 POS_Z);

#endif
