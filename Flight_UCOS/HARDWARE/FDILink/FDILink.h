/*!
 *	\file		FDILink.h
 *  \author		FDISYSTEMS 
 *	\date		05 February 2023
 *
 *	\brief		Contains main FDILink methods.
 *
 *	\section CodeCopyright Copyright Notice 
 *	Copyright (C) 2018-2023, FDISYSTEMS  . All rights reserved.
 *	
 *	This source code is intended for use only by FDISYSTEMS and
 *	those that have explicit written permission to use it from
 *	FDISYSTEMS.
 *	
 *	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 *	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 *	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 *	PARTICULAR PURPOSE.
 */

/*!
 *	\mainpage FDISYSTEMS Communication library documentation
 *	Welcome to the FDILink library documentation.<br>
 *	This documentation describes all functions implemented in the FDILink library.
 */
#ifndef __SERIAL_BOOT_H
#define __SERIAL_BOOT_H
#include <stdint.h>
//未初始化
#define FDILink_Status_Uninitialized 		0
//运行
#define FDILink_Status_Running 				2
//正忙
#define FDILink_Status_Busy 				3

#define FDILink_Frame_Start					0
#define FDILink_Frame_CMD					1
#define FDILink_Frame_Length				2
#define FDILink_Frame_SerialNumber			3
#define FDILink_Frame_CRC8					4
#define FDILink_Frame_CRC16H				5
#define FDILink_Frame_CRC16L				6
#define FDILink_Frame_Data					7
#define FDILink_Frame_End					8

#define GPIO1								1
#define GPIO2								2

enum DGNSS
{
	QXWZ_DSK_KEY,
	QXWZ_DSK_SECRET,
	QXWZ_DEV_ID,
	QXWZ_DEV_TYPE,
	NTRIP_SVR_DOMAIN,
	NTRIP_SVR_PORT,
	NTRIP_ACCOUNT,
	NTRIP_PASSWORD,
	NTRIP_MOUNT,
	BASE_STATION_SOURCE,
	USR_AUTHEMTICATION,
	NET_INFO_IMEI,
	NET_INFO_CCID,
};
	

enum IMU 
{
	IMU_TEMP0,
	IMU_ACC_AVG, //重力加速度
	IMU_TO_BODY_ALGN_ROLL,
	IMU_TO_BODY_ALGN_PITCH,
	IMU_TO_BODY_ALGN_YAW,

	IMU_LEVEL_ALGN_ROLL,
	IMU_LEVEL_ALGN_PITCH,

	//IMU_ROT;
	IMU_MAG_INCL,
	IMU_MAG_DECL,
	IMU_PRESS_SENSE,

	/*************** ACC *********************/
	IMU_RANGE_ACC,  
	IMU_ACC_BIAS_X,
	IMU_ACC_BIAS_Y,
	IMU_ACC_BIAS_Z,
	IMU_ACC_BIAS_TEMP1_X,
	IMU_ACC_BIAS_TEMP1_Y,
	IMU_ACC_BIAS_TEMP1_Z,
	IMU_ACC_BIAS_TEMP2_X,
	IMU_ACC_BIAS_TEMP2_Y,
	IMU_ACC_BIAS_TEMP2_Z,
	IMU_ACC_BIAS_TEMP3_X,
	IMU_ACC_BIAS_TEMP3_Y,
	IMU_ACC_BIAS_TEMP3_Z,
	IMU_ACC_BIAS_DTEMP_X,
	IMU_ACC_BIAS_DTEMP_Y,
	IMU_ACC_BIAS_DTEMP_Z,
	IMU_ACC_SCAL_CORR_X,
	IMU_ACC_SCAL_CORR_Y,
	IMU_ACC_SCAL_CORR_Z,
	IMU_ACC_SCAL_CORR_TEMP1_X,
	IMU_ACC_SCAL_CORR_TEMP1_Y,
	IMU_ACC_SCAL_CORR_TEMP1_Z,
	IMU_ACC_SCAL_CORR_TEMP2_X,
	IMU_ACC_SCAL_CORR_TEMP2_Y,
	IMU_ACC_SCAL_CORR_TEMP2_Z,
	IMU_ACC_SCAL_CORR_TEMP3_X,
	IMU_ACC_SCAL_CORR_TEMP3_Y,
	IMU_ACC_SCAL_CORR_TEMP3_Z,
	IMU_ACC_SCAL_CORR_DTEMP_X,
	IMU_ACC_SCAL_CORR_DTEMP_Y,
	IMU_ACC_SCAL_CORR_DTEMP_Z,
	IMU_ACC_ALGN_XY,
	IMU_ACC_ALGN_XZ,
	IMU_ACC_ALGN_YX,
	IMU_ACC_ALGN_YZ,
	IMU_ACC_ALGN_ZX,
	IMU_ACC_ALGN_ZY,
	IMU_RANGE_GYO, 
	IMU_GYO_BIAS_X,
	IMU_GYO_BIAS_Y,
	IMU_GYO_BIAS_Z,
	IMU_GYO_BIAS_TEMP1_X,
	IMU_GYO_BIAS_TEMP1_Y,
	IMU_GYO_BIAS_TEMP1_Z,
	IMU_GYO_BIAS_TEMP2_X,
	IMU_GYO_BIAS_TEMP2_Y,
	IMU_GYO_BIAS_TEMP2_Z,
	IMU_GYO_BIAS_TEMP3_X,
	IMU_GYO_BIAS_TEMP3_Y,
	IMU_GYO_BIAS_TEMP3_Z,
	IMU_GYO_BIAS_DTEMP_X,
	IMU_GYO_BIAS_DTEMP_Y,
	IMU_GYO_BIAS_DTEMP_Z,
	IMU_GYO_SCAL_CORR_X,
	IMU_GYO_SCAL_CORR_Y,
	IMU_GYO_SCAL_CORR_Z,
	IMU_GYO_SCAL_CORR_TEMP1_X,
	IMU_GYO_SCAL_CORR_TEMP1_Y,
	IMU_GYO_SCAL_CORR_TEMP1_Z,
	IMU_GYO_SCAL_CORR_TEMP2_X,
	IMU_GYO_SCAL_CORR_TEMP2_Y,
	IMU_GYO_SCAL_CORR_TEMP2_Z,
	IMU_GYO_SCAL_CORR_TEMP3_X,
	IMU_GYO_SCAL_CORR_TEMP3_Y,
	IMU_GYO_SCAL_CORR_TEMP3_Z,
	IMU_GYO_SCAL_CORR_DTEMP_X,
	IMU_GYO_SCAL_CORR_DTEMP_Y,
	IMU_GYO_SCAL_CORR_DTEMP_Z,
	IMU_GYO_ALGN_XY,
	IMU_GYO_ALGN_XZ,
	IMU_GYO_ALGN_YX,
	IMU_GYO_ALGN_YZ,
	IMU_GYO_ALGN_ZX,
	IMU_GYO_ALGN_ZY,

	IMU_GYO_TEMP_CALIB,
	IMU_GYO_TEMP_CALIB_MIN,
	IMU_GYO_TEMP_CALIB_MAX,

	IMU_ACOMPASS_A,
	IMU_ACOMPASS_B,
	IMU_ACOMPASS_C,
	IMU_ACOMPASS_D,
	IMU_ACOMPASS_E,
	IMU_MAG_BIAS_X,
	IMU_MAG_BIAS_Y,
	IMU_MAG_BIAS_Z,
	IMU_MAG_BIAS_TEMP1_X,
	IMU_MAG_BIAS_TEMP1_Y,
	IMU_MAG_BIAS_TEMP1_Z,
	IMU_MAG_BIAS_TEMP2_X,
	IMU_MAG_BIAS_TEMP2_Y,
	IMU_MAG_BIAS_TEMP2_Z,
	IMU_MAG_BIAS_TEMP3_X,
	IMU_MAG_BIAS_TEMP3_Y,
	IMU_MAG_BIAS_TEMP3_Z,
	IMU_MAG_BIAS_DTEMP_X,
	IMU_MAG_BIAS_DTEMP_Y,
	IMU_MAG_BIAS_DTEMP_Z,
	IMU_MAG_SCAL_CORR_X,
	IMU_MAG_SCAL_CORR_Y,
	IMU_MAG_SCAL_CORR_Z,
	IMU_MAG_SCAL_CORR_TEMP1_X,
	IMU_MAG_SCAL_CORR_TEMP1_Y,
	IMU_MAG_SCAL_CORR_TEMP1_Z,
	IMU_MAG_SCAL_CORR_TEMP2_X,
	IMU_MAG_SCAL_CORR_TEMP2_Y,
	IMU_MAG_SCAL_CORR_TEMP2_Z,
	IMU_MAG_SCAL_CORR_TEMP3_X,
	IMU_MAG_SCAL_CORR_TEMP3_Y,
	IMU_MAG_SCAL_CORR_TEMP3_Z,
	IMU_MAG_SCAL_CORR_DTEMP_X,
	IMU_MAG_SCAL_CORR_DTEMP_Y,
	IMU_MAG_SCAL_CORR_DTEMP_Z,
	IMU_MAG_ALGN_XY,
	IMU_MAG_ALGN_XZ,
	IMU_MAG_ALGN_YX,
	IMU_MAG_ALGN_YZ,
	IMU_MAG_ALGN_ZX,
	IMU_MAG_ALGN_ZY,
	IMU_MAG_TO_ACC_ROLL,
	IMU_MAG_TO_ACC_PITCH,
	IMU_MAG_TEMP_CALIB,
	IMU_ACC_ENABLE,
	IMU_GYO_ENABLE,
	IMU_MAG_ENABLE,
};

typedef enum comm
{
	COM_NONE,
	COM1,
	COM2,
	COM3,
	COM4,
	COM5,
	COM6,
	COM7,
}comm_t;

typedef enum commBaud
{
	COMM_BAUD_ERROR,
	//常用串口波特率
	COMM_BAUD_9600,
	COMM_BAUD_19200,
	COMM_BAUD_38400,
	COMM_BAUD_76800,
	COMM_BAUD_115200,
	COMM_BAUD_230400,
	COMM_BAUD_460800,
	COMM_BAUD_921600,
	//接近STM32F4极限的串口波特率,由于精度的问题无法靠近常用的波特率
	COMM_BAUD_2625000,
	COMM_BAUD_5250000,
	COMM_BAUD_10500000,
	//SBUS波特率
	COMM_BAUD_100000,
	//常用CAN波特率
	COMM_BAUD_250000,
	COMM_BAUD_500000,
	COMM_BAUD_1000000,
}commBaud_t;

typedef enum commStreamTypes
{
	COMM_STREAM_TYPE_NONE = 0,
	
	COMM_STREAM_TYPE_MAIN,
	COMM_STREAM_TYPE_NAV,
	COMM_STREAM_TYPE_RTCM,
#ifdef DETA100_SERIES
	COMM_STREAM_TYPE_RTCM_EC600,
#endif
	COMM_STREAM_TYPE_NMEA,
	COMM_STREAM_TYPE_NMEA_OUT,
#ifdef DETA100_SERIES
	COMM_STREAM_TYPE_UBLOX,
	COMM_STREAM_TYPE_UM982,
	COMM_STREAM_TYPE_UBLOX_MOVING,
	COMM_STREAM_TYPE_UBLOX_ROVER,
#else
	COMM_STREAM_TYPE_UBLOX,
#endif	
	COMM_STREAM_TYPE_EXT_POS,        //外部位置
	COMM_STREAM_TYPE_EXT_VEL,        //外部速度
	COMM_STREAM_TYPE_EXT_PV,         //外部位置速度
	COMM_STREAM_TYPE_EXT_ATT,		 //外部姿态角度
	COMM_STREAM_TYPE_EXT_TIME,       //外部时间
	COMM_STREAM_TYPE_EXT_HEADING,    //外部航向
	COMM_STREAM_TYPE_EXT_DEPTH,      //外部航向
	COMM_STREAM_TYPE_EXT_SLAM1,      //外部SLAM1
	COMM_STREAM_TYPE_EXT_SLAM2,      //外部SLAM2
	COMM_STREAM_TYPE_EXT_PITOT_PRESS,//外部气压计
	COMM_STREAM_TYPE_EXT_AIR,        //外部空速管
	COMM_STREAM_TYPE_EXT_ODOM,       //外部里程计
	COMM_STREAM_TYPE_EXT_LIDAR,      //外部激光雷达
	COMM_STREAM_TYPE_OPTICAL_FLOW,	
	
	COMM_STREAM_TYPE_SCOUT_MINI,     //SCOUT MINI小车CAN总线编码器数据

	//COMM_STREAM_TYPE_FDILINK      = COMM_STREAM_TYPE_MAIN,	// FDILink
	//COMM_STREAM_TYPE_QGC          = COMM_STREAM_TYPE_MAIN,	// MAVLink telemetry format
}commStreamTypes_t;

enum userDefine 
{
	USER_DEFINE_ROLL,
	USER_DEFINE_PITCH,
	USER_DEFINE_YAW,
	USER_DEFINE_VELN,
	USER_DEFINE_VELE,
	USER_DEFINE_VELD,
	USER_DEFINE_HOLDLAT_1,
	USER_DEFINE_HOLDLAT_2,
	USER_DEFINE_HOLDLON_1,
	USER_DEFINE_HOLDLON_2,
	USER_DEFINE_CAN_ID,
};

enum gpioFunctionType 
{
	GPIOS_INACTIVE                        = 0,
	GPIOS_1PPS_OUTPUT                     = 1, //1PPS Output, Digital Output                      //TIM Output 联动模式
	GPIOS_1PPS_INPUT                      = 2, //1PPS Input, Digital Input                        //TIM Input  同步模式
	GPIOS_GNSS_FIX_OUTPUT                 = 3, //GNSS Fix Output, Digital Output                  //GPO
	GPIOS_ODOMETER_INPUT                  = 4, //Odometer Input, Frequency Input                  //TIM Input  计数模式
	GPIOS_ZERO_VELOCITY_INPUT             = 5, //Zero Velocity Input, Digital Input               //GPI
	GPIOS_DISABLE_GNSS                    = 6, //Disable GNSS , Digital Input                     //GPI
	GPIOS_DISABLE_PRESSURE                = 7, //Disable Pressure , Digital Input                 //GPI
	GPIOS_SET_ZERO_ORIENTATION_ALIGNMENT  = 8, //Set Zero Orientation Alignment , Digital Output  //GPI
	GPIOS_RAW_SENSORS_PACKET_TRIGGER      = 9, //Raw Sensors Packet Trigger , Digital Output      //TIM Output 同步模式 PWM模式
	GPIOS_LEFT_WHEEL_SPEED_SENSOR         = 10,//Left Wheel Speed Sensor ,Frequency Input         //TIM Input  计数模式
	GPIOS_RIGHT_WHEEL_SPEED_SENSOR        = 11,//right Wheel Speed Sensor ,Frequency Input        //TIM Input  计数模式
	GPIOS_WHEEL_ENCODER_PHASE_A           = 12,//Wheel Encoder PhaseA ,Frequency Input            //TIM Input  AB相模式 TIM不同无法实现
	GPIOS_WHEEL_ENCODER_PHASE_B           = 13,//Wheel Encoder PhaseB ,Frequency Input            //TIM Input  AB相模式 TIM不同无法实现
	GPIOS_EVENT_1_INPUT                   = 14,//Event1 Input ,Digital Output                     //GPI
	GPIOS_EVENT_2_INPUT                   = 15,//Event2 Input ,Digital Output                     //GPI
	GPIOS_SET_RESTART                     = 16,//Set Restart , Digital Output                     //GPI
	GPIOS_UART_TX                         = 17,//Set Restart , Digital Output                     //UART Tx
	GPIOS_UART_RX                         = 18,//Set Restart , Digital Input                      //UART Rx
};

typedef enum AID
{
	AID_ACCEL_GRAVITY,
	AID_MAG_V_MAGNETIC,
	AID_INIT_YAW_USE_MAG,
	AID_EXT_HEADING_UPDATE,
	AID_EXT_ATT_UPDATE,
	AID_EXT_VEL_UPDATE,
	AID_EXT_POS_UPDATE,
	AID_GNSS_VEL_UPDATE,
	AID_GNSS_POS_UPDATE,
#if defined EPSILON_SERIES || defined DETA100_SERIES
	AID_GNSS_DUAL_ANT_HEADING_UPDATE,
#if !defined DETA100_SERIES
	AID_FFT_HEAVE_ENABLED,
#endif
#endif
	AID_GNSS_TRACK_HEADING_UPDATE,
	AID_BRO_ALT_UPDATE,
	AID_OPTICFLOW_UPDATE,
	AID_ZERO_RATE_UPDATE,
	AID_ZERO_VEL_UPDATE,
	AID_ZERO_POS_UPDATE,
	AID_ODOMETER_VEL_UPDATE,
	AID_CAR_YZ_ZERO_VEL_NHC_ENABLED,
	AID_CAR_CENT_ACCEL_NHC_ENABLED,
	AID_GYO_TRUN_ON_TARE_ENABLED,
}AID_t;

typedef struct
{
	uint8_t Start;
	uint8_t CMD;
	uint8_t Length;
	uint8_t SerialNumber;
	uint8_t CRC8;
	union
	{
		uint16_t CRC16;
		struct
		{
			uint8_t CRC16L;
			uint8_t CRC16H;
		};
	};
	uint8_t Data[0];
}__attribute__((packed)) FDILink_Frame_t;

#define Storage_Type_Only_Code		0x0800
#define Storage_Type_Only_Data		0x0400
#define Storage_Type_Both			0x0000

#define Storage_Type_NONE			0x0000
#define Storage_Type_NOR_FLASH		0x0010
#define Storage_Type_NAND_FLASH		0x0020
#define Storage_Type_SRAM			0x0030
#define Storage_Type_DRAM			0x0040
#define Storage_Type_EEPROM			0x0050
#define Storage_Type_FRAM			0x0060
#define Storage_Type_ROM			0x0070
#define Storage_Type_UNKOWN			0x0080

#define Storage_Type_ExNOR_FLASH	0x8010
#define Storage_Type_ExNAND_FLASH	0x8020
#define Storage_Type_ExSRAM			0x8030
#define Storage_Type_ExDRAM			0x8040
#define Storage_Type_ExEEPROM		0x8050
#define Storage_Type_ExFRAM			0x8060
#define Storage_Type_ExUNKOWN		0x8080



typedef struct{
	uint32_t		Address;
	uint32_t		Length;
	uint32_t		Type;
}Storage_Info_t;



typedef struct{
	uint32_t		Address;
	uint32_t		Length;
	int32_t			Checked;
	uint32_t		Version;
}Firmware_Info_t;


typedef struct
{
	uint32_t 		Version_List_ID;			//列表版本ID
	uint32_t		Board_ID;					//PCB的ID
	uint32_t		Boot_Address;				//单位字节
	uint32_t		Boot_Length;				//单位字节
	uint32_t		APP_Entry;					//应用程序入口
	uint32_t		Chip_ID;					//芯片缩略ID
	uint32_t		Full_ID[4];
	Firmware_Info_t	Firmware[3];				//固件区
	Storage_Info_t	Storage[4];					//存储器
}__attribute__((packed)) FDILink_Version_t;
/***************************************************************
 *	下行指令
 **************************************************************/
#define FDILink_FrameCMD_Version			0x0c	//	获得版本
#define FDILink_FrameCMD_Keep 				0x0d	//	保持连接
#define FDILink_FrameCMD_Unlock 			0x0e	//	解锁
#define FDILink_FrameCMD_Erasure 			0x0f	//	擦除
#define FDILink_FrameCMD_Download 			0x10	//	下载
#define FDILink_FrameCMD_Upload 			0x11	//	上传
#define FDILink_FrameCMD_Verify 			0x12	//	存储校验
#define FDILink_FrameCMD_Reset 				0x13	//	跳转
#define FDILink_FrameCMD_Firmware			0x14	//	获得固件信息
#define FDILink_FrameCMD_Download_Message 	0x16	//	下载

/***************************************************************
 *	上行指令
 **************************************************************/
#define FDILink_FrameCMD_Catalog 			0x40	//	目录
#define FDILink_FrameCMD_Status 			0x41	//	当前状态
#define FDILink_FrameCMD_Error 				0x42	//	错误

#define FDILink_StatusType_Reconnecting		0x01
#define FDILink_StatusType_Processing		0x02
#define FDILink_StatusType_Idle				0x03
#define FDILink_StatusType_Break			0x04

#define FDILink_Return_OK					0
#define FDILink_Error_InvalidStart			-10	//无效的Start符
#define FDILink_Error_InvalidStatus			-11	//无效的状态
#define FDILink_Error_CRC8					-12	//错误的CRC8
#define FDILink_Error_CRC16					-13	//错误的CRC16

/***************************************************************
*	帧符号
***************************************************************/

#define FDILink_STX_Flag 0xFC
#define FDILink_EDX_Flag 0xFD
#define FDILink_Connect_Flag 0xFD

typedef struct FDILink_Status
{
	int 				BootStatus;
	int					RxStatus;
	int 				RxType;
	int 				RxDataLeft;
	int		 			RxNumber;
	int		 			TxNumber;
	int 				CRC8_Verify;
	int 				CRC16_Verify;
	uint32_t 			BufferIndex;
	uint8_t 			FDILink_Frame_Buffer[12];
	uint8_t 			Buffer[256];
}FDILink_Status_t;

#define Level    0
#define Acce     1
#define Gyro     2
#define Enable   1
#define Disable  0
#define FDI_NO_ERROR 0
#define FDI_ERROR    1

int fdiComProtocolInit(FDILink_Status_t* FDILink);
int fdiComProtocolSend(FDILink_Status_t* FDILink, uint8_t type, uint8_t * buf, int len);
int fdiComBufferTrans(uint8_t* buffer, FDILink_Status_t* FDILink, uint8_t type, void* buf, int len);
int fdiComProtocolReceive(FDILink_Status_t* FDILink, uint8_t * buf, int len);
int fdiGetPacket(uint8_t ID);																	//查询数据帧并解析
//int fdiComGetAxis(void);																					//查询设备安装的方向
//int fdiComGetAnte(void);																					//查询双天线航向与载体前向夹角							
//int fdiGetDgnss(int DGNSS);																		//读取GNSS设置参数

void fdiResetAll(FDILink_Status_t* FDILink);				//全部回到原始状态
void fdiDecodeBuffer(int PACKET_ID,void* buf);



#include "fdilink_decode.h"
extern FDILink_VersionData_Packet_t 	VersionData;
extern FDILink_IMUData_Packet_t 		IMUData;
extern FDILink_AHRSData_Packet_t 		AHRSData;
extern FDILink_INSGPSData_Packet_t 	INSGPSData;
extern System_State_Packet_t 			Sysdata;
extern Unix_Time_Packet_t 				TimeData;
extern Formatted_Time_Packet_t 		FTimeData;
extern Position_Standard_Deviation_Packet_t Posidata;
extern Velocity_Standard_Deviation_Packet_t Velodata;
extern Euler_Orientation_Standard_Deviation_Packet_t Euledata;
extern Quaternion_Orientation_Standard_Deviation_Packet_t Quatdata;
extern Raw_Sensors_Packet_t 	Raw_Sensor_data;
extern Raw_GNSS_Packet_t 		Raw_GNSS_data;

extern Euler_Orientation_Packet_t 			Euler_data;


#define VERSIONDATA         	 "39"
#define MSG_IMU                "40"
#define MSG_AHRS               "41"
#define MSG_INSGPS             "42"
#define MSG_SYS_STATE          "50"
#define MSG_UNIX_TIME          "51"
#define MSG_FORMAT_TIME        "52"
#define MSG_STATUS             "53"
#define MSG_POS_STD_DEV        "54"
#define MSG_VEL_STD_DEV        "55"
#define MSG_EULER_ORIEN_STD_DEV "56"
#define MSG_QUAT_ORIEN_STD_DEV "57"
#define MSG_RAW_SENSORS        "58"
#define MSG_RAW_GNSS           "59"
#define MSG_SATELLITE          "5a"
#define MSG_DETAILED_SATELLITE "5b"
#define MSG_GEODETIC_POS       "5c"
#define MSG_ECEF_POS           "5d"
#define MSG_UTM_POS            "5e"
#define MSG_NED_VEL            "5f"
#define MSG_BODY_VEL           "60"
#define MSG_ACCELERATION       "61"
#define MSG_BODY_ACCELERATION  "62"
#define MSG_EULER_ORIEN        "63"
#define MSG_QUAT_ORIEN         "64"
#define MSG_DCM_ORIEN          "65"
#define MSG_ANGULAR_VEL        "66"
#define MSG_ANGULAR_ACC        "67"
#define MSG_RUNNING_TIME       "6d"
#define MSG_LOCAL_MAG_FIELD    "6e"
#define MSG_ODOMETER_STATE     "6f"
#define MSG_GEOID_HEIGHT       "72"
#define MSG_RTCM_CORRECTIONS   "73"
#define MSG_WIND               "75"
#define MSG_HEAVE              "76"
#define MSG_RAW_SATELLITE      "77"
#define MSG_GNSS_DUAL_ANT      "78"
#define MSG_GIMBAL_STATE       "7a"
#define MSG_AUTOMOTIVE         "7b"
#define MSG_PACKET_TIMER_PERIOD "7c"
#define MSG_PACKETS_PERIOD     "7d"
#define MSG_INSTALL_ALIGN      "80"
#define MSG_FILTER_OPTIONS     "81"
#define MSG_GPIO_CONFIG        "82"
#define MSG_MAG_CALI_VALUES    "83"
#define MSG_MAG_CALI_CONFIG    "84"
#define MSG_MAG_CALI_STATUS    "85"
#define MSG_ODOMETER_CONFIG    "86"
#define MSG_SET_ZERO_ORIENT_ALIGN "87"
#define MSG_REF_POINT_OFFSET   "88"
#define MSG_USER_DATA          "8a"
#define MSG_BAUD_RATES         "a0"
#define MSG_SENSOR_RANGES      "a1"
#define MSG_GPIO_OUTPUT_CONFIG "a2"
#define MSG_GPIO_INPUT_CONFIG  "a3"
#define MSG_DUAL_ANT           "a4"

//定义串口发送函数
//格式 int Serial_Send(void* buffer,int length)
#define Serial_Send(...) 0

#endif
