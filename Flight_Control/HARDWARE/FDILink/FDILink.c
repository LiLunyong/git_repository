#include "string.h"
#include "stdio.h"
#include "FDILink.h"
#include "fdilink_decode.h"
//#include "FDI_config.h"
//----------------------------------------------------------------------//
//				          - FDI_LINK Function file                     -//
//----------------------------------------------------------------------//
extern uint8_t CRC8_Table(uint8_t* p, uint8_t counter);
extern uint16_t CRC16_Table(uint8_t* p, uint8_t counter);


#include "usart.h"


#ifndef FDI_ASSERT
#define FDI_ASSERT(x)
#endif
/**
  * @brief Reset the FDILink error status.
  * @param[in] FDILink Pointer to the FDILink status structure.
  */
void fdiErrorOccurred(FDILink_Status_t* FDILink)
{
	FDILink->RxStatus = FDILink_Frame_Start;
}
/**
  * @brief Insert a value into the FDILink buffer.
  * @param[in] FDILink Pointer to the FDILink status structure.
  * @param[in] value Value to be inserted.
  */
void fdiInsertBuffer(FDILink_Status_t* FDILink, uint8_t value)
{
	if (FDILink->RxDataLeft <= 0)
	{
		fdiErrorOccurred(FDILink);
		return;
	}
	if (FDILink->RxStatus != FDILink_Frame_Data)
	{
		fdiErrorOccurred(FDILink);
		return;
	}
	FDILink->Buffer[FDILink->BufferIndex++] = value;  //将元素放入队列尾部
	if (FDILink->BufferIndex >= 256)
	{
		fdiErrorOccurred(FDILink);
		return;
	}
	FDILink->RxDataLeft--;
}


/**
  * @brief Handle the running data received in the FDILink communication.
  * @param[in] FDILink Pointer to the FDILink status structure.
  * @param[in] value Received value.
	*	@return 0 for success, -3 if invalid RxStatus, -1 for error, 1 for complete frame received.
  */
int fdiRuningReceiveData(FDILink_Status_t* FDILink, uint8_t value)
{
	if (FDILink->RxStatus < FDILink_Frame_Start || FDILink->RxStatus > FDILink_Frame_End)
	{
		fdiErrorOccurred(FDILink);
		return -3;
	}
	FDILink->FDILink_Frame_Buffer[FDILink->RxStatus] = value;
	switch (FDILink->RxStatus)
	{
		case FDILink_Frame_Start:
			fdiResetAll(FDILink);
			if (value == FDILink_Connect_Flag)
			{
				return 0;
			}
			if (value != FDILink_STX_Flag)
			{
				fdiErrorOccurred(FDILink);
				return -1;
			}
			FDILink->RxStatus = FDILink_Frame_CMD;
			break;
		case FDILink_Frame_CMD:
			FDILink->RxType = value;
			FDILink->RxStatus = FDILink_Frame_Length;
			break;
		case FDILink_Frame_Length:
			FDILink->RxDataLeft = value;
			FDILink->RxStatus = FDILink_Frame_SerialNumber;
			break;
		case FDILink_Frame_SerialNumber:
			FDILink->RxNumber = value;
			FDILink->RxStatus = FDILink_Frame_CRC8;
			break;
		case FDILink_Frame_CRC8:
			FDILink->CRC8_Verify = value;
			if (CRC8_Table(FDILink->FDILink_Frame_Buffer, FDILink_Frame_CRC8) != FDILink->CRC8_Verify)
			{
				fdiErrorOccurred(FDILink);
				return -1;
			}
			if(FDILink->RxDataLeft == 0)
			{
				FDILink->RxStatus = FDILink_Frame_Start;
				return 1;
			}
			FDILink->RxStatus = FDILink_Frame_CRC16H;
			break;
		case FDILink_Frame_CRC16H:
			FDILink->CRC16_Verify = value;
			FDILink->RxStatus = FDILink_Frame_CRC16L;
			break;
		case FDILink_Frame_CRC16L:
			FDILink->CRC16_Verify = (FDILink->CRC16_Verify << 8) | value;
			FDILink->RxStatus = FDILink_Frame_Data;
			break;
		case FDILink_Frame_Data:
			if (FDILink->RxDataLeft)
			{
				fdiInsertBuffer(FDILink,value);
				if (FDILink->RxDataLeft == 0)
				{
					FDILink->RxStatus = FDILink_Frame_End;
				}
				break;
			}
			else
			{
				FDILink->RxStatus = FDILink_Frame_End;
			}

		case FDILink_Frame_End:
		{
			if (value != FDILink_EDX_Flag)
			{
				fdiErrorOccurred(FDILink);
				return -1;
			}
			uint16_t CRC16 = CRC16_Table(FDILink->Buffer, FDILink->BufferIndex);
			if (CRC16 != FDILink->CRC16_Verify)
			{
				fdiErrorOccurred(FDILink);
				return -1;
			}
			FDILink->RxStatus = FDILink_Frame_Start;
			return 1;
		}
		default:
			fdiErrorOccurred(FDILink);
			return -1;
	}
	return 0;
}

/**
  * @brief Check the data length in the FDILink buffer.
  * @param[in]  FDILink Pointer to the FDILink status structure.
  * @param[in]  len Expected data length.
	*	@return 0 for success, -1 for error.
  */
int fdiCheckBata(FDILink_Status_t* FDILink, uint8_t len)
{
	if (FDILink->BufferIndex != len)
	{
		fdiErrorOccurred(FDILink);
		return -1;
	}
	return 0;
}
/**
  * @brief Apply the effect based on the received FDILink data.
  * @param[in] FDILink Pointer to the FDILink status structure.
  */
void fdiEffectFeature(FDILink_Status_t* FDILink)
{
	fdiDecodeBuffer(FDILink->RxType, FDILink->Buffer);
}
/**
  * @brief Send a FDI link frame.
  * @param[in] type Frame type.
  * @param[in] len Length of the data buffer.	(0-255)
	* @param[in] type Frame type.
	* @param[in] FDILink Pointer to the FDILink status structure.
	*	@param[in] buf Pointer to the data buffer.
	* @return Number of bytes sent.
  */

int fdiComProtocolSend(FDILink_Status_t* FDILink, uint8_t type, uint8_t * buf, int len)
{
	uint8_t buffer[256];
	FDI_ASSERT(len < 248);
	buffer[FDILink_Frame_Start] = FDILink_STX_Flag;
	buffer[FDILink_Frame_CMD] = type;
	buffer[FDILink_Frame_Length] = len;
	buffer[FDILink_Frame_SerialNumber] = FDILink->TxNumber++;
	uint8_t CRC8 = CRC8_Table(buffer, FDILink_Frame_CRC8);
	buffer[FDILink_Frame_CRC8] = CRC8;
	if(len == 0)
	{
		//没有CRC16校验和结束符
		return Serial_Send(buffer,FDILink_Frame_CRC8 + 1);
	}
	else
	{
		uint8_t* buf_data = buffer + FDILink_Frame_Data;
		memcpy(buf_data,buf,len);
		uint16_t CRC16 = CRC16_Table(buf_data, len);
		buffer[FDILink_Frame_CRC16H] = (CRC16 >> 8);
		buffer[FDILink_Frame_CRC16L] = (CRC16 & 0xff);
		buffer[FDILink_Frame_End + len - 1] = FDILink_EDX_Flag;
		return Serial_Send(buffer,FDILink_Frame_End + len);
	}
}

/**
  * @brief Pack the data into the FDILink buffer for transmission.
  * @param[out] buffer Pointer to the buffer to store the packed data.
	* @param[in] FDILink Pointer to the FDILink status structure.
  * @param[in] len Length of the data buffer.	(0-255)
	* @param[in] type Data type.
	* @param[in] buf Pointer to the data buffer.
	*	@param[in] len Length of the data buffer.
	* @return Length of the packed data 
  */
int fdiComBufferTrans(uint8_t* buffer, FDILink_Status_t* FDILink, uint8_t type, void* buf, int len)
{
	FDI_ASSERT(len < 248);
	buffer[FDILink_Frame_Start] = FDILink_STX_Flag;
	buffer[FDILink_Frame_CMD] = type;
	buffer[FDILink_Frame_Length] = len;
	buffer[FDILink_Frame_SerialNumber] = FDILink->TxNumber++;
	uint8_t CRC8 = CRC8_Table(buffer, FDILink_Frame_CRC8);
	buffer[FDILink_Frame_CRC8] = CRC8;

	uint8_t* buf_data = buffer + FDILink_Frame_Data;
	int i;
	//memcpy(buf_data,buf,len);
	for(i = 0;i < len;i++)
	{
		buf_data[i] = ((uint8_t*)buf)[i];
	}
	uint16_t CRC16 = CRC16_Table(buf_data, len);
	buffer[FDILink_Frame_CRC16H] = (CRC16 >> 8);
	buffer[FDILink_Frame_CRC16L] = (CRC16 & 0xff);
	buffer[FDILink_Frame_End + len - 1] = FDILink_EDX_Flag;
	return FDILink_Frame_End + len;
}
/**
  * @brief Pack the data into a FDI link frame.
  * @param[out] buffer Pointer to the output buffer.
  * @param[in] FDILink Pointer to the FDILink status structure.
	* @param[in] type Frame type.
	* @param[in] buf Pointer to the data buffer.
	*	@param[in] len Length of the data buffer.
	* @return  Size of the packed frame.
  */
static inline int fdiPackRecelveData(FDILink_Status_t* FDILink, uint8_t value)
{
	FDI_ASSERT(FDILink->BootStatus == FDILink_Status_Running);
	uint8_t result = fdiRuningReceiveData(FDILink, value);
	if (result == 1)
	{
		fdiEffectFeature(FDILink);
	}
	return result;
}
/**
  * @brief Initialize the FDILink status structure.
	* @return return 0 (always).
  */
int fdiComProtocolInit(FDILink_Status_t* FDILink)
{
	FDILink->BufferIndex = 0;
	FDILink->BootStatus = FDILink_Status_Running;
	FDILink->RxStatus = FDILink_Frame_Start;
	int i;
	for (i = 0; i < 256; i++)
	{
		FDILink->Buffer[i] = 0;
	}
	return 0;
}
/**
  * @brief Receive data into the FDILink buffer.
  * @param[in] FDILink Pointer to the FDILink status structure.
	* @param[in] buf Pointer to the received data buffer.
	*	@param[in] len Length of the received data buffer.
	* @return return 0 (always).
  */

int fdiComProtocolReceive(FDILink_Status_t* FDILink, uint8_t * buf, int len)
{
	int i;
	for(i = 0;i < len;i++)
	{
		fdiPackRecelveData(FDILink, buf[i]);
	}
	return 0;
}


/*!
 *  请求数据帧并解析，ID为要请求的数据帧ID编号。系统会返回当前时刻对应数据输出，如果该帧被设置为固定频率输出，则会持续输出。
 *  使用此指令会自动解析要获得的数据至构建好的结构体中，fdiDecodeBuffer中配置了部分数据包，用户可自行添加需要解析的数据包
 *	Request and analysis data frame.
 *	\param[out]	None
 *	\param[in]	ID - the data frame ID number to be requested.
 *	\return		FDI_NO_ERROR if we have entered configuration mode.
 */
//int fdiGetPacket(uint8_t ID)
//{
//	
//	uint8_t buffer[4];
//	buffer[0] = ID;
//	buffer[1] = 0;
//	buffer[2] = 0;
//	buffer[3] = 0;
//	fdiComBufferTrans(p_buff, &_FDILink, 0xA0, buffer, sizeof(buffer));
//	int i;
//	for(i =0;i<150;i++)
//	{
//		//请求ID发送
////			HAL_Delay(10);
////			HAL_UART_Transmit_IT(&huart1, p_buff, 12);
//		//接收ID数据
//		fdiComProtocolReceive(&_FDILink, USART2_RX_BUF, USART2_RX_STA);
//	}
//	return FDI_NO_ERROR;
//}




///*!
// *  读取GNSS设置参数，paramName为需要获取的参数名称
// *	Read parameters.
// *	\param[out]	None
// *	\param[in]	paramName - Parameter name to be obtained.
// *	\return		FDI_NO_ERROR if we have set the param.
// */
//int fdiGetDgnss(int DGNSS)
//{
//	char* paramName;
//	memset(paramName, 0, strlen(paramName));
//	switch(DGNSS)
//	{
//		case QXWZ_DSK_KEY:
//			memcpy(paramName, "QXWZ_DSK_KEY", strlen("QXWZ_DSK_KEY"));break;
//		case QXWZ_DSK_SECRET:
//			memcpy(paramName, "QXWZ_DSK_SECRET", strlen("QXWZ_DSK_SECRET"));break;
//		case QXWZ_DEV_ID:
//			memcpy(paramName, "QXWZ_DEV_ID", strlen("QXWZ_DEV_ID"));break;
//		case QXWZ_DEV_TYPE:
//			memcpy(paramName, "QXWZ_DEV_TYPE", strlen("QXWZ_DEV_TYPE"));break;
//		case NTRIP_SVR_DOMAIN:
//			memcpy(paramName, "NTRIP_SVR_DOMAIN", strlen("NTRIP_SVR_DOMAIN"));break;
//		case NTRIP_SVR_PORT:
//			memcpy(paramName, "NTRIP_SVR_PORT", strlen("NTRIP_SVR_PORT"));break;
//		case NTRIP_ACCOUNT:
//			memcpy(paramName, "NTRIP_ACCOUNT", strlen("NTRIP_ACCOUNT"));break;
//		case NTRIP_PASSWORD:
//			memcpy(paramName, "NTRIP_PASSWORD", strlen("NTRIP_PASSWORD"));break;
//		case NTRIP_MOUNT:
//			memcpy(paramName, "NTRIP_MOUNT", strlen("NTRIP_MOUNT"));break;
//		case BASE_STATION_SOURCE:
//			memcpy(paramName, "BASE_STATION_SOURCE", strlen("BASE_STATION_SOURCE"));break;
//		case USR_AUTHEMTICATION:
//			memcpy(paramName, "USR_AUTHEMTICATION", strlen("USR_AUTHEMTICATION"));break;
//		case NET_INFO_IMEI:
//			memcpy(paramName, "NET_INFO_IMEI", strlen("NET_INFO_IMEI"));break;
//		case NET_INFO_CCID:
//			memcpy(paramName, "NET_INFO_CCID", strlen("NET_INFO_CCID"));break;
//	}
//	char send_buff[128];
//	sprintf(send_buff, "#fdgnss get %s\r\n", paramName);
//	HAL_UART_Transmit_IT(&huart1,(uint8_t*)send_buff,strlen(send_buff));
//	HAL_Delay(1000);
//	HAL_UART_Transmit_IT(&huart2,(uint8_t*)send_buff,strlen(send_buff));
//	USART2_RX_STA = 0;
//	HAL_Delay(100);

//	HAL_UART_Transmit_IT(&huart1,USART2_RX_BUF,USART2_RX_STA);
//	while(huart1.gState != HAL_UART_STATE_READY){};
//	return FDI_NO_ERROR;
//	HAL_Delay(100);
//}



void fdiResetAll(FDILink_Status_t* FDILink)
{
	FDILink->RxStatus = FDILink_Frame_Start;
	FDILink->RxDataLeft = 0;
	FDILink->RxType = 0;
	FDILink->BufferIndex = 0;
	FDILink->TxNumber = 0;
}




/*************************************关键参数********************************************/
FDILink_VersionData_Packet_t 	VersionData;
FDILink_IMUData_Packet_t 		IMUData;
FDILink_AHRSData_Packet_t 		AHRSData;
FDILink_INSGPSData_Packet_t 	INSGPSData;
System_State_Packet_t 			Sysdata;
Unix_Time_Packet_t 				TimeData;
Formatted_Time_Packet_t 		FTimeData;
Position_Standard_Deviation_Packet_t Posidata;
Velocity_Standard_Deviation_Packet_t Velodata;
Euler_Orientation_Standard_Deviation_Packet_t Euledata;
Quaternion_Orientation_Standard_Deviation_Packet_t Quatdata;
Raw_Sensors_Packet_t 			Raw_Sensor_data;
Raw_GNSS_Packet_t 				Raw_GNSS_data;

Euler_Orientation_Packet_t 			Euler_data;

/*!
**************************************关键函数***********************************************
*	Decode the FDI link packet based on the packet ID.
*	@param[in] PACKET_ID Packet ID (0-255)
*	@param[in] buf Pointer to the buffer containing the packet data.
*	@return None.
*/
void fdiDecodeBuffer(int PACKET_ID,void* buf)
{
	switch(PACKET_ID)
	{
		case FDILINK_VERSIONDATA_PACKET_ID:       //FDILINK_VERSIONDATA 0x39 OK
		{
			memcpy(&VersionData, buf, sizeof(FDILink_VersionData_Packet_t));
			break;
		}
		case FDILINK_IMUDATA_PACKET_ID:  		 //MSG_IMU 0x40 OK
		{
			memcpy(&IMUData, buf, sizeof(FDILink_IMUData_Packet_t));
			break;
		}
		case FDILINK_AHRSDATA_PACKET_ID:  	 //MSG_AHRS 0x41 OK
		{
			memcpy(&AHRSData, buf, sizeof(FDILink_AHRSData_Packet_t));
			break;
		}
		case FDILINK_INSGPSDATA_PACKET_ID:   //MSG_INS/GPS 0x42 OK
		{
			memcpy(&INSGPSData, buf, sizeof(FDILink_INSGPSData_Packet_t));
			break;
		}
		case System_State_Packet_ID: 				 //MSG_SYS_STATE 0x50 OK
		{
			memcpy(&Sysdata,buf,sizeof(System_State_Packet_t));
			break;
		}
		case Unix_Time_Packet_ID:   		     //UNIX_TIME 0X51 OK
		{
			memcpy(&TimeData,buf,sizeof(Unix_Time_Packet_t));
			break;
		}
		case Formatted_Time_Packet_ID:       //FORMATTED_TIME 0X52 OK
		{
			memcpy(&FTimeData,buf,sizeof(Formatted_Time_Packet_t));
			break;
		}	
		case Position_Standard_Deviation_Packet_ID:   //POSITION_STANDARD OX54 OK
		{
			memcpy(&Posidata,buf,sizeof(Position_Standard_Deviation_Packet_t));
			break;
		}
		case Velocity_Standard_Deviation_Packet_ID:   //VELOCITY_STANDARD OX55 OK
		{
			memcpy(&Velodata,buf,sizeof(Velocity_Standard_Deviation_Packet_t));
			break;
		}
		case Euler_Orientation_Standard_Deviation_Packet_ID:   //EULER_ORIENTATION_STANDARD OX56 OK
		{
			memcpy(&Euledata,buf,sizeof(Euler_Orientation_Standard_Deviation_Packet_t));
			break;
		}
		case Quaternion_Orientation_Standard_Deviation_Packet_ID:   //QUATERNION_ORIENTATION_STANDARD OX57
		{
			memcpy(&Quatdata,buf,sizeof(Quaternion_Orientation_Standard_Deviation_Packet_t));
			break;
		}
		case Raw_Sensors_Packet_ID:   			 //RAW_SENSORS OX58
		{
			memcpy(&Raw_Sensor_data,buf,sizeof(Raw_Sensors_Packet_t));
			break;
		}		
		case Raw_GNSS_Packet_ID:             //MSG_RAW_GNSS 0x59
		{
			memcpy(&Raw_GNSS_data,buf,sizeof(Raw_GNSS_Packet_t));
						break;
		}
		case Satellites_Packet_ID:           //SATELLITES 0x5A
		{
			Satellites_Packet_t data;
			memcpy(&data,buf,sizeof(Satellites_Packet_t));
						break;
		}
		case Detailed_Satellites_Packet_ID:  //DETAILED_SATELLITES 0x5B
		{
			Detailed_Satellites_Packet_t data;
			memcpy(&data,buf,sizeof(Detailed_Satellites_Packet_t));
						break;
		}
		case Geodetic_Position_Packet_ID:    //GEODETIC_POSITION 0x5C
		{
			Geodetic_Position_Packet_t data;
			memcpy(&data,buf,sizeof(Geodetic_Position_Packet_t));
						break;
		}
		case ECEF_Position_Packet_ID:        //GEODETIC_POSITION 0x5D
		{
			ECEF_Position_Packet_t data;
			memcpy(&data,buf,sizeof(ECEF_Position_Packet_t));
						break;
		}
		case UTM_Position_Packet_ID:         //UIM_POSITION 0x5E
		{
			UTM_Position_Packet_t data;
			memcpy(&data,buf,sizeof(UTM_Position_Packet_t));
						break;
		}
		case NED_Velocity_Packet_ID:         //NED_VELOCITY 0x5F
		{
			NED_Velocity_Packet_t data;
			memcpy(&data,buf,sizeof(NED_Velocity_Packet_t));
						break;
		}
		case Body_Velocity_Packet_ID:        //BODY_VELOCITY 0x60
		{
			Body_Velocity_Packet_t data;
			memcpy(&data,buf,sizeof(Body_Velocity_Packet_t));
						break;
		}
		case Acceleration_Packet_ID:         //ACCELERATION 0x61
		{
			Acceleration_Packet_t data;
			memcpy(&data,buf,sizeof(Acceleration_Packet_t));
						break;
		}		
		case Body_Acceleration_Packet_ID:    //BODT_ACCELERATION 0x62
		{
			Body_Acceleration_Packet_t data;
			memcpy(&data,buf,sizeof(Body_Acceleration_Packet_t));
						break;
		}		
		case Euler_Orientation_Packet_ID:    //EULER_ORIENTATION 0x63
		{
			memcpy(&Euler_data,buf,sizeof(Euler_Orientation_Packet_t));
						break;
		}
		case Quaternion_Orientation_Packet_ID://QUATERNOION_ORIENTATION 0x64
		{
			Quaternion_Orientation_Packet_t data;
			memcpy(&data,buf,sizeof(Quaternion_Orientation_Packet_t));
						break;
		}
		case DCM_Orientation_Packet_ID:      //DCM_ORIENTATION 0x65
		{
			DCM_Orientation_Packet_t data;
			memcpy(&data,buf,sizeof(DCM_Orientation_Packet_t));
						break;
		}
		case Angular_Velocity_Packet_ID:     //ANGUAR_VELOCITY 0x66
		{
			Angular_Velocity_Packet_t data;
			memcpy(&data,buf,sizeof(Angular_Velocity_Packet_t));
						break;
		}
		case Angular_Acceleration_Packet_ID: //ANGUAR_ACCELERATION 0x67
		{
			Angular_Acceleration_Packet_t data;
			memcpy(&data,buf,sizeof(Angular_Acceleration_Packet_t));
						break;
		}
		case External_Position_And_Velocity_Packet_ID://EXTERNAL_POSITION_AND_VELOCITY 0x68
		{
			External_Position_And_Velocity_Packet_t data;
			memcpy(&data,buf,sizeof(External_Position_And_Velocity_Packet_t));
						break;
		}
		case External_Position_Packet_ID:    //EXTERNAL_POSITION 0x69
		{
			External_Position_Packet_t data;
			memcpy(&data,buf,sizeof(External_Position_Packet_t));
						break;
		}
		case External_Velocity_Packet_ID:    //EXTERNAL_VELOCITY 0x6A
		{
			External_Velocity_Packet_t data;
			memcpy(&data,buf,sizeof(External_Velocity_Packet_t));
						break;
		}
		case External_Body_Velocity_Packet_ID://EXTERNAL_BODT_VELOCITY 0x6B
		{
			External_Body_Velocity_Packet_t data;
			memcpy(&data,buf,sizeof(External_Body_Velocity_Packet_t));
						break;
		}
		case External_Heading_Packet_ID:     //EXTERNAL_HEADING 0x6C
		{
			External_Heading_Packet_t data;
			memcpy(&data,buf,sizeof(External_Heading_Packet_t));
						break;
		}
		case External_Time_Packet_ID:        //EXTERNAL_TIME 0x70
		{
			External_Time_Packet_t data;
			memcpy(&data,buf,sizeof(External_Time_Packet_t));
						break;
		}
		case External_Depth_Packet_ID:       //EXTERNAL_DEPTH 0x71
		{
			External_Depth_Packet_t data;
			memcpy(&data,buf,sizeof(External_Depth_Packet_t));
						break;
		}
		case External_Pitot_Pressure_Packet_ID://EXTERNAL_DEPTH 072
		{
			External_Pitot_Pressure_Packet_t data;
			memcpy(&data,buf,sizeof(External_Pitot_Pressure_Packet_t));
						break;
		}
		case External_Air_Data_Packet_ID:    //EXTERNAL_AIE_DATA 0x69
		{
			External_Air_Data_Packet_t data;
			memcpy(&data,buf,sizeof(External_Air_Data_Packet_t));
						break;
		}
		case External_LIDAR_Packet_ID:       //EXTERNAL_LIDAR_DATA 0x91
		{
			External_LIDAR_Packet_t data;
			memcpy(&data,buf,sizeof(External_LIDAR_Packet_t));
						break;
		}		
		case External_Odom_Data_Packet_ID:   //EXTERNAL_ODOM_DATA 0x90
		{
			External_Odom_Data_Packet_t data;
			memcpy(&data,buf,sizeof(External_Odom_Data_Packet_t));
						break;
		}
		case External_SLAM1_Packet_ID:       //EXTERNAL_SALAM1 0x91
		{
			External_SLAM1_Packet_t data;
			memcpy(&data,buf,sizeof(External_SLAM1_Packet_t));
						break;
		}
		case External_SLAM2_Packet_ID:       //EXTERNAL_SALAM2 0x92
		{
			External_SLAM2_Packet_t data;
			memcpy(&data,buf,sizeof(External_SLAM2_Packet_t));
						break;
		}
		case Running_Time_Packet_ID:         //RUNNING_TIME_PACKET 0x6D
		{
			Running_Time_Packet_t data;
			memcpy(&data,buf,sizeof(Running_Time_Packet_t));
						break;
		}
		case Local_Magnetic_Field_Packet_ID: //LOCAL 0x6E
		{
			Local_Magnetic_Field_Packet_t data;
			memcpy(&data,buf,sizeof(Local_Magnetic_Field_Packet_t));
						break;
		}
		case Odometer_State_Packet_ID:       //ODOMETER 0x6F
		{
			Odometer_State_Packet_t data;
			memcpy(&data,buf,sizeof(Odometer_State_Packet_t));
						break;
		}
		case Geoid_Height_Packet_ID:         //GEOID_HEIGHT 0x72
		{
			Geoid_Height_Packet_t data;
			memcpy(&data,buf,sizeof(Geoid_Height_Packet_t));
						break;
		}
		case RTCM_Corrections_Packet_ID:     //RTCM_CORRECTIONS 0x73
		{
			RTCM_Corrections_Packet_t data;
			memcpy(&data,buf,sizeof(RTCM_Corrections_Packet_t));
						break;
		}
		case Wind_Packet_ID:                 //WIND 0x75
		{
			Wind_Packet_t data;
			memcpy(&data,buf,sizeof(Wind_Packet_t));
						break;
		}
		case Heave_Packet_ID:                //HEAVE 0x76
		{
			Heave_Packet_t data;
			memcpy(&data,buf,sizeof(Heave_Packet_t));
						break;
		}
		case Raw_Satellite_Data_Packet_ID:   //RAW_SATELLITE 0x77
		{
			Raw_Satellite_Data_Packet_t data;
			memcpy(&data,buf,sizeof(Raw_Satellite_Data_Packet_t));
						break;
		}
		case GNSS_DUAL_ANT_Data_Packet_ID:   //GNSS_DUAL 0x78
		{
			GNSS_DUAL_ANT_Data_Packet_t data;
			memcpy(&data,buf,sizeof(GNSS_DUAL_ANT_Data_Packet_t));
						break;
		}
		case Gimbal_State_Packet_ID:         //GIMBAL_STATE 0x7A
		{
			Gimbal_State_Packet_t data;
			memcpy(&data,buf,sizeof(Gimbal_State_Packet_t));
						break;
		}		
		case Automotive_Packet_ID:           //AUTOMOTIVE 0x7B
		{
			Automotive_Packet_t data;
			memcpy(&data,buf,sizeof(Automotive_Packet_t));
						break;
		}
		case Installation_Alignment_Packet_ID://INSTALLATION_ALIGNMENT 0x80
		{
			Installation_Alignment_Packet_t data;
			memcpy(&data,buf,sizeof(Installation_Alignment_Packet_t));
						break;
		}
		case Filter_Options_Packet_ID:       //FILTER_OPTIONS 0x81
		{
			Filter_Options_Packet_t data;
			memcpy(&data,buf,sizeof(Filter_Options_Packet_t));
						break;
		}
		case GPIO_Configuration_Packet_ID:   //GPIO_CONFIG Ox82
		{
			GPIO_Configuration_Packet_t data;
			memcpy(&data,buf,sizeof(GPIO_Configuration_Packet_t));
						break;
		}		
		case Magnetic_Calibration_Values_Packet_ID://MAGNETIC_CAIL 0x83
		{
			Magnetic_Calibration_Values_Packet_t data;
			memcpy(&data,buf,sizeof(Magnetic_Calibration_Values_Packet_t));
						break;
		}		
		case Magnetic_Calibration_Configuration_Packet_ID://MAGNETIC_CAIL_CONFIG 0x84
		{
			Magnetic_Calibration_Configuration_Packet_t data;
			memcpy(&data,buf,sizeof(Magnetic_Calibration_Configuration_Packet_t));
						break;
		}			
		case Magnetic_Calibration_Status_Packet_ID://MAGNETIC_CAIL_STATUS 0x85
		{
			Magnetic_Calibration_Status_Packet_t data;
			memcpy(&data,buf,sizeof(Magnetic_Calibration_Status_Packet_t));
						break;
		}			
		case Odometer_Configuration_Packet_ID://ODOMETER_CONFIG 0x86
		{
			Odometer_Configuration_Packet_t data;
			memcpy(&data,buf,sizeof(Odometer_Configuration_Packet_t));
						break;
		}		
		case Set_Zero_Orientation_Alignment_Packet_ID://SET_ZERO 0x87
		{
			Set_Zero_Orientation_Alignment_Packet_t data;
			memcpy(&data,buf,sizeof(Set_Zero_Orientation_Alignment_Packet_t));
						break;
		}		
		case Reference_Point_Offsets_Packet_ID://REFERENCE_POINT 0x88
		{
			Reference_Point_Offsets_Packet_t data;
			memcpy(&data,buf,sizeof(Reference_Point_Offsets_Packet_t));
						break;
		}			
		case User_Data_Packet_ID:             //USER_DATA 0x8A
		{
			User_Data_Packet_t data;
			memcpy(&data,buf,sizeof(User_Data_Packet_t));
						break;
		}			
		case Baud_Rates_Packet_ID:            //BAUD_SATES 0xA0
		{
			Baud_Rates_Packet_t data;
			memcpy(&data,buf,sizeof(Baud_Rates_Packet_t));
						break;
		}		
		case Sensor_Ranges_Packet_ID:         //SENSOR_RANGES 0xA1
		{
			Sensor_Ranges_Packet_t data;
			memcpy(&data,buf,sizeof(Sensor_Ranges_Packet_t));
						break;
		}		
		case GPIO_Output_Configuration_Packet_ID://GPIO_OUTPUT_CONFIG 0xA2
		{
			GPIO_Output_Configuration_Packet_t data;
			memcpy(&data,buf,sizeof(GPIO_Output_Configuration_Packet_t));
						break;
		}		
		case GPIO_Input_Configuration_Packet_ID: //GPIO_INPUT_CONFIG 0xA3
		{
			GPIO_Input_Configuration_Packet_t data;
			memcpy(&data,buf,sizeof(GPIO_Input_Configuration_Packet_t));
						break;
		}			
		case Dual_Antenna_Configuration_Packet_ID://DUAL_ANTENNA_CONFIG 0xA4
		{
			Dual_Antenna_Configuration_Packet_t data;
			memcpy(&data,buf,sizeof(Dual_Antenna_Configuration_Packet_t));
						break;
		}
		
	}
	
}



