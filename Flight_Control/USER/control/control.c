#include "string.h"
#include "math.h"
#include "sys.h"
#include "includes.h"					//ucos 使用	  

#include "control.h"
#include "usart.h"
#include "led.h"
#include "ANO_TC.h"
#include "spl06_001.h"
#include "ELRS.h"
#include "IMU_UART.h"
#include "fdilink_decode.h"
#include "FDILink.h"



//*******************************************姿态解算**********************************************//
//*******************根据三轴加速度和三轴陀螺仪、三轴磁力计，解算出四元数和欧拉角*********************//
//---------------------------------------------------------------------------------------------------
// Definitions
#define T			0.025f		//这两个的准确性直接影响了角度反应的快慢
#define halfT		0.0125f
#define twoKpDef	5.0f	// proportional gain
#define twoKiDef	0.2f	// 2 * integral gain
//---------------------------------------------------------------------------------------------------
// Variable definitions
volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
volatile float w_last[3]; 	// 中间计算结果// 上一时刻的角速度
volatile float AHRS_Roll, AHRS_Pitc, AHRS_Yawh;  // 输出结果 // 偏航角，俯仰角，翻滚角   单位：度
volatile float C_nb_11, C_nb_12, C_nb_13, C_nb_21, C_nb_22, C_nb_23, C_nb_31, C_nb_32, C_nb_33;		// 坐标系旋转矩阵
//---------------------------------------------------------------------------------------------------



//*******************************************PID控制**********************************************//
//***************************定义角度环PID、角速度环PID结构体并初始化参数****************************//
//---------------------------------------------------------------------------------------------------
PID_TYPE 	PID_ROL_Angle;	//角度环PID
PID_TYPE 	PID_PIT_Angle;
PID_TYPE 	PID_YAW_Angle;

PID_TYPE 	PID_ROL_Rate;	//角速度环PID
PID_TYPE 	PID_PIT_Rate;
PID_TYPE 	PID_YAW_Rate;

PID_TYPE 	PID_Server_Posi;	//舵机PID
//---------------------------------------------------------------------------------------------------



//********************************************滤波器***********************************************//
//********************************定义卡尔曼滤波器结构体并初始化参数*********************************//
//---------------------------------------------------------------------------------------------------
KFP KFP_height = {0.02, 0, 0, 0, 0.001, 0.543};
//---------------------------------------------------------------------------------------------------








//*************************************************************************************************//
//*******************************************姿态解算**********************************************//
//*************************************************************************************************//
// @功能：快速获得开方的倒数  网上有说明
float invSqrt(float number)		
{
    long i;
    float x, y;
    const float f = 1.5f;

    x = number * 0.5f;
    y = number;
    i = *((long *)&y);
    i = 0x5f375a86 - (i >> 1);
    y = *((float *)&i);
    y = y * (f - (x * y * y));
    return y;
}


// 根据三轴加速度和三轴陀螺仪、三轴磁力计，解算出四元数和欧拉角		// 单位：m/s^2   rad/s 
void MahonyAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) 
{
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	
	float q_last[4]; 	// 因为计算机计算是串行的，算了q[0]再直接算q[1]会用到最新的q[0],不能这样！所以要另存！整成并行的
	float w_new[3];  	// 采样时刻的角速度
	float k10 = 0.0f, k11 = 0.0f, k12 = 0.0f, k13 = 0.0f; 	// 龙格库塔算法用到的中间值
	float k20 = 0.0f, k21 = 0.0f, k22 = 0.0f, k23 = 0.0f;
	float k30 = 0.0f, k31 = 0.0f, k32 = 0.0f, k33 = 0.0f;
	float k40 = 0.0f, k41 = 0.0f, k42 = 0.0f, k43 = 0.0f;
 
	
//	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
//	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
//		MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
//		return;
//	}
 
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
 
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     
 
		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   
 
        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;   
 
        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
 
		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
 
 
		//-------------------------------------------------------------------------------------------------
        w_new[0] = gx; // 陀螺仪角速度 //为弧度！！！！！！！！！！！！！！！！！！
        w_new[1] = gy;
        w_new[2] = gz;

        w_last[0] = 0.5f * (w_new[0] + w_last[0]); // 梯形法求角速度均值
        w_last[1] = 0.5f * (w_new[1] + w_last[1]);
        w_last[2] = 0.5f * (w_new[2] + w_last[2]);
		//-------------------------------------------------------------------------------------------------
 
 
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex;	// integral error scaled by Ki
			integralFBy += twoKi * halfey;
			integralFBz += twoKi * halfez;
			w_last[0] += integralFBx;	// apply integral feedback
			w_last[1] += integralFBy;
			w_last[2] += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}
 
		// Apply proportional feedback
		w_last[0] += twoKp * halfex;
		w_last[1] += twoKp * halfey;
		w_last[2] += twoKp * halfez;
	}
	
	//-------------------------------------------------------------------------------------------------
	q_last[0] = q0; // 上一时刻的四元数
	q_last[1] = q1;
	q_last[2] = q2;
	q_last[3] = q3;
	
	k10 = 0.5f * (-w_last[0] * q_last[1] - w_last[1] * q_last[2] - w_last[2] * q_last[3]); 		// 四阶龙格库塔方法更新四元素
	k11 = 0.5f * (w_last[0] * q_last[0] + w_last[2] * q_last[2] - w_last[1] * q_last[3]);
	k12 = 0.5f * (w_last[1] * q_last[0] - w_last[2] * q_last[1] + w_last[0] * q_last[3]);
	k13 = 0.5f * (w_last[2] * q_last[0] + w_last[1] * q_last[1] - w_last[0] * q_last[2]);

	k20 = 0.5f * (halfT * (q_last[0] + halfT * k10) + (halfT - w_last[0]) * (q_last[1] + halfT * k11) + (halfT - w_last[1]) * (q_last[2] + halfT * k12) + (halfT - w_last[2]) * (q_last[3] + halfT * k13));
	k21 = 0.5f * ((halfT + w_last[0]) * (q_last[0] + halfT * k10) + halfT * (q_last[1] + halfT * k11) + (halfT + w_last[2]) * (q_last[2] + halfT * k12) + (halfT - w_last[1]) * (q_last[3] + halfT * k13));
	k22 = 0.5f * ((halfT + w_last[1]) * (q_last[0] + halfT * k10) + (halfT - w_last[2]) * (q_last[1] + halfT * k11) + halfT * (q_last[2] + halfT * k12) + (halfT + w_last[0]) * (q_last[3] + halfT * k13));
	k23 = 0.5f * ((halfT + w_last[2]) * (q_last[0] + halfT * k10) + (halfT + w_last[1]) * (q_last[1] + halfT * k11) + (halfT - w_last[0]) * (q_last[2] + halfT * k12) + halfT * (q_last[3] + halfT * k13));

	k30 = 0.5f * (halfT * (q_last[0] + halfT * k20) + (halfT - w_last[0]) * (q_last[1] + halfT * k21) + (halfT - w_last[1]) * (q_last[2] + halfT * k22) + (halfT - w_last[2]) * (q_last[3] + halfT * k23));
	k31 = 0.5f * ((halfT + w_last[0]) * (q_last[0] + halfT * k20) + halfT * (q_last[1] + halfT * k21) + (halfT + w_last[2]) * (q_last[2] + halfT * k22) + (halfT - w_last[1]) * (q_last[3] + halfT * k23));
	k32 = 0.5f * ((halfT + w_last[1]) * (q_last[0] + halfT * k20) + (halfT - w_last[2]) * (q_last[1] + halfT * k21) + halfT * (q_last[2] + halfT * k22) + (halfT + w_last[0]) * (q_last[3] + halfT * k23));
	k33 = 0.5f * ((halfT + w_last[2]) * (q_last[0] + halfT * k20) + (halfT + w_last[1]) * (q_last[1] + halfT * k21) + (halfT - w_last[0]) * (q_last[2] + halfT * k22) + halfT * (q_last[3] + halfT * k23));

	k40 = 0.5f * (T * (q_last[0] + T * k30) + (T - w_last[0]) * (q_last[1] + T * k31) + (T - w_last[1]) * (q_last[2] + T * k32) + (T - w_last[2]) * (q_last[3] + T * k33));
	k41 = 0.5f * ((T + w_last[0]) * (q_last[0] + T * k30) + T * (q_last[1] + T * k31) + (T + w_last[2]) * (q_last[2] + T * k32) + (T - w_last[1]) * (q_last[3] + T * k33));
	k42 = 0.5f * ((T + w_last[1]) * (q_last[0] + T * k30) + (T - w_last[2]) * (q_last[1] + T * k31) + T * (q_last[2] + T * k32) + (T + w_last[0]) * (q_last[3] + T * k33));
	k43 = 0.5f * ((T + w_last[2]) * (q_last[0] + T * k30) + (T + w_last[1]) * (q_last[1] + T * k31) + (T - w_last[0]) * (q_last[2] + T * k32) + T * (q_last[3] + T * k33));

	q0 = q_last[0] + T / 6.0f * (k10 + 2 * k20 + 2 * k30 + k40);
	q1 = q_last[1] + T / 6.0f * (k11 + 2 * k21 + 2 * k31 + k41);
	q2 = q_last[2] + T / 6.0f * (k12 + 2 * k22 + 2 * k32 + k42);
	q3 = q_last[3] + T / 6.0f * (k13 + 2 * k23 + 2 * k33 + k43);
	
	
	w_last[0] = w_new[0]; // 记录这一时刻的采样值，留下一时刻用
	w_last[1] = w_new[1];
	w_last[2] = w_new[2];
	//-------------------------------------------------------------------------------------------------
	
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	
	AHRS_Pitc = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180.0 / PI;                                     // pitch ,转换为度数
	AHRS_Roll = atan2(2 * (q2 * q3 + q0 * q1), -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180.0 / PI; 		// roll
	AHRS_Yawh = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180.0 / PI;
	

	//-------------------------------------------------------------------------------------------------	
	C_nb_11 = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
	C_nb_12 = 2 * (q1 * q2 - q0 * q3);
	C_nb_13 = 2 * (q1 * q3 + q0 * q2);
	C_nb_21 = 2 * (q1 * q2 + q0 * q3);
	C_nb_22 = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
	C_nb_23 = 2 * (q2 * q3 - q0 * q1);
	C_nb_31 = 2 * (q1 * q3 - q0 * q2);
	C_nb_32 = 2 * (q2 * q3 + q0 * q1);
	C_nb_33 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

//	ACCX_n = C_nb_11 * ax + C_nb_12 * ay + C_nb_13 * az; // ACCX_n是在地理坐标系上的加速度，ax是机体坐标系的
//	ACCY_n = C_nb_21 * ax + C_nb_22 * ay + C_nb_23 * az; // 单位是m/s^2
//	ACCZ_n = C_nb_31 * ax + C_nb_32 * ay + C_nb_33 * az;
	//-------------------------------------------------------------------------------------------------
	
}
 

// 仅根据三轴加速度和三轴陀螺仪，解算出四元数和欧拉角
void MahonyAHRSupdateIMU(float ax, float ay, float az, float gx, float gy, float gz) 
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	
	float q_last[4]; 	// 因为计算机计算是串行的，算了q[0]再直接算q[1]会用到最新的q[0],不能这样！所以要另存！整成并行的
	float w_new[3];  	// 采样时刻的角速度
	float k10 = 0.0f, k11 = 0.0f, k12 = 0.0f, k13 = 0.0f; 	// 龙格库塔算法用到的中间值
	float k20 = 0.0f, k21 = 0.0f, k22 = 0.0f, k23 = 0.0f;
	float k30 = 0.0f, k31 = 0.0f, k32 = 0.0f, k33 = 0.0f;
	float k40 = 0.0f, k41 = 0.0f, k42 = 0.0f, k43 = 0.0f;
	
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
 
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        
 
		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = 2 * (q1 * q3 - q0 * q2);
		halfvy = 2 * (q0 * q1 + q2 * q3);
		halfvz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);
		
		
		//-------------------------------------------------------------------------------------------------
        w_new[0] = gx; // 陀螺仪角速度 //为弧度！！！！！！！！！！！！！！！！！！
        w_new[1] = gy;
        w_new[2] = gz;

        w_last[0] = 0.5f * (w_new[0] + w_last[0]); // 梯形法求角速度均值
        w_last[1] = 0.5f * (w_new[1] + w_last[1]);
        w_last[2] = 0.5f * (w_new[2] + w_last[2]);
		//-------------------------------------------------------------------------------------------------
		
 
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex;	// integral error scaled by Ki
			integralFBy += twoKi * halfey;
			integralFBz += twoKi * halfez;
			w_last[0] += integralFBx;	// apply integral feedback
			w_last[1] += integralFBy;
			w_last[2] += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}
 
		// Apply proportional feedback
		w_last[0] += twoKp * halfex;
		w_last[1] += twoKp * halfey;
		w_last[2] += twoKp * halfez;
	}
	
	//-------------------------------------------------------------------------------------------------
	q_last[0] = q0; // 上一时刻的四元数
	q_last[1] = q1;
	q_last[2] = q2;
	q_last[3] = q3;
	
	k10 = 0.5f * (-w_last[0] * q_last[1] - w_last[1] * q_last[2] - w_last[2] * q_last[3]); 		// 四阶龙格库塔方法更新四元素
	k11 = 0.5f * (w_last[0] * q_last[0] + w_last[2] * q_last[2] - w_last[1] * q_last[3]);
	k12 = 0.5f * (w_last[1] * q_last[0] - w_last[2] * q_last[1] + w_last[0] * q_last[3]);
	k13 = 0.5f * (w_last[2] * q_last[0] + w_last[1] * q_last[1] - w_last[0] * q_last[2]);

	k20 = 0.5f * (halfT * (q_last[0] + halfT * k10) + (halfT - w_last[0]) * (q_last[1] + halfT * k11) + (halfT - w_last[1]) * (q_last[2] + halfT * k12) + (halfT - w_last[2]) * (q_last[3] + halfT * k13));
	k21 = 0.5f * ((halfT + w_last[0]) * (q_last[0] + halfT * k10) + halfT * (q_last[1] + halfT * k11) + (halfT + w_last[2]) * (q_last[2] + halfT * k12) + (halfT - w_last[1]) * (q_last[3] + halfT * k13));
	k22 = 0.5f * ((halfT + w_last[1]) * (q_last[0] + halfT * k10) + (halfT - w_last[2]) * (q_last[1] + halfT * k11) + halfT * (q_last[2] + halfT * k12) + (halfT + w_last[0]) * (q_last[3] + halfT * k13));
	k23 = 0.5f * ((halfT + w_last[2]) * (q_last[0] + halfT * k10) + (halfT + w_last[1]) * (q_last[1] + halfT * k11) + (halfT - w_last[0]) * (q_last[2] + halfT * k12) + halfT * (q_last[3] + halfT * k13));

	k30 = 0.5f * (halfT * (q_last[0] + halfT * k20) + (halfT - w_last[0]) * (q_last[1] + halfT * k21) + (halfT - w_last[1]) * (q_last[2] + halfT * k22) + (halfT - w_last[2]) * (q_last[3] + halfT * k23));
	k31 = 0.5f * ((halfT + w_last[0]) * (q_last[0] + halfT * k20) + halfT * (q_last[1] + halfT * k21) + (halfT + w_last[2]) * (q_last[2] + halfT * k22) + (halfT - w_last[1]) * (q_last[3] + halfT * k23));
	k32 = 0.5f * ((halfT + w_last[1]) * (q_last[0] + halfT * k20) + (halfT - w_last[2]) * (q_last[1] + halfT * k21) + halfT * (q_last[2] + halfT * k22) + (halfT + w_last[0]) * (q_last[3] + halfT * k23));
	k33 = 0.5f * ((halfT + w_last[2]) * (q_last[0] + halfT * k20) + (halfT + w_last[1]) * (q_last[1] + halfT * k21) + (halfT - w_last[0]) * (q_last[2] + halfT * k22) + halfT * (q_last[3] + halfT * k23));

	k40 = 0.5f * (T * (q_last[0] + T * k30) + (T - w_last[0]) * (q_last[1] + T * k31) + (T - w_last[1]) * (q_last[2] + T * k32) + (T - w_last[2]) * (q_last[3] + T * k33));
	k41 = 0.5f * ((T + w_last[0]) * (q_last[0] + T * k30) + T * (q_last[1] + T * k31) + (T + w_last[2]) * (q_last[2] + T * k32) + (T - w_last[1]) * (q_last[3] + T * k33));
	k42 = 0.5f * ((T + w_last[1]) * (q_last[0] + T * k30) + (T - w_last[2]) * (q_last[1] + T * k31) + T * (q_last[2] + T * k32) + (T + w_last[0]) * (q_last[3] + T * k33));
	k43 = 0.5f * ((T + w_last[2]) * (q_last[0] + T * k30) + (T + w_last[1]) * (q_last[1] + T * k31) + (T - w_last[0]) * (q_last[2] + T * k32) + T * (q_last[3] + T * k33));

	q0 = q_last[0] + T / 6.0f * (k10 + 2 * k20 + 2 * k30 + k40);
	q1 = q_last[1] + T / 6.0f * (k11 + 2 * k21 + 2 * k31 + k41);
	q2 = q_last[2] + T / 6.0f * (k12 + 2 * k22 + 2 * k32 + k42);
	q3 = q_last[3] + T / 6.0f * (k13 + 2 * k23 + 2 * k33 + k43);
	
	
	w_last[0] = w_new[0]; // 记录这一时刻的采样值，留下一时刻用
	w_last[1] = w_new[1];
	w_last[2] = w_new[2];
	//-------------------------------------------------------------------------------------------------
	
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	
	AHRS_Pitc = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180.0f / PI;                                     // pitch ,转换为度数
	AHRS_Roll = atan2(2 * (q2 * q3 + q0 * q1), -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180.0f / PI; 		// roll
	AHRS_Yawh = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180.0f / PI;
	
	
	//-------------------------------------------------------------------------------------------------
	C_nb_11 = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
	C_nb_12 = 2 * (q1 * q2 - q0 * q3);
	C_nb_13 = 2 * (q1 * q3 + q0 * q2);
	C_nb_21 = 2 * (q1 * q2 + q0 * q3);
	C_nb_22 = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
	C_nb_23 = 2 * (q2 * q3 - q0 * q1);
	C_nb_31 = 2 * (q1 * q3 - q0 * q2);
	C_nb_32 = 2 * (q2 * q3 + q0 * q1);
	C_nb_33 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

//	ACCX_n = C_nb_11 * ax + C_nb_12 * ay + C_nb_13 * az; // ACCX_n是在地理坐标系上的加速度，ax是机体坐标系的
//	ACCY_n = C_nb_21 * ax + C_nb_22 * ay + C_nb_23 * az; // 单位是m/s^2
//	ACCZ_n = C_nb_31 * ax + C_nb_32 * ay + C_nb_33 * az;
	//-------------------------------------------------------------------------------------------------
	
}




//*************************************************************************************************//
//*********************************************PID控制**********************************************//
//*************************************************************************************************//
// 位置式PID计算
void PID_Position_Cal(PID_TYPE* PID, float target, float measure)
{
	//int dt =0:	//采样时间(也就是扫描时间10ms)
	PID->Error = target - measure;				//误差
	PID->Differ = PID->Error - PID->PreError;	//微分量
	
	if(aircraft_take_off)		// 起飞后才开始积分
	{
			if(measure > PID->I_limit || measure < -PID->I_limit)		//积分分离
			{
				PID->I_limit_flag = 0;
			}
			else
			{	
				PID->I_limit_flag = 1;
				PID->Integral += PID->Error;		//误差积分
				if(PID->Integral > PID->I_range)	//积分限幅
					PID->Integral = PID->I_range;
				if(PID->Integral < -PID->I_range)	//积分限幅
					PID->Integral = -PID->I_range;
				
			}
	}else
	{
		PID->Integral = 0;
	}
	
	PID->Pout = PID->P * PID->Error;	//比例控制输出
	PID->Dout = PID->D * PID->Differ;	//微分控制输出
	PID->Iout = PID->I * PID->Integral * PID->I_limit_flag;	//积分控制输出

	PID->OutPut = PID->Pout + PID->Iout + PID->Dout;	//PID控制总输出
	PID->PreError = PID->Error;		//前一个误差值
}


// 舵机位置式PID计算
void PID_Posi_Server_Cal(PID_TYPE* PID, float target, float measure)
{
	
	PID->Error = target - measure;				//误差
	PID->Differ = PID->Error - PID->PreError;	//微分量
	
	
	if(measure > PID->I_limit || measure < -PID->I_limit)		//积分分离
	{
		PID->I_limit_flag = 0;
	}
	else
	{	
		PID->I_limit_flag = 1;
		PID->Integral += PID->Error;		//误差积分
		if(PID->Integral > PID->I_range)	//积分限幅
			PID->Integral = PID->I_range;
		if(PID->Integral < -PID->I_range)	//积分限幅
			PID->Integral = -PID->I_range;
		
	}
	
	PID->Pout = PID->P * PID->Error;	//比例控制输出
	PID->Dout = PID->D * PID->Differ;	//微分控制输出
	PID->Iout = PID->I * PID->Integral * PID->I_limit_flag;	//积分控制输出

	PID->OutPut = PID->Pout + PID->Iout + PID->Dout;	//PID控制总输出
	PID->PreError = PID->Error;		//前一个误差值
	
}



// PID参数初始化
void PID_Param_Init(void)
{
	// ROLL
	PID_ROL_Angle.P = 2;
	PID_ROL_Angle.I = 0.05;
	PID_ROL_Angle.I_limit_flag = 0;
	PID_ROL_Angle.I_limit = 35;
	PID_ROL_Angle.I_range = 200;
	
	PID_ROL_Rate.P = 2;
	PID_ROL_Rate.I = 0.05;
	PID_ROL_Rate.I_limit_flag = 0;
	PID_ROL_Rate.I_limit = 150;
	PID_ROL_Rate.I_range = 200;
	
	// PITCH
	PID_PIT_Angle.P = 2;
	PID_PIT_Angle.I = 0.05;
	PID_PIT_Angle.I_limit_flag = 0;
	PID_PIT_Angle.I_limit = 35;
	PID_PIT_Angle.I_range = 200;
	
	PID_PIT_Rate.P = 2;
	PID_PIT_Rate.I = 0.05;
	PID_PIT_Rate.I_limit_flag = 0;
	PID_PIT_Rate.I_limit = 150;
	PID_PIT_Rate.I_range = 200;
	
	// YAW
	PID_YAW_Angle.P = 2;
	PID_YAW_Angle.I = 0.05;
	PID_YAW_Angle.I_limit_flag = 0;
	PID_YAW_Angle.I_limit = 35;
	PID_YAW_Angle.I_range = 200;
	
	PID_YAW_Rate.P = 2;
	PID_YAW_Rate.I = 0.05;
	PID_YAW_Rate.I_limit_flag = 0;
	PID_YAW_Rate.I_limit = 150;
	PID_YAW_Rate.I_range = 200;
	
	
	
	// 舵机控制的PID
	PID_Server_Posi.P = 1;	//参数
	PID_Server_Posi.I = 0.05;
	PID_Server_Posi.D = 0.00;
	PID_Server_Posi.I_limit = 250;	//积分分离
	PID_Server_Posi.I_range = 500;	//积分限幅
	PID_Server_Posi.I_limit_flag = 0;	//积分分离标志

}







//*************************************************************************************************//
//********************************************高度融合***********************************************//
//*************************************************************************************************//
// 卡尔曼滤波融合
void kalman_init(KalmanAlt *kf, float dt, float var_accel, float var_alt) {
    // 状态转移 F
    kf->F[0][0]=1; kf->F[0][1]=dt;
    kf->F[1][0]=0; kf->F[1][1]=1;
    // 控制矩阵 G
    kf->G[0] = 0.5f*dt*dt;
    kf->G[1] = dt;
    // 初始状态协方差 P
    memset(kf->P, 0, sizeof(kf->P));
    kf->P[0][0] = 1; kf->P[1][1] = 1;
    // 过程噪声 Q（由加速度计方差构造）
    kf->Q[0][0] = var_accel * kf->G[0]*kf->G[0];
    kf->Q[0][1] = var_accel * kf->G[0]*kf->G[1];
    kf->Q[1][0] = kf->Q[0][1];
    kf->Q[1][1] = var_accel * kf->G[1]*kf->G[1];
//    kf->Q[0][0] = var_accel;
//    kf->Q[0][1] = 0;
//    kf->Q[1][0] = 0;
//    kf->Q[1][1] = var_accel;
    // 观测矩阵 H 与观测噪声 R
    kf->H[0] = 1; kf->H[1] = 0;
    kf->R = var_alt;
    // 初始状态
    kf->x[0] = 0; kf->x[1] = 0;
}
				 
void kalman_predict(KalmanAlt *kf, float accel) {
	int i=0, j=0;
    // x = F*x + G*accel
    float x0 = kf->F[0][0]*kf->x[0] + kf->F[0][1]*kf->x[1] + kf->G[0]*accel;
    float x1 = kf->F[1][0]*kf->x[0] + kf->F[1][1]*kf->x[1] + kf->G[1]*accel;
    kf->x[0]=x0; kf->x[1]=x1;
    // P = F*P*F^T + Q
    float FP[2][2] = {
        {kf->F[0][0]*kf->P[0][0] + kf->F[0][1]*kf->P[1][0],
         kf->F[0][0]*kf->P[0][1] + kf->F[0][1]*kf->P[1][1]},
        {kf->F[1][0]*kf->P[0][0] + kf->F[1][1]*kf->P[1][0],
         kf->F[1][0]*kf->P[0][1] + kf->F[1][1]*kf->P[1][1]}
    };
    float FPFt[2][2] = {
        {FP[0][0]*kf->F[0][0] + FP[0][1]*kf->F[0][1],
         FP[0][0]*kf->F[1][0] + FP[0][1]*kf->F[1][1]},
        {FP[1][0]*kf->F[0][0] + FP[1][1]*kf->F[0][1],
         FP[1][0]*kf->F[1][0] + FP[1][1]*kf->F[1][1]}
    };
    for(i=0;i<2;i++) for(j=0;j<2;j++)
        kf->P[i][j] = FPFt[i][j] + kf->Q[i][j];
}
void kalman_update(KalmanAlt *kf, float z) {
    // 计算预先残差 y = z - H*x
    float y = z - (kf->H[0]*kf->x[0] + kf->H[1]*kf->x[1]);
    // 计算残差协方差 S = H*P*H^T + R
    float S = kf->P[0][0]*kf->H[0]*kf->H[0]
             + (kf->P[0][1]+kf->P[1][0])*kf->H[0]*kf->H[1]
             + kf->P[1][1]*kf->H[1]*kf->H[1]
             + kf->R;
    // 计算卡尔曼增益 K = P*H^T / S
    float K0 = (kf->P[0][0]*kf->H[0] + kf->P[0][1]*kf->H[1]) / S;
    float K1 = (kf->P[1][0]*kf->H[0] + kf->P[1][1]*kf->H[1]) / S;
    // 更新状态 x = x + K*y
    kf->x[0] += K0 * y;
    kf->x[1] += K1 * y;
    // 更新协方差 P = (I - K*H)*P
    float P00 = (1-K0*kf->H[0])*kf->P[0][0] - K0*kf->H[1]*kf->P[1][0];
    float P01 = (1-K0*kf->H[0])*kf->P[0][1] - K0*kf->H[1]*kf->P[1][1];
    float P10 = -K1*kf->H[0]*kf->P[0][0] + (1-K1*kf->H[1])*kf->P[1][0];
    float P11 = -K1*kf->H[0]*kf->P[0][1] + (1-K1*kf->H[1])*kf->P[1][1];
    kf->P[0][0]=P00; kf->P[0][1]=P01;
    kf->P[1][0]=P10; kf->P[1][1]=P11;
}






//*************************************************************************************************//
//********************************************滤波器************************************************//
//*************************************************************************************************//
// 卡尔曼滤波器
/*@param KFP *kfp 卡尔曼结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）*/
float kalmanFilter(KFP *kfp, float input)
{
    // 预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
    kfp->Now_P = kfp->LastP + kfp->Q;
    // 卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
    kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
    // 更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
    kfp->out = kfp->out + kfp->Kg * (input - kfp->out); // 因为这一次的预测值就是上一次的输出值
    // 更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
    kfp->LastP = (1 - kfp->Kg) * kfp->Now_P;
    return kfp->out;
}





//*************************************************************************************************//
//*******************************************定时器中断*********************************************//
//*************************************************************************************************//
// 定时器中断设置
void TIM2_Getsample_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // 时钟使能

    TIM_TimeBaseStructure.TIM_Period = arr;                     // 设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
    TIM_TimeBaseStructure.TIM_Prescaler = psc;                  // 设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;                // 设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);             // 根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_Trigger, ENABLE); // 使能定时器2更新触发中断
    TIM_Cmd(TIM2, ENABLE);                                      // 使能TIMx外设

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;           // TIM2中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 先占优先级0级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        // 从优先级3级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);
}


void TIM2_IRQHandler(void)
{
	//进入中断
	OSIntEnter();    
	
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) // 溢出中断
    {
		
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // 清除中断标志位
	
	//退出中断
	OSIntExit();    
}






//*************************************************************************************************//
//*******************************************废弃的代码*********************************************//
//*************************************************************************************************//
//// 定义姿态解算结构体并初始化参数(全局变量)
//Attitude_Algorithm_Param 	Att_Algo_Param = {
//    .Kp = 10.0f,	// Kp比例增益 决定了加速度计的收敛速度
//    .Ki = 0.2f,  	// Ki积分增益 决定了陀螺仪偏差的收敛速度
//	.dt = 0.01,   			// 采样时间
//	.half_dt = 0.005,   	// 采样时间的一半
//	
//    .q = {1.0, 0.0, 0.0, 0.0},	// 四元数
//	.k10 = 0.0, .k11 = 0.0, .k12 = 0.0, .k13 = 0.0, // 龙格库塔算法用到的中间值
//	.k20 = 0.0, .k21 = 0.0, .k22 = 0.0, .k23 = 0.0,
//	.k30 = 0.0, .k31 = 0.0, .k32 = 0.0, .k33 = 0.0,
//	.k40 = 0.0, .k41 = 0.0, .k42 = 0.0, .k43 = 0.0,
//	
//	.exInt = 0.0, .eyInt = 0.0, .ezInt = 0.0 		// 这些都是姿态解算误差计算的值，网上有说明
//};


////***********根据三轴加速度和三轴陀螺仪，解算出四元数和欧拉角***********//
////*********************** 单位：m/s^2   rad/s ***********************//
//void Attitude_Algorithm_Cal(Attitude_Algorithm_Param * Att, 
//							double ACCX, double ACCY, double ACCZ, 
//							double GYROX, double GYROY, double GYROZ)
//{		
//	// 当前时刻三轴加速度
//	Att->axf = ACCX;		//都额外加负号，不然方向就刚好相反了
//	Att->ayf = ACCY; 
//	Att->azf = ACCZ;  
//	
//	// 归一化
//    Att->norm = invSqrt(Att->axf * Att->axf + Att->ayf * Att->ayf + Att->azf * Att->azf);
//    
//	// 向量a 为传感器重力 飞行器分量
//	Att->axf = Att->axf * Att->norm;         
//    Att->ayf = Att->ayf * Att->norm;
//    Att->azf = Att->azf * Att->norm;
//	
//	// v为把重力反向旋转到飞行器参考系时重力的向量
//    Att->vx = 2 * (Att->q[1] * Att->q[3] - Att->q[0] * Att->q[2]);
//    Att->vy = 2 * (Att->q[0] * Att->q[1] + Att->q[2] * Att->q[3]);
//    Att->vz = Att->q[0] * Att->q[0] - Att->q[1] * Att->q[1] - Att->q[2] * Att->q[2] + Att->q[3] * Att->q[3];
// 
//	// 误差
//    Att->ex = (Att->ayf * Att->vz - Att->azf * Att->vy);
//    Att->ey = (Att->azf * Att->vx - Att->axf * Att->vz);
//    Att->ez = (Att->axf * Att->vy - Att->ayf * Att->vx);
// 
//	// 积分误差比例积分增益
//    Att->exInt = Att->exInt + Att->ex * Att->Ki * Att->half_dt;
//    Att->eyInt = Att->eyInt + Att->ey * Att->Ki * Att->half_dt;
//    Att->ezInt = Att->ezInt + Att->ez * Att->Ki * Att->half_dt;

//    Att->w_new[0] = GYROX; 	// 转为弧度！！！！！！！！！！！！！！！！！！ * PI / 180
//    Att->w_new[1] = GYROY;
//    Att->w_new[2] = GYROZ;
//	
//	// 梯形法求角速度均值
//    Att->w_last[0] = 0.5f * (Att->w_new[0] + Att->w_last[0]);
//    Att->w_last[1] = 0.5f * (Att->w_new[1] + Att->w_last[1]);
//    Att->w_last[2] = 0.5f * (Att->w_new[2] + Att->w_last[2]);

//    Att->w_last[0] = Att->w_last[0] + Att->Kp * Att->ex + Att->exInt; // 调整后的陀螺仪测量
//    Att->w_last[1] = Att->w_last[1] + Att->Kp * Att->ey + Att->eyInt;
//    Att->w_last[2] = Att->w_last[2] + Att->Kp * Att->ez + Att->ezInt;

//    Att->q_last[0] = Att->q[0]; // q保存着上一时刻的四元数，先取出来存着
//    Att->q_last[1] = Att->q[1];
//    Att->q_last[2] = Att->q[2];
//    Att->q_last[3] = Att->q[3];
//	
//	// 四阶龙格库塔方法更新四元素
//    Att->k10 = 0.5 * (-Att->w_last[0] * Att->q_last[1] - Att->w_last[1] * Att->q_last[2] - Att->w_last[2] * Att->q_last[3]);        
//    Att->k11 = 0.5 * (Att->w_last[0] * Att->q_last[0] + Att->w_last[2] * Att->q_last[2] - Att->w_last[1] * Att->q_last[3]);
//    Att->k12 = 0.5 * (Att->w_last[1] * Att->q_last[0] - Att->w_last[2] * Att->q_last[1] + Att->w_last[0] * Att->q_last[3]);
//    Att->k13 = 0.5 * (Att->w_last[2] * Att->q_last[0] + Att->w_last[1] * Att->q_last[1] - Att->w_last[0] * Att->q_last[2]);

//    Att->k20 = 0.5 * (Att->half_dt * (Att->q_last[0] + Att->half_dt * Att->k10) + (Att->half_dt - Att->w_last[0]) * (Att->q_last[1] + Att->half_dt * Att->k11) + (Att->half_dt - Att->w_last[1]) * (Att->q_last[2] + Att->half_dt * Att->k12) + (Att->half_dt - Att->w_last[2]) * (Att->q_last[3] + Att->half_dt * Att->k13));
//    Att->k21 = 0.5 * ((Att->half_dt + Att->w_last[0]) * (Att->q_last[0] + Att->half_dt * Att->k10) + Att->half_dt * (Att->q_last[1] + Att->half_dt * Att->k11) + (Att->half_dt + Att->w_last[2]) * (Att->q_last[2] + Att->half_dt * Att->k12) + (Att->half_dt - Att->w_last[1]) * (Att->q_last[3] + Att->half_dt * Att->k13));
//    Att->k22 = 0.5 * ((Att->half_dt + Att->w_last[1]) * (Att->q_last[0] + Att->half_dt * Att->k10) + (Att->half_dt - Att->w_last[2]) * (Att->q_last[1] + Att->half_dt * Att->k11) + Att->half_dt * (Att->q_last[2] + Att->half_dt * Att->k12) + (Att->half_dt + Att->w_last[0]) * (Att->q_last[3] + Att->half_dt * Att->k13));
//    Att->k23 = 0.5 * ((Att->half_dt + Att->w_last[2]) * (Att->q_last[0] + Att->half_dt * Att->k10) + (Att->half_dt + Att->w_last[1]) * (Att->q_last[1] + Att->half_dt * Att->k11) + (Att->half_dt - Att->w_last[0]) * (Att->q_last[2] + Att->half_dt * Att->k12) + Att->half_dt * (Att->q_last[3] + Att->half_dt * Att->k13));

//    Att->k30 = 0.5 * (Att->half_dt * (Att->q_last[0] + Att->half_dt * Att->k20) + (Att->half_dt - Att->w_last[0]) * (Att->q_last[1] + Att->half_dt * Att->k21) + (Att->half_dt - Att->w_last[1]) * (Att->q_last[2] + Att->half_dt * Att->k22) + (Att->half_dt - Att->w_last[2]) * (Att->q_last[3] + Att->half_dt * Att->k23));
//    Att->k31 = 0.5 * ((Att->half_dt + Att->w_last[0]) * (Att->q_last[0] + Att->half_dt * Att->k20) + Att->half_dt * (Att->q_last[1] + Att->half_dt * Att->k21) + (Att->half_dt + Att->w_last[2]) * (Att->q_last[2] + Att->half_dt * Att->k22) + (Att->half_dt - Att->w_last[1]) * (Att->q_last[3] + Att->half_dt * Att->k23));
//    Att->k32 = 0.5 * ((Att->half_dt + Att->w_last[1]) * (Att->q_last[0] + Att->half_dt * Att->k20) + (Att->half_dt - Att->w_last[2]) * (Att->q_last[1] + Att->half_dt * Att->k21) + Att->half_dt * (Att->q_last[2] + Att->half_dt * Att->k22) + (Att->half_dt + Att->w_last[0]) * (Att->q_last[3] + Att->half_dt * Att->k23));
//    Att->k33 = 0.5 * ((Att->half_dt + Att->w_last[2]) * (Att->q_last[0] + Att->half_dt * Att->k20) + (Att->half_dt + Att->w_last[1]) * (Att->q_last[1] + Att->half_dt * Att->k21) + (Att->half_dt - Att->w_last[0]) * (Att->q_last[2] + Att->half_dt * Att->k22) + Att->half_dt * (Att->q_last[3] + Att->half_dt * Att->k23));

//    Att->k40 = 0.5 * (Att->dt * (Att->q_last[0] + Att->dt * Att->k30) + (Att->dt - Att->w_last[0]) * (Att->q_last[1] + Att->dt * Att->k31) + (Att->dt - Att->w_last[1]) * (Att->q_last[2] + Att->dt * Att->k32) + (Att->dt - Att->w_last[2]) * (Att->q_last[3] + Att->dt * Att->k33));
//    Att->k41 = 0.5 * ((Att->dt + Att->w_last[0]) * (Att->q_last[0] + Att->dt * Att->k30) + Att->dt * (Att->q_last[1] + Att->dt * Att->k31) + (Att->dt + Att->w_last[2]) * (Att->q_last[2] + Att->dt * Att->k32) + (Att->dt - Att->w_last[1]) * (Att->q_last[3] + Att->dt * Att->k33));
//    Att->k42 = 0.5 * ((Att->dt + Att->w_last[1]) * (Att->q_last[0] + Att->dt * Att->k30) + (Att->dt - Att->w_last[2]) * (Att->q_last[1] + Att->dt * Att->k31) + Att->dt * (Att->q_last[2] + Att->dt * Att->k32) + (Att->dt + Att->w_last[0]) * (Att->q_last[3] + Att->dt * Att->k33));
//    Att->k43 = 0.5 * ((Att->dt + Att->w_last[2]) * (Att->q_last[0] + Att->dt * Att->k30) + (Att->dt + Att->w_last[1]) * (Att->q_last[1] + Att->dt * Att->k31) + (Att->dt - Att->w_last[0]) * (Att->q_last[2] + Att->dt * Att->k32) + Att->dt * (Att->q_last[3] + Att->dt * Att->k33));

//    Att->q[0] = Att->q_last[0] + Att->dt / 6.0 * (Att->k10 + 2 * Att->k20 + 2 * Att->k30 + Att->k40);
//    Att->q[1] = Att->q_last[1] + Att->dt / 6.0 * (Att->k11 + 2 * Att->k21 + 2 * Att->k31 + Att->k41);
//    Att->q[2] = Att->q_last[2] + Att->dt / 6.0 * (Att->k12 + 2 * Att->k22 + 2 * Att->k32 + Att->k42);
//    Att->q[3] = Att->q_last[3] + Att->dt / 6.0 * (Att->k13 + 2 * Att->k23 + 2 * Att->k33 + Att->k43);
//	
//	// 四元素的归一化
//    Att->norm = invSqrt(Att->q[0] * Att->q[0] + Att->q[1] * Att->q[1] + Att->q[2] * Att->q[2] + Att->q[3] * Att->q[3]);
//    Att->q[0] = Att->q[0] * Att->norm;
//    Att->q[1] = Att->q[1] * Att->norm;
//    Att->q[2] = Att->q[2] * Att->norm;
//    Att->q[3] = Att->q[3] * Att->norm;
//	
//	// 记录这一时刻的采样值，留下一时刻用
//    Att->w_last[0] = Att->w_new[0];
//    Att->w_last[1] = Att->w_new[1];
//    Att->w_last[2] = Att->w_new[2];
//	
//	// 转换为度数
//    Att->q2Pitc = asin(-2 * Att->q[1] * Att->q[3] + 2 * Att->q[0] * Att->q[2]) * 180.0 / PI;   
//    Att->q2Roll = atan2(2 * (Att->q[2] * Att->q[3] + Att->q[0] * Att->q[1]), -2 * Att->q[1] * Att->q[1] - 2 * Att->q[2] * Att->q[2] + 1) * 180.0 / PI; 
//    Att->q2Yawh = atan2(2 * (Att->q[1] * Att->q[2] + Att->q[0] * Att->q[3]), Att->q[0] * Att->q[0] + Att->q[1] * Att->q[1] - Att->q[2] * Att->q[2] - Att->q[3] * Att->q[3]) * 180.0 / PI;


////		//*************************AHRS源代码：网上可以搜出来的*******************//
////		//***********根据三轴加速度和三轴陀螺仪，解算出四元数和欧拉角***********//
////        axf = ACCX, ayf = ACCY, azf = ACCZ;                // 当前时刻三轴加速度
////        norm = invSqrt(axf * axf + ayf * ayf + azf * azf); // 归一化
////        axf = axf * norm;                                  // 向量a 为传感器重力 飞行器分量
////        ayf = ayf * norm;
////        azf = azf * norm;

////        vx = 2 * (q[1] * q[3] - q[0] * q[2]); // v为把重力反向旋转到飞行器参考系时重力的向量
////        vy = 2 * (q[0] * q[1] + q[2] * q[3]);
////        vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

////        ex = (ayf * vz - azf * vy); // 误差
////        ey = (azf * vx - axf * vz);
////        ez = (axf * vy - ayf * vx);

////        exInt = exInt + ex * Ki * (halfT); // 积分误差比例积分增益
////        eyInt = eyInt + ey * Ki * (halfT);
////        ezInt = ezInt + ez * Ki * (halfT);

////        w_new[0] = GYROX * PI / 180; //  - Winb[0]度转换为弧度！！！！！！！！！！！！！！！！！！
////        w_new[1] = GYROY * PI / 180; //  - Winb[1]陀螺仪角速度 减去有害角速度
////        w_new[2] = GYROZ * PI / 180; //  - Winb[2]

////        w_last[0] = 0.5f * (w_new[0] + w_last[0]); // 梯形法求角速度均值
////        w_last[1] = 0.5f * (w_new[1] + w_last[1]);
////        w_last[2] = 0.5f * (w_new[2] + w_last[2]);

////        w_last[0] = w_last[0] + Kp * ex + exInt; // 调整后的陀螺仪测量
////        w_last[1] = w_last[1] + Kp * ey + eyInt;
////        w_last[2] = w_last[2] + Kp * ez + ezInt;

////        q_last[0] = q[0]; // 上一时刻的四元数
////        q_last[1] = q[1];
////        q_last[2] = q[2];
////        q_last[3] = q[3];

////        k10 = 0.5 * (-w_last[0] * q_last[1] - w_last[1] * q_last[2] - w_last[2] * q_last[3]);        // 四阶龙格库塔方法更新四元素
////        k11 = 0.5 * (w_last[0] * q_last[0] + w_last[2] * q_last[2] - w_last[1] * q_last[3]);
////        k12 = 0.5 * (w_last[1] * q_last[0] - w_last[2] * q_last[1] + w_last[0] * q_last[3]);
////        k13 = 0.5 * (w_last[2] * q_last[0] + w_last[1] * q_last[1] - w_last[0] * q_last[2]);

////        k20 = 0.5 * (halfT * (q_last[0] + halfT * k10) + (halfT - w_last[0]) * (q_last[1] + halfT * k11) + (halfT - w_last[1]) * (q_last[2] + halfT * k12) + (halfT - w_last[2]) * (q_last[3] + halfT * k13));
////        k21 = 0.5 * ((halfT + w_last[0]) * (q_last[0] + halfT * k10) + halfT * (q_last[1] + halfT * k11) + (halfT + w_last[2]) * (q_last[2] + halfT * k12) + (halfT - w_last[1]) * (q_last[3] + halfT * k13));
////        k22 = 0.5 * ((halfT + w_last[1]) * (q_last[0] + halfT * k10) + (halfT - w_last[2]) * (q_last[1] + halfT * k11) + halfT * (q_last[2] + halfT * k12) + (halfT + w_last[0]) * (q_last[3] + halfT * k13));
////        k23 = 0.5 * ((halfT + w_last[2]) * (q_last[0] + halfT * k10) + (halfT + w_last[1]) * (q_last[1] + halfT * k11) + (halfT - w_last[0]) * (q_last[2] + halfT * k12) + halfT * (q_last[3] + halfT * k13));

////        k30 = 0.5 * (halfT * (q_last[0] + halfT * k20) + (halfT - w_last[0]) * (q_last[1] + halfT * k21) + (halfT - w_last[1]) * (q_last[2] + halfT * k22) + (halfT - w_last[2]) * (q_last[3] + halfT * k23));
////        k31 = 0.5 * ((halfT + w_last[0]) * (q_last[0] + halfT * k20) + halfT * (q_last[1] + halfT * k21) + (halfT + w_last[2]) * (q_last[2] + halfT * k22) + (halfT - w_last[1]) * (q_last[3] + halfT * k23));
////        k32 = 0.5 * ((halfT + w_last[1]) * (q_last[0] + halfT * k20) + (halfT - w_last[2]) * (q_last[1] + halfT * k21) + halfT * (q_last[2] + halfT * k22) + (halfT + w_last[0]) * (q_last[3] + halfT * k23));
////        k33 = 0.5 * ((halfT + w_last[2]) * (q_last[0] + halfT * k20) + (halfT + w_last[1]) * (q_last[1] + halfT * k21) + (halfT - w_last[0]) * (q_last[2] + halfT * k22) + halfT * (q_last[3] + halfT * k23));

////        k40 = 0.5 * (T * (q_last[0] + T * k30) + (T - w_last[0]) * (q_last[1] + T * k31) + (T - w_last[1]) * (q_last[2] + T * k32) + (T - w_last[2]) * (q_last[3] + T * k33));
////        k41 = 0.5 * ((T + w_last[0]) * (q_last[0] + T * k30) + T * (q_last[1] + T * k31) + (T + w_last[2]) * (q_last[2] + T * k32) + (T - w_last[1]) * (q_last[3] + T * k33));
////        k42 = 0.5 * ((T + w_last[1]) * (q_last[0] + T * k30) + (T - w_last[2]) * (q_last[1] + T * k31) + T * (q_last[2] + T * k32) + (T + w_last[0]) * (q_last[3] + T * k33));
////        k43 = 0.5 * ((T + w_last[2]) * (q_last[0] + T * k30) + (T + w_last[1]) * (q_last[1] + T * k31) + (T - w_last[0]) * (q_last[2] + T * k32) + T * (q_last[3] + T * k33));

////        q[0] = q_last[0] + T / 6.0 * (k10 + 2 * k20 + 2 * k30 + k40);
////        q[1] = q_last[1] + T / 6.0 * (k11 + 2 * k21 + 2 * k31 + k41);
////        q[2] = q_last[2] + T / 6.0 * (k12 + 2 * k22 + 2 * k32 + k42);
////        q[3] = q_last[3] + T / 6.0 * (k13 + 2 * k23 + 2 * k33 + k43);

////        norm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]); // 四元素的归一化
////        q[0] = q[0] * norm;
////        q[1] = q[1] * norm;
////        q[2] = q[2] * norm;
////        q[3] = q[3] * norm;

////        w_last[0] = w_new[0]; // 记录这一时刻的采样值，留下一时刻用
////        w_last[1] = w_new[1];
////        w_last[2] = w_new[2];

////        C_nb[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]; // 更新姿态旋转矩阵 机体-->地理坐标系
////        C_nb[0][1] = 2 * (q[1] * q[2] - q[0] * q[3]);
////        C_nb[0][2] = 2 * (q[1] * q[3] + q[0] * q[2]);
////        C_nb[1][0] = 2 * (q[1] * q[2] + q[0] * q[3]);
////        C_nb[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
////        C_nb[1][2] = 2 * (q[2] * q[3] - q[0] * q[1]);
////        C_nb[2][0] = 2 * (q[1] * q[3] - q[0] * q[2]);
////        C_nb[2][1] = 2 * (q[2] * q[3] + q[0] * q[1]);
////        C_nb[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

////        q2Pitch = asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180.0 / PI;                                      // pitch ,转换为度数
////        q2Roll = atan2(2 * (q[2] * q[3] + q[0] * q[1]), -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1) * 180.0 / PI; // rollv
////        q2Yaw = atan2(2 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * 180.0 / PI;
//}














//		printf("Longitude:%f\tLatitude:%f\tStatus:%d\n", Raw_GNSS_data.Longitude, Raw_GNSS_data.Latitude, Raw_GNSS_data.Status.GNSS_Fix_Status);
		


//		roll_deg = AHRSData.Roll * (180.0 / PI);			// 传感器测量值 
//		pitch_deg = AHRSData.Pitch * (180.0 / PI);
//		yaw_deg = AHRSData.Heading * (180.0 / PI);


//		if (ELRS_ch6 > 900){
//			LED_PC13 = 1; }
//		else{
//			LED_PC13 = 0;
//		}

//		printf("Roll:%f\tPitch:%f\tHeading:%f\n", AHRSData.Roll * (180.0 / PI), AHRSData.Pitch * (180.0 / PI), AHRSData.Heading * (180.0 / PI));



//        acc_nb[0] = C_nb[0][0] * ACCX + C_nb[0][1] * ACCY + C_nb[0][2] * ACCZ; // acc_nb是在地理坐标系上的加速度，ACCX是机体坐标系的,测得的
//        acc_nb[1] = C_nb[1][0] * ACCX + C_nb[1][1] * ACCY + C_nb[1][2] * ACCZ; // 单位是m/s^2
//        acc_nb[2] = C_nb[2][0] * ACCX + C_nb[2][1] * ACCY + C_nb[2][2] * ACCZ;


//        // USART1接收到的指令解析
//        if (COMM_u8_1 == 0xAA) // 5550AABBCCDD
//        {
//            LED_PA1 = !LED_PA1;	
//        }
//        else if (COMM_u8_1 == 0x66) // 555066BBCCDD
//        {
//            LED_PA1 = 0;
//        }
//        else if (COMM_u8_1 == 0x77) // 555077BBCCDD
//        {
//            LED_PA1 = 1;
//        }

////////////////////////IIC////////////////////////////////////
//unsigned char JY901_chrTemp[20];
//double ACCX, ACCY, ACCZ, GYROX, GYROY, GYROZ, MagX, MagY, MagZ;

// double G_gravity = 0;

//double mx_offset = 0.0f, my_offset = 0.0f, mz_offset = 0.0f; // 磁力计校准零偏初始化
//double mx_min = 0.0f, my_min = 0.0f, mz_min = 0.0f;          // 磁力计校准零偏用的中间变量
//double mx_max = 0.0f, my_max = 0.0f, mz_max = 0.0f;          // 磁力计校准零偏用的中间变量
//extern int LED_PD12_state;

/***********************以下为捷联惯导***********************/
//double C_bn[3][3];                                                    // 地理坐标系到机体坐标系的旋转矩阵
//double Cen[3][3] = {{-0.562740871400957429, 0.826633359872979656, 0}, // 初始位置矩阵
//                    {0.945978016222641147 * 0.826633359872979656, -0.945978016222641147 * 0.562740871400957429, -0.3242307709386579},
//                    {-0.3242307709386579 * 0.826633359872979656, -0.3242307709386579 * 0.562740871400957429, 0.945978016222641147}};
//double Cen_1[3][3];   // 先存好先前的位置矩阵,才进行更新
//int c_i, c_j;         // 旋转矩阵转置时用到的变量而已
//double dVenn[3];      /*载体的对地加速度*/
//double Venn[3];       /*载体的对地速度*/
//double dVenn_last[3]; /*上一时刻载体的对地加速度*/
//double Venn_last[3];  /*上一时刻载体的对地速度*/
//double calculate_Alt; // 高度计算值
//double Wenn[3];       // 这个好像是 与 载体的对地速度、 RN, RM   有关
//float Re = 6378254;
//float Rp = 6356803;
//double f = 1.0 / 298.3;
//double e2 = 2 * (1.0 / 298.3) - (1.0 / 298.3) * (1.0 / 298.3);
//double RN, RM;
//double L = 108.918978, Lambda = 34.24556; // 西安的经纬度
//double Wie = 7.292115e-5;
//double tanL;
//double Rn1;
//double Rm1;
//double ge_a[3];                // 哥氏加速度
//double g0 = 9.780325333434361; // 重力加速度
//double Wien[3];                // 由于地球自传引起的载体的角速度
//double Winb[3];                // 有害角速度

//// Cen[0][0] = -sin(Lambda); // 位置矩阵赋初值
//// Cen[0][1] = cos(Lambda);
//// Cen[0][2] = 0;
//// Cen[1][0] = sin(L) * cos(Lambda);
//// Cen[1][1] = -sin(L) * sin(Lambda);
//// Cen[1][2] = cos(L);
//// Cen[2][0] = cos(L) * cos(Lambda);
//// Cen[2][1] = cos(L) * sin(Lambda);
//// Cen[2][2] = sin(L);


//short CharToShort(unsigned char cData[]);
//// 这函数是IIC读取用到的
//short CharToShort(unsigned char cData[])
//{
//    return ((short)cData[1] << 8) | cData[0];
//}

//        IIC_Read_nByte(0x50, AX, 18, &JY901_chrTemp[0]);                   // JY901B采集的信号
//        ACCX = (double)CharToShort(&JY901_chrTemp[0]) / 32768 * 16 * 9.8f; // m/s^2
//        ACCY = (double)CharToShort(&JY901_chrTemp[2]) / 32768 * 16 * 9.8f;
//        ACCZ = (double)CharToShort(&JY901_chrTemp[4]) / 32768 * 16 * 9.8f;
//        GYROX = (double)CharToShort(&JY901_chrTemp[6]) / 32768 * 2000; // °/s
//        GYROY = (double)CharToShort(&JY901_chrTemp[8]) / 32768 * 2000;
//        GYROZ = (double)CharToShort(&JY901_chrTemp[10]) / 32768 * 2000;
//        MagX = (double)CharToShort(&JY901_chrTemp[12]);
//        MagY = (double)CharToShort(&JY901_chrTemp[14]);
//        MagZ = (double)CharToShort(&JY901_chrTemp[16]);

//        /*计算载体的对地加速度*/
//        dVenn[0] = acc_nb[0];
//        dVenn[1] = acc_nb[1];
//        dVenn[2] = acc_nb[2] - g0 - 0.1f;

//        /*更新载体的对地速度*/
//        Venn[0] = Venn[0] + T * 0.5f * (dVenn_last[0] + dVenn[0]); // 和前面一样梯形法求均值
//        Venn[1] = Venn[1] + T * 0.5f * (dVenn_last[1] + dVenn[1]);
//        Venn[2] = Venn[2] + T * 0.5f * (dVenn_last[2] + dVenn[2]);

//        dVenn_last[0] = dVenn[0]; // 记录这一时刻的对地加速度，留下一时刻用
//        dVenn_last[1] = dVenn[1];
//        dVenn_last[2] = dVenn[2];

//        calculate_Alt = calculate_Alt + T * 0.5f * (Venn_last[2] + Venn[2]);

//        Venn_last[0] = Venn[0]; // 记录这一时刻的对地速度，留下一时刻用
//        Venn_last[1] = Venn[1];
//        Venn_last[2] = Venn[2];

//        /*7.更新载体由于地球自传引起的载体的角速度Wien以及有害角速度Winb*/
//        RN = Re * invSqrt(1 - e2 * sin(L) * sin(L));
//        RM = RN * (1 - e2) / (1 - e2 * sin(L) * sin(L));
//        Wenn[0] = -Venn[1] / (RM + 300); // * Rm1
//        Wenn[1] = Venn[0] / (RN + 300);  //* Rn1
//        Wenn[2] = Venn[0] * tan(L) / (RN + 300);
//        // printf("RN=%.6f,RM=%.6f\r\n", RN, RM);

//        Wien[0] = Wie * Cen[0][2];
//        Wien[1] = Wie * Cen[1][2];
//        Wien[2] = Wie * Cen[2][2];

//        /*8.哥氏加速度*/ // 和图片上的相反，是负的
//        ge_a[0] = (Wien[1] + Wien[1] + Wenn[1]) * Venn[2] - (Wien[2] + Wien[2] + Wenn[2]) * Venn[1];
//        ge_a[1] = (Wien[2] + Wien[2] + Wenn[2]) * Venn[0] - (Wien[0] + Wien[0] + Wenn[0]) * Venn[2];
//        ge_a[2] = (Wien[0] + Wien[0] + Wenn[0]) * Venn[1] - (Wien[1] + Wien[1] + Wenn[1]) * Venn[0];

//        for (c_i = 0; c_i < 3; c_i++)
//        {
//            for (c_j = 0; c_j < 3; c_j++)
//            {
//                C_bn[c_j][c_i] = C_nb[c_i][c_j]; // 姿态矩阵转置
//            }
//        }

//        // 9.n系转到b系上的角速度
//        Winb[0] = C_bn[0][0] * (Wien[0] + Wenn[0]) + C_bn[0][1] * (Wien[1] + Wenn[1]) + C_bn[0][2] * (Wien[2] + Wenn[2]);
//        Winb[1] = C_bn[1][0] * (Wien[0] + Wenn[0]) + C_bn[1][1] * (Wien[1] + Wenn[1]) + C_bn[1][2] * (Wien[2] + Wenn[2]);
//        Winb[2] = C_bn[2][0] * (Wien[0] + Wenn[0]) + C_bn[2][1] * (Wien[1] + Wenn[1]) + C_bn[2][2] * (Wien[2] + Wenn[2]);

//        /*更新位置矩阵Cen*/
//        for (c_i = 0; c_i < 3; c_i++)
//        {
//            for (c_j = 0; c_j < 3; c_j++)
//            {
//                Cen_1[c_i][c_j] = Cen[c_i][c_j]; // 先保存, 因为矩阵一算一步就变了，要保持一致
//            }
//        }
//        Cen[0][0] = Cen_1[0][0] + (Wenn[2] * Cen_1[1][0] - Wenn[1] * Cen_1[2][0]) * T;
//        Cen[0][1] = Cen_1[0][1] + (Wenn[2] * Cen_1[1][1] - Wenn[1] * Cen_1[2][1]) * T;
//        Cen[0][2] = Cen_1[0][2] + (Wenn[2] * Cen_1[1][2] - Wenn[1] * Cen_1[2][2]) * T;
//        Cen[1][0] = Cen_1[1][0] + (Wenn[0] * Cen_1[2][0] - Wenn[2] * Cen_1[0][0]) * T;
//        Cen[1][1] = Cen_1[1][1] + (Wenn[0] * Cen_1[2][1] - Wenn[2] * Cen_1[0][1]) * T;
//        Cen[1][2] = Cen_1[1][2] + (Wenn[0] * Cen_1[2][2] - Wenn[2] * Cen_1[0][2]) * T;
//        Cen[2][0] = Cen_1[2][0] + (Wenn[1] * Cen_1[0][0] - Wenn[0] * Cen_1[1][0]) * T;
//        Cen[2][1] = Cen_1[2][1] + (Wenn[1] * Cen_1[0][1] - Wenn[0] * Cen_1[1][1]) * T;
//        Cen[2][2] = Cen_1[2][2] + (Wenn[1] * Cen_1[0][2] - Wenn[0] * Cen_1[1][2]) * T;

//        L = 180.0 - asin(Cen[2][2]) * 180.0 / PI;
//        Lambda = atan(Cen[2][1] / Cen[2][0]) * 180.0 / PI;

//        if (LED_PD12_state == 0)
//        {
//            if (MagX < mx_min)
//            {
//                mx_min = MagX;
//            }
//            if (MagX > mx_max)
//            {
//                mx_max = MagX;
//            }
//        }
//        else
//        {
//            mx_offset = (mx_max + mx_min) / 2;
//            my_offset = (my_max + my_min) / 2;
//            mz_offset = (mz_max + mz_min) / 2;
//        }

// void Find_G_gravity(void)
// {
//     float C_nb_31, C_nb_32, C_nb_33, ACCZ_nb; // ACCX_nb是在地理坐标系上的加速度，ACCX是机体坐标系的// 单位是m/s^2
//     int i = 0;
//     for (i = 0; i < 1000; i++)
//     {
//         C_nb_31 = 2 * (Q1 * Q3 - Q0 * Q2);
//         C_nb_32 = 2 * (Q2 * Q3 + Q0 * Q1);
//         C_nb_33 = Q0 * Q0 - Q1 * Q1 - Q2 * Q2 + Q3 * Q3;
//         ACCZ_nb = C_nb_31 * ACCX + C_nb_32 * ACCY + C_nb_33 * ACCZ;
//         G_gravity += ACCZ_nb;
//         printf("ACCZ_nb=%.8f m/s^2\r\n", ACCZ_nb);
//         delay_ms(10);
//         LED_PA1 = !LED_PA1;
//     }
//     G_gravity /= 1000;
//     if (G_gravity > 9.7 && G_gravity < 9.85)
//     {
//         printf("G_gravity=%.8f m/s^2\r\n", G_gravity);
//     }
// }

// if (fabs(GYROX) > 14)
// {
//     LED_PA1 = 0;
// }
// else
// {
//     LED_PA1 = 1;
// }
// sprintf((char *)str, "0x50:  a:%.3f %.3f %.3f w:%.3f %.3f %.3f  h:%.0f %.0f %.0f \r\n", ACCX, ACCY, ACCZ, GYROX, GYROY, GYROZ, MagX, MagY, MagZ);
// printf(str);

// if (spl_height > 600)
// {
		// printf("\r\n\r\n\r\nspl_presure=%.5f!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n\r\n\r\n", spl_presure);
// }
// printf("L=%.6f,Lambda=%.6f\r\n", L, Lambda);
// printf("L=%.6f,Lambda=%.6f\r\n", L, Lambda);

// velocity_x += ACCX_nb * T;
// velocity_y += ACCY_nb * T;
// velocity_z += (ACCZ_nb - G_gravity) * T;
// position_x += velocity_x * T;
// position_y += velocity_y * T;
// position_z += velocity_z * T;
// printf("velocity_x_y_z\t%.3f\t%.3f\t%.3f\t   position_x_y_z\t%.3f\t%.3f\t%.3f\r\n",
// 	   (float)velocity_x, (float)velocity_y, (float)velocity_z, (float)position_x, (float)position_y, (float)position_z);
// printf("ROLL, PITCH, YAW:%.3f\t%.3f\t%.3f\t", ROLL, PITCH, YAW);
// printf("position:%.3f\t%.3f\t%.3f\r\n", position_x, position_y, position_z);
// ANO_TC_Send03(ROLL * 100, PITCH * -100, YAW * -100, 1);				 // 这个是上位机问题才要乘以-1，整体要乘以100才发送


// static double velocity_x = 0, position_x = 0, velocity_y = 0, position_y = 0, velocity_z = 0, position_z = 0;
// double Pressure2Hight = 44300.0 * (1.0 - pow(((double)Pressure / 101325.0), (1.0 / 5.256))); // 单位是米
// 
// unsigned char str[100];




/*
		// float C_nb_11 = Q0 * Q0 + Q1 * Q1 - Q2 * Q2 - Q3 * Q3;
        // float C_nb_12 = 2 * (Q1 * Q2 - Q0 * Q3);
        // float C_nb_13 = 2 * (Q1 * Q3 + Q0 * Q2);
        // float C_nb_21 = 2 * (Q1 * Q2 + Q0 * Q3);
        // float C_nb_22 = Q0 * Q0 - Q1 * Q1 + Q2 * Q2 - Q3 * Q3;
        // float C_nb_23 = 2 * (Q2 * Q3 - Q0 * Q1);
        // float C_nb_31 = 2 * (Q1 * Q3 - Q0 * Q2);
        // float C_nb_32 = 2 * (Q2 * Q3 + Q0 * Q1);
        // float C_nb_33 = Q0 * Q0 - Q1 * Q1 - Q2 * Q2 + Q3 * Q3;
        // float ACCX_nb = C_nb_11 * ACCX + C_nb_12 * ACCY + C_nb_13 * ACCZ; // ACCX_nb是在地理坐标系上的加速度，ACCX是机体坐标系的
        // float ACCY_nb = C_nb_21 * ACCX + C_nb_22 * ACCY + C_nb_23 * ACCZ; // 单位是m/s^2
        // float ACCZ_nb = C_nb_31 * ACCX + C_nb_32 * ACCY + C_nb_33 * ACCZ;


// double temp_C1, temp_C2;
        // temp_C1 = Cen[1][0] + (Wenn[0] * Cen[2][0] - Wenn[2] * Cen[0][0]) * T;
        // temp_C2 = Cen[2][0] + (Wenn[1] * Cen[0][0] - Wenn[0] * Cen[1][0]) * T;
        // Cen[1][0] = temp_C1;
        // Cen[2][0] = temp_C2;
        // temp_C1 = Cen[1][1] + (Wenn[0] * Cen[2][1] - Wenn[2] * Cen[0][1]) * T;
        // temp_C2 = Cen[2][1] + (Wenn[1] * Cen[0][1] - Wenn[0] * Cen[1][1]) * T;
        // Cen[1][1] = temp_C1;
        // Cen[2][1] = temp_C2;
        // temp_C1 = Cen[1][2] + (Wenn[0] * Cen[2][2] - Wenn[2] * Cen[0][2]) * T;
        // temp_C2 = Cen[2][2] + (Wenn[1] * Cen[0][2] - Wenn[0] * Cen[1][2]) * T;
        // Cen[1][2] = temp_C1;
        // Cen[2][2] = temp_C2;
        // Cen[0][0] = Cen[1][1] * Cen[2][2] - Cen[1][2] * Cen[2][1];
        // Cen[0][1] = Cen[1][2] * Cen[2][0] - Cen[1][0] * Cen[2][2];
        // Cen[0][2] = Cen[1][0] * Cen[2][1] - Cen[1][1] * Cen[2][0];


        // q[0] = q_last[0] + T * 0.5f * (-w_last[0] * q_last[1] - w_last[1] * q_last[2] - w_last[2] * q_last[3]);
        // q[1] = q_last[1] + T * 0.5f * (w_last[0] * q_last[0] + w_last[2] * q_last[2] - w_last[1] * q_last[3]);
        // q[2] = q_last[2] + T * 0.5f * (w_last[1] * q_last[0] - w_last[2] * q_last[1] + w_last[0] * q_last[3]);
        // q[3] = q_last[3] + T * 0.5f * (w_last[2] * q_last[0] + w_last[1] * q_last[1] - w_last[0] * q_last[2]);

这部分应该是成功了的
        float C_nb_11 = Q0 * Q0 + Q1 * Q1 - Q2 * Q2 - Q3 * Q3;
        float C_nb_12 = 2 * (Q1 * Q2 - Q0 * Q3);
        float C_nb_13 = 2 * (Q1 * Q3 + Q0 * Q2);
        float C_nb_21 = 2 * (Q1 * Q2 + Q0 * Q3);
        float C_nb_22 = Q0 * Q0 - Q1 * Q1 + Q2 * Q2 - Q3 * Q3;
        float C_nb_23 = 2 * (Q2 * Q3 - Q0 * Q1);
        float C_nb_31 = 2 * (Q1 * Q3 - Q0 * Q2);
        float C_nb_32 = 2 * (Q2 * Q3 + Q0 * Q1);
        float C_nb_33 = Q0 * Q0 - Q1 * Q1 - Q2 * Q2 + Q3 * Q3;

        float ACCX_nb = C_nb_11 * ACCX + C_nb_12 * ACCY + C_nb_13 * ACCZ; // ACCX_nb是在地理坐标系上的加速度，ACCX是机体坐标系的
        float ACCY_nb = C_nb_21 * ACCX + C_nb_22 * ACCY + C_nb_23 * ACCZ; // 单位是m/s^2
        float ACCZ_nb = C_nb_31 * ACCX + C_nb_32 * ACCY + C_nb_33 * ACCZ;

        float Pressure2Hight = 44300 * (1 - pow(((double)Pressure / 101325.0), (1.0 / 5.256))); // 单位是米
        Pressure2Hight = kalmanFilter(&KFP_height, (float)Pressure2Hight);

        ANO_TC_Send05(Pressure2Hight * 100, 0, 1);											// 单位是米变为厘米发送
        ANO_TC_Send03(ROLL * 100, PITCH * (-100), YAW * (-100), 1);							// 这个是上位机问题才要乘以-1，整体要乘以100才发送
        ANO_TC_Send01(ACCX_nb * 100, ACCY_nb * 100, ACCZ_nb * 100, GYROX, GYROY, GYROZ, 1); // 加速度(m/s^2)，角速度(°/s) (ACCZ_nb - 9.8f) * 100
        // printf("%.3f\t%.3f\t%.3f\r\n", (float)ROLL, (float)PITCH, (float)YAW);				// Openlog黑匣子存数据

        TIM_SetCompare3(TIM3, 1850 + PITCH / 90 * 100); // PB0

        // USART1接收到的指令
        if (COMM_u8_1 == 0xAA) // 5550AABBCCDD
        {
            LED_PA1 = !LED_PA1;
        }
        else if (COMM_u8_1 == 0x66) // 555066BBCCDD
        {
            LED_PA1 = 0;
        }
        else if (COMM_u8_1 == 0x77) // 555077BBCCDD
        {
            LED_PA1 = 1;
        }
*/

/*
Pressure2Hight = kalmanFilter(&KFP_height, (float)Pressure2Hight);


// u8 sum=0,iii=0;
// float RALLX = ROLL * 3.1415926f / 180, PITCHX = -PITCH * 3.1415926f / 180, YAWX = YAW * 3.1415926f / 180;
// float C_nb_11_2 = (float)(cos(YAWX) * cos(PITCHX));
// float C_nb_12_2 = (float)(-sin(YAWX) * cos(RALLX) + cos(YAWX) * sin(PITCHX) * sin(RALLX));
// float C_nb_13_2 = (float)(sin(YAWX) * sin(RALLX) + cos(YAWX) * sin(PITCHX) * cos(RALLX));
// float C_nb_21_2 = (float)(sin(YAWX) * cos(PITCHX));
// float C_nb_22_2 = (float)(cos(YAWX) * cos(RALLX) + sin(YAWX) * sin(PITCHX) * sin(RALLX));
// float C_nb_23_2 = (float)(-cos(YAWX) * sin(RALLX) + sin(YAWX) * sin(PITCHX) * cos(RALLX));
// float C_nb_31_2 = (float)(-sin(PITCHX));
// float C_nb_32_2 = (float)(cos(PITCHX) * sin(RALLX));
// float C_nb_33_2 = (float)(cos(PITCHX) * cos(RALLX));

// if(Receive_USART3_ok)//串口接收完毕
// {
// 	for(sum=0,iii=0;iii<(Sonar_data[3]+4);iii++)//rgb_data[3]=3
// 	sum+=Sonar_data[iii];
// 	if(sum==Sonar_data[iii])//校验和判断
// 	{
// 		distance=(Sonar_data[4]<<8)|Sonar_data[5];
// 	}
// 	Receive_USART3_ok=0;//处理数据完毕标志
// }
// ANO_TC_Send05(distance, 0, 1);
// send_com(0x01);//发送读指令

// float RALLX = ROLL * 3.1415926f / 180, PITCHX = PITCH * 3.1415926f / 180, YAWX = YAW * 3.1415926f / 180;
// float C_nb_11_2 = (float)(cos(YAWX) * cos(RALLX));
// float C_nb_12_2 = (float)(-sin(YAWX) * cos(PITCHX) + cos(YAWX) * sin(PITCHX) * sin(RALLX));
// float C_nb_13_2 = (float)(sin(YAWX) * sin(PITCHX) + cos(YAWX) * sin(RALLX) * cos(PITCHX));
// float C_nb_21_2 = (float)(sin(YAWX) * cos(RALLX));
// float C_nb_22_2 = (float)(cos(YAWX) * cos(PITCHX) + sin(YAWX) * sin(PITCHX) * sin(RALLX));
// float C_nb_23_2 = (float)(-cos(YAWX) * sin(PITCHX) + sin(YAWX) * sin(RALLX) * cos(PITCHX));
// float C_nb_31_2 = (float)(-sin(RALLX));
// float C_nb_32_2 = (float)(cos(RALLX) * sin(PITCHX));
// float C_nb_33_2 = (float)(cos(PITCHX) * cos(RALLX));
*/



/* LShang的坐标系转换
float RALLX = ROLL * 3.1415926f / 180, PITCHX = PITCH * 3.1415926f / 180, YAWX = YAW * 3.1415926f / 180;
float C_nb_11 = (float)(cos(PITCHX) * cos(YAWX));
float C_nb_12 = (float)(-cos(RALLX) * sin(PITCHX) * cos(YAWX) + sin(RALLX) * sin(YAWX));
float C_nb_13 = (float)(sin(RALLX) * sin(PITCHX) * cos(YAWX) + cos(RALLX) * sin(YAWX));
float C_nb_21 = (float)(sin(PITCHX));
float C_nb_22 = (float)(cos(RALLX) * cos(PITCHX));
float C_nb_23 = (float)(-sin(RALLX) * cos(PITCHX));
float C_nb_31 = (float)(-cos(PITCHX) * sin(YAWX));
float C_nb_32 = (float)(cos(RALLX) * sin(PITCHX) * sin(YAWX) + sin(RALLX) * cos(YAWX));
float C_nb_33 = (float)(-sin(RALLX) * sin(PITCHX) * sin(YAWX) + cos(RALLX) * cos(YAWX)); */
