#include "string.h"
#include "math.h"
#include "sys.h"
#include "includes.h"					//ucos ʹ��	  

#include "control.h"
#include "usart.h"
#include "led.h"
#include "ANO_TC.h"
#include "spl06_001.h"
#include "ELRS.h"
#include "IMU_UART.h"
#include "fdilink_decode.h"
#include "FDILink.h"



//*******************************************��̬����**********************************************//
//*******************����������ٶȺ����������ǡ���������ƣ��������Ԫ����ŷ����*********************//
//---------------------------------------------------------------------------------------------------
// Definitions
#define T			0.025f		//��������׼ȷ��ֱ��Ӱ���˽Ƕȷ�Ӧ�Ŀ���
#define halfT		0.0125f
#define twoKpDef	5.0f	// proportional gain
#define twoKiDef	0.2f	// 2 * integral gain
//---------------------------------------------------------------------------------------------------
// Variable definitions
volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
volatile float w_last[3]; 	// �м������// ��һʱ�̵Ľ��ٶ�
volatile float AHRS_Roll, AHRS_Pitc, AHRS_Yawh;  // ������ // ƫ���ǣ������ǣ�������   ��λ����
volatile float C_nb_11, C_nb_12, C_nb_13, C_nb_21, C_nb_22, C_nb_23, C_nb_31, C_nb_32, C_nb_33;		// ����ϵ��ת����
//---------------------------------------------------------------------------------------------------



//*******************************************PID����**********************************************//
//***************************����ǶȻ�PID�����ٶȻ�PID�ṹ�岢��ʼ������****************************//
//---------------------------------------------------------------------------------------------------
PID_TYPE 	PID_ROL_Angle;	//�ǶȻ�PID
PID_TYPE 	PID_PIT_Angle;
PID_TYPE 	PID_YAW_Angle;

PID_TYPE 	PID_ROL_Rate;	//���ٶȻ�PID
PID_TYPE 	PID_PIT_Rate;
PID_TYPE 	PID_YAW_Rate;

PID_TYPE 	PID_Server_Posi;	//���PID
//---------------------------------------------------------------------------------------------------



//********************************************�˲���***********************************************//
//********************************���忨�����˲����ṹ�岢��ʼ������*********************************//
//---------------------------------------------------------------------------------------------------
KFP KFP_height = {0.02, 0, 0, 0, 0.001, 0.543};
//---------------------------------------------------------------------------------------------------








//*************************************************************************************************//
//*******************************************��̬����**********************************************//
//*************************************************************************************************//
// @���ܣ����ٻ�ÿ����ĵ���  ������˵��
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


// ����������ٶȺ����������ǡ���������ƣ��������Ԫ����ŷ����		// ��λ��m/s^2   rad/s 
void MahonyAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) 
{
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	
	float q_last[4]; 	// ��Ϊ����������Ǵ��еģ�����q[0]��ֱ����q[1]���õ����µ�q[0],��������������Ҫ��棡���ɲ��е�
	float w_new[3];  	// ����ʱ�̵Ľ��ٶ�
	float k10 = 0.0f, k11 = 0.0f, k12 = 0.0f, k13 = 0.0f; 	// ��������㷨�õ����м�ֵ
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
        w_new[0] = gx; // �����ǽ��ٶ� //Ϊ���ȣ�����������������������������������
        w_new[1] = gy;
        w_new[2] = gz;

        w_last[0] = 0.5f * (w_new[0] + w_last[0]); // ���η�����ٶȾ�ֵ
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
	q_last[0] = q0; // ��һʱ�̵���Ԫ��
	q_last[1] = q1;
	q_last[2] = q2;
	q_last[3] = q3;
	
	k10 = 0.5f * (-w_last[0] * q_last[1] - w_last[1] * q_last[2] - w_last[2] * q_last[3]); 		// �Ľ������������������Ԫ��
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
	
	
	w_last[0] = w_new[0]; // ��¼��һʱ�̵Ĳ���ֵ������һʱ����
	w_last[1] = w_new[1];
	w_last[2] = w_new[2];
	//-------------------------------------------------------------------------------------------------
	
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	
	AHRS_Pitc = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180.0 / PI;                                     // pitch ,ת��Ϊ����
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

//	ACCX_n = C_nb_11 * ax + C_nb_12 * ay + C_nb_13 * az; // ACCX_n���ڵ�������ϵ�ϵļ��ٶȣ�ax�ǻ�������ϵ��
//	ACCY_n = C_nb_21 * ax + C_nb_22 * ay + C_nb_23 * az; // ��λ��m/s^2
//	ACCZ_n = C_nb_31 * ax + C_nb_32 * ay + C_nb_33 * az;
	//-------------------------------------------------------------------------------------------------
	
}
 

// ������������ٶȺ����������ǣ��������Ԫ����ŷ����
void MahonyAHRSupdateIMU(float ax, float ay, float az, float gx, float gy, float gz) 
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	
	float q_last[4]; 	// ��Ϊ����������Ǵ��еģ�����q[0]��ֱ����q[1]���õ����µ�q[0],��������������Ҫ��棡���ɲ��е�
	float w_new[3];  	// ����ʱ�̵Ľ��ٶ�
	float k10 = 0.0f, k11 = 0.0f, k12 = 0.0f, k13 = 0.0f; 	// ��������㷨�õ����м�ֵ
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
        w_new[0] = gx; // �����ǽ��ٶ� //Ϊ���ȣ�����������������������������������
        w_new[1] = gy;
        w_new[2] = gz;

        w_last[0] = 0.5f * (w_new[0] + w_last[0]); // ���η�����ٶȾ�ֵ
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
	q_last[0] = q0; // ��һʱ�̵���Ԫ��
	q_last[1] = q1;
	q_last[2] = q2;
	q_last[3] = q3;
	
	k10 = 0.5f * (-w_last[0] * q_last[1] - w_last[1] * q_last[2] - w_last[2] * q_last[3]); 		// �Ľ������������������Ԫ��
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
	
	
	w_last[0] = w_new[0]; // ��¼��һʱ�̵Ĳ���ֵ������һʱ����
	w_last[1] = w_new[1];
	w_last[2] = w_new[2];
	//-------------------------------------------------------------------------------------------------
	
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	
	AHRS_Pitc = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180.0f / PI;                                     // pitch ,ת��Ϊ����
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

//	ACCX_n = C_nb_11 * ax + C_nb_12 * ay + C_nb_13 * az; // ACCX_n���ڵ�������ϵ�ϵļ��ٶȣ�ax�ǻ�������ϵ��
//	ACCY_n = C_nb_21 * ax + C_nb_22 * ay + C_nb_23 * az; // ��λ��m/s^2
//	ACCZ_n = C_nb_31 * ax + C_nb_32 * ay + C_nb_33 * az;
	//-------------------------------------------------------------------------------------------------
	
}




//*************************************************************************************************//
//*********************************************PID����**********************************************//
//*************************************************************************************************//
// λ��ʽPID����
void PID_Position_Cal(PID_TYPE* PID, float target, float measure)
{
	//int dt =0:	//����ʱ��(Ҳ����ɨ��ʱ��10ms)
	PID->Error = target - measure;				//���
	PID->Differ = PID->Error - PID->PreError;	//΢����
	
	if(aircraft_take_off)		// ��ɺ�ſ�ʼ����
	{
			if(measure > PID->I_limit || measure < -PID->I_limit)		//���ַ���
			{
				PID->I_limit_flag = 0;
			}
			else
			{	
				PID->I_limit_flag = 1;
				PID->Integral += PID->Error;		//������
				if(PID->Integral > PID->I_range)	//�����޷�
					PID->Integral = PID->I_range;
				if(PID->Integral < -PID->I_range)	//�����޷�
					PID->Integral = -PID->I_range;
				
			}
	}else
	{
		PID->Integral = 0;
	}
	
	PID->Pout = PID->P * PID->Error;	//�����������
	PID->Dout = PID->D * PID->Differ;	//΢�ֿ������
	PID->Iout = PID->I * PID->Integral * PID->I_limit_flag;	//���ֿ������

	PID->OutPut = PID->Pout + PID->Iout + PID->Dout;	//PID���������
	PID->PreError = PID->Error;		//ǰһ�����ֵ
}


// ���λ��ʽPID����
void PID_Posi_Server_Cal(PID_TYPE* PID, float target, float measure)
{
	
	PID->Error = target - measure;				//���
	PID->Differ = PID->Error - PID->PreError;	//΢����
	
	
	if(measure > PID->I_limit || measure < -PID->I_limit)		//���ַ���
	{
		PID->I_limit_flag = 0;
	}
	else
	{	
		PID->I_limit_flag = 1;
		PID->Integral += PID->Error;		//������
		if(PID->Integral > PID->I_range)	//�����޷�
			PID->Integral = PID->I_range;
		if(PID->Integral < -PID->I_range)	//�����޷�
			PID->Integral = -PID->I_range;
		
	}
	
	PID->Pout = PID->P * PID->Error;	//�����������
	PID->Dout = PID->D * PID->Differ;	//΢�ֿ������
	PID->Iout = PID->I * PID->Integral * PID->I_limit_flag;	//���ֿ������

	PID->OutPut = PID->Pout + PID->Iout + PID->Dout;	//PID���������
	PID->PreError = PID->Error;		//ǰһ�����ֵ
	
}



// PID������ʼ��
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
	
	
	
	// ������Ƶ�PID
	PID_Server_Posi.P = 1;	//����
	PID_Server_Posi.I = 0.05;
	PID_Server_Posi.D = 0.00;
	PID_Server_Posi.I_limit = 250;	//���ַ���
	PID_Server_Posi.I_range = 500;	//�����޷�
	PID_Server_Posi.I_limit_flag = 0;	//���ַ����־

}







//*************************************************************************************************//
//********************************************�߶��ں�***********************************************//
//*************************************************************************************************//
// �������˲��ں�
void kalman_init(KalmanAlt *kf, float dt, float var_accel, float var_alt) {
    // ״̬ת�� F
    kf->F[0][0]=1; kf->F[0][1]=dt;
    kf->F[1][0]=0; kf->F[1][1]=1;
    // ���ƾ��� G
    kf->G[0] = 0.5f*dt*dt;
    kf->G[1] = dt;
    // ��ʼ״̬Э���� P
    memset(kf->P, 0, sizeof(kf->P));
    kf->P[0][0] = 1; kf->P[1][1] = 1;
    // �������� Q���ɼ��ٶȼƷ���죩
    kf->Q[0][0] = var_accel * kf->G[0]*kf->G[0];
    kf->Q[0][1] = var_accel * kf->G[0]*kf->G[1];
    kf->Q[1][0] = kf->Q[0][1];
    kf->Q[1][1] = var_accel * kf->G[1]*kf->G[1];
//    kf->Q[0][0] = var_accel;
//    kf->Q[0][1] = 0;
//    kf->Q[1][0] = 0;
//    kf->Q[1][1] = var_accel;
    // �۲���� H ��۲����� R
    kf->H[0] = 1; kf->H[1] = 0;
    kf->R = var_alt;
    // ��ʼ״̬
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
    // ����Ԥ�Ȳв� y = z - H*x
    float y = z - (kf->H[0]*kf->x[0] + kf->H[1]*kf->x[1]);
    // ����в�Э���� S = H*P*H^T + R
    float S = kf->P[0][0]*kf->H[0]*kf->H[0]
             + (kf->P[0][1]+kf->P[1][0])*kf->H[0]*kf->H[1]
             + kf->P[1][1]*kf->H[1]*kf->H[1]
             + kf->R;
    // ���㿨�������� K = P*H^T / S
    float K0 = (kf->P[0][0]*kf->H[0] + kf->P[0][1]*kf->H[1]) / S;
    float K1 = (kf->P[1][0]*kf->H[0] + kf->P[1][1]*kf->H[1]) / S;
    // ����״̬ x = x + K*y
    kf->x[0] += K0 * y;
    kf->x[1] += K1 * y;
    // ����Э���� P = (I - K*H)*P
    float P00 = (1-K0*kf->H[0])*kf->P[0][0] - K0*kf->H[1]*kf->P[1][0];
    float P01 = (1-K0*kf->H[0])*kf->P[0][1] - K0*kf->H[1]*kf->P[1][1];
    float P10 = -K1*kf->H[0]*kf->P[0][0] + (1-K1*kf->H[1])*kf->P[1][0];
    float P11 = -K1*kf->H[0]*kf->P[0][1] + (1-K1*kf->H[1])*kf->P[1][1];
    kf->P[0][0]=P00; kf->P[0][1]=P01;
    kf->P[1][0]=P10; kf->P[1][1]=P11;
}






//*************************************************************************************************//
//********************************************�˲���************************************************//
//*************************************************************************************************//
// �������˲���
/*@param KFP *kfp �������ṹ�����
 *   float input ��Ҫ�˲��Ĳ����Ĳ���ֵ�����������Ĳɼ�ֵ��
 *@return �˲���Ĳ���������ֵ��*/
float kalmanFilter(KFP *kfp, float input)
{
    // Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
    kfp->Now_P = kfp->LastP + kfp->Q;
    // ���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
    kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
    // ��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
    kfp->out = kfp->out + kfp->Kg * (input - kfp->out); // ��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ
    // ����Э�����: ���ε�ϵͳЭ����� kfp->LastP ����һ������׼����
    kfp->LastP = (1 - kfp->Kg) * kfp->Now_P;
    return kfp->out;
}





//*************************************************************************************************//
//*******************************************��ʱ���ж�*********************************************//
//*************************************************************************************************//
// ��ʱ���ж�����
void TIM2_Getsample_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // ʱ��ʹ��

    TIM_TimeBaseStructure.TIM_Period = arr;                     // ��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
    TIM_TimeBaseStructure.TIM_Prescaler = psc;                  // ����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;                // ����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);             // ����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_Trigger, ENABLE); // ʹ�ܶ�ʱ��2���´����ж�
    TIM_Cmd(TIM2, ENABLE);                                      // ʹ��TIMx����

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;           // TIM2�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // ��ռ���ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        // �����ȼ�3��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);
}


void TIM2_IRQHandler(void)
{
	//�����ж�
	OSIntEnter();    
	
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) // ����ж�
    {
		
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // ����жϱ�־λ
	
	//�˳��ж�
	OSIntExit();    
}






//*************************************************************************************************//
//*******************************************�����Ĵ���*********************************************//
//*************************************************************************************************//
//// ������̬����ṹ�岢��ʼ������(ȫ�ֱ���)
//Attitude_Algorithm_Param 	Att_Algo_Param = {
//    .Kp = 10.0f,	// Kp�������� �����˼��ٶȼƵ������ٶ�
//    .Ki = 0.2f,  	// Ki�������� ������������ƫ��������ٶ�
//	.dt = 0.01,   			// ����ʱ��
//	.half_dt = 0.005,   	// ����ʱ���һ��
//	
//    .q = {1.0, 0.0, 0.0, 0.0},	// ��Ԫ��
//	.k10 = 0.0, .k11 = 0.0, .k12 = 0.0, .k13 = 0.0, // ��������㷨�õ����м�ֵ
//	.k20 = 0.0, .k21 = 0.0, .k22 = 0.0, .k23 = 0.0,
//	.k30 = 0.0, .k31 = 0.0, .k32 = 0.0, .k33 = 0.0,
//	.k40 = 0.0, .k41 = 0.0, .k42 = 0.0, .k43 = 0.0,
//	
//	.exInt = 0.0, .eyInt = 0.0, .ezInt = 0.0 		// ��Щ������̬�����������ֵ��������˵��
//};


////***********����������ٶȺ����������ǣ��������Ԫ����ŷ����***********//
////*********************** ��λ��m/s^2   rad/s ***********************//
//void Attitude_Algorithm_Cal(Attitude_Algorithm_Param * Att, 
//							double ACCX, double ACCY, double ACCZ, 
//							double GYROX, double GYROY, double GYROZ)
//{		
//	// ��ǰʱ��������ٶ�
//	Att->axf = ACCX;		//������Ӹ��ţ���Ȼ����͸պ��෴��
//	Att->ayf = ACCY; 
//	Att->azf = ACCZ;  
//	
//	// ��һ��
//    Att->norm = invSqrt(Att->axf * Att->axf + Att->ayf * Att->ayf + Att->azf * Att->azf);
//    
//	// ����a Ϊ���������� ����������
//	Att->axf = Att->axf * Att->norm;         
//    Att->ayf = Att->ayf * Att->norm;
//    Att->azf = Att->azf * Att->norm;
//	
//	// vΪ������������ת���������ο�ϵʱ����������
//    Att->vx = 2 * (Att->q[1] * Att->q[3] - Att->q[0] * Att->q[2]);
//    Att->vy = 2 * (Att->q[0] * Att->q[1] + Att->q[2] * Att->q[3]);
//    Att->vz = Att->q[0] * Att->q[0] - Att->q[1] * Att->q[1] - Att->q[2] * Att->q[2] + Att->q[3] * Att->q[3];
// 
//	// ���
//    Att->ex = (Att->ayf * Att->vz - Att->azf * Att->vy);
//    Att->ey = (Att->azf * Att->vx - Att->axf * Att->vz);
//    Att->ez = (Att->axf * Att->vy - Att->ayf * Att->vx);
// 
//	// ������������������
//    Att->exInt = Att->exInt + Att->ex * Att->Ki * Att->half_dt;
//    Att->eyInt = Att->eyInt + Att->ey * Att->Ki * Att->half_dt;
//    Att->ezInt = Att->ezInt + Att->ez * Att->Ki * Att->half_dt;

//    Att->w_new[0] = GYROX; 	// תΪ���ȣ����������������������������������� * PI / 180
//    Att->w_new[1] = GYROY;
//    Att->w_new[2] = GYROZ;
//	
//	// ���η�����ٶȾ�ֵ
//    Att->w_last[0] = 0.5f * (Att->w_new[0] + Att->w_last[0]);
//    Att->w_last[1] = 0.5f * (Att->w_new[1] + Att->w_last[1]);
//    Att->w_last[2] = 0.5f * (Att->w_new[2] + Att->w_last[2]);

//    Att->w_last[0] = Att->w_last[0] + Att->Kp * Att->ex + Att->exInt; // ������������ǲ���
//    Att->w_last[1] = Att->w_last[1] + Att->Kp * Att->ey + Att->eyInt;
//    Att->w_last[2] = Att->w_last[2] + Att->Kp * Att->ez + Att->ezInt;

//    Att->q_last[0] = Att->q[0]; // q��������һʱ�̵���Ԫ������ȡ��������
//    Att->q_last[1] = Att->q[1];
//    Att->q_last[2] = Att->q[2];
//    Att->q_last[3] = Att->q[3];
//	
//	// �Ľ������������������Ԫ��
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
//	// ��Ԫ�صĹ�һ��
//    Att->norm = invSqrt(Att->q[0] * Att->q[0] + Att->q[1] * Att->q[1] + Att->q[2] * Att->q[2] + Att->q[3] * Att->q[3]);
//    Att->q[0] = Att->q[0] * Att->norm;
//    Att->q[1] = Att->q[1] * Att->norm;
//    Att->q[2] = Att->q[2] * Att->norm;
//    Att->q[3] = Att->q[3] * Att->norm;
//	
//	// ��¼��һʱ�̵Ĳ���ֵ������һʱ����
//    Att->w_last[0] = Att->w_new[0];
//    Att->w_last[1] = Att->w_new[1];
//    Att->w_last[2] = Att->w_new[2];
//	
//	// ת��Ϊ����
//    Att->q2Pitc = asin(-2 * Att->q[1] * Att->q[3] + 2 * Att->q[0] * Att->q[2]) * 180.0 / PI;   
//    Att->q2Roll = atan2(2 * (Att->q[2] * Att->q[3] + Att->q[0] * Att->q[1]), -2 * Att->q[1] * Att->q[1] - 2 * Att->q[2] * Att->q[2] + 1) * 180.0 / PI; 
//    Att->q2Yawh = atan2(2 * (Att->q[1] * Att->q[2] + Att->q[0] * Att->q[3]), Att->q[0] * Att->q[0] + Att->q[1] * Att->q[1] - Att->q[2] * Att->q[2] - Att->q[3] * Att->q[3]) * 180.0 / PI;


////		//*************************AHRSԴ���룺���Ͽ����ѳ�����*******************//
////		//***********����������ٶȺ����������ǣ��������Ԫ����ŷ����***********//
////        axf = ACCX, ayf = ACCY, azf = ACCZ;                // ��ǰʱ��������ٶ�
////        norm = invSqrt(axf * axf + ayf * ayf + azf * azf); // ��һ��
////        axf = axf * norm;                                  // ����a Ϊ���������� ����������
////        ayf = ayf * norm;
////        azf = azf * norm;

////        vx = 2 * (q[1] * q[3] - q[0] * q[2]); // vΪ������������ת���������ο�ϵʱ����������
////        vy = 2 * (q[0] * q[1] + q[2] * q[3]);
////        vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

////        ex = (ayf * vz - azf * vy); // ���
////        ey = (azf * vx - axf * vz);
////        ez = (axf * vy - ayf * vx);

////        exInt = exInt + ex * Ki * (halfT); // ������������������
////        eyInt = eyInt + ey * Ki * (halfT);
////        ezInt = ezInt + ez * Ki * (halfT);

////        w_new[0] = GYROX * PI / 180; //  - Winb[0]��ת��Ϊ���ȣ�����������������������������������
////        w_new[1] = GYROY * PI / 180; //  - Winb[1]�����ǽ��ٶ� ��ȥ�к����ٶ�
////        w_new[2] = GYROZ * PI / 180; //  - Winb[2]

////        w_last[0] = 0.5f * (w_new[0] + w_last[0]); // ���η�����ٶȾ�ֵ
////        w_last[1] = 0.5f * (w_new[1] + w_last[1]);
////        w_last[2] = 0.5f * (w_new[2] + w_last[2]);

////        w_last[0] = w_last[0] + Kp * ex + exInt; // ������������ǲ���
////        w_last[1] = w_last[1] + Kp * ey + eyInt;
////        w_last[2] = w_last[2] + Kp * ez + ezInt;

////        q_last[0] = q[0]; // ��һʱ�̵���Ԫ��
////        q_last[1] = q[1];
////        q_last[2] = q[2];
////        q_last[3] = q[3];

////        k10 = 0.5 * (-w_last[0] * q_last[1] - w_last[1] * q_last[2] - w_last[2] * q_last[3]);        // �Ľ������������������Ԫ��
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

////        norm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]); // ��Ԫ�صĹ�һ��
////        q[0] = q[0] * norm;
////        q[1] = q[1] * norm;
////        q[2] = q[2] * norm;
////        q[3] = q[3] * norm;

////        w_last[0] = w_new[0]; // ��¼��һʱ�̵Ĳ���ֵ������һʱ����
////        w_last[1] = w_new[1];
////        w_last[2] = w_new[2];

////        C_nb[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]; // ������̬��ת���� ����-->��������ϵ
////        C_nb[0][1] = 2 * (q[1] * q[2] - q[0] * q[3]);
////        C_nb[0][2] = 2 * (q[1] * q[3] + q[0] * q[2]);
////        C_nb[1][0] = 2 * (q[1] * q[2] + q[0] * q[3]);
////        C_nb[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
////        C_nb[1][2] = 2 * (q[2] * q[3] - q[0] * q[1]);
////        C_nb[2][0] = 2 * (q[1] * q[3] - q[0] * q[2]);
////        C_nb[2][1] = 2 * (q[2] * q[3] + q[0] * q[1]);
////        C_nb[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

////        q2Pitch = asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180.0 / PI;                                      // pitch ,ת��Ϊ����
////        q2Roll = atan2(2 * (q[2] * q[3] + q[0] * q[1]), -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1) * 180.0 / PI; // rollv
////        q2Yaw = atan2(2 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * 180.0 / PI;
//}














//		printf("Longitude:%f\tLatitude:%f\tStatus:%d\n", Raw_GNSS_data.Longitude, Raw_GNSS_data.Latitude, Raw_GNSS_data.Status.GNSS_Fix_Status);
		


//		roll_deg = AHRSData.Roll * (180.0 / PI);			// ����������ֵ 
//		pitch_deg = AHRSData.Pitch * (180.0 / PI);
//		yaw_deg = AHRSData.Heading * (180.0 / PI);


//		if (ELRS_ch6 > 900){
//			LED_PC13 = 1; }
//		else{
//			LED_PC13 = 0;
//		}

//		printf("Roll:%f\tPitch:%f\tHeading:%f\n", AHRSData.Roll * (180.0 / PI), AHRSData.Pitch * (180.0 / PI), AHRSData.Heading * (180.0 / PI));



//        acc_nb[0] = C_nb[0][0] * ACCX + C_nb[0][1] * ACCY + C_nb[0][2] * ACCZ; // acc_nb���ڵ�������ϵ�ϵļ��ٶȣ�ACCX�ǻ�������ϵ��,��õ�
//        acc_nb[1] = C_nb[1][0] * ACCX + C_nb[1][1] * ACCY + C_nb[1][2] * ACCZ; // ��λ��m/s^2
//        acc_nb[2] = C_nb[2][0] * ACCX + C_nb[2][1] * ACCY + C_nb[2][2] * ACCZ;


//        // USART1���յ���ָ�����
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

//double mx_offset = 0.0f, my_offset = 0.0f, mz_offset = 0.0f; // ������У׼��ƫ��ʼ��
//double mx_min = 0.0f, my_min = 0.0f, mz_min = 0.0f;          // ������У׼��ƫ�õ��м����
//double mx_max = 0.0f, my_max = 0.0f, mz_max = 0.0f;          // ������У׼��ƫ�õ��м����
//extern int LED_PD12_state;

/***********************����Ϊ�����ߵ�***********************/
//double C_bn[3][3];                                                    // ��������ϵ����������ϵ����ת����
//double Cen[3][3] = {{-0.562740871400957429, 0.826633359872979656, 0}, // ��ʼλ�þ���
//                    {0.945978016222641147 * 0.826633359872979656, -0.945978016222641147 * 0.562740871400957429, -0.3242307709386579},
//                    {-0.3242307709386579 * 0.826633359872979656, -0.3242307709386579 * 0.562740871400957429, 0.945978016222641147}};
//double Cen_1[3][3];   // �ȴ����ǰ��λ�þ���,�Ž��и���
//int c_i, c_j;         // ��ת����ת��ʱ�õ��ı�������
//double dVenn[3];      /*����ĶԵؼ��ٶ�*/
//double Venn[3];       /*����ĶԵ��ٶ�*/
//double dVenn_last[3]; /*��һʱ������ĶԵؼ��ٶ�*/
//double Venn_last[3];  /*��һʱ������ĶԵ��ٶ�*/
//double calculate_Alt; // �߶ȼ���ֵ
//double Wenn[3];       // ��������� �� ����ĶԵ��ٶȡ� RN, RM   �й�
//float Re = 6378254;
//float Rp = 6356803;
//double f = 1.0 / 298.3;
//double e2 = 2 * (1.0 / 298.3) - (1.0 / 298.3) * (1.0 / 298.3);
//double RN, RM;
//double L = 108.918978, Lambda = 34.24556; // �����ľ�γ��
//double Wie = 7.292115e-5;
//double tanL;
//double Rn1;
//double Rm1;
//double ge_a[3];                // ���ϼ��ٶ�
//double g0 = 9.780325333434361; // �������ٶ�
//double Wien[3];                // ���ڵ����Դ����������Ľ��ٶ�
//double Winb[3];                // �к����ٶ�

//// Cen[0][0] = -sin(Lambda); // λ�þ��󸳳�ֵ
//// Cen[0][1] = cos(Lambda);
//// Cen[0][2] = 0;
//// Cen[1][0] = sin(L) * cos(Lambda);
//// Cen[1][1] = -sin(L) * sin(Lambda);
//// Cen[1][2] = cos(L);
//// Cen[2][0] = cos(L) * cos(Lambda);
//// Cen[2][1] = cos(L) * sin(Lambda);
//// Cen[2][2] = sin(L);


//short CharToShort(unsigned char cData[]);
//// �⺯����IIC��ȡ�õ���
//short CharToShort(unsigned char cData[])
//{
//    return ((short)cData[1] << 8) | cData[0];
//}

//        IIC_Read_nByte(0x50, AX, 18, &JY901_chrTemp[0]);                   // JY901B�ɼ����ź�
//        ACCX = (double)CharToShort(&JY901_chrTemp[0]) / 32768 * 16 * 9.8f; // m/s^2
//        ACCY = (double)CharToShort(&JY901_chrTemp[2]) / 32768 * 16 * 9.8f;
//        ACCZ = (double)CharToShort(&JY901_chrTemp[4]) / 32768 * 16 * 9.8f;
//        GYROX = (double)CharToShort(&JY901_chrTemp[6]) / 32768 * 2000; // ��/s
//        GYROY = (double)CharToShort(&JY901_chrTemp[8]) / 32768 * 2000;
//        GYROZ = (double)CharToShort(&JY901_chrTemp[10]) / 32768 * 2000;
//        MagX = (double)CharToShort(&JY901_chrTemp[12]);
//        MagY = (double)CharToShort(&JY901_chrTemp[14]);
//        MagZ = (double)CharToShort(&JY901_chrTemp[16]);

//        /*��������ĶԵؼ��ٶ�*/
//        dVenn[0] = acc_nb[0];
//        dVenn[1] = acc_nb[1];
//        dVenn[2] = acc_nb[2] - g0 - 0.1f;

//        /*��������ĶԵ��ٶ�*/
//        Venn[0] = Venn[0] + T * 0.5f * (dVenn_last[0] + dVenn[0]); // ��ǰ��һ�����η����ֵ
//        Venn[1] = Venn[1] + T * 0.5f * (dVenn_last[1] + dVenn[1]);
//        Venn[2] = Venn[2] + T * 0.5f * (dVenn_last[2] + dVenn[2]);

//        dVenn_last[0] = dVenn[0]; // ��¼��һʱ�̵ĶԵؼ��ٶȣ�����һʱ����
//        dVenn_last[1] = dVenn[1];
//        dVenn_last[2] = dVenn[2];

//        calculate_Alt = calculate_Alt + T * 0.5f * (Venn_last[2] + Venn[2]);

//        Venn_last[0] = Venn[0]; // ��¼��һʱ�̵ĶԵ��ٶȣ�����һʱ����
//        Venn_last[1] = Venn[1];
//        Venn_last[2] = Venn[2];

//        /*7.�����������ڵ����Դ����������Ľ��ٶ�Wien�Լ��к����ٶ�Winb*/
//        RN = Re * invSqrt(1 - e2 * sin(L) * sin(L));
//        RM = RN * (1 - e2) / (1 - e2 * sin(L) * sin(L));
//        Wenn[0] = -Venn[1] / (RM + 300); // * Rm1
//        Wenn[1] = Venn[0] / (RN + 300);  //* Rn1
//        Wenn[2] = Venn[0] * tan(L) / (RN + 300);
//        // printf("RN=%.6f,RM=%.6f\r\n", RN, RM);

//        Wien[0] = Wie * Cen[0][2];
//        Wien[1] = Wie * Cen[1][2];
//        Wien[2] = Wie * Cen[2][2];

//        /*8.���ϼ��ٶ�*/ // ��ͼƬ�ϵ��෴���Ǹ���
//        ge_a[0] = (Wien[1] + Wien[1] + Wenn[1]) * Venn[2] - (Wien[2] + Wien[2] + Wenn[2]) * Venn[1];
//        ge_a[1] = (Wien[2] + Wien[2] + Wenn[2]) * Venn[0] - (Wien[0] + Wien[0] + Wenn[0]) * Venn[2];
//        ge_a[2] = (Wien[0] + Wien[0] + Wenn[0]) * Venn[1] - (Wien[1] + Wien[1] + Wenn[1]) * Venn[0];

//        for (c_i = 0; c_i < 3; c_i++)
//        {
//            for (c_j = 0; c_j < 3; c_j++)
//            {
//                C_bn[c_j][c_i] = C_nb[c_i][c_j]; // ��̬����ת��
//            }
//        }

//        // 9.nϵת��bϵ�ϵĽ��ٶ�
//        Winb[0] = C_bn[0][0] * (Wien[0] + Wenn[0]) + C_bn[0][1] * (Wien[1] + Wenn[1]) + C_bn[0][2] * (Wien[2] + Wenn[2]);
//        Winb[1] = C_bn[1][0] * (Wien[0] + Wenn[0]) + C_bn[1][1] * (Wien[1] + Wenn[1]) + C_bn[1][2] * (Wien[2] + Wenn[2]);
//        Winb[2] = C_bn[2][0] * (Wien[0] + Wenn[0]) + C_bn[2][1] * (Wien[1] + Wenn[1]) + C_bn[2][2] * (Wien[2] + Wenn[2]);

//        /*����λ�þ���Cen*/
//        for (c_i = 0; c_i < 3; c_i++)
//        {
//            for (c_j = 0; c_j < 3; c_j++)
//            {
//                Cen_1[c_i][c_j] = Cen[c_i][c_j]; // �ȱ���, ��Ϊ����һ��һ���ͱ��ˣ�Ҫ����һ��
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
//     float C_nb_31, C_nb_32, C_nb_33, ACCZ_nb; // ACCX_nb���ڵ�������ϵ�ϵļ��ٶȣ�ACCX�ǻ�������ϵ��// ��λ��m/s^2
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
// ANO_TC_Send03(ROLL * 100, PITCH * -100, YAW * -100, 1);				 // �������λ�������Ҫ����-1������Ҫ����100�ŷ���


// static double velocity_x = 0, position_x = 0, velocity_y = 0, position_y = 0, velocity_z = 0, position_z = 0;
// double Pressure2Hight = 44300.0 * (1.0 - pow(((double)Pressure / 101325.0), (1.0 / 5.256))); // ��λ����
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
        // float ACCX_nb = C_nb_11 * ACCX + C_nb_12 * ACCY + C_nb_13 * ACCZ; // ACCX_nb���ڵ�������ϵ�ϵļ��ٶȣ�ACCX�ǻ�������ϵ��
        // float ACCY_nb = C_nb_21 * ACCX + C_nb_22 * ACCY + C_nb_23 * ACCZ; // ��λ��m/s^2
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

�ⲿ��Ӧ���ǳɹ��˵�
        float C_nb_11 = Q0 * Q0 + Q1 * Q1 - Q2 * Q2 - Q3 * Q3;
        float C_nb_12 = 2 * (Q1 * Q2 - Q0 * Q3);
        float C_nb_13 = 2 * (Q1 * Q3 + Q0 * Q2);
        float C_nb_21 = 2 * (Q1 * Q2 + Q0 * Q3);
        float C_nb_22 = Q0 * Q0 - Q1 * Q1 + Q2 * Q2 - Q3 * Q3;
        float C_nb_23 = 2 * (Q2 * Q3 - Q0 * Q1);
        float C_nb_31 = 2 * (Q1 * Q3 - Q0 * Q2);
        float C_nb_32 = 2 * (Q2 * Q3 + Q0 * Q1);
        float C_nb_33 = Q0 * Q0 - Q1 * Q1 - Q2 * Q2 + Q3 * Q3;

        float ACCX_nb = C_nb_11 * ACCX + C_nb_12 * ACCY + C_nb_13 * ACCZ; // ACCX_nb���ڵ�������ϵ�ϵļ��ٶȣ�ACCX�ǻ�������ϵ��
        float ACCY_nb = C_nb_21 * ACCX + C_nb_22 * ACCY + C_nb_23 * ACCZ; // ��λ��m/s^2
        float ACCZ_nb = C_nb_31 * ACCX + C_nb_32 * ACCY + C_nb_33 * ACCZ;

        float Pressure2Hight = 44300 * (1 - pow(((double)Pressure / 101325.0), (1.0 / 5.256))); // ��λ����
        Pressure2Hight = kalmanFilter(&KFP_height, (float)Pressure2Hight);

        ANO_TC_Send05(Pressure2Hight * 100, 0, 1);											// ��λ���ױ�Ϊ���׷���
        ANO_TC_Send03(ROLL * 100, PITCH * (-100), YAW * (-100), 1);							// �������λ�������Ҫ����-1������Ҫ����100�ŷ���
        ANO_TC_Send01(ACCX_nb * 100, ACCY_nb * 100, ACCZ_nb * 100, GYROX, GYROY, GYROZ, 1); // ���ٶ�(m/s^2)�����ٶ�(��/s) (ACCZ_nb - 9.8f) * 100
        // printf("%.3f\t%.3f\t%.3f\r\n", (float)ROLL, (float)PITCH, (float)YAW);				// Openlog��ϻ�Ӵ�����

        TIM_SetCompare3(TIM3, 1850 + PITCH / 90 * 100); // PB0

        // USART1���յ���ָ��
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

// if(Receive_USART3_ok)//���ڽ������
// {
// 	for(sum=0,iii=0;iii<(Sonar_data[3]+4);iii++)//rgb_data[3]=3
// 	sum+=Sonar_data[iii];
// 	if(sum==Sonar_data[iii])//У����ж�
// 	{
// 		distance=(Sonar_data[4]<<8)|Sonar_data[5];
// 	}
// 	Receive_USART3_ok=0;//����������ϱ�־
// }
// ANO_TC_Send05(distance, 0, 1);
// send_com(0x01);//���Ͷ�ָ��

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



/* LShang������ϵת��
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
