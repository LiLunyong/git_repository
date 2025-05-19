#ifndef __CONTROL_H
#define __CONTROL_H
#include "string.h"
#include "math.h"
#include "sys.h"
#include "usart.h"
#include "led.h"
#include "ANO_TC.h"
#include "sdio_sdcard.h"
#include "ff.h"
#include "exfuns.h"
#include "malloc.h"


#define PI 3.14159265358979323846f




//*******************************************��̬����**********************************************//
extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
extern volatile float AHRS_Roll, AHRS_Pitc, AHRS_Yawh;   // ƫ���ǣ������ǣ�������   ��λ����
extern volatile float C_nb_11, C_nb_12, C_nb_13, C_nb_21, C_nb_22, C_nb_23, C_nb_31, C_nb_32, C_nb_33;		// ����ϵ��ת����

void MahonyAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float ax, float ay, float az, float gx, float gy, float gz);





//*******************************************�߶��ں�**********************************************//
typedef struct {
    float x[2];         // ״̬���� [�߶�; �ٶ�]
    float P[2][2];      // ״̬Э����
    float F[2][2];      // ״̬ת�ƾ���
    float G[2];         // �����������
    float Q[2][2];      // ��������Э����
    float H[2];         // �۲����1��2��
    float R;            // �۲�����Э����
} KalmanAlt;

void kalman_init(KalmanAlt *kf, float dt, float var_accel, float var_alt);
void kalman_predict(KalmanAlt *kf, float accel);
void kalman_update(KalmanAlt *kf, float z);






//*******************************************PID����**********************************************//
// PID�ṹ�嶨��
typedef struct PID
{
	float P;	//����
	float I;
	float D;

	float Error;	//������
	float Integral;	//������
	float Differ;	//΢����
	
	float PreError;	//��һ�����
	float I_limit;	//���ַ���
	float I_range;	//�����޷�
	uint8_t I_limit_flag;	//���ַ����־
	
	float Pout;		//���������
	float Iout;		//���������
	float Dout;		//΢�������
	float OutPut;	//PID�����������
	
}PID_TYPE;

extern PID_TYPE 	PID_ROL_Angle;	//�ǶȻ�PID
extern PID_TYPE 	PID_PIT_Angle;
extern PID_TYPE 	PID_YAW_Angle;
extern PID_TYPE 	PID_ROL_Rate;	//���ٶȻ�PID
extern PID_TYPE 	PID_PIT_Rate;
extern PID_TYPE 	PID_YAW_Rate;
extern PID_TYPE 	PID_Server_Posi;	//���PID

extern uint8_t  	aircraft_take_off;


void PID_Position_Cal(PID_TYPE* PID, float target, float measure);
void PID_Posi_Server_Cal(PID_TYPE* PID, float target, float measure);
void PID_Param_Init(void);




//*******************************************�������˲�**********************************************//
// ���忨�����ṹ��
typedef struct
{
    float LastP; // �ϴι���Э���� ��ʼ��ֵΪ0.02
    float Now_P; // ��ǰ����Э���� ��ʼ��ֵΪ0
    float out;   // �������˲������ ��ʼ��ֵΪ0
    float Kg;    // ���������� ��ʼ��ֵΪ0
    float Q;     // ��������Э���� ��ʼ��ֵΪ0.001
    float R;     // �۲�����Э���� ��ʼ��ֵΪ0.543
} KFP;           // Kalman Filter parameter

float kalmanFilter(KFP *kfp, float input);





//*******************************************��ʱ���ж�**********************************************//
void TIM2_Getsample_Init(u16 arr, u16 psc);








//*******************************************֮ǰ�Ĳ�����**********************************************//
//// ��̬����ṹ�嶨��
//typedef struct Att_Algo_Param
//{
//	// �㷨����
//	float Kp;	// Kp�������� �����˼��ٶȼƵ������ٶ� 	// proportional gain governs rate of convergence toaccelerometer/magnetometer
//	float Ki;  	// Ki�������� ������������ƫ��������ٶ�	// integral gain governs rate of convergenceof gyroscope biases
//	double dt;   		// ����ʱ��
//	double half_dt;   	// ����ʱ���һ��
//	
//    // ״̬����
//	double norm;    	// ��Ԫ���Ĺ�һ���õ����м�ֵ
//	double q[4];		// ��Ԫ��
//	double q_last[4]; 	// ��Ϊ����������Ǵ��еģ�����q[0]��ֱ����q[1]���õ����µ�q[0],��������������Ҫ��棡���ɲ��е�
//	double w_new[3];  	// ����ʱ�̵Ľ��ٶ�
//	double w_last[3]; 	// ��һʱ�̵Ľ��ٶ�
//	
//    // �м������
//	double k10, k11, k12, k13; 	// ��������㷨�õ����м�ֵ
//	double k20, k21, k22, k23;
//	double k30, k31, k32, k33;
//	double k40, k41, k42, k43;
//	double vx, vy, vz; 			// ��Щ������̬�����������ֵ��������˵��
//	double ex, ey, ez;
//	double axf, ayf, azf;
//	double exInt, eyInt, ezInt;
//	
//	// ������
//	double q2Roll, q2Pitc, q2Yawh;   // ƫ���ǣ������ǣ�������

//}Attitude_Algorithm_Param;

//extern Attitude_Algorithm_Param 	Att_Algo_Param;
//void Attitude_Algorithm_Cal(Attitude_Algorithm_Param * Att, 
//							double ACCX, double ACCY, double ACCZ, 
//							double GYROX, double GYROY, double GYROZ);



//		// ��̬����
//		Attitude_Algorithm_Cal(&Att_Algo_Param, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
//		// ��̬�����õ��ĽǶ�ֵ
//		roll_deg = Att_Algo_Param.q2Roll;			// �Ƕ�       // ����������ֵ 
//		pitch_deg = Att_Algo_Param.q2Pitc;			// ��λ ��
//		yaw_deg = Att_Algo_Param.q2Yawh;




#endif



