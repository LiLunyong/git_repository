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




//*******************************************姿态解算**********************************************//
extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
extern volatile float AHRS_Roll, AHRS_Pitc, AHRS_Yawh;   // 偏航角，俯仰角，翻滚角   单位：度
extern volatile float C_nb_11, C_nb_12, C_nb_13, C_nb_21, C_nb_22, C_nb_23, C_nb_31, C_nb_32, C_nb_33;		// 坐标系旋转矩阵

void MahonyAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float ax, float ay, float az, float gx, float gy, float gz);





//*******************************************高度融合**********************************************//
typedef struct {
    float x[2];         // 状态向量 [高度; 速度]
    float P[2][2];      // 状态协方差
    float F[2][2];      // 状态转移矩阵
    float G[2];         // 控制输入矩阵
    float Q[2][2];      // 过程噪声协方差
    float H[2];         // 观测矩阵（1×2）
    float R;            // 观测噪声协方差
} KalmanAlt;

void kalman_init(KalmanAlt *kf, float dt, float var_accel, float var_alt);
void kalman_predict(KalmanAlt *kf, float accel);
void kalman_update(KalmanAlt *kf, float z);






//*******************************************PID控制**********************************************//
// PID结构体定义
typedef struct PID
{
	float P;	//参数
	float I;
	float D;

	float Error;	//比例项
	float Integral;	//积分项
	float Differ;	//微分项
	
	float PreError;	//上一次误差
	float I_limit;	//积分分离
	float I_range;	//积分限幅
	uint8_t I_limit_flag;	//积分分离标志
	
	float Pout;		//比例项输出
	float Iout;		//积分项输出
	float Dout;		//微分项输出
	float OutPut;	//PID控制器总输出
	
}PID_TYPE;

extern PID_TYPE 	PID_ROL_Angle;	//角度环PID
extern PID_TYPE 	PID_PIT_Angle;
extern PID_TYPE 	PID_YAW_Angle;
extern PID_TYPE 	PID_ROL_Rate;	//角速度环PID
extern PID_TYPE 	PID_PIT_Rate;
extern PID_TYPE 	PID_YAW_Rate;
extern PID_TYPE 	PID_Server_Posi;	//舵机PID

extern uint8_t  	aircraft_take_off;


void PID_Position_Cal(PID_TYPE* PID, float target, float measure);
void PID_Posi_Server_Cal(PID_TYPE* PID, float target, float measure);
void PID_Param_Init(void);




//*******************************************卡尔曼滤波**********************************************//
// 定义卡尔曼结构体
typedef struct
{
    float LastP; // 上次估算协方差 初始化值为0.02
    float Now_P; // 当前估算协方差 初始化值为0
    float out;   // 卡尔曼滤波器输出 初始化值为0
    float Kg;    // 卡尔曼增益 初始化值为0
    float Q;     // 过程噪声协方差 初始化值为0.001
    float R;     // 观测噪声协方差 初始化值为0.543
} KFP;           // Kalman Filter parameter

float kalmanFilter(KFP *kfp, float input);





//*******************************************定时器中断**********************************************//
void TIM2_Getsample_Init(u16 arr, u16 psc);








//*******************************************之前的不用了**********************************************//
//// 姿态解算结构体定义
//typedef struct Att_Algo_Param
//{
//	// 算法参数
//	float Kp;	// Kp比例增益 决定了加速度计的收敛速度 	// proportional gain governs rate of convergence toaccelerometer/magnetometer
//	float Ki;  	// Ki积分增益 决定了陀螺仪偏差的收敛速度	// integral gain governs rate of convergenceof gyroscope biases
//	double dt;   		// 采样时间
//	double half_dt;   	// 采样时间的一半
//	
//    // 状态变量
//	double norm;    	// 四元数的归一化用到的中间值
//	double q[4];		// 四元数
//	double q_last[4]; 	// 因为计算机计算是串行的，算了q[0]再直接算q[1]会用到最新的q[0],不能这样！所以要另存！整成并行的
//	double w_new[3];  	// 采样时刻的角速度
//	double w_last[3]; 	// 上一时刻的角速度
//	
//    // 中间计算结果
//	double k10, k11, k12, k13; 	// 龙格库塔算法用到的中间值
//	double k20, k21, k22, k23;
//	double k30, k31, k32, k33;
//	double k40, k41, k42, k43;
//	double vx, vy, vz; 			// 这些都是姿态解算误差计算的值，网上有说明
//	double ex, ey, ez;
//	double axf, ayf, azf;
//	double exInt, eyInt, ezInt;
//	
//	// 输出结果
//	double q2Roll, q2Pitc, q2Yawh;   // 偏航角，俯仰角，翻滚角

//}Attitude_Algorithm_Param;

//extern Attitude_Algorithm_Param 	Att_Algo_Param;
//void Attitude_Algorithm_Cal(Attitude_Algorithm_Param * Att, 
//							double ACCX, double ACCY, double ACCZ, 
//							double GYROX, double GYROY, double GYROZ);



//		// 姿态解算
//		Attitude_Algorithm_Cal(&Att_Algo_Param, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
//		// 姿态解算后得到的角度值
//		roll_deg = Att_Algo_Param.q2Roll;			// 角度       // 传感器测量值 
//		pitch_deg = Att_Algo_Param.q2Pitc;			// 单位 度
//		yaw_deg = Att_Algo_Param.q2Yawh;




#endif



