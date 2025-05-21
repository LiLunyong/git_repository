#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "includes.h"					//ucos 使用	  

#include "ANO_TC.h"
#include "control.h"
#include "wdg.h"
#include "led.h"
#include "pwm.h"
#include "sdio_sdcard.h"
#include "ff.h"
#include "exfuns.h"
#include "malloc.h"
#include "IIC.h"
#include "spl06_001.h"
#include "ELRS.h"
#include "IMU_UART.h"
#include "fdilink_decode.h"
#include "FDILink.h"
#include "GNSS.h"	
#include "adc.h"
#include "distance.h"


//************************************************************************************************//
//****************************************初始化UCOS变量******************************************//
//***********************************************************************************************//
OS_TCB Task_TCB_init;		//初始化任务控制块
OS_TCB Task_TCB_height;
OS_TCB Task_TCB_sd;
OS_TCB Task_TCB_arhs;
OS_TCB Task_TCB_pid;

void task_init(void *parg);
void task_height(void *parg);
void task_sd(void *parg);
void task_arhs(void *parg);
void task_pid(void *parg);

CPU_STK task_stk_init[128];			//初始化任务的任务堆栈，大小为128字，也就是512字节   32位系统: 1字 = 4字节
CPU_STK task_stk_height[128];		//OSTaskCreate里记得改这个数，对应上，不然出现系统崩溃，看门狗重启
CPU_STK task_stk_sd[128];	// 
CPU_STK task_stk_arhs[512];	// 姿态解算函数里需要的变量比较多
CPU_STK task_stk_pid[128];	


OS_SEM		g_sem_sd;
OS_MUTEX    g_mutex_usart;



//************************************************************************************************//
//****************************************初始化全局变量******************************************//
//********************************* 尽量不要使用double 尽量使用float *****************************//
//***********************************************************************************************//
float Rad_to_Deg = 180.0f / PI;		// 角度和弧度的转换
float Deg_to_Rad = PI / 180.0f;

KalmanAlt 		kf_alt;
float spl_presure; // 温度补偿后的气压值 单位par 帕		//IIC接收的气压计数据
float spl_height;  // 解算后的气压高度值，单位m 米
float battery_voltage; //ADC检测到的电池电压 单位V 伏

float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z,mag_x, mag_y, mag_z;		// 传感器加速度计和陀螺仪的原始数据
float imu_T, imu_P,imu_PT;
uint64_t imu_Timestamp; 

float ACCX_n, ACCY_n, ACCZ_n;			//转换到地理坐标系上的加速度
float roll_deg, pitch_deg, yaw_deg;										// 姿态解算后得到的角度值
float target_roll, target_pitch, target_yaw;
float throttle;
float Moto_PWM1, Moto_PWM2, Moto_PWM3, Moto_PWM4;

char sd_buf[256];	  	// SD卡存储  SD卡的发送缓冲区
char sd_filename[20];	// 写入SD卡的，文件名

float server_target, server_measure;



//************************************************************************************************//
//****************************************初始化用到的函数*****************************************//
//************************************************************************************************//
void led_init(void);		//硬件初始化函数
void pwm_init(void);
void spl06_init(void);
void sd_init(void);
void kalman_alt_init(void);

void wireless_uart_Config(void);		//配置函数
void fdisystem_Config(void);




//主函数
int main(void)
{
	OS_ERR err;
	systick_init();  													//时钟初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);						//中断分组配置
	usart_init(115200);  				 									//串口初始化 115200 9600

	//OS初始化，它是第一个运行的函数,初始化各种的全局变量，例如中断嵌套计数器、优先级、存储器
	OSInit(&err);
	
	//创建初始化任务
	//把所有初始化内容放在task_init，启动OS进行任务调度后才能执行，以防提前运行到有关OS的导致崩溃
	OSTaskCreate(	(OS_TCB *)&Task_TCB_init,								//任务控制块
					(CPU_CHAR *)"Task_init",								//任务的名字
					(OS_TASK_PTR)task_init,									//任务函数
					(void *)0,												//传递参数
					(OS_PRIO)5,											 	//任务的优先级	
					(CPU_STK *)task_stk_init,								//任务堆栈基地址
					(CPU_STK_SIZE)128/10,									//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					(CPU_STK_SIZE)128,										//任务堆栈大小			
					(OS_MSG_QTY)0,											//禁止任务消息队列
					(OS_TICK)0,												//默认时间片长度																
					(void  *)0,												//不需要补充用户存储区
					(OS_OPT)OS_OPT_TASK_NONE,								//没有任何选项
					&err													//返回的错误码
				);
	

	//创建信号量
	OSSemCreate(&g_sem_sd, "g_sem_sd", 0, &err);    // 第三个参数不为0，不然一开始用等不到信号量，为0一般都用于任务同步使用
	OSMutexCreate(&g_mutex_usart, "g_mutex_usart", &err);
	
					
	//启动OS，进行任务调度
	OSStart(&err);
	while(1);
}


/**************************************************************************************/
/***********************************UCOS任务函数***************************************/
/**************************************************************************************/
void task_init(void *parg)
{
	int wdg_i;
	OS_ERR err;
	CPU_SR_ALLOC();		// 如果使用CPU_CRITICAL_ENTER();就必须要CPU_SR_ALLOC();
	
	
//	wireless_uart_Config();	// 配置无线数传的波特率  //仅需配置一次，配置完记得注释了，而且另一个数传（电脑端）也需要相同的配置
	led_init();
	IIC_Init(); // I2C接口功能初始化   这个必须先初始化才能初始化对应的有关硬件
	spl06_init();
	sd_init();
    pwm_init(); // TIM3 TIM4 总共八个PWM初始化
	Adc_Init();
	
//	Initial_UART2(420000);	// 初始化ELRS串口  //如果遥控器连接不上,看门狗会失效，来不及喂狗
	Initial_UART3(921600);	// 初始化IMU串口
	Initial_UART4(38400);	// 初始化GNSS串口
	
	//无法进入配置模式的原因可能是级间排线接触不良 //需要fdiSetReboot才能 
	//配置完重启后无效可能是因为延时的时间不够
	//配置完记得在USART3_IRQHandler里注释掉回传到uart1的代码
//	fdisystem_Config();

	PID_Param_Init();
	kalman_alt_init();
	
	//创建任务1
	OSTaskCreate(	(OS_TCB *)&Task_TCB_height,								//任务控制块，等同于线程id
					(CPU_CHAR *)"Task_height",								//任务的名字，名字可以自定义的
					(OS_TASK_PTR)task_height,								//任务函数，等同于线程函数
					(void *)0,												//传递参数，等同于线程的传递参数
					(OS_PRIO)6,											 	//任务的优先级6		
					(CPU_STK *)task_stk_height,								//任务堆栈基地址
					(CPU_STK_SIZE)128/10,									//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					(CPU_STK_SIZE)128,										//任务堆栈大小			
					(OS_MSG_QTY)0,											//禁止任务消息队列
					(OS_TICK)0,												//默认时间片长度																
					(void  *)0,												//不需要补充用户存储区
					(OS_OPT)OS_OPT_TASK_NONE,								//没有任何选项
					&err													//返回的错误码
				);

	//创建任务2
	OSTaskCreate(	(OS_TCB *)&Task_TCB_sd,									//任务控制块
					(CPU_CHAR *)"Task_sd",									//任务的名字
					(OS_TASK_PTR)task_sd,									//任务函数
					(void *)0,												//传递参数
					(OS_PRIO)5,											 	//任务的优先级7		
					(CPU_STK *)task_stk_sd,									//任务堆栈基地址
					(CPU_STK_SIZE)128/10,									//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					(CPU_STK_SIZE)128,										//任务堆栈大小			
					(OS_MSG_QTY)0,											//禁止任务消息队列
					(OS_TICK)0,												//默认时间片长度																
					(void  *)0,												//不需要补充用户存储区
					(OS_OPT)OS_OPT_TASK_NONE,								//没有任何选项
					&err													//返回的错误码
				);
	
	//创建任务3
	OSTaskCreate(	(OS_TCB *)&Task_TCB_arhs,								//任务控制块
					(CPU_CHAR *)"Task_arhs",								//任务的名字
					(OS_TASK_PTR)task_arhs,									//任务函数
					(void *)0,												//传递参数
					(OS_PRIO)6,											 	//任务的优先级7		
					(CPU_STK *)task_stk_arhs,								//任务堆栈基地址
					(CPU_STK_SIZE)512/10,									//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					(CPU_STK_SIZE)512,										//任务堆栈大小			
					(OS_MSG_QTY)0,											//禁止任务消息队列
					(OS_TICK)0,												//默认时间片长度																
					(void  *)0,												//不需要补充用户存储区
					(OS_OPT)OS_OPT_TASK_NONE,								//没有任何选项
					&err													//返回的错误码
				);
					
	//创建任务4
	OSTaskCreate(	(OS_TCB *)&Task_TCB_pid,								//任务控制块
					(CPU_CHAR *)"Task_pid",									//任务的名字
					(OS_TASK_PTR)task_pid,									//任务函数
					(void *)0,												//传递参数
					(OS_PRIO)6,											 	//任务的优先级7		
					(CPU_STK *)task_stk_pid,								//任务堆栈基地址
					(CPU_STK_SIZE)128/10,									//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					(CPU_STK_SIZE)128,										//任务堆栈大小			
					(OS_MSG_QTY)0,											//禁止任务消息队列
					(OS_TICK)0,												//默认时间片长度																
					(void  *)0,												//不需要补充用户存储区
					(OS_OPT)OS_OPT_TASK_NONE,								//没有任何选项
					&err													//返回的错误码
				);
					
	IWDG_Init(4, 1000);// IWDG_Feed(); // 喂狗// 分频数为4*2^4=64,32kHz/64,重载值为500,溢出时间为1s//时间计算(大概):Tout=((4*2^prer)*rlr)/32 (ms)
	
	while(1)
	{
		for(wdg_i = 0; wdg_i < 9; wdg_i++) {
			delay_ms(100);
			
			CPU_CRITICAL_ENTER();   // OS_CFG_ISR_POST_DEFERRED_EN 为1就不关中断
			IWDG_Feed(); 	// 喂狗
			CPU_CRITICAL_EXIT();     //立即退出临界区
		}
		
        delay_ms(50);
        LED_PC13 = !LED_PC13;
        delay_ms(50);
        LED_PC13 = !LED_PC13;
		
		CPU_CRITICAL_ENTER();   // OS_CFG_ISR_POST_DEFERRED_EN 为1就不关中断
		IWDG_Feed(); 	// 喂狗
		CPU_CRITICAL_EXIT();     //立即退出临界区
	}
}






void task_height(void *parg)
{
	uint16_t 	volt_i;
	uint16_t 	count_spl_i = 0;
	OS_ERR 		err;


	while(1)
	{
		if(count_spl_i < 500)		// 等姿态解算结果稳定以后再进行高度的融合 500 * 10ms = 5s
		{	
			count_spl_i += 1;
		}
		else{
			//SPL06气压计读取
			spl_presure = user_spl0601_get_presure(); 							// 温度补偿后的气压值 单位par 帕
			spl_height = 44300 * (1 - pow(spl_presure / 101325, 1 / 5.256f));   	// 解算后的气压高度值，单位m 米
			
			
			//电池电压测量值
			battery_voltage = 0;
			for(volt_i = 0; volt_i < 6000; volt_i++) {
				battery_voltage += ADC_Value_Buffer[volt_i] & 0x0FFF;
			}
			battery_voltage = battery_voltage / 6000 * (3.3f / 4096) * 10 ; // 取平均，10是倍数（乘以10才是真正的电压）
			//battery_voltage = (float)Get_Adc_Average(ADC_Channel_7, 20) * (3.3 / 4096) * 10; // 获取通道5的转换值，20次取平均，10是倍数（乘以10才是真正的电压）
			
			
			
			//卡尔曼滤波融合（加速度计和气压计）
			kalman_predict(&kf_alt, ACCZ_n - 9.72f);  // 输入去重重力后的加速度
			kalman_update(&kf_alt, spl_height);    // 气压计高度   //printf("Step → 高度=%.2f m, 速度=%.2f m/s\n", kf_alt.x[0], kf_alt.x[1]);



//			//**************************************向匿名上位机发送数据*****************************************//
//			OSMutexPend(&g_mutex_usart, 0, OS_OPT_PEND_BLOCKING, NULL, &err);	//阻塞等待互斥锁
//			ANO_TC_Send02(mag_x, mag_y, mag_z, spl_height * 100, imu_T * 100, 1, 1); // 罗盘、气压、温度传感器数据
//			ANO_TC_Send05(kf_alt.x[0] * 100, 0, 1); 		// 算出来的高度			// 单位是米, 变为厘米发送
//			ANO_TC_Send0D(battery_voltage * 100, 0);
//			OSMutexPost(&g_mutex_usart, OS_OPT_POST_NONE, &err);				//释放互斥锁
		}

//		printf("gnss_nmea0183: 经度：%f\t 维度：%f !\r\n", gnss_nmea0183.gpsData.location.lng, gnss_nmea0183.gpsData.location.lat);
		delay_ms(10);
	}
}




void task_sd(void *parg)
{
	OS_ERR 		err;
	FRESULT 	res;
	static int 	sync_cnt = 0;
	
	
	while(1)
	{
		// 等待那边给信号才往下执行
		OSSemPend(&g_sem_sd, 0, OS_OPT_PEND_BLOCKING, NULL, &err);	// 阻塞等待信号量 
		
		// 上位机显示  显示这个任务正在运行中	// 即电压显示0-99
		OSMutexPend(&g_mutex_usart, 0, OS_OPT_PEND_BLOCKING, NULL, &err);	//阻塞等待互斥锁
		ANO_TC_Send0D(sync_cnt * 100, 0);
		OSMutexPost(&g_mutex_usart, OS_OPT_POST_NONE, &err);				//释放互斥锁
		
		//*********SD卡的同步操作f_sync(file)不能被其他任务打断，设置优先级高一点，不然容易卡住出不来************//
		//****************************SD卡写入不耗时，但打开、关闭和同步操作特别耗时***************************//
		//******************写入（若100字节数据，0.01-0.1ms），打开（20ms），关闭（20ms），同步（5ms）************//
		// 每100次写入就同步一次, 因为同步非常耗时间（5ms）
		if (++sync_cnt >= 100) {
			res = f_sync(file);      		// 确保缓冲区内容写入扇区		// 不用关闭操作,关闭太耗时间了（20ms）
			if (res != FR_OK) { } else { }	// 返回值不作处理
			sync_cnt = 0;		// 清零		// 每100次写入就同步一次
		}
	}
}




void task_arhs(void *parg)
{
	OS_ERR 		err;
	
	// 打开文件
	f_open(file, sd_filename, FA_WRITE); 	// 文件指针+绝对路径+读写方式,全都要对
	

	while(1)
	{
		// 传感器加速度计和陀螺仪、磁力计的原始数据
		acc_x = -IMUData.Accelerometer_X;		// 单位 m/s^2
		acc_y = -IMUData.Accelerometer_Y;		//取负号，解算出来的才是正常的
		acc_z = -IMUData.Accelerometer_Z;
		gyro_x = IMUData.Gyroscope_X;			// 单位 rad/s
		gyro_y = IMUData.Gyroscope_Y;
		gyro_z = IMUData.Gyroscope_Z;
		mag_x = IMUData.Magnetometer_X;
		mag_y = IMUData.Magnetometer_Y;
		mag_z = IMUData.Magnetometer_Z;
		imu_T = IMUData.IMU_Temperature;
		imu_P = IMUData.Pressure;
		imu_PT = IMUData.Pressure_Temperature;
		imu_Timestamp = IMUData.Timestamp;
		
		// 姿态解算
//		MahonyAHRSupdate(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z);
		MahonyAHRSupdateIMU(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);		// 不采用磁力计进行姿态解算
		// 姿态解算后得到的角度值
		roll_deg = AHRS_Roll;			// 角度       // 传感器测量值 
		pitch_deg = AHRS_Pitc;			// 单位 度
		yaw_deg = AHRS_Yawh;
		
		
//		// 读取模块的角度值
//		roll_deg = Euler_data.Roll * Rad_to_Deg;			// 角度       // 传感器测量值 
//		pitch_deg = Euler_data.Pitch * Rad_to_Deg;			// 单位 度
//		yaw_deg = Euler_data.Heading * Rad_to_Deg;
		
		
		ACCX_n = C_nb_11 * acc_x + C_nb_12 * acc_y + C_nb_13 * acc_z; // ACCX_n是在地理坐标系上的加速度，ax是机体坐标系的
		ACCY_n = C_nb_21 * acc_x + C_nb_22 * acc_y + C_nb_23 * acc_z; // 单位是m/s^2
		ACCZ_n = C_nb_31 * acc_x + C_nb_32 * acc_y + C_nb_33 * acc_z;
		
	
		//**************************************向匿名上位机发送数据*****************************************//
		OSMutexPend(&g_mutex_usart, 0, OS_OPT_PEND_BLOCKING, NULL, &err);	//阻塞等待互斥锁
		ANO_TC_Send03(roll_deg * 100, pitch_deg * 100, yaw_deg * 100, 1);
//		ANO_TC_Send01(ACCX_n * 1000, ACCY_n * 1000, ACCZ_n * 1000, gyro_x * 1000, gyro_y * 1000, gyro_z * 1000, 1); 
//		ANO_TC_Send01(acc_x * 1000, acc_y * 1000, acc_z * 1000, gyro_x * 1000, gyro_y * 1000, gyro_z * 1000, 1);	// 加速度(单位m/s^2)，角速度(单位°/s) 
//		ANO_TC_Send02((unsigned)stk_free, (unsigned)stk_used, 0, 0 * 100, 0 * 100, 1, 1); // 罗盘、气压、温度传感器数据
//		ANO_TC_Send04(q[0] * 10000, q[1] * 10000, q[2] * 10000, q[3] * 10000, 1);
//		ANO_TC_Send30(1, 0, 108.914937 * 10000000, 34.246872 * 10000000, 0 / 10, 0, 0, 0, 0 / 100, 0 / 100, 0 / 100);
//		ANO_TC_Send32(position_x * 100, position_y * 100, position_z * 100);   //发送xyz位置信息		// 单位是米, 变为厘米发送
		OSMutexPost(&g_mutex_usart, OS_OPT_POST_NONE, &err);				//释放互斥锁
		
		
		//*****************************************向SD卡写入数据******************************************//
		sprintf(sd_buf, "%.4f\t%.4f\t%.4f\r\n", (float)roll_deg, (float)pitch_deg, (float)yaw_deg);	  // 先把变量转成字符串
		//f_lseek(file, file->fsize);		// 指针指到文件尾，追加，如果不是关闭重新打开，就不用这个语句f_lseek
		f_write(file, sd_buf, strlen(sd_buf), &bw); // 写入一行数据
			
		
		// 通知那边，通知就行了，一定次数后会同步，只需要那边同步
		// 如果用消息队列，把消息传到那边再写入，反而容易写入出问题（写入不全）
		OSSemPost(&g_sem_sd, OS_OPT_POST_1, &err);		//释放信号量
		
		
		delay_ms(10);
	}
//	f_close(file);	// 关闭文件 
}


void task_pid(void *parg)
{
//	OS_ERR err;

	while(1)
	{
		
//		// 遥控器摇杆——>也就是 目标角度值
//		target_roll = ((double)ELRS_ch1 - 992) / 818.0 * 20;		// 遥控器目标角度值	    // 原通道值174—1811
//		target_pitch = ((double)ELRS_ch2 - 992) / 818.0 * 20;		// 两边各20度 [-20, 20]
//		target_yaw = ((double)ELRS_ch4 - 992) / 818.0 * 20;
//		throttle = ((double)ELRS_ch3 - 174) / 1637.0 * 1000 + 1000;	// 电调的PWM占空比应该是1000-2000之间


//		// PID角度环    					// PID控制 
//		PID_Position_Cal(&PID_ROL_Angle, target_roll, roll_deg);			// 输入角度 输出角速度
//		PID_Position_Cal(&PID_PIT_Angle, target_pitch, pitch_deg);
//		// PID_Position_Cal(&PID_YAW_Angle, target_yaw, yaw_deg);
//		
//		// PID角速度环    
//		PID_Position_Cal(&PID_ROL_Rate, PID_ROL_Angle.OutPut, gyro_x * Rad_to_Deg);	// 输入角度环的输出，输出电机控制量
//		PID_Position_Cal(&PID_PIT_Rate, PID_PIT_Angle.OutPut, gyro_y * Rad_to_Deg);
//		PID_Position_Cal(&PID_YAW_Rate, target_yaw * PID_YAW_Angle.P, gyro_z * Rad_to_Deg);
//		
//		
//		// 电机动力分配    
//		if(aircraft_take_off == 1)
//		{
//			Moto_PWM1 = throttle + PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;
//			Moto_PWM2 = throttle - PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;
//			Moto_PWM3 = throttle - PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;
//			Moto_PWM4 = throttle + PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;
//		}
//		else{
//			Moto_PWM1 = 0;
//			Moto_PWM2 = 0;
//			Moto_PWM3 = 0;
//			Moto_PWM4 = 0;
//		}
//		printf("Moto_PWM1:%d\tMoto_PWM2:%d\tMoto_PWM3:%d\tMoto_PWM4:%d\t\r\n", (int)Moto_PWM1, (int)Moto_PWM2, (int)Moto_PWM3, (int)Moto_PWM4);


//		printf("PID_ROL_Angle.OutPut:%f\ttarget_roll:%f\troll_deg:%f\t\r\n", (float)PID_ROL_Angle.OutPut, (float)target_roll, (float) roll_deg);

		// PWM控制舵机
//		TIM_SetCompare1(TIM3, 500 + ((float)ELRS_ch1 - 174) / 1637 * 2000); 	// PB0
//		TIM_SetCompare2(TIM3, 500 + ((float)ELRS_ch2 - 174) / 1637 * 2000);	// 值都是174到1811之间(差值: 1637)
//		TIM_SetCompare3(TIM3, 500 + ((float)ELRS_ch3 - 174) / 1637 * 2000);	// PWM1——PWM8
//		TIM_SetCompare4(TIM3, 500 + ((float)ELRS_ch4 - 174) / 1637 * 2000);
//		TIM_SetCompare1(TIM1, 500 + ((float)ELRS_ch5 - 174) / 1637 * 2000);
//		TIM_SetCompare2(TIM1, 500 + ((float)ELRS_ch6 - 174) / 1637 * 2000);
//		TIM_SetCompare3(TIM1, 500 + ((float)ELRS_ch7 - 174) / 1637 * 2000);
//		TIM_SetCompare4(TIM1, 500 + ((float)ELRS_ch8 - 174) / 1637 * 2000);

//		server_target = ((double)ELRS_ch3 - 174) / 1637.0 * 2000 + 500;				//目标值就是摇杆信号值				
//		PID_Posi_Server_Cal(&PID_Server_Posi, server_target, server_measure);		//测量值就是当前输出值
//		server_measure += PID_Server_Posi.OutPut;
//		if (server_measure < 500){server_measure = 500;}		// 输出限幅
//		else if (server_measure > 2500){server_measure = 2500;}
//		TIM_SetCompare4(TIM1, server_measure);
//		ANO_TC_Send05(server_target * 100,0, 1);
//		ANO_TC_Send05(server_measure * 100, 0, 1);
		
		
		
		delay_ms(10);
	}
}





/**************************************************************************************/
/***********************************硬件执行的函数**************************************/
/**************************************************************************************/
void led_init(void)
{
	u16 i = 0;
    LED_Init();
//    EXTI11_Init();
    for (i = 0; i < 20; i++) // 久一点,等手离远一点
    {
        delay_ms(100);
        LED_PC13 = !LED_PC13;
        //BB_PD10 = !BB_PD10;
        LED_PD12 = !LED_PD12;
        LED_PD13 = !LED_PD13;
    }
    LED_PC13 = 1;
    BB_PD10 = 1;
    LED_PD12 = 0;
	LED_PD13 = 0;
	printf("led_init successful!!!!!!\r\n");
}


void pwm_init(void)
{ 
	// 84Mhz,计数频率为84Mhz/84,计数1000000则为1s，重装载值20000，所以PWM频率为 50hz.
	// 计数1000000则为1s，计数1则为1us，舵机的脉宽在0.5ms和2.5ms之间，所以需要控制在500-2500之间
    TIM3_PWM_Init(1000000 / 50 - 1, 84 - 1);
	
	
	// 高级定时器，频率是上面的两倍
	// 84 * 2 Mhz,计数频率为(84 * 2)Mhz/(84 * 2),计数1000000则为1s，重装载值20000，所以PWM频率为50hz.
	// 计数1000000则为1s，计数1则为1us，电调的脉宽在1000us和2000us之间，所以需要控制在1000-2000之间
    TIM1_PWM_Init(1000000 / 50 - 1, 84 * 2 - 1);
	
	
//	TIM2_Getsample_Init(100000 / 50 - 1, 840 - 1); // (1)要最后初始化,太早采样的值是异常的  (2)不能太大到100,不然读数据有问题
//  TIM4_PWM_Init(100000 / 50 - 1, 840 - 1);	// 暂时，主控板上没有设置有连线
	printf("pwm_init successful!!!!!!\r\n");
}



void spl06_init(void)
{
    if (spl0601_init() != 0) // SPL06初始化
    {
        printf("SPL06初始化失败!\r\n\r\n");
    }
	else
	{
		printf("spl06_init successful!!!!!!\r\n");
	}
}

void sd_init(void)
{
    // char buf1[512]; 这个数组的大小关系到能不能初始化成功
	//内存问题记得改malloc.h  #define MEM1_MAX_SIZE	  80*1024  //最大管理内存 100K  //  原来是100*1024
    u32 total, free;
    int res;
	DIR dir;               /* Directory object */
	FILINFO fno;           /* File information */
	char *ext;
	UINT txt_count = 0;
	int scan_counter = 0;		// 避免陷入死循环
	
	
    res = SD_Init();
    LED_PD13 = 1;
    while (res) // 检测不到SD卡
    {
        printf("SD Card Init Error: %d !\r\n", res);
        res = SD_Init();
    }
    //show_sdcard_info();                                                // 打印SD卡相关信息
    printf("SD       Size:  %d   MB\r\n", (int)(SDCardInfo.CardCapacity >> 20)); // 显示SD卡容量
    my_mem_init(SRAMIN);                                               // 初始化内部内存池
    my_mem_init(SRAMCCM);                                              // 初始化CCM内存池
    exfuns_init();                                                     // 为fatfs相关变量申请内存
    f_mount(fs[0], "0:", 1);                                           // 挂载SD卡
    while (exf_getfree("0", &total, &free))                            // 得到SD卡的总容量和剩余容量
    {
        printf("SD Card Fatfs Error!\r\n");
        delay_ms(200);
    }
    printf("SD Total Size:  %d   MB\r\n", total >> 10); // 显示SD卡总容量 MB
    printf("SD  Free Size:  %d   MB\r\n", free >> 10);  // 显示SD卡剩余容量 MB

	
	// 统计SD卡文件夹下的txt文件数量，以命名新的txt文件
    /* 1. 打开根目录 */
    res = f_opendir(&dir, "0:/");
    if (res != FR_OK) {
        /* 打开失败，视需要返回错误码或打印日志 */
		printf("SD f_opendir != FR_OK\r\n");
    }
    /* 2. 遍历目录 */
    for (;;) {
        res = f_readdir(&dir, &fno);    /* 读取下一个条目 */
        if (res != FR_OK || fno.fname[0] == 0 || scan_counter++ >= 1000) break;  /* 结束或出错 */

        /* 3. 只统计普通文件 */
        if (!(fno.fattrib & AM_DIR)) {
            /* 查找扩展名 */
            ext = strrchr(fno.fname, '.');
            if (ext) {
                /* 忽略大小写比较 .txt */
                if (strcasecmp(ext, ".txt") == 0) {
                    txt_count++;
                }
            }
        }
    }
    /* 4. 关闭目录 */
    f_closedir(&dir);
	printf("SD txt_count = %d\r\n", txt_count);
//	printf("SD scan_counter = %d\r\n", scan_counter);

	// 根据统计的txt文件数量，创建并命名新的txt文件
	sprintf(sd_filename, "0:/test_%d.txt", txt_count + 1);
    f_open(file, sd_filename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ); // 创建文件//文件指针+绝对路径+读写方式,全都要对
    f_close(file);
    LED_PD13 = 0;
	printf("SD filename = %s\r\n", sd_filename);
    delay_ms(200);
	printf("sd_init successful!!!!!!\r\n");
    delay_ms(200);
}


void kalman_alt_init(void)
{
    kalman_init(&kf_alt, 0.029, 0.9, 0.10);	// 分别是 dt = 0.02; var_accel=0.1, var_alt=1.0  协方差越大，就越不信任这个数据，反之则越信任
	//SPL06气压计读取
	spl_presure = user_spl0601_get_presure(); 							// 温度补偿后的气压值 单位par 帕
	spl_height = 44300 * (1 - pow(spl_presure / 101325, 1 / 5.256));   	// 解算后的气压高度值，单位m 米
	kf_alt.x[0] = spl_height;
}


void wireless_uart_Config(void)
{
	/***************************** 配置无线数传的波特率 *********************************/
	int i=0;
	char uart_set_cmd[] = {0xC0, 0x00, 0x00, 0x38, 0x00, 0x40};	 //配置115200波特率，即0x38，默认的9600波特率是0x18
	usart_init(9600); // PA9和PA10的USART1初始化
	
	GPIO_SetBits(GPIOA, GPIO_Pin_11);				// MDO MD1  配置模式
	GPIO_SetBits(GPIOA, GPIO_Pin_12);
	delay_ms(500);
	
	for (i = 0; i < 6; i++)
		UsartSendByte(USART1, uart_set_cmd[i]); 	// 发送设置的参数
	delay_ms(500);	//延迟很重要！！！！！！！！！
	
	for (i = 0; i < 3; i++)
		UsartSendByte(USART1, 0xC3); 				// 发送复位指令
	delay_ms(500);
	
	usart_init(115200);
//	while(1){
//		//配置完以后死循环，只需要配置一次，正常使用情况下，这个函数不需要执行，注释掉，而且另一个数传（电脑端）也需要相同的配置
//		delay_ms(100);
//		LED_PC13 = !LED_PC13;
//	}

}


void fdisystem_Config(void)
{
	/***************************** 配置IMU输出内容 *********************************/
	fdiComSetConfig();		//两个 *#OK
	
//			You can choose the frequency you want from the following options
//			0   :No output
//			1   :1Hz
//			2   :2Hz
//			5   :5Hz
//			10  :10Hz
//			20  :20Hz
//			50  :50Hz
//			100 :100Hz
//			200 :200Hz
//			400 :400Hz (IMU data only)
	 
	//配置发送的数据包内容及频率			//(1.需要数据包内容，2.频率)
//	fdiComSetConfigPacketSentMsg(VERSIONDATA, 0);	
	fdiComSetConfigPacketSentMsg(MSG_IMU, 200);	
//	fdiComSetConfigPacketSentMsg(MSG_AHRS, 0);	
//	fdiComSetConfigPacketSentMsg(MSG_INSGPS, 0);	
//	fdiComSetConfigPacketSentMsg(MSG_SYS_STATE, 0);	
//	fdiComSetConfigPacketSentMsg(MSG_UNIX_TIME, 0);	
//	fdiComSetConfigPacketSentMsg(MSG_FORMAT_TIME, 0);	
//	fdiComSetConfigPacketSentMsg(MSG_STATUS, 0);
//	fdiComSetConfigPacketSentMsg(MSG_POS_STD_DEV, 0);
//	fdiComSetConfigPacketSentMsg(MSG_VEL_STD_DEV, 0);
//	fdiComSetConfigPacketSentMsg(MSG_EULER_ORIEN_STD_DEV, 0);
//	fdiComSetConfigPacketSentMsg(MSG_QUAT_ORIEN_STD_DEV, 0);
//	fdiComSetConfigPacketSentMsg(MSG_RAW_SENSORS, 0);
//	fdiComSetConfigPacketSentMsg(MSG_RAW_GNSS, 0);
//	fdiComSetConfigPacketSentMsg(MSG_SATELLITE, 0);
//	fdiComSetConfigPacketSentMsg(MSG_DETAILED_SATELLITE, 0);
//	fdiComSetConfigPacketSentMsg(MSG_GEODETIC_POS, 0);
//	fdiComSetConfigPacketSentMsg(MSG_ECEF_POS, 0);
//	fdiComSetConfigPacketSentMsg(MSG_UTM_POS, 0);
//	fdiComSetConfigPacketSentMsg(MSG_NED_VEL, 0);
//	fdiComSetConfigPacketSentMsg(MSG_BODY_VEL, 0);
//	fdiComSetConfigPacketSentMsg(MSG_ACCELERATION, 0);
//	fdiComSetConfigPacketSentMsg(MSG_BODY_ACCELERATION, 0);
	fdiComSetConfigPacketSentMsg(MSG_EULER_ORIEN, 200);
//	fdiComSetConfigPacketSentMsg(MSG_QUAT_ORIEN, 0);
//	fdiComSetConfigPacketSentMsg(MSG_DCM_ORIEN, 0);
//	fdiComSetConfigPacketSentMsg(MSG_ANGULAR_VEL, 0);
//	fdiComSetConfigPacketSentMsg(MSG_ANGULAR_ACC, 0);
//	fdiComSetConfigPacketSentMsg(MSG_RUNNING_TIME, 0);
//	fdiComSetConfigPacketSentMsg(MSG_LOCAL_MAG_FIELD, 0);
//	fdiComSetConfigPacketSentMsg(MSG_ODOMETER_STATE, 0);
//	fdiComSetConfigPacketSentMsg(MSG_GEOID_HEIGHT, 0);
//	fdiComSetConfigPacketSentMsg(MSG_RTCM_CORRECTIONS, 0);
//	fdiComSetConfigPacketSentMsg(MSG_WIND, 0);
//	fdiComSetConfigPacketSentMsg(MSG_HEAVE, 0);
//	fdiComSetConfigPacketSentMsg(MSG_RAW_SATELLITE, 0);
//	fdiComSetConfigPacketSentMsg(MSG_GNSS_DUAL_ANT, 0);
//	fdiComSetConfigPacketSentMsg(MSG_GIMBAL_STATE, 0);
//	fdiComSetConfigPacketSentMsg(MSG_AUTOMOTIVE, 0);
//	fdiComSetConfigPacketSentMsg(MSG_PACKET_TIMER_PERIOD, 0);
//	fdiComSetConfigPacketSentMsg(MSG_PACKETS_PERIOD, 0);
//	fdiComSetConfigPacketSentMsg(MSG_INSTALL_ALIGN, 0);
//	fdiComSetConfigPacketSentMsg(MSG_FILTER_OPTIONS, 0);
//	fdiComSetConfigPacketSentMsg(MSG_GPIO_CONFIG, 0);
//	fdiComSetConfigPacketSentMsg(MSG_MAG_CALI_VALUES, 0);
//	fdiComSetConfigPacketSentMsg(MSG_MAG_CALI_CONFIG, 0);
//	fdiComSetConfigPacketSentMsg(MSG_MAG_CALI_STATUS, 0);
//	fdiComSetConfigPacketSentMsg(MSG_ODOMETER_CONFIG, 0);
//	fdiComSetConfigPacketSentMsg(MSG_SET_ZERO_ORIENT_ALIGN, 0);
//	fdiComSetConfigPacketSentMsg(MSG_REF_POINT_OFFSET, 0);
//	fdiComSetConfigPacketSentMsg(MSG_USER_DATA, 0);
//	fdiComSetConfigPacketSentMsg(MSG_BAUD_RATES, 0);
//	fdiComSetConfigPacketSentMsg(MSG_SENSOR_RANGES, 0);
//	fdiComSetConfigPacketSentMsg(MSG_GPIO_OUTPUT_CONFIG, 0);
//	fdiComSetConfigPacketSentMsg(MSG_GPIO_INPUT_CONFIG, 0);
//	fdiComSetConfigPacketSentMsg(MSG_DUAL_ANT, 0);
//	fdiComSetConfigMsg();
	
	//查询数据
//	fdiComGetParam("MSG_RAW_SENSORS");
//	fdiComGetParam("MSG_RAW_GNSS");
	
	//配置保存
	fdiSetSave();
	//退出配置
//	fdiSetDeconfig();	//看用户手册，退出配置和配置重启不能同时用！！！！！！！！！！
	//配置重启
	fdiSetReboot();		//两个 *#OK
}











//***********************************************************




//		OSTaskSuspend(&Task_TCB_1, &err);
//		OSSemPend(&g_sem, 0, OS_OPT_PEND_BLOCKING, NULL, &err);		//阻塞等待信号量
//		OSMutexPend(&g_mutex, 0, OS_OPT_PEND_BLOCKING, NULL, &err);

//		printf("task2 is running ...\r\n");

//		OSTaskResume(&Task_TCB_1, &err);
//		OSSemPost(&g_sem, OS_OPT_POST_1, &err);		//释放信号量
//		OSMutexPost(&g_mutex, OS_OPT_POST_NONE, &err);


//OS_Q    	g_queue_sd;
//	OSQCreate(&g_queue_sd, "g_queue_sd", 16, &err); 	//创建消息队列，支持16条消息
//		// 消息队列发送
//		OSQPost(&g_queue_sd, (void *)sd_buf, strlen(sd_buf), OS_OPT_POST_FIFO, &err);	
//	OS_MSG_SIZE		msg_size;	//消息的大小
//	char *p = NULL;
//		p = OSQPend(&g_queue_sd, 0, OS_OPT_PEND_BLOCKING, &msg_size, NULL, &err);		//等待消息队列
//		if(p && msg_size)		//p指针有效，且msg_size>0
//		{
//			myfree(SRAMIN, p);	// 释放动态内存	//不释放会出现: 写入的都是空白数据
//		}





		// 传感器加速度计和陀螺仪、磁力计的原始数据
//		acc_x = -Raw_Sensor_data.Accelerometer_X;		// 单位 m/s^2
//		acc_y = -Raw_Sensor_data.Accelerometer_Y;		//取负号，解算出来的才是正常的
//		acc_z = -Raw_Sensor_data.Accelerometer_Z;
//		gyro_x = Raw_Sensor_data.Gyroscope_X;			// 单位 rad/s
//		gyro_y = Raw_Sensor_data.Gyroscope_Y;
//		gyro_z = Raw_Sensor_data.Gyroscope_Z;
//		mag_x = Raw_Sensor_data.Magnetometer_X;
//		mag_y = Raw_Sensor_data.Magnetometer_Y;
//		mag_z = Raw_Sensor_data.Magnetometer_Z;
//		imu_T = Raw_Sensor_data.IMU_Temperature;
//		imu_P = Raw_Sensor_data.Pressure;
//		imu_PT = Raw_Sensor_data.Pressure_Temperature;
		


