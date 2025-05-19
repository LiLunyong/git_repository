#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "includes.h"					//ucos ʹ��	  

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
//****************************************��ʼ��UCOS����******************************************//
//***********************************************************************************************//
OS_TCB Task_TCB_init;		//��ʼ��������ƿ�
OS_TCB Task_TCB_height;
OS_TCB Task_TCB_sd;
OS_TCB Task_TCB_arhs;
OS_TCB Task_TCB_pid;

void task_init(void *parg);
void task_height(void *parg);
void task_sd(void *parg);
void task_arhs(void *parg);
void task_pid(void *parg);

CPU_STK task_stk_init[128];			//��ʼ������������ջ����СΪ128�֣�Ҳ����512�ֽ�   32λϵͳ: 1�� = 4�ֽ�
CPU_STK task_stk_height[128];		//OSTaskCreate��ǵø����������Ӧ�ϣ���Ȼ����ϵͳ���������Ź�����
CPU_STK task_stk_sd[128];	// 
CPU_STK task_stk_arhs[512];	// ��̬���㺯������Ҫ�ı����Ƚ϶�
CPU_STK task_stk_pid[128];	


OS_SEM		g_sem_sd;
OS_MUTEX    g_mutex_usart;
//OS_Q    	g_queue_sd;



//************************************************************************************************//
//****************************************��ʼ��ȫ�ֱ���******************************************//
//********************************* ������Ҫʹ��double ����ʹ��float *****************************//
//***********************************************************************************************//
float Rad_to_Deg = 180.0f / PI;		// �ǶȺͻ��ȵ�ת��
float Deg_to_Rad = PI / 180.0f;

KalmanAlt 		kf_alt;
float spl_presure; // �¶Ȳ��������ѹֵ ��λpar ��		//IIC���յ���ѹ������
float spl_height;  // ��������ѹ�߶�ֵ����λm ��
float battery_voltage; //ADC��⵽�ĵ�ص�ѹ ��λV ��

float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z,mag_x, mag_y, mag_z;		// ���������ٶȼƺ������ǵ�ԭʼ����
float imu_T, imu_P,imu_PT;
uint64_t imu_Timestamp; 

float ACCX_n, ACCY_n, ACCZ_n;			//ת������������ϵ�ϵļ��ٶ�
float roll_deg, pitch_deg, yaw_deg;										// ��̬�����õ��ĽǶ�ֵ
float target_roll, target_pitch, target_yaw;
float throttle;
float Moto_PWM1, Moto_PWM2, Moto_PWM3, Moto_PWM4;

char sd_buf[256];	  	// SD���洢  SD���ķ��ͻ�����
char sd_filename[20];	// д��SD���ģ��ļ���

float server_target, server_measure;



//************************************************************************************************//
//****************************************��ʼ���õ��ĺ���*****************************************//
//************************************************************************************************//
void led_init(void);		//Ӳ����ʼ������
void pwm_init(void);
void spl06_init(void);
void sd_init(void);
void kalman_alt_init(void);

void wireless_uart_Config(void);		//���ú���
void fdisystem_Config(void);





//������
int main(void)
{
	OS_ERR err;
	systick_init();  													//ʱ�ӳ�ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);						//�жϷ�������
	usart_init(115200);  				 									//���ڳ�ʼ�� 115200 9600

	//OS��ʼ�������ǵ�һ�����еĺ���,��ʼ�����ֵ�ȫ�ֱ����������ж�Ƕ�׼����������ȼ����洢��
	OSInit(&err);
	
	//������ʼ������
	//�����г�ʼ�����ݷ���task_init������OS����������Ⱥ����ִ�У��Է���ǰ���е��й�OS�ĵ��±���
	OSTaskCreate(	(OS_TCB *)&Task_TCB_init,								//������ƿ�
					(CPU_CHAR *)"Task_init",								//���������
					(OS_TASK_PTR)task_init,									//������
					(void *)0,												//���ݲ���
					(OS_PRIO)5,											 	//��������ȼ�	
					(CPU_STK *)task_stk_init,								//�����ջ����ַ
					(CPU_STK_SIZE)128/10,									//�����ջ�����λ���õ����λ�ã��������ټ���ʹ��
					(CPU_STK_SIZE)128,										//�����ջ��С			
					(OS_MSG_QTY)0,											//��ֹ������Ϣ����
					(OS_TICK)0,												//Ĭ��ʱ��Ƭ����																
					(void  *)0,												//����Ҫ�����û��洢��
					(OS_OPT)OS_OPT_TASK_NONE,								//û���κ�ѡ��
					&err													//���صĴ�����
				);
	

	//�����ź���
	OSSemCreate(&g_sem_sd, "g_sem_sd", 0, &err);    // ������������Ϊ0����Ȼһ��ʼ�õȲ����ź�����Ϊ0һ�㶼��������ͬ��ʹ��
	OSMutexCreate(&g_mutex_usart, "g_mutex_usart", &err);
//	OSQCreate(&g_queue_sd, "g_queue_sd", 16, &err); 	//������Ϣ���У�֧��16����Ϣ
	
					
	//����OS�������������
	OSStart(&err);
	while(1);
}


/**************************************************************************************/
/***********************************UCOS������***************************************/
/**************************************************************************************/
void task_init(void *parg)
{
	int wdg_i;
	OS_ERR err;
	CPU_SR_ALLOC();		// ���ʹ��CPU_CRITICAL_ENTER();�ͱ���ҪCPU_SR_ALLOC();
	
	
//	wireless_uart_Config();	// �������������Ĳ�����  //��������һ�Σ�������ǵ�ע���ˣ�������һ�����������Զˣ�Ҳ��Ҫ��ͬ������
	led_init();
	IIC_Init(); // I2C�ӿڹ��ܳ�ʼ��   ��������ȳ�ʼ�����ܳ�ʼ����Ӧ���й�Ӳ��
	spl06_init();
	sd_init();
    pwm_init(); // TIM3 TIM4 �ܹ��˸�PWM��ʼ��
	Adc_Init();
	
//	Initial_UART2(420000);	// ��ʼ��ELRS����  //���ң�������Ӳ���,���Ź���ʧЧ��������ι��
	Initial_UART3(921600);	// ��ʼ��IMU����
	Initial_UART4(38400);	// ��ʼ��GNSS����
	
	//�޷���������ģʽ��ԭ������Ǽ������߽Ӵ����� //��ҪfdiSetReboot���� 
	//��������������Ч��������Ϊ��ʱ��ʱ�䲻��
	//������ǵ���USART3_IRQHandler��ע�͵��ش���uart1�Ĵ���
//	fdisystem_Config();

	PID_Param_Init();
	kalman_alt_init();
	
	//��������1
	OSTaskCreate(	(OS_TCB *)&Task_TCB_height,								//������ƿ飬��ͬ���߳�id
					(CPU_CHAR *)"Task_height",								//��������֣����ֿ����Զ����
					(OS_TASK_PTR)task_height,								//����������ͬ���̺߳���
					(void *)0,												//���ݲ�������ͬ���̵߳Ĵ��ݲ���
					(OS_PRIO)6,											 	//��������ȼ�6		
					(CPU_STK *)task_stk_height,									//�����ջ����ַ
					(CPU_STK_SIZE)128/10,									//�����ջ�����λ���õ����λ�ã��������ټ���ʹ��
					(CPU_STK_SIZE)128,										//�����ջ��С			
					(OS_MSG_QTY)0,											//��ֹ������Ϣ����
					(OS_TICK)0,												//Ĭ��ʱ��Ƭ����																
					(void  *)0,												//����Ҫ�����û��洢��
					(OS_OPT)OS_OPT_TASK_NONE,								//û���κ�ѡ��
					&err													//���صĴ�����
				);

	//��������2
	OSTaskCreate(	(OS_TCB *)&Task_TCB_sd,									//������ƿ�
					(CPU_CHAR *)"Task_sd",									//���������
					(OS_TASK_PTR)task_sd,									//������
					(void *)0,												//���ݲ���
					(OS_PRIO)6,											 	//��������ȼ�7		
					(CPU_STK *)task_stk_sd,									//�����ջ����ַ
					(CPU_STK_SIZE)128/10,									//�����ջ�����λ���õ����λ�ã��������ټ���ʹ��
					(CPU_STK_SIZE)128,										//�����ջ��С			
					(OS_MSG_QTY)0,											//��ֹ������Ϣ����
					(OS_TICK)0,												//Ĭ��ʱ��Ƭ����																
					(void  *)0,												//����Ҫ�����û��洢��
					(OS_OPT)OS_OPT_TASK_NONE,								//û���κ�ѡ��
					&err													//���صĴ�����
				);
	
	//��������3
	OSTaskCreate(	(OS_TCB *)&Task_TCB_arhs,								//������ƿ�
					(CPU_CHAR *)"Task_arhs",								//���������
					(OS_TASK_PTR)task_arhs,									//������
					(void *)0,												//���ݲ���
					(OS_PRIO)6,											 	//��������ȼ�7		
					(CPU_STK *)task_stk_arhs,								//�����ջ����ַ
					(CPU_STK_SIZE)512/10,									//�����ջ�����λ���õ����λ�ã��������ټ���ʹ��
					(CPU_STK_SIZE)512,										//�����ջ��С			
					(OS_MSG_QTY)0,											//��ֹ������Ϣ����
					(OS_TICK)0,												//Ĭ��ʱ��Ƭ����																
					(void  *)0,												//����Ҫ�����û��洢��
					(OS_OPT)OS_OPT_TASK_NONE,								//û���κ�ѡ��
					&err													//���صĴ�����
				);
					
	//��������4
	OSTaskCreate(	(OS_TCB *)&Task_TCB_pid,								//������ƿ�
					(CPU_CHAR *)"Task_pid",									//���������
					(OS_TASK_PTR)task_pid,									//������
					(void *)0,												//���ݲ���
					(OS_PRIO)6,											 	//��������ȼ�7		
					(CPU_STK *)task_stk_pid,								//�����ջ����ַ
					(CPU_STK_SIZE)128/10,									//�����ջ�����λ���õ����λ�ã��������ټ���ʹ��
					(CPU_STK_SIZE)128,										//�����ջ��С			
					(OS_MSG_QTY)0,											//��ֹ������Ϣ����
					(OS_TICK)0,												//Ĭ��ʱ��Ƭ����																
					(void  *)0,												//����Ҫ�����û��洢��
					(OS_OPT)OS_OPT_TASK_NONE,								//û���κ�ѡ��
					&err													//���صĴ�����
				);
					
	IWDG_Init(4, 1000);// IWDG_Feed(); // ι��// ��Ƶ��Ϊ4*2^4=64,32kHz/64,����ֵΪ500,���ʱ��Ϊ1s//ʱ�����(���):Tout=((4*2^prer)*rlr)/32 (ms)
	
	while(1)
	{
		for(wdg_i = 0; wdg_i < 9; wdg_i++) {
			delay_ms(100);
			
			CPU_CRITICAL_ENTER();   // OS_CFG_ISR_POST_DEFERRED_EN Ϊ1�Ͳ����ж�
			IWDG_Feed(); 	// ι��
			CPU_CRITICAL_EXIT();     //�����˳��ٽ���
		}
		
        delay_ms(50);
        LED_PC13 = !LED_PC13;
        delay_ms(50);
        LED_PC13 = !LED_PC13;
		
		CPU_CRITICAL_ENTER();   // OS_CFG_ISR_POST_DEFERRED_EN Ϊ1�Ͳ����ж�
		IWDG_Feed(); 	// ι��
		CPU_CRITICAL_EXIT();     //�����˳��ٽ���
	}
}





//		OSTaskSuspend(&Task_TCB_1, &err);
//		OSSemPend(&g_sem, 0, OS_OPT_PEND_BLOCKING, NULL, &err);		//�����ȴ��ź���
//		OSMutexPend(&g_mutex, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
//		printf("task2 is running ...\r\n");
//		OSTaskResume(&Task_TCB_1, &err);
//		OSSemPost(&g_sem, OS_OPT_POST_1, &err);		//�ͷ��ź���
//		OSMutexPost(&g_mutex, OS_OPT_POST_NONE, &err);




void task_height(void *parg)
{
	uint16_t volt_i;
	uint16_t count_spl_i = 0;
	OS_ERR err;

	while(1)
	{
		if(count_spl_i < 500)		// ����̬�������ȶ��Ժ��ٽ��и߶ȵ��ں� 500 * 10ms = 5s
		{	
			count_spl_i += 1;
		}
		else{
			//SPL06��ѹ�ƶ�ȡ
			spl_presure = user_spl0601_get_presure(); 							// �¶Ȳ��������ѹֵ ��λpar ��
			spl_height = 44300 * (1 - pow(spl_presure / 101325, 1 / 5.256f));   	// ��������ѹ�߶�ֵ����λm ��
			
			
			//��ص�ѹ����ֵ
			battery_voltage = 0;
			for(volt_i = 0; volt_i < 6000; volt_i++) {
				battery_voltage += ADC_Value_Buffer[volt_i] & 0x0FFF;
			}
			battery_voltage = battery_voltage / 6000 * (3.3f / 4096) * 10 ; // ȡƽ����10�Ǳ���������10���������ĵ�ѹ��
			//battery_voltage = (float)Get_Adc_Average(ADC_Channel_7, 20) * (3.3 / 4096) * 10; // ��ȡͨ��5��ת��ֵ��20��ȡƽ����10�Ǳ���������10���������ĵ�ѹ��
			
			
			
			//�������˲��ںϣ����ٶȼƺ���ѹ�ƣ�
			kalman_predict(&kf_alt, ACCZ_n - 9.72f);  // ����ȥ��������ļ��ٶ�
			kalman_update(&kf_alt, spl_height);    // ��ѹ�Ƹ߶�   //printf("Step �� �߶�=%.2f m, �ٶ�=%.2f m/s\n", kf_alt.x[0], kf_alt.x[1]);



//			//**************************************��������λ����������*****************************************//
//			OSMutexPend(&g_mutex_usart, 0, OS_OPT_PEND_BLOCKING, NULL, &err);	//�����ȴ�������
//			
//			ANO_TC_Send02(mag_x, mag_y, mag_z, spl_height * 100, imu_T * 100, 1, 1); // ���̡���ѹ���¶ȴ���������
//			ANO_TC_Send05(kf_alt.x[0] * 100, 0, 1); 		// ������ĸ߶�			// ��λ����, ��Ϊ���׷���
//			ANO_TC_Send0D(battery_voltage * 100, 0);
//			
//			OSMutexPost(&g_mutex_usart, OS_OPT_POST_NONE, &err);				//�ͷŻ�����
		}
		

//		printf("gnss_nmea0183: ���ȣ�%f\t ά�ȣ�%f !\r\n", gnss_nmea0183.gpsData.location.lng, gnss_nmea0183.gpsData.location.lat);
		delay_ms(10);
	}
}




void task_sd(void *parg)
{
	OS_ERR 			err;
//	OS_MSG_SIZE		msg_size;	//��Ϣ�Ĵ�С
//	char *p = NULL;
	FRESULT res;
	static int sync_cnt = 0;
	
//	delay_ms(100);

	
	while(1)
	{
//		//�ȴ���Ϣ����
//		p = OSQPend(&g_queue_sd, 0, OS_OPT_PEND_BLOCKING, &msg_size, NULL, &err);
//		
//		//pָ����Ч����msg_size>0
//		if(p && msg_size)
//		{
////			//*****************************************SD���洢�ĳ���*******************************************//
////			f_lseek(file, file->fsize);			   // ָ��ָ���ļ�β,׷��
////			f_write(file, p, msg_size, &bw); // д��һ������			//f_write(file, "\r\n", 2, &bw); // д��һ������
//			
//			myfree(SRAMIN, p);	// �ͷŶ�̬�ڴ�	//���ͷŻ����: д��Ķ��ǿհ�����
//		}
		
		OSSemPend(&g_sem_sd, 0, OS_OPT_PEND_BLOCKING, NULL, &err);		//�����ȴ��ź���

		OSMutexPend(&g_mutex_usart, 0, OS_OPT_PEND_BLOCKING, NULL, &err);	//�����ȴ�������
		ANO_TC_Send0D(sync_cnt * 100, 0);
		OSMutexPost(&g_mutex_usart, OS_OPT_POST_NONE, &err);				//�ͷŻ�����
		
		
		//****************************SD��д�벻��ʱ�����򿪡��رպ�ͬ�������ر��ʱ******************************//
		//******************д�루��100�ֽ����ݣ�0.01-0.1ms�����򿪣�20ms�����رգ�20ms����ͬ����5ms��************//
		// ÿ100��д��ͬ��һ��, ��Ϊͬ���ǳ���ʱ�䣨5ms��
		if (++sync_cnt >= 100) {
			res = f_sync(file);      	// ȷ������������д������		// ���ùر�,�ر�̫��ʱ���ˣ�20ms��
			
			if (res != FR_OK) {
			}
			else
			{

//				//**************************************��������λ����������*****************************************//
//				OSMutexPend(&g_mutex_usart, 0, OS_OPT_PEND_BLOCKING, NULL, &err);	//�����ȴ�������
//				
//				ANO_TC_Send05(kf_alt.x[0] * 100, 0, 1); 		// ������ĸ߶�			// ��λ����, ��Ϊ���׷���
//				ANO_TC_Send0D(battery_voltage * 100, 0);
//				
//				OSMutexPost(&g_mutex_usart, OS_OPT_POST_NONE, &err);				//�ͷŻ�����
			
			}
			
			sync_cnt = 0;
		}
		
		
		
		
//		delay_ms(1);
	}

	
//	f_close(file);		// �ر��ļ�
}




void task_arhs(void *parg)
{
	OS_ERR err;
	FRESULT res;
	
	
	f_open(file, sd_filename, FA_WRITE); // �ļ�ָ��+����·��+��д��ʽ,ȫ��Ҫ��
	

	while(1)
	{
		// ���������ٶȼƺ������ǡ������Ƶ�ԭʼ����
		acc_x = -IMUData.Accelerometer_X;		// ��λ m/s^2
		acc_y = -IMUData.Accelerometer_Y;		//ȡ���ţ���������Ĳ���������
		acc_z = -IMUData.Accelerometer_Z;
		gyro_x = IMUData.Gyroscope_X;			// ��λ rad/s
		gyro_y = IMUData.Gyroscope_Y;
		gyro_z = IMUData.Gyroscope_Z;
		mag_x = IMUData.Magnetometer_X;
		mag_y = IMUData.Magnetometer_Y;
		mag_z = IMUData.Magnetometer_Z;
		imu_T = IMUData.IMU_Temperature;
		imu_P = IMUData.Pressure;
		imu_PT = IMUData.Pressure_Temperature;
		imu_Timestamp = IMUData.Timestamp;
		
		// ��̬����
//		MahonyAHRSupdate(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z);
		MahonyAHRSupdateIMU(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);		// �����ô����ƽ�����̬����
		// ��̬�����õ��ĽǶ�ֵ
		roll_deg = AHRS_Roll;			// �Ƕ�       // ����������ֵ 
		pitch_deg = AHRS_Pitc;			// ��λ ��
		yaw_deg = AHRS_Yawh;
		
		
//		// ��ȡģ��ĽǶ�ֵ
//		roll_deg = Euler_data.Roll * Rad_to_Deg;			// �Ƕ�       // ����������ֵ 
//		pitch_deg = Euler_data.Pitch * Rad_to_Deg;			// ��λ ��
//		yaw_deg = Euler_data.Heading * Rad_to_Deg;
		
		
		ACCX_n = C_nb_11 * acc_x + C_nb_12 * acc_y + C_nb_13 * acc_z; // ACCX_n���ڵ�������ϵ�ϵļ��ٶȣ�ax�ǻ�������ϵ��
		ACCY_n = C_nb_21 * acc_x + C_nb_22 * acc_y + C_nb_23 * acc_z; // ��λ��m/s^2
		ACCZ_n = C_nb_31 * acc_x + C_nb_32 * acc_y + C_nb_33 * acc_z;
		
	
		//**************************************��������λ����������*****************************************//
		OSMutexPend(&g_mutex_usart, 0, OS_OPT_PEND_BLOCKING, NULL, &err);	//�����ȴ�������
		ANO_TC_Send03(roll_deg * 100, pitch_deg * 100, yaw_deg * 100, 1);
//		ANO_TC_Send01(ACCX_n * 1000, ACCY_n * 1000, ACCZ_n * 1000, gyro_x * 1000, gyro_y * 1000, gyro_z * 1000, 1); 
//		ANO_TC_Send01(acc_x * 1000, acc_y * 1000, acc_z * 1000, gyro_x * 1000, gyro_y * 1000, gyro_z * 1000, 1);	// ���ٶ�(��λm/s^2)�����ٶ�(��λ��/s) 
//		ANO_TC_Send02((unsigned)stk_free, (unsigned)stk_used, 0, 0 * 100, 0 * 100, 1, 1); // ���̡���ѹ���¶ȴ���������
//		ANO_TC_Send04(q[0] * 10000, q[1] * 10000, q[2] * 10000, q[3] * 10000, 1);
//		ANO_TC_Send30(1, 0, 108.914937 * 10000000, 34.246872 * 10000000, 0 / 10, 0, 0, 0, 0 / 100, 0 / 100, 0 / 100);
//		ANO_TC_Send32(position_x * 100, position_y * 100, position_z * 100);   //����xyzλ����Ϣ		// ��λ����, ��Ϊ���׷���
		OSMutexPost(&g_mutex_usart, OS_OPT_POST_NONE, &err);				//�ͷŻ�����
		
		
		//*****************************************��SD��д������******************************************//
		sprintf(sd_buf, "%.4f\t%.4f\t%.4f\r\n", (float)roll_deg, (float)pitch_deg, (float)yaw_deg);	  // �Ȱѱ���ת���ַ���
		//f_lseek(file, file->fsize);			   // ָ��ָ���ļ�β,׷��
		res = f_write(file, sd_buf, strlen(sd_buf), &bw); // д��һ������
		
//		if (res != FR_OK) {
//			OSMutexPend(&g_mutex_usart, 0, OS_OPT_PEND_BLOCKING, NULL, &err);	//�����ȴ�������
//			printf("\r\n\r\nд��ʧ�ܣ������룺%d\r\n\r\n", res);
//			OSMutexPost(&g_mutex_usart, OS_OPT_POST_NONE, &err);				//�ͷŻ�����
//		}
		
//		// ��Ϣ���У�֪ͨ�Ǳߣ�֪ͨ�����ˣ�һ���������ͬ����ֻ��Ҫ�Ǳ�ͬ�����������Ϣ�����Ǳ���д�룬��������д������⣨д�벻ȫ��
//		OSQPost(&g_queue_sd, (void *)sd_buf, strlen(sd_buf), OS_OPT_POST_FIFO, &err);		
		
		OSSemPost(&g_sem_sd, OS_OPT_POST_1, &err);		//�ͷ��ź���
		
		
		
		
		
		delay_ms(10);
	}
	

//	f_close(file);	// �ر��ļ� 
}


void task_pid(void *parg)
{
//	OS_ERR err;

	while(1)
	{
		
//		// ң����ҡ�ˡ���>Ҳ���� Ŀ��Ƕ�ֵ
//		target_roll = ((double)ELRS_ch1 - 992) / 818.0 * 20;		// ң����Ŀ��Ƕ�ֵ	    // ԭͨ��ֵ174��1811
//		target_pitch = ((double)ELRS_ch2 - 992) / 818.0 * 20;		// ���߸�20�� [-20, 20]
//		target_yaw = ((double)ELRS_ch4 - 992) / 818.0 * 20;
//		throttle = ((double)ELRS_ch3 - 174) / 1637.0 * 1000 + 1000;	// �����PWMռ�ձ�Ӧ����1000-2000֮��


//		// PID�ǶȻ�    					// PID���� 
//		PID_Position_Cal(&PID_ROL_Angle, target_roll, roll_deg);			// ����Ƕ� ������ٶ�
//		PID_Position_Cal(&PID_PIT_Angle, target_pitch, pitch_deg);
//		// PID_Position_Cal(&PID_YAW_Angle, target_yaw, yaw_deg);
//		
//		// PID���ٶȻ�    
//		PID_Position_Cal(&PID_ROL_Rate, PID_ROL_Angle.OutPut, gyro_x * Rad_to_Deg);	// ����ǶȻ��������������������
//		PID_Position_Cal(&PID_PIT_Rate, PID_PIT_Angle.OutPut, gyro_y * Rad_to_Deg);
//		PID_Position_Cal(&PID_YAW_Rate, target_yaw * PID_YAW_Angle.P, gyro_z * Rad_to_Deg);
//		
//		
//		// �����������    
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

		// PWM���ƶ��
//		TIM_SetCompare1(TIM3, 500 + ((float)ELRS_ch1 - 174) / 1637 * 2000); 	// PB0
//		TIM_SetCompare2(TIM3, 500 + ((float)ELRS_ch2 - 174) / 1637 * 2000);	// ֵ����174��1811֮��(��ֵ: 1637)
//		TIM_SetCompare3(TIM3, 500 + ((float)ELRS_ch3 - 174) / 1637 * 2000);	// PWM1����PWM8
//		TIM_SetCompare4(TIM3, 500 + ((float)ELRS_ch4 - 174) / 1637 * 2000);
//		TIM_SetCompare1(TIM1, 500 + ((float)ELRS_ch5 - 174) / 1637 * 2000);
//		TIM_SetCompare2(TIM1, 500 + ((float)ELRS_ch6 - 174) / 1637 * 2000);
//		TIM_SetCompare3(TIM1, 500 + ((float)ELRS_ch7 - 174) / 1637 * 2000);
//		TIM_SetCompare4(TIM1, 500 + ((float)ELRS_ch8 - 174) / 1637 * 2000);

//		server_target = ((double)ELRS_ch3 - 174) / 1637.0 * 2000 + 500;				//Ŀ��ֵ����ҡ���ź�ֵ				
//		PID_Posi_Server_Cal(&PID_Server_Posi, server_target, server_measure);		//����ֵ���ǵ�ǰ���ֵ
//		server_measure += PID_Server_Posi.OutPut;
//		if (server_measure < 500){server_measure = 500;}		// ����޷�
//		else if (server_measure > 2500){server_measure = 2500;}
//		TIM_SetCompare4(TIM1, server_measure);
//		ANO_TC_Send05(server_target * 100,0, 1);
//		ANO_TC_Send05(server_measure * 100, 0, 1);
		
		
		
		delay_ms(10);
	}
}





/**************************************************************************************/
/***********************************Ӳ��ִ�еĺ���**************************************/
/**************************************************************************************/
void led_init(void)
{
	u16 i = 0;
    LED_Init();
//    EXTI11_Init();
    for (i = 0; i < 20; i++) // ��һ��,������Զһ��
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
	// 84Mhz,����Ƶ��Ϊ84Mhz/84,����1000000��Ϊ1s����װ��ֵ20000������PWMƵ��Ϊ 50hz.
	// ����1000000��Ϊ1s������1��Ϊ1us�������������0.5ms��2.5ms֮�䣬������Ҫ������500-2500֮��
    TIM3_PWM_Init(1000000 / 50 - 1, 84 - 1);
	
	
	// �߼���ʱ����Ƶ�������������
	// 84 * 2 Mhz,����Ƶ��Ϊ(84 * 2)Mhz/(84 * 2),����1000000��Ϊ1s����װ��ֵ20000������PWMƵ��Ϊ50hz.
	// ����1000000��Ϊ1s������1��Ϊ1us�������������1000us��2000us֮�䣬������Ҫ������1000-2000֮��
    TIM1_PWM_Init(1000000 / 50 - 1, 84 * 2 - 1);
	
	
//	TIM2_Getsample_Init(100000 / 50 - 1, 840 - 1); // (1)Ҫ����ʼ��,̫�������ֵ���쳣��  (2)����̫��100,��Ȼ������������
//  TIM4_PWM_Init(100000 / 50 - 1, 840 - 1);	// ��ʱ�����ذ���û������������
	printf("pwm_init successful!!!!!!\r\n");
}



void spl06_init(void)
{
    if (spl0601_init() != 0) // SPL06��ʼ��
    {
        printf("SPL06��ʼ��ʧ��!\r\n\r\n");
    }
	else
	{
		printf("spl06_init successful!!!!!!\r\n");
	}
}

void sd_init(void)
{
    // char buf1[512]; �������Ĵ�С��ϵ���ܲ��ܳ�ʼ���ɹ�
	//�ڴ�����ǵø�malloc.h  #define MEM1_MAX_SIZE	  80*1024  //�������ڴ� 100K  //  ԭ����100*1024
    u32 total, free;
    int res;
	DIR dir;               /* Directory object */
	FILINFO fno;           /* File information */
	char *ext;
	UINT txt_count = 0;
	int scan_counter = 0;		// ����������ѭ��
	
	
    res = SD_Init();
    LED_PD13 = 1;
    while (res) // ��ⲻ��SD��
    {
        printf("SD Card Init Error: %d !\r\n", res);
        res = SD_Init();
    }
    //show_sdcard_info();                                                // ��ӡSD�������Ϣ
    printf("SD       Size:  %d   MB\r\n", (int)(SDCardInfo.CardCapacity >> 20)); // ��ʾSD������
    my_mem_init(SRAMIN);                                               // ��ʼ���ڲ��ڴ��
    my_mem_init(SRAMCCM);                                              // ��ʼ��CCM�ڴ��
    exfuns_init();                                                     // Ϊfatfs��ر��������ڴ�
    f_mount(fs[0], "0:", 1);                                           // ����SD��
    while (exf_getfree("0", &total, &free))                            // �õ�SD������������ʣ������
    {
        printf("SD Card Fatfs Error!\r\n");
        delay_ms(200);
    }
    printf("SD Total Size:  %d   MB\r\n", total >> 10); // ��ʾSD�������� MB
    printf("SD  Free Size:  %d   MB\r\n", free >> 10);  // ��ʾSD��ʣ������ MB

	
	// ͳ��SD���ļ����µ�txt�ļ��������������µ�txt�ļ�
    /* 1. �򿪸�Ŀ¼ */
    res = f_opendir(&dir, "0:/");
    if (res != FR_OK) {
        /* ��ʧ�ܣ�����Ҫ���ش�������ӡ��־ */
		printf("SD f_opendir != FR_OK\r\n");
    }
    /* 2. ����Ŀ¼ */
    for (;;) {
        res = f_readdir(&dir, &fno);    /* ��ȡ��һ����Ŀ */
        if (res != FR_OK || fno.fname[0] == 0 || scan_counter++ >= 1000) break;  /* ��������� */

        /* 3. ֻͳ����ͨ�ļ� */
        if (!(fno.fattrib & AM_DIR)) {
            /* ������չ�� */
            ext = strrchr(fno.fname, '.');
            if (ext) {
                /* ���Դ�Сд�Ƚ� .txt */
                if (strcasecmp(ext, ".txt") == 0) {
                    txt_count++;
                }
            }
        }
    }
    /* 4. �ر�Ŀ¼ */
    f_closedir(&dir);
	printf("SD txt_count = %d\r\n", txt_count);
//	printf("SD scan_counter = %d\r\n", scan_counter);

	// ����ͳ�Ƶ�txt�ļ������������������µ�txt�ļ�
	sprintf(sd_filename, "0:/test_%d.txt", txt_count + 1);
    f_open(file, sd_filename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ); // �����ļ�//�ļ�ָ��+����·��+��д��ʽ,ȫ��Ҫ��
    f_close(file);
    LED_PD13 = 0;
	printf("SD filename = %s\r\n", sd_filename);
    delay_ms(200);
	printf("sd_init successful!!!!!!\r\n");
    delay_ms(200);
}


void kalman_alt_init(void)
{
    kalman_init(&kf_alt, 0.029, 0.9, 0.10);	// �ֱ��� dt = 0.02; var_accel=0.1, var_alt=1.0  Э����Խ�󣬾�Խ������������ݣ���֮��Խ����
	//SPL06��ѹ�ƶ�ȡ
	spl_presure = user_spl0601_get_presure(); 							// �¶Ȳ��������ѹֵ ��λpar ��
	spl_height = 44300 * (1 - pow(spl_presure / 101325, 1 / 5.256));   	// ��������ѹ�߶�ֵ����λm ��
	kf_alt.x[0] = spl_height;
}


void wireless_uart_Config(void)
{
	/***************************** �������������Ĳ����� *********************************/
	int i=0;
	char uart_set_cmd[] = {0xC0, 0x00, 0x00, 0x38, 0x00, 0x40};	 //����115200�����ʣ���0x38��Ĭ�ϵ�9600��������0x18
	usart_init(9600); // PA9��PA10��USART1��ʼ��
	
	GPIO_SetBits(GPIOA, GPIO_Pin_11);				// MDO MD1  ����ģʽ
	GPIO_SetBits(GPIOA, GPIO_Pin_12);
	delay_ms(500);
	
	for (i = 0; i < 6; i++)
		UsartSendByte(USART1, uart_set_cmd[i]); 	// �������õĲ���
	delay_ms(500);	//�ӳٺ���Ҫ������������������
	
	for (i = 0; i < 3; i++)
		UsartSendByte(USART1, 0xC3); 				// ���͸�λָ��
	delay_ms(500);
	
	usart_init(115200);
//	while(1){
//		//�������Ժ���ѭ����ֻ��Ҫ����һ�Σ�����ʹ������£������������Ҫִ�У�ע�͵���������һ�����������Զˣ�Ҳ��Ҫ��ͬ������
//		delay_ms(100);
//		LED_PC13 = !LED_PC13;
//	}

}

void fdisystem_Config(void)
{
	/***************************** ����IMU������� *********************************/
	fdiComSetConfig();		//���� *#OK
	
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
	 
	//���÷��͵����ݰ����ݼ�Ƶ��			//(1.��Ҫ���ݰ����ݣ�2.Ƶ��)
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
	
	//��ѯ����
//	fdiComGetParam("MSG_RAW_SENSORS");
//	fdiComGetParam("MSG_RAW_GNSS");
	
	//���ñ���
	fdiSetSave();
	//�˳�����
//	fdiSetDeconfig();	//���û��ֲᣬ�˳����ú�������������ͬʱ�ã�������������������
	//��������
	fdiSetReboot();		//���� *#OK
}
	
//***********************************************************




		// ���������ٶȼƺ������ǡ������Ƶ�ԭʼ����
//		acc_x = -Raw_Sensor_data.Accelerometer_X;		// ��λ m/s^2
//		acc_y = -Raw_Sensor_data.Accelerometer_Y;		//ȡ���ţ���������Ĳ���������
//		acc_z = -Raw_Sensor_data.Accelerometer_Z;
//		gyro_x = Raw_Sensor_data.Gyroscope_X;			// ��λ rad/s
//		gyro_y = Raw_Sensor_data.Gyroscope_Y;
//		gyro_z = Raw_Sensor_data.Gyroscope_Z;
//		mag_x = Raw_Sensor_data.Magnetometer_X;
//		mag_y = Raw_Sensor_data.Magnetometer_Y;
//		mag_z = Raw_Sensor_data.Magnetometer_Z;
//		imu_T = Raw_Sensor_data.IMU_Temperature;
//		imu_P = Raw_Sensor_data.Pressure;
//		imu_PT = Raw_Sensor_data.Pressure_Temperature;
		


