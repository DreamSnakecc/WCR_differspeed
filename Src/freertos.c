/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "system_config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
bool start_motor = true;
bool disable_allmotor = true;
bool corrEnable = true;
float corr_finished = true;

MotorTypeDef* motors_2006[3] = {&Motor_3,&Motor_4,&Motor_6};
MotorTypeDef* motors_3508[3] = {&Motor_1,&Motor_2,&Motor_5};

float motor_speed = 0;

float motor_angle = 0;

float velocity[3]={0};//rpm

float angle[3] = {0};

//uint8_t TransBuffer [BUFFERSIZE] = {0x0};

uint16_t Thruster_Dcval[6]={1500,1500,1500,1500,1500,1500};   //推进器PWM占空比
uint16_t Motor_Dcval[3]={1500,1500,1500};   //水下电机PWM占空比
uint16_t Fan_Dcval[3]={0,0,0};   //离心风机PWM占空比
uint16_t Pca_Fan_Dcval[3]={0,0,0};
uint16_t Fan_Test[4]={0,0,0,0};
bool Flag_UnderWaterMotor = true;
float Pca_9685_Cycle = 4.096;
int16_t Instruct = 0x01<<8|0x8c;   //释放电机刹车
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;
osThreadId myTask06Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartTask(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Monitor_Task(void const * argument);
void Init_Task(void const * argument);
void Regulate_Task(void const * argument);
void Handle_Task(void const * argument);
void Usart_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, Monitor_Task, osPriorityIdle, 1, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, Init_Task, osPriorityIdle, 2, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, Regulate_Task, osPriorityIdle, 3, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of myTask05 */
  osThreadDef(myTask05, Handle_Task, osPriorityIdle, 4, 128);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* definition and creation of myTask06 */
  osThreadDef(myTask06, Usart_Task, osPriorityIdle, 5, 128);
  myTask06Handle = osThreadCreate(osThread(myTask06), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Monitor_Task */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Monitor_Task */
void Monitor_Task(void const * argument)
{
  /* USER CODE BEGIN Monitor_Task */
  /* Infinite loop */
  for(;;)
  {
		//Robot_State_Controll();
    osDelay(1);
  }
  /* USER CODE END Monitor_Task */
}

/* USER CODE BEGIN Header_Init_Task */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Init_Task */
void Init_Task(void const * argument)
{
  /* USER CODE BEGIN Init_Task */
  /* Infinite loop */
  for(;;)
  {
			
		if(disable_allmotor)//停止所有电机（保护措施）
			{
				for(int i =0;i<3;i++)
				{
					motors_2006[i]->State = IDLE;
					motors_3508[i]->State = IDLE;
				}
				Instruct = 0x01<<8|0x8c;  //取消抱闸
				CAN1_cmd_chassis(Instruct,Motor_1.SpeedCloseLoop,Motor_2.SpeedCloseLoop,Motor_5.PWM,Motor_6.PWM);
				Instruct = 0xA2;    //差速轮速度闭环模式
				disable_allmotor = false;
			}
			
		else if(corr_finished)
			{
				for(int i =0;i<3;i++)
					{
						if(m2006_lock>0)
						{
							motors_2006[i]->State = PIDPOSITION;//PIDPOSITION;

							
						}
						else{
							motors_2006[i]->State = IDLE;
						  Motor_3.PositionMeasure = 0;
						  Motor_4.PositionMeasure = 0;
						  Motor_6.PositionMeasure = 0;

													
						}
					
						motors_3508[i]->State = PIDSPEED;//PIDSPEED;	
						

					}
       }

			/*
		if(HAL_GPIO_ReadPin(WT_SENSE1_GPIO_PORT, WT_SENSE1_GPIO_PIN) == 1)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1,GPIO_PIN_RESET);
		}
		if(HAL_GPIO_ReadPin(WT_SENSE2_GPIO_PORT, WT_SENSE2_GPIO_PIN) == 1)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2,GPIO_PIN_RESET);
		}
		if(HAL_GPIO_ReadPin(WT_SENSE3_GPIO_PORT, WT_SENSE3_GPIO_PIN) == 1)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3,GPIO_PIN_RESET);
		}
		if(HAL_GPIO_ReadPin(WT_SENSE4_GPIO_PORT, WT_SENSE4_GPIO_PIN) == 1)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4,GPIO_PIN_RESET);
		}	
		if(HAL_GPIO_ReadPin(WT_SENSE5_GPIO_PORT, WT_SENSE5_GPIO_PIN) == 1)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5,GPIO_PIN_RESET);
		}		
		if(HAL_GPIO_ReadPin(WT_SENSE6_GPIO_PORT, WT_SENSE6_GPIO_PIN) == 1)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6,GPIO_PIN_RESET);
		}	
		

		if(HAL_GPIO_ReadPin(WT_SENSE1_GPIO_PORT, WT_SENSE1_GPIO_PIN) == 0)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1,GPIO_PIN_SET);
		}
		if(HAL_GPIO_ReadPin(WT_SENSE2_GPIO_PORT, WT_SENSE2_GPIO_PIN) == 0)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2,GPIO_PIN_SET);
		}
		if(HAL_GPIO_ReadPin(WT_SENSE3_GPIO_PORT, WT_SENSE3_GPIO_PIN) == 0)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3,GPIO_PIN_SET);
		}
		if(HAL_GPIO_ReadPin(WT_SENSE4_GPIO_PORT, WT_SENSE4_GPIO_PIN) == 0)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4,GPIO_PIN_SET);
		}	
		if(HAL_GPIO_ReadPin(WT_SENSE5_GPIO_PORT, WT_SENSE5_GPIO_PIN) == 0)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5,GPIO_PIN_SET);
		}		
		if(HAL_GPIO_ReadPin(WT_SENSE6_GPIO_PORT, WT_SENSE6_GPIO_PIN) == 0)
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6,GPIO_PIN_SET);
		}
*/		
			

			 osDelay(1);
   }
  /* USER CODE END Init_Task */
}

/* USER CODE BEGIN Header_Regulate_Task */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Regulate_Task */
void Regulate_Task(void const * argument)
{
  /* USER CODE BEGIN Regulate_Task */
  /* Infinite loop */
  for(;;)
  {
		switch(Motor_5.State)
		{
			case PIDPOSITION:				
				Motor_5.SpeedExpected =  ClassicPidRegulate(Motor_5.PositionExpected,Motor_5.PositionMeasure,&MotorPositionPid5);
			case PIDSPEED:						
				Motor_5.CurrentExpected = ClassicPidRegulate(Motor_5.SpeedExpected,Motor_5.SpeedMeasure,&MotorSpeedPid5);		
      case MOTOR_CURRENT: 					 
				Motor_5.PWM = Motor_5.CurrentExpected;//ClassicPidRegulate(Motor_5.CurrentExpected,Motor_5.CurrentMeasure,&MotorCurrentPid1);
				break; 
			case MOTOR_PWM:			
				break;
			default:
				Motor_5.PWM = 0;
				break;	
		}
		switch(Motor_6.State)
		{
			case PIDPOSITION:				
				Motor_6.SpeedExpected =  MotorClassicPidRegulate(Motor_6.PositionExpected,Motor_6.PositionMeasure,&MotorPositionPid6);
			case PIDSPEED:						
				Motor_6.CurrentExpected = ClassicPidRegulate(Motor_6.SpeedExpected,Motor_6.SpeedMeasure,&MotorSpeedPid6);		
      case MOTOR_CURRENT: 					 
				Motor_6.PWM = Motor_6.CurrentExpected;//ClassicPidRegulate(Motor_6.CurrentExpected,Motor_6.CurrentMeasure,&MotorCurrentPid1);
				break; 
			case MOTOR_PWM:			
				break;
			default:
				Motor_6.PWM = 0;
				break;	
		}
		CAN1_cmd_chassis(Instruct,Motor_1.SpeedCloseLoop,Motor_2.SpeedCloseLoop,Motor_5.PWM,Motor_6.PWM);
  	//CAN2_cmd_chassis(Motor_5.PWM,Motor_6.PWM,0,0);
//		
    osDelay(1);
  }
  /* USER CODE END Regulate_Task */
}

/* USER CODE BEGIN Header_Handle_Task */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Handle_Task */
void Handle_Task(void const * argument)
{
  /* USER CODE BEGIN Handle_Task */
		PCA9685_Init(1000,0);
	  //IIC_Init();	
	  //PCA9685_setFreq(1000);
  /* Infinite loop */
  for(;;)
  {
/*推进器初始化*/
		
		 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//PA0
		 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);//PA1
//		 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);//PA2,与串口2的Tx冲突
//		 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);//PA3，与串口2的Rx冲突
		 	
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, Thruster_Dcval[0]);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, Thruster_Dcval[1]);

			
	   HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);//PI5
		 HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);//PI6
		 HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);//PI7
		 HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);//PI7
		
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, Thruster_Dcval[2]);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, Thruster_Dcval[3]);		
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, Thruster_Dcval[4]);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, Thruster_Dcval[5]);
		//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, Thruster_Dcval[5]);
/*    ————       */
/*离心风机初始化*/
		HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//PD12
		HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);//PD13
		HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//PD14
		HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);//PD15
	
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, Fan_Dcval[0]);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, Fan_Dcval[1]);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, Fan_Dcval[2]);
		//__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, Fan_Dcval[1]);
/*    ————       */
/*水下电机初始化*/
		HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);//PH10
		HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);//PH11
		HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);//

		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, Motor_Dcval[0]);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, Motor_Dcval[1]);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, Motor_Dcval[2]);
    //__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, MotorVal);


		
/*    ————       */
/*
		//水下
		 if(HAL_GPIO_ReadPin(WT_SENSE1_GPIO_PORT, WT_SENSE1_GPIO_PIN) == 0 && 
		   HAL_GPIO_ReadPin(WT_SENSE3_GPIO_PORT, WT_SENSE3_GPIO_PIN) == 0 && 
			 HAL_GPIO_ReadPin(WT_SENSE6_GPIO_PORT, WT_SENSE6_GPIO_PIN) == 0)
		 {
			 	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, Motor_Dcval[0]);
				__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, Motor_Dcval[1]);
				__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, Motor_Dcval[2]);
		 }
		 
		 //水上
		 else if(HAL_GPIO_ReadPin(WT_SENSE1_GPIO_PORT, WT_SENSE1_GPIO_PIN) == 1 && 
		   HAL_GPIO_ReadPin(WT_SENSE3_GPIO_PORT, WT_SENSE3_GPIO_PIN) == 1 && 
			 HAL_GPIO_ReadPin(WT_SENSE6_GPIO_PORT, WT_SENSE6_GPIO_PIN) == 1)
		 {
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, Fan_Dcval[0]);
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, Fan_Dcval[1]);
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, Fan_Dcval[2]);
		 }
		 		
		
		 else 
		 {
		   //出水
			 if(HAL_GPIO_ReadPin(WT_SENSE1_GPIO_PORT, WT_SENSE1_GPIO_PIN) == 1)
				{
					for(;;)
					{
						if(HAL_GPIO_ReadPin(WT_SENSE1_GPIO_PORT, WT_SENSE1_GPIO_PIN) == 1 &&
								 HAL_GPIO_ReadPin(WT_SENSE2_GPIO_PORT, WT_SENSE2_GPIO_PIN) == 0)
							 {
									for(;;)
									{
										if(HAL_GPIO_ReadPin(WT_SENSE1_GPIO_PORT, WT_SENSE1_GPIO_PIN) == 1 &&
											 HAL_GPIO_ReadPin(WT_SENSE2_GPIO_PORT, WT_SENSE2_GPIO_PIN) == 1)
										 {
												
											 __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 80);  //开启1号风机 80%
											 break;
										 }
									}
									__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 1500);  //停止1号电机
									break;
							 }
						}
					for(;;)
						{
							if(HAL_GPIO_ReadPin(WT_SENSE3_GPIO_PORT, WT_SENSE3_GPIO_PIN) == 1 &&
								 HAL_GPIO_ReadPin(WT_SENSE4_GPIO_PORT, WT_SENSE4_GPIO_PIN) == 0)
							 {
									for(;;)
									{
										if(HAL_GPIO_ReadPin(WT_SENSE3_GPIO_PORT, WT_SENSE3_GPIO_PIN) == 1 &&
											 HAL_GPIO_ReadPin(WT_SENSE4_GPIO_PORT, WT_SENSE4_GPIO_PIN) == 1)
										 {
												
											 __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 80);  //开启2号风机 80%
											 break;
										 }
									}
									__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 1500);  //停止2号电机
									break;
							 }
						 }
						 
					for(;;)
						{
							if(HAL_GPIO_ReadPin(WT_SENSE5_GPIO_PORT, WT_SENSE5_GPIO_PIN) == 1 &&
								 HAL_GPIO_ReadPin(WT_SENSE6_GPIO_PORT, WT_SENSE6_GPIO_PIN) == 0)
							 {
									for(;;)
									{
										if(HAL_GPIO_ReadPin(WT_SENSE5_GPIO_PORT, WT_SENSE5_GPIO_PIN) == 1 &&
											 HAL_GPIO_ReadPin(WT_SENSE6_GPIO_PORT, WT_SENSE6_GPIO_PIN) == 1)
										 {
												
											 __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 80);  //开启3号风机 80%
											 break;
										 }
									}
									__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 1500);  //停止3号电机
									break;
							 }
						 }
				}
				
       //入水
       if(HAL_GPIO_ReadPin(WT_SENSE1_GPIO_PORT, WT_SENSE1_GPIO_PIN) ==0)
				{
					for(;;)
					{
						if(HAL_GPIO_ReadPin(WT_SENSE1_GPIO_PORT, WT_SENSE1_GPIO_PIN) == 0 &&
								 HAL_GPIO_ReadPin(WT_SENSE2_GPIO_PORT, WT_SENSE2_GPIO_PIN) == 1)
							 {
							 		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);  //关闭1号风机 
									__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 1300);  //开启1号电机
									
									for(;;)
									{
										if(HAL_GPIO_ReadPin(WT_SENSE1_GPIO_PORT, WT_SENSE1_GPIO_PIN) == 0 &&
											 HAL_GPIO_ReadPin(WT_SENSE2_GPIO_PORT, WT_SENSE2_GPIO_PIN) == 0)
										 {
												

											 break;
										 }
									}
									
									break;
							 }
						}
					for(;;)
						{
							if(HAL_GPIO_ReadPin(WT_SENSE3_GPIO_PORT, WT_SENSE3_GPIO_PIN) == 0 &&
								 HAL_GPIO_ReadPin(WT_SENSE4_GPIO_PORT, WT_SENSE4_GPIO_PIN) == 1)
							 {
							    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);  //关闭2号风机
									__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 1300);  //开启2号电机
									
							 		for(;;)
									{
										if(HAL_GPIO_ReadPin(WT_SENSE3_GPIO_PORT, WT_SENSE3_GPIO_PIN) == 0 &&
											 HAL_GPIO_ReadPin(WT_SENSE4_GPIO_PORT, WT_SENSE4_GPIO_PIN) == 0)
										 {
												
											  
											 break;
										 }
									}
									
									break;
							 }
						 }
						 
					for(;;)
						{
							if(HAL_GPIO_ReadPin(WT_SENSE5_GPIO_PORT, WT_SENSE5_GPIO_PIN) == 0 &&
								 HAL_GPIO_ReadPin(WT_SENSE6_GPIO_PORT, WT_SENSE6_GPIO_PIN) == 1)
							 {
							    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);  //关闭3号风机
									__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 1300);  //开启3号电机
									
									for(;;)
									{
										if(HAL_GPIO_ReadPin(WT_SENSE5_GPIO_PORT, WT_SENSE5_GPIO_PIN) == 0 &&
											 HAL_GPIO_ReadPin(WT_SENSE6_GPIO_PORT, WT_SENSE6_GPIO_PIN) == 0)
										 {
												
											 
											 break;
										 }
									}
									
									break;
							 }
						 }
				}
				
	   }
		 

			 
		*/
	

		
		osDelay(1);
  }
  /* USER CODE END Handle_Task */
}

/* USER CODE BEGIN Header_Usart_Task */
/**
* @brief Function implementing the myTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Usart_Task */
void Usart_Task(void const * argument)
{
  /* USER CODE BEGIN Usart_Task */
	//uint8_t TransBuffer[14] = {0x04,0xff,0x06,0x40,0x06,0x40,0x04,0x4c,0x00,0x64,0x00,0x64,0x00,0x64}; 
	//uint8_t TransBuffer [BUFFERSIZE] = {0x0};
  /* Infinite loop */
  for(;;)
  {
		
		//Switch_Sense();

		 if(recv_end_flag == 1)  //从jetsonnano接收完成标志
		{
			Jetsonnaon_Decode();      			
			HAL_UART_Transmit(&huart6,Usart6_Tbuffer,14,10); 						
			rx_len = 0;//清除计数
			recv_end_flag = 0;//清除接收结束标志位
			//memset(ReceiveBuffer2,0,rx_len);
    }	
		//Jetsonnaon_Decode();
		//HAL_UART_Transmit(&huart6,Usart6_Tbuffer,14,10); 
		Adc_Decode();
		DataSend_Jetsonnano();
		
		HAL_UART_Receive_DMA(&huart6,Usart6_Rbuffer,BUFFERSIZE);
		HAL_UART_Receive_DMA(&huart7,Uart7_Rbuffer,BUFFERSIZE);
		HAL_UART_Receive_DMA(&huart8,ReceiveBuffer2,BUFFERSIZE);
    
    osDelay(100);
  }
  /* USER CODE END Usart_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

  /* USER CODE BEGIN StartTask03 */



  /* USER CODE END StartTask03 */

/* USER CODE END Application */
