/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "Motor_Control.h"
#include "tim.h"

 uint8_t Usart6_Rbuffer[BUFFERSIZE] = {0};  		//Usart6,与中间板通信
 uint8_t Usart6_Tbuffer[BUFFERSIZE] = {0};
 uint8_t Uart7_Rbuffer[10] = {0};  		
 uint8_t Uart7_Tbuffer[BUFFERSIZE] = {0};
 uint8_t ReceiveBuffer2[BUFFERSIZE] = {0};    //Usart8,Jetsonnano与stm32通信 
 uint8_t TransBuffer2Jetson[BUFFERSIZE] = {0};
 uint8_t send_f103_flag = 0;
 uint16_t m2006_lock = 1;
 uint32_t Ccr1[6];
//uint32_t MotorVal[3];

/* USER CODE END 0 */


UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart6;


DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_uart8_rx;
DMA_HandleTypeDef hdma_uart8_tx;
DMA_HandleTypeDef hdma_usart6_tx;
DMA_HandleTypeDef hdma_usart6_rx;


 
/* UART7 init function */
void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */
  __HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE); //使能IDLE中断
	HAL_UART_Receive_DMA(&huart7,Uart7_Rbuffer,BUFFERSIZE);
  /* USER CODE END UART7_Init 2 */

}
/* UART8 init function */
void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */
  __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE); //使能IDLE中断
	HAL_UART_Receive_DMA(&huart8,ReceiveBuffer2,BUFFERSIZE);
  /* USER CODE END UART8_Init 2 */

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE); //使能IDLE中断
	HAL_UART_Receive_DMA(&huart6, Usart6_Rbuffer,BUFFERSIZE);
  /* USER CODE END USART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
	
   if(uartHandle->Instance==UART7)
  {
  /* USER CODE BEGIN UART7_MspInit 0 */

  /* USER CODE END UART7_MspInit 0 */
    /* UART7 clock enable */
    __HAL_RCC_UART7_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**UART7 GPIO Configuration
    PE8     ------> UART7_TX
    PE7     ------> UART7_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART7;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
		
		/* UART7 DMA Init */
    /* UART7_RX Init */
    hdma_uart7_rx.Instance = DMA1_Stream3;
    hdma_uart7_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_uart7_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart7_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart7_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart7_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart7_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart7_rx.Init.Mode = DMA_NORMAL;
    hdma_uart7_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart7_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart7_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart7_rx);



    /* UART7 interrupt Init */
    HAL_NVIC_SetPriority(UART7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART7_IRQn);
  /* USER CODE BEGIN UART7_MspInit 1 */

  /* USER CODE END UART7_MspInit 1 */
  }
  else if(uartHandle->Instance==UART8)
  {
  /* USER CODE BEGIN UART8_MspInit 0 */

  /* USER CODE END UART8_MspInit 0 */
    /* UART8 clock enable */
    __HAL_RCC_UART8_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**UART8 GPIO Configuration
    PE1     ------> UART8_TX
    PE0     ------> UART8_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART8;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* UART8 DMA Init */
    /* UART8_RX Init */
    hdma_uart8_rx.Instance = DMA1_Stream6;
    hdma_uart8_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_uart8_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart8_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart8_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart8_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart8_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart8_rx.Init.Mode = DMA_NORMAL;
    hdma_uart8_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_uart8_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart8_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart8_rx);

    /* UART8_TX Init */
    hdma_uart8_tx.Instance = DMA1_Stream0;
    hdma_uart8_tx.Init.Channel = DMA_CHANNEL_5;
    hdma_uart8_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart8_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart8_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart8_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart8_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart8_tx.Init.Mode = DMA_NORMAL;
    hdma_uart8_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_uart8_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart8_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_uart8_tx);

    /* UART8 interrupt Init */
    HAL_NVIC_SetPriority(UART8_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART8_IRQn);
  /* USER CODE BEGIN UART8_MspInit 1 */

  /* USER CODE END UART8_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* USART6 DMA Init */
    /* USART6_RX Init */
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_NORMAL;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart6_rx);
		
		/* USART6_TX Init */
    hdma_usart6_tx.Instance = DMA2_Stream6;
    hdma_usart6_tx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_tx.Init.Mode = DMA_NORMAL;
    hdma_usart6_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart6_tx);
		
		/* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);

  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART7)
  {
  /* USER CODE BEGIN UART7_MspDeInit 0 */

  /* USER CODE END UART7_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART7_CLK_DISABLE();

    /**UART7 GPIO Configuration
    PE8     ------> UART7_TX
    PE7     ------> UART7_RX
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_8|GPIO_PIN_7);

    /* UART7 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* UART7 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART7_IRQn);
  /* USER CODE BEGIN UART7_MspDeInit 1 */

  /* USER CODE END UART7_MspDeInit 1 */
  }
  else if(uartHandle->Instance==UART8)
  {
  /* USER CODE BEGIN UART8_MspDeInit 0 */

  /* USER CODE END UART8_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART8_CLK_DISABLE();

    /**UART8 GPIO Configuration
    PE1     ------> UART8_TX
    PE0     ------> UART8_RX
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_1|GPIO_PIN_0);

    /* UART8 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* UART8 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART8_IRQn);
  /* USER CODE BEGIN UART8_MspDeInit 1 */

  /* USER CODE END UART8_MspDeInit 1 */
  }
	
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14|GPIO_PIN_9);

    /* USART6 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
		HAL_DMA_DeInit(uartHandle->hdmatx);
		/* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
	}
	
}

/* USER CODE BEGIN 1 */
int fputc(int ch, FILE *f){
 uint8_t temp[1] = {ch};
 HAL_UART_Transmit(&huart8, temp, 1, 0xffff);
return ch;
}

int fgetc(FILE * f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart8,&ch, 1, 0xffff);
  return ch;
}

////电机减速比//2006
//float ratio_2006 = 36.0f*7;//
////驱动电机减速比//3508
//float ratio_3508 = 100.0f;//

void Jetsonnaon_Decode()//获取Jetsonnaon数据
{
 	
					
  int R_C = 75;   //差速轮半径，mm
	int R_D = 40;  //舵轮半径，mm
	float w = 0.1; //机器人角速度
	
	
	if(ReceiveBuffer2[0]==0x01 && ReceiveBuffer2[1]==0x02)
		{
			
				  int j=0;
				  int begin_bum=1;
			
			    Motor_5.SpeedExpected=(float)(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
					//Motor_5.SpeedExpected=Motor_5.SpeedExpected/1000;				  
					j+=4;           //舵轮1
				 
					Motor_1.SpeedCloseLoop=(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);//差速轮1速度，mm/s
					Motor_1.SpeedCloseLoop=Motor_1.SpeedCloseLoop*360*100*36/(2*PI*R_C);	//输入到电机的闭环速度值		  
					j+=4;          //左差速轮
							
					Motor_2.SpeedCloseLoop=(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);//差速轮2速度，mm/s
					Motor_2.SpeedCloseLoop=Motor_2.SpeedCloseLoop*360*100*36/(2*PI*R_C);	//输入到电机的闭环速度值				  
					j+=4;				   //右差速轮

					
				}
	if(ReceiveBuffer2[0]==0x02 && ReceiveBuffer2[1]==0x03)
		{
					int j=0;
				  int begin_bum=1;
			    float Position3,Position4,Position6 = 0;
			
//					Motor_6.PositionExpected=(float)(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
//					//Motor_6.PositionExpected=Motor_6.PositionExpected*8192*ratio_2006/360;
//					j+=4;
					if(ReceiveBuffer2[5] == 0x0)//停止
					{
						Motor_5.SpeedExpected = 0;
						Motor_1.SpeedCloseLoop = 0;
						Motor_2.SpeedCloseLoop = 0;
						Motor_6.PositionExpected =0; 
					}
			    if(ReceiveBuffer2[5] == 0x1)//右转
					{
						Motor_6.PositionExpected = -90; // 度
						Motor_5.SpeedExpected = 293*w;    //mm/s
						
						Motor_1.SpeedCloseLoop = -245*w*360*100*36/(2*PI*R_C);//245为差速轮轮距的一半，36为电机减速比
						Motor_2.SpeedCloseLoop = 245*w*360*100*36/(2*PI*R_C);					  
					}
					if(ReceiveBuffer2[5] == 0x2)//左转
					{
						Motor_6.PositionExpected = -90; // 度
						Motor_5.SpeedExpected = -293*w;    //mm/s
						
						Motor_1.SpeedCloseLoop = 245*w*360*100*36/(2*PI*R_C);//245为差速轮轮距的一半，36为电机减速比
						Motor_2.SpeedCloseLoop = -245*w*360*100*36/(2*PI*R_C);	
					  
					}
					
					if(ReceiveBuffer2[5] == 0x3)//直行
					{
						Motor_5.SpeedExpected = 20;
						Motor_1.SpeedCloseLoop = 20*360*100*36/(2*PI*R_C);
						Motor_2.SpeedCloseLoop = 20*360*100*36/(2*PI*R_C);
						Motor_6.PositionExpected =0; 
					  
					}
					if(ReceiveBuffer2[2] == 0x4)//倒退
					{
						Motor_5.SpeedExpected = -30;
						Motor_1.SpeedCloseLoop = -30*360*100*36/(2*PI*R_C);
						Motor_2.SpeedCloseLoop = -30*360*100*36/(2*PI*R_C);
						Motor_6.PositionExpected =0; 
					  
					}
					
				}
	if(ReceiveBuffer2[0]==0x03 && ReceiveBuffer2[1]==0x04)
		{
					int j=0;
				  int begin_bum=1;
			    Usart6_Tbuffer[0] = 0x04;
			    Usart6_Tbuffer[1] = 0xff;
					//水下电机
					Motor_Dcval[0]=(uint16_t)(ReceiveBuffer2[j+1+begin_bum]<<8|ReceiveBuffer2[j+2+begin_bum]);
			    Usart6_Tbuffer[2] = ReceiveBuffer2[j+1+begin_bum];//水下电机1 PWM控制值
					Usart6_Tbuffer[3] = ReceiveBuffer2[j+2+begin_bum];
			    j+=2;
					
					Motor_Dcval[1]=(uint16_t)(ReceiveBuffer2[j+1+begin_bum]<<8|ReceiveBuffer2[j+2+begin_bum]);			
					Usart6_Tbuffer[4] = ReceiveBuffer2[j+1+begin_bum];//水下电机2 PWM控制值
					Usart6_Tbuffer[5] = ReceiveBuffer2[j+2+begin_bum];
					j+=2;
			    
					Motor_Dcval[2]=(uint16_t)(ReceiveBuffer2[j+1+begin_bum]<<8|ReceiveBuffer2[j+2+begin_bum]);			
					Usart6_Tbuffer[6] = ReceiveBuffer2[j+1+begin_bum];//水下电机3 PWM控制值
					Usart6_Tbuffer[7] = ReceiveBuffer2[j+2+begin_bum];
					j+=2;
			    
		}
	if(ReceiveBuffer2[0]==0x04 && ReceiveBuffer2[1]==0x05)
		{
					int j=0;
					
				  int begin_bum=1;//(1+4)*4
					Usart6_Tbuffer[0] = 0x04;
			    Usart6_Tbuffer[1] = 0xff;
					//离心风机
				  Fan_Dcval[0]=(uint16_t)(ReceiveBuffer2[j+1+begin_bum]<<8|ReceiveBuffer2[j+2+begin_bum]);
			    Usart6_Tbuffer[8] = ReceiveBuffer2[j+1+begin_bum];//离心风机1 PWM控制值
					Usart6_Tbuffer[9] = ReceiveBuffer2[j+2+begin_bum];
					j+=2;
					if(Fan_Dcval[0]>950)
					{
						Fan_Dcval[0] = 950;
					}
					
					Fan_Dcval[1]=(uint16_t)(ReceiveBuffer2[j+1+begin_bum]<<8|ReceiveBuffer2[j+2+begin_bum]);
					
					Usart6_Tbuffer[10] = ReceiveBuffer2[j+1+begin_bum];//离心风机2 PWM控制值
					Usart6_Tbuffer[11] = ReceiveBuffer2[j+2+begin_bum];
					j+=2;
					if(Fan_Dcval[1]>950)
					{
						Fan_Dcval[1] = 950;
					}
			    
					Fan_Dcval[2]=(uint16_t)(ReceiveBuffer2[j+1+begin_bum]<<8|ReceiveBuffer2[j+2+begin_bum]);
					
					Usart6_Tbuffer[12] = ReceiveBuffer2[j+1+begin_bum];//离心风机3 PWM控制值
					Usart6_Tbuffer[13] = ReceiveBuffer2[j+2+begin_bum];
			    j+=2;
					if(Fan_Dcval[2]>950)
					{
						Fan_Dcval[2] = 950;
					}

			
		}
			
		

	if(ReceiveBuffer2[0]==0x05 && ReceiveBuffer2[1]==0x06)
			{
	
				int j=0;
				int begin_bum=1;			
			
			//水下推进器
				Thruster_Dcval[0] = (uint16_t)(ReceiveBuffer2[j+1+begin_bum]<<8|ReceiveBuffer2[j+2+begin_bum]);
				j+=2;
				
				Thruster_Dcval[1] = (uint16_t)(ReceiveBuffer2[j+1+begin_bum]<<8|ReceiveBuffer2[j+2+begin_bum]);
				j+=2;
				
				Thruster_Dcval[2] = (uint16_t)(ReceiveBuffer2[j+1+begin_bum]<<8|ReceiveBuffer2[j+2+begin_bum]);
				j+=2;
				
				Thruster_Dcval[3] = (uint16_t)(ReceiveBuffer2[j+1+begin_bum]<<8|ReceiveBuffer2[j+2+begin_bum]);
				j+=2;
				
				Thruster_Dcval[4] = (uint16_t)(ReceiveBuffer2[j+1+begin_bum]<<8|ReceiveBuffer2[j+2+begin_bum]);
				j+=2;
				
				Thruster_Dcval[5] = (uint16_t)(ReceiveBuffer2[j+1+begin_bum]<<8|ReceiveBuffer2[j+2+begin_bum]);
				//j+=2;
			

 }
			if(ReceiveBuffer2[0]==0x06 && ReceiveBuffer2[1]==0x07)
			{
	
				int j=0;
				int begin_bum=1;
				m2006_lock = (uint16_t)(ReceiveBuffer2[j+1+begin_bum]<<8|ReceiveBuffer2[j+2+begin_bum]);
			}
			
			if(ReceiveBuffer2[0]==0xEE && ReceiveBuffer2[1]==0xFF )
			{
	
				int j=0;
				int begin_bum=1;
				float Position3,Position4,Position6 = 0;

					

				Motor_3.PositionExpected=(float)(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
				Motor_3.PositionExpected=Motor_3.PositionExpected/1000;
				j+=4;
		

				Motor_4.PositionExpected=(float)(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
				Motor_4.PositionExpected=Motor_4.PositionExpected/1000;
				j+=4;

				Motor_6.PositionExpected=(float)(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
				Motor_6.PositionExpected=Motor_6.PositionExpected/1000;
				j+=4;		
				
				//3508
				Motor_1.SpeedExpected=(float)(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
				Motor_1.SpeedExpected=Motor_1.SpeedExpected/1000*ratio_3508*4.77;				  
				j+=4;
							
				Motor_2.SpeedExpected=(float)(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
				Motor_2.SpeedExpected=Motor_2.SpeedExpected/1000*ratio_3508*4.77;				  
				j+=4;				

				Motor_5.SpeedExpected=(float)(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
				Motor_5.SpeedExpected=Motor_5.SpeedExpected/1000*ratio_3508*4.77;			  
				j+=4;
			
		}



/*
if(ReceiveBuffer2[0]==0x06 && ReceiveBuffer2[1]==0x07)
		{
			
				  int j=0;
				  int begin_bum=1;
				 
					Motor_1.SpeedExpected=(float)(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
					//Motor_1.SpeedExpected=Motor_1.SpeedExpected*ratio_3508;				  
					j+=4;
							
					Motor_2.SpeedExpected=(float)(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
					//Motor_2.SpeedExpected=Motor_2.SpeedExpected*ratio_3508;				  
					j+=4;				

					Motor_5.SpeedExpected=(float)(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
					//Motor_5.SpeedExpected=Motor_5.SpeedExpected*ratio_3508;				  
					j+=4;

			
					Motor_3.PositionExpected=(float)(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
					//Motor_3.PositionExpected=Motor_3.PositionExpected*8192*ratio_2006/360;
					j+=4;
			

					Motor_4.PositionExpected=(float)(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
					//Motor_4.PositionExpected=Motor_4.PositionExpected*8192*ratio_2006/360;
          j+=4;

					Motor_6.PositionExpected=(float)(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
					//Motor_6.PositionExpected=Motor_6.PositionExpected*8192*ratio_2006/360;
					j+=4;

					//水下电机
					Motor_Dcval[0]=(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
					j+=4;
					
					Motor_Dcval[1]=(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
					j+=4;
			    
					Motor_Dcval[2]=(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
					j+=4;

	
					
					//离心风机
				  Fan_Dcval[0]=(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
					j+=4;
					if(Fan_Dcval[0]>950)
					{
						Fan_Dcval[0] = 950;
					}
					
					Fan_Dcval[1]=(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
					j+=4;
					if(Fan_Dcval[1]>950)
					{
						Fan_Dcval[1] = 950;
					}
			    
					Fan_Dcval[2]=(int)(ReceiveBuffer2[j+1+begin_bum]<<24|ReceiveBuffer2[j+2+begin_bum]<<16|ReceiveBuffer2[j+3+begin_bum]<<8|ReceiveBuffer2[j+4+begin_bum]);
					if(Fan_Dcval[2]>950)
					{
						Fan_Dcval[2] = 950;
					}
			    
//转发f103						
//				 for(j=0;j<8;j++)
//				{
//					TransBuffer1[j]=ReceiveBuffer2[j+1+begin_bum];
//				}
//				//j+=1;
//				TransBuffer1[j] = 0x0d;
//				j+=1;
//				TransBuffer1[j] = 0x0a;
//        send_f103_flag = 1;				

			

			
		

	if(ReceiveBuffer2[0]==0x05 && ReceiveBuffer2[1]==0x06)
			{
	
				int j=0;
				int begin_bum=1;			
			
			//水下推进器
				Thruster_Dcval[0] = (uint16_t)(ReceiveBuffer2[j+1+begin_bum]<<8|ReceiveBuffer2[j+2+begin_bum]);
				j+=2;
				
				Thruster_Dcval[1] = (uint16_t)(ReceiveBuffer2[j+1+begin_bum]<<8|ReceiveBuffer2[j+2+begin_bum]);
				j+=2;
				
				Thruster_Dcval[2] = (uint16_t)(ReceiveBuffer2[j+1+begin_bum]<<8|ReceiveBuffer2[j+2+begin_bum]);
				j+=2;
				
				Thruster_Dcval[3] = (uint16_t)(ReceiveBuffer2[j+1+begin_bum]<<8|ReceiveBuffer2[j+2+begin_bum]);
				j+=2;
				
				Thruster_Dcval[4] = (uint16_t)(ReceiveBuffer2[j+1+begin_bum]<<8|ReceiveBuffer2[j+2+begin_bum]);
				j+=2;
				
				Thruster_Dcval[5] = (uint16_t)(ReceiveBuffer2[j+1+begin_bum]<<8|ReceiveBuffer2[j+2+begin_bum]);
				//j+=2;
			}
			*/
			

 
}
uint16_t Adsorp_Force_receive[3] = {0};
float Adsorp_Force_send[3] = {0};
void Adc_Decode()
{
	if(Uart7_Rbuffer[0] == 0x0A && Uart7_Rbuffer[1] == 0x0D)
	{
		
		int j = 0;
		int i = 0;		
		for(int i = 0;i<3;i++)
		{
			Adsorp_Force_receive[i] = Uart7_Rbuffer[j+2]<<8| Uart7_Rbuffer[j+3];
		  Adsorp_Force_send[i] = (float)Adsorp_Force_receive[i]/4095*666;      // 吸附力接受顺序 3 1 2 ，单位N					
			j+=2;
		}
//		Adsorp_Force_receive[i] = Uart7_Rbuffer[j+2]<<8| Uart7_Rbuffer[j+3];
//	  Adsorp_Force_send[i] = (float)Adsorp_Force_receive[i]/4095*666;
//		j+=2;
//		i+=1;
//		Adsorp_Force_receive[i] = Uart7_Rbuffer[j+2]<<8| Uart7_Rbuffer[j+3];
//	  Adsorp_Force_send[i] = (float)Adsorp_Force_receive[i]/4095*666;
//		j+=2;
//		i+=1;
//		Adsorp_Force_receive[i] = Uart7_Rbuffer[j+2]<<8| Uart7_Rbuffer[j+3];
//	  Adsorp_Force_send[i] = (float)Adsorp_Force_receive[i]/4095*666;
//		j+=2;
//		i+=1;
	}
}

void Motor_Data_Packed(uint8_t *Tansbuffer,MotorTypeDef Motor,int16_t prefix)
{
	int j=0;
  //int begin_bum=1;	
//	TransBuffer2Jetson[0] = 0x51;
//	TransBuffer2Jetson[1] = 0x51;
	Motor.PositionMeasure = Motor.PositionMeasure*1000;
  
	Tansbuffer[j+prefix+1] = (int)Motor.PositionMeasure;
	Tansbuffer[j+prefix+2] = (int)Motor.PositionMeasure>>8;
	Tansbuffer[j+prefix+3] = (int)Motor.PositionMeasure>>16;
	Tansbuffer[j+prefix+4] = (int)Motor.PositionMeasure>>24;
	j+=4;
	
	Motor.SpeedMeasure = Motor.SpeedMeasure*1000;  
	Tansbuffer[j+prefix+1] = (int)Motor.SpeedMeasure;
	Tansbuffer[j+prefix+2] = (int)Motor.SpeedMeasure>>8;
	Tansbuffer[j+prefix+3] = (int)Motor.SpeedMeasure>>16;
	Tansbuffer[j+prefix+4] = (int)Motor.SpeedMeasure>>24;
	j+=4;
	
	Motor.CurrentMeasure = Motor.CurrentMeasure*1000;
	Tansbuffer[j+prefix+1] = (int)Motor.CurrentMeasure;
	Tansbuffer[j+prefix+2] = (int)Motor.CurrentMeasure>>8;
	Tansbuffer[j+prefix+3] = (int)Motor.CurrentMeasure>>16;
	Tansbuffer[j+prefix+4] = (int)Motor.CurrentMeasure>>24;		
	
}

void DataSend_Jetsonnano()
{
	int begin_bum=1;
	TransBuffer2Jetson[0] = 0x51;
	TransBuffer2Jetson[1] = 0x51;
	Motor_Data_Packed(TransBuffer2Jetson,Motor_6,begin_bum);//1 舵轮转向电机
	begin_bum+=12;
	
	Motor_Data_Packed(TransBuffer2Jetson,Motor_5,begin_bum);//2 舵轮驱动电机
	begin_bum+=12;
	
	Motor_Data_Packed(TransBuffer2Jetson,Motor_1,begin_bum);//3 差速轮电机1
	begin_bum+=12;
	
	Motor_Data_Packed(TransBuffer2Jetson,Motor_2,begin_bum);//4 差速轮电机2
	begin_bum+=12;
	
	Adsorp_Force_send[1] = Adsorp_Force_send[1]*1000;                 //吸附腔1吸附力
	TransBuffer2Jetson[begin_bum+1]=(int)Adsorp_Force_send[1];
	TransBuffer2Jetson[begin_bum+2]=(int)Adsorp_Force_send[1]>>8;
	TransBuffer2Jetson[begin_bum+3]=(int)Adsorp_Force_send[1]>>16;
	TransBuffer2Jetson[begin_bum+4]=(int)Adsorp_Force_send[1]>>24;
	begin_bum+=4;
	
	Adsorp_Force_send[2] = Adsorp_Force_send[2]*1000;                 //吸附腔2吸附力
	TransBuffer2Jetson[begin_bum+1]=(int)Adsorp_Force_send[2];
	TransBuffer2Jetson[begin_bum+2]=(int)Adsorp_Force_send[2]>>8;
	TransBuffer2Jetson[begin_bum+3]=(int)Adsorp_Force_send[2]>>16;
	TransBuffer2Jetson[begin_bum+4]=(int)Adsorp_Force_send[2]>>24;
	begin_bum+=4;	
	
  Adsorp_Force_send[0] = Adsorp_Force_send[0]*1000;                 //吸附腔3吸附力
	TransBuffer2Jetson[begin_bum+1]=(int)Adsorp_Force_send[0];
	TransBuffer2Jetson[begin_bum+2]=(int)Adsorp_Force_send[0]>>8;
	TransBuffer2Jetson[begin_bum+3]=(int)Adsorp_Force_send[0]>>16;
	TransBuffer2Jetson[begin_bum+4]=(int)Adsorp_Force_send[0]>>24;
	begin_bum+=4;
  TransBuffer2Jetson[62] = 0x0D;
	TransBuffer2Jetson[63] = 0x0A;
	HAL_UART_Transmit_DMA(&huart8,TransBuffer2Jetson,62);
	
}
/*
void Switch_Sense()
{
	  TransBuffer[0]=0x09;
		//123碰地
		if(HAL_GPIO_ReadPin(Switch_Sense1_GPIO_PORT,Switch_Sense1_GPIO_PIN))
		{
			TransBuffer[1]=0x01;
		}
		if(HAL_GPIO_ReadPin(Switch_Sense2_GPIO_PORT,Switch_Sense2_GPIO_PIN))
		{
			TransBuffer[2]=0x02;
		}
		if(HAL_GPIO_ReadPin(Switch_Sense3_GPIO_PORT,Switch_Sense3_GPIO_PIN))
		{
			TransBuffer[3]=0x03;
		}
		//123未碰地
		if(!HAL_GPIO_ReadPin(Switch_Sense1_GPIO_PORT,Switch_Sense1_GPIO_PIN))
		{
			TransBuffer[1]=0x00;
		}
		if(!HAL_GPIO_ReadPin(Switch_Sense2_GPIO_PORT,Switch_Sense2_GPIO_PIN))
		{
			TransBuffer[2]=0x00;
		}
		if(!HAL_GPIO_ReadPin(Switch_Sense3_GPIO_PORT,Switch_Sense3_GPIO_PIN))
		{
			TransBuffer[3]=0x00;
		}
		
		//456碰地	
		if(HAL_GPIO_ReadPin(Switch_Sense4_GPIO_PORT,Switch_Sense4_GPIO_PIN))
		{
			TransBuffer[4]=0x04;
		}
		if(HAL_GPIO_ReadPin(Switch_Sense5_GPIO_PORT,Switch_Sense5_GPIO_PIN))
		{
			TransBuffer[5]=0x05;
		}
		if(HAL_GPIO_ReadPin(Switch_Sense6_GPIO_PORT,Switch_Sense6_GPIO_PIN))
		{
			TransBuffer[6]=0x06;
		}
		//456未碰地	
		if(!HAL_GPIO_ReadPin(Switch_Sense4_GPIO_PORT,Switch_Sense4_GPIO_PIN))
		{
			TransBuffer[4]=0x00;
		}
		if(!HAL_GPIO_ReadPin(Switch_Sense5_GPIO_PORT,Switch_Sense5_GPIO_PIN))
		{
			TransBuffer[5]=0x00;
		}
		if(!HAL_GPIO_ReadPin(Switch_Sense6_GPIO_PORT,Switch_Sense6_GPIO_PIN))
		{
			TransBuffer[6]=0x00;
		}
		
		//789碰地			
		if(HAL_GPIO_ReadPin(Switch_Sense7_GPIO_PORT,Switch_Sense7_GPIO_PIN))
		{
			TransBuffer[7]=0x07;
		}
		if(HAL_GPIO_ReadPin(Switch_Sense8_GPIO_PORT,Switch_Sense8_GPIO_PIN))
		{
			TransBuffer[8]=0x08;
		}
		if(HAL_GPIO_ReadPin(Switch_Sense9_GPIO_PORT,Switch_Sense9_GPIO_PIN))
		{
			TransBuffer[9]=0x09;
		}
		//789未碰地
		if(!HAL_GPIO_ReadPin(Switch_Sense7_GPIO_PORT,Switch_Sense7_GPIO_PIN))
		{
			TransBuffer[7]=0x00;
		}
		if(!HAL_GPIO_ReadPin(Switch_Sense8_GPIO_PORT,Switch_Sense8_GPIO_PIN))
		{
			TransBuffer[8]=0x00;
		}
		if(!HAL_GPIO_ReadPin(Switch_Sense9_GPIO_PORT,Switch_Sense9_GPIO_PIN))
		{
			TransBuffer[9]=0x00;
		}

}
*/
				 

/* USER CODE END 1 */
