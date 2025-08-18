/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "system_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
 uint8_t rx_len = 0 ;             //接收一帧数据的长度
 uint8_t recv_end_flag = 0;        //Uart7一帧数据接收完成标志
// uint8_t Uart8_recv_end_flag = 0;        //Uart8一帧数据接收完成标志
// uint8_t Uart3_recv_end_flag = 0;        //Uart3一帧数据接收完成标志
// uint8_t Uart6_recv_end_flag = 0;        //Uart6一帧数据接收完成标志
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_uart8_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim6;


/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart8_tx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
//void DMA1_Stream1_IRQHandler(void)
//{
//  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

//  /* USER CODE END DMA1_Stream1_IRQn 0 */
//  HAL_DMA_IRQHandler(&hdma_usart7_tx);
//  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

//  /* USER CODE END DMA1_Stream1_IRQn 1 */
//}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart7_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}
/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart8_rx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */
//  HAL_UART_Receive_DMA(&huart8,ReceiveBuffer2,BUFFERSIZE);
  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles CAN1 TX interrupts.
  */
void CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_TX_IRQn 0 */

  /* USER CODE END CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_TX_IRQn 1 */

  /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles CAN2 TX interrupts.
  */
void CAN2_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_TX_IRQn 0 */

  /* USER CODE END CAN2_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_TX_IRQn 1 */

  /* USER CODE END CAN2_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}


/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
	uint32_t tmp_flag = 0;
	uint32_t temp;
	tmp_flag =__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE); //获取IDLE标志位
	if((tmp_flag != RESET))//idle标志被置位
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);//清除标志位
		HAL_UART_DMAStop(&huart6); //
		temp  =  __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);// 获取DMA中未传输的数据个数 
	}
  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/**
  * @brief This function handles UART7 global interrupt.
  */
void UART7_IRQHandler(void)
{
  /* USER CODE BEGIN UART7_IRQn 0 */
	uint32_t tmp_flag = 0;
	uint32_t temp;
	tmp_flag =__HAL_UART_GET_FLAG(&huart7,UART_FLAG_IDLE); //获取IDLE标志位
	if((tmp_flag != RESET))//idle标志被置位
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&huart7);//清除标志位
		HAL_UART_DMAStop(&huart7); //
		temp  =  __HAL_DMA_GET_COUNTER(&hdma_uart7_rx);// 获取DMA中未传输的数据个数 
		rx_len =  BUFFERSIZE - temp;                   //总计数减去未传输的数据个数，得到已经接收的数据个数
					
	 }
  /* USER CODE END UART7_IRQn 0 */   //HAL_UART_Transmit_DMA
  HAL_UART_IRQHandler(&huart7);
  /* USER CODE BEGIN UART7_IRQn 1 */

  /* USER CODE END UART7_IRQn 1 */
}

/**
  * @brief This function handles UART8 global interrupt.
  */
void UART8_IRQHandler(void)
{
  /* USER CODE BEGIN UART8_IRQn 0 */
  uint32_t tmp_flag = 0;
	uint32_t temp;
	tmp_flag =__HAL_UART_GET_FLAG(&huart8,UART_FLAG_IDLE); //获取IDLE标志位
	if((tmp_flag != RESET))//idle标志被置位
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&huart8);//清除标志位
		//temp = huart1.Instance->SR;      //清除状态寄存器SR,读取SR寄存器可以实现清除SR寄存器的功能
		//temp = huart1.Instance->DR;      //读取数据寄存器中的数据
		//这两句和上面那句等效
		HAL_UART_DMAStop(&huart8); //
		temp  =  __HAL_DMA_GET_COUNTER(&hdma_uart8_rx);// 获取DMA中未传输的数据个数   
		//temp  = hdma_usart1_rx.Instance->NDTR;       //读取NDTR寄存器 获取DMA中未传输的数据个数，
		//这句和上面那句等效
		//rx_len =  BUFFERSIZE - temp;                   //总计数减去未传输的数据个数，得到已经接收的数据个数
		recv_end_flag = 1;	                           // 接受完成标志位置1	
		
	 }
  /* USER CODE END UART8_IRQn 0 */
  HAL_UART_IRQHandler(&huart8);
  /* USER CODE BEGIN UART8_IRQn 1 */

  /* USER CODE END UART8_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

//占空比 0 --- 100
//u16 CCR1_dc = 20;
//u16 CCR2_dc = 40;
//u16 CCR3_dc = 60;
//u16 CCR4_dc = 80;

//u32 capture = 0;
//u8 flag1 = 0, flag2 = 0, flag3 = 0, flag4 = 0;
//u16 setcap = 0;
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
	
  HAL_TIM_IRQHandler(&htim4);
	
  /* USER CODE BEGIN TIM4_IRQn 1 */
	/*
    if( __HAL_TIM_GET_FLAG( &htim4, TIM_IT_CC1 ) != RESET )
    {
        __HAL_TIM_CLEAR_IT( &htim4, TIM_IT_CC1 );
        capture = __HAL_TIM_GetCompare(&htim4, TIM_CHANNEL_1);
			
			
        //设置占空比
        if( flag1 == 0 )
        {
            flag1 = 1;
            setcap = capture + ( u32 )CCR1_Val * CCR1_dc / 100;
        }
        else
        {
            flag1 = 0;
            setcap = capture + ( u32 )CCR1_Val  * ( 100 - CCR1_dc ) / 100;
        }
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, setcap);      
    }

			if( __HAL_TIM_GET_FLAG( &htim4, TIM_IT_CC2 ) != RESET )
	{
			__HAL_TIM_CLEAR_IT( &htim4, TIM_IT_CC2 );
			capture = __HAL_TIM_GetCompare(&htim4, TIM_CHANNEL_2);
		
		
			//设置占空比
			if( flag2 == 0 )
			{
					flag2 = 1;
					setcap = capture + ( u32 )CCR2_Val * CCR2_dc / 100;
			}
			else
			{
					flag2 = 0;
					setcap = capture + ( u32 )CCR2_Val  * ( 100 - CCR2_dc ) / 100;
			}
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, setcap);      
	}



			if( __HAL_TIM_GET_FLAG( &htim4, TIM_IT_CC3 ) != RESET )
	{
			__HAL_TIM_CLEAR_IT( &htim4, TIM_IT_CC3 );
			capture = __HAL_TIM_GetCompare(&htim4, TIM_CHANNEL_3);
		
		
			//设置占空比
			if( flag3 == 0 )
			{
					flag3 = 1;
					setcap = capture + ( u32 )CCR3_Val * CCR3_dc / 100;
			}
			else
			{
					flag3 = 0;
					setcap = capture + ( u32 )CCR3_Val  * ( 100 - CCR3_dc ) / 100;
			}
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, setcap);      
	}


		if( __HAL_TIM_GET_FLAG( &htim4, TIM_IT_CC4 ) != RESET )
	{
			__HAL_TIM_CLEAR_IT( &htim4, TIM_IT_CC4 );
			capture = __HAL_TIM_GetCompare(&htim4, TIM_CHANNEL_4);
		
		
			//设置占空比
			if( flag4 == 0 )
			{
					flag4 = 1;
					setcap = capture + ( u32 )CCR4_Val * CCR4_dc / 100;
			}
			else
			{
					flag4 = 0;
					setcap = capture + ( u32 )CCR4_Val  * ( 100 - CCR4_dc ) / 100;
			}
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, setcap);      
	}
	
	*/


}
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_IRQn 0 */

  /* USER CODE END TIM8_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_IRQn 1 */

  /* USER CODE END TIM8_IRQn 1 */
}
/* USER CODE END 1 */
