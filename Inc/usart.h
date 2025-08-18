/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


extern UART_HandleTypeDef huart7;

extern UART_HandleTypeDef huart8;

extern UART_HandleTypeDef huart6;


/* USER CODE BEGIN Private defines */
#define BUFFERSIZE                  100

void Adc_Decode(void);
void Jetsonnaon_Decode(void);
void Switch_Sense(void);
void DataSend_Jetsonnano(void);
/* USER CODE END Private defines */

void MX_UART7_Init(void);
void MX_UART8_Init(void);
void MX_USART6_UART_Init(void);



/* USER CODE BEGIN Prototypes */
extern uint8_t Usart6_Rbuffer[BUFFERSIZE];
extern uint8_t Usart6_Tbuffer[BUFFERSIZE];
extern uint8_t Uart7_Rbuffer[10];
extern uint8_t Uart7_Tbuffer[BUFFERSIZE];
extern uint8_t Uart8_Tbuffer[BUFFERSIZE];
extern uint8_t ReceiveBuffer2[BUFFERSIZE];    //Usart8
extern uint8_t TransBuffer2Jetson[BUFFERSIZE];    //Usart8
extern uint8_t TransBuffer[14];              //Usart8����

extern  uint8_t rx_len ;             //����һ֡���ݵĳ���
extern  uint8_t recv_end_flag;        //һ֡���ݽ�����ɱ�־
extern  uint8_t send_f103_flag;      //���͵�f103��־
extern  uint16_t m2006_lock ;             //2006������־
extern  uint16_t Adsorp_Force_receive[3];     //������
extern  float Adsorp_Force_send[3]; 
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

