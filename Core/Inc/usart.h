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

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
#define UART_BUFFSIZE  100  

typedef struct{
    uint16_t Uart_SendLens;  //待发送数据长�????
    uint16_t Uart_RecvLens;  //接收到的数据长度
    uint8_t Uart_SentBuff[UART_BUFFSIZE];	// 发�?�数�????
    uint8_t Uart_RecvBuff[UART_BUFFSIZE];	// 接收数组
    void (*Send_function)(uint8_t data);	// 发�?�函数指�????
    void (*Analysis_function)(uint8_t data);	// 解析函数指针
}UART_STR;

extern UART_STR Uart2_Str;
extern UART_STR Uart3_Str;


/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void RS4851_send_mode(void);//RS4851发�?�模�???
void RS4851_receive_mode(void);//RS4851接收模式

void RS4852_send_mode(void);//RS4851发�?�模�???
void RS4852_receive_mode(void);//RS4851接收模式
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

