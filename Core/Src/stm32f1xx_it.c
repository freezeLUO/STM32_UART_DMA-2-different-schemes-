/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

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
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
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
  * @brief This function handles Prefetch fault, memory access fault.
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
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
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

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  //HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) == SET) // 触发空闲中断
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    uint16_t temp = 0;
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);                 // 清除串口1空闲中断标志�???????????
    HAL_UART_DMAStop(&huart3);                         // 关闭DMA
    temp = huart3.Instance->SR;                           // 读取UART的中断状态寄存器（ISR），这�?�常用于清除中断标志，因为读取该寄存器可以自动清除中断标志，当串口接收到数据时，bit5 RXNE就会自动变成1，当接收完一帧数据后，bit4就会变成1.
    temp = huart3.Instance->DR;                           // 读取接收数据寄存器（RDR），这同样有助于清除接收完成的中断标�??
    temp = hdma_usart3_rx.Instance->CNDTR;                // 获取DMA接收通道的当前剩余数据个数，即尚未传输到内存的数据�??
    Uart3_Str.Uart_RecvLens = UART_BUFFSIZE - temp; // 总接收数据长�?? - 未传输的数据个数 = 已经接收到的数据长度
    //HAL_UART_RxCpltCallback(&huart3);                     // 串口接收回调函数
    RS4852_send_mode();
    HAL_UART_Transmit(&huart3, Uart3_Str.Uart_RecvBuff, Uart3_Str.Uart_RecvLens,100); // 将接收到的数据打印出�??
    RS4852_receive_mode();
    Uart3_Str.Uart_RecvLens = 0; // 清空接收长度
  }
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  HAL_UART_Receive_DMA(&huart3, Uart3_Str.Uart_RecvBuff, UART_BUFFSIZE);
  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *UartHandle, uint16_t Size) 
{

    
    if (UartHandle == &huart2)
    {
      RS4851_send_mode();

      //HAL_UART_Transmit_DMA(&huart2, (uint8_t *)Uart2_Str.Uart_RecvBuff, Size);//回显
       if(HAL_UART_Transmit(&huart2, (uint8_t *)Uart2_Str.Uart_RecvBuff,Size,100)!= HAL_OK) 
        {
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        }
      //HAL_UART_Transmit(&huart2, (uint8_t *)Uart2_Str.Uart_RecvBuff, Size, 100);//回显
      RS4851_receive_mode();
      //memset(Uart2_Str.Uart_RecvBuff, 0, Size);
      
      //�????????启DMA传输UART空闲中断中接收的数据，并在接收到UART空闲中断后停止传�????????
      HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Uart2_Str.Uart_RecvBuff, sizeof(Uart2_Str) - 1);
      __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
}



/* USER CODE END 1 */
