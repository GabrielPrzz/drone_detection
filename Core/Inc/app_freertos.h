/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.h
  * Description        : FreeRTOS applicative header file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#ifndef __APP_FREERTOS_H
#define __APP_FREERTOS_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Exported macro -------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */
extern osThreadId_t defaultTaskHandle;
extern osThreadId_t ESP32_TaskHandle;
extern osThreadId_t HC12_TaskHandle;
extern osThreadId_t Battery_TaskHandle;
extern osThreadId_t GPS_TaskHandle;
extern osThreadId_t DetectionTaskHandle;
extern osThreadId_t HC12_Rx_TaskHandle;
extern osThreadId_t SleepTaskHandle;
extern osMutexId_t rssiMutexHandle;
extern osMutexId_t honeyCombMutexHandle;
extern osMutexId_t printUartMutexHandle;
extern osMutexId_t retxMutexHandle;
extern osMessageQueueId_t hc12QueueHandle;
extern osMessageQueueId_t scoreQueueHandle;
extern osSemaphoreId_t tim3SemHandle;
extern osSemaphoreId_t uart4RxSemHandle;
extern osSemaphoreId_t hc12RxSemHandle;
extern osSemaphoreId_t gpsSemHandle;
extern osSemaphoreId_t chargerSemHandle;
extern osSemaphoreId_t i2c2RxSemHandle;
extern osSemaphoreId_t lpuart1RxSemHandle;
extern osSemaphoreId_t newRssiSemHandle;
extern osSemaphoreId_t sleepAckSemHandle;
extern osSemaphoreId_t sleepSemHandle;
extern osSemaphoreId_t wkpCmdSemHandle;
extern osSemaphoreId_t esp32AckSemHandle;

/* Exported function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void ESP32_Task(void *argument);
void HC12_Task(void *argument);
void Battery_Task(void *argument);
void GPS_Task(void *argument);
void DetectionTask(void *argument);
void HC12_Rx_Task(void *argument);
void SleepTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

#ifdef __cplusplus
}
#endif
#endif /* __APP_FREERTOS_H */
