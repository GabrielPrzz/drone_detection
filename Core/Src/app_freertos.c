/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
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

/* Includes ------------------------------------------------------------------*/
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "arm_math.h"
#include "Esp32.h"
#include "LoRa.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern HoneyComb_m honey_comb;
extern uint8_t rssi_index;
extern int8_t uart_rx_buffer[15];
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim3;
extern LoRa myLoRa;

void print_debug(const char *msg);
void new_rssi_load(int8_t* data, scan_t* history, uint8_t* index);
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ESP32_Comm */
osThreadId_t ESP32_CommHandle;
const osThreadAttr_t ESP32_Comm_attributes = {
  .name = "ESP32_Comm",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 512 * 4
};
/* Definitions for LoRa_task */
osThreadId_t LoRa_taskHandle;
const osThreadAttr_t LoRa_task_attributes = {
  .name = "LoRa_task",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 512 * 4
};
/* Definitions for Timing_Task */
osThreadId_t Timing_TaskHandle;
const osThreadAttr_t Timing_Task_attributes = {
  .name = "Timing_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for rssiMutex */
osMutexId_t rssiMutexHandle;
const osMutexAttr_t rssiMutex_attributes = {
  .name = "rssiMutex"
};
/* Definitions for loraMutex */
osMutexId_t loraMutexHandle;
const osMutexAttr_t loraMutex_attributes = {
  .name = "loraMutex"
};
/* Definitions for honeyCombMutex */
osMutexId_t honeyCombMutexHandle;
const osMutexAttr_t honeyCombMutex_attributes = {
  .name = "honeyCombMutex"
};
/* Definitions for printUartMutex */
osMutexId_t printUartMutexHandle;
const osMutexAttr_t printUartMutex_attributes = {
  .name = "printUartMutex"
};
/* Definitions for loraQueue */
osMessageQueueId_t loraQueueHandle;
const osMessageQueueAttr_t loraQueue_attributes = {
  .name = "loraQueue"
};
/* Definitions for tim3Sem */
osSemaphoreId_t tim3SemHandle;
const osSemaphoreAttr_t tim3Sem_attributes = {
  .name = "tim3Sem"
};
/* Definitions for uart4RxSem */
osSemaphoreId_t uart4RxSemHandle;
const osSemaphoreAttr_t uart4RxSem_attributes = {
  .name = "uart4RxSem"
};
/* Definitions for loraRxSem */
osSemaphoreId_t loraRxSemHandle;
const osSemaphoreAttr_t loraRxSem_attributes = {
  .name = "loraRxSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* creation of rssiMutex */
  rssiMutexHandle = osMutexNew(&rssiMutex_attributes);

  /* creation of loraMutex */
  loraMutexHandle = osMutexNew(&loraMutex_attributes);

  /* creation of honeyCombMutex */
  honeyCombMutexHandle = osMutexNew(&honeyCombMutex_attributes);

  /* creation of printUartMutex */
  printUartMutexHandle = osMutexNew(&printUartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */
  /* creation of tim3Sem */
  tim3SemHandle = osSemaphoreNew(1, 0, &tim3Sem_attributes);

  /* creation of uart4RxSem */
  uart4RxSemHandle = osSemaphoreNew(1, 0, &uart4RxSem_attributes);

  /* creation of loraRxSem */
  loraRxSemHandle = osSemaphoreNew(1, 0, &loraRxSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */
  /* creation of loraQueue */
  loraQueueHandle = osMessageQueueNew (4, sizeof(uint8_t), &loraQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ESP32_Comm */
  ESP32_CommHandle = osThreadNew(StartTask02, NULL, &ESP32_Comm_attributes);

  /* creation of LoRa_task */
  LoRa_taskHandle = osThreadNew(StartTask03, NULL, &LoRa_task_attributes);

  /* creation of Timing_Task */
  Timing_TaskHandle = osThreadNew(StartTask04, NULL, &Timing_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END defaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the ESP32_Comm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN ESP32_Comm */
  uint8_t error_flag = 0;
  char debug[80];
  /* Infinite loop */
  for(;;)
  {
	error_flag = 0;
	osSemaphoreAcquire(uart4RxSemHandle, osWaitForever);

	if ((uint8_t)uart_rx_buffer[0] != ESP32_CHECK) {
		error_flag = 1;
		HAL_UART_Receive_DMA(&huart4, uart_rx_buffer, 15);
	}

	if ((uint8_t)uart_rx_buffer[14] != calculate_crc8(&uart_rx_buffer[1], 13)) {
		error_flag = 1;
		print_debug("CRC ERR\r\n");
		HAL_UART_Receive_DMA(&huart4, uart_rx_buffer, 15);
	}

	if(!error_flag) {
		sprintf(debug, "RX[%d]: %d/%d/%d\r\n",
				rssi_index, uart_rx_buffer[1], uart_rx_buffer[7], uart_rx_buffer[13]);
		print_debug(debug);

		osMutexAcquire(rssiMutexHandle, osWaitForever);
		new_rssi_load(&uart_rx_buffer[1], honey_comb.transmission.rssi_buffer, &rssi_index);
		osMutexRelease(rssiMutexHandle);

		uint8_t cmd = 1;
		osMessageQueuePut(loraQueueHandle, &cmd, 0, 0);

		HAL_UART_Receive_DMA(&huart4, uart_rx_buffer, 15);
	}
  }
  /* USER CODE END ESP32_Comm */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the LoRa_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN LoRa_task */
  uint8_t status;
  uint8_t cmd;
  uint8_t master_ack;
  uint8_t ack;
  /* Infinite loop */
  for(;;)
  {
	  if (osMessageQueueGet(loraQueueHandle, &cmd, NULL, 1000) == osOK) { //Esperamos recibir algun comando
		osMutexAcquire(honeyCombMutexHandle, osWaitForever);
		status = honey_comb.status;
		master_ack = honey_comb.master_acknowledge;
		osMutexRelease(honeyCombMutexHandle);

		osMutexAcquire(loraMutexHandle, osWaitForever);

		if (cmd == 1) {
			if (status == SCAN || status == DETECTION) {
				LoRa_transmit_scan_pkg(&myLoRa, &honey_comb);
			} else if (status == TRIANGULATION) {
				LoRa_transmit_triang_pkg(&myLoRa, &honey_comb);
			}
		} else if (cmd == 2) {
			if (!master_ack) {
				ack = LoRa_Master_connection(&myLoRa, &honey_comb, loraRxSemHandle);
				if (ack) {
					osMutexAcquire(honeyCombMutexHandle, osWaitForever);
					honey_comb.master_acknowledge = 1;
					if(honey_comb.status == NODE_ERROR) { 						//Si hay conexion y algo falla se transmite
						LoRa_transmit_error_pkg(&myLoRa, &honey_comb);			//Envia el status de los devices para visualizar error en dashboard y central unit
					} else {
						honey_comb.status = SCAN;
						HAL_UART_Receive_DMA(&huart4, uart_rx_buffer, 15);
					}
					osMutexRelease(honeyCombMutexHandle);
				}
			}
		}
		osMutexRelease(loraMutexHandle);
	 }
  }
  /* USER CODE END LoRa_task */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Timing_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN Timing_Task */
  uint8_t cmd;
  uint8_t status;
  uint8_t master_ack;
  HAL_TIM_Base_Start_IT(&htim3);
  /* Infinite loop */
  for(;;)
  {
	    osSemaphoreAcquire(tim3SemHandle, osWaitForever);

	    osMutexAcquire(honeyCombMutexHandle, osWaitForever);
	    master_ack = honey_comb.master_acknowledge;
	    status = honey_comb.status;
	    osMutexRelease(honeyCombMutexHandle);

	    if (master_ack) {
	    	if(status == NODE_ERROR) {
	    		print_debug("NODE_ERROR_WAITING_FIX\r\n");
	    	}
	    	else {
		        esp32_scan(&huart4);
		        print_debug("SCAN\r\n");
	    	}
	    }
	    else {
			print_debug("Finding...\r\n");
			cmd = 2;
			osMessageQueuePut(loraQueueHandle, &cmd, 0, 0);
	    }
  }
  /* USER CODE END Timing_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void print_debug(const char *msg) {
    osMutexAcquire(printUartMutexHandle, osWaitForever);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    osMutexRelease(printUartMutexHandle);
}

void new_rssi_load(int8_t* data, scan_t* history, uint8_t* index) {
	memcpy(history[*index].rssi, data, RSSI_BUFFER_SIZE);
	*index = (*index + 1) % HISTORY_SIZE;
}
/* USER CODE END Application */

