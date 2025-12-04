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
#include "MAX17048.h"
#include "Neo_M6.h"
#include "HC12.h"
#include "FFT_DroneDetection_Microphone.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//THRESHOLDS
#define NET_COUNT_TH     19.19f
#define CH_ACTIVE_TH     6.06f
#define DELTA_TH         1.40f
#define CH_CONC_TH       0.41f
#define RANGE_TH         22.89f
#define DETECTION_THRESHOLD 70		//Contempla minimo un 70 en signal y un 80 en mic

//HIVEMASTER_COMMANDS
#define SLEEP_CMD 0xA0
#define DETECTION_ACK_CMD 0xB0
#define DRONE_LOST_ACK_CMD 0xC0
#define ENERGY_ACK_CMD 0xC1
#define GPS_ACK_CMD 0xC2
#define DRONE_LOST_AUX_CMD 0xD0
#define WKP_CMD 0xE0

//TRANSMISSION
#define MAX_RETRIES      5
#define TRIANGULATION_CMD 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern HoneyComb_m honey_comb;
extern uint8_t rssi_index;
extern int8_t uart_rx_buffer[15];
extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart5;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern uint8_t gps_buffer[512];
extern GPS_Data_t gps_data;
extern uint8_t adc_half;
RetxTracker retx_tracker = {0, 0};
uint8_t current_cmd = 0;


void print_debug(const char *msg);
void new_rssi_load(int8_t* data, scan_t* history, uint8_t* index);
uint8_t drone_signal_detection(scan_t* history);
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ESP32_Task */
osThreadId_t ESP32_TaskHandle;
const osThreadAttr_t ESP32_Task_attributes = {
  .name = "ESP32_Task",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};
/* Definitions for HC12_Task */
osThreadId_t HC12_TaskHandle;
const osThreadAttr_t HC12_Task_attributes = {
  .name = "HC12_Task",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 512 * 4
};
/* Definitions for Battery_Task */
osThreadId_t Battery_TaskHandle;
const osThreadAttr_t Battery_Task_attributes = {
  .name = "Battery_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for GPS_Task */
osThreadId_t GPS_TaskHandle;
const osThreadAttr_t GPS_Task_attributes = {
  .name = "GPS_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for DetectionTask */
osThreadId_t DetectionTaskHandle;
const osThreadAttr_t DetectionTask_attributes = {
  .name = "DetectionTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 12000 * 4
};
/* Definitions for HC12_Rx_Task */
osThreadId_t HC12_Rx_TaskHandle;
const osThreadAttr_t HC12_Rx_Task_attributes = {
  .name = "HC12_Rx_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for SleepTask */
osThreadId_t SleepTaskHandle;
const osThreadAttr_t SleepTask_attributes = {
  .name = "SleepTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for rssiMutex */
osMutexId_t rssiMutexHandle;
const osMutexAttr_t rssiMutex_attributes = {
  .name = "rssiMutex"
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
/* Definitions for retxMutex */
osMutexId_t retxMutexHandle;
const osMutexAttr_t retxMutex_attributes = {
  .name = "retxMutex"
};
/* Definitions for hc12Queue */
osMessageQueueId_t hc12QueueHandle;
const osMessageQueueAttr_t hc12Queue_attributes = {
  .name = "hc12Queue"
};
/* Definitions for scoreQueue */
osMessageQueueId_t scoreQueueHandle;
const osMessageQueueAttr_t scoreQueue_attributes = {
  .name = "scoreQueue"
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
/* Definitions for hc12RxSem */
osSemaphoreId_t hc12RxSemHandle;
const osSemaphoreAttr_t hc12RxSem_attributes = {
  .name = "hc12RxSem"
};
/* Definitions for gpsSem */
osSemaphoreId_t gpsSemHandle;
const osSemaphoreAttr_t gpsSem_attributes = {
  .name = "gpsSem"
};
/* Definitions for chargerSem */
osSemaphoreId_t chargerSemHandle;
const osSemaphoreAttr_t chargerSem_attributes = {
  .name = "chargerSem"
};
/* Definitions for i2c2RxSem */
osSemaphoreId_t i2c2RxSemHandle;
const osSemaphoreAttr_t i2c2RxSem_attributes = {
  .name = "i2c2RxSem"
};
/* Definitions for lpuart1RxSem */
osSemaphoreId_t lpuart1RxSemHandle;
const osSemaphoreAttr_t lpuart1RxSem_attributes = {
  .name = "lpuart1RxSem"
};
/* Definitions for newRssiSem */
osSemaphoreId_t newRssiSemHandle;
const osSemaphoreAttr_t newRssiSem_attributes = {
  .name = "newRssiSem"
};
/* Definitions for sleepAckSem */
osSemaphoreId_t sleepAckSemHandle;
const osSemaphoreAttr_t sleepAckSem_attributes = {
  .name = "sleepAckSem"
};
/* Definitions for sleepSem */
osSemaphoreId_t sleepSemHandle;
const osSemaphoreAttr_t sleepSem_attributes = {
  .name = "sleepSem"
};
/* Definitions for wkpCmdSem */
osSemaphoreId_t wkpCmdSemHandle;
const osSemaphoreAttr_t wkpCmdSem_attributes = {
  .name = "wkpCmdSem"
};
/* Definitions for esp32AckSem */
osSemaphoreId_t esp32AckSemHandle;
const osSemaphoreAttr_t esp32AckSem_attributes = {
  .name = "esp32AckSem"
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

  /* creation of honeyCombMutex */
  honeyCombMutexHandle = osMutexNew(&honeyCombMutex_attributes);

  /* creation of printUartMutex */
  printUartMutexHandle = osMutexNew(&printUartMutex_attributes);

  /* creation of retxMutex */
  retxMutexHandle = osMutexNew(&retxMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */
  /* creation of tim3Sem */
  tim3SemHandle = osSemaphoreNew(1, 0, &tim3Sem_attributes);

  /* creation of uart4RxSem */
  uart4RxSemHandle = osSemaphoreNew(1, 0, &uart4RxSem_attributes);

  /* creation of hc12RxSem */
  hc12RxSemHandle = osSemaphoreNew(1, 0, &hc12RxSem_attributes);

  /* creation of gpsSem */
  gpsSemHandle = osSemaphoreNew(1, 0, &gpsSem_attributes);

  /* creation of chargerSem */
  chargerSemHandle = osSemaphoreNew(1, 0, &chargerSem_attributes);

  /* creation of i2c2RxSem */
  i2c2RxSemHandle = osSemaphoreNew(1, 0, &i2c2RxSem_attributes);

  /* creation of lpuart1RxSem */
  lpuart1RxSemHandle = osSemaphoreNew(1, 0, &lpuart1RxSem_attributes);

  /* creation of newRssiSem */
  newRssiSemHandle = osSemaphoreNew(1, 0, &newRssiSem_attributes);

  /* creation of sleepAckSem */
  sleepAckSemHandle = osSemaphoreNew(1, 0, &sleepAckSem_attributes);

  /* creation of sleepSem */
  sleepSemHandle = osSemaphoreNew(1, 0, &sleepSem_attributes);

  /* creation of wkpCmdSem */
  wkpCmdSemHandle = osSemaphoreNew(1, 0, &wkpCmdSem_attributes);

  /* creation of esp32AckSem */
  esp32AckSemHandle = osSemaphoreNew(1, 0, &esp32AckSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */
  /* creation of hc12Queue */
  hc12QueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &hc12Queue_attributes);
  /* creation of scoreQueue */
  scoreQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &scoreQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ESP32_Task */
  ESP32_TaskHandle = osThreadNew(ESP32_Task, NULL, &ESP32_Task_attributes);

  /* creation of HC12_Task */
  HC12_TaskHandle = osThreadNew(HC12_Task, NULL, &HC12_Task_attributes);

  /* creation of Battery_Task */
  Battery_TaskHandle = osThreadNew(Battery_Task, NULL, &Battery_Task_attributes);

  /* creation of GPS_Task */
  GPS_TaskHandle = osThreadNew(GPS_Task, NULL, &GPS_Task_attributes);

  /* creation of DetectionTask */
  DetectionTaskHandle = osThreadNew(DetectionTask, NULL, &DetectionTask_attributes);

  /* creation of HC12_Rx_Task */
  HC12_Rx_TaskHandle = osThreadNew(HC12_Rx_Task, NULL, &HC12_Rx_Task_attributes);

  /* creation of SleepTask */
  SleepTaskHandle = osThreadNew(SleepTask, NULL, &SleepTask_attributes);

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

/* USER CODE BEGIN Header_ESP32_Task */
/**
* @brief Function implementing the ESP32_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ESP32_Task */
void ESP32_Task(void *argument)
{
  /* USER CODE BEGIN ESP32_Task */
  uint8_t error_flag;
  uint8_t response;
  char debug[50];
  /* Infinite loop */
  for(;;)
  {
	  error_flag = 0;
	  if(osSemaphoreAcquire(tim3SemHandle, osWaitForever) == osOK) {
		  esp32_wkp(&huart4);
		  HAL_UART_Receive_DMA(&huart4, &response, 1);
		  if(osSemaphoreAcquire(uart4RxSemHandle, 1000) == osOK) {
			  if (response == ESP32_ACK) {
				  osDelay(10);
				  esp32_scan(&huart4);
				  HAL_UART_Receive_DMA(&huart4, uart_rx_buffer, 15);
				  print_debug("SCAN\r\n");

					if(osSemaphoreAcquire(uart4RxSemHandle, 1500) == osOK) {
						if ((uint8_t)uart_rx_buffer[0] != ESP32_CHECK) {
							error_flag = 1;
						}

						if ((uint8_t)uart_rx_buffer[14] != calculate_crc8(&uart_rx_buffer[1], 13)) {
							error_flag = 1;
							print_debug("CRC ERR\r\n");
						}

						if(!error_flag) {
							sprintf(debug, "RX[%d]: %d/%d/%d\r\n",
									rssi_index, uart_rx_buffer[1], uart_rx_buffer[7], uart_rx_buffer[13]);
							print_debug(debug);

							osMutexAcquire(rssiMutexHandle, osWaitForever);
							new_rssi_load(&uart_rx_buffer[1], honey_comb.transmission.rssi_buffer, &rssi_index);
							osMutexRelease(rssiMutexHandle);
						}
						//Prendemos semaforo new_rssi
						osSemaphoreRelease(newRssiSemHandle);
					}
			  }
		  }
		  else {
			  print_debug("WKP TIMEOUT\r\n");
			  HAL_UART_DMAStop(&huart4);  //Abortar DMA pendiente
		  }
	  }
  }
  /* USER CODE END ESP32_Task */
}

/* USER CODE BEGIN Header_HC12_Task */
/**
* @brief Function implementing the HC12_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HC12_Task */
void HC12_Task(void *argument)
{
  /* USER CODE BEGIN HC12_Task */
	  uint8_t cmd;
  /* Infinite loop */
  for(;;)
  {
	osMutexAcquire(retxMutexHandle, osWaitForever);
	if (current_cmd == 0) {
	  //Intentar obtener ALERT primero (timeout 0 = no bloquear)
	  if (osMessageQueueGet(hc12QueueHandle, &cmd, NULL, 0) == osOK) {
		  if (cmd == 1) {  //Es ALERT
			  current_cmd = cmd;
			  retx_tracker.pending_cmd = cmd;
			  retx_tracker.retry_count = 0;
		  } else {
			  //No es ALERT, devolver a la cola y esperar
			  osMessageQueuePut(hc12QueueHandle, &cmd, 0, 0);
			  cmd = 0;
		  }
	  }

	  //Si no obtuvimos ALERT, esperar cualquier comando
	  if (current_cmd == 0 && osMessageQueueGet(hc12QueueHandle, &cmd, NULL, 10) == osOK) {
		  current_cmd = cmd;
		  retx_tracker.pending_cmd = cmd;
		  retx_tracker.retry_count = 0;
	  }
	}
	osMutexRelease(retxMutexHandle);

	//Reintentar actual
	osMutexAcquire(retxMutexHandle, osWaitForever);
	if (retx_tracker.pending_cmd > 0 &&
		retx_tracker.pending_cmd != TRIANGULATION_CMD &&
		retx_tracker.retry_count < MAX_RETRIES) {

		cmd = retx_tracker.pending_cmd;
		retx_tracker.retry_count++;
		osMutexRelease(retxMutexHandle);

		//Enviar paquete
		osMutexAcquire(honeyCombMutexHandle, osWaitForever);

		switch (cmd) {
			case 1:	//ALERT (DETECTION o DRONE_LOST o SLEEPING)
				HC12_transmit_alert_pkg(&huart5, &honey_comb);
				print_debug("HC12_TX: ALERT\r\n");
				break;

			case 2:	//ENERGY
				HC12_transmit_energy_pkg(&huart5, &honey_comb);
				print_debug("HC12_TX: ENERGY\r\n");
				break;

			case 3:	//GPS
				HC12_transmit_gps_pkg(&huart5, &honey_comb);
				print_debug("HC12_TX: GPS\r\n");
				break;

			case TRIANGULATION_CMD:	//TRIANGULATION (constante=4)
				HC12_transmit_triang_pkg(&huart5, &honey_comb);
				print_debug("HC12_TX: TRIANG\r\n");
				break;
		}

		osMutexRelease(honeyCombMutexHandle);
		osDelay(300);  //Espaciado entre reintentos (HC12 más lento que LoRa)
	}
	else if (retx_tracker.retry_count >= MAX_RETRIES) {
		//Falló tras 5 reintentos
		print_debug("HC12_TX: MAX_RETRIES REACHED\r\n");
		current_cmd = 0;  //Liberar para siguiente
		retx_tracker.retry_count = 0;
		retx_tracker.pending_cmd = 0;
		osMutexRelease(retxMutexHandle);
	}
	else {
		osMutexRelease(retxMutexHandle);
	}
	osDelay(100);
	}
  /* USER CODE END HC12_Task */
}

/* USER CODE BEGIN Header_Battery_Task */
/**
* @brief Function implementing the Battery_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Battery_Task */
void Battery_Task(void *argument)
{
  /* USER CODE BEGIN Battery_Task */
  uint8_t cmd;
  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreAcquire(chargerSemHandle, osWaitForever) == osOK) {
		osMutexAcquire(honeyCombMutexHandle, osWaitForever);
		honey_comb.transmission.energy_data.Voltage = MAX17048_getVoltage(&hi2c2);
		honey_comb.transmission.energy_data.Percentage = MAX17048_getPercentage(&hi2c2);
		honey_comb.transmission.energy_data.DischargeRate = MAX17048_getDisChargeRate(&hi2c2);
		honey_comb.transmission.transmission_type = ENERGY;
		osMutexRelease(honeyCombMutexHandle);

		cmd = 2;
		osMessageQueuePut(hc12QueueHandle, &cmd, 0, 100);
	  }
  }
  /* USER CODE END Battery_Task */
}

/* USER CODE BEGIN Header_GPS_Task */
/**
* @brief Function implementing the GPS_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GPS_Task */
void GPS_Task(void *argument)
{
  /* USER CODE BEGIN GPS_Task */
	uint8_t cmd;
	float_t lat, lon;
  /* Infinite loop */
  for(;;)
  {
      if(osSemaphoreAcquire(gpsSemHandle, osWaitForever) == osOK) {
    	  if(osSemaphoreAcquire(lpuart1RxSemHandle, 1000) == osOK) {
    		  if(Process_GPS_Buffer(gps_buffer, &lat, &lon)) {
    			  osMutexAcquire(honeyCombMutexHandle, osWaitForever);
				  honey_comb.transmission.location_data.latitude = lat;
				  honey_comb.transmission.location_data.longitude = lon;
				  honey_comb.transmission.transmission_type = GPS;
				  osMutexRelease(honeyCombMutexHandle);

				  cmd = 3;
				  osMessageQueuePut(hc12QueueHandle, &cmd, 0, 100);
				  print_debug("GPS OK\r\n");
			  }
    		  else {
				  print_debug("GPS NO FIX\r\n");
			  }
    	  }
      }
  }
  /* USER CODE END GPS_Task */
}

/* USER CODE BEGIN Header_DetectionTask */
/**
* @brief Function implementing the DetectionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DetectionTask */
void DetectionTask(void *argument)
{
  /* USER CODE BEGIN DetectionTask */
	char debug[35];

	uint8_t cmd;
	uint8_t status;
	float_t signal_score, mic_score, total_score;

  /* Infinite loop */
  for(;;)
  {
	if(honey_comb.node_role == detector) {
		signal_score = 0, mic_score = 0, total_score = 0;
		if(osSemaphoreAcquire(newRssiSemHandle, osWaitForever) == osOK) {
			osMutexAcquire(rssiMutexHandle, osWaitForever);
			signal_score = (float_t)drone_signal_detection(honey_comb.transmission.rssi_buffer);
			osMutexRelease(rssiMutexHandle);

			mic_score = Microphone_getDetectionScore(adc_half);

			total_score = ((signal_score * 0.8))+((mic_score * 0.2));

            sprintf(debug, "Mic:%.0f Sig:%.0f Tot:%.0f\r\n",
                    mic_score, signal_score, total_score);
            print_debug(debug);

			osMutexAcquire(honeyCombMutexHandle, osWaitForever);
			status = honey_comb.status;
			osMutexRelease(honeyCombMutexHandle);

			if(total_score > DETECTION_THRESHOLD) {
				if(status == TRIANGULATION) { 							//Detecto y ya estoy triangulando
					cmd = 4;
					osMessageQueuePut(hc12QueueHandle, &cmd, 0, 100);
				}
				else {
					osMutexAcquire(honeyCombMutexHandle, osWaitForever);//Preparo para iniciar detección
					honey_comb.status = DETECTION;
					honey_comb.transmission.transmission_type = ALERT;
					osMutexRelease(honeyCombMutexHandle);
					print_debug("DRON DETECTADO, esperamos central\r\n");
					cmd = 1;
					osMessageQueuePut(hc12QueueHandle, &cmd, 0, 100);
				}
			} else if ((total_score < DETECTION_THRESHOLD) && status == TRIANGULATION) {
				print_debug("DRON PERDIDO...\r\n");
				honey_comb.status = DRONE_LOST;
				cmd = 1;
				osMessageQueuePut(hc12QueueHandle, &cmd, 0, 100);
			}
		}
	}
	else {
		osDelay(5000);
	}
  }
  /* USER CODE END DetectionTask */
}

/* USER CODE BEGIN Header_HC12_Rx_Task */
/**
* @brief Function implementing the HC12_Rx_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HC12_Rx_Task */
void HC12_Rx_Task(void *argument)
{
  /* USER CODE BEGIN HC12_Rx_Task */
    uint8_t initialized = 0;

    HC12_StartReceiveIdleDMA(&huart5, honey_comb.rx_buffer, HC12_MAX_SIZE);
    print_debug("HC12_Rx_Task: RX DMA iniciado\r\n");
  /* Infinite loop */
  for(;;)
  {
	//INICIALIZACIÓN: Poner HC12 en modo escucha
	if (!initialized) {
	   HC12_StartReceiveIdleDMA(&huart5, honey_comb.rx_buffer, HC12_MAX_SIZE);
	   print_debug("HC12_Rx_Task: RX DMA iniciado\r\n");
	   initialized = 1;
	}

	//ESPERAR INTERRUPCIÓN: Timeout corto para evitar gaps largos
	if (osSemaphoreAcquire(hc12RxSemHandle, 100) == osOK) {

	   osMutexAcquire(honeyCombMutexHandle, osWaitForever);

	   //VALIDAR FORMATO Y ID
	   if (honey_comb.rx_buffer[0] == honey_comb.baliza_id) {

		   //PROCESAR ACK ESPERADO (desde TX)
		   if (honey_comb.rx_buffer[1] == DETECTION_ACK_CMD) {  // 0xB0
			   honey_comb.status = TRIANGULATION;
			   print_debug("HC12_RX: DETECTION_ACK [OK]\r\n");

			   //Limpiar flag de retransmisión
			   osMutexAcquire(retxMutexHandle, osWaitForever);
			   if (retx_tracker.pending_cmd == 1 && current_cmd == 1) {
				   retx_tracker.pending_cmd = 0;
				   retx_tracker.retry_count = 0;
				   current_cmd = 0;
			   }
			   osMutexRelease(retxMutexHandle);

		   } else if (honey_comb.rx_buffer[1] == DRONE_LOST_ACK_CMD) {  // 0xC0
			   honey_comb.status = SCAN;
			   print_debug("HC12_RX: DRONE_LOST_ACK [OK]\r\n");

			   //Limpiar flag de retransmisión
			   osMutexAcquire(retxMutexHandle, osWaitForever);
			   if (retx_tracker.pending_cmd == 1 && current_cmd == 1) {
				   retx_tracker.pending_cmd = 0;
				   retx_tracker.retry_count = 0;
				   current_cmd = 0;
			   }
			   osMutexRelease(retxMutexHandle);

		   } else if (honey_comb.rx_buffer[1] == ENERGY_ACK_CMD) {  // 0xC1
			   print_debug("HC12_RX: ENERGY_ACK [OK]\r\n");

			   osMutexAcquire(retxMutexHandle, osWaitForever);
			   if (retx_tracker.pending_cmd == 2 && current_cmd == 2) {
				   retx_tracker.pending_cmd = 0;
				   retx_tracker.retry_count = 0;
				   current_cmd = 0;
			   }
			   osMutexRelease(retxMutexHandle);

		   } else if (honey_comb.rx_buffer[1] == GPS_ACK_CMD) {  // 0xC2
			   print_debug("HC12_RX: GPS_ACK [OK]\r\n");

			   osMutexAcquire(retxMutexHandle, osWaitForever);
			   if (retx_tracker.pending_cmd == 3 && current_cmd == 3) {
				   retx_tracker.pending_cmd = 0;
				   retx_tracker.retry_count = 0;
				   current_cmd = 0;
			   }
			   osMutexRelease(retxMutexHandle);

		   } else if (honey_comb.rx_buffer[1] == WKP_CMD) {  // 0xE0
			   print_debug("HC12_RX: WKP_CMD [OK]\r\n");
			   if (honey_comb.status == SLEEPING) {
				   osSemaphoreRelease(wkpCmdSemHandle);
			   }

		   }
		   //PROCESAR COMANDO INESPERADO (desde SCAN/SLEEPING)
		   else if (honey_comb.rx_buffer[1] == SLEEP_CMD) {  // 0xA0
			   print_debug("HC12_RX: SLEEP_CMD [OK]\r\n");
			   osSemaphoreRelease(sleepSemHandle);

		   } else if (honey_comb.rx_buffer[1] == DRONE_LOST_AUX_CMD &&  // 0xD0
					  honey_comb.node_role == aux) {
			   print_debug("HC12_RX: DRONE_LOST_AUX [OK]\r\n");
			   osSemaphoreRelease(sleepSemHandle);
		   }
		   else {
			   print_debug("HC12_RX: Unknown CMD\r\n");
		   }
	   }
	   else if (honey_comb.rx_buffer[0] != 0x00) {
		   print_debug("HC12_RX: ID mismatch\r\n");
	   }

	   osMutexRelease(honeyCombMutexHandle);

	   //REINICIAR ESCUCHA: Volver a escuchar después de procesar
	   HC12_StartReceiveIdleDMA(&huart5, honey_comb.rx_buffer, HC12_MAX_SIZE);
	}
	else {
	   //TIMEOUT: No recibimos nada en 100ms
	   //Simplemente reiniciar RX
	   HC12_StartReceiveIdleDMA(&huart5, honey_comb.rx_buffer, HC12_MAX_SIZE);
	}
	}
  /* USER CODE END HC12_Rx_Task */
}

/* USER CODE BEGIN Header_SleepTask */
/**
* @brief Function implementing the SleepTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SleepTask */
void SleepTask(void *argument)
{
  /* USER CODE BEGIN SleepTask */
	uint8_t cmd;
  /* Infinite loop */
  for(;;)
  {
	if (osSemaphoreAcquire(sleepSemHandle, osWaitForever) == osOK) {
		print_debug("TONY ENTRANDO SLEEP MODE\r\n");

		osMutexAcquire(honeyCombMutexHandle,osWaitForever);
		honey_comb.status = SLEEPING;
		honey_comb.transmission.transmission_type = ALERT;
		osMutexRelease(honeyCombMutexHandle);

		cmd = 1;
		osMessageQueuePut(hc12QueueHandle, &cmd, 0, 100);

		osDelay(1000); //Tiempo que esperamos para avisar a central que en efecto vamos a dormir
        //DESACTIVAR INTERRUPCIONES UART ANTES DE DORMIR
        HAL_UART_DMAStop(&huart4);    //ESP32
        HAL_UART_DMAStop(&hlpuart1);  //GPS
        HAL_NVIC_DisableIRQ(UART4_IRQn);
        HAL_NVIC_DisableIRQ(LPUART1_IRQn);

		//Entramos a SLEEP y SOLO salimos de ahi por el reset
        while(1) {
            HAL_SuspendTick();
            HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

            //Despierta aquí por IRQ LoRa
            HAL_ResumeTick();

            //Validar comando WKP (timeout 2s)
            if (osSemaphoreAcquire(wkpCmdSemHandle, 2000) == osOK) {
                print_debug("WKP validado, reseteando\r\n");
                osDelay(100);
                HAL_NVIC_SystemReset();
            }
            //Si timeout o comando inválido, el nodo vuelve a dormir
            print_debug("Comando invalido, re-sleep\r\n");
            //Buelve a HAL_PWR_EnterSTOPMode
        }
	}
  }
  /* USER CODE END SleepTask */
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

uint8_t drone_signal_detection(scan_t* history) {
    //Concatenar últimas 4 ventanas de scan
    int8_t rssi_data[52];
    char debug_msg[80];

    for (int i = 0; i < 13; i++) {
        rssi_data[i] = history[(rssi_index - 3 + HISTORY_SIZE) % HISTORY_SIZE].rssi[i];
        rssi_data[13 + i] = history[(rssi_index - 2 + HISTORY_SIZE) % HISTORY_SIZE].rssi[i];
        rssi_data[26 + i] = history[(rssi_index - 1 + HISTORY_SIZE) % HISTORY_SIZE].rssi[i];
        rssi_data[39 + i] = history[rssi_index].rssi[i];
    }

    uint8_t drone_votes = 0;

    /* Feature 1: net_count */
    uint16_t net_count = 0;
    for (int i = 0; i < 52; i++) {
        if (rssi_data[i] > -80) net_count++;
    }
    sprintf(debug_msg, "F1_net_count: %d (threshold: %.2f)\r\n", net_count, NET_COUNT_TH);
    print_debug(debug_msg);
    if (net_count < NET_COUNT_TH) drone_votes++;

    /* Feature 2: ch_active */
    uint16_t ch_bitmap = 0;
    for (int i = 0; i < 52; i++) {
        ch_bitmap |= (1 << (i % 14));
    }
    uint8_t ch_active = 0;
    for (int i = 0; i < 14; i++) {
        if (ch_bitmap & (1 << i)) ch_active++;
    }
    sprintf(debug_msg, "F2_ch_active: %d (threshold: %.2f)\r\n", ch_active, CH_ACTIVE_TH);
    print_debug(debug_msg);
    if (ch_active < CH_ACTIVE_TH) drone_votes++;

    /* Feature 3: delta (CMSIS std dev) */
    float32_t rssi_float[52];
    for (int i = 0; i < 52; i++) {
        rssi_float[i] = (float32_t)rssi_data[i];
    }
    float32_t delta;
    arm_std_f32(rssi_float, 52, &delta);
    sprintf(debug_msg, "F3_delta: %.2f (threshold: %.2f)\r\n", delta, DELTA_TH);
    print_debug(debug_msg);
    if (delta > DELTA_TH) drone_votes++;

    /* Feature 4: ch_conc */
    float32_t ch_conc = (float32_t)ch_active / 14.0f;
    sprintf(debug_msg, "F4_ch_conc: %.2f (threshold: %.2f)\r\n", ch_conc, CH_CONC_TH);
    print_debug(debug_msg);
    if (ch_conc > CH_CONC_TH) drone_votes++;

    /* Feature 5: range (CMSIS max/min) */
    uint32_t idx_max, idx_min;
    float32_t max_rssi, min_rssi;
    arm_max_f32(rssi_float, 52, &max_rssi, &idx_max);
    arm_min_f32(rssi_float, 52, &min_rssi, &idx_min);
    float32_t range = max_rssi - min_rssi;
    sprintf(debug_msg, "F5_range: %.2f (threshold: %.2f)\r\n", range, RANGE_TH);
    print_debug(debug_msg);
    if (range < RANGE_TH) drone_votes++;

    sprintf(debug_msg, "VOTES: %d/5 -> SCORE: %d%%\r\n", drone_votes, (drone_votes * 100) / 5);
    print_debug(debug_msg);

    return (drone_votes * 100) / 5;
}

/* USER CODE END Application */

