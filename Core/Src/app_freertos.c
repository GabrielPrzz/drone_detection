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
#include "MAX17048.h"
#include "Neo_M6.h"
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
#define DETECTION_THRESHOLD 60

//HIVEMASTER_COMMANDS
#define SLEEP_CMD 0xA0
#define DETECTION_ACK_CMD 0xB0
#define DRONE_LOST_ACK_CMD 0xC0
#define DRONE_LOST_AUX_CMD 0xD0
#define WKP_CMD 0xE0
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
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern LoRa myLoRa;
extern uint8_t gps_buffer[512];
extern GPS_Data_t gps_data;


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
/* Definitions for LoRa_Task */
osThreadId_t LoRa_TaskHandle;
const osThreadAttr_t LoRa_Task_attributes = {
  .name = "LoRa_Task",
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
  .stack_size = 512 * 4
};
/* Definitions for LoRa_Rx_Task */
osThreadId_t LoRa_Rx_TaskHandle;
const osThreadAttr_t LoRa_Rx_Task_attributes = {
  .name = "LoRa_Rx_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
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
/* Definitions for loraRxSem */
osSemaphoreId_t loraRxSemHandle;
const osSemaphoreAttr_t loraRxSem_attributes = {
  .name = "loraRxSem"
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
  /* creation of loraQueue */
  loraQueueHandle = osMessageQueueNew (5, sizeof(uint8_t), &loraQueue_attributes);
  /* creation of scoreQueue */
  scoreQueueHandle = osMessageQueueNew (3, sizeof(uint8_t), &scoreQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ESP32_Task */
  ESP32_TaskHandle = osThreadNew(ESP32_Task, NULL, &ESP32_Task_attributes);

  /* creation of LoRa_Task */
  LoRa_TaskHandle = osThreadNew(LoRa_Task, NULL, &LoRa_Task_attributes);

  /* creation of Battery_Task */
  Battery_TaskHandle = osThreadNew(Battery_Task, NULL, &Battery_Task_attributes);

  /* creation of GPS_Task */
  GPS_TaskHandle = osThreadNew(GPS_Task, NULL, &GPS_Task_attributes);

  /* creation of DetectionTask */
  DetectionTaskHandle = osThreadNew(DetectionTask, NULL, &DetectionTask_attributes);

  /* creation of LoRa_Rx_Task */
  LoRa_Rx_TaskHandle = osThreadNew(LoRa_Rx_Task, NULL, &LoRa_Rx_Task_attributes);

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

/* USER CODE BEGIN Header_LoRa_Task */
/**
* @brief Function implementing the LoRa_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LoRa_Task */
void LoRa_Task(void *argument)
{
  /* USER CODE BEGIN LoRa_Task */
	uint8_t cmd;
  /* Infinite loop */
  for(;;)
  {
	  if (osMessageQueueGet(loraQueueHandle, &cmd, NULL, 1000) == osOK) { //Esperamos recibir algun comando
		  osMutexAcquire(loraMutexHandle, osWaitForever);
		  osMutexAcquire(honeyCombMutexHandle,osWaitForever);
		  switch (cmd) {
		  	  case 1:	//ALERT (SLEEP o DETECTION)
		  		LoRa_transmit_alert_pkg(&myLoRa, &honey_comb);
		  		LoRa_gotoMode(&myLoRa, RXSINGLE_MODE);			//Ponemos en Modo Rx, esperando respuesta
		  		osDelay(10);
			  break;

		  	  case 2:	//BATTERY
		  		LoRa_transmit_energy_pkg(&myLoRa, &honey_comb);
		  	  break;

		  	  case 3:	//GPS
		  		LoRa_transmit_gps_pkg(&myLoRa, &honey_comb);
		  	  break;

		  	  default:	//TRIANGULATION
		  		LoRa_transmit_triang_pkg(&myLoRa, &honey_comb);
		  	  break;
		  }
		  osMutexRelease(honeyCombMutexHandle);
		  osMutexRelease(loraMutexHandle);
	  }
  }
  /* USER CODE END LoRa_Task */
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
		osMessageQueuePut(loraQueueHandle, &cmd, 0, 100);
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
				  osMessageQueuePut(loraQueueHandle, &cmd, 0, 100);
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
	uint8_t signal_score, mic_score;
	uint8_t total_score;

  /* Infinite loop */
  for(;;)
  {
	if(honey_comb.node_role == detector) {
		signal_score = 0, mic_score = 0, total_score = 0;
		if(osSemaphoreAcquire(newRssiSemHandle, 500) == osOK) {
			osMutexAcquire(rssiMutexHandle, osWaitForever);
			signal_score = drone_signal_detection(honey_comb.transmission.rssi_buffer);
			osMutexRelease(rssiMutexHandle);

			//Falta implementar microfono

			total_score = ((signal_score*80)/100)+((mic_score * 20)/100);

			sprintf(debug, "Mic_Score: %d\r\n",
					mic_score);
			print_debug(debug);
			sprintf(debug, "Signal Score: %d\r\n",
					signal_score);
			print_debug(debug);
			sprintf(debug, "Total Score: %d\r\n",
					total_score);
			print_debug(debug);

			osMutexAcquire(honeyCombMutexHandle, osWaitForever);
			status = honey_comb.status;
			osMutexRelease(honeyCombMutexHandle);

			if(total_score > DETECTION_THRESHOLD) {
				if(status == TRIANGULATION) { 							//Detecto y ya estoy triangulando
					cmd = 4;
					osMessageQueuePut(loraQueueHandle, &cmd, 0, 100);
				}
				else {
					osMutexAcquire(honeyCombMutexHandle, osWaitForever);//Preparo para iniciar detección
					honey_comb.status = DETECTION;
					honey_comb.transmission.transmission_type = ALERT;
					osMutexRelease(honeyCombMutexHandle);
					print_debug("DRON DETECTADO, esperamos central\r\n");
					cmd = 1;
					osMessageQueuePut(loraQueueHandle, &cmd, 0, 100);
				}
			} else if ((total_score < DETECTION_THRESHOLD) && status == TRIANGULATION) {
				print_debug("DRON PERDIDO...\r\n");
				honey_comb.status = DRONE_LOST;
				cmd = 1;
				osMessageQueuePut(loraQueueHandle, &cmd, 0, 100);
			}
		}
	}
	osDelay(100);
  }
  /* USER CODE END DetectionTask */
}

/* USER CODE BEGIN Header_LoRa_Rx_Task */
/**
* @brief Function implementing the LoRa_Rx_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LoRa_Rx_Task */
void LoRa_Rx_Task(void *argument)
{
  /* USER CODE BEGIN LoRa_Rx_Task */
    uint8_t num_bytes;
    uint8_t initialized = 0;
  /* Infinite loop */
    for(;;)
       {
	   //INICIALIZACIÓN: Poner LoRa en modo escucha (una sola vez)
	   if (!initialized) {
		   osMutexAcquire(loraMutexHandle, osWaitForever);
		   LoRa_gotoMode(&myLoRa, RXSINGLE_MODE);
		   osMutexRelease(loraMutexHandle);
		   initialized = 1;
	   }

	   //ESPERAR INTERRUPCIÓN: Timeout corto para evitar gaps largos
	   if (osSemaphoreAcquire(loraRxSemHandle, 50) == osOK) {

		   osMutexAcquire(honeyCombMutexHandle, osWaitForever);
		   osMutexAcquire(loraMutexHandle, osWaitForever);

		   //RECEPCIÓN SIN CAMBIAR MODO: Ya estamos en RXSINGLE_MODE
		   num_bytes = LoRa_receive_no_mode_change(&myLoRa, honey_comb.rx_buffer,LORA_MAX_SIZE);

		   if (num_bytes > 0 && honey_comb.rx_buffer[0] == honey_comb.baliza_id) {

			   //PROCESAR COMANDO ESPERADO (desde DETECTION)
			   if (honey_comb.rx_buffer[1] == DETECTION_ACK_CMD) {
				   honey_comb.status = TRIANGULATION;
				   print_debug("RX: DETECTION_ACK [OK]\r\n");

			   } else if (honey_comb.rx_buffer[1] == DRONE_LOST_ACK_CMD) {
				   honey_comb.status = SCAN;
				   print_debug("RX: DRONE_LOST_ACK [OK]\r\n");

			   } else if (honey_comb.rx_buffer[1] == WKP_CMD) {
				   print_debug("RX: WKP_CMD [OK]\r\n");
				   osSemaphoreRelease(wkpCmdSemHandle);
			   }

			   //PROCESAR COMANDO INESPERADO (desde SCAN/SLEEPING)
			   else if (honey_comb.rx_buffer[1] == SLEEP_CMD) {
				   print_debug("RX: SLEEP_CMD [OK]\r\n");
				   osSemaphoreRelease(sleepSemHandle);

			   } else if (honey_comb.rx_buffer[1] == DRONE_LOST_AUX_CMD &&
						  honey_comb.node_role == aux) {
				   print_debug("RX: DRONE_LOST_AUX [OK]\r\n");
				   osSemaphoreRelease(sleepSemHandle);
			   }
		   }
		   else if (num_bytes > 0) {
			   print_debug("RX: Invalid packet received\r\n");
		   }

		   osMutexRelease(loraMutexHandle);
		   osMutexRelease(honeyCombMutexHandle);

		   //REINICIAR ESCUCHA: Volver a RXSINGLE_MODE después de procesar
		   osMutexAcquire(loraMutexHandle, osWaitForever);
		   LoRa_gotoMode(&myLoRa, RXSINGLE_MODE);
		   osMutexRelease(loraMutexHandle);
	   }
	   else {
		   //TIMEOUT: No recibimos nada en 50ms
		   //Asegurar que estamos en RXSINGLE_MODE (por si acaso)
		   osMutexAcquire(loraMutexHandle, 50);
		   LoRa_gotoMode(&myLoRa, RXSINGLE_MODE);
		   osMutexRelease(loraMutexHandle);
	   }
   }
  /* USER CODE END LoRa_Rx_Task */
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
		osMessageQueuePut(loraQueueHandle, &cmd, 0, 100);

		//Entramos a SLEEP
		HAL_SuspendTick();
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
		//Despertamos aqui
		if (osSemaphoreAcquire(wkpCmdSemHandle, osWaitForever) == osOK) {
			//Esperamos validar que nos haya despertado el comando correcto
			HAL_NVIC_SystemReset();
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
    // Concatenar últimas 2 ventanas de scan
    int8_t rssi_data[26];
    for (int i = 0; i < 13; i++) {
        rssi_data[i] = history[(rssi_index - 1 + HISTORY_SIZE) % HISTORY_SIZE].rssi[i];
        rssi_data[13 + i] = history[rssi_index].rssi[i];
    }

    uint8_t drone_votes = 0;

    /* Feature 1: net_count */
    uint16_t net_count = 0;
    for (int i = 0; i < 26; i++) {
        if (rssi_data[i] > -80) net_count++;
    }
    if (net_count < NET_COUNT_TH) drone_votes++;

    /* Feature 2: ch_active */
    uint16_t ch_bitmap = 0;
    for (int i = 0; i < 26; i++) {
        ch_bitmap |= (1 << (i % 14));
    }
    uint8_t ch_active = 0;
    for (int i = 0; i < 14; i++) {
        if (ch_bitmap & (1 << i)) ch_active++;
    }
    if (ch_active < CH_ACTIVE_TH) drone_votes++;

    /* Feature 3: delta (CMSIS std dev) */
    float32_t rssi_float[26];
    for (int i = 0; i < 26; i++) {
        rssi_float[i] = (float32_t)rssi_data[i];
    }
    float32_t delta;
    arm_std_f32(rssi_float, 26, &delta);
    if (delta > DELTA_TH) drone_votes++;

    /* Feature 4: ch_conc */
    float32_t ch_conc = (float32_t)ch_active / 14.0f;
    if (ch_conc > CH_CONC_TH) drone_votes++;

    /* Feature 5: range (CMSIS max/min) */
    uint32_t idx_max, idx_min;
    float32_t max_rssi, min_rssi;
    arm_max_f32(rssi_float, 26, &max_rssi, &idx_max);
    arm_min_f32(rssi_float, 26, &min_rssi, &idx_min);
    float32_t range = max_rssi - min_rssi;
    if (range < RANGE_TH) drone_votes++;

    return (drone_votes * 100) / 5;
}

/* USER CODE END Application */

