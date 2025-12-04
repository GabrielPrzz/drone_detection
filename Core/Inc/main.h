/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os2.h"
#include <string.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN_Pin GPIO_PIN_13
#define BTN_GPIO_Port GPIOC
#define BTN_EXTI_IRQn EXTI13_IRQn
#define ESP32_WKP_Pin GPIO_PIN_5
#define ESP32_WKP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define RSSI_BUFFER_SIZE 		13
#define HISTORY_SIZE 			6

#define LORA_MAX_SIZE 			64
#define LORA_MASTER_CONNECTION_PKG_SIZE 2
#define LORA_ACK_PKG_SIZE		3	//|ID|OxAA|role|
#define LORA_ERROR_PKG_SIZE		7
#define LORA_ENERGY_PKG_SIZE	15
#define LORA_ALERT_PKG_SIZE		3
#define LORA_GPS_PKG_SIZE		11
#define LORA_TRIANG_PKG_SIZE 	15

typedef enum {
	INITIALIZATION,
	NODE_ERROR,							//Alguno de los modulos, ajeno al LoRa fallo o no esta funcionando correctamente
	SCAN,								//Solo escanendo el ambiente pero sin detectar nada todavia, funcionamiento normal
	DETECTION,							//ALERTA previa a la triangulación, central debe confirmar recepcion de la alerta
	TRIANGULATION,						//DRON DETECTADO y se estan mandando datos para la triangulacion, NECESITA ACK DE CENTRAL ANTES DE ENTRAR
	DRONE_LOST,
	SLEEPING,							//NECESITA ACK DE CENTRAL ANTES DE ENTRAR
} BALIZA_STATE;

typedef enum {							//Señales que avisan que tipo de dato mandamos
	ENERGY,
	GPS,
	ALERT,
} TX_TYPE;

typedef enum {
	undefined,
	detector,
	aux,
} system_role;

typedef struct {
    int8_t rssi[RSSI_BUFFER_SIZE]; 		//Este rssi sirve para detectar dron
} scan_t;

typedef struct {
	float_t Voltage;
	float_t Percentage;
	float_t DischargeRate;
} energy_t;

typedef struct {
	float_t latitude;
	float_t longitude;
} gps_t;

typedef struct {
    TX_TYPE transmission_type;
    energy_t energy_data;
    scan_t rssi_buffer[HISTORY_SIZE]; 	//Buffer que almacena HISTORY SIZE arreglos de las lecturas en los 13 canales
    gps_t location_data;
} lora_package;


typedef struct { 						//States para verificación en las Bálizas
	uint8_t HC12_State;					//1 para errores, 0 para OK
	uint8_t Esp32_State;
	uint8_t GPS_State;
	uint8_t Charger_State;
	uint8_t Microphone_State;
} MODULES;

typedef struct {
	uint8_t baliza_id;					//ID ASIGNADO
	BALIZA_STATE status;				//STATUS DE LA BALIZA
	uint8_t rx_buffer[LORA_MAX_SIZE];
	uint8_t tx_buffer[LORA_MAX_SIZE];
	lora_package transmission;			//Variable utilizada para transmitir data
	MODULES devices;					//Variable que almacena el estado de los modulos conectados

	system_role node_role;
	uint8_t master_acknowledge;
	uint8_t rx_len;
} HoneyComb_m;

typedef struct {
    uint8_t pending_cmd;        // Comando pendiente de ACK (0 = ninguno)
    uint8_t retry_count;        // Contador de reintentos
} RetxTracker;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
