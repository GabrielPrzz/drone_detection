/*
 * Esp32.h
 *
 *  Created on: Nov 8, 2025
 *      Author: davox
 */

#include "main.h"

#ifndef ESP32_H_
#define ESP32_H_

#define ESP32_CHECK			0xAA //Señal enviada para ver si esta bien
#define ESP32_ACK			0xBB //Respuesta esperada
#define ESP32_SCAN_CMD		0xCC //Comando para escaneo

uint8_t esp32_connection(UART_HandleTypeDef* _uart);
void esp32_wkp(UART_HandleTypeDef* _uart);
void esp32_scan(UART_HandleTypeDef* _uart);
uint8_t calculate_crc8(int8_t *data, uint8_t len);

#endif /* ESP32_H_ */
