/*
 * Esp32.c
 *
 *  Created on: Nov 8, 2025
 *      Author: davox
 */

#include "Esp32.h"

uint8_t esp32_connection(UART_HandleTypeDef* _uart) {
    uint8_t response = 0;
    uint8_t check_msg = ESP32_CHECK;

    //Despertar ESP32
    HAL_GPIO_WritePin(ESP32_WKP_GPIO_Port, ESP32_WKP_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(ESP32_WKP_GPIO_Port, ESP32_WKP_Pin, GPIO_PIN_RESET);

    //Enviar CHECK
    HAL_UART_Transmit(_uart, &check_msg, 1, 100);

    //Esperar ACK
    if (HAL_UART_Receive(_uart, &response, 1, 500) == HAL_OK) {
        if(response == ESP32_ACK) {
        	HAL_UART_AbortReceive(_uart);
        	HAL_Delay(50);
        	return 0;
        }
    }
    HAL_UART_AbortReceive(_uart);
    return 1;
}

void esp32_scan(UART_HandleTypeDef *_uart) {
    uint8_t scan_msg = ESP32_SCAN_CMD;
    HAL_UART_Transmit(_uart, &scan_msg, 1, 100);
}

