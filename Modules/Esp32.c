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
    uint8_t retries = 3;

    while(retries--) {
        HAL_GPIO_WritePin(ESP32_WKP_GPIO_Port, ESP32_WKP_Pin, GPIO_PIN_SET);
        HAL_Delay(5);
        HAL_GPIO_WritePin(ESP32_WKP_GPIO_Port, ESP32_WKP_Pin, GPIO_PIN_RESET);
        HAL_Delay(30);

        HAL_UART_Transmit(_uart, &check_msg, 1, 100);

        if (HAL_UART_Receive(_uart, &response, 1, 2000) == HAL_OK) {
            if(response == ESP32_ACK) {
                HAL_UART_AbortReceive(_uart);
                HAL_Delay(10);
                return 0;
            }
        }
        HAL_UART_AbortReceive(_uart);
    }
    return 1;
}

void esp32_wkp(UART_HandleTypeDef* _uart) {
    uint8_t response = 0;
    uint8_t check_msg = ESP32_CHECK;

	HAL_GPIO_WritePin(ESP32_WKP_GPIO_Port, ESP32_WKP_Pin, GPIO_PIN_SET);
	osDelay(5);
	HAL_GPIO_WritePin(ESP32_WKP_GPIO_Port, ESP32_WKP_Pin, GPIO_PIN_RESET);
	osDelay(30);

	HAL_UART_Transmit(_uart, &check_msg, 1, 100);
}

void esp32_scan(UART_HandleTypeDef *_uart) {
    uint8_t scan_msg = ESP32_SCAN_CMD;
    HAL_UART_Transmit(_uart, &scan_msg, 1, 100);
}

uint8_t calculate_crc8(int8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}

