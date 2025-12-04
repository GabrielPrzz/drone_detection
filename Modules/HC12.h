/*
 * HC12.h
 *
 *  Created on: Dec 3, 2025
 *      Author: davox
 */

#ifndef HC12_H_
#define HC12_H_

#include "main.h"

/* Packet Sizes */
#define HC12_ALERT_PKG_SIZE         3   //[ID][STATUS][TX_TYPE]
#define HC12_ENERGY_PKG_SIZE        15  //[ID][STATUS][TX_TYPE][VOLTAGE(4)][%(4)][RATE(4)]
#define HC12_GPS_PKG_SIZE           11  //[ID][STATUS][TX_TYPE][LAT(4)][LONG(4)]
#define HC12_TRIANG_PKG_SIZE        15  //[ID][STATUS][13×RSSI]
#define HC12_ACK_PKG_SIZE           3   //[ID][0xAA][NODE_ROLE]
#define HC12_ERROR_PKG_SIZE         7   //[ID][STATUS][ESP32][HC12][GPS][CHRG][MIC]
#define HC12_MASTER_CONNECT_SIZE    2   //[ID][STATUS]
#define HC12_MAX_SIZE               32

/* AT Commands */
#define HC12_CMD_BAUDRATE   "AT+B9600\r\n"
#define HC12_CMD_CHANNEL    "AT+C001\r\n"
#define HC12_CMD_POWER      "AT+P8\r\n"
#define HC12_CMD_MODE       "AT+FU3\r\n"

/* Function Prototypes */
void HC12_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *set_port, uint16_t set_pin);
void HC12_SendATCommand(UART_HandleTypeDef *huart, const char* cmd);
void HC12_StartReceiveIdleDMA(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size);

// Transmission functions (non-blocking DMA)
void HC12_transmit_alert_pkg(UART_HandleTypeDef *huart, HoneyComb_m *comb);
void HC12_transmit_energy_pkg(UART_HandleTypeDef *huart, HoneyComb_m *comb);
void HC12_transmit_error_pkg(UART_HandleTypeDef *huart, HoneyComb_m *comb);
void HC12_transmit_gps_pkg(UART_HandleTypeDef *huart, HoneyComb_m *comb);
void HC12_transmit_triang_pkg(UART_HandleTypeDef *huart, HoneyComb_m *comb);
void HC12_Master_connection(UART_HandleTypeDef *huart, HoneyComb_m *comb);


#endif /* HC12_H_ */
