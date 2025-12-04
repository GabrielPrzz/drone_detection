/*
 * HC12.c
 *
 *  Created on: Dec 3, 2025
 *      Author: davox
 */

/*
 * HC12.c
 * HC-12 UART Wireless Module Implementation
 */

#include "HC12.h"
#include <string.h>


void HC12_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *set_port, uint16_t set_pin) {
    HAL_GPIO_WritePin(set_port, set_pin, GPIO_PIN_RESET);
    HAL_Delay(100);

    HC12_SendATCommand(huart, HC12_CMD_BAUDRATE);
    HAL_Delay(50);
    HC12_SendATCommand(huart, HC12_CMD_CHANNEL);
    HAL_Delay(50);
    HC12_SendATCommand(huart, HC12_CMD_POWER);
    HAL_Delay(50);
    HC12_SendATCommand(huart, HC12_CMD_MODE);
    HAL_Delay(50);

    HAL_GPIO_WritePin(set_port, set_pin, GPIO_PIN_SET);
    HAL_Delay(100);
}

void HC12_SendATCommand(UART_HandleTypeDef *huart, const char* cmd) {
    HAL_UART_Transmit(huart, (uint8_t*)cmd, strlen(cmd), 100);
}

void HC12_StartReceiveIdleDMA(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size) {
    HAL_UARTEx_ReceiveToIdle_DMA(huart, buffer, size);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

HC12_transmit_alert_pkg(UART_HandleTypeDef *huart, HoneyComb_m *comb) {
    //__Formato__: [ID][STATUS][TX_TYPE]
    static uint8_t pkg[HC12_ALERT_PKG_SIZE];

    if((comb->status == DETECTION || comb->status == SLEEPING || comb->status == DRONE_LOST) &&
       (comb->transmission.transmission_type == ALERT)) {

        pkg[0] = comb->baliza_id;
        pkg[1] = comb->status;
        pkg[2] = comb->transmission.transmission_type;

        HAL_UART_Transmit_DMA(huart, pkg, HC12_ALERT_PKG_SIZE);
    }
}

void HC12_transmit_energy_pkg(UART_HandleTypeDef *huart, HoneyComb_m *comb) {
    //__Formato__: [ID][STATUS][TX_TYPE][VOLTAGE(4)][PERCENTAGE(4)][RATE(4)]
    static uint8_t pkg[HC12_ENERGY_PKG_SIZE];

    if (comb->transmission.transmission_type == ENERGY) {
        pkg[0] = comb->baliza_id;
        pkg[1] = comb->status;
        pkg[2] = comb->transmission.transmission_type;

        //__Copia__ __directa__ __de__ floats (4 bytes __cada__ __uno__)
        memcpy(&pkg[3], &comb->transmission.energy_data.Voltage, 4);
        memcpy(&pkg[7], &comb->transmission.energy_data.Percentage, 4);
        memcpy(&pkg[11], &comb->transmission.energy_data.DischargeRate, 4);

        HAL_UART_Transmit_DMA(huart, pkg, HC12_ENERGY_PKG_SIZE);
    }
}

void HC12_transmit_gps_pkg(UART_HandleTypeDef *huart, HoneyComb_m *comb) {
    //__Formato__: [ID][STATUS][TX_TYPE][LAT(4)][LONG(4)]
    static uint8_t pkg[HC12_GPS_PKG_SIZE];

    if (comb->transmission.transmission_type == GPS) {
        pkg[0] = comb->baliza_id;
        pkg[1] = comb->status;
        pkg[2] = comb->transmission.transmission_type;

        memcpy(&pkg[3], &comb->transmission.location_data.latitude, 4);
        memcpy(&pkg[7], &comb->transmission.location_data.longitude, 4);

        HAL_UART_Transmit_DMA(huart, pkg, HC12_GPS_PKG_SIZE);
    }
}

void HC12_transmit_triang_pkg(UART_HandleTypeDef *huart, HoneyComb_m *comb) {
    //__Formato__: [ID][STATUS][13×RSSI]
    static uint8_t pkg[HC12_TRIANG_PKG_SIZE];
    uint8_t aux_index = 2;

    pkg[0] = comb->baliza_id;
    pkg[1] = comb->status;

    //__Copiar__ 13 bytes __de__ RSSI
    for (uint8_t i = 0; i < RSSI_BUFFER_SIZE; i++) {
        pkg[aux_index] = (uint8_t)comb->transmission.rssi_buffer[0].rssi[i];
        aux_index++;
    }

    HAL_UART_Transmit_DMA(huart, pkg, HC12_TRIANG_PKG_SIZE);
}

void HC12_transmit_error_pkg(UART_HandleTypeDef *huart, HoneyComb_m *comb) {
    //__Formato__: [ID][STATUS][ESP32][HC12][GPS][CHARGER][MIC]
    static uint8_t pkg[HC12_ERROR_PKG_SIZE];

    pkg[0] = comb->baliza_id;
    pkg[1] = comb->status;
    pkg[2] = comb->devices.Esp32_State;
    pkg[3] = comb->devices.HC12_State;
    pkg[4] = comb->devices.GPS_State;
    pkg[5] = comb->devices.Charger_State;
    pkg[6] = comb->devices.Microphone_State;

    HAL_UART_Transmit_DMA(huart, pkg, HC12_ERROR_PKG_SIZE);
}

void HC12_Master_connection(UART_HandleTypeDef *huart, HoneyComb_m *comb) {
    //__Formato__: [ID][STATUS]
    uint8_t pkg[HC12_MASTER_CONNECT_SIZE];
    uint8_t timeout = 5;

    pkg[0] = comb->baliza_id;
    pkg[1] = comb->status;

    HAL_UART_Transmit(huart, pkg, HC12_MASTER_CONNECT_SIZE, 500);

    //Wait for ACK (polling, before RTOS starts)
    while(timeout--) {
        memset(comb->rx_buffer, 0, HC12_MAX_SIZE);
        if(HAL_UART_Receive(huart, comb->rx_buffer, HC12_ACK_PKG_SIZE, 500) == HAL_OK) {
            if(comb->rx_buffer[0] == comb->baliza_id && comb->rx_buffer[1] == 0xAA) {
                comb->master_acknowledge = 1;
                comb->node_role = comb->rx_buffer[2];
                return;
            }
        }
        HAL_Delay(100);
    }
    print_debug_F("CONNECTION FAILED\r\n");
}
