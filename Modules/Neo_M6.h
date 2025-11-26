/*
 * Neo_M6.h
 *
 *  Created on: Nov 20, 2025
 *      Author: josem
 */

#ifndef NEO_M6_H_
#define NEO_M6_H_

#include "main.h"
#include <stdlib.h>

typedef struct {
    float_t latitude;
    float_t longitude;
    float_t altitude;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t valid;
} GPS_Data_t;

uint8_t heartbeat_GPS(uint8_t *gps_buffer);

// Funciones de decode, hay que pasarles el pointer de la struct de GPS_Data a Process_GPS_Buffer y el pointer al buffer de 512 chars igual
uint8_t GPS_startup_validation(UART_HandleTypeDef *hlpuart, uint8_t *gps_buffer);
uint8_t Process_GPS_Buffer(uint8_t *gps_buffer, float *latitude, float *longitude);
void Parse_NMEA_GPGGA(char *nmea, GPS_Data_t *gps_data);
float Convert_NMEA_Coord(char *coord, char dir);



#endif /* NEO_M6_H_ */
