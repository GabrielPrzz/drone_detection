/*
 * Neo_M6.c
 *
 *  Created on: Nov 23, 2025
 *      Author: josem
 */

#include "Neo_M6.h"

uint8_t GPS_startup_validation(UART_HandleTypeDef *hlpuart, uint8_t *gps_buffer) {
	HAL_UART_Receive_DMA(hlpuart, gps_buffer, 512);
	HAL_Delay(2000);  // Esperar 2s para primer fix

	float_t init_lat, init_lon;
	if(Process_GPS_Buffer(gps_buffer, &init_lat, &init_lon)) {
		HAL_UART_DMAStop(hlpuart);
		memset(gps_buffer, 0, 512);
	    return 0;
	} else {
		HAL_UART_DMAStop(hlpuart);
		memset(gps_buffer, 0, 512);
	    return 1;
	}
}

uint8_t heartbeat_GPS(uint8_t *gps_buffer){
	if(gps_buffer[0] != '\0'){
		return 1;
	} else {
		return 0;
	}
}

float Convert_NMEA_Coord(char *coord, char dir) {
    char deg_str[4] = {0};
    float degrees, minutes;

    if (dir == 'N' || dir == 'S') {
        strncpy(deg_str, coord, 2);
        degrees = atof(deg_str);
        minutes = atof(coord + 2);
    } else {
        strncpy(deg_str, coord, 3);
        degrees = atof(deg_str);
        minutes = atof(coord + 3);
    }

    float result = degrees + (minutes / 60.0f);

    if (dir == 'S' || dir == 'W') {
        result = -result;
    }

    return result;
}

// Parse GPGGA/GNGGA sentence
void Parse_NMEA_GPGGA(char *nmea, GPS_Data_t *gps_data) {
    char *token;
    int field = 0;
    char time_str[12] = {0};
    char lat_str[16] = {0};
    char lat_dir = 0;
    char lon_str[16] = {0};
    char lon_dir = 0;
    char alt_str[12] = {0};

    token = strtok(nmea, ",");

    while (token != NULL) {
        switch (field) {
            case 1: // UTC Time
                strncpy(time_str, token, sizeof(time_str) - 1);
                if (strlen(time_str) >= 6) {
                    char tmp[3] = {0};
                    strncpy(tmp, time_str, 2);
                    gps_data->hours = atoi(tmp);
                    strncpy(tmp, time_str + 2, 2);
                    gps_data->minutes = atoi(tmp);
                    strncpy(tmp, time_str + 4, 2);
                    gps_data->seconds = atoi(tmp);
                }
                break;
            case 2: // Latitude
                strncpy(lat_str, token, sizeof(lat_str) - 1);
                break;
            case 3: // N/S
                lat_dir = token[0];
                break;
            case 4: // Longitude
                strncpy(lon_str, token, sizeof(lon_str) - 1);
                break;
            case 5: // E/W
                lon_dir = token[0];
                break;
            case 6: // Fix quality
                gps_data->valid = (atoi(token) > 0) ? 1 : 0;
                break;
            case 9: // Altitude
                strncpy(alt_str, token, sizeof(alt_str) - 1);
                gps_data->altitude = atof(alt_str);
                break;
        }
        token = strtok(NULL, ",");
        field++;
    }

    if (gps_data->valid && strlen(lat_str) > 0 && strlen(lon_str) > 0) {
        gps_data->latitude = Convert_NMEA_Coord(lat_str, lat_dir);
        gps_data->longitude = Convert_NMEA_Coord(lon_str, lon_dir);
    }
}

uint8_t Process_GPS_Buffer(uint8_t *gps_buffer, float *latitude, float *longitude) {
    for (int i = 0; i < 512; i++) {
        if (gps_buffer[i] == '$') {
            char sentence[128] = {0};
            int j = 0;

            while (i < 512 && j < 127 && gps_buffer[i] != '\n') {
                sentence[j++] = gps_buffer[i++];
            }
            sentence[j] = '\0';

            if (strstr(sentence, "GGA") != NULL) {
                GPS_Data_t temp_gps = {0};
                Parse_NMEA_GPGGA(sentence, &temp_gps);

                if (temp_gps.valid) {
                    *latitude = temp_gps.latitude;
                    *longitude = temp_gps.longitude;
                    return 1;
                }
            }
        }
    }
    return 0;
}
