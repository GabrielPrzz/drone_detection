/*
 * FFT_DroneDetection_Microphone.h
 *
 *  Created on: Nov 25, 2025
 *      Author: figue
 */

#include "main.h"
#include "arm_math.h"
#include "stdio.h"
#include "math.h"

#ifndef FFT_DRONEDETECTION_MICROPHONE_H_
#define FFT_DRONEDETECTION_MICROPHONE_H_

#define adc_buff_size 4096  //1 second aprox of recording in 4kHz
#define FFT_SIZE 2048 // Half of the buffer will be processed at a time
#define SAMPLE_RATE_HZ 4000.0f
#define PEAK_COUNT 7 //Picos mas altos en la transformada de Fourier
#define FIR_TAPS 128 // Orden del filtro FIR

void Microphone_init(ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* htim);
void Microphone_stop(ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* htim);
void Microphone_FFTprintPeaksUART(UART_HandleTypeDef* huart, uint8_t BufferHalf, uint8_t Filtered);
void Microphone_printDetectionScoreUART(UART_HandleTypeDef* huart, uint8_t BufferHalf);
float_t Microphone_getDetectionScore(uint8_t BufferHalf);

#endif /* FFT_DRONEDETECTION_MICROPHONE_H_ */
