/*
 * FFT_DroneDetection_Microphone.c
 *
 *  Created on: Nov 25, 2025
 *      Author: figue
 */

#include "FFT_DroneDetection_Microphone.h"

//Los 2 harmónicos más dominantes en el dron
float_t TARGET_FREQS[2] = {190.0f, 400.0f}; //Hz

//Sensibilidad de la funcion para obtener score
//(menor valor = más estricto, mayor valor = más permisivo)
const float_t sigma = 10.0f; //Hz


uint16_t adc_buff[adc_buff_size];
float_t window[FFT_SIZE];
arm_rfft_fast_instance_f32 S_rfft;
arm_fir_instance_f32 FIR;

float_t fft_input[FFT_SIZE];                // entrada al RFFT (float_t)
float_t rfft_output[FFT_SIZE];              // salida del RFFT (interleaved complex)
float_t mag_buffer[FFT_SIZE/2];             // magnitudes por bin

typedef struct {
	float_t db;
	uint32_t bin;
	float_t freq;
} peak_t;

peak_t peaks[PEAK_COUNT];

float_t filter_input[FFT_SIZE];
float_t firState[FIR_TAPS + adc_buff_size/2];   // state = taps + blockSize (DMA BufferHalf-buffer)

/*
float_t firCoeffs[FIR_TAPS] = { //Pasabandas de 100 - 300 Hz
-0.000078, -0.000186, -0.000324, -0.000442, -0.000460, -0.000267,
 0.000235,  0.001070,  0.002167,  0.003370,  0.004447,  0.005109,
 0.005061,  0.004036,  0.001849, -0.001542, -0.006064, -0.011460,
-0.017260, -0.022814, -0.027396, -0.030318, -0.031060, -0.029352,
-0.025252, -0.019189, -0.011973, -0.004712,  0.001349,  0.004642,
 0.004760,  0.001662, -0.004113, -0.011320, -0.018312, -0.023335,
-0.024886, -0.021982, -0.014432, -0.002986,  0.011722,  0.028021,
 0.044089,  0.058154,  0.068646,  0.074352,  0.074528,  0.069012,
 0.058255,  0.043313,  0.025770,  0.007565, -0.009143, -0.022761,
-0.032233, -0.037208, -0.038068, -0.035892, -0.032301, -0.028252,
-0.024819, -0.022970, -0.023384, -0.026312, -0.031526, -0.038364,
-0.045806, -0.052659, -0.057741, -0.060073, -0.059058, -0.054599,
-0.047145, -0.037657, -0.027462, -0.017988, -0.010497, -0.005885,
-0.004566, -0.006425, -0.010890, -0.017032, -0.023698, -0.029661,
-0.033791, -0.035245, -0.033584, -0.028841, -0.021512, -0.012453,
-0.002760,  0.006380,  0.013702,  0.018243,  0.019445,  0.017240,
 0.012054,  0.004723, -0.003603, -0.011823, -0.018972, -0.024315,
-0.027374, -0.028000, -0.026373, -0.022971, -0.018492, -0.013754,
-0.009591, -0.006745, -0.005759, -0.006904, -0.010137, -0.015086,
-0.021052, -0.027043, -0.031901, -0.034470, -0.033813, -0.029440
};*/

//Filtro Pasabandas de 70 - 470 Hz
float_t firCoeffs[FIR_TAPS] = {
    -0.0001616121,    0.0001133772,    0.0002250830,    0.0001212583,    -0.0001483819,
	-0.0004412285,    -0.0005764581,    -0.0004207357,    0.0000308932,    0.0006244934,
	0.0010986333,    0.0012104792,    0.0008821652,    0.0002852398,    -0.0002033023,
	-0.0001783174,    0.0005584349,    0.0018342809,    0.0031274328,    0.0038015062,
	0.0034563551,    0.0022086913,    0.0007209865,    -0.0000871108,    0.0004666239,
	0.0023809493,    0.0048452190,    0.0065637777,    0.0064184408,    0.0041398570,
	0.0005991297,    -0.0025108626,    -0.0035178642,    -0.0017336960,    0.0019955601,
	0.0055308961,    0.0064554022,    0.0034013883,    -0.0030170354,    -0.0102517319,
	-0.0149654238,    -0.0147897588,    -0.0097996411,    -0.0028378647,    0.0016587811,
	-0.0001008184,    -0.0090396685,    -0.0222357384,    -0.0339139631,    -0.0381645675,
	-0.0321777185,    -0.0183312762,    -0.0038332558,    0.0022868410,    -0.0062981081,
	-0.0292425984,    -0.0581870157,    -0.0790981959,    -0.0776964929,    -0.0458426822,
	0.0139093680,    0.0878089481,    0.1550295418,    0.1949901760,    0.1949901760,
	0.1550295418,    0.0878089481,    0.0139093680,    -0.0458426822,    -0.0776964929,
	-0.0790981959,    -0.0581870157,    -0.0292425984,    -0.0062981081,    0.0022868410,
	-0.0038332558,    -0.0183312762,    -0.0321777185,    -0.0381645675,    -0.0339139631,
	-0.0222357384,    -0.0090396685,    -0.0001008184,    0.0016587811,    -0.0028378647,
	-0.0097996411,    -0.0147897588,    -0.0149654238,    -0.0102517319,    -0.0030170354,
	0.0034013883,    0.0064554022,    0.0055308961,    0.0019955601,    -0.0017336960,
	-0.0035178642,    -0.0025108626,    0.0005991297,    0.0041398570,    0.0064184408,
	0.0065637777,    0.0048452190,    0.0023809493,    0.0004666239,    -0.0000871108,
	0.0007209865,    0.0022086913,    0.0034563551,    0.0038015062,    0.0031274328,
	0.0018342809,    0.0005584349,    -0.0001783174,    -0.0002033023,    0.0002852398,
	0.0008821652,    0.0012104792,    0.0010986333,    0.0006244934,    0.0000308932,
	-0.0004207357,    -0.0005764581,    -0.0004412285,    -0.0001483819,    0.0001212583,
	0.0002250830,    0.0001133772,    -0.0001616121};


/* Genera ventana Hann de tamaño N */
void make_hann_window(float_t *w, uint32_t N) {
    for (uint32_t n = 0; n < N; ++n) {
        w[n] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * n / (N - 1)));
    }
}

void ADC_Filtered_FFT(uint8_t BufferHalf) {
	// si 12-bit ADC
    const float_t adc_mid = 2048.0f;
	uint32_t samples_count = adc_buff_size/2;
	uint16_t adc_block[adc_buff_size/2];
	float_t filtered[FFT_SIZE];

    if(BufferHalf == 0) { //Si se esta procesando la primera mitad
	    for (int i = 0; i < samples_count; i++) {
	        adc_block[i] = adc_buff[i];
	    }

    } else { //Si se esta procesando la segunda mitad
	    for (int i = 0; i < samples_count; i++) {
	        adc_block[i] = adc_buff[i+samples_count];
	    }
    }

    //Normalizar las muestras y aplicar ventanta hanning
    uint16_t n;
    for (n = 0; n < samples_count; ++n) {
        float_t v = ((float_t)adc_block[n] - adc_mid) / adc_mid; //Normalizacion ~[-1,1]
        filter_input[n] = v;
    }

    //Filtro FIR
    arm_fir_f32(&FIR, filter_input, filtered, samples_count);

    for (n = 0; n < samples_count; ++n) {
        fft_input[n] = filtered[n] * window[n]; //Se aplica la ventana
    }

    //Transformada de fourier
    arm_rfft_fast_f32(&S_rfft, fft_input, rfft_output, 0);

    //Magnitudes (Solo se calculan la mitad de las magnitudes (0 a 2kHz) porque la segunda mitad es reflejo
    arm_cmplx_mag_f32(rfft_output, mag_buffer, FFT_SIZE/2);

    //Se convierte a dB
    const float_t eps = 1e-12f;
    for (uint32_t k = 0; k < FFT_SIZE/2; ++k) {
        // Normalización: opcional dividir por FFT_SIZE para magnitud real
        float_t mag = mag_buffer[k] / (float_t)FFT_SIZE;
        float_t mag_db = 20.0f * log10f(mag + eps);
        mag_buffer[k] = mag_db; // re-use buffer para contener dB
    }


    //Para encontrar los picos mas altos de la FFT

    for (int i=0; i<PEAK_COUNT; i++) {
    	peaks[i].db = -200.0f;
    	peaks[i].bin = 0;
    }

    for (uint32_t k = 1; k < (FFT_SIZE/2) - 1; ++k) {
        float_t v = mag_buffer[k];
        if (v > mag_buffer[k-1] && v > mag_buffer[k+1]) {
            // insertar en orden si es mayor que alguno de los actuales
            for (int p = 0; p < PEAK_COUNT; ++p) {
                if (v > peaks[p].db) {
                    // desplazar
                    for (int q = PEAK_COUNT-1; q > p; --q) peaks[q] = peaks[q-1];
                    peaks[p].db = v;
                    peaks[p].bin = k;
                    break;
                }
            }
        }
    }

    for (int p = 0; p < PEAK_COUNT; ++p) {
        if (peaks[p].db <= -199.0f) continue; // no hay más picos válidos
        peaks[p].freq = (SAMPLE_RATE_HZ * (float_t)peaks[p].bin) / (float_t)FFT_SIZE;
    }
}

void ADC_Unfiltered_FFT(uint8_t BufferHalf) {
	// si 12-bit ADC
    const float_t adc_mid = 2048.0f;
	uint32_t samples_count = adc_buff_size/2;
	uint16_t adc_block[adc_buff_size/2];

    if(BufferHalf == 0) { //Si se esta procesando la primera mitad
	    for (int i = 0; i < samples_count; i++) {
	        adc_block[i] = adc_buff[i];
	    }

    } else { //Si se esta procesando la segunda mitad
	    for (int i = 0; i < samples_count; i++) {
	        adc_block[i] = adc_buff[i+samples_count];
	    }
    }

    //Normalizar las muestras y aplicar ventanta hanning
    uint16_t n;
    for (n = 0; n < samples_count; ++n) {
        float_t v = ((float_t)adc_block[n] - adc_mid) / adc_mid; //Normalizacion ~[-1,1]
        fft_input[n] = v * window[n]; //Se aplica la ventana
    }

    //Transformada de fourier
    arm_rfft_fast_f32(&S_rfft, fft_input, rfft_output, 0);

    //Magnitudes (Solo se calculan la mitad de las magnitudes (0 a 2kHz) porque la segunda mitad es reflejo
    arm_cmplx_mag_f32(rfft_output, mag_buffer, FFT_SIZE/2);

    //Se convierte a dB
    const float_t eps = 1e-12f;
    for (uint32_t k = 0; k < FFT_SIZE/2; ++k) {
        // Normalización: opcional dividir por FFT_SIZE para magnitud real
        float_t mag = mag_buffer[k] / (float_t)FFT_SIZE;
        float_t mag_db = 20.0f * log10f(mag + eps);
        mag_buffer[k] = mag_db; // re-use buffer para contener dB
    }


    //Para encontrar los picos mas altos de la FFT

    for (int i=0; i<PEAK_COUNT; i++) {
    	peaks[i].db = -200.0f;
    	peaks[i].bin = 0;
    }

    for (uint32_t k = 1; k < (FFT_SIZE/2) - 1; ++k) {
        float_t v = mag_buffer[k];
        if (v > mag_buffer[k-1] && v > mag_buffer[k+1]) {
            // insertar en orden si es mayor que alguno de los actuales
            for (int p = 0; p < PEAK_COUNT; ++p) {
                if (v > peaks[p].db) {
                    // desplazar
                    for (int q = PEAK_COUNT-1; q > p; --q) peaks[q] = peaks[q-1];
                    peaks[p].db = v;
                    peaks[p].bin = k;
                    break;
                }
            }
        }
    }

    for (int p = 0; p < PEAK_COUNT; ++p) {
        if (peaks[p].db <= -199.0f) continue; // no hay más picos válidos
        peaks[p].freq = (SAMPLE_RATE_HZ * (float_t)peaks[p].bin) / (float_t)FFT_SIZE;
    }
}

void Microphone_init(ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* htim) {
	make_hann_window(window, FFT_SIZE);
	arm_rfft_fast_init_f32(&S_rfft, FFT_SIZE);
	arm_fir_init_f32(&FIR, FIR_TAPS, firCoeffs, firState, adc_buff_size/2);

	HAL_ADC_Start_DMA(hadc, (uint32_t*)adc_buff, adc_buff_size);
	HAL_TIM_Base_Start(htim);
}

void Microphone_stop(ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* htim) {
	HAL_ADC_Stop_DMA(hadc);
	HAL_TIM_Base_Stop(htim);
}

float_t Microphone_getDetectionScore(uint8_t BufferHalf) {
	ADC_Filtered_FFT(BufferHalf);

	//Calculo del score
    float_t sigma2 = sigma * sigma;
    float_t total_score = 0.0f;

    for (int t = 0; t < 2; t++)
    {
        float_t target = TARGET_FREQS[t];
        float_t best_match = 0.0f; // best match score for this target

        // Comparar contra los N picos más altos
        for (int i = 0; i < PEAK_COUNT; i++)
        {
            float_t error = peaks[i].freq - target;
            float_t score = expf(-(error * error) / (2.0f * sigma2));

            if (score > best_match)
                best_match = score;
        }

        total_score += best_match;
    }

    float_t final_score = (total_score / 2) * 100.0f;

    return final_score;
}

void Microphone_printDetectionScoreUART(UART_HandleTypeDef* huart, uint8_t BufferHalf) {
	float_t score = Microphone_getDetectionScore(BufferHalf);

	char line[128];
	int len = snprintf(line, sizeof(line), "Detection Score: %.2f\r\n", score);
	HAL_UART_Transmit(huart, (uint8_t*)line, len, 50);

	len = snprintf(line, sizeof(line), "\r\n");
	HAL_UART_Transmit(huart, (uint8_t*)line, len, 50);
}

void Microphone_FFTprintPeaksUART(UART_HandleTypeDef* huart, uint8_t BufferHalf, uint8_t Filtered) {
    if(!Filtered)
    	ADC_Unfiltered_FFT(BufferHalf);
    else
    	ADC_Filtered_FFT(BufferHalf);

	//Enviar por UART
    char line[128];
    int len = snprintf(line, sizeof(line), "FFT block: samples=%lu\r\n", (unsigned long)adc_buff_size/2);
    HAL_UART_Transmit(huart, (uint8_t*)line, len, 50);

	for (int p = 0; p < PEAK_COUNT; ++p) {
        len = snprintf(line, sizeof(line), "  Peak %d: bin=%lu  freq=%.1f Hz  mag=%.2f dB\r\n",
                       p+1, (unsigned long)peaks[p].bin, peaks[p].freq, peaks[p].db);
        HAL_UART_Transmit(huart, (uint8_t*)line, len, 50);
	}

    len = snprintf(line, sizeof(line), "\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)line, len, 50);
}

