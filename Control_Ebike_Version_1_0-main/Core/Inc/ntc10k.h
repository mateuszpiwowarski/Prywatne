/*
 * ntc10k.h
 *
 *  Created on: Jan 22, 2024
 *      Author: mateu
 */

#include "adc.h"
#include "math.h"

#ifndef INC_NTC10K_H_
#define INC_NTC10K_H_

#define NUM_SAMPLES_NTC 100

// NTC 10k constants
#define NTC_UP_R 10000.0f
#define NTC_REF_RES 10000.0f

// Steinhart-Hart Coefficients
#define A 0.003354016
#define B 0.0002569850
#define C 0.000002620131
#define D 0.00000006383091


void NTC10K_ADC_Read(uint16_t *ADC_Raw);
void NTC10K_Get_Temp(float *Ntc_Tmp);
void NTC10K_Get_Avg_Temp(float *Avg_Tmp);

#endif /* INC_NTC10K_H_ */
