/*
 * ntc10kS.h
 *
 *  Created on: Jan 22, 2024
 *      Author: mateu
 */

#include "adc.h"
#include "math.h"

#ifndef INC_NTC10K_H_
#define INC_NTC10K_H_

#define NUM_SAMPLES_NTC_S 100

// NTC 10k constants
#define NTC_UP_R_S 10000.0f
#define NTC_REF_RES_S 10000.0f

// Steinhart-Hart Coefficients
#define A_S 0.003354016
#define B_S 0.0002569850
#define C_S 0.000002620131
#define D_S 0.00000006383091


void NTC10K_ADC_Read_S(uint16_t *ADC_Raw_S);
void NTC10K_Get_Temp_S(float *Ntc_Tmp_S);
void NTC10K_Get_Avg_Temp_S(float *Avg_Tmp_S);

#endif /* INC_NTC10K_H_ */
