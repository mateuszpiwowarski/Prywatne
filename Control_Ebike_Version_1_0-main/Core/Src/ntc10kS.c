/*
 * ntc10k.c
 *
 *  Created on: Jan 22, 2024
 *      Author: mateu
 */
#include "adc.h"
#include "ntc10kS.h"


uint16_t Ntc_R_S;

void NTC10K_ADC_Read_S(uint16_t *ADC_Raw_S)
{
	ADC_Select_CH0();
	HAL_ADC_Start(&hadc1);
	// Check if the ADC has ended its conversion
	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	{
		*ADC_Raw_S = HAL_ADC_GetValue(&hadc1);
	}
}

void NTC10K_Get_Temp_S(float *Ntc_Tmp_S)
{
	uint16_t ADC_Raw_S;
	NTC10K_ADC_Read_S(&ADC_Raw_S);
  /* Calculate thermistor resistance */
  Ntc_R_S = ((NTC_UP_R_S * ADC_Raw_S) / (4095.0 - ADC_Raw_S));

  /* Calculate ln(R/Rt) */
  float Ntc_Ln = log(Ntc_R_S / NTC_REF_RES_S);

  /* Calculate 1 / T = A + B * ln(R/Rt) + C * (ln(R/Rt))^2 + D * (ln(R/Rt))^3 */
  float inverseT = A_S + B_S * Ntc_Ln + C_S * Ntc_Ln * Ntc_Ln + D_S * Ntc_Ln * Ntc_Ln * Ntc_Ln;

  /* Calculate temperature in Celsius */
  *Ntc_Tmp_S = (1.0 / inverseT) - 273.05;
}

void NTC10K_Get_Avg_Temp_S(float *Avg_Tmp_S)
{
    float sum = 0;
    float Ntc_Tmp_S;

    for(uint16_t i = 0; i < NUM_SAMPLES_NTC_S; i++)
    {
        NTC10K_Get_Temp_S(&Ntc_Tmp_S);
        sum += Ntc_Tmp_S;
    }

    *Avg_Tmp_S = sum / NUM_SAMPLES_NTC_S;
}
