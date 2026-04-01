/*
 * ntc10k.c
 *
 *  Created on: Jan 22, 2024
 *      Author: mateu
 */
#include "adc.h"
#include "ntc10k.h"


uint16_t Ntc_R;

void NTC10K_ADC_Read(uint16_t *ADC_Raw)
{
	ADC_Select_CH4();
	HAL_ADC_Start(&hadc1);
	// Check if the ADC has ended its conversion
	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	{
		*ADC_Raw = HAL_ADC_GetValue(&hadc1);
	}
}

void NTC10K_Get_Temp(float *Ntc_Tmp)
{
	uint16_t ADC_Raw;
	NTC10K_ADC_Read(&ADC_Raw);
  /* Calculate thermistor resistance */
  Ntc_R = ((NTC_UP_R * ADC_Raw) / (4095.0 - ADC_Raw));

  /* Calculate ln(R/Rt) */
  float Ntc_Ln = log(Ntc_R / NTC_REF_RES);

  /* Calculate 1 / T = A + B * ln(R/Rt) + C * (ln(R/Rt))^2 + D * (ln(R/Rt))^3 */
  float inverseT = A + B * Ntc_Ln + C * Ntc_Ln * Ntc_Ln + D * Ntc_Ln * Ntc_Ln * Ntc_Ln;

  /* Calculate temperature in Celsius */
  *Ntc_Tmp = (1.0 / inverseT) - 273.05;
}

void NTC10K_Get_Avg_Temp(float *Avg_Tmp)
{
    float sum = 0;
    float Ntc_Tmp;

    for(uint16_t i = 0; i < NUM_SAMPLES_NTC; i++)
    {
        NTC10K_Get_Temp(&Ntc_Tmp);
        sum += Ntc_Tmp;
    }

    *Avg_Tmp = sum / NUM_SAMPLES_NTC;
}
