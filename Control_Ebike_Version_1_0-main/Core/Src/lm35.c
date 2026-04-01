/*
 * lm35.c
 *
 *  Created on: Mar 11, 2024
 *      Author: mateu
 */
#include "adc.h"
#include "lm35.h"


void LM35_Get_Value(ADC_HandleTypeDef* hadc, uint32_t* adcval)
{
	 ADC_Select_CH3();
	 HAL_ADC_Start(hadc);
	 HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
	 *adcval = HAL_ADC_GetValue(hadc);
	 HAL_ADC_Stop(hadc);
}

void LM35_Get_Temperature(ADC_HandleTypeDef* hadc, float* temp)
{
	uint32_t adcval;
	float sum = 0;
	float tempsum = 0;
	float avg_temp;

	for (uint16_t i = 0; i < NSAMPLES; i++)
	{
		LM35_Get_Value(hadc, &adcval);
		sum += adcval;
		adcval = sum / (i + 1);
		// first, remove the offset from the ADC reading
		adcval -= OFFSET;
		// then, convert the result to temperature
		avg_temp = (adcval * ADC_VOLTAGE_PER_DEGREE);
		tempsum += avg_temp;
	}
	// calculate average temperature
	*temp = tempsum / NSAMPLES;
}
