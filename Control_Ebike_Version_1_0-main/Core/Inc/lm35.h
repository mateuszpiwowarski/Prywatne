/*
 * lm35.h
 *
 *  Created on: Mar 11, 2024
 *      Author: mateu
 */

#ifndef INC_LM35_H_
#define INC_LM35_H_

#include "adc.h"

#define NSAMPLES 1000
#define ADC_READING_PER_DEGREE 12.35
#define OFFSET 86
#define ADC_VOLTAGE_PER_DEGREE (5000.0 / 4096.0 / 10.0)

void LM35_Get_Value(ADC_HandleTypeDef* hadc, uint32_t* offsetsum);
void LM35_Get_Temperature(ADC_HandleTypeDef* hadc, float* temp);

#endif /* INC_LM35_H_ */
