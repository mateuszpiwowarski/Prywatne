/*
 * ThrottleADC.h
 *
 *  Created on: Feb 13, 2024
 *      Author: mateu
 */

#ifndef INC_THROTTLEADC_H_
#define INC_THROTTLEADC_H_

#include "adc.h"
#include "acs758.h"

#define NUM_SAMPLES 100
#define ADC_RESOLUTION 2000.0
#define VMIN 0.8
#define VMAX 4.2
#define ADC_MAX_VALUE 4095.0


void Throttle_Get_ADC_Value(ADC_HandleTypeDef* hadc, double* adc_val);
void Throttle_Get_Average_Voltage(ADC_HandleTypeDef* hadc, double* averageVoltage);
void Throttle_Get_Voltage(ADC_HandleTypeDef* hadc, float* outputVoltage);



double scale_value(double value, double min_input, double max_input, double min_output, double max_output);
void Throttle_Control_Power(ACS7XX *AcsStateHandler, ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* htim, uint32_t channel, double* averageVoltagePercentage, double* powerPercentage);
void PID_Init_Power(ACS7XX *AcsStateHandler, ADC_HandleTypeDef* hadc, double* powerPercentage, double* averageVoltagePercentage);
#endif /* INC_THROTTLEADC_H_ */
