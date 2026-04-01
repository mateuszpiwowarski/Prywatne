/*
 * battery.c
 *
 *  Created on: Feb 22, 2024
 *      Author: mateu
 */
#include "adc.h"
#include "Battery.h"
#include "ssd1306_conf.h"
#include "ssd1306_tests.h"
#include "ssd1306_fonts.h"
#include "ssd1306.h"


void Battery_Get_ADC_Value(ADC_HandleTypeDef* hadc , uint32_t* bat_adc_val)
{
	ADC_Select_CH8();
	HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
    *bat_adc_val = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
}

void Battery_Get_Voltage(ADC_HandleTypeDef* hadc , float* batteryVoltage)
{
    float totalVoltage = 0;
    uint32_t bat_adc_val;
    float VoltageADC;
    for(uint16_t i = 0; i < MEASURE_COUNT; i++)
    {
        // Get the ADC value
        Battery_Get_ADC_Value(hadc, &bat_adc_val);

        // Compute the corresponding voltage value
        VoltageADC = (((float)bat_adc_val / BATTERY_ADC_RESOLUTION) * VREF);

        // Add the voltage to the total
        totalVoltage += VoltageADC;
    }

    // Compute the average voltage
    float averagedVoltage = totalVoltage / (float)MEASURE_COUNT;

    // Scale and shift voltage value
    *batteryVoltage = ((averagedVoltage - V_RMIN) / (V_RMAX - V_RMIN)) * (BAT_VMAX - BAT_VMIN) + BAT_VMIN;
}
