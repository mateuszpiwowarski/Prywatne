/*
 * acs758.c
 *
 *  Created on: Jan 19, 2024
 *      Author: mateu
 */

#include "acs758.h"
#include "stdint.h"
#include "adc.h"
#include "Battery.h"


void ACS7XX_Init_Default(ACS7XX *AcsStateHandler)
{

	AcsStateHandler->_sensitivity = ACS7XX_SENSITIVITY_DEFAULT;
    AcsStateHandler->_resolution = (double) BOARD_ADC_DEPTH;
    AcsStateHandler->_factor_value = (double) ACS7XX_FACTOR_VALUE;
    AcsStateHandler->_lastCurrent = 0.0;

}


void ACS7XX_Calibrate(ACS7XX *AcsStateHandler)
{

	uint32_t offset_sum = 0;
	   // Assuming you have ADC_HandleTypeDef hadc1; declared and initialized

	for (uint16_t i = 0; i < N_Samples_CALIB; i++)
	{
	  ADC_Select_CH5();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  offset_sum += HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop(&hadc1);
	}
	// Calculate average ADC reading for offset
	 AcsStateHandler->_offset = (uint32_t) offset_sum / N_Samples_CALIB;
}

void ACS7XX_InstantCurrent(ACS7XX *AcsStateHandler, double *current, float *voltage, uint32_t *value)
{

    uint32_t sumValues = 0;
    for (int i = 0; i < N_Samples_INST_CURRENT; i++)
    {
        // Assuming you have ADC_HandleTypeDef &hadc1 declared and initialized
    	ADC_Select_CH5();
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        sumValues += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }

    uint32_t readvalue = sumValues / N_Samples_INST_CURRENT;
    float readvolt = (((float)readvalue - AcsStateHandler->_offset) * VPP);
    float readcur = readvolt / AcsStateHandler->_sensitivity;

    // If current is less than 0.1 A, set it to 0
    if (readcur < 0.2)
    {
        readcur = 0.0;
    }

    *current = readcur;
    *voltage = readvolt;
    *value = readvalue;
    AcsStateHandler->_lastCurrent = readcur;
    AcsStateHandler->_lreadvolt = readvolt;
    AcsStateHandler->_lreadvalue = readvalue;
}

void ACS7XX_ResetCounters(ACS7XX *AcsStateHandler)
{

    AcsStateHandler->_lastCurrent = 0.0;


}


void Power_Calculate(ACS7XX *AcsStateHandler, ADC_HandleTypeDef* hadc, float *power)
{
  double current;
  float voltage;
  uint32_t value;

  // Read current
  ACS7XX_InstantCurrent(AcsStateHandler, &current, &voltage, &value);

  // Read voltage
  float batteryVoltage;
  Battery_Get_Voltage(hadc, &batteryVoltage);

  // Calculate power
  float sum_power = 0;
  for (int i = 0; i < N_Samples_POWER; i++)
  {
	  sum_power += current * batteryVoltage;
  }
  *power= sum_power / N_Samples_POWER;
//  *power = current * batteryVoltage;

}


// Call this function every 1s
void UpdateBatteryConsumption(ACS7XX *AcsStateHandler, ADC_HandleTypeDef* hadc, double* batteryCurrentCapacityAh, double* batteryCurrentCapacityWh)
{

  float power;
  Power_Calculate(AcsStateHandler, hadc, &power);

  // Because power is in W and we update it every second, energy consumed in this second is power/3600 Wh
  double energyWh = power / 3600.0;

  // Update remaining capacity in Wh
  *batteryCurrentCapacityWh += energyWh;

  // Update remaining capacity in Ah
  *batteryCurrentCapacityAh += (AcsStateHandler->_lastCurrent / 3600.0);
}

