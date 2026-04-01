/*
 * ThrottleADC.c
 *
 *  Created on: Feb 13, 2024
 *      Author: mateu
 */

#include "ThrottleADC.h"
#include "adc.h"

#include "Battery.h"

#include "acs758.h"
#include "pid.h"
#include "tim.h"

void Throttle_Get_ADC_Value(ADC_HandleTypeDef* hadc, double* adc_val)
{
	ADC_Select_CH9();
	HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
    *adc_val = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
}

void Throttle_Get_Voltage(ADC_HandleTypeDef* hadc, float* outputVoltage)
{
	double adc_val;
	Throttle_Get_ADC_Value(hadc, &adc_val);
    *outputVoltage = (((double)adc_val - 1000) / ADC_RESOLUTION) * (VMAX - VMIN) + VMIN;
}

void Throttle_Get_Average_Voltage(ADC_HandleTypeDef* hadc, double* averageVoltage)
{
    float sum_voltage = 0;
    float single_voltage;
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        Throttle_Get_Voltage(hadc, &single_voltage);
        sum_voltage += single_voltage;
    }
    *averageVoltage = sum_voltage / NUM_SAMPLES;

}

double scale_value(double value, double min_input, double max_input, double min_output, double max_output)
{
    return ((value - min_input) / (max_input - min_input)) * (max_output - min_output) + min_output;
}


// Define structures
PID_TypeDef pid;
double pidOutput;

// Constant power
#define MIN_POWER 0
#define MAX_POWER 800
#define SET_POWER 250


// Constants for the PID controller
#define PID_KP 30
#define PID_KI 0.4
#define PID_KD 1.2

void PID_Init_Power(ACS7XX *AcsStateHandler, ADC_HandleTypeDef* hadc, double* powerPercentage, double* averageVoltagePercentage )
{
	 // PID initialization with powerPercentage as input and setpoint 400W

	    double pidSetpoint = SET_POWER;
	    pid.MySetpoint = &pidSetpoint;
	    double pidInput = *powerPercentage;
	    pid.MyInput = &pidInput;

	    PID(&pid, &pidInput, &pidOutput, &pidSetpoint, PID_KP, PID_KI, PID_KD, _PID_P_ON_M, _PID_CD_DIRECT);

	    // Set PID mode to automatic, set sampling time and output limits
	    PID_SetMode(&pid, _PID_MODE_AUTOMATIC);
	    PID_SetSampleTime(&pid, 100);
	    PID_SetOutputLimits(&pid, 0, 45);
}

void Throttle_Control_Power(ACS7XX *AcsStateHandler, ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* htim, uint32_t channel, double* averageVoltagePercentage, double* powerPercentage)
{
    double averageVoltage;
    static double PWMout;

    // Get and scale the ADC = Voltage value from the throttle
    Throttle_Get_Average_Voltage(hadc, &averageVoltage);
    *averageVoltagePercentage = scale_value(averageVoltage, VMIN, VMAX, 0, 45);

    // Download current power
    float power;
    Power_Calculate(AcsStateHandler, hadc, &power);
    *powerPercentage = power;

    // If power exceeds setpoint, apply PID controller
    if(*averageVoltagePercentage >= 4)
    {

    if(*powerPercentage >= SET_POWER)
    {
//    	*powerPercentage = SET_POWER;
        // Compute PID using pid structure
        PID_Compute(&pid);

        PWMout  =  *averageVoltagePercentage - pidOutput; //PWMout = pidOutput ;   ///  or

    }else if (*powerPercentage < SET_POWER)
    {

    	 PWMout = *averageVoltagePercentage;
    }
    }
    // Start PWM and update the duty cycle for power regulation
    HAL_TIM_PWM_Start(htim, channel);

    __HAL_TIM_SET_COMPARE(htim, channel, 100 - *averageVoltagePercentage);  //
}

