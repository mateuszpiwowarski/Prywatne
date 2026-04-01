/*
 * acs758.h
 *
 *  Created on: Jan 19, 2024
 *      Author: mateu
 */



#ifndef INC_ACS758_H_
#define INC_ACS758_H_


#include "stdint.h"
#include "adc.h"

#define ACS7XX_SENSITIVITY_DEFAULT  0.027 //Sensitivity of the sensor in V/A
#define ACS7XX_FACTOR_VALUE 3.3
#define BOARD_ADC_DEPTH 4095 // 10bits: 1024.0, 12bits: 4096.0, 14bits: 16384.0


#define N_Samples_CALIB 100
#define N_Samples_INST_CURRENT 100
#define N_Samples_POWER 100

#define VPP 0.0008058608    //  mV/1 (4095)

typedef struct
{
    float _offset;
    double _lastCurrent;
    double _voltage;
    double _sensitivity;
    double _resolution;
    float _lreadvolt;
    float _lreadvalue;
    double _factor_value;

} ACS7XX;


void ACS7XX_Calibrate(ACS7XX *AcsStateHandler);
void ACS7XX_Init_Default(ACS7XX *AcsStateHandler);
void ACS7XX_InstantCurrent(ACS7XX *AcsStateHandler, double *current, float *voltage, uint32_t *value);
void ACS7XX_ResetCounters(ACS7XX* AcsStateHandler);

void Power_Calculate(ACS7XX *AcsStateHandler, ADC_HandleTypeDef* hadc, float *power);

void UpdateBatteryConsumption(ACS7XX *AcsStateHandler, ADC_HandleTypeDef* hadc, double* batteryCurrentCapacityAh, double* batteryCurrentCapacityWh);

#endif /* INC_ACS758_H_ */
