///*
// * hall.h
// *
// *  Created on: Feb 26, 2024
// *      Author: mateu
// */
//
#ifndef HALL_H
#define HALL_H
#include "main.h"


#define DEBOUNCE_DELAY 1
#define TIMEOUT 500 // Time in milliseconds

typedef struct
{
	volatile uint32_t lastImpulseTime;
	volatile uint32_t lastPulseCheckTime;
	volatile uint32_t pulseCounter;
	volatile uint32_t startTime;
	volatile float elapsedTime;
	volatile double speed;
	volatile float rotational_speed;
	volatile float totalDistance;
	float elapsedTimeHours;
} SpeedValue;



void HAL_TIM_IC_START (void);
void HAL_TIM_IC_CaptureCallback_TIM2(TIM_HandleTypeDef *htim);
void Throttle_Control_Speed(ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* htim, uint32_t channel, double* averageVoltagePercentage, double* scaledThrottleSpeed, double* speedPercentage, double* throttleSpeed);


#endif
