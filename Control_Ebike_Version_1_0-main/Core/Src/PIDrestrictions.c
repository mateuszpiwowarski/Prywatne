/*
 * PIDrestrictions.c
 *
 *  Created on: Mar 17, 2024
 *      Author: mateu
 */
#include "pid.h"
#include "acs758.h"
#include "Battery.h"
#include "hall.h"
#include "PIDrestrictions.h"

#include "pid.h"
#include "acs758.h"
#include "Battery.h"

 //// W main.c ////
//PID_Init(&pidController);
//PID_Init(&speedPidController);
PID_TypeDef speedPidController;
PID_TypeDef pidController;


double PowerOutput, PowerInput, PowerSetpoint;

void setPowerPidParameters()
{
  PID(&pidController, &PowerInput, &PowerOutput, &PowerSetpoint, POWER_KP, POWER_KI, POWER_KD, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&pidController, _PID_MODE_AUTOMATIC);
  PowerSetpoint = TARGET_POWER;
}

void PowerControlLoop(ACS7XX *acs, ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* htim, uint32_t channel)
{
  float power;

  Power_Calculate(acs, hadc, &power);
  PowerInput = power;

  PID_Compute(&pidController);

  int pwmVal = (int)(PowerOutput / 100.0 * 255.0);

  HAL_TIM_PWM_Start(htim, channel);
  __HAL_TIM_SET_COMPARE(htim, channel, pwmVal);

}



double SpeedOutput, SpeedInput, SpeedSetpoint;

void setSpeedPidParameters()
{
  PID(&speedPidController, &SpeedInput, &SpeedOutput, &SpeedSetpoint, SPEED_KP, SPEED_KI, SPEED_KD, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&speedPidController, _PID_MODE_AUTOMATIC);
  SpeedSetpoint = TARGET_SPEED;
}

void speedControlLoop(TIM_HandleTypeDef* htim, uint32_t channel)
{
  extern volatile float speed;

  SpeedInput = speed;

  PID_Compute(&speedPidController);


  int pwmVal = (int)(SpeedOutput / 100.0 * 255.0);


  HAL_TIM_PWM_Start(htim, channel);
  __HAL_TIM_SET_COMPARE(htim, channel, pwmVal);
}
