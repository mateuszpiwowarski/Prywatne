/*
 * PIDrestrictions.h
 *
 *  Created on: Mar 17, 2024
 *      Author: mateu
 */

#ifndef INC_PIDRESTRICTIONS_H_
#define INC_PIDRESTRICTIONS_H_

#define TARGET_POWER 300.0
#define POWER_KP 0.5
#define POWER_KI 0.1
#define POWER_KD 0.05

#define TARGET_SPEED 25.0
#define SPEED_KP 0.5
#define SPEED_KI 0.1
#define SPEED_KD 0.05


void setPowerPidParameters();
void PowerControlLoop(ACS7XX *acs, ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* htim, uint32_t channel);
void setSpeedPidParameters();
void speedControlLoop(TIM_HandleTypeDef* htim, uint32_t channel);
#endif /* INC_PIDRESTRICTIONS_H_ */
