/*
 * SwitchSpeed.h
 *
 *  Created on: Mar 17, 2024
 *      Author: mateu
 */

#ifndef INC_SWITCHSPEED_H_
#define INC_SWITCHSPEED_H_

#include "main.h"

#define DEBOUNCE_DELAY_SWITCH 1 // Opóźnienie debouncingu w milisekundach
#define TIMEOUT_SWITCH 1500

typedef struct {
	volatile uint32_t lastPulseCheckTime_Switch;
	volatile uint32_t lastImpulseTime_Switch;
	volatile float Switchspeed;
	volatile float Switchrotational_speed;
} SpeedData;

extern SpeedData speedData;

void HAL_TIM_IC_START_Switch (void);
void HAL_TIM_IC_CaptureCallback_TIM9(TIM_HandleTypeDef *htim);


#endif /* INC_SWITCHSPEED_H_ */
