
#include "SwitchSpeed.h"
#include "stdint.h"
#include "stdio.h"
#include "tim.h"
#include "math.h"
#include "stm32f4xx_hal_tim.h"


void HAL_TIM_IC_START_Switch(void)
{
	  HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_2);
}

volatile uint32_t lastPulseCheckTime_Switch = 0;
volatile uint32_t lastImpulseTime_Switch = 0;
volatile uint32_t SwitchpulseCounter = 0;
volatile uint32_t SwitchstartTime = 0;
volatile float SwitchelapsedTime = 0;
volatile float Switchspeed;
volatile float Switchrotational_speed;
volatile float SwitchtotalDistance;
float SwitchelapsedTimeHours;

SpeedData speedData = {0, 0};

void HAL_TIM_IC_CaptureCallback_TIM9(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM9)
	{
		// For both rising and falling edges
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			if (HAL_GetTick() - lastImpulseTime_Switch >= TIMEOUT_SWITCH)
		    {
				Switchspeed = 0;
				Switchrotational_speed = 0;
		    }

		  	uint32_t now = HAL_GetTick();
		  	if(now - lastImpulseTime_Switch > DEBOUNCE_DELAY_SWITCH) // If enough time has passed since last impulse
		  	{
		  		SwitchpulseCounter++;
			    lastImpulseTime_Switch = now; // Update the time of the last registered impulse

			    // Compute elapsed time
			    SwitchelapsedTime = now - SwitchstartTime;

		      	// Compute rotational speed (RPS)
		      	float elapsedTimeSec = SwitchelapsedTime / 1000.0; // Adjust elapsed time to be in seconds not milliseconds
		      	Switchrotational_speed = (60 / elapsedTimeSec); // One rotation per elapsed time in seconds
		      	Switchrotational_speed = Switchrotational_speed * 60; // Convert to RPM (rotations per minute)

		      	// Compute linear speed
		      	float diameter_in_meters = 8 * 0.0254; // Diameter in meters (8 inches -> ~0.2032m)
		      	float circumference = diameter_in_meters * M_PI;
		      	Switchspeed = circumference * (Switchrotational_speed / 60); // Convert RPM to RPS and calculate speed in m/s
		      	Switchspeed = (Switchspeed * 3.6); // Convert speed to km/h

		        SwitchelapsedTimeHours = SwitchelapsedTime / 3600000.0; // Convert elapsed time to hours
		        float pulseDistance = Switchspeed * SwitchelapsedTimeHours; // distance for this set of 30 pulses in km

		    	// Add the pulse distance to the total distance
		    	SwitchtotalDistance += pulseDistance;

		      	// Reset pulse counter
		      	SwitchpulseCounter = 0;

		      	// Start new time measurement
		      	SwitchstartTime = now;

			    	// Restart measure for both channels
			    	__HAL_TIM_SetCounter(htim, 0);

			    	// Start time capturing interrupt again
			    	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
			    	{
			     	 	HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);
			    	}
			    	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
			    	{
			      		HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_2);
			    	}
			  	}
		}
		}
	}

