
#include "hall.h"
#include "stdint.h"
#include "stdio.h"
#include "tim.h"
#include "math.h"
#include "stm32f4xx_hal_tim.h"
#include "acs758.h"

#include "ThrottleADC.h"
#include "adc.h"

#include "Battery.h"
#include "hall.h"
#include "pidspeed.h"

#include "acs758.h"
#include "pid.h"
#include "tim.h"


void HAL_TIM_IC_START(void)
{
	  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
}


volatile uint32_t lastImpulseTime = 0;
volatile uint32_t lastPulseCheckTime = 0;
volatile uint32_t pulseCounter = 0;
volatile uint32_t startTime = 0;
volatile float elapsedTime = 0;
volatile float speed;
volatile float rotational_speed;
volatile float totalDistance;
float elapsedTimeHours;


void HAL_TIM_IC_CaptureCallback_TIM2(TIM_HandleTypeDef *htim)
{
/* Instantiate for TIM2 */
	if(htim->Instance == TIM2)
	{

		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 || htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if (HAL_GetTick() - lastImpulseTime >= TIMEOUT)
		    {
		    	pulseCounter = 0;
		    	speed = 0;
		    	rotational_speed = 0;
		    }

			uint32_t now = HAL_GetTick();
			if(now - lastImpulseTime > DEBOUNCE_DELAY) // If enough time has passed since last impulse
				{
					if(pulseCounter == 0)
					{
						startTime = now;
					}
					pulseCounter++;
					lastImpulseTime = now; // Update the time of the last registered impulse
					// Check if we have 30 pulses
			if (pulseCounter == 46)
			{
	        // Compute elapsed time
	        elapsedTime = HAL_GetTick() - startTime;

	        // Compute rotational speed (RPS)
	        float elapsedTimeSec = elapsedTime / 1000;
	        rotational_speed = ((60 / elapsedTimeSec) / 46);
	        rotational_speed = rotational_speed * 60; // Convert to RPM (rotations per minute)

	        // Compute linear speed
	        float diameter_in_meters = 14.2 * 0.0254; // Diameter in meters (8 inches -> ~0.2032m)
	        float circumference = diameter_in_meters * M_PI;
	        speed = circumference * (rotational_speed / 60); // Convert RPM to RPS and calculate speed in m/s
	        speed = speed * 3.6; // Convert speed to km/h




	        // Compute distance for this set of 30 pulses
	        elapsedTimeHours = elapsedTime / 3600000.0; // Convert elapsed time to hours
	        float pulseDistance = speed * elapsedTimeHours; // distance for this set of 30 pulses in km

	    	// Add the pulse distance to the total distance
	    	totalDistance += pulseDistance;

	        // Reset pulse counter
	        pulseCounter = 0;

	        // Start new time measurement
	        startTime = 0;

	      }
	      lastImpulseTime = HAL_GetTick();
	      // Restart measure for both channels
	      __HAL_TIM_SetCounter(htim, 0);

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


//void Throttle_Control_Speed(ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* htim, uint32_t channel, double* averageVoltagePercentage, double* scaledThrottleSpeed, double* speedPercentage, double* throttleSpeed)
//{
//    double averageVoltage;
//    double pidOutput;
//
//
//    // Stałe moc
//    #define MIN_SPEED 0.0
//    #define MAX_SPEED 20.0
//    #define MAX_SETPOINT 100.0
//
//
//	// Stałe dla regulatora PID
//    #define PID_KP_SPEED 6.5
//    #define PID_KI_SPEED 0.1
//    #define PID_KD_SPEED 10.1
//
//    // Zdefiniuj struktury
//    PID_TypeDef_Speed pid;
//
//    // Pobierz i skaluj wartość ADC = Voltage z manetki
//    Throttle_Get_Average_Voltage(hadc, &averageVoltage);
//    *averageVoltagePercentage = scale_value(averageVoltage, VMIN, VMAX, 0, MAX_SETPOINT);
//
//    // Skaluj prędkość
////    *speedPercentage = scale_value(speed, 0, 100, 0, MAX_SETPOINT);  // MAX_SPEED needs to be defined elsewhere
//    *speedPercentage = speed;  // MAX_SPEED needs to be defined elsewhere
//    if(*speedPercentage >= MAX_SPEED)
//    {
//        // Inicjalizacja PID with averageVoltagePercentage jako setpoint i speedPercentage jako wejście
//        pid.MyInputSpeed = speedPercentage;
//        pid.MySetpointSpeed = averageVoltagePercentage;
//        PID_Speed(&pid, speedPercentage, &pidOutput, averageVoltagePercentage, PID_KP_SPEED, PID_KI_SPEED, PID_KD_SPEED, _PID_P_ON_M_SPEED, _PID_CD_DIRECT_SPEED);
//
//        // Ustaw tryb PID na automatyczny, ustaw czas próbkowania i limity wyjścia
//        PID_SetMode_Speed(&pid, _PID_MODE_AUTOMATIC_SPEED);
//        PID_SetSampleTime_Speed(&pid, 100);
//        PID_SetOutputLimits_Speed(&pid, 0, 100);
//
//        // Compute PID using differential value
//        PID_Compute_Speed(&pid);
//
//        *throttleSpeed = pidOutput;
//        *scaledThrottleSpeed = scale_value(*throttleSpeed, 0, 0.9, 0, 100);
//    }
//    else
//    {
//        *throttleSpeed = *averageVoltagePercentage; // bez PID, manetka napędza koło bezpośrednio
//        *scaledThrottleSpeed = *throttleSpeed;
//    }
//
//
//
//    // ... kod do fizycznego ustawienia mocy na podstawie wyjścia z regulatora PID ...
//    HAL_TIM_PWM_Start(htim, channel);
//    // Aktualizacja PWM dla regulatora mocy:
//    __HAL_TIM_SET_COMPARE(htim, channel, 100 - *scaledThrottleSpeed);
//}
