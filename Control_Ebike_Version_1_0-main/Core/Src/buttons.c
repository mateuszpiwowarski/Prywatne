/*
 * buttons.c
 *
 *  Created on: Jan 21, 2024
 *      Author: mateu
 */
#include "main.h"

#include "buttons.h"
#include "stm32f4xx_hal_conf.h"


uint8_t Button_IsPressed(BUTTON_T *button)
{
    return HAL_GPIO_ReadPin(button->GPIO_Port, button->GPIO_Pin) == GPIO_PIN_RESET;
}



//// Button Init /////
void Button_Init(BUTTON_T* Key, GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin, uint32_t TimerDebounce,
				uint32_t TimerLongPress, uint32_t TimerRepeat)
{
    /* Configure GPIO pins for input */
	Key->Button_State = IDLE;		// Set initial state for the button
	Key->GPIO_Port = GPIO_Port;		// Remember GPIO Port for the button
	Key->GPIO_Pin = GPIO_Pin;		// Remember GPIO Pin for the button

	Key->TimerDebounce = TimerDebounce;		// Remember Debounce Time for the button
	Key->TimerLongPress = TimerLongPress;	// Remember LongPress Time for the button
	Key->TimerRepeat = TimerRepeat;			// Remember Repeat Time for the button
}

//// Time Settings Function ////
void Butto_Set_Debounce_Time(BUTTON_T* Key, uint32_t Milliseconds)
{
	Key -> TimerDebounce = Milliseconds;
}

void Butto_Set_LongPress_Time(BUTTON_T* Key, uint32_t Milliseconds)
{
	Key -> TimerLongPress = Milliseconds;
}

void Butto_Set_Repeat_Time(BUTTON_T* Key, uint32_t Milliseconds)
{
	Key -> TimerRepeat = Milliseconds;
}



//// Register CallBacks /////
void Button_Register_Press_CallBack(BUTTON_T* Key, void *Callback)
{
	Key -> ButtonPress = Callback;			// Set new callback for button Press
}

void Button_Register_LongPress_CallBack(BUTTON_T* Key, void *Callback)
{
	Key -> ButtonLongPress = Callback;		// Set new callback for button LongPress
}

void Button_Register_Repeat_CallBack(BUTTON_T* Key, void *Callback)
{
	Key -> ButtonRepeat = Callback;			// Set new callback for button Repeat
}






//// State Routine ///   FUNKCJE PRYWATNE
void Button_Idle_Routine(BUTTON_T* Key)
{
	// Check if button was pressed
	if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(Key -> GPIO_Port, Key -> GPIO_Pin))
	{
		// Button was pressed for the first time
		Key -> Button_State = DEBOUNCE;				// Jump to DEBOUNCE State
		Key -> LastTick = HAL_GetTick();			// Remember current tick for Debounce software timer
	}
}




void Button_Debounce_Routine(BUTTON_T* Key)
{
	// Wait for Debounce Timer elapsed
	if((HAL_GetTick() - Key -> LastTick) > Key -> TimerDebounce)
	{
		// After Debounce Timer elapsed check if button is still pressed
		if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(Key -> GPIO_Port, Key -> GPIO_Pin))
		{
			// Still pressed
			Key -> Button_State = PRESSED;			// Jump to PRESSED state
			Key -> LastTick = HAL_GetTick();		// Remember current tick for Debounce software timer

			if(Key -> ButtonPress != NULL)			// Check if callback for pressed button exists
			{
				Key -> ButtonPress();				// If exists - do the callback function
			}
		}
		else
		{
			// If button was released durong debounce time
			Key -> Button_State = IDLE;				// Go back do IDLE state
		}
	}

}

void Button_Press_Routine(BUTTON_T* Key)
{
	// Check if button was released
	if(GPIO_PIN_SET == HAL_GPIO_ReadPin(Key -> GPIO_Port, Key -> GPIO_Pin))
	{
		// If released - go back to IDLE state
		Key -> Button_State = IDLE;
	}
	else
	{
		// Wait for LongPress Timer elapsed
		if((HAL_GetTick() - Key -> LastTick) > Key -> TimerLongPress)
		{
			// Still pressed
			Key -> Button_State = REPEAT;				// Jump to PRESSED state
			Key -> LastTick = HAL_GetTick();			// Remember current tick for Debounce software timer

			if(Key -> ButtonLongPress != NULL)			// Check if callback for pressed button exists
			{
				Key -> ButtonLongPress();				// If exists - do the callback function
			}
		}

	}
}

void Button_Repeat_Routine(BUTTON_T* Key)
{
	// Check if button was released
	if(GPIO_PIN_SET == HAL_GPIO_ReadPin(Key -> GPIO_Port, Key -> GPIO_Pin))
	{
		// If released - go back to IDLE state
		Key -> Button_State = IDLE;
	}
	else
	{
		// Wait for Repeat Timer elapsed
		if((HAL_GetTick() - Key -> LastTick) > Key -> TimerRepeat)
		{
			Key -> LastTick = HAL_GetTick();			// Remember current tick for Debounce software timer

			if(Key -> ButtonRepeat != NULL)				// Check if callback for pressed button exists
			{
				Key -> ButtonRepeat();					// If exists - do the callback function
			}
		}
	}

}






//// State Machine /////
void Button_Task(BUTTON_T* Key)
{
	switch(Key -> Button_State)
	{
	case IDLE:
		Button_Idle_Routine(Key);
		break;

	case DEBOUNCE:
		Button_Debounce_Routine(Key);
		break;

	case PRESSED:
		Button_Press_Routine(Key);
		break;

	case REPEAT:
		Button_Repeat_Routine(Key);
		break;
	}
}
