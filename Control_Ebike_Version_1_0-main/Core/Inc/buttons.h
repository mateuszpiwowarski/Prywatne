/*
 * buttons.h
 *
 *  Created on: Jan 21, 2024
 *      Author: mateu
 */

#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_


///  State For State Machine
typedef enum
{
	IDLE = 0,
	DEBOUNCE,
	PRESSED,

	REPEAT,
}BUTTON_STATE;


/// Struct for Button
typedef struct
{
    GPIO_TypeDef* 	GPIO_Port;			// GPIO Port for a button
    uint16_t 		GPIO_Pin;			// GPIO Pin for a button
    BUTTON_STATE	Button_State;		// Button current state

    uint32_t		LastTick;			// Last remembered time before steps
    uint32_t		TimerDebounce;		// Fixed, settable time for Debounce timer

    uint32_t 		TimerLongPress;		// Fixed, settable time for LongPress timer
    uint32_t 		TimerRepeat;		// Fixed, settable time for Repeat timer

    void(*ButtonPress)(void);
    void(*ButtonLongPress)(void);			//// CALLBACKS
    void(*ButtonRepeat)(void);
} BUTTON_T;

uint8_t Button_IsPressed(BUTTON_T *button);

void Butto_Set_Debounce_Time(BUTTON_T* Key, uint32_t Milliseconds);
void Butto_Set_LongPress_Time(BUTTON_T* Key, uint32_t Milliseconds);
void Butto_Set_Repeat_Time(BUTTON_T* Key, uint32_t Milliseconds);

void Button_Init(BUTTON_T* Key, GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin, uint32_t TimerDebounce,
				uint32_t TimerLongPress, uint32_t TimerRepeat);

void Button_Task(BUTTON_T* Key);

void Button_Register_Press_CallBack(BUTTON_T* Key, void *Callback);
void Button_Register_LongPress_CallBack(BUTTON_T* Key, void *Callback);
void Button_Register_Repeat_CallBack(BUTTON_T* Key, void *Callback);

#endif /* INC_BUTTONS_H_ */
